import json
from types import SimpleNamespace

import pytest

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.webrtc.webrtcd import (
  OPVIEW_COMPACT_FIELDS, CerealOutgoingMessageProxy, StreamRequestError, compact_fields, compact_model_v2,
  parse_stream_request,
)


def _xyz(n):
  return SimpleNamespace(x=[0.12345] * n, y=[1.0] * n, z=[-2.0] * n)


def _fake_model():
  return SimpleNamespace(frameId=7, position=_xyz(3), laneLines=[_xyz(3)] * 4, laneLineProbs=[0.91234] * 4,
                         roadEdges=[_xyz(3)] * 2, roadEdgeStds=[0.5] * 2, acceleration=SimpleNamespace(x=[0.33333] * 3),
                         leadsV3=[SimpleNamespace(prob=0.23456, x=[4.0004] * 6, y=[0.1] * 6)] * 3)


class TestOpviewStreamRequest(OpenpilotTestCase):
  def test_opview_format(self):
    body, is_opview = parse_stream_request({"sdp": "x", "initCamera": "road", "bridge_services_in": [],
                                            "bridge_services_out": ["carState", "modelV2"], "someNewField": 1})
    assert is_opview
    assert body.cameras == ["road"]
    assert body.enabled is True
    assert body.bridge_services_out == ["carState", "modelV2"]

  def test_upstream_format_unchanged(self):
    body, is_opview = parse_stream_request({"sdp": "x", "cameras": ["wideRoad"], "enabled": False,
                                            "bridge_services_out": ["carState"]})
    assert not is_opview
    assert body.cameras == ["wideRoad"] and body.enabled is False

  def test_upstream_missing_fields_still_rejected(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x"})
    assert e.value.status == 400

  def test_unknown_service_is_500(self):
    # 不認得的 service 回 500（與 dp0111pre / stock webrtcd 一致）
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "road", "bridge_services_out": ["carState", "noSuchService"]})
    assert e.value.status == 500

  def test_renamed_services(self):
    # opview 用舊名稱；這版改名為 extrinsicsCalibration / narrowRoadCameraState
    body, _ = parse_stream_request({"sdp": "x", "initCamera": "road",
                                    "bridge_services_out": ["carState", "liveCalibration", "roadCameraState", "modelV2"]})
    assert body.bridge_services_out == ["carState", "extrinsicsCalibration", "narrowRoadCameraState", "modelV2"]
    proxy = CerealOutgoingMessageProxy(body.bridge_services_out, opview=True)
    assert proxy.out_names == {"extrinsicsCalibration": "liveCalibration", "narrowRoadCameraState": "roadCameraState"}

  def test_invalid_camera(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "rear", "bridge_services_out": []})
    assert e.value.status == 400

  def test_lan_cannot_publish(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "road", "bridge_services_in": ["testJoystick"]}, remote_is_local=False)
    assert e.value.status == 403
    # localhost（sunnylink / athena / body）照舊可以
    body, _ = parse_stream_request({"sdp": "x", "cameras": ["road"], "enabled": True, "bridge_services_in": ["testJoystick"]})
    assert body.bridge_services_in == ["testJoystick"]


  def test_privacy_blocks_driver_camera(self):
    with pytest.raises(StreamRequestError) as e:
      parse_stream_request({"sdp": "x", "initCamera": "driver"}, allow_driver_camera=False)
    assert e.value.status == 403
    with pytest.raises(StreamRequestError):
      parse_stream_request({"sdp": "x", "cameras": ["driver"], "enabled": True}, allow_driver_camera=False)
    body, _ = parse_stream_request({"sdp": "x", "initCamera": "road"}, allow_driver_camera=False)
    assert body.cameras == ["road"]


class TestOpviewTdxDropped(OpenpilotTestCase):
  # opview 已內建 TDX：opview 請求帶 tdx（舊版 App）時拿掉，不訂閱、不回 500；upstream 格式不受影響
  def test_opview_tdx_removed(self):
    body, is_opview = parse_stream_request({"sdp": "x", "initCamera": "road",
                                            "bridge_services_out": ["carState", "tdx", "modelV2"]})
    assert is_opview
    assert "tdx" not in body.bridge_services_out
    assert body.bridge_services_out[0] == "carState" and "modelV2" in body.bridge_services_out

  def test_upstream_request_unchanged(self):
    body, is_opview = parse_stream_request({"sdp": "x", "cameras": ["road"], "bridge_services_out": ["carState"],
                                            "bridge_services_in": [], "enabled": True})
    assert not is_opview
    assert body.bridge_services_out == ["carState"]


class TestOpviewOutgoingProxy(OpenpilotTestCase):
  def test_compact_model(self):
    d = compact_model_v2(_fake_model())
    assert set(d) == {"frameId", "position", "laneLines", "laneLineProbs", "roadEdges", "roadEdgeStds", "acceleration", "leadsV3"}
    assert d["laneLineProbs"][0] == 0.912 and d["acceleration"]["x"][0] == 0.333
    assert len(d["leadsV3"]) == 3 and d["leadsV3"][0] == {"prob": 0.235, "x": [4.0] * 6, "y": [0.1] * 6}
    json.dumps(d)

  def _proxy(self, mocker, send_result=True):
    proxy = CerealOutgoingMessageProxy(["carState"], opview=True)
    proxy.sm = mocker.MagicMock()
    proxy.sm.updated = {"carState": True}
    proxy.sm.__getitem__.return_value = 1.5
    proxy.sm.logMonoTime = {"carState": 1}
    proxy.sm.valid = {"carState": True}
    ch = mocker.Mock(spec=["is_open", "send"])  # 不提供 buffered_amount：opview 路徑不得呼叫它
    ch.is_open.return_value = True
    ch.send.return_value = send_result
    proxy.add_channel(ch)
    return proxy, ch

  def test_congested_channel_backs_off(self, mocker):
    # send() 回 False（進了緩衝佇列）→ 暫停 OPVIEW_BACKOFF_S，期間不再送
    proxy, ch = self._proxy(mocker, send_result=False)
    proxy.update()
    assert ch.send.call_count == 1 and proxy.congested_since is not None
    proxy.last_sent.clear()
    proxy.update()
    assert ch.send.call_count == 1 and proxy.skipped == 1
    proxy.backoff_until = 0.0
    ch.send.return_value = True
    proxy.update()
    assert ch.send.call_count == 2 and proxy.congested_since is None

  def test_rate_limit(self, mocker):
    proxy, ch = self._proxy(mocker)
    proxy.update()
    proxy.update()  # 30 ms 內第二次不送
    assert ch.send.call_count == 1
    assert json.loads(ch.send.call_args.args[0]) == {"type": "carState", "logMonoTime": 1, "valid": True, "data": 1.5}



class TestWebrtcdMemory(OpenpilotTestCase):
  # mallopt(M_ARENA_MAX, 2) / malloc_trim(0)：用假的 libc 確認呼叫參數；非 glibc 時不丟例外、回傳 False
  class _FakeLibc:
    def __init__(self):
      self.calls = []

    def mallopt(self, param, value):
      self.calls.append(("mallopt", param, value))
      return 1

    def malloc_trim(self, pad):
      self.calls.append(("malloc_trim", pad))
      return 1

  def test_limit_and_trim_call_glibc(self):
    from unittest import mock
    from openpilot.system.webrtc import webrtcd
    fake = self._FakeLibc()
    with mock.patch.object(webrtcd, "_libc", fake):
      assert webrtcd.limit_malloc_arenas() is True
      assert webrtcd.release_memory()[0] is True
    assert fake.calls == [("mallopt", -8, 2), ("malloc_trim", 0)]

  def test_without_glibc_is_noop(self):
    from unittest import mock
    from openpilot.system.webrtc import webrtcd
    with mock.patch.object(webrtcd, "_libc", False):
      assert webrtcd.limit_malloc_arenas() is False
      assert webrtcd.release_memory()[0] is False

  def test_real_libc_does_not_raise(self):
    from unittest import mock
    from openpilot.system.webrtc import webrtcd
    with mock.patch.object(webrtcd, "_libc", None):  # 重新載入真正的 libc（非 glibc 上得到 False）
      webrtcd.release_memory()



class TestOpviewCompactFields(OpenpilotTestCase):
  # 只送 opview 讀的欄位；不存在的欄位略過；子結構（radarState.leadOne）只取指定欄位
  def test_car_state_subset(self):
    cs = SimpleNamespace(vEgo=12.5, aEgo=0.1, steeringAngleDeg=-3.0, leftBlinker=False, rightBlinker=True,
                         leftBlindspot=False, rightBlindspot=False, gearShifter="drive", standstill=False,
                         gasPressed=False, vEgoCluster=12.4, vCruiseCluster=60.0,
                         cruiseState=SimpleNamespace(speed=16.6), buttonEvents=[1, 2, 3], wheelSpeeds=None)
    d = compact_fields(cs, OPVIEW_COMPACT_FIELDS["carState"])
    assert set(d) == set(OPVIEW_COMPACT_FIELDS["carState"])
    assert d["gearShifter"] == "drive" and d["vEgo"] == 12.5 and d["rightBlinker"] is True
    assert "cruiseState" not in d and "buttonEvents" not in d

  def test_missing_field_is_skipped(self):
    # controlsState 在新版 schema 已沒有 vCruiseDEPRECATED：送出空 dict（與 to_dict() 沒有該欄位相同）
    assert compact_fields(SimpleNamespace(curvature=0.01), OPVIEW_COMPACT_FIELDS["controlsState"]) == {}

  def test_radar_state_leads(self):
    lead = SimpleNamespace(status=True, dRel=21.0, yRel=0.3, vRel=-1.0, vLead=12.0, aLeadK=0.0, modelProb=0.9,
                           radar=True, radarTrackId=5, fcw=False, aLeadTau=1.5)
    rs = SimpleNamespace(leadOne=lead, leadTwo=lead, mdMonoTime=1, cumLagMs=0.0)
    d = compact_fields(rs, OPVIEW_COMPACT_FIELDS["radarState"])
    assert set(d) == {"leadOne", "leadTwo"}
    assert d["leadOne"] == {"status": True, "dRel": 21.0, "yRel": 0.3, "vRel": -1.0, "vLead": 12.0,
                            "aLeadK": 0.0, "modelProb": 0.9, "radar": True}

  def test_radar_state_leads_present(self):
    # openpilot / sunnypilot 新版 cereal：status 改名 present，也要送出（opview 依此判斷有沒有前車）
    lead = SimpleNamespace(present=True, dRel=21.0, yRel=0.3, vRel=-1.0, vLead=12.0, aLeadK=0.0, modelProb=0.9, radar=True)
    d = compact_fields(SimpleNamespace(leadOne=lead, leadTwo=lead), OPVIEW_COMPACT_FIELDS["radarState"])
    assert d["leadOne"]["present"] is True and "status" not in d["leadOne"]

  def test_lists_become_lists(self):
    cal = SimpleNamespace(calStatus="calibrated", rpyCalib=(0.0, 0.01, 0.02), wideFromDeviceEuler=[0.0] * 3,
                          height=[1.22], validBlocks=20)
    d = compact_fields(cal, OPVIEW_COMPACT_FIELDS["liveCalibration"])
    assert d == {"calStatus": "calibrated", "rpyCalib": [0.0, 0.01, 0.02], "wideFromDeviceEuler": [0.0] * 3,
                 "height": [1.22]}
    json.dumps(d)
