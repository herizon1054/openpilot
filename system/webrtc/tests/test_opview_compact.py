from types import SimpleNamespace

from openpilot.system.webrtc.webrtcd import OPVIEW_COMPACT_FIELDS, compact_fields, compact_model_v2


class TestOpviewCompact:
  def test_radar_state_status(self):
    # dragonpilot cereal：有沒有前車叫 status
    lead = SimpleNamespace(status=True, dRel=21.0, yRel=0.3, vRel=-1.0, vLead=12.0, aLeadK=0.0, modelProb=0.9,
                           radar=True, radarTrackId=5, fcw=False)
    d = compact_fields(SimpleNamespace(leadOne=lead, leadTwo=lead), OPVIEW_COMPACT_FIELDS["radarState"])
    assert d["leadOne"] == {"status": True, "dRel": 21.0, "yRel": 0.3, "vRel": -1.0, "vLead": 12.0,
                            "aLeadK": 0.0, "modelProb": 0.9, "radar": True}

  def test_missing_field_is_skipped(self):
    # controlsState 沒有 vCruiseDEPRECATED：送出空 dict（opview 改用 carState.vCruiseCluster）
    assert compact_fields(SimpleNamespace(curvature=0.01), OPVIEW_COMPACT_FIELDS["controlsState"]) == {}

  def test_car_state_only_listed_fields(self):
    cs = SimpleNamespace(vEgo=10.0, aEgo=0.1, steeringAngleDeg=1.0, leftBlinker=False, rightBlinker=True,
                         leftBlindspot=False, rightBlindspot=False, gearShifter="drive", standstill=False,
                         gasPressed=False, vEgoCluster=10.1, vCruiseCluster=30.0, cruiseState=object())
    d = compact_fields(cs, OPVIEW_COMPACT_FIELDS["carState"])
    assert "cruiseState" not in d and d["gearShifter"] == "drive" and len(d) == 12

  def test_model_v2_orientation_rate(self):
    # opview v2.0.13：只送 orientationRate.z 的 t = 0；模型沒有輸出時送空 list
    xyz = SimpleNamespace(x=[1.0] * 3, y=[2.0] * 3, z=[3.0] * 3)
    m = SimpleNamespace(frameId=1, position=xyz, laneLines=[xyz] * 4, laneLineProbs=[0.9] * 4, roadEdges=[xyz] * 2,
                        roadEdgeStds=[0.5] * 2, acceleration=SimpleNamespace(x=[0.1]),
                        orientationRate=SimpleNamespace(z=[0.12345, 0.2]), leadsV3=[])
    assert compact_model_v2(m)["orientationRate"] == {"z": [0.123]}
    m.orientationRate = SimpleNamespace(z=[])
    assert compact_model_v2(m)["orientationRate"] == {"z": []}



class _Point:
  """雷達點替身：measured=None 代表這個 opendbc 不提供 measured（讀取會 AttributeError）"""
  def __init__(self, track_id, d_rel, measured):
    self.trackId, self.dRel, self.yRel, self.vRel, self._measured = track_id, d_rel, 0.5, -1.0, measured

  @property
  def measured(self):
    if self._measured is None:
      raise AttributeError("measured is deprecated")
    return self._measured


class TestOpviewTracks:
  def _reader(self, monkeypatch, points):
    from openpilot.system.webrtc import webrtcd
    reader = webrtcd.OpviewTracksReader()
    if reader.sock is None:
      reader.name, reader.sock = "liveTracks", object()
    msg = SimpleNamespace(**{reader.name: SimpleNamespace(points=points)})
    monkeypatch.setattr(webrtcd.messaging, "recv_one_or_none", lambda sock: msg)
    return reader

  def test_no_measured_flag_keeps_points(self, monkeypatch):
    # 移植指南 6.1：整筆沒有 measured 時不過濾（dp0111pre 參考版只用 p.measured，一個點都送不出去）
    reader = self._reader(monkeypatch, [_Point(1, 10.0, None), _Point(2, 60.0, None), _Point(3, 20.0, None)])
    assert [p[0] for p in reader.get()] == [1, 3]

  def test_measured_filters_when_present(self, monkeypatch):
    reader = self._reader(monkeypatch, [_Point(1, 10.0, True), _Point(3, 20.0, False)])
    assert [p[0] for p in reader.get()] == [1]
    info = reader.info()
    assert info["raw"] == 2 and info["meas"] == 1 and info["rx"] == 1


def test_only_localhost_may_bridge_in():
  from openpilot.system.webrtc.webrtcd import is_local_request
  assert is_local_request(SimpleNamespace(remote="127.0.0.1"))
  assert is_local_request(SimpleNamespace(remote="::1"))
  assert not is_local_request(SimpleNamespace(remote="192.168.43.20"))
  assert not is_local_request(SimpleNamespace(remote=None))


def test_egpu_falls_back_without_jetlink(monkeypatch):
  # Jetlink 關閉、沒有 USB GPU：與 dp0111pre 相同的欄位，另附 source
  from openpilot.system.webrtc import webrtcd
  reader = webrtcd.OpviewEgpuReader()
  monkeypatch.setattr(reader, "_get_bool", lambda key: False)
  monkeypatch.setattr(reader, "_link_on", lambda: False)
  assert reader.get() == {"present": False, "active": False, "loading": False, "compiled": False, "source": "usbgpu"}


def test_egpu_reports_jetlink(monkeypatch):
  from openpilot.system.webrtc import webrtcd
  reader = webrtcd.OpviewEgpuReader()
  monkeypatch.setattr(reader, "_get_bool", lambda key: False)
  monkeypatch.setattr(reader, "_link_on", lambda: True)
  status = SimpleNamespace(present=True, active=False, runnable=True, running="joining", progress=None)
  import dragonpilot.jetlink_adapter as adapter
  monkeypatch.setattr(adapter, "status", lambda: status)
  assert reader.get() == {"present": True, "active": False, "loading": True, "compiled": True, "source": "jetlink"}
