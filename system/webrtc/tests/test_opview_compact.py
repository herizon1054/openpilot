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

