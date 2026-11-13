from types import SimpleNamespace

from openpilot.system.webrtc.webrtcd import OPVIEW_COMPACT_FIELDS, compact_fields


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
