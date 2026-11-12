import math
import statistics
from collections import deque

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# dp: Hyundai Custin 前雷達（從 openpilot-full 移植）。只有 HyundaiFlags.CUSTIN_RADAR 的車走這條路，
# 其他 Hyundai/Kia 的 Mando 0x500 解析與原本 dp 完全相同。
#
# The Custin carries its track list at 0x238 on bus 1 rather than in the Mando 0x500 block.
# Fields were reverse engineered from this car's own logs, see opendbc/dbc/custin_radar.dbc.
# The thirty addresses are ten targets of three messages each, not thirty tracks: over twenty
# seconds of driving the valid-frame counts of 0x238 and 0x239 match exactly and the pattern
# repeats every third address, while the two companions swing across the full field where a
# real lateral offset would sit. Reading all thirty invented two targets for every real one.
# LONG_DIST/LAT_DIST are a cartesian pair in metres. The lateral field was first read as an
# azimuth in degrees, one bit too wide: its lowest bit reads 1 in 90% of frames where a real
# data bit sits at 50%, and dropping it makes a stationary target's dy/dt match -yaw_rate*x
# (fitted slope 2.05 -> 1.13, r=0.89 over 341 samples taken while turning).
CUSTIN_RADAR_START_ADDR = 0x238
CUSTIN_RADAR_STRIDE = 3
CUSTIN_RADAR_TRACKS = 10
CUSTIN_RADAR_ADDRS = tuple(range(CUSTIN_RADAR_START_ADDR,
                                CUSTIN_RADAR_START_ADDR + CUSTIN_RADAR_STRIDE * CUSTIN_RADAR_TRACKS,
                                CUSTIN_RADAR_STRIDE))
CUSTIN_PRIMARY_ONLY = True  # see the comment in _update
CUSTIN_MIN_RANGE = 2.0      # below this is bumper clutter, and an empty slot reads zero
# except on the stock ACC's own pick, which carries neither. Over 2026-09-03 the primary
# slot read under two metres on 507 frames: 506 of them below 3 km/h, every one of those
# dead ahead within 0.2 m of centre with the score saturated, and exactly one at road speed
# - that one 2.5 m off to the side. It is the car we are stopped behind. Throwing it away
# left only vision, whose range on a stationary car at four metres swung between 3.3 and
# 14.8 m as motorcycles went past, and the planner crept at a gap that was not there.
CUSTIN_PRIMARY_MIN_RANGE = 0.5
CUSTIN_MAX_ABS_Y = 5.5      # this lane and the two beside it; roadside structure sits outside
CUSTIN_WINDOW = 12          # ~0.36 s of history at 33 Hz
CUSTIN_MIN_SAMPLES = 6
CUSTIN_MAX_GAP = 0.2        # seconds without an update before a slot is considered reused
CUSTIN_MIN_SCORE = 30       # the radar's own tracking score, 31 is saturated
CUSTIN_LANE_SCORE = 20      # a low score in our own lane is still a car, see _update
CUSTIN_LANE_HALF_WIDTH = 1.5  # metres, where a car we are following actually sits
CUSTIN_MIN_HITS = 8         # consecutive good frames before a track is handed over
CUSTIN_MAX_JUMP = 3.0       # metres of range jump that means a different object took the slot


class CustinSlot:
  """Speed comes from the track's own V_ABS; range is averaged over a short window."""

  def __init__(self):
    self.hist: deque = deque(maxlen=CUSTIN_WINDOW)

  def reset(self):
    self.hist.clear()

  def update(self, t, d_rel, v_abs) -> bool:
    """-> True when a different object has taken the slot."""
    swapped = bool(self.hist) and (t - self.hist[-1][0] > CUSTIN_MAX_GAP
                                   or abs(d_rel - self.hist[-1][1]) > CUSTIN_MAX_JUMP)
    if swapped:
      self.hist.clear()
    self.hist.append((t, d_rel, v_abs))
    return swapped

  def solve(self, v_ego):
    """-> (dRel, vRel) or None, with vRel taken from the radar rather than from the range.

    Differentiating range over this window used to supply vRel, and it reads about 1.5 m of
    measurement noise on a target 30 m out: over a third of a second that is several m/s of
    speed that is not there. Measured against a centred +-1.5 s fit of the range, on the one
    target radard follows, with a slow or stopped car ahead: differentiating called it
    'opening' by more than 2 m/s on 5.1% of frames, which is the planner being told to
    accelerate towards a stationary car. Reading V_ABS does that on none of them, and its
    p90 error is 1.26 m/s against 3.03. Lengthening the window instead only trades the
    error for lag (0.9 s: 2.2% opening, but 8.5% falsely closing against 7.1%).

    The median rejects the odd bad frame; a range that does not move still counts as valid
    and just reports the speed the radar gives it.
    """
    if not self.hist:
      return None
    v_rel = statistics.median([p[2] for p in self.hist]) - v_ego
    if len(self.hist) < CUSTIN_MIN_SAMPLES:
      return self.hist[-1][1], v_rel
    ts = [p[0] for p in self.hist]
    ds = [p[1] for p in self.hist]
    n = len(ts)
    mean_t = sum(ts) / n
    mean_d = sum(ds) / n
    return mean_d + v_rel * (ts[-1] - mean_t), v_rel

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  if CP.flags & HyundaiFlags.CUSTIN_RADAR:
    addrs, freq = CUSTIN_RADAR_ADDRS, 33
  else:
    addrs, freq = tuple(range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)), 50

  messages = [(f"RADAR_TRACK_{addr:x}", freq) for addr in addrs]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


def get_speed_can_parser(CP):
  # The Custin track list only carries each target's absolute speed, so the car's own
  # speed has to come off the powertrain bus to turn that into a relative speed.
  return CANParser(DBC[CP.carFingerprint][Bus.pt], [("WHL_SPD11", 50)], 0)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.custin = bool(CP.flags & HyundaiFlags.CUSTIN_RADAR)
    self.addrs = CUSTIN_RADAR_ADDRS if self.custin else tuple(range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT))
    self.trigger_msg = self.addrs[-1]
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)
    self.scp = get_speed_can_parser(CP) if self.custin and self.rcp is not None else None
    self.v_ego = 0.
    self.slots = {addr: CustinSlot() for addr in CUSTIN_RADAR_ADDRS} if self.custin else {}
    self.frame_time = 0.
    self.hits: dict[int, int] = dict.fromkeys(self.slots, 0)
    # every track, where ret.points is only what radard may follow (Custin only; dp has no
    # radarTracksSP to publish it on, it is kept for debugging / a future display)
    self.all_points: list = []

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.custin and can_strings:
      self.frame_time = can_strings[-1][0] * 1e-9

    if self.scp is not None:
      self.scp.update(can_strings)
      whl = self.scp.vl["WHL_SPD11"]
      speeds = (whl["WHL_SPD_FL"], whl["WHL_SPD_FR"], whl["WHL_SPD_RL"], whl["WHL_SPD_RR"])
      self.v_ego = sum(speeds) / 4. * CV.KPH_TO_MS * self.CP.wheelSpeedFactor

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.custin:
      return self._update_custin(ret)

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].measured = True
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = float('nan')

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_custin(self, ret):
    # 0x238 is the target the car's own ACC has locked onto, and it tracks the stock ACC's
    # reported distance to 0.10 m. The rest of the list is raw, and a guardrail scanned
    # along its length holds a steady range at a lane's worth of lateral offset: radard
    # reads that as a car keeping station ahead and picks it over the real one, which
    # measured worse than vision alone. So the whole list is decoded, kept in all_points
    # for watching the lanes beside us, while radard is handed the primary target alone.
    for addr in self.addrs:
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      rng = msg['LONG_DIST']
      y_rel = -msg['LAT_DIST']
      x_rel = rng
      slot = self.slots[addr]

      # STATE does not tell targets from scenery here, and a guardrail read along its
      # length looks like a car at our own speed, so gate on range and lateral offset.
      # the primary target is the car's own pick, so it is not second-guessed on offset
      primary = addr == self.addrs[0]
      floor = CUSTIN_PRIMARY_MIN_RANGE if primary else CUSTIN_MIN_RANGE
      # SCORE is not a measure of how good a reading is. Measured against everything
      # else on the target over 23592 frames (2026-09-11), it tracks lateral offset and
      # track age instead: a saturated 31 sits 0.47 m off centre and has been held for
      # 234 frames, while 1-19 sits at 5.77 m and is seven frames old. Among targets
      # 20-45 m out and within 1.5 m of centre, a below-gate score is no less steady
      # frame to frame than a saturated one (0.100 vs 0.200 m). The 30 floor blinded the
      # planner for 2.2 s on 2026-09-11 13:42 as the lead braked from 40 to 16 km/h while
      # SCORE dithered 28-31. Do not simply lower the floor: 51.5% of the frames it rejects
      # sit more than 2 m off centre (guardrail and next-lane traffic). Lateral offset is the
      # radar's own steady reading (0.045 m of jitter), so let that decide what counts as ours.
      in_lane = abs(y_rel) < CUSTIN_LANE_HALF_WIDTH
      score_ok = (msg['SCORE'] >= CUSTIN_MIN_SCORE
                  or (in_lane and msg['SCORE'] >= CUSTIN_LANE_SCORE))
      keep = (rng > floor and score_ok
              and (primary or abs(y_rel) < CUSTIN_MAX_ABS_Y))
      if keep:
        # 0x238 is whichever target the stock ACC has chosen, not a fixed slot, so it
        # changes car without saying so. The range history is thrown away on a jump, and
        # the track id has to change with it, or radard sees one car teleporting (behind
        # two motorcycles on 2026-09-02 the lead moved 9.0 -> 5.5 m in a frame). The Kalman
        # filter's speed, the preferred-track match and the MPC's obstacle all build on it.
        if slot.update(self.frame_time, x_rel, msg['V_ABS']):
          self.pts[addr].trackId = self.track_id
          self.track_id += 1
        self.hits[addr] = min(self.hits[addr] + 1, CUSTIN_MIN_HITS)
      else:
        slot.reset()
        self.hits[addr] = 0

      fit = slot.solve(self.v_ego)
      valid = fit is not None and self.hits[addr] >= CUSTIN_MIN_HITS
      if valid:
        self.pts[addr].dRel, self.pts[addr].vRel = fit
        self.pts[addr].yRel = y_rel
        # dp schema: the radar measured this target (not an estimate); it gives no
        # relative acceleration or lateral speed
        self.pts[addr].measured = True
        self.pts[addr].aRel = float('nan')
        self.pts[addr].yvRel = float('nan')
      else:
        del self.pts[addr]

    self.all_points = list(self.pts.values())
    if CUSTIN_PRIMARY_ONLY:
      primary_pt = self.pts.get(self.addrs[0])
      ret.points = [primary_pt] if primary_pt is not None else []
    else:
      ret.points = self.all_points
    return ret
