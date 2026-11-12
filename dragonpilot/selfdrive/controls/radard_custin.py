#!/usr/bin/env python3
"""
dragonpilot radard：Hyundai Custin 雷達融合（從 openpilot-full 的 selfdrive/controls/radard.py 移植）。

只在 CP.flags 帶 HyundaiFlags.CUSTIN_RADAR 時由 radard.main() 載入（見 radard.DP_RADARD_CUSTIN）。
其他 Hyundai/Kia 與所有品牌仍走原廠 RadarD；Toyota 仍走 radard_ext。本模組不 monkey patch radard，
所有邏輯都在本檔的函式與 RadarDCustin 裡，原廠 radard.get_lead / match_vision_to_track 維持不變。

移植的機制（全部只用 Custin 的 rlog 驗證，數據見 openpilot-full 的 RADAR.md）：
  1. match_vision_to_track_custin：距離 gate 25% -> 7% / 2 m，加上 same_object（速度 + 橫向一致即同一物），
     same_object 再受 MAX_SAME_OBJECT_DIST_GAP（50 m）與 MIN_SAME_OBJECT_HEADWAY（0.6 s）限制；
     stopped_nearer：視覺前車靜止時，比視覺近很多的靜止雷達點不採用。
  2. preferred_track_id（StarPilot）：上一幀的雷達前車用放寬 gate 再驗一次，減少雷達/視覺來回切換。
  3. LeadHold：模型機率短暫掉落（<= 3 s、45 m 內）時沿用上一個前車；停等時雷達掉線而視覺讀更遠也沿用。
  4. RADAR_KEEP：視覺剛確認過的雷達 track，2 s 內仍是同一 track、橫向一致就繼續採用（夜間視覺不準）。
  5. lane_mismatch：原廠 ACC 把 0x238 換到隔壁車道較快的車、視覺本車道前車較慢時，連續 0.3 s 改用視覺。

與 openpilot-full 的刻意分歧（只為配合 dp 的 schema）：
  - radarState 欄位 'present' -> dp 的 'status'；保留 dp 的 'fcw'（Track.get_RadarState 原樣）。
  - 訂閱 'liveTracks'（dp）而非 'radarTracks'；Track.update 帶 measured（dp 介面）。
  - 不用 radard_ext 的 dp_kwargs（radard_ext 只給 Toyota）。
"""
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import (
  RadarD, Track, laplacian_pdf, get_RadarState_from_vision, RADAR_TO_CAMERA,
)

# How far the radar and vision may disagree on range and still be called the same car.
# Sized to reject only the absurd: accepted matches sit 0.8 m apart at the median.
MAX_SAME_OBJECT_DIST_GAP = 50.0  # m

# A range means nothing without the speed it is read at: 8 m is 0.58 s of headway at 50
# km/h and 2.9 s at 10 km/h. Over 49 minutes of accepted matches this radar placed the lead
# inside 0.6 s of headway on 0.53% of frames, at a median of 4.9 m while vision looked 92.7
# m out and the car was doing 54 km/h. Nothing is there. Below 15 km/h the ratio cannot fire
# at all, which leaves stop-and-go and junction crawls alone, and it never fires past 45 m.
MIN_SAME_OBJECT_HEADWAY = 0.6  # s
MIN_HEADWAY_GATE_SPEED = 4.17  # m/s, 15 km/h

# When vision's lead is stopped, speed says nothing: every pole, parked scooter and kerb object
# is doing 0 too, so same_object passes on speed and lateral alone. 2026-10-31 15:15:46 at 18
# km/h: vision's lead was a stopped car at 35-48 m, the radar held something stopped 2.1 m to
# the side at 9 m, matching took it, and the planner asked for -3.3 m/s2 on an empty road. So a
# radar track that is itself stopped, with vision's lead stopped, and nearer than vision by more
# than this share of vision's range, is not the same car - vision's lead is used. One way only:
# a radar track farther than vision's is left alone.
STOPPED_V = 2.0          # m/s
STOPPED_NEARER = 0.30    # of vision's range
STOPPED_NEARER_MIN = 5.0  # m

# The lead going away is usually not the lead leaving. Over 2.27 hours of this car's own
# logs leadOne vanished 268 times an hour, and 52% of those came back within a second at a
# distance that carried on from where it left off, with the raw vision probability still
# sitting at 0.35 - the model blinking, not a car pulling away. Holding through the blink is
# worth 2.1:1 inside 25 m and 2.9:1 from 25 to 45. Past 45 m it is 0.9:1, so nothing is held.
# The gate is the model's own probability, not the filtered one radard decides with.
LEAD_HOLD_MAX_DIST = 45.
LEAD_HOLD_MIN_PROB = 0.2
LEAD_HOLD_MAX_TIME = 3.0
LEAD_HOLD_MIN_SPEED = 3.0

# Stopped, the same hold is needed for the opposite failure: vision still has a lead but the
# radar track under it has gone, and vision reads a stopped car at 3-4 m as anywhere between
# 3.3 and 14.8 m, so the planner sees the gap open and creeps. Only held when vision reports
# the lead further away than the radar last did: reading it closer is believed immediately.
LEAD_HOLD_STANDSTILL_SPEED = 0.5  # m/s
LEAD_HOLD_STANDSTILL_DIST = 10.0  # m
LEAD_HOLD_STANDSTILL_JUMP = 1.0  # m

# Keep the radar track vision has just confirmed (2026-10-06). At night vision's own reported
# uncertainty grows and the match starts turning the radar away although the radar track is
# the same car. The track that was the lead stays the lead for RADAR_KEEP_S while it is the
# same track, within RADAR_KEEP_DY of vision's lead sideways, and vision does not see something
# nearer than RADAR_KEEP_NEAR of its range.
RADAR_KEEP_S = 2.0
RADAR_KEEP_DY = 1.0       # m
RADAR_KEEP_NEAR = 0.7     # vision nearer than this share of the track's range -> believe vision
RADAR_KEEP_REFRESH = False   # True keeps it as long as the rest holds; measured the same as 2 s

# The radar lead sits in the next lane while vision's sits in ours and is slower: the stock ACC
# moved 0x238 to a car 1.52 m to our right while the car dead ahead braked (2026-09-27). Only
# ever swaps to the more cautious reading: vision confident, in our lane, and slower than the
# radar car. A radar lead that is nearer or slower than vision's is never dropped here.
LANE_MISMATCH_VISION_PROB = 0.9
LANE_MISMATCH_VISION_Y = 0.8     # m, vision's lead within our lane
LANE_MISMATCH_RADAR_Y = 1.0      # m, radar's outside it
LANE_MISMATCH_DY = 1.0           # m apart laterally
LANE_MISMATCH_DV = 1.5           # m/s, radar's car that much faster than vision's
LANE_MISMATCH_MAX_DIST = 70.     # m
LANE_MISMATCH_FRAMES = int(round(0.3 / DT_MDL))


def match_vision_to_track_custin(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                                 preferred_track_id: int = -1):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  def stopped_nearer(t):
    """see STOPPED_V"""
    return (abs(lead.v[0]) < STOPPED_V and abs(t.vLead) < STOPPED_V and
            offset_vision_dist - t.dRel > max(STOPPED_NEARER * offset_vision_dist, STOPPED_NEARER_MIN))

  # The Custin's radar reports a guardrail scanned along its length as a steady target in our
  # own lane; at 25% the gate is 28 m wide at 110 m, so the rail got accepted. 7% cut that from
  # 56% of frames to 4% while the stock ACC's own target still matched on 99%. That gate tests
  # vision's distance, which degrades at night, so same_object (speed + lateral, which this
  # radar measures directly) also counts - night acceptance 66.7 -> 91.6%. The range must still
  # be in the same world (50 m) and at a sane headway: a bridge noise barrier at our speed on
  # our centreline read 7 m against vision's 101 and braked the car at 60 km/h on 2026-09-06.
  same_object = abs(track.vRel + v_ego - lead.v[0]) < 1.5 and abs(track.yRel + lead.y[0]) < 2.0
  same_object = same_object and abs(track.dRel - offset_vision_dist) < MAX_SAME_OBJECT_DIST_GAP
  same_object = same_object and (v_ego <= MIN_HEADWAY_GATE_SPEED
                                 or track.dRel > MIN_SAME_OBJECT_HEADWAY * v_ego)
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.07, 2.0]) or same_object
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return None if stopped_nearer(track) else track

  # The strict gate blinks: on 2026-09-02 the lead fell from radar to vision 1905 times in 63
  # minutes and 95.1% of the time the same track came back ~0.10 s later. Give the track we were
  # already using a second, looser look (StarPilot's preferred_track_id), still measured against
  # this frame's vision lead. cnt >= 3 keeps a just-appeared track from being preferred.
  preferred = tracks.get(preferred_track_id)
  if preferred is not None and preferred.cnt >= 3:
    pref_same = abs(preferred.vRel + v_ego - lead.v[0]) < 2.5 and abs(preferred.yRel + lead.y[0]) < 3.0
    pref_dist = abs(preferred.dRel - offset_vision_dist) < max([(offset_vision_dist)*.12, 4.0]) or pref_same
    pref_vel = (abs(preferred.vRel + v_ego - lead.v[0]) < 13) or (v_ego + preferred.vRel > 3)
    if pref_dist and pref_vel:
      return None if stopped_nearer(preferred) else preferred
  return None


def get_lead_custin(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
                    model_v_ego: float, lead_prob: float, low_speed_override: bool = True,
                    preferred_track_id: int = -1) -> dict[str, Any]:
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track_custin(v_ego, lead_msg, tracks, preferred_track_id)
  else:
    track = None

  lead_dict: dict[str, Any] = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class LeadHold:
  """Carries the last lead through a dip in the model's confidence. See LEAD_HOLD_* above."""

  def __init__(self):
    self.lead: dict[str, Any] | None = None
    self.held = 0.

  def _standstill_radar_loss(self, lead: dict[str, Any], v_ego: float) -> bool:
    """Stopped, and the radar track under the lead has gone while vision reads it further.

    get_lead returns a bare {'status': False} when there is no lead at all, so the other keys
    only exist once something has been seen (reading them unguarded crashed radard in
    openpilot-full on 2026-09-08 with KeyError: 'radar').
    """
    return (v_ego < LEAD_HOLD_STANDSTILL_SPEED and self.lead is not None
            and lead['status']
            and self.lead['radar'] and not lead['radar']
            and self.lead['dRel'] < LEAD_HOLD_STANDSTILL_DIST
            and lead['dRel'] > self.lead['dRel'] + LEAD_HOLD_STANDSTILL_JUMP)

  def update(self, lead: dict[str, Any], raw_prob: float, v_ego: float) -> dict[str, Any]:
    standstill_loss = self._standstill_radar_loss(lead, v_ego)
    if lead['status'] and not standstill_loss:
      self.lead = dict(lead)
      self.held = 0.
      return lead

    if (self.lead is None or (v_ego < LEAD_HOLD_MIN_SPEED and not standstill_loss)
        or raw_prob < LEAD_HOLD_MIN_PROB
        or self.held >= LEAD_HOLD_MAX_TIME or self.lead['dRel'] > LEAD_HOLD_MAX_DIST):
      self.lead = None
      self.held = 0.
      return lead

    self.held += DT_MDL
    # let it keep closing at the speed it was last measured doing, so the gap does not freeze
    self.lead['dRel'] = max(self.lead['dRel'] + self.lead['vRel'] * DT_MDL, 0.)
    return dict(self.lead)


def lane_mismatch(lead: dict[str, Any], lead_msg: capnp._DynamicStructReader) -> bool:
  if not (lead.get('status') and lead.get('radar')):
    return False
  vis_y = -lead_msg.y[0]
  return bool(lead_msg.prob > LANE_MISMATCH_VISION_PROB and abs(vis_y) < LANE_MISMATCH_VISION_Y
              and abs(lead['yRel']) > LANE_MISMATCH_RADAR_Y and abs(lead['yRel'] - vis_y) > LANE_MISMATCH_DY
              and lead['vLead'] - lead_msg.v[0] > LANE_MISMATCH_DV and lead['dRel'] < LANE_MISMATCH_MAX_DIST)


class RadarDCustin(RadarD):
  def __init__(self, delay: float = 0.0):
    super().__init__(delay)
    self.prev_lead_track_ids = [-1, -1]
    self.lead_hold = LeadHold()
    self.lane_mismatch_frames = 0
    self.keep_id = -1
    self.keep_age = 1e9

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]
      v_lead = rpt[2] + self.v_ego_hist[0]
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      lead_one = get_lead_custin(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                                 self.lead_prob_filters[0].x, low_speed_override=True,
                                 preferred_track_id=self.prev_lead_track_ids[0])
      # see RADAR_KEEP_*
      if lead_one.get('radar'):
        self.keep_id, self.keep_age = int(lead_one['radarTrackId']), 0.
      else:
        self.keep_age += DT_MDL
        trk = self.tracks.get(self.keep_id)
        if (RADAR_KEEP_S > 0. and lead_one['status'] and trk is not None and trk.cnt >= 3 and self.keep_age < RADAR_KEEP_S
            and abs(trk.yRel + leads_v3[0].y[0]) < RADAR_KEEP_DY and lead_one['dRel'] >= RADAR_KEEP_NEAR * trk.dRel):
          lead_one = trk.get_RadarState(self.lead_prob_filters[0].x)
          if RADAR_KEEP_REFRESH:
            self.keep_age = 0.
      # see LANE_MISMATCH_*: the radar has moved to the next lane and vision's lead is slower
      self.lane_mismatch_frames = self.lane_mismatch_frames + 1 if lane_mismatch(lead_one, leads_v3[0]) else 0
      if self.lane_mismatch_frames >= LANE_MISMATCH_FRAMES:
        lead_one = get_RadarState_from_vision(leads_v3[0], self.v_ego, model_v_ego, self.lead_prob_filters[0].x)
      self.radar_state.leadOne = self.lead_hold.update(lead_one, leads_v3[0].prob, self.v_ego)
      self.radar_state.leadTwo = get_lead_custin(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
                                                 self.lead_prob_filters[1].x, low_speed_override=False,
                                                 preferred_track_id=self.prev_lead_track_ids[1])
      self.prev_lead_track_ids = [int(self.radar_state.leadOne.radarTrackId),
                                  int(self.radar_state.leadTwo.radarTrackId)]
