#!/usr/bin/env python3
import capnp
import numpy as np
from typing import Any
from openpilot.cereal import messaging
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, get_custom_yrel, RADAR_TO_CAMERA
)
from openpilot.common.swaglog import cloudlog

LANE_WIDTH_FALLBACK = 1.5
LANE_HYSTERESIS_MARGIN = 0.5
FUZZY_BOUNDS = [0.5, 1.5]

ALPHA_BASE = 0.2
ALPHA_DOWN = 0.1

BRAKE_THRES_RANGE = [-3.0, -1.2]
MULT_RANGE = [1.2, 1.0]
CUTIN_DIST_LIMIT = 40.0
DYNAMIC_SPEED_PCT = 0.2

CAM_PROB_SPEED_RANGE = [10.0, 25.0]
CAM_PROB_RANGE = [0.5, 0.3]
STATIC_EMA_CAP = 0.6

EMA_VAL_RANGE = [0.4, 0.8]
PROB_THRES_RANGE = [0.5, 0.3]

RELEASE_FRAMES = 5
SELECT_HOLDOVER_FRAMES = 3

MODEL_TAU_MIN_PROB = 0.5
MODEL_TAU_BRAKE_A = -0.5
MODEL_TAU_SUSTAINED = 0.5
MODEL_TAU_SPURIOUS = 3.0

_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0},
    1: {'track': None, 'absent': 0}
}

def get_model_lead_tau(lead_msg, lead_prob: float) -> float | None:
  if lead_prob < MODEL_TAU_MIN_PROB or len(lead_msg.a) < 2:
    return None

  a0 = float(lead_msg.a[0])
  a1 = float(lead_msg.a[1])

  if a0 > MODEL_TAU_BRAKE_A:
    return None
  if a1 < 0.5 * a0:
    return MODEL_TAU_SUSTAINED
  if a1 > 0.1 * a0:
    return MODEL_TAU_SPURIOUS

  return None

class TrackSP(Track):
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    self.ema_confidence = {0: 0.4, 1: 0.4}
    self.holdover_frames = {0: 0, 1: 0}
    self.is_out_of_lane = False

  def _check_spatial_boundaries(self, vision_y: float) -> bool:
    left_bound = vision_y + LANE_WIDTH_FALLBACK
    right_bound = vision_y - LANE_WIDTH_FALLBACK
    current_y = self.yRel

    if not self.is_out_of_lane:
      if current_y > (left_bound + LANE_HYSTERESIS_MARGIN) or current_y < (right_bound - LANE_HYSTERESIS_MARGIN):
        self.is_out_of_lane = True
    else:
      if right_bound <= current_y <= left_bound:
        self.is_out_of_lane = False

    return not self.is_out_of_lane

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    score_d = float(np.interp(err_d, FUZZY_BOUNDS, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))

    return score_d * score_y * score_v

  def _calculate_threat_multipliers(self, v_ego: float) -> float:
    brake_mult = float(np.interp(self.aLeadK, BRAKE_THRES_RANGE, MULT_RANGE))
    cutin_mult = 1.0

    if self.dRel < CUTIN_DIST_LIMIT and abs(self.yRel) > 1.0:
      v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)
      cutin_mult = float(np.interp(self.vRel, [-v_limit, v_limit], MULT_RANGE))

    final_alpha = ALPHA_BASE * brake_mult * cutin_mult
    return min(1.0, final_alpha)

  def _apply_slow_protection(self, v_ego: float, cam_prob: float, current_ema: float) -> float:
    abs_v_lead = abs(self.vRel + v_ego)
    dynamic_v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)

    if abs_v_lead < dynamic_v_limit:
      dynamic_cam_prob_thres = float(np.interp(v_ego, CAM_PROB_SPEED_RANGE, CAM_PROB_RANGE))
      if cam_prob < dynamic_cam_prob_thres:
        return min(current_ema, STATIC_EMA_CAP)

    return current_ema

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    is_invalid = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)

    fuzzy_score = 0.0
    if not is_invalid:
      is_valid_spatial = self._check_spatial_boundaries(vision_y)
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego)
      is_invalid = not is_valid_spatial or fuzzy_score == 0.0

    if is_invalid:
      if self.holdover_frames[lead_idx] > 0:
        self.holdover_frames[lead_idx] -= 1
        return
      else:
        self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
        return

    self.holdover_frames[lead_idx] = RELEASE_FRAMES

    final_alpha_up = self._calculate_threat_multipliers(v_ego)
    target_ema = fuzzy_score
    alpha = final_alpha_up if fuzzy_score > 0.5 else ALPHA_DOWN

    new_ema = alpha * target_ema + (1 - alpha) * self.ema_confidence[lead_idx]
    new_ema = self._apply_slow_protection(v_ego, lead_prob, new_ema)

    self.ema_confidence[lead_idx] = new_ema


def get_lead_ext(
  v_ego: float,
  ready: bool,
  tracks: dict[int, TrackSP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  lead_prob: float,
  CP: structs.CarParams,
  CP_SP: structs.CarParamsSP,
  low_speed_override: bool = True,
) -> dict[str, Any]:
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob)

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  selected_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    selected_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)

  cache = _LEAD_STATE_CACHE[lead_idx]
  if selected_track is not None:
    cache['track'] = selected_track
    cache['absent'] = 0
  elif cache['track'] is not None:
    cache['absent'] += 1
    if cache['absent'] <= SELECT_HOLDOVER_FRAMES:
      selected_track = cache['track']
    else:
      cache['track'] = None
      cache['absent'] = 0

  lead_dict = {'present': False}
  if selected_track is not None:
    lead_dict = selected_track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)

    model_tau = get_model_lead_tau(lead_msg, lead_prob)
    if model_tau is not None:
      lead_dict['aLeadTau'] = model_tau

    if current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarDSP_EarlyLock] Target {lead_idx} | "
        f"Prob: {lead_prob:.2f} (Thres: {current_prob_thres:.2f})"
      )

  elif (selected_track is None) and ready and (lead_prob > current_prob_thres):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict

radard.Track = TrackSP
radard.get_lead = get_lead_ext

class RadarDSP(RadarD):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    super().__init__(CP, CP_SP, delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
