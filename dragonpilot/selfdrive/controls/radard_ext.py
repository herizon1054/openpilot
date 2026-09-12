#!/usr/bin/env python3
import capnp
import numpy as np
from typing import Any
from cereal import messaging, car

# ==============================================================================
# 1. 引入整個 radard 模組進行 Monkey Patch (動態替換)
# ==============================================================================
from openpilot.selfdrive.controls import radard

# 2. 正常引入我們需要的元件與原始函數 (移除 DP 中不存在的 structs 與客製函數)
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, RADAR_TO_CAMERA
)

# 3. 引入 cloudlog 用於記錄我們自訂的提早鎖定事件
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# 提早鎖定 (Early Lock) 擴充模組參數設定
# ==============================================================================
LANE_WIDTH_FALLBACK = 1.5           # 預測車道基準單側半寬 (m)
LANE_HYSTERESIS_MARGIN = 0.5        # 邊界外的遲滯容錯預度 (m)
FUZZY_BOUNDS = [0.5, 1.5]           # 物理誤差 (m 或 m/s): 0.5 以內給滿分 1.0，大於 1.5 總分歸零

ALPHA_BASE = 0.2                    # 常規上升學習率
ALPHA_DOWN = 0.1                    # 常規下降與短路過濾時的衰減學習率

BRAKE_THRES_RANGE = [-3.0, -1.2]    # 急煞觸發區間 (m/s²)
MULT_RANGE = [1.2, 1.0]             # 對應威脅倍率
CUTIN_DIST_LIMIT = 40.0             # 評估切入威脅的最大縱向有效距離 (m)
DYNAMIC_SPEED_PCT = 0.2             # 動態相對速度閥值比例

CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 動態相機門檻車速區間
CAM_PROB_RANGE = [0.5, 0.3]         # 動態相機審查門檻
STATIC_EMA_CAP = 0.6                # 目標未達審查門檻時的 EMA 天花板

EMA_VAL_RANGE = [0.4, 0.8]          # 本地 EMA 信心度 X 軸
PROB_THRES_RANGE = [0.5, 0.3]       # 映射出對應的「視覺提早放行門檻」 Y 軸

RELEASE_FRAMES = 5                  # 目標短暫丟失或出界時的 EMA 續命凍結幀數
SELECT_HOLDOVER_FRAMES = 3          # 雷達硬體斷流時，強制維持上一幀鎖定的幀數

MODEL_TAU_MIN_PROB = 0.5            # 啟動驗證的最低視覺機率
MODEL_TAU_BRAKE_A = -0.5            # 啟動驗證的最低急煞門檻 (m/s²)
MODEL_TAU_SUSTAINED = 0.5           # 視覺確認急煞持續
MODEL_TAU_SPURIOUS = 3.0            # 視覺預測即將加速

# 全域快取：改回 Candy 版邏輯，直接快取 Track 物件本身
# dp: 額外加上 last_aLeadK，用來在「凍結中」跟「剛恢復匹配」兩種情況下，
# 都對輸出的 aLeadK 做變化率限制，避免瞬間跳動觸發幽靈煞車
_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0, 'last_aLeadK': None},
    1: {'track': None, 'absent': 0, 'last_aLeadK': None}
}
MAX_ALEADK_DELTA_PER_FRAME = 1.0    # aLeadK 每幀最大允許變化量 (m/s²)，可依實測調整


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


class TrackDP(Track):
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

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float, is_turning: bool = False):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # dp: 「必須真實量測」這道門檻改成只在轉彎時生效——
    # 轉彎時保留保護，避免旁側車道目標因外推值誤判成切入本車道；
    # 直行/巡航時放行，避免正常雷達漏拍拖慢插隊車輛的信心度累積、反應變慢半拍。
    is_invalid = (is_turning and not self.measured) or abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    
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
  tracks: dict[int, TrackDP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  lead_prob: float,
  is_turning: bool = False,
  low_speed_override: bool = True,
) -> dict[str, Any]:
  """
  DP 適配版：移除了 CP 與 CP_SP，純粹依靠 DP 的系統參數運作。
  新增 is_turning：由 radard.py 依方向盤角度/角速度判斷是否正在轉彎，
  轉出去給 process_track_logic 決定是否要求「必須真實量測」。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob, is_turning)

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  current_prob_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))

  selected_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    selected_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)

  # 狀態機記憶：還原 Candy 版的殭屍物件強制續命邏輯 (直接快取物件)
  cache = _LEAD_STATE_CACHE[lead_idx]
  if selected_track is not None:
    cache['track'] = selected_track
    cache['absent'] = 0
  elif cache['track'] is not None:
    cache['absent'] += 1
    if cache['absent'] <= SELECT_HOLDOVER_FRAMES:
      selected_track = cache['track']  # 強制回傳上一刻的凍結物件，維持鎖定
    else:
      cache['track'] = None
      cache['absent'] = 0
      cache['last_aLeadK'] = None  # lead 真正消失，重置參考基準，避免下一個新目標被錯誤地拿舊值做限制

  lead_dict = {'status': False}
  if selected_track is not None:
    lead_dict = selected_track.get_RadarState(lead_prob)

    # dp: 不管是「凍結續命中」還是「剛恢復匹配、瞬間跳到最新卡曼值」，
    # 都對 aLeadK 做變化率限制，避免瞬間跳動被誤判成前車突然減速（幽靈煞車）。
    # 只限制 aLeadK，dRel/yRel/vRel 不受影響，維持插隊偵測所需的位置即時性。
    if cache['last_aLeadK'] is not None:
      raw_aLeadK = lead_dict['aLeadK']
      delta = float(np.clip(raw_aLeadK - cache['last_aLeadK'], -MAX_ALEADK_DELTA_PER_FRAME, MAX_ALEADK_DELTA_PER_FRAME))
      lead_dict['aLeadK'] = float(cache['last_aLeadK'] + delta)
    cache['last_aLeadK'] = float(lead_dict['aLeadK'])

    # 視覺加速度雙重驗證阻尼
    model_tau = get_model_lead_tau(lead_msg, lead_prob)
    if model_tau is not None:
      lead_dict['aLeadTau'] = model_tau

    if current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarD_EarlyLock_DP] 提早鎖定/續命成功！目標 {lead_idx} | "
        f"相機機率: {lead_prob:.2f} (動態門檻: {current_prob_thres:.2f})"
      )

  elif (selected_track is None) and ready and (lead_prob > current_prob_thres):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)
    _LEAD_STATE_CACHE[lead_idx]['last_aLeadK'] = None  # 純視覺後備路徑不經過雷達物件，重置參考基準


  # 原廠底線救援
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


# ==============================================================================
# 雙重 Monkey Patching
# ==============================================================================
radard.Track = TrackDP
radard.get_lead = get_lead_ext


class RadarDExt(RadarD):
  """
  DP 版專屬：初始化參數對齊 DP 的單一 delay 參數。
  """
  def __init__(self, delay: float = 0.0):
    super().__init__(delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
