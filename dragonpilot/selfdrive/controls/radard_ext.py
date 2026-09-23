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
from openpilot.common.realtime import DT_MDL

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

# dp: 比照原廠 match_vision_to_track() 的 vel_sane 擇一寬鬆備援：
#   vel_sane = (誤差 < 10) OR (v_ego + vRel > 3)
# 「正在明顯接近」時，就算速度誤差略大也視為合理，不讓 score_v 把總分拉到 0。
# 要求連續多幀都符合「正在接近」才啟用，避免單幀雷達雜訊造成的速度突然跳動誤觸發。
VEL_SANE_FALLBACK_SPEED = 3.0       # 原廠門檻：接近速度 (v_ego + vRel) 超過此值視為明顯接近
VEL_SANE_CONFIRM_FRAMES = 3         # 需連續幾幀都符合才真正啟用寬鬆備援
VEL_SANE_FALLBACK_SCORE = 1.0       # 啟用後 score_v 的下限，1.0 = 完全比照原廠「視為合理」的語意

# dp: 雷達主導救援（取代原本的兩段式持續性救援）
# 原本的救援用 valid_streak_frames（雷達與視覺吻合的連續幀數）當證據，並把門檻降到 0.4/0.3，
# 但持續計數本身依賴視覺位置吻合，視覺一不可靠就先中斷。10 段 log 重播中，因救援而
# 改變的前車選擇累計 0 幀。改為以「雷達本身的穩定性」當證據：
#   一個雷達目標連續 RADAR_RESCUE_CONFIRM_FRAMES 幀同時滿足下列條件，即成為救援候選：
#     - 真實量測（measured）
#     - 向前移動：絕對速度 > max(RADAR_RESCUE_MIN_SPEED, RADAR_RESCUE_MIN_SPEED_PCT * v_ego)，
#       排除靜止物（路邊違停、護欄、ETC 門架）與對向來車
#     - 位在模型路徑走廊內：|yRel - 模型路徑y(d)| < RADAR_RESCUE_CORRIDOR
#     - 距離在 [RADAR_RESCUE_MIN_DIST, RADAR_RESCUE_MAX_DIST]
#     - 方向盤角度 < RADAR_RESCUE_MAX_ANGLE（路口轉彎時關閉）
#     - 模型路徑有效（車速 > MODEL_PATH_MIN_SPEED）
#   任一幀不符合即歸零重新計數。
# 採用條件（僅 leadOne）：
#     - 視覺信心度 >= RADAR_RESCUE_MIN_PROB（總視覺信心度門檻下限），以「未濾波」的原始
#       leadsV3[0].prob 判斷（最嚴格版本）。radard.py 的 lead_prob 經非對稱濾波（上升瞬間到位、
#       下降每幀 alpha=0.2），濾波值恆 >= 原始值，單幀尖峰會被撐住約 5~6 幀；改用原始值後，
#       原始機率一低於 0.1 當幀就停止採用。
#     - 目前沒有前車，或救援候選比現有前車更近
# 10 段 log（走廊 ±1.0m）：純雷達規則觸發時，未曾落在車道線外 0.5m 以上的目標；
# 92104 14.5s（88m 外 21 km/h 慢車，視覺 0.2~0.3）與 92102 33.6s（92m 外 16 km/h，
# 視覺 0.14）為原邏輯遺漏、此規則可補回的本車道慢車。
RADAR_RESCUE_MIN_PROB = 0.1         # 總視覺信心度門檻下限
RADAR_RESCUE_CORRIDOR = 1.0         # 模型路徑左右各 1.0m
RADAR_RESCUE_CONFIRM_FRAMES = int(1.0 / DT_MDL)   # 連續 1 秒
RADAR_RESCUE_MIN_DIST = 5.0
RADAR_RESCUE_MAX_DIST = 120.0
RADAR_RESCUE_MIN_SPEED = 3.0        # m/s
RADAR_RESCUE_MIN_SPEED_PCT = 0.2
RADAR_RESCUE_MAX_ANGLE = 25.0       # deg

# dp(log 驗證修正 1b-v2): 橫向排除改為「無狀態」且必須同時滿足：
#   (a) 落在本車預測路徑走廊外：|yRel - 路徑y(d)| > LANE_CORRIDOR_HALF_WIDTH，且
#   (b) 與視覺前車不一致：橫向差 > LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN (2.0m)，
#       或絕對速度差 > max(LANE_GATE_DV_MIN, LANE_GATE_DV_PCT * |視覺前車速度|)。
# 7 段 log 統計（同一物體的雷達/視覺配對 344 組）：雷達與視覺的橫向差在 20~40m 有 20% 超過
# 2.0m，所以不能只看視覺 y；舊版 1.5m 回歸門檻會把真前車永久鎖在出界狀態（92102 43.4s 起
# 連續 2.8 秒、92104 26.8s 起停等中）。走廊內的目標一律不因視覺 y 不一致而排除。
LANE_CORRIDOR_HALF_WIDTH = 1.75     # 半個車道寬 (m)
# 近距離雷達橫向偏差補償：Toyota 雷達在近距離常打到車尾角落，log 實測 <10m 時雷達與視覺
# 橫向差中位數約 1.0m（92104 26.8s 停等時，正前方 6m 的真前車 R1809 雷達 y≈-2.4m）。
# 距離 <= 8m 走廊半寬加 1.0m，15m 以上不加，中間線性內插。
LANE_CORRIDOR_NEAR_BP = [8.0, 15.0]
LANE_CORRIDOR_NEAR_EXTRA = [1.0, 0.0]
LANE_GATE_DV_MIN = 2.0              # m/s
LANE_GATE_DV_PCT = 0.25

# dp(修正 4): 模型路徑（modelV2.position）預測。車速低於此值時 position.x 會擠在 0 附近、
# 不單調，退回原本的自行車模型。
MODEL_PATH_MIN_SPEED = 3.0          # m/s
MODEL_PATH_MIN_LENGTH = 5.0         # 模型路徑最短有效長度 (m)

# dp(修正 5): fuzzy 距離容差隨距離放大。原本固定 [0.5, 1.5] m，視覺測距在中遠距離
# 誤差常超過 1.5m，導致 EMA 在 ~30m 外無法累積。
# 比照原廠 dist_sane 精神（25% 或 5m），但取較保守的比例。
FUZZY_D_BOUNDS_PCT = [0.05, 0.12]   # 滿分 / 歸零 對應的距離比例

# 全域快取：改回 Candy 版邏輯，直接快取 Track 物件本身
# dp: 額外加上 last_aLeadK，用來在「凍結中」跟「剛恢復匹配」兩種情況下，
# 都對輸出的 aLeadK 做變化率限制，避免瞬間跳動觸發幽靈煞車
_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0, 'last_aLeadK': None, 'rescue': False},
    1: {'track': None, 'absent': 0, 'last_aLeadK': None, 'rescue': False}
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
    # dp(修正 1): 依 lead_idx 分開存。原本單一 bool 會被 leadOne/leadTwo 互相覆寫遲滯狀態。
    self.is_out_of_lane = {0: False, 1: False}
    self.closing_speed_streak = {0: 0, 1: 0}   # dp: 連續幾幀符合「正在快速接近」
    self.radar_rescue_frames = 0               # dp: 連續符合雷達主導救援條件的幀數（與 lead_idx 無關）

  def _check_closing_speed_fallback(self, lead_idx: int, v_ego: float) -> bool:
    # 比照原廠 vel_sane 的 (v_ego + vRel > 3) 這個條件，但要求連續 N 幀都成立
    # 才真正啟用寬鬆備援，避免單幀雷達雜訊造成的速度瞬間跳動誤觸發。
    is_closing_fast = (v_ego + self.vRel) > VEL_SANE_FALLBACK_SPEED
    if is_closing_fast:
      self.closing_speed_streak[lead_idx] = min(self.closing_speed_streak[lead_idx] + 1, VEL_SANE_CONFIRM_FRAMES)
    else:
      self.closing_speed_streak[lead_idx] = 0
    return self.closing_speed_streak[lead_idx] >= VEL_SANE_CONFIRM_FRAMES

  def _update_lane_gate(self, lead_idx: int, vision_y: float, vision_v: float, v_ego: float, path_y: float) -> bool:
    # dp(修正 1b-v2): 無狀態橫向排除，取代原本 _check_spatial_boundaries() 的遲滯鎖存。
    # 原本的遲滯邏輯實際上從未生效（>2.0m 時提早 return，跳過了設定出界的程式），
    # 補上之後又因 1.5m 回歸門檻，讓雷達/視覺橫向偏差 1.5~2.0m 的真前車被永久鎖在出界。
    # 單幀誤判由 get_lead_ext 的 SELECT_HOLDOVER_FRAMES 續命銜接。
    corridor_half = LANE_CORRIDOR_HALF_WIDTH + float(np.interp(self.dRel, LANE_CORRIDOR_NEAR_BP, LANE_CORRIDOR_NEAR_EXTRA))
    outside_corridor = abs(self.yRel - path_y) > corridor_half
    lateral_mismatch = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    speed_mismatch = abs((self.vRel + v_ego) - vision_v) > max(LANE_GATE_DV_MIN, LANE_GATE_DV_PCT * abs(vision_v))
    self.is_out_of_lane[lead_idx] = outside_corridor and (lateral_mismatch or speed_mismatch)
    return self.is_out_of_lane[lead_idx]

  def update_radar_rescue(self, v_ego: float, path_valid: bool, path_y: float, steering_angle_deg: float) -> None:
    # dp: 每幀呼叫一次（只在 leadOne 那次呼叫時更新，避免一幀累加兩次）
    eligible = (path_valid and bool(self.measured) and
                RADAR_RESCUE_MIN_DIST < self.dRel < RADAR_RESCUE_MAX_DIST and
                abs(self.yRel - path_y) < RADAR_RESCUE_CORRIDOR and
                (self.vRel + v_ego) > max(RADAR_RESCUE_MIN_SPEED, RADAR_RESCUE_MIN_SPEED_PCT * v_ego) and
                abs(steering_angle_deg) < RADAR_RESCUE_MAX_ANGLE)
    self.radar_rescue_frames = self.radar_rescue_frames + 1 if eligible else 0

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float, lead_idx: int) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    d_bounds = [max(FUZZY_BOUNDS[0], FUZZY_D_BOUNDS_PCT[0] * offset_vision_dist),
                max(FUZZY_BOUNDS[1], FUZZY_D_BOUNDS_PCT[1] * offset_vision_dist)]
    score_d = float(np.interp(err_d, d_bounds, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))

    # dp: 比照原廠 vel_sane 的擇一寬鬆備援——連續多幀確認正在快速接近時，
    # 即使速度誤差略大，也不讓 score_v 把總分拉到 0。
    if self._check_closing_speed_fallback(lead_idx, v_ego):
      score_v = max(score_v, VEL_SANE_FALLBACK_SCORE)

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

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float, is_turning: bool = False,
                          path_y: float = 0.0):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # dp: 「必須真實量測」這道門檻改成只在轉彎時生效——
    # 轉彎時保留保護，避免旁側車道目標因外推值誤判成切入本車道；
    # 直行/巡航時放行，避免正常雷達漏拍拖慢插隊車輛的信心度累積、反應變慢半拍。
    #
    # dp(log 驗證修正 1b-v2): 原本 is_out_of_lane 從未被設為 True（見 _update_lane_gate 說明），
    # valid_tracks 的橫向過濾等於不存在。log 實證：92216 55.0s 車速 46 km/h，右車道線外
    # 1.1~1.9m、80m 處的靜止物被選為前車；92101 22.1s 右側 5~7m 的靜止物被選為前車。
    is_out = self._update_lane_gate(lead_idx, vision_y, vision_v, v_ego, path_y)
    is_lateral_far = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    is_invalid = (is_turning and not self.measured) or is_lateral_far or is_out

    fuzzy_score = 0.0
    if not is_invalid:
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego, lead_idx)
      is_invalid = fuzzy_score == 0.0

    if is_invalid:
      if self.holdover_frames[lead_idx] > 0:
        self.holdover_frames[lead_idx] -= 1
        return   # 續命寬限期內的短暫失效，EMA 維持不變
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
  steering_angle_deg: float = 0.0,
  steer_ratio: float = 15.0,
  wheelbase: float = 2.7,
  low_speed_override: bool = True,
  raw_lead_prob: float | None = None,
  path_x: list[float] | None = None,
  path_y: list[float] | None = None,
) -> dict[str, Any]:
  """
  DP 適配版：移除了 CP 與 CP_SP，純粹依靠 DP 的系統參數運作。
  新增 is_turning：由 radard.py 依方向盤角度/角速度判斷是否正在轉彎，
  轉出去給 process_track_logic 決定是否要求「必須真實量測」。
  steering_angle_deg：雷達主導救援的方向盤角度開關。
  raw_lead_prob：未濾波的原始 lead 機率，雷達主導救援的視覺信心度下限只看這個值；
  未提供時（例如測試）退回使用濾波後的 lead_prob。
  steer_ratio/wheelbase：原本供自行車模型路徑預測使用，已改用模型路徑（path_x/path_y），
  目前未使用，保留參數僅為維持 radard.py 的呼叫介面。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  # dp(修正 4): 優先使用模型規劃路徑（modelV2.position；模型座標 y 向右為正，取負號
  # 轉成 radar 座標 y 向左為正；x 由攝影機起算，故以 dRel + RADAR_TO_CAMERA 查表）。
  # 模型路徑能預見前方彎道，彎道入口「車頭還直、路已經在彎」時不會把彎道外側的
  # 路邊物體框進走廊。低速或路徑無效時，橫向閘門假設直行，雷達主導救援則停用。
  use_model_path = False
  px = py = None
  if path_x is not None and path_y is not None and len(path_x) >= 2 and len(path_x) == len(path_y):
    px = np.asarray(path_x, dtype=float)
    py = np.asarray(path_y, dtype=float)
    if v_ego > MODEL_PATH_MIN_SPEED and bool(np.all(np.diff(px) > 0)) and (px[-1] - px[0]) > MODEL_PATH_MIN_LENGTH:
      use_model_path = True

  def _path_y_at(d: float) -> float:
    # 走廊中心：模型路徑有效時用模型路徑，否則假設直行（低速/停等/路口大角度轉彎時，
    # 自行車模型的二次外推會失真，不適合拿來做排除）。
    if use_model_path:
      return -float(np.interp(d + RADAR_TO_CAMERA, px, py))
    return 0.0

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob, is_turning, path_y=_path_y_at(track.dRel))

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane[lead_idx] and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  normal_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))
  current_prob_thres = normal_thres   # 總視覺信心度門檻（雷達主導救援生效時會同步降到 RADAR_RESCUE_MIN_PROB）

  # dp: 雷達主導救援——只處理 leadOne，每幀更新一次所有雷達目標的連續幀數
  rescue_track = None
  rescue_prob = lead_prob if raw_lead_prob is None else raw_lead_prob
  if lead_idx == 0:
    for track in tracks.values():
      track.update_radar_rescue(v_ego, use_model_path, _path_y_at(track.dRel), steering_angle_deg)
    rescue_candidates = [t for t in tracks.values() if t.radar_rescue_frames >= RADAR_RESCUE_CONFIRM_FRAMES]
    if ready and rescue_prob >= RADAR_RESCUE_MIN_PROB and len(rescue_candidates) > 0:
      rescue_track = min(rescue_candidates, key=lambda t: t.dRel)

  matched_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    matched_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)

  # 狀態機記憶：還原 Candy 版的殭屍物件強制續命邏輯 (直接快取物件)
  # dp: 先判斷「這一幀若沒有配對成功，是否會由續命沿用上一個目標」，讓雷達主導救援能拿
  # 「現有前車」做距離比較。cache['rescue'] 標記快取中的目標是否來自雷達主導救援。
  cache = _LEAD_STATE_CACHE[lead_idx]
  held_track = None
  if matched_track is None and cache['track'] is not None and cache['absent'] + 1 <= SELECT_HOLDOVER_FRAMES:
    held_track = cache['track']
  held_is_rescue = held_track is not None and cache['rescue']

  # 純視覺後備前車候選：門檻刻意維持 normal_thres（最低 0.3），不跟著雷達主導救援降到 0.1。
  # 0.1 只允許用在「雷達已連續 1 秒確認的本車道移動目標」，不允許單獨由視覺產生前車。
  vision_cand = None
  if matched_track is None and ready and lead_prob > normal_thres:
    vision_cand = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  # 現有前車距離（不含來自救援的續命目標，救援目標每幀都要重新和現有前車比較）
  if matched_track is not None:
    existing_d = matched_track.dRel
  elif held_track is not None and not held_is_rescue:
    existing_d = held_track.dRel
  elif vision_cand is not None:
    existing_d = vision_cand['dRel']
  else:
    existing_d = float('inf')

  # dp: 雷達主導救援採用判斷——沒有前車，或救援候選比現有前車更近才採用
  is_radar_rescue = (rescue_track is not None and rescue_track is not matched_track and
                     rescue_track.dRel < existing_d)

  vision_lead = None
  if is_radar_rescue:
    selected_track = rescue_track
    cache['track'] = rescue_track
    cache['absent'] = 0
    cache['rescue'] = True
    # 同步總視覺信心度門檻：這一幀的前車是在 RADAR_RESCUE_MIN_PROB 門檻下被接受的
    current_prob_thres = min(current_prob_thres, RADAR_RESCUE_MIN_PROB)
    cloudlog.debug(
      f"[RadarD_RadarRescue_DP] 雷達主導救援！目標 {lead_idx} | 雷達 {rescue_track.identifier} "
      f"d={rescue_track.dRel:.1f} y={rescue_track.yRel:.2f} v={rescue_track.vRel + v_ego:.1f} | "
      f"連續 {rescue_track.radar_rescue_frames} 幀 | 相機機率(原始/濾波): {rescue_prob:.2f}/{lead_prob:.2f} "
      f"(原門檻: {normal_thres:.2f} → {current_prob_thres:.2f}) | 原前車距離: {existing_d:.1f}"
    )
  elif matched_track is not None:
    selected_track = matched_track
    cache['track'] = matched_track
    cache['absent'] = 0
    cache['rescue'] = False
  else:
    selected_track = None
    if held_is_rescue:
      # 最嚴格版本：來自雷達主導救援的目標不享有續命。救援條件（原始視覺機率 >= 0.1、
      # 雷達連續 1 秒、移動、走廊內、真實量測）任一項在這一幀不成立，救援前車當幀就撤銷，
      # 不用 SELECT_HOLDOVER_FRAMES 延長。
      cache['track'] = None
      cache['absent'] = 0
      cache['last_aLeadK'] = None
      cache['rescue'] = False
    elif cache['track'] is not None:
      cache['absent'] += 1
      if cache['absent'] <= SELECT_HOLDOVER_FRAMES:
        selected_track = cache['track']  # 強制回傳上一刻的凍結物件，維持鎖定
      else:
        cache['track'] = None
        cache['absent'] = 0
        cache['last_aLeadK'] = None  # lead 真正消失，重置參考基準，避免下一個新目標被錯誤地拿舊值做限制
        cache['rescue'] = False
    if selected_track is None:
      vision_lead = vision_cand

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

    if not is_radar_rescue and current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarD_EarlyLock_DP] 提早鎖定/續命成功！目標 {lead_idx} | "
        f"相機機率: {lead_prob:.2f} (動態門檻: {current_prob_thres:.2f})"
      )

  elif vision_lead is not None:
    lead_dict = vision_lead
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
  def __init__(self, delay: float = 0.0, steer_ratio: float = 15.0, wheelbase: float = 2.7):
    super().__init__(delay, steer_ratio, wheelbase)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
