#!/usr/bin/env python3
import capnp
import math
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

# dp: 持續性救援 —— 目標連續被判定為有效（valid_streak_frames，容忍續命寬限期內
# 的短暫失效，不因單幀漏拍就中斷），即使信心度（ema_confidence）因為幾何比對品質
# 普通、一直停在初始值附近沒有爬高，持續時間本身也視為證據，門檻依持續時間分級放寬
# （見下方兩段式門檻）。以下四個條件先框定啟用範圍：
PERSISTENCE_RESCUE_MAX_ANGLE = 25.0  # 方向盤角度需在 ±此值以內才啟用；25 度對應時速 40 左右常見彎道
                                      # 的估計角度，遠低於市區路口轉彎所需角度（170 度以上），確保真正
                                      # 轉彎時機制一定關閉，同時不會讓一般彎道就整個失效。
PERSISTENCE_RESCUE_MAX_DIST_CAP = 100.0  # 最長偵測距離上限 (m)，車速達到/超過對應值後不再繼續放大
PERSISTENCE_RESCUE_DIST_PER_KMH = 1.0    # 每 1 km/h 車速對應 1 公尺偵測距離（車速 10km/h→10m，100km/h→100m 封頂）
PERSISTENCE_RESCUE_MAX_LATERAL_DEVIATION = 1.75  # 半個車道寬度 (m)，用曲率反推「預測路徑偏移這麼多之前，
                                                  # 距離都算可信」的距離上限，取代原本隨手訂的線性遞減
PERSISTENCE_RESCUE_MAX_YREL = 1.0    # 車道中心左右各 1 公尺，超出此範圍視為非本車道目標，不啟用救援

# dp: 兩段式持續性救援 —— 雷達持續偵測（valid_streak_frames）越久，容許的視覺信心度
# 門檻越寬鬆，分兩級漸進，不是一次到位：
#   持續 1.0 秒 → 門檻最多放寬到 0.4
#   持續 2.0 秒 → 門檻最多放寬到 0.3（PROB_THRES_RANGE 裡最寬鬆的一端）
PERSISTENCE_RESCUE_TIER1_SEC = 1.0
PERSISTENCE_RESCUE_TIER1_FRAMES = int(PERSISTENCE_RESCUE_TIER1_SEC / DT_MDL)
PERSISTENCE_RESCUE_TIER1_PROB_THRES = 0.4

PERSISTENCE_RESCUE_TIER2_SEC = 2.0
PERSISTENCE_RESCUE_TIER2_FRAMES = int(PERSISTENCE_RESCUE_TIER2_SEC / DT_MDL)
PERSISTENCE_RESCUE_TIER2_PROB_THRES = 0.3

# dp: 視覺機率不用跟雷達秒數掛勾——雷達的持續時間本身就是「這是真實物體」的證據，
# 視覺只需要短暫確認幾幀，排除單幀雜訊尖峰即可，不需要也撐滿跟雷達一樣長的時間
# （硬性要求視覺也要長時間可靠，會跟「視覺本來就不可靠、靠雷達補強」的設計初衷矛盾）。
PERSISTENCE_RESCUE_PROB_CONFIRM_FRAMES = 3

# dp(log 驗證修正 A): 上面的 3 幀確認，必須用「未濾波」的原始 leadsV3[i].prob 計數。
# radard.py 傳進來的 lead_prob 已經過非對稱濾波（上升瞬間到位、下降 alpha=0.2），
# 單幀 0.45 尖峰會被撐成 0.45→0.38→0.32，連續 3 幀 >0.3，確認形同虛設。
# 門檻比較（lead_prob > current_prob_thres）仍沿用濾波後的值，行為與原本一致。

# dp(修正 6): 靜止/慢速目標不參與持續性救援。判定標準與 _apply_slow_protection 相同：
# 目標絕對速度 < max(1.0, DYNAMIC_SPEED_PCT * v_ego)。路邊違停、施工護欄、待轉機車
# 屬於這類，交給原廠 low_speed_override 與正常 0.5 門檻處理。

# dp(修正 7): 救援偵測距離下限。原本 v_kmh * 1.0，15~30 km/h 市區走走停停時只有 15~30m。
PERSISTENCE_RESCUE_MIN_DIST = 20.0

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
# 誤差常超過 1.5m，導致 EMA 與 valid_streak 在 ~30m 外無法累積。
# 比照原廠 dist_sane 精神（25% 或 5m），但取較保守的比例。
FUZZY_D_BOUNDS_PCT = [0.05, 0.12]   # 滿分 / 歸零 對應的距離比例

# 全域快取：改回 Candy 版邏輯，直接快取 Track 物件本身
# dp: 額外加上 last_aLeadK，用來在「凍結中」跟「剛恢復匹配」兩種情況下，
# 都對輸出的 aLeadK 做變化率限制，避免瞬間跳動觸發幽靈煞車
# 另外加上 prob_tier1_frames/prob_tier2_frames：視覺機率連續維持在對應門檻之上
# 的幀數，只需連續 PERSISTENCE_RESCUE_PROB_CONFIRM_FRAMES 幀即可，不用跟雷達秒數一樣長。
_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0, 'last_aLeadK': None, 'prob_tier1_frames': 0, 'prob_tier2_frames': 0},
    1: {'track': None, 'absent': 0, 'last_aLeadK': None, 'prob_tier1_frames': 0, 'prob_tier2_frames': 0}
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
    self.valid_streak_frames = {0: 0, 1: 0}    # dp: 連續被判定為有效（含續命寬限期）的幀數

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

  def is_slow_or_stationary(self, v_ego: float) -> bool:
    return abs(self.vRel + v_ego) < max(1.0, DYNAMIC_SPEED_PCT * v_ego)

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
        return   # 續命寬限期內的短暫失效，視為連續的一部分，不中斷 valid_streak_frames
      else:
        self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
        self.valid_streak_frames[lead_idx] = 0   # 寬限期已用盡，真正失效，連續紀錄歸零
        return

    self.holdover_frames[lead_idx] = RELEASE_FRAMES
    self.valid_streak_frames[lead_idx] += 1

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
  新增 steering_angle_deg/steer_ratio/wheelbase：用自行車模型從方向盤角度反推
  路徑曲率，供「持續性救援」判斷目標是否落在預測路徑走廊內，並讓角度越大、
  偵測距離連續縮短，取代原本單純的角度二元開關。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  # dp(修正 4): 優先使用模型規劃路徑（modelV2.position；模型座標 y 向右為正，取負號
  # 轉成 radar 座標 y 向左為正；x 由攝影機起算，故以 dRel + RADAR_TO_CAMERA 查表）。
  # 模型路徑能預見前方彎道，彎道入口「車頭還直、路已經在彎」時不會把彎道外側的
  # 路邊物體框進走廊。低速或路徑無效時退回原本的自行車模型。
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

  # dp: 兩段式持續性救援——找出符合「距離在範圍內、橫向在車道中心 ±範圍內」的
  # valid track 裡，持續有效幀數最長的一個，依它的持續時間套用對應的寬鬆門檻。
  # 方向盤角度超標（正在轉彎）時整段不啟用。
  normal_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))
  current_prob_thres = normal_thres
  is_persistent = False
  best_streak = 0

  # dp: 視覺機率連續維持在對應門檻之上的幀數，只需短暫確認（見下方 PROB_CONFIRM_FRAMES），
  # 不用跟雷達的持續秒數一樣長。
  rescue_cache = _LEAD_STATE_CACHE[lead_idx]
  # dp(修正 A): 用原始機率計數（見 PERSISTENCE_RESCUE_PROB_CONFIRM_FRAMES 下方說明）
  confirm_prob = lead_prob if raw_lead_prob is None else raw_lead_prob
  rescue_cache['prob_tier1_frames'] = rescue_cache['prob_tier1_frames'] + 1 if confirm_prob > PERSISTENCE_RESCUE_TIER1_PROB_THRES else 0
  rescue_cache['prob_tier2_frames'] = rescue_cache['prob_tier2_frames'] + 1 if confirm_prob > PERSISTENCE_RESCUE_TIER2_PROB_THRES else 0

  # dp: 最長偵測距離隨車速動態放大——車速越快，需要救援機制涵蓋的距離越遠
  # （高速時同樣的物理距離，留給反應的時間更短），車速 10km/h 對應 10m，
  # 100km/h 以上封頂在 PERSISTENCE_RESCUE_MAX_DIST_CAP（100m）。
  v_ego_kmh = v_ego * 3.6
  # dp(修正 7): 加上 PERSISTENCE_RESCUE_MIN_DIST 下限
  base_max_dist = min(PERSISTENCE_RESCUE_MAX_DIST_CAP,
                      max(PERSISTENCE_RESCUE_MIN_DIST, v_ego_kmh * PERSISTENCE_RESCUE_DIST_PER_KMH))

  # dp: 路徑預測——用自行車模型從方向盤角度反推路徑曲率，算出每個雷達目標所在
  # 距離上，預測路徑應該落在哪個橫向位置，取代原本單純假設「直線、車道中心」
  # 的固定 ±PERSISTENCE_RESCUE_MAX_YREL 判斷。直行時 predicted_y≈0，效果跟原本
  # 一樣；彎道時預測路徑會跟著彎，判斷更準確。
  if steer_ratio > 0 and wheelbase > 0:
    curvature = math.tan(math.radians(steering_angle_deg) / steer_ratio) / wheelbase
  else:
    curvature = 0.0

  # dp: 用同一個曲率，反推「預測路徑偏移超過半個車道寬之前，這個距離都還算可信」
  # 的距離上限——用實際幾何算出來，取代原本隨角度線性遞減的粗略估計。彎道越急
  # （曲率越大），這個距離上限自然越短；完全直行（curvature≈0）時不受此限制。
  if use_model_path:
    persistence_rescue_max_dist = min(base_max_dist, float(px[-1] - RADAR_TO_CAMERA))
  elif abs(curvature) > 1e-6:
    curvature_max_dist = math.sqrt(2 * PERSISTENCE_RESCUE_MAX_LATERAL_DEVIATION / abs(curvature))
    persistence_rescue_max_dist = min(base_max_dist, curvature_max_dist)
  else:
    persistence_rescue_max_dist = base_max_dist

  # dp: 角度硬性開關——超過 PERSISTENCE_RESCUE_MAX_ANGLE（25 度）代表已經是真正的
  # 轉彎/路口動作，不是一般彎道，直接把距離歸零，確保機制完全關閉，不只依賴
  # 曲率公式自然收斂（曲率公式本身即使角度很大也不會真的算出 0）。
  if abs(steering_angle_deg) >= PERSISTENCE_RESCUE_MAX_ANGLE:
    persistence_rescue_max_dist = 0.0

  eligible_streaks = []
  for track in valid_tracks.values():
    if track.dRel > persistence_rescue_max_dist:
      continue
    # dp(修正 6): 靜止/慢速目標不參與救援
    if track.is_slow_or_stationary(v_ego):
      continue
    if use_model_path:
      predicted_y = _path_y_at(track.dRel)
    else:
      predicted_y = curvature * (track.dRel ** 2) / 2.0
    if abs(track.yRel - predicted_y) <= PERSISTENCE_RESCUE_MAX_YREL:
      eligible_streaks.append(track.valid_streak_frames[lead_idx])
  best_streak = max(eligible_streaks, default=0)

  if (best_streak >= PERSISTENCE_RESCUE_TIER2_FRAMES and
      rescue_cache['prob_tier2_frames'] >= PERSISTENCE_RESCUE_PROB_CONFIRM_FRAMES):
    current_prob_thres = min(current_prob_thres, PERSISTENCE_RESCUE_TIER2_PROB_THRES)
    is_persistent = True
  elif (best_streak >= PERSISTENCE_RESCUE_TIER1_FRAMES and
        rescue_cache['prob_tier1_frames'] >= PERSISTENCE_RESCUE_PROB_CONFIRM_FRAMES):
    current_prob_thres = min(current_prob_thres, PERSISTENCE_RESCUE_TIER1_PROB_THRES)
    is_persistent = True

  if is_persistent and normal_thres > current_prob_thres and normal_thres >= lead_prob > current_prob_thres:
    cloudlog.debug(
      f"[RadarD_Persistence_DP] 持續性救援啟用！目標 {lead_idx} | "
      f"信心度: {max_ema_confidence:.2f} | 相機機率: {lead_prob:.2f} | 雷達持續幀數: {best_streak} | "
      f"視覺維持幀數(tier1/tier2): {rescue_cache['prob_tier1_frames']}/{rescue_cache['prob_tier2_frames']} "
      f"(原門檻: {normal_thres:.2f} → 救援後: {current_prob_thres:.2f})"
    )


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
  def __init__(self, delay: float = 0.0, steer_ratio: float = 15.0, wheelbase: float = 2.7):
    super().__init__(delay, steer_ratio, wheelbase)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
