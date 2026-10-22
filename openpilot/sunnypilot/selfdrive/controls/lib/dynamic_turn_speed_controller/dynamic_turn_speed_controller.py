"""
================================================================================
Dynamic Turn Speed Controller (DTSC) - Final Pro Edition
自動駕駛動態彎道速度控制器 (四階段狀態機 + TTA距離運算 + 非對稱Jerk + 50% EMA防護版) 🚀
================================================================================

【系統運作核心理論 (System Architecture & Control Theory)】
本系統屏棄了傳統依賴「靜態地圖」或「單一曲率門檻」的生硬煞車方式，
改採最先進的「時空軌跡重疊 (Spatio-Temporal Overlap)」與「四階段動態狀態機」。

1. 信心濾波與 EMA 防護 (Confidence & EMA Filter)：
   結合時空平移累積信心度，並在彎心特徵 (K與G值) 提取時加入 50% EMA 指數平滑，
   徹底濾除 AI 視覺模型單幀的雜訊與錯誤預測，消滅幽靈點煞。

2. 危險剝離與彎心鎖定 (Danger Extraction & Apex Targeting)：
   在可信賴的視距內尋找「真正的入彎點」(G>0.3 或 曲率>0.005)，
   直接抓取彎心 (Apex) 數據，反推絕對安全車速。

   🌟 [防護] 實車 G 力強制介入：
   若 AI 信心累積太慢，但底盤實測感受到的橫向 G 力已超越舒適極限，
   將無視 AI，強制啟動減速接管，構築絕對安全底線。

3. 四階段狀態機與雙模控制 (4-Stage State Machine & Dual-Mode Control)：
   - [第一階段] 啟用判定：AI 預見彎道 OR 底盤 G 力超標。
   - [第二階段] 狀態定位：以 0.0m 為界，區分預先減速區 (Pre-decel) 與 彎中動態區 (Turning)。
   - [第三階段] 減速實作：入彎前利用距離反推 TTA 減速；彎中結合當下G力與未來1秒預測G力取最小防護。
   - [第四階段] 出彎判斷：使用實體曲率並執行 20 幀連續驗證，確保 S 彎不提早加速。
================================================================================
"""

import numpy as np
from openpilot.cereal import messaging
from opendbc.car import structs
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase

# ==========================================================
# [0. 系統全域常數與物理極限設定]
# ==========================================================
# 【系統輸出權限界線】
MAX_ACCEL = 1.0        # 解鎖正向加速權限
MIN_ACCEL = ACCEL_MIN  # 最大允許煞車力道線

# 【時間與空間網格化】
NUM_SLOTS = 200
DT_MDL = 0.05
LINEAR_T = np.arange(0.0, 10.0, DT_MDL)
MODEL_T = np.array(ModelConstants.T_IDXS)

# 【彎道特徵與信心門檻】
MIN_CURVATURE = 0.002
TRIGGER_LAT_ACCEL = 0.6
CONF_THRESHOLD = 0.60

# 【人類舒適度動態查表】
COMFORT_SPEEDS_KPH = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0]
COMFORT_LAT_ACCELS = [1.2, 1.2, 1.2, 1.3, 1.4, 1.5, 1.6, 1.6, 1.8, 1.8, 1.8, 1.8, 1.8]


class DynamicTurnSpeedController(TargetsBase):
  def __init__(self, CP: structs.CarParams, mpc=None):
    super().__init__(CP, mpc)
    self.log_timer = 0.0
    self.exit_frame_count = 0

    # 【狀態記憶體配置】
    self.confidence_table = np.zeros(NUM_SLOTS)
    self.ui_confidence = np.zeros(len(MODEL_T))

    # 輸出控制記憶體
    self.v_target_history = np.zeros(10)
    self.a_target_history = np.zeros(10)
    self.last_a_target_raw = 0.0  # 用於 Jerk 限制器

    # EMA 濾波器記憶體
    self.ema_initialized = False
    self.k_future_ema = 0.0
    self.g_future_ema = 0.0

  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    model_msg = sm['modelV2']
    ctrl_name = self.__class__.__name__

    # 【基礎防呆機制】
    if model_msg is None or len(model_msg.position.x) == 0:
      self.v_target, self.a_target = v_cruise, a_ego
      return super().update_target(sm, v_ego, a_ego, v_cruise)

    # ==========================================================
    # 車體狀態與虛擬軌跡萃取
    # ==========================================================
    cs_curvature = sm['controlsState'].curvature if sm.valid['controlsState'] else 0.0

    x_raw = np.array(model_msg.position.x)
    v_raw = np.maximum(np.array(model_msg.velocity.x), 0.1)
    y_rate_raw = np.abs(np.array(model_msg.orientationRate.z))
    pitch_raw = np.array(model_msg.orientation.y)

    x_200 = np.interp(LINEAR_T, MODEL_T, x_raw)
    pitch_200 = np.interp(LINEAR_T, MODEL_T, pitch_raw)

    v_raw_k = np.maximum(v_raw, 1.0)
    k_raw = y_rate_raw / v_raw_k

    g_200 = np.interp(LINEAR_T, MODEL_T, y_rate_raw * v_raw)
    k_200_raw = np.interp(LINEAR_T, MODEL_T, k_raw)

    slope_factor = np.clip(np.abs(pitch_200) * 5.0, 0.0, 0.5)
    k_200_safe = k_200_raw * (1.0 + slope_factor)

    # ==========================================================
    # 信心感知器
    # ==========================================================
    self.confidence_table[:-1] = self.confidence_table[1:]
    self.confidence_table[-1] = 0.0

    curve_mask = (g_200 > TRIGGER_LAT_ACCEL) | (k_200_safe > MIN_CURVATURE)
    self.confidence_table[curve_mask] += 0.05
    self.confidence_table[~curve_mask] -= 0.01
    self.confidence_table = np.clip(self.confidence_table, 0.0, 1.0)

    # ==========================================================
    # 實車動態極限與 G 力計算
    # ==========================================================
    comfort_speeds_ms = [v * CV.KPH_TO_MS for v in COMFORT_SPEEDS_KPH]
    comfort_g_limit = float(np.interp(v_ego, comfort_speeds_ms, COMFORT_LAT_ACCELS))

    # 直接利用查表得出的 G 力極限反推實車安全限速
    act_k_raw = max(abs(cs_curvature), 1e-5)
    act_k_safe = act_k_raw * (1.0 + np.clip(-pitch_200[0] * 5.0, 0.0, 0.5))
    v_actual_decision = np.sqrt(comfort_g_limit / act_k_safe)

    current_lat_accel = (v_ego ** 2) * act_k_raw
    is_g_force_override = (current_lat_accel > comfort_g_limit) and (act_k_raw > MIN_CURVATURE)

    # ==========================================================
    # AI 動態視距與 50% EMA 特徵過濾
    # ==========================================================
    ai_sees_curve = False
    v_model_decision = v_cruise
    k_future_max = 0.0001
    g_future_max = 0.0
    dist_to_entry, dyn_horizon_dist = 0.0, 0.0
    entry_sec, horizon_sec, entry_conf, horizon_conf = 0.0, 0.0, 0.0, 0.0

    valid_indices = np.where(self.confidence_table > CONF_THRESHOLD)[0]

    if len(valid_indices) > 0:
      ai_sees_curve = True
      horizon_idx = valid_indices[-1]

      dangerous_pts = np.where((g_200[valid_indices] > 0.3) | (k_200_safe[valid_indices] > 0.005))[0]
      entry_idx = valid_indices[dangerous_pts[0]] if len(dangerous_pts) > 0 else valid_indices[0]

      dist_to_entry = x_200[entry_idx]
      dyn_horizon_dist = x_200[horizon_idx]
      entry_sec, horizon_sec = entry_idx * DT_MDL, horizon_idx * DT_MDL
      entry_conf, horizon_conf = self.confidence_table[entry_idx] * 100, self.confidence_table[horizon_idx] * 100

      # 徹底過濾掉模型單幀預測的錯誤跳動
      k_raw_max = float(np.max(k_200_safe[valid_indices]))
      g_raw_max = float(np.max(g_200[valid_indices]))

      if not self.ema_initialized:
        self.k_future_ema = k_raw_max
        self.g_future_ema = g_raw_max
        self.ema_initialized = True
      else:
        # Alpha = 0.5 (新舊各佔一半，形成平滑趨勢)
        self.k_future_ema = (k_raw_max * 0.5) + (self.k_future_ema * 0.5)
        self.g_future_ema = (g_raw_max * 0.5) + (self.g_future_ema * 0.5)

      k_future_max = self.k_future_ema
      g_future_max = self.g_future_ema

      # 直接以查表極限計算預測的安全速度
      v_model_decision = np.sqrt(comfort_g_limit / max(k_future_max, 1e-5))
    else:
      self.ema_initialized = False # 無彎道時重置 EMA

    # ==========================================================
    # 【第一階段】 啟用/停用判斷
    # ==========================================================
    is_curve_ahead = False

    if ai_sees_curve:
      is_curve_ahead = True
      v_decision_final = min(v_model_decision, v_actual_decision, v_cruise)
    elif is_g_force_override:
      is_curve_ahead = True
      v_decision_final = min(v_actual_decision, v_cruise)
      dist_to_entry = 0.0
      k_future_max = act_k_safe
      g_future_max = current_lat_accel
    else:
      v_decision_final = v_cruise

    if v_ego > 0.1 and not self.action and is_curve_ahead:
      self.action = True

    # ==========================================================
    # 【第二階段】 狀態定位 (以 0.0m 完美分界)
    # ==========================================================
    effective_dist = 0.0 if (is_g_force_override and not ai_sees_curve) else dist_to_entry

    is_pre_deceleration = effective_dist > 0.0
    is_in_curve_dynamic = effective_dist <= 0.0

    # ==========================================================
    # 【第三階段】 實作減速
    # ==========================================================
    a_target_raw = 0.0

    if self.action:
      if is_curve_ahead:
        # --------------------------------------------------------
        # (A) 預先減速實作：TTA 距離推算法
        # --------------------------------------------------------
        if is_pre_deceleration:
          if v_ego > v_decision_final:
            # 🌟 [利用距離計算 TTA]
            # TTA = 當前位置到目標位置的距離 / 當前車速
            tta = effective_dist / max(v_ego, 1.0)
            # 再用速度差除以 TTA 得出完美的等減速度
            a_req = (v_decision_final - v_ego) / max(tta, 0.1)
            a_target_raw = min(a_req, 0.0)
          else:
            # 🌟 極簡防護：若前方確實有離心威脅，鎖死油門滑過去
            if g_future_max > (comfort_g_limit * 0.5):
              a_target_raw = 0.0
            else:
              a_target_raw = (v_decision_final - v_ego) / 2.5

        # --------------------------------------------------------
        # (B) 彎中動態實作：實車 G 力限速 + 未來 1 秒預測融合
        # --------------------------------------------------------
        elif is_in_curve_dynamic:
          # 取未來 1 秒 (20 幀) 預測，與當下實體極限共同防護致動器延遲造成的入彎加速
          k_ahead_1s = float(np.max(k_200_safe[:20]))
          v_pred_1s_k_safe = np.sqrt(comfort_g_limit / max(k_ahead_1s, 1e-5))

          v_curve_target = min(v_actual_decision, v_pred_1s_k_safe, v_cruise)
          speed_diff_curve = v_curve_target - v_ego
          a_target_raw = speed_diff_curve / 1.5

      else:
        # --------------------------------------------------------
        # (C) 出彎過渡期實作：平滑加速
        # --------------------------------------------------------
        speed_diff_exit = v_cruise - v_ego
        a_target_raw = speed_diff_exit / 2.0


    # ==========================================================
    # 【第四階段】 判斷出彎
    # ==========================================================
    exit_condition_raw = (abs(cs_curvature) < MIN_CURVATURE) and (not is_curve_ahead)

    if exit_condition_raw:
      self.exit_frame_count += 1
    else:
      self.exit_frame_count = 0

    exit_condition = (self.exit_frame_count >= 20)

    if (self.action and exit_condition) or v_ego <= 0.1:
      self.action = False
      self.exit_frame_count = 0


    # ==========================================================
    # [非對稱 Jerk 限制器 與 輸出平滑]
    # ==========================================================
    if self.action:
      # 🌟 [非對稱 Jerk 限制器] 限制加速度的劇烈變化
      # 為了確保安全，我們允許煞車踩得比較快 (-2.5)，但補油必須相對柔和 (+1.0)
      JERK_UP_LIMIT = 1.0    # m/s³
      JERK_DOWN_LIMIT = -2.5 # m/s³

      max_delta_up = JERK_UP_LIMIT * DT_MDL
      max_delta_down = JERK_DOWN_LIMIT * DT_MDL

      delta_a = a_target_raw - self.last_a_target_raw
      delta_a = np.clip(delta_a, max_delta_down, max_delta_up)

      a_target_raw = self.last_a_target_raw + delta_a
      self.last_a_target_raw = a_target_raw
    else:
      # 待機時對齊實體現況，防止啟動瞬間爆衝
      self.last_a_target_raw = a_ego
      self.ema_initialized = False

    # 10 幀環形平滑緩衝
    self.a_target_history[1:] = self.a_target_history[:-1]
    self.a_target_history[0] = a_target_raw
    a_target_smoothed = float(np.mean(self.a_target_history))

    self.a_target = np.clip(a_target_smoothed, MIN_ACCEL, MAX_ACCEL)

    if self.action:
      # 統一依賴查表極限 (comfort_g_limit) 計算虛擬目標車速
      v_target_raw = np.sqrt(comfort_g_limit / max(k_future_max, 1e-5)) + (self.a_target * 4.0)
      v_target_raw = np.clip(v_target_raw, 0.0, v_cruise)

      self.v_target_history[1:] = self.v_target_history[:-1]
      self.v_target_history[0] = v_target_raw
      self.v_target = float(np.mean(self.v_target_history))
    else:
      self.v_target_history.fill(v_ego)
      self.a_target_history.fill(a_ego)
      self.v_target, self.a_target = v_cruise, a_ego

    # ==========================================================
    # [UI 視覺渲染與遙測日誌]
    # ==========================================================
    self.ui_confidence = np.interp(MODEL_T, LINEAR_T, self.confidence_table)
    self.log_timer += DT_MDL

    if self.action and self.log_timer >= 0.5:
        if not is_curve_ahead:
          state_str = "🏁 出彎加速"
        elif is_g_force_override and not ai_sees_curve:
          state_str = "⚠️ 實車Ｇ力強制介入"
        elif is_in_curve_dynamic:
          state_str = "🎯 彎中實車G力限速"
        else:
          state_str = "📉 預先減速"

        log_msg = (
          f"[{ctrl_name}] {state_str} | "
          f"V(當前/安全/輸出): {v_ego * 3.6:4.1f}/{v_decision_final * 3.6:4.1f}/{self.v_target * 3.6:4.1f} | "
          f"A(加速度輸出): {self.a_target:5.2f} | "
          f"極限(曲率/橫向G值): {k_future_max:.4f}/{g_future_max:.2f} | "
          f"入彎: {entry_sec:.1f}s/{dist_to_entry:.1f}m/{entry_conf:.0f}% | "
          f"最遠: {horizon_sec:.1f}s/{dyn_horizon_dist:.1f}m/{horizon_conf:.0f}%"
        )
        cloudlog.debug(log_msg)
        print(log_msg)
        self.log_timer = 0.0

    return super().update_target(sm, v_ego, a_ego, v_cruise)