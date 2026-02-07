import time
import numpy as np
from cereal import log
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# [移植注意] 引入 MPC 函式庫
# ==============================================================================
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  COMFORT_BRAKE, STOP_DISTANCE, get_safe_obstacle_distance, 
  get_stopped_equivalence_factor, get_T_FOLLOW
)

# =========================================================
# ACM (Active Coasting Management) 參數設定區
# =========================================================

# --- 1. 滑行速度區間設定 (單位：km/h) ---
SPEED_OFFSET_MIN_KPH = 2.0 
SPEED_OFFSET_MAX_FLAT_KPH = 10.0
SPEED_OFFSET_MAX_DOWNHILL_KPH = 5.0

# --- 2. 坡度邏輯設定 (單位：弧度 Radians) ---
PITCH_UPHILL_THRESHOLD = 0.015    # > 1.5% 上坡
PITCH_DOWNHILL_THRESHOLD = -0.030 # < -3.0% 下坡

# --- 3. 動態 TTC 與其他設定 ---
TTC_BP = [10., 30.]
TTC_V  = [2.0, 3.0]

EMERGENCY_TTC = 2.0
EMERGENCY_RELATIVE_SPEED = 10.0
EMERGENCY_DECEL_THRESHOLD = -1.5

LEAD_COOLDOWN_TIME = 0.5
SPEED_BP = [0., 10., 20., 30.]
MIN_DIST_V = [15., 20., 25., 30.]

# ==============================================================================
# [Soft Hold 參數設定] 防止煞車點頭 (Anti-Nodding)
# ==============================================================================
SOFT_HOLD_ACCEL = -0.00       # 強制限制加速度上限
SOFT_HOLD_RANGE_MIN = 0.76    # 觸發下限：76%
SOFT_HOLD_RANGE_MAX = 1.00    # 觸發上限：100%

# ==============================================================================
# [Soft Stop 參數設定] 防止煞停頓挫 (Soft Stop)
# 原理更新：引入「線性遞減」邏輯，越接近停止，煞車力道限制越嚴格。
# ==============================================================================
SOFT_STOP_SPEED_MAX = 5.0     # 啟用速度：5 m/s (18 km/h)
SOFT_STOP_SPEED_MIN = 0.5     # [修改] 極低速：0.5 m/s (1.8 km/h)

# [修改] 煞車力道動態區間
# 速度 > 18km/h 時，允許最大煞車力道為 -1.35 (正常減速)
# 速度 < 1.8km/h 時，限制最大煞車力道為 -0.50 (極致柔順，消除點頭)
SOFT_STOP_MAX_DECEL = -1.35   
SOFT_STOP_MIN_DECEL = -0.50   

SOFT_STOP_RANGE_CRITICAL = 0.50 # 緊急界線：距離剩不到 50% 則取消限制 (安全優先)

class ACM:
  def __init__(self):
    self.enabled = False
    self._is_in_coast_window = False
    self._has_lead = False
    self._active_prev = False
    self._last_lead_time = 0.0

    self.active = False
    self.just_disabled = False
    
    self.current_ttc_threshold = 3.0
    self.current_pitch = 0.0
    self.current_max_offset = 0.0 

    # [移植注意] 需紀錄駕駛風格
    self.personality = log.LongitudinalPersonality.standard

  def _check_emergency_conditions(self, lead, v_ego, current_time):
    if not lead or not lead.status:
      return False

    self.lead_ttc = lead.dRel / max(v_ego, 0.1)
    relative_speed = v_ego - lead.vLead
    min_dist_for_speed = np.interp(v_ego, SPEED_BP, MIN_DIST_V)

    if lead.dRel < min_dist_for_speed and (
        self.lead_ttc < EMERGENCY_TTC or
        relative_speed > EMERGENCY_RELATIVE_SPEED):

      self._last_lead_time = current_time
      if self.active:
        cloudlog.warning(f"ACM emergency disable: dRel={lead.dRel:.1f}m, TTC={self.lead_ttc:.1f}s")
      return True

    return False

  def _update_lead_status(self, lead, v_ego, current_time):
    if lead and lead.status:
      self.lead_ttc = lead.dRel / max(v_ego, 0.1)
      self.current_ttc_threshold = np.interp(v_ego, TTC_BP, TTC_V)

      if self.lead_ttc < self.current_ttc_threshold:
        self._has_lead = True
        self._last_lead_time = current_time
      else:
        self._has_lead = False
    else:
      self._has_lead = False
      self.lead_ttc = float('inf')

  def _check_cooldown(self, current_time):
    time_since_lead = current_time - self._last_lead_time
    return time_since_lead < LEAD_COOLDOWN_TIME

  def _should_activate(self, user_ctrl_lon, v_ego, v_cruise, in_cooldown, pitch):
    if pitch > PITCH_UPHILL_THRESHOLD:
        self._is_in_coast_window = False
        return False

    if pitch < PITCH_DOWNHILL_THRESHOLD:
        self.current_max_offset = SPEED_OFFSET_MAX_DOWNHILL_KPH 
    else:
        self.current_max_offset = SPEED_OFFSET_MAX_FLAT_KPH     

    lower_bound = v_cruise - (SPEED_OFFSET_MIN_KPH / 3.6)
    upper_bound = v_cruise + (self.current_max_offset / 3.6)
    
    self._is_in_coast_window = lower_bound < v_ego < upper_bound

    return (not user_ctrl_lon and
            not self._has_lead and
            not in_cooldown and
            self._is_in_coast_window)

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, personality=log.LongitudinalPersonality.standard):
    self.personality = personality 
    
    if not self.enabled or len(cc.orientationNED) != 3:
      self.active = False
      return

    self.current_pitch = cc.orientationNED[1]
    current_time = time.monotonic()
    lead = rs.leadOne

    if self._check_emergency_conditions(lead, v_ego, current_time):
      self.active = False
      self._active_prev = self.active
      return

    self._update_lead_status(lead, v_ego, current_time)
    in_cooldown = self._check_cooldown(current_time)
    
    self.active = self._should_activate(user_ctrl_lon, v_ego, v_cruise, in_cooldown, self.current_pitch)

    self.just_disabled = self._active_prev and not self.active
    if self.active and not self._active_prev:
      pitch_deg = self.current_pitch * 57.2958
      cloudlog.info(f"ACM ON: v={v_ego*3.6:.0f}, pitch={pitch_deg:.1f}deg, Max+{self.current_max_offset:.0f}kph")
    elif self.just_disabled:
      cloudlog.info("ACM OFF")

    self._active_prev = self.active

  # ============================================================================
  # [Soft Hold + Soft Stop 核心邏輯]
  # ============================================================================
  def _apply_soft_hold(self, a_desired_trajectory, v_ego, lead):
    if not lead.status:
      return a_desired_trajectory

    if self.current_pitch > PITCH_UPHILL_THRESHOLD or self.current_pitch < PITCH_DOWNHILL_THRESHOLD:
      return a_desired_trajectory

    # [保留] 防止前車靜止後蠕動
    if lead.vRel > 0.1 and lead.vLead > 0.2:
        return a_desired_trajectory

    t_follow = get_T_FOLLOW(self.personality)
    desired_dist = get_safe_obstacle_distance(v_ego, t_follow)
    lead_obstacle_dist = lead.dRel + get_stopped_equivalence_factor(lead.vLead)

    if desired_dist < 0.1:
      ratio = 10.0
    else:
      ratio = lead_obstacle_dist / desired_dist

    # --- 邏輯 A: Soft Hold (遠距滑行) ---
    if SOFT_HOLD_RANGE_MIN < ratio < SOFT_HOLD_RANGE_MAX:
      a_desired_trajectory = np.minimum(a_desired_trajectory, SOFT_HOLD_ACCEL)

    # --- 邏輯 B: Soft Stop (近距線性柔順煞停) ---
    # [修改] 這裡進行了核心優化
    if (v_ego < SOFT_STOP_SPEED_MAX) and (ratio > SOFT_STOP_RANGE_CRITICAL):
        
        # 1. 計算動態煞車限制 (Dynamic Brake Limit)
        #    利用 np.interp 根據當前速度線性調整最大允許的煞車力道
        #    v_ego 接近 5.0m/s -> 限制為 -1.35 (允許較大力度)
        #    v_ego 接近 0.5m/s -> 限制為 -0.50 (只允許輕踩)
        current_brake_limit = np.interp(
            v_ego, 
            [SOFT_STOP_SPEED_MIN, SOFT_STOP_SPEED_MAX], 
            [SOFT_STOP_MIN_DECEL, SOFT_STOP_MAX_DECEL]
        )
        
        # 2. 應用限制
        #    如果 MPC 請求更強的煞車 (例如 -2.0)，會被限制在 current_brake_limit
        a_desired_trajectory = np.maximum(a_desired_trajectory, current_brake_limit)

    return a_desired_trajectory

  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0, lead=None):
    
    traj = a_desired_trajectory

    if self.active:
      min_accel = np.min(traj)
      if min_accel < EMERGENCY_DECEL_THRESHOLD:
        cloudlog.warning(f"ACM aborting: MPC requested {min_accel:.2f} m/s² braking")
        self.active = False
      else:
        modified_trajectory = np.copy(traj)
        for i in range(len(modified_trajectory)):
          if -1.0 < modified_trajectory[i] < 0:
            modified_trajectory[i] = 0.0
        traj = modified_trajectory
    
    if lead is not None:
        traj = self._apply_soft_hold(traj, v_ego, lead)
    
    return traj
