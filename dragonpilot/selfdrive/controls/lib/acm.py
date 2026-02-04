import time
import numpy as np
from cereal import log
from openpilot.common.swaglog import cloudlog
# [新增] 引入 MPC 計算安全距離所需的常數與函式
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

# --- [新增] Soft Hold (預防點頭) 參數 ---
SOFT_HOLD_ACCEL = -0.00       # 限制為 0 加速度 (滑行)，避免正向加速
SOFT_HOLD_RANGE_MIN = 0.76    # 觸發下限 (76%)
SOFT_HOLD_RANGE_MAX = 1.00    # 觸發上限 (100%)

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

    # [新增] 記錄駕駛風格
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

  # [修改] 增加 personality 參數
  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, personality=log.LongitudinalPersonality.standard):
    self.personality = personality # 更新風格
    
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

  # [新增] Soft Hold 核心邏輯
  def _apply_soft_hold(self, a_desired_trajectory, v_ego, lead):
    # 1. 如果沒有前車，直接跳過
    if not lead.status:
      return a_desired_trajectory

    # 2. 坡度檢查：太陡的上坡或下坡都不啟用
    # 上坡 > 1.5% (0.015) -> 不啟用 (需要動力爬坡)
    # 下坡 < -3.0% (-0.030) -> 不啟用 (避免滑行過快，交給 MPC/DTSC)
    if self.current_pitch > PITCH_UPHILL_THRESHOLD or self.current_pitch < PITCH_DOWNHILL_THRESHOLD:
      return a_desired_trajectory

    # 3. 取得當前風格的跟車時間 (T_FOLLOW)
    t_follow = get_T_FOLLOW(self.personality)

    # 4. 計算 100% 理想安全距離 (參考 long_mpc 公式)
    desired_dist = get_safe_obstacle_distance(v_ego, t_follow)

    # 5. 計算前車等效距離 (雷達距離 + 靜止等效因子)
    lead_obstacle_dist = lead.dRel + get_stopped_equivalence_factor(lead.vLead)

    # 6. 計算距離比例 Ratio
    if desired_dist < 0.1:
      ratio = 10.0
    else:
      ratio = lead_obstacle_dist / desired_dist

    # 7. 判斷是否在 76% ~ 100% 緩衝區間
    if SOFT_HOLD_RANGE_MIN < ratio < SOFT_HOLD_RANGE_MAX:
      # 強制將加速度上限壓在 -0.00 (滑行)
      # 若 MPC 原本要加速 (>0)，會被壓回 0
      # 若 MPC 原本要煞車 (<-0.5)，保持原煞車值
      return np.minimum(a_desired_trajectory, SOFT_HOLD_ACCEL)

    return a_desired_trajectory

  # [修改] 增加 v_ego 與 lead 參數
  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0, lead=None):
    
    traj = a_desired_trajectory

    # 1. 執行原本的 ACM 邏輯 (遠距離滑行)
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
    
    # 2. 執行 Soft Hold 邏輯 (近距離緩衝)
    # 這是最後一道防線，權限高於 ACM
    if lead is not None:
        traj = self._apply_soft_hold(traj, v_ego, lead)
    
    return traj
