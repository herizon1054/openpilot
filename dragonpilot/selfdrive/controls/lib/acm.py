import time
import numpy as np
from openpilot.common.swaglog import cloudlog

# =========================================================
# ACM (Active Coasting Management) 參數設定區
# =========================================================

# --- 1. 滑行速度區間設定 (單位：km/h) ---

# [下限] 固定為「定速 - 2 km/h」 (低於此值補油)
SPEED_OFFSET_MIN_KPH = 2.0 

# [上限 - 雙模式設定]
# 模式 A: 一般路況 (平路或緩下坡)，允許滑行至「定速 + 10 km/h」
SPEED_OFFSET_MAX_FLAT_KPH = 10.0

# 模式 B: 陡下坡時 (超過 3% 坡度)，為了安全，限制只允許滑行至「定速 + 5 km/h」
# 說明：超過 5km/h 就會立刻關閉 ACM，讓 Openpilot 介入煞車
SPEED_OFFSET_MAX_DOWNHILL_KPH = 5.0

# --- 2. 坡度邏輯設定 (單位：弧度 Radians) ---
# 0.015 rad 約等於 0.86 度 (1.5% 坡度)
# 0.030 rad 約等於 1.72 度 (3.0% 坡度)

# 上坡門檻：大於 1.5% (0.015) -> 禁止滑行，確保爬坡有力
PITCH_UPHILL_THRESHOLD = 0.015    

# 下坡門檻：小於 -3.0% (-0.03) -> 切換為嚴格模式 (+5km/h)
# 意思：在 0% ~ 3% 的緩下坡，我們依然允許滑到 +10km/h (模式 A)
PITCH_DOWNHILL_THRESHOLD = -0.030 

# --- 3. 動態 TTC (碰撞時間) 安全設定 ---
# 速度 [36kph, 108kph] -> TTC [2.0s, 3.0s] 平滑過渡
TTC_BP = [10., 30.]
TTC_V  = [1.5, 2.0]

# --- 4. 緊急狀況閾值 ---
EMERGENCY_TTC = 2.0
EMERGENCY_RELATIVE_SPEED = 10.0
EMERGENCY_DECEL_THRESHOLD = -1.5

# --- 5. 其他安全設定 ---
LEAD_COOLDOWN_TIME = 0.5
SPEED_BP = [0., 10., 20., 30.]
MIN_DIST_V = [15., 20., 25., 30.]


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
    # 1. 上坡判斷：大於 1.5% 禁止滑行 (保留原設定，確保爬坡不掉速)
    if pitch > PITCH_UPHILL_THRESHOLD:
        self._is_in_coast_window = False
        return False

    # 2. 決定「速度上限」是寬鬆 (+10) 還是嚴格 (+5)
    # 修改點：只有坡度比 -3% 更陡 (例如 -4%, -5%) 才會觸發嚴格模式
    if pitch < PITCH_DOWNHILL_THRESHOLD:
        self.current_max_offset = SPEED_OFFSET_MAX_DOWNHILL_KPH # +5 km/h
    else:
        self.current_max_offset = SPEED_OFFSET_MAX_FLAT_KPH     # +10 km/h (平路或緩下坡)

    # 3. 計算速度區間
    lower_bound = v_cruise - (SPEED_OFFSET_MIN_KPH / 3.6)
    upper_bound = v_cruise + (self.current_max_offset / 3.6)
    
    self._is_in_coast_window = lower_bound < v_ego < upper_bound

    return (not user_ctrl_lon and
            not self._has_lead and
            not in_cooldown and
            self._is_in_coast_window)

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise):
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
      # Log 顯示當下的上限設定
      cloudlog.info(f"ACM ON: v={v_ego*3.6:.0f}, pitch={pitch_deg:.1f}deg, Max+{self.current_max_offset:.0f}kph")
    elif self.just_disabled:
      cloudlog.info("ACM OFF")

    self._active_prev = self.active

  def update_a_desired_trajectory(self, a_desired_trajectory):
    if not self.active:
      return a_desired_trajectory

    min_accel = np.min(a_desired_trajectory)
    if min_accel < EMERGENCY_DECEL_THRESHOLD:
      cloudlog.warning(f"ACM aborting: MPC requested {min_accel:.2f} m/s² braking")
      self.active = False
      return a_desired_trajectory

    modified_trajectory = np.copy(a_desired_trajectory)
    for i in range(len(modified_trajectory)):
      if -1.0 < modified_trajectory[i] < 0:
        modified_trajectory[i] = 0.0
    
    return modified_trajectory
