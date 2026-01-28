import time
import numpy as np
from openpilot.common.swaglog import cloudlog

# =========================================================
# ACM (Active Coasting Management) 參數設定區
# =========================================================

# --- 1. 滑行速度區間設定 (單位：km/h) ---
SPEED_OFFSET_MIN_KPH = 2.0 
SPEED_OFFSET_MAX_FLAT_KPH = 10.0
SPEED_OFFSET_MAX_DOWNHILL_KPH = 5.0

# --- 2. 坡度邏輯設定 (單位：弧度 Radians) ---
PITCH_UPHILL_THRESHOLD = 0.015    
PITCH_DOWNHILL_THRESHOLD = -0.030 

# --- 3. 動態 TTC (碰撞時間) 安全設定 ---
TTC_BP = [10., 30.]

# [新功能] 區分有雷達與無雷達的設定
# 有雷達且正常運作: 允許較短的 TTC [1.5s, 2.5s] (較激進)
TTC_RADAR_V = [1.5, 2.5]

# 無雷達 (Vision only) 或 雷達故障時: 強制使用保守 TTC [3.0s, 3.0s] (安全措施)
TTC_VISION_V = [3.0, 3.0]

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
    
    # 用於 Log 紀錄當前是否處於 Vision 安全模式
    self.using_vision_ttc = False

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

  def _update_lead_status(self, lead, v_ego, current_time, ttc_values):
    if lead and lead.status:
      self.lead_ttc = lead.dRel / max(v_ego, 0.1)
      
      # [核心修改] 使用傳入的 ttc_values (可能是 Radar 或 Vision 版) 進行計算
      self.current_ttc_threshold = np.interp(v_ego, TTC_BP, ttc_values)

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

  # [API 變更] 新增 CP (CarParams) 和 radar_errors 參數
  # 用於判斷車輛配置與即時雷達健康狀況
  def update_states(self, CP, cc, rs, radar_errors, user_ctrl_lon, v_ego, v_cruise):
    if not self.enabled or len(cc.orientationNED) != 3:
      self.active = False
      return

    self.current_pitch = cc.orientationNED[1]
    current_time = time.monotonic()
    lead = rs.leadOne

    # --- 1. 雷達狀態檢查與 TTC 模式選擇 ---
    is_radar_faulted = False
    
    # 如果有傳入雷達錯誤資訊 (來自 RadarData.errors)
    if radar_errors is not None:
        # canError: 通訊丟失, radarFault: 硬體故障
        if radar_errors.canError or radar_errors.radarFault:
            is_radar_faulted = True
        
        # (可選) 若要將暫時遮蔽也視為故障，可取消下一行註解
        # if radar_errors.radarUnavailableTemporary: is_radar_faulted = True

    # 決策邏輯：
    # A. 車輛配置本來就沒雷達 (CP.radarUnavailable)
    # B. 雷達壞了 (is_radar_faulted)
    # 滿足任一條件 -> 使用保守 Vision TTC
    if CP.radarUnavailable or is_radar_faulted:
        current_ttc_values = TTC_VISION_V
        self.using_vision_ttc = True
        
        # 如果 ACM 正在作動中突然偵測到故障，印出警告
        if is_radar_faulted and self.active and not self._active_prev:
             cloudlog.warning("ACM: Radar fault detected! Safety fallback to Vision TTC.")
    else:
        current_ttc_values = TTC_RADAR_V
        self.using_vision_ttc = False

    # --- 2. 檢查緊急狀況 ---
    if self._check_emergency_conditions(lead, v_ego, current_time):
      self.active = False
      self._active_prev = self.active
      return

    # --- 3. 更新前車狀態 (傳入決定的 TTC 表) ---
    self._update_lead_status(lead, v_ego, current_time, current_ttc_values)
    
    in_cooldown = self._check_cooldown(current_time)
    
    self.active = self._should_activate(user_ctrl_lon, v_ego, v_cruise, in_cooldown, self.current_pitch)

    self.just_disabled = self._active_prev and not self.active
    if self.active and not self._active_prev:
      pitch_deg = self.current_pitch * 57.2958
      # Log 顯示當前模式 (RadarTTC 或 VisionTTC)
      mode_str = "VisionTTC" if self.using_vision_ttc else "RadarTTC"
      cloudlog.info(f"ACM ON ({mode_str}): v={v_ego*3.6:.0f}, pitch={pitch_deg:.1f}deg, Max+{self.current_max_offset:.0f}kph")
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
