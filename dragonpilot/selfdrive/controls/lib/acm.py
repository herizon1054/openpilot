import time
import numpy as np
from cereal import log
from openpilot.common.swaglog import cloudlog

# 匯入 Openpilot 原廠 MPC (模型預測控制) 相關的安全距離與參數計算公式
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  COMFORT_BRAKE, STOP_DISTANCE, get_safe_obstacle_distance,
  get_stopped_equivalence_factor, get_T_FOLLOW
)

# =========================================================
# ACM (Active Coasting Management) 主動滑行管理系統 - 參數設定區
# =========================================================

# --- 滑行速度容許範圍 ---
SPEED_OFFSET_MIN_KPH = 1.0             
SPEED_OFFSET_MAX_FLAT_KPH = 15.0       
SPEED_OFFSET_MAX_DOWNHILL_KPH = 5.0    

# --- 坡度判斷門檻 ---
PITCH_UPHILL_THRESHOLD = 0.015         
PITCH_DOWNHILL_THRESHOLD = -0.030      

# --- TTC (Time To Collision) 碰撞時間設定 ---
TTC_BP = [10., 30.]                    
TTC_V  = [3.0, 3.0]                    

# --- 緊急狀況解除滑行設定 ---
EMERGENCY_TTC = 2.0                    
EMERGENCY_RELATIVE_SPEED = 10.0        
EMERGENCY_DECEL_THRESHOLD = -1.5       

# --- 系統冷卻與安全距離設定 ---
LEAD_COOLDOWN_TIME = 0.5               
SPEED_BP = [0., 10., 20., 30.]         
MIN_DIST_V = [5., 10., 15., 20.]       

# --- Soft Hold (柔和跟車/滑行介入) 設定 ---
SOFT_HOLD_RANGE_MIN = 0.76             
SOFT_HOLD_RANGE_MAX = 0.99             
SOFT_HOLD_TTC_THRESHOLD = 2.0          # [新增] TTC 大於此數值時，暫停 Soft Hold 運作

# 車速 (km/h) 對應 最高加速度限制 (m/s²) 的插值陣列
SOFT_HOLD_SPEED_BP = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
SOFT_HOLD_ACCEL_V  = [ 0.7,  0.6,  0.5,  0.4,  0.3,  0.2,  0.1]


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

    self.personality = log.LongitudinalPersonality.standard 
    self._dtsc_is_active = False          

    self._soft_hold_factor = 1.0

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

  def _should_activate(self, user_ctrl_lon, v_ego, v_cruise, in_cooldown, pitch, dtsc_is_active):
    if dtsc_is_active:
        self._is_in_coast_window = False
        return False
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

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, personality=log.LongitudinalPersonality.standard, dtsc_is_active=False):
    self.personality = personality
    self._dtsc_is_active = dtsc_is_active 

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
    self.active = self._should_activate(user_ctrl_lon, v_ego, v_cruise, in_cooldown, self.current_pitch, dtsc_is_active)

    self.just_disabled = self._active_prev and not self.active
    if self.active and not self._active_prev:
      pitch_deg = self.current_pitch * 57.2958
      cloudlog.info(f"ACM ON: v={v_ego*3.6:.0f}, pitch={pitch_deg:.1f}deg, Max+{self.current_max_offset:.0f}kph")
    elif self.just_disabled:
      cloudlog.info("ACM OFF")

    self._active_prev = self.active

  def _apply_soft_hold(self, a_desired_trajectory, v_ego, lead, t_follow):
    target_factor = 1.0   
    ratio = 10.0          
    
    # 計算當前車速並透過插值取得對應的 Soft Hold 允許最高加速度
    v_ego_kph = v_ego * 3.6
    current_soft_hold_accel = np.interp(v_ego_kph, SOFT_HOLD_SPEED_BP, SOFT_HOLD_ACCEL_V)

    if lead is not None and lead.status:
        is_lead_braking = lead.aLeadK < -0.1
        
        # [新增] 獨立計算當下給 Soft Hold 邏輯使用的 TTC
        current_ttc = lead.dRel / max(v_ego, 0.1)
        
        # 車速 10 以下且前車正在煞車時，保持完全不加速的保守設定
        if v_ego_kph <= 10.0 and is_lead_braking:
            current_soft_hold_accel = -0.00

        if self.current_pitch <= PITCH_UPHILL_THRESHOLD:
            desired_dist = get_safe_obstacle_distance(v_ego, t_follow)
            lead_obstacle_dist = lead.dRel + get_stopped_equivalence_factor(lead.vLead)

            if desired_dist < 0.1:
                ratio = 10.0
            else:
                ratio = lead_obstacle_dist / desired_dist

            distance_factor = 1.0 

            # [修改] 若距離進入比例範圍內，且 TTC 小於等於門檻值 (2.0) 才允許觸發
            if SOFT_HOLD_RANGE_MIN < ratio < SOFT_HOLD_RANGE_MAX and current_ttc <= SOFT_HOLD_TTC_THRESHOLD:
                distance_factor = 0.0

            v_rel_factor = np.interp(lead.vRel, [-2.5, -0.5], [0.0, 1.0])
            target_factor = max(distance_factor, v_rel_factor)

    if target_factor > self._soft_hold_factor:
        alpha = 0.10
    else:
        alpha = 0.20

    self._soft_hold_factor = (1.0 - alpha) * self._soft_hold_factor + alpha * target_factor

    if self._soft_hold_factor < 0.99:
        dynamic_limit = np.maximum(a_desired_trajectory, 0.0) * self._soft_hold_factor + current_soft_hold_accel * (1.0 - self._soft_hold_factor)
        a_desired_trajectory = np.minimum(a_desired_trajectory, dynamic_limit)

    return a_desired_trajectory

  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0, lead=None, t_follow=None):
    if getattr(self, '_dtsc_is_active', False):
        return a_desired_trajectory

    traj = a_desired_trajectory

    if self.active:
      min_accel = np.min(traj)
      if min_accel < EMERGENCY_DECEL_THRESHOLD:
        cloudlog.warning(f"ACM aborting: MPC requested {min_accel:.2f} m/s² braking")
        self.active = False
      else:
        if not (lead is not None and lead.status):
          modified_trajectory = np.copy(traj)
          for i in range(len(modified_trajectory)):
            if -0.3 < modified_trajectory[i] < 0:
              modified_trajectory[i] = 0.0
          traj = modified_trajectory

    if t_follow is None:
        t_follow = get_T_FOLLOW(self.personality)

    traj = self._apply_soft_hold(traj, v_ego, lead, t_follow)
    return traj
