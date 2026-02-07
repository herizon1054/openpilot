import time
import numpy as np
from cereal import log
from openpilot.common.swaglog import cloudlog

# ==============================================================================
# [移植注意] 引入 MPC 函式庫
# 目的：為了計算與 MPC 一致的「安全跟車距離」，必須引用 long_mpc 的物理常數與公式。
# 若路徑不同，請修改此處 import 路徑。
# ==============================================================================
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  COMFORT_BRAKE, STOP_DISTANCE, get_safe_obstacle_distance, 
  get_stopped_equivalence_factor, get_T_FOLLOW
)

# =========================================================
# [參數設定區] ACM & Soft Hold 參數
# =========================================================

# --- 1. ACM 滑行速度區間設定 (單位：km/h) ---
SPEED_OFFSET_MIN_KPH = 2.0 
SPEED_OFFSET_MAX_FLAT_KPH = 10.0
SPEED_OFFSET_MAX_DOWNHILL_KPH = 5.0

# --- 2. 坡度邏輯設定 (單位：弧度 Radians) ---
# 0.015 rad 約為 1.5% 坡度
PITCH_UPHILL_THRESHOLD = 0.015    
PITCH_DOWNHILL_THRESHOLD = -0.030 

# --- 3. Soft Hold (防點頭) 參數 ---
# 作用：在進入急煞區(DANGER_ZONE)之前，提早強迫滑行
SOFT_HOLD_ACCEL = -0.00       # 強制加速度上限 (0.0=滑行)
SOFT_HOLD_RANGE_MIN = 0.76    # 觸發下限：76% 安全距離 (低於此值交給 MPC 急煞)
SOFT_HOLD_RANGE_MAX = 1.00    # 觸發上限：100% 安全距離

# --- 4. Soft Stop (防頓挫) 參數 ---
# 作用：低速煞停時限制最大減速度，避免點頭
SOFT_STOP_SPEED_MAX = 5.0     # 啟用速度：< 5 m/s (18 km/h)
SOFT_STOP_MAX_DECEL = -1.50   # 最大煞車力道限制
SOFT_STOP_RANGE_CRITICAL = 0.50 # 緊急界線：距離剩 50% 時取消限制

# --- 5. 其他常數 ---
TTC_BP = [10., 30.]
TTC_V  = [2.0, 3.0]
EMERGENCY_TTC = 2.0
EMERGENCY_RELATIVE_SPEED = 10.0
EMERGENCY_DECEL_THRESHOLD = -1.5
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

    # [移植注意] 需紀錄駕駛風格 (Personality) 以計算正確的 T_FOLLOW
    self.personality = log.LongitudinalPersonality.standard

  # ============================================================================
  # 邏輯區塊 1: 狀態更新與 ACM 啟用判斷
  # ============================================================================
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

  # [移植注意] update_states 介面變更：必須傳入 personality
  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, personality=log.LongitudinalPersonality.standard):
    self.personality = personality # 儲存當前 OP 的駕駛風格設定
    
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
  # 邏輯區塊 2: Soft Hold + Soft Stop 核心邏輯 (主要修改處)
  # ============================================================================
  def _apply_soft_hold(self, a_desired_trajectory, v_ego, lead):
    """
    對 MPC 輸出的軌跡進行修正，實現舒適跟車與煞停。
    """
    # 1. 前置檢查：無前車則不介入
    if not lead.status:
      return a_desired_trajectory

    # 2. 坡度保護：
    #    上坡 (>1.5%)：需要油門爬坡，禁止限制加速度。
    #    下坡 (<-3.0%)：重力加速，禁止強制滑行。
    if self.current_pitch > PITCH_UPHILL_THRESHOLD or self.current_pitch < PITCH_DOWNHILL_THRESHOLD:
      return a_desired_trajectory

    # 3. [起步/加速保護]
    #    如果前車正在遠離 (vRel > 0) 且確實有速度 (vLead > 0.2)，
    #    代表前車起步或加速中，立即解除限制，避免跟車遲鈍。
    if lead.vRel > 0.1 and lead.vLead > 0.2:
        return a_desired_trajectory

    # 4. [前車靜止保護]
    #    當前車速度極低 (視為靜止，< 1.8 km/h) 時，不啟用 Soft Hold 滑行。
    #    讓 MPC 執行最後的煞停動作，避免因強制滑行導致煞不住。
    if lead.vLead < 0.5: 
        return a_desired_trajectory

    # 5. 計算 100% 理想安全距離
    #    使用與 long_mpc 相同的公式計算，確保邏輯一致
    t_follow = get_T_FOLLOW(self.personality)
    desired_dist = get_safe_obstacle_distance(v_ego, t_follow)
    
    # 計算前車等效障礙物距離
    lead_obstacle_dist = lead.dRel + get_stopped_equivalence_factor(lead.vLead)

    # 6. 計算距離比例 (Ratio = 實際距離 / 理想距離)
    if desired_dist < 0.1:
      ratio = 10.0
    else:
      ratio = lead_obstacle_dist / desired_dist

    # --- 邏輯 A: Soft Hold (防止煞車點頭) ---
    #    區間：76% < 距離 < 100%
    #    動作：限制正向加速度上限為 0 (強制滑行)
    #    目的：在進入 MPC 急煞區前先滑行減速，避免稍後重煞。
    if SOFT_HOLD_RANGE_MIN < ratio < SOFT_HOLD_RANGE_MAX:
      a_desired_trajectory = np.minimum(a_desired_trajectory, SOFT_HOLD_ACCEL)

    # --- 邏輯 B: Soft Stop (防止低速煞停頓挫) ---
    #    條件：低速 (<18kph) 且 距離尚可 (>50%)
    #    動作：限制最大煞車力道 (不能煞太猛)
    if (v_ego < SOFT_STOP_SPEED_MAX) and (ratio > SOFT_STOP_RANGE_CRITICAL):
        # np.maximum 限制負值不要太負 (例如 -3.0 -> -1.35)
        a_desired_trajectory = np.maximum(a_desired_trajectory, SOFT_STOP_MAX_DECEL)

    return a_desired_trajectory

  # ============================================================================
  # 邏輯區塊 3: 軌跡修正入口
  # ============================================================================
  # [移植注意] 此函式需在 long_mpc.py 或 planner 中被呼叫
  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0, lead=None):
    
    traj = a_desired_trajectory

    # --- 階段 1: ACM 原生邏輯 (無前車、遠距離滑行) ---
    if self.active:
      min_accel = np.min(traj)
      # 安全檢查：若 MPC 請求急煞，強制退出 ACM
      if min_accel < EMERGENCY_DECEL_THRESHOLD:
        cloudlog.warning(f"ACM aborting: MPC requested {min_accel:.2f} m/s² braking")
        self.active = False
      else:
        # ACM 運作中：將微減速 (-1.0 ~ 0) 全部抹平為 0 (滑行)
        modified_trajectory = np.copy(traj)
        for i in range(len(modified_trajectory)):
          if -1.0 < modified_trajectory[i] < 0:
            modified_trajectory[i] = 0.0
        traj = modified_trajectory
    
    # --- 階段 2: Soft Hold & Stop 邏輯 (跟車模式) ---
    # 這是最後一道防線，權限高於 ACM。
    # 即使 ACM 說可以滑行，如果進入 Soft Hold 區間，這裡會再次檢查並覆寫。
    if lead is not None:
        traj = self._apply_soft_hold(traj, v_ego, lead)
    
    return traj
