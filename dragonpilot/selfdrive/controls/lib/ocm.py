"""
Copyright (c) 2025, Rick Lan
Modified OCM (Overdrive Coasting Mode) integrated with DTSC strict abort logic.

dp0111pre v2（重寫）：
  用途：定速 100，踩油門超車到 120 後放開，前方無車時「不給油、不煞車」自然滑行回 100，
  回到設定速度附近後交還 MPC 定速。

  與 v1 的差異（v1 為什麼無效 / 有害，見審查報告 H1）：
    1. v1 的 self.enabled 永遠是 False，update_states() 每幀直接 return，功能從未作動。
       v2 由 planner 依 dp_flags 每幀設定 enabled。
    2. v1 把 a_desired_trajectory 裡 -1.0 < a < 0 改成 0，但 v_desired_trajectory 沒改，
       get_accel_from_plan() 的 a_target = 2·Δv/t − a_now 會因 a_now 變 0 而把煞車加倍
       （−0.5 → −1.0）。而且 a_target = 0 代表「維持車速」（會給油），不是滑行。
       v2 不動任何軌跡，改由 planner 把最終輸出鎖在「自然滑行加速度」：
         - 上限：accel_clip[1] <= 滑行加速度（不給油，等同原廠 allow_throttle=False 的機制）
         - 下限：輸出 = 滑行加速度（不因為「超過設定速度」而煞車）
       Toyota 的 PCM 依加速度指令自行決定油門／煞車；指令 = 自然滑行減速度時，理論上兩者都不作動。
    3. 只在「MPC 的控制來源是 cruise（定速）」時作用。前車、紅綠燈、e2e（blended 中模型想煞）
       一旦成為控制來源，當幀立即解除，交還正常控制，不做漸變（安全路徑不延遲）。
"""

import time
import numpy as np
from cereal import log
from openpilot.common.swaglog import cloudlog

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource

# =========================================================
# 滑行速度與安全參數設定區
# =========================================================
OVERTAKE_THRESHOLD = 2.0 / 3.6   # 2 km/h - 比定速快 2 公里以上才允許進入滑行
HYSTERESIS_OFFSET = 1.0 / 3.6    # 1 km/h - 滑行到「設定速度 + 1 km/h」以下才解除，交還定速
TTC_THRESHOLD = 3.0              # 秒 - 前車 TTC 小於此值即停用

# 車速開關門檻 (50 km/h 打開，40 km/h 關閉)
MIN_SPEED_ENABLE = 50.0 / 3.6
MIN_SPEED_DISABLE = 40.0 / 3.6

# 超速上限 (加入遲滯區間防震盪)
# v2：原本 20/25 km/h，「定速 100 踩到 120」剛好卡在邊界（> 120 就不允許），改為 30/35 km/h。
# 超過這個幅度代表落差太大，純滑行要很久才回到設定速度，交還 MPC 以定速減速度處理。
OVERSPEED_ACTIVE_MARGIN = 35.0 / 3.6   # 正在滑行時最高容忍超速 35 km/h
OVERSPEED_INACTIVE_MARGIN = 30.0 / 3.6 # 未滑行時超速 30 km/h 以內才允許啟動

# 緊急安全防線
EMERGENCY_TTC = 2.0
EMERGENCY_RELATIVE_SPEED = 10.0
EMERGENCY_DECEL_THRESHOLD = -1.5 # MPC/e2e 要求的減速度強於此值，立刻交還控制權

# 動態安全距離參數
SPEED_BP = [0., 10., 20., 30.]
MIN_DIST_V = [5., 10., 15., 20.]

# 坡度參數
PITCH_SMOOTH_ALPHA_UP = 0.30
PITCH_SMOOTH_ALPHA_DOWN = 0.15
PITCH_DOWNHILL_THRESHOLD = -0.030      # 判定為下坡的閾值 (-3% 坡度)：下坡滑行會越滑越快，不啟動

# 滑行加速度微調 (m/s²)：疊加在 planner 的 get_coast_accel(pitch, v_ego) 上。
# get_coast_accel 已改為依車速查表的自然滑行減速度（見 longitudinal_planner.py 的
# DP_COAST_BASE，依 Corolla TSS2 rlog 校正）。指令比自然滑行更負時 Toyota PCM 會動用煞車
# （前輪附近有聲音、PCM_CRUISE.ACC_BRAKING=1）。
# 若 OCM 中仍聽到煞車聲 / ACC_BRAKING=1 → 調正（例如 +0.05，減速度變小）；
# 若感覺仍有補油、減速太慢 → 調負（例如 −0.05）。0.0 = 不修正。
COAST_ACCEL_OFFSET = 0.0


class OCM:
  def __init__(self):
    self.enabled = False      # 由 planner 依 dp_flags 每幀設定

    self.active = False
    self.just_disabled = False
    self.speed_allowed = False

    self._is_speed_over_cruise = False
    self._has_lead = False
    self._active_prev = False
    self._last_lead_time = 0.0
    self.lead_ttc = float('inf')

    # 坡度記憶
    self.current_pitch = 0.0
    self._is_first_pitch = True

  def _update_pitch(self, orientation_ned):
    """更新並平滑化道路坡度資訊"""
    if len(orientation_ned) == 3:
      new_pitch = orientation_ned[1]
      if self._is_first_pitch:
        self.current_pitch = new_pitch
        self._is_first_pitch = False
      else:
        alpha = PITCH_SMOOTH_ALPHA_UP if new_pitch > self.current_pitch else PITCH_SMOOTH_ALPHA_DOWN
        self.current_pitch = alpha * new_pitch + (1.0 - alpha) * self.current_pitch

  def _check_emergency_conditions(self, lead, v_ego, current_time):
    """緊急狀況判斷 (結合動態安全距離)"""
    if not lead or not lead.status:
      return False

    closing_speed = max(v_ego - lead.vLead, 0.1)
    self.lead_ttc = lead.dRel / closing_speed
    relative_speed = v_ego - lead.vLead

    min_dist_for_speed = np.interp(v_ego, SPEED_BP, MIN_DIST_V)

    if (self.lead_ttc < EMERGENCY_TTC) or \
       (relative_speed > EMERGENCY_RELATIVE_SPEED) or \
       (lead.dRel < min_dist_for_speed and relative_speed > 0):
      self._last_lead_time = current_time
      if self.active:
        cloudlog.warning(f"OCM emergency disable: dRel={lead.dRel:.1f}m, TTC={self.lead_ttc:.1f}s")
      return True

    return False

  def _update_lead_status(self, lead, v_ego, current_time):
    if lead and lead.status:
      closing_speed = max(v_ego - lead.vLead, 0.1)
      self.lead_ttc = lead.dRel / closing_speed
      if self.lead_ttc < TTC_THRESHOLD:
        self._has_lead = True
        self._last_lead_time = current_time
      else:
        self._has_lead = False
    else:
      self._has_lead = False

  def _should_activate(self, user_ctrl_lon, v_ego, v_cruise, in_cooldown):
    # 當前車速低於巡航車速時，絕對不允許啟動滑行
    if v_ego < v_cruise:
      return False

    # 坡度安全防護：明顯下坡 (-3% 以上) 不允許純滑行
    if self.current_pitch < PITCH_DOWNHILL_THRESHOLD:
      return False

    # 超速上限防護 (加入遲滯區間防震盪)
    overspeed_margin = OVERSPEED_ACTIVE_MARGIN if self.active else OVERSPEED_INACTIVE_MARGIN
    if v_ego > (v_cruise + overspeed_margin):
      return False

    if self.active:
      self._is_speed_over_cruise = v_ego > (v_cruise + HYSTERESIS_OFFSET)
    else:
      self._is_speed_over_cruise = v_ego >= (v_cruise + OVERTAKE_THRESHOLD)

    return (not user_ctrl_lon and not self._has_lead and
            not in_cooldown and self._is_speed_over_cruise)

  def _set_inactive(self, reason: str | None = None):
    self.active = False
    self.just_disabled = self._active_prev and not self.active
    if self.just_disabled and reason is not None:
      cloudlog.info(f"OCM deactivated: {reason}")
    self._active_prev = self.active

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, dtsc_active=False,
                    plan_source=LongitudinalPlanSource.cruise, a_target=0.0, a_plan_min=0.0):
    """
    plan_source：本幀 MPC/e2e 最終的控制來源（planner 的 self.mpc.source，已含 e2e 覆寫）。
                 不是 cruise 代表前車／紅綠燈／模型在主導，OCM 不介入。
    a_target   ：本幀未經 OCM 修改的輸出加速度（acc = MPC，blended = min(MPC, e2e)）。
    a_plan_min ：本幀 MPC 加速度軌跡 a_desired_trajectory 的最小值（v1 的安全檢查，保留）：
                 MPC 預判未來需要急煞時不攔截，即使當下的控制來源仍是 cruise。
    a_target 或 a_plan_min 任一個強於 EMERGENCY_DECEL_THRESHOLD 即不介入。
    """
    # 車速總開關狀態更新 (40關閉 50打開)
    if v_ego >= MIN_SPEED_ENABLE:
      self.speed_allowed = True
    elif v_ego <= MIN_SPEED_DISABLE:
      self.speed_allowed = False

    # 絕對硬防線：當前車速低於巡航車速時，強制關閉
    if v_ego < v_cruise:
      self._set_inactive("below cruise speed")
      return

    # DTSC 正在彎道減速：交給 DTSC
    if dtsc_active:
      self._set_inactive("DTSC is currently active in curve")
      return

    # 開關未啟用、車速未達標、或姿態資料不完整
    if not self.enabled or not self.speed_allowed or len(cc.orientationNED) != 3:
      self._set_inactive()
      return

    self._update_pitch(cc.orientationNED)

    current_time = time.monotonic()
    lead = rs.leadOne

    if self._check_emergency_conditions(lead, v_ego, current_time):
      self._set_inactive("lead emergency")
      return

    # v2：控制來源不是定速（前車、紅綠燈、e2e 模型想煞），或要求的減速度夠強 → 不介入
    if plan_source != LongitudinalPlanSource.cruise:
      self._last_lead_time = current_time   # 同上，0.5 s 冷卻，避免來源在 cruise/lead 間跳動時反覆進出
      self._set_inactive(f"plan source {plan_source}")
      return
    # 安全檢查（沿用 v1）：MPC 判定需要急煞（整段計畫軌跡的最小值）或本幀輸出要求強減速，不予攔截
    min_accel = min(a_target, a_plan_min)
    if min_accel < EMERGENCY_DECEL_THRESHOLD:
      if self.active:
        cloudlog.warning(f"OCM aborting: requested {min_accel:.2f} m/s^2 braking (target={a_target:.2f}, plan_min={a_plan_min:.2f})")
      # 與前車解除共用 0.5 s 冷卻，避免急煞條件一消失下一幀就重新滑行、在煞/滑之間來回跳
      self._last_lead_time = current_time
      self._set_inactive(f"strong decel requested {min_accel:.2f} m/s^2")
      return

    self._update_lead_status(lead, v_ego, current_time)
    in_cooldown = (current_time - self._last_lead_time) < 0.5

    self.active = self._should_activate(user_ctrl_lon, v_ego, v_cruise, in_cooldown)
    self.just_disabled = self._active_prev and not self.active

    if self.active and not self._active_prev:
      cloudlog.info(f"OCM activated: v_ego={v_ego*3.6:.1f} km/h, v_cruise={v_cruise*3.6:.1f} km/h, pitch={self.current_pitch:.3f}")
    elif self.just_disabled:
      cloudlog.info("OCM deactivated (Normal condition met)")

    self._active_prev = self.active

  @staticmethod
  def get_coast_target(accel_coast: float) -> float:
    """滑行時的加速度目標：planner 的 get_coast_accel(pitch, v_ego)（自然滑行查表）+ 實車微調。
    只會 <= 0：上坡時 get_coast_accel 本身更負（自然減速更快），不會變成要求加速。"""
    return min(accel_coast + COAST_ACCEL_OFFSET, 0.0)
