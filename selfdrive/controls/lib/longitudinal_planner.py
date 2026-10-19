#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import CRUISE_MIN_ACCEL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from dragonpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerDP
from dragonpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanSource as DPPlanSource
from dragonpilot.selfdrive.controls.lib.ocm import OCM
from dragonpilot.selfdrive.controls.lib.aem import AEM
from dragonpilot.selfdrive.controls.lib.apm import APM

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD_ACC = 0.4   # mode=='acc' 使用，維持原廠值，行為不變
# mode=='blended'(e2e) 且 AEM「停用」時的固定門檻。獨立常數，與 aem.py 的
# BASE_THROTTLE_LOW_SPEED_VALUE / BASE_THROTTLE_HIGH_SPEED_VALUE 互不影響——
# AEM 啟用時完全不會用到這個常數。
ALLOW_THROTTLE_THRESHOLD_E2E = 0.2
# mode=='blended' 且 AEM 判定接近模型停止線（aem.near_stop_active）時使用：比 e2e 基礎
# 門檻保守、但仍低於 ACC 的 0.4。實際採用 max(AEM 基礎門檻, 本值)，只會讓門檻變高。
ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP = 0.3
MIN_ALLOW_THROTTLE_SPEED = 2.5

# v10：throttle_prob（modelV2.meta.disengagePredictions.gasPressProbs[1]）單幀雜訊極大——
# 三份市區 40~50 km/h 路測 rlog 實測，同一次連續煞停過程中，這個值每 50 ms 可在
# 0.06~0.58 之間跳動，標準差 0.19~0.36。原廠直接拿原始值比較門檻，allow_throttle 會
# 每幀在 True/False 間橫跳，反映到 accel_clip[1] 在「完全放開」與「夾到滑行曲線」之間
# 跳動，即「油門剎車頓挫感」的直接成因。修法：先低通濾波再加遲滯（僅套用在 blended）。
# 驗證（門檻 0.2、遲滯 0.15 的組合）：三份 log 的 allow_throttle 切換次數從 22~34 次/分鐘
# 降到 0~4 次/分鐘。
THROTTLE_PROB_LPF_ALPHA = 0.05   # 等效時間常數約 1 秒（原 0.2 約 0.22 秒）；越小越不易被
                                  # 單幀雜訊誤判，但追上持續的訊號變化也越慢
ALLOW_THROTTLE_HYSTERESIS = 0.15  # allow_throttle 為 True 時，解除門檻 = 進入門檻 - 遲滯量
# v11：遲滯量上限為進入門檻的這個比例，保證解除門檻 >= 0 且不會讓 allow_throttle 被永久鎖死。
# v10 直接用固定 0.15，當 AEM 給的門檻是 0.1 或 0.0 時，解除門檻變成 -0.05 / -0.15，
# 而 throttle_prob_filtered 不可能 < 0，一旦變成 True 就永遠不會再變回 False。
# 0.75 的選擇是為了讓已用 rlog 驗證過的「門檻 0.2 / 遲滯 0.15 / 解除 0.05」組合完全不變：
#   門檻 0.3（接近停止線） -> 遲滯 0.15  -> 解除 0.15
#   門檻 0.2（AEM 停用）   -> 遲滯 0.15  -> 解除 0.05（與 v10 相同）
#   門檻 0.1（AEM 低速）   -> 遲滯 0.075 -> 解除 0.025
#   門檻 0.0（AEM 高速）   -> 遲滯 0     -> 解除 0.0（= 刻意設定的「不 coast」，見 aem.py）
ALLOW_THROTTLE_HYSTERESIS_MAX_RATIO = 0.75

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# dp：e2e 正向加速度放大係數（見 update() 內說明）
E2E_ACCEL_BOOST_FACTOR = 1.5

# dp(v12.1): e2e 減速保險（e2e decel guard）
# 用途：acc 模式下，模型（e2e）已經感知到應該減速，但 MPC 仍要加速／維持時，把輸出往 e2e 拉，
# 當作一層保險。e2e 為負代表模型整體判斷「該減速了」，不依賴雷達前車選擇，因此能補到
# 雷達／前車選擇還沒反應過來的時段。不切換 mode：accel_clip、allow_throttle、MPC 約束、
# 彎道加速度限制、shouldStop 全部維持 acc 的設定，只修改最終輸出的加速度目標。
# 只在 AEM 開啟（dp_flags & DPFlags.AEM）且 mode == 'acc' 時運作；AEM 關閉時行為與 v11 完全相同。
# 進入（三項同時成立，連續 E2E_GUARD_CONFIRM_FRAMES 幀）：
#   (1) e2e < E2E_GUARD_ENTER_ACCEL
#   (2) e2e − mpc < E2E_GUARD_ENTER_DIFF
#       只看差值時，「MPC 想加速、e2e 只是比較不想加速（兩者皆正）」也會觸發，等於讓 e2e 壓住
#       加速（log 032 第 18 秒、033 第 12 秒，mpc +0.41/+0.45、e2e +0.11/+0.15）。
#   (3) 模型油門機率 gasPressProbs[1]（原始值）< E2E_GUARD_ENTER_GAS_PROB
#       v12.1 新增。跟車時 e2e 常長時間維持 -0.1~-0.3（模型偏好較大車距），只靠 (1)(2) 會在
#       正常跟車加速時觸發：log 036 第 51~57 秒、037 第 46~50 秒，前車加速、MPC +0.3~+0.6，
#       e2e 一直在 -0.12~-0.30，保險壓住加速各約 7 秒。這兩段油門機率 0.93~0.99；
#       032 藍車切入時油門機率掉到 0.59~0.66（煞車機率升到 0.2~0.3），可以區分。
# 退出（維持滿 E2E_GUARD_MIN_HOLD_S 後，任一成立）：e2e >= 0，或油門機率 >= E2E_GUARD_EXIT_GAS_PROB。
#   另有 E2E_GUARD_MAX_HOLD_S 硬上限：保險是短時保護，不該長時間接管。
# 輸出：mpc + w × (min(mpc, e2e) − mpc)，權重 w 在 E2E_GUARD_RAMP_S 秒內 0↔1 漸變，
#   避免進出時輸出一幀跳 0.3~1 m/s²。只會讓輸出變小（更保守），不會讓輸出超過 MPC。
# e2e 取模型原始值（不含 E2E_ACCEL_BOOST_FACTOR 放大）。
# 驗證（log 030/032/033/034/035/036/037，約 390 秒 acc 行駛，63~108 km/h，mpc 由 longitudinalPlan 反推）：
#   - 032 藍車切入：第 57.5 秒觸發（駕駛踩剎車前 0.3 秒），輸出 0.25 秒內由 +0.05 降到 -0.29。
#   - 036 第 48 秒、037 第 54 秒：前車減速、MPC 本身也在減速時觸發，多減約 2 km/h。
#   - 壓住 MPC 加速（MPC > +0.05 且輸出被拉低）的時間：0 秒（v12 為 13.7 秒）。
# ⚠️ 只有國道 log 驗證過，市區低速情境尚未驗證。
E2E_GUARD_ENABLED = True           # 總開關；實際運作還需要 AEM 開啟（dp_flags & DPFlags.AEM），AEM 關閉時完全不介入
E2E_GUARD_ENTER_ACCEL = -0.2        # m/s²
E2E_GUARD_ENTER_DIFF = -0.25        # m/s²（e2e − mpc）
E2E_GUARD_ENTER_GAS_PROB = 0.8      # v12.1：油門機率低於此值才可進入
E2E_GUARD_EXIT_GAS_PROB = 0.9       # v12.1：油門機率回到此值以上即可退出
E2E_GUARD_CONFIRM_FRAMES = 4        # 0.2 秒
E2E_GUARD_MIN_HOLD_S = 0.5          # 秒
E2E_GUARD_MAX_HOLD_S = 3.0          # v12.1：秒，硬上限
E2E_GUARD_RAMP_S = 0.3              # 秒

class DPFlags:
  OCM = 1
  AEM = 2
  APM = 2 ** 2
  DTSC = 2 ** 3
  pass

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]

class LongitudinalPlanner(LongitudinalPlannerDP):
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    
    LongitudinalPlannerDP.__init__(self, self.CP, self.mpc)
    
    self.mpc.mode = 'acc'
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True
    self.throttle_prob_filtered = 1.0   # v10：throttle_prob 低通濾波狀態，見 THROTTLE_PROB_LPF_ALPHA
    # dp(v12): e2e 減速保險狀態，見 E2E_GUARD_*
    self.e2e_guard_active = False
    self.e2e_guard_cnt = 0
    self.e2e_guard_t = 0.0
    self.e2e_guard_w = 0.0

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.ocm = OCM()
    self.aem = AEM()
    self.apm = APM()

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm, dp_flags = 0):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    LongitudinalPlannerDP.update(self, sm)

    if dp_flags & DPFlags.AEM:
      # ⚠️ self.traffic_stop.stop_dist_m 要到後段 LongitudinalPlannerDP.update_targets()
      # 才會刷新成這一幀的值，這裡讀到的是上一幀（約 50 ms 前）。near-stop 判斷本身有
      # 0.5 秒防彈跳與 10 m 遲滯，一幀落差可忽略；若要完全同步需把 update_targets()
      # 提前，但會牽動它使用 self.v_desired_filter.x / self.a_desired（上一幀平滑值）
      # 作為輸入的既有設計，風險較高，暫不動。
      # ⚠️ AEM 啟用時，get_mode() 預設忽略這裡傳入的 mode（即實驗模式開關），屬刻意分歧，
      # 見 aem.py 檔頭與 AEM_EXPERIMENTAL_OFF_FORCES_ACC。
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo,
                              stop_dist_m=self.traffic_stop.near_stop_dist_m)  # 不受障礙物閘門影響的停止線距離
      mode = self.aem.get_mode(mode)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or not v_cruise_initialized
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if dp_accel_clip := LongitudinalPlannerDP.get_accel_clip(self, v_ego, mode):
      accel_clip = dp_accel_clip
    elif mode == 'acc':
      accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]
    # dp(H3a 修正): 彎道加速度限制改為「acc 模式一律套用」。原本寫在 elif 裡，
    # 開啟動態加速度個性（dp_accel_clip 有值）時整段被跳過，例如 AEM 在低速路口轉彎
    # 切到 acc 時，原廠允許 0 m/s²、dp 卻允許約 1.5 m/s²。順序：先取 dp 上限，再取彎道限制。
    if mode == 'acc':
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])

    # v11：濾波每一幀都更新（不分 mode）。v10 只在 blended 分支更新，從 acc 切回 blended
    # 時會用到停留在 acc 期間凍結的過時值。acc 分支本身仍用原始值判斷，不受影響。
    self.throttle_prob_filtered = (THROTTLE_PROB_LPF_ALPHA * throttle_prob
                                    + (1.0 - THROTTLE_PROB_LPF_ALPHA) * self.throttle_prob_filtered)

    if mode == 'blended':
      # AEM 啟用：基礎門檻由 AEM 依車速決定，接近停止線時取較保守者；
      # AEM 停用：固定 ALLOW_THROTTLE_THRESHOLD_E2E。方向燈不影響節流門檻（亦不影響 AEM）。
      if dp_flags & DPFlags.AEM:
        allow_throttle_threshold = self.aem.base_throttle_threshold
        if self.aem.near_stop_active:
          allow_throttle_threshold = max(allow_throttle_threshold, ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP)
      else:
        allow_throttle_threshold = ALLOW_THROTTLE_THRESHOLD_E2E
      # 濾波 + 遲滯（見常數區說明）。遲滯量受 ALLOW_THROTTLE_HYSTERESIS_MAX_RATIO 限制，
      # 解除門檻永遠 >= 0，避免 allow_throttle 被鎖死在 True。
      hysteresis = min(ALLOW_THROTTLE_HYSTERESIS,
                       max(allow_throttle_threshold, 0.0) * ALLOW_THROTTLE_HYSTERESIS_MAX_RATIO)
      effective_threshold = (allow_throttle_threshold - hysteresis
                              if self.allow_throttle else allow_throttle_threshold)
      self.allow_throttle = self.throttle_prob_filtered > effective_threshold or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    else:
      # acc 模式：原廠寫法，直接拿原始 throttle_prob 比較，不濾波、不遲滯
      allow_throttle_threshold = ALLOW_THROTTLE_THRESHOLD_ACC
      self.allow_throttle = throttle_prob > allow_throttle_threshold or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # dp(OCM v2): 上一幀仍在滑行 -> MPC 的加速度上限壓到滑行加速度（不給油），
    # 與上面 allow_throttle=False 同一個機制，讓 MPC 計畫與最終輸出一致。
    # OCM 狀態在下方輸出段才更新（需要本幀 MPC 的控制來源），這裡用上一幀的狀態，落後 1 幀。
    if (dp_flags & DPFlags.OCM) and self.ocm.active and len(sm['carControl'].orientationNED) == 3:
      accel_clip[1] = min(accel_clip[1], max(OCM.get_coast_target(accel_coast), accel_clip[0]))

    personality = sm['selfdriveState'].personality
    if dp_flags & DPFlags.APM:
      self.apm.update(sm)
      has_lead = sm['radarState'].leadOne.status
      v_lead = sm['radarState'].leadOne.vLead if has_lead else 0.0
      a_lead = sm['radarState'].leadOne.aLeadK if has_lead else 0.0
      d_lead = sm['radarState'].leadOne.dRel if has_lead else 0.0

      personality = self.apm.get_personality(
        v_ego=v_ego,
        has_lead=has_lead,
        v_lead=v_lead,
        a_lead=a_lead,
        d_lead=d_lead,
        personality=personality
      )

    self.mpc.set_weights(prev_accel_constraint, personality=personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    a_min_dtsc_out, a_max_dtsc_out = None, None
    is_dtsc_active = False

    if dp_flags & DPFlags.DTSC:
      a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(sm['modelV2'], v_ego, accel_clip[0], accel_clip[1])
      is_dtsc_active = self.dtsc.active
      
      a_min_dtsc_out = np.maximum(accel_clip[0], a_min_dtsc)
      a_max_dtsc_out = np.minimum(accel_clip[1], a_max_dtsc)
      
      for i in range(len(a_min_dtsc_out)):
        if a_min_dtsc_out[i] > a_max_dtsc_out[i]:
          a_min_dtsc_out[i] = a_max_dtsc_out[i] - 0.05
    else:
      horizon_len = len(T_IDXS_MPC)
      a_min_dtsc_out = np.ones(horizon_len) * accel_clip[0]
      a_max_dtsc_out = np.ones(horizon_len) * accel_clip[1]

    v_cruise_target, a_target_from_dp = LongitudinalPlannerDP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)
    a_cruise_min_override = LongitudinalPlannerDP.get_cruise_min_accel(self, v_ego)
    # dp(H3b 修正): 動態加速度個性的巡航最小加速度只用在「一般定速」的舒適降速。
    # 強制減速（駕駛監控第三級、softDisabling）或 DTSC／紅綠燈主導 v 目標時，
    # 至少給原廠 CRUISE_MIN_ACCEL（−1.2），避免降速被壓到 −0.07~−0.13 m/s²
    # （模擬：100 km/h 降到一半由 11 s 拖到 22 s）。
    if a_cruise_min_override is not None and (force_slow_decel or self.source != DPPlanSource.cruise):
      a_cruise_min_override = min(a_cruise_min_override, CRUISE_MIN_ACCEL)

    if force_slow_decel:
      v_cruise_target = 0.0

    # 傳遞 a_min_arr 與 a_max_arr 給修改後的 MPC
    # dp: 紅綠燈/停止標誌虛擬停止線直接餵給 MPC 當障礙物（而非只靠 v_cruise_target
    # 軟限速），求解出來的煞車曲線比純降速平順。self.traffic_stop.stop_dist_m 由上面
    # LongitudinalPlannerDP.update_targets() 這一幀已經算好；None 代表目前沒有主動
    # 停等中，MPC 端會用 disabled sentinel（1000m），不影響一般行駛。
    self.mpc.update(sm['radarState'], v_cruise_target, personality=personality, a_cruise_min_override=a_cruise_min_override,
                     a_min_arr=a_min_dtsc_out, a_max_arr=a_max_dtsc_out, traffic_stop_obstacle_m=self.traffic_stop.stop_dist_m)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    # dp(OCM v2): 不再修改 a_desired_trajectory（v1 只改 a 不改 v，會讓 get_accel_from_plan 的煞車加倍），
    # 改在下方輸出段直接處理最終加速度目標。

    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    output_a_target_e2e_raw = output_a_target_e2e   # dp(v12): e2e 減速保險使用未放大的原始值

    # dp: 對 e2e 的正向加速度意圖放大（×E2E_ACCEL_BOOST_FACTOR），補償 desired_accel 偏保守
    # 的傾向（它是從模型預測的 plan 軌跡微分出來的「預期值」，訓練資料是一般人類的溫和
    # 示範；調 LONG_SMOOTH_SECONDS / ALLOW_THROTTLE_THRESHOLD 等下游參數不會改變這個值的大小）。
    # ⚠️ 刻意分歧：送進 min() 比較的 e2e 值偏離模型原始預測。放大後仍取 min(mpc, e2e)，
    # mpc 依然兜底，最終輸出不會超過 mpc 認為安全的範圍；效果只在「e2e 原本比 mpc 保守」
    # 的情境讓結果更貼近 mpc。只放大正值，減速／煞車方向不受影響。
    if output_a_target_e2e > 0:
      output_a_target_e2e = min(output_a_target_e2e * E2E_ACCEL_BOOST_FACTOR, ACCEL_MAX)

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
      # dp(v12): e2e 減速保險（只改加速度目標，shouldStop 仍只看 MPC）
      if E2E_GUARD_ENABLED and (dp_flags & DPFlags.AEM) and not reset_state:
        self._update_e2e_guard(output_a_target_mpc, output_a_target_e2e_raw, throttle_prob)
        if self.e2e_guard_w > 0.0:
          guarded = min(output_a_target_mpc, output_a_target_e2e_raw)
          output_a_target = output_a_target_mpc + self.e2e_guard_w * (guarded - output_a_target_mpc)
          if output_a_target < output_a_target_mpc:
            try:
              from cereal import log
              self.mpc.source = log.LongitudinalPlan.LongitudinalPlanSource.e2e
            except ImportError:
              pass
      else:
        self._reset_e2e_guard()
    else:
      self._reset_e2e_guard()   # dp(v12): blended 本來就取 min(mpc, e2e)，保險不需要運作
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      
      if output_a_target < output_a_target_mpc:
        try:
          from cereal import log
          self.mpc.source = log.LongitudinalPlan.LongitudinalPlanSource.e2e
        except ImportError:
          pass

    # dp(OCM v2): 超車後滑行回設定速度。條件與解除見 ocm.py；只在控制來源仍是 cruise 時作用，
    # 前車／紅綠燈／e2e 一旦主導（self.mpc.source 已含上面的 e2e 覆寫）當幀解除。
    # 作用中：輸出 = 自然滑行加速度（不給油、也不因超過設定速度而煞車）。
    self.ocm.enabled = bool(dp_flags & DPFlags.OCM)
    if self.ocm.enabled:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.ocm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise, dtsc_active=is_dtsc_active,
                             plan_source=self.mpc.source, a_target=float(output_a_target),
                             a_plan_min=float(np.min(self.a_desired_trajectory)))
      if self.ocm.active:
        output_a_target = OCM.get_coast_target(accel_coast)
        self.output_should_stop = False
    else:
      self.ocm.active = False

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def _reset_e2e_guard(self):
    self.e2e_guard_active = False
    self.e2e_guard_cnt = 0
    self.e2e_guard_t = 0.0
    self.e2e_guard_w = 0.0

  def _update_e2e_guard(self, a_mpc, a_e2e, gas_prob):
    # dp(v12.1): e2e 減速保險狀態機，見 E2E_GUARD_*；gas_prob 為 gasPressProbs[1] 原始值
    if not self.e2e_guard_active:
      hit = (a_e2e < E2E_GUARD_ENTER_ACCEL and (a_e2e - a_mpc) < E2E_GUARD_ENTER_DIFF and
             gas_prob < E2E_GUARD_ENTER_GAS_PROB)
      self.e2e_guard_cnt = self.e2e_guard_cnt + 1 if hit else 0
      if self.e2e_guard_cnt >= E2E_GUARD_CONFIRM_FRAMES:
        self.e2e_guard_active = True
        self.e2e_guard_t = 0.0
        cloudlog.info(f"[E2EGuard_DP] enter mpc={a_mpc:.2f} e2e={a_e2e:.2f} gas={gas_prob:.2f}")
    else:
      self.e2e_guard_t += self.dt
      released = a_e2e >= 0.0 or gas_prob >= E2E_GUARD_EXIT_GAS_PROB
      if (released and self.e2e_guard_t >= E2E_GUARD_MIN_HOLD_S) or self.e2e_guard_t >= E2E_GUARD_MAX_HOLD_S:
        self.e2e_guard_active = False
        self.e2e_guard_cnt = 0
        cloudlog.info(f"[E2EGuard_DP] exit after {self.e2e_guard_t:.2f}s mpc={a_mpc:.2f} e2e={a_e2e:.2f} gas={gas_prob:.2f}")
    step = self.dt / E2E_GUARD_RAMP_S
    if self.e2e_guard_active:
      self.e2e_guard_w = min(1.0, self.e2e_guard_w + step)
    else:
      self.e2e_guard_w = max(0.0, self.e2e_guard_w - step)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
    
    if hasattr(self, 'publish_longitudinal_plan_dp'):
      self.publish_longitudinal_plan_dp(sm, pm)
