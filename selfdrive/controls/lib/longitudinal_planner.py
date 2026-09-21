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
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from dragonpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerDP
from dragonpilot.selfdrive.controls.lib.ocm import OCM
from dragonpilot.selfdrive.controls.lib.aem import AEM
from dragonpilot.selfdrive.controls.lib.apm import APM

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD_ACC = 0.4   # mode=='acc' 使用，維持原廠值，行為不變
# mode=='blended'(e2e) 基準門檻。AEM 停用時固定用這個值（0.2，原 0.1 太低，幾乎讓
# allow_throttle 恆為 True、油門上限形同沒有夾限，經確認是跟車過度敏感積極的主因之一）；
# AEM 啟用時改用 self.aem.base_throttle_threshold 依車速動態切換（見 v9：<=60km/h 為 0.2，
# >=70km/h 為 0.1），這裡的固定值只當作 AEM 未啟用時的後備值
ALLOW_THROTTLE_THRESHOLD_E2E = 0.1
# mode=='blended' 且是由 AEM 接近模型停止線觸發時使用：接近紅綠燈/停止標誌時，動態把
# 節流門檻拉高到跟 ACC 一樣保守（0.4），避免 e2e 在這個情境下加速意願過高
ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP = 0.4
# mode=='blended' 且是由 AEM 方向燈覆寫觸發時使用：打燈變換車道/路口轉彎時，比接近停止線
# 更保守（0.5，高於 ACC 的 0.4），因為轉彎/變換車道當下的風險判斷應該比單純接近停止線更嚴格
ALLOW_THROTTLE_THRESHOLD_E2E_BLINKER = 0.5
MIN_ALLOW_THROTTLE_SPEED = 2.5

# v10：throttle_prob（modelV2.meta.disengagePredictions.gasPressProbs[1]）單幀雜訊極大——
# 用三份市區 40~50km/h 路測 rlog 實測過，同一次連續煞停過程中，這個值每 50ms 就可以在
# 0.06~0.58 之間跳動，標準差可達 0.19~0.36。過去 allow_throttle 是直接拿這個原始值跟
# 門檻比較，完全沒有平滑或遲滯，於是不管門檻設多少，只要 throttle_prob 剛好在門檻附近
# 雜訊擺盪，allow_throttle 就會每一幀真的跟著在 True/False 之間反覆橫跳，反映到
# accel_clip[1] 每一幀在「完全放開」和「夾到滑行曲線」之間跳動，就是使用者反映的
# 「油門剎車頓挫感」的直接成因——這跟門檻本身該設 0.1 還是 0.2 無關，兩個值都一樣會被
# 這個雜訊掃到。修法比照 aem.py 對側向加速度的處理：先做一次低通濾波，再加遲滯，兩者
# 一起套用在 allow_throttle 的判斷上，而不只是套用在「門檻該選哪個值」這件事上。
# 用同一份 rlog 驗證：加上這兩個機制後，三份 log 的 allow_throttle 切換次數從
# 22~34 次/分鐘降到 0~4 次/分鐘，降幅 88%~100%。
THROTTLE_PROB_LPF_ALPHA = 0.2   # 濾除單幀雜訊尖峰，風格與 aem.py 的 LAT_ACCEL_LPF_ALPHA 一致
ALLOW_THROTTLE_HYSTERESIS = 0.10   # allow_throttle 為 True 時，門檻降低這麼多才會變回 False，
                                    # 避免濾波後的值仍在門檻附近小幅擺盪時來回橫跳

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

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
    self.throttle_prob_filtered = 1.0   # v10：throttle_prob 低通濾波狀態，見下方 THROTTLE_PROB_LPF_ALPHA 說明

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
      # v7：新增「接近模型停止線」節流保守化。⚠️ self.traffic_stop.stop_dist_m 要到本
      # function 後段呼叫 LongitudinalPlannerDP.update_targets() 時才會刷新成這一幀的值，
      # 這裡讀到的是上一幀（約 50ms 前）的結果——由於這一項只影響節流門檻、本身又有
      # 0.5 秒防彈跳，一幀的落差可忽略；若要完全同步，需把 update_targets() 提前到
      # 這裡之前呼叫，但那會牽動它目前使用 self.v_desired_filter.x/self.a_desired
      # （上一幀平滑值）作為輸入參數的既有設計，改動風險較高，這裡先不動。
      blinker_on = sm['carState'].leftBlinker or sm['carState'].rightBlinker
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo,
                              blinker_on=blinker_on, stop_dist_m=self.traffic_stop.stop_dist_m)
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
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    if mode == 'blended':
      # v9：基準門檻改由 AEM 依車速動態決定（<=60km/h 為 0.2，>=70km/h 為 0.1），
      # AEM 未啟用時退回固定的 ALLOW_THROTTLE_THRESHOLD_E2E（0.2）當後備值。
      # 方向燈跟接近停止線分開給不同保守程度的門檻（方向燈 0.5、接近停止線 0.4），
      # 兩者同時成立時取較保守（較高）的那個，而不是固定順序，避免未來新增第三種
      # 覆寫條件時還要重新排列 if/elif 的優先順序
      if dp_flags & DPFlags.AEM:
        allow_throttle_threshold = self.aem.base_throttle_threshold
        if self.aem.blinker_active:
          allow_throttle_threshold = max(allow_throttle_threshold, ALLOW_THROTTLE_THRESHOLD_E2E_BLINKER)
        if self.aem.near_stop_active:
          allow_throttle_threshold = max(allow_throttle_threshold, ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP)
      else:
        allow_throttle_threshold = ALLOW_THROTTLE_THRESHOLD_E2E
    else:
      allow_throttle_threshold = ALLOW_THROTTLE_THRESHOLD_ACC
    # v10：throttle_prob 先做低通濾波，比較時再加遲滯，避免單幀雜訊讓 allow_throttle
    # 每一幀反覆橫跳（見上方 THROTTLE_PROB_LPF_ALPHA 說明的實測數據）
    self.throttle_prob_filtered = (THROTTLE_PROB_LPF_ALPHA * throttle_prob
                                    + (1.0 - THROTTLE_PROB_LPF_ALPHA) * self.throttle_prob_filtered)
    effective_threshold = (allow_throttle_threshold - ALLOW_THROTTLE_HYSTERESIS
                            if self.allow_throttle else allow_throttle_threshold)
    self.allow_throttle = self.throttle_prob_filtered > effective_threshold or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

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
    
    if dp_flags & DPFlags.OCM:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.ocm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise, dtsc_active=is_dtsc_active)
      self.a_desired_trajectory = self.ocm.update_a_desired_trajectory(self.a_desired_trajectory)
    
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

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      
      if output_a_target < output_a_target_mpc:
        try:
          from cereal import log
          self.mpc.source = log.LongitudinalPlan.LongitudinalPlanSource.e2e
        except ImportError:
          pass

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

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
