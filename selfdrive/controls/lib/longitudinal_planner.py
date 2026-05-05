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
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.radard import RadarD
# --- DP Imports ---
from dragonpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerDP
from dragonpilot.selfdrive.controls.lib.acm import ACM
from dragonpilot.selfdrive.controls.lib.aem import AEM
from dragonpilot.selfdrive.controls.lib.apm import APM
from dragonpilot.selfdrive.controls.lib.dasr import DASR
#from dragonpilot.selfdrive.controls.lib.radard import RadarD   # ✅ 新增

try:
  from dragonpilot.dashy.maa.lib.longitudinal_helper import LongitudinalHelper, RadarStateWrapper
  MAA_PLANNER_AVAILABLE = True
except ImportError:
  MAA_PLANNER_AVAILABLE = False
  RadarStateWrapper = None


class DPFlags:
  ACM = 1
  AEM = 2
  DTSC = 2 ** 2
  APM = 2 ** 3
  DASR = 2 ** 4


# A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MAX_BP = [0., 2.77, 5.55, 8.33, 11.11, 13.89, 16.6, 19.4, 22.22, 25, 27.78, 33.33]
A_CRUISE_MAX_VALS = [1.41, 1.3, 1.1, 0.75, 0.6, 0.45, 0.4, 0.35, 0.35, 0.33, 0.31, 0.29]

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


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

    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    # DP Modules
    self.acm = ACM()
    self.aem = AEM()
    self.apm = APM()
    self.dasr = DASR()
    self.rd = RadarD()                 # ✅ RD 初始化
    self.maa_helper = LongitudinalHelper() if MAA_PLANNER_AVAILABLE else None

  # ---- 以下所有函数与原版保持一致，仅 update() 有少量插入 ----

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
      x = v = a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1] \
      if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1 else 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm, dp_flags=0):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    LongitudinalPlannerDP.update(self, sm)

    if dp_flags & DPFlags.AEM:
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo)
      mode = self.aem.get_mode(mode)

    accel_coast = get_coast_accel(sm['carControl'].orientationNED[1]) \
      if len(sm['carControl'].orientationNED) == 3 else ACCEL_MAX

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
    else:
      accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if self.maa_helper is not None:
      v_cruise, accel_clip, virtual_lead = self.maa_helper.process(sm, v_ego, v_cruise, accel_clip)
    else:
      virtual_lead = None

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(
        v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2],
        [accel_clip[1], clipped_accel_coast]
      )
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    personality = sm['selfdriveState'].personality
    if dp_flags & DPFlags.APM:
      lead_one = sm['radarState'].leadOne
      has_lead = lead_one.status
      v_lead = lead_one.vLead if has_lead else 0.0
      a_lead = lead_one.aLeadK if has_lead else 0.0
      d_lead = lead_one.dRel if has_lead else 0.0

      personality = self.apm.get_personality(v_ego, has_lead, v_lead, a_lead, d_lead, personality)

    self.mpc.set_weights(prev_accel_constraint, personality=personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    if virtual_lead and RadarStateWrapper:
      radar_state = RadarStateWrapper(sm['radarState'], virtual_lead)
    else:
      radar_state = sm['radarState']

    if dp_flags & DPFlags.DTSC:
      a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(sm['modelV2'], v_ego, accel_clip[0], accel_clip[1])
      for i in range(len(a_min_dtsc)):
        self.mpc.params[i, 0] = max(accel_clip[0], a_min_dtsc[i])
        self.mpc.params[i, 1] = min(accel_clip[1], a_max_dtsc[i])
        if self.mpc.params[i, 0] > self.mpc.params[i, 1]:
          self.mpc.params[i, 0] = self.mpc.params[i, 1] - 0.05

    v_cruise_target, a_target_from_dp = LongitudinalPlannerDP.update_targets(
      self, sm, self.v_desired_filter.x, self.a_desired, v_cruise
    )
    a_cruise_min_override = LongitudinalPlannerDP.get_cruise_min_accel(self, v_ego)

    if force_slow_decel:
      v_cruise_target = 0.0

    self.mpc.update(radar_state, v_cruise_target, personality=personality,
                    a_cruise_min_override=a_cruise_min_override)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)

    # ===============================
    # ACM
    # ===============================
    if dp_flags & DPFlags.ACM:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(
        sm['carControl'], sm['radarState'],
        user_control, v_ego, v_cruise, mode=mode,
        personality=personality, dtsc_is_active=getattr(self.dtsc, 'active', False)
      )

      lead = sm['radarState'].leadOne
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(
        self.a_desired_trajectory,
        v_ego=v_ego,
        lead=lead
      )

    # ==================================================
    # ✅ 雷达阻尼参数配置
    # ==================================================
    # 实验模式 (Blended)
    BLENDED_MIN_DAMP = 0.6
    BLENDED_MAX_DAMP = 1.0

    # ACC 模式
    ACC_MIN_DAMP = 0.3
    ACC_MAX_DAMP = 1.0
    # ===============================
    # ✅ Radar damping（crossing / static）
    # ===============================
    damping = 1.0
    if dp_flags & (DPFlags.DTSC | DPFlags.ACM):
      crossing_factor = self.rd.crossing_slowdown_factor
      static_factor = self.rd.static_slowdown_factor
   
      damping = min(crossing_factor, static_factor)
      if mode == 'blended':
          damping = max(BLENDED_MIN_DAMP, min(BLENDED_MAX_DAMP, damping))
      else:
          damping = max(ACC_MIN_DAMP, min(ACC_MAX_DAMP, damping))

    self.a_desired_trajectory *= damping

    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x += self.dt * (self.a_desired + a_prev) / 2.0

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory, self.a_desired_trajectory,
      CONTROL_N_T_IDX, action_t=action_t,
      vEgoStopping=self.CP.vEgoStopping
    )

    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'blended':
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    if dp_flags & DPFlags.DASR:
      has_lead = sm['radarState'].leadOne.status
      self.dasr.update(v_ego, output_a_target, has_lead=has_lead)
      slew_rate_down = self.dasr.slew_rate_down
      slew_rate_up = self.dasr.slew_rate_up
    else:
      slew_rate_down = slew_rate_up = 0.05

    for idx in range(2):
      accel_clip[idx] = np.clip(
        accel_clip[idx],
        self.prev_accel_clip[idx] - slew_rate_down,
        self.prev_accel_clip[idx] + slew_rate_up
      )

    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip.copy()

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
