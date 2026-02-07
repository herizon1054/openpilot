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
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, SOURCES
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

# ============================================================
# DragonPilot Imports
# ============================================================
from dragonpilot.selfdrive.controls.lib.acm import ACM
from dragonpilot.selfdrive.controls.lib.aem import AEM
from dragonpilot.selfdrive.controls.lib.dtsc import DTSC

class DPFlags:
  ACM = 1
  AEM = 2
  DTSC = 2 ** 2

# ============================================================

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
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

    # ============================================================
    # DragonPilot Init
    # ============================================================
    self.acm = ACM()
    self.aem = AEM()
    self.dtsc = DTSC(aggressiveness=0.8)
    # ============================================================

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

  def update(self, sm, dp_flags=0):
    # ============================================================
    # DP AEM Integration (Pre-Calculation)
    # ============================================================
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'
    if dp_flags & DPFlags.AEM:
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo)
      mode = self.aem.get_mode(mode)
    # ============================================================

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

    # ============================================================
    # DP AEM Logic
    # ============================================================
    if mode == 'acc':
      accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]
    # ============================================================

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # ============================================================
    # DP DTSC Integration (Hybrid Control)
    # ============================================================
    if dp_flags & DPFlags.DTSC:
      dtsc_suggested_speed = getattr(self.dtsc, 'get_suggested_speed', lambda: 255.0)()
      if dtsc_suggested_speed < 254.0:
          v_cruise = min(v_cruise, dtsc_suggested_speed)

      a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(
        sm['modelV2'], v_ego, accel_clip[0], accel_clip[1])

      for i in range(len(a_min_dtsc)):
        target_min = max(accel_clip[0], a_min_dtsc[i])
        target_max = min(accel_clip[1], a_max_dtsc[i])
        if target_min > target_max:
             target_min = target_max - 0.05
        self.mpc.params[i, 0] = target_min
        self.mpc.params[i, 1] = target_max
    # ============================================================

    # Run MPC
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # ============================================================
    # DP ACM Integration (Modified for 76% Coasting Fix)
    # ============================================================
    if dp_flags & DPFlags.ACM:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise, 
                             personality=sm['selfdriveState'].personality)
      
      # [DP TWEAK] 起步保護邏輯：
      # 將門檻提高到 30 km/h (約 8.33 m/s)。
      # 當車速低於此數值且 MPC 請求正向加速 (>0.1) 時，強制跳過 ACM 的軌跡修正。
      # 確保 0-30km/h 的起步過程動力連貫，不會被 76% 滑行邏輯切斷。
      is_starting_up = v_ego < 8.33 and self.a_desired_trajectory[0] > 0.1
      
      if not is_starting_up:
        self.a_desired_trajectory = self.acm.update_a_desired_trajectory(
            self.a_desired_trajectory,
            v_ego=v_ego,
            lead=sm['radarState'].leadOne
        )
    # ============================================================

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

    if mode == 'blended':
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = SOURCES[3]
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    # ============================================================
    # DP Dynamic Slew Rate (Fix for slow acceleration buildup)
    # ============================================================
    decel_slew_rate = 0.05 
    
    is_dtsc_active = False
    if dp_flags & DPFlags.DTSC:
        is_dtsc_active = getattr(self.dtsc, 'active', False)

    is_aem_braking = (mode == 'blended' and output_a_target < 0.0)
    is_dtsc_braking = (is_dtsc_active and output_a_target < 0.0)

    if is_dtsc_braking:
        decel_slew_rate = 0.10
    elif is_aem_braking:
        decel_slew_rate = 0.05

    # [DP TWEAK] 動態加速 Slew Rate：
    # 解決起步時加速度被鎖定在 +0.05/step 導致的階梯式爬升問題。
    # 當車速 < 8.33 m/s (30 km/h) 時，允許加速度每幀增加 0.12，讓起步更靈敏。
    accel_up_slew = 0.12 if v_ego < 8.33 else 0.05

    for idx in range(2):
      accel_clip[idx] = np.clip(
          accel_clip[idx], 
          self.prev_accel_clip[idx] - decel_slew_rate, 
          self.prev_accel_clip[idx] + accel_up_slew
      )
    
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
