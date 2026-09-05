#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop_controller import TrafficStopController

A_CRUISE_MAX_VALS = [1.6, 1.1, 0.6, 0.2]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.1, 0.6, 0.2]
A_CRUISE_MIN = -1.2
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


def taper_toward_less_conservative_output(candidate_min: float, prev_output: float, j_taper: float, dt: float) -> float:
  """Only limits the *increase* (less braking / more accel) direction when the winning candidate
  source just changed to a less conservative one than last frame; a candidate that wants to brake
  *harder* than prev_output is never limited (min() lets it through immediately and uncapped).
  See the CONFIRMED DESIGN comment at its call site in LongitudinalPlanner.update() for why this
  exists: it smooths the specific candidate-source "seam" left when e2e drops out of the pool."""
  return min(candidate_min, prev_output + j_taper * dt)

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.traffic_stop_controller = TrafficStopController()
    self.traffic_stop_active = False
    self._prev_output_source = None

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    # Traffic-light / stop-sign virtual stop-line obstacle (sunnypilot addition, ported from carrot).
    # Uses the driving model's own predicted trajectory; no separate traffic-light classifier needed.
    # CONFIRMED DESIGN (differs from cp on purpose): unlike cp's `long_mpc.py`, which fully disables
    # this obstacle whenever mode=='blended' (`stop_x = 1000.0`), this call is unconditional here --
    # it runs identically regardless of Normal (ACC) mode, Experimental Mode, or Dynamic Experimental
    # Control's per-frame acc/blended switching. The obstacle always feeds the MPC below; only the
    # separate e2e *candidate* further down gets excluded while a stop is active (see comment there).
    # This keeps the safety net active in every driving mode rather than having it silently drop out
    # whenever DEC decides to go blended for unrelated reasons (standstill, FCW, radarless slowdown).
    traffic_stop_result = self.traffic_stop_controller.update(
      sm['modelV2'], sm['carState'], sm['radarState'], v_ego, self.output_a_target, v_cruise)
    if traffic_stop_result.v_cruise_limited is not None:
      v_cruise = min(v_cruise, traffic_stop_result.v_cruise_limited)
    self.traffic_stop_active = traffic_stop_result.stop_dist_m is not None

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality,
                     traffic_stop_obstacle_m=traffic_stop_result.stop_dist_m)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    # While actively managing a traffic-stop (obstacle already reflected in output_a_target_mpc
    # above, regardless of mode -- see the CONFIRMED DESIGN comment above), exclude only the e2e
    # candidate. min() over all candidates already guarantees the virtual obstacle acts as a floor
    # even with e2e included, but e2e's own stop-line judgement is not reliable enough here and its
    # noise can cause visible jerkiness (unnecessary re-accel/re-brake) right at the moment we're
    # trying to hold a smooth stop. This bypass is automatically gated by the TrafficStopEnabled
    # toggle, since traffic_stop_active is only ever True when the toggle is on. It applies the same
    # way whether is_e2e came from static Experimental Mode or from DEC's per-frame decision --
    # there is no mode-based path that skips it.
    if is_e2e and not self.traffic_stop_active:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # Mitigate a candidate-source "seam": e2e's own end-to-end judgement often anticipates the
    # need to slow down (e.g. for a red light) earlier than the rule-based traffic-stop detector
    # formally commits (self.traffic_stop_active), since the detector deliberately waits for the
    # model's trajectory to clearly settle before triggering, to avoid false positives. While e2e
    # is in the candidate pool and winning, it quietly covers for that detection lag. The instant
    # e2e drops out of the pool -- either because is_e2e flips to False (user leaves Experimental
    # Mode) or because traffic_stop_active flips to True (rule-based detector takes over and
    # excludes e2e, see the candidates.append comment above) -- the remaining candidates can
    # momentarily be *less* conservative than what e2e was just providing, producing a visible
    # accelerate-then-brake blip right as the detector catches up a few frames later.
    # Only the *increase* (accelerate) direction is limited here, using the same jerk budget as
    # get_cruise_accel's own j_cruise; braking harder is always let through immediately and
    # uncapped, so this can never delay a genuinely required deceleration (FCW, lead cut-in,
    # etc). The limit self-resolves within about a second as the ceiling rises each frame, or
    # sooner once a comparably conservative candidate (e.g. the newly-injected traffic-stop
    # obstacle) takes over on its own.
    if self._prev_output_source == LongitudinalPlanSource.e2e and self.mpc.source != LongitudinalPlanSource.e2e:
      j_taper = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
      output_a_target = taper_toward_less_conservative_output(output_a_target, self.output_a_target, j_taper, self.dt)
    self._prev_output_source = self.mpc.source

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
