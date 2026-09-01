"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

TrafficStopController: stateful layer for the traffic-light / stop-sign
virtual stop-line obstacle. Ported from carrot openpilot fork's
`selfdrive/carrot/carrot_functions.py` (`CarrotPlanner.check_model_stopping`
and the XState/TrafficState machine inside `CarrotPlanner.update`),
re-modularized for sunnypilot master-dev with a UI toggle.

This version has been checked line-by-line against the real cp source
(previously only a written spec was available). Differences from the first
port pass that were caught by that comparison are marked "FIXED (verified
against cp source)" below. Intentional, permanent simplifications relative
to cp (dropped ATC-nav/DrivingMode/user_stop_distance/carrot_stay_stop
features, XState.lead/e2ePrepare folded into CRUISE) are unchanged from the
original spec and are not re-flagged here.
"""
from collections import deque
from dataclasses import dataclass
from enum import IntEnum

from numpy import clip, interp

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop import (
  is_traffic_stop_entry_allowed,
  get_traffic_stop_reference_speed,
  get_virtual_traffic_stop_distance,
  get_traffic_stop_obstacle_distance,
)

STOP_MODEL_IDX = -2                            # 33-point model trajectory, 2nd-to-last point (cp: x[31])
NO_STOP_DISTANCE_M = 1000.0                    # "no lead" sentinel distance

# FIXED (verified against cp source): cp's CarrotPlanner.comfortBrake is 2.4, not 2.5.
DEFAULT_COMFORT_BRAKE = 2.4                    # m/s^2 baseline for the v_cruise soft-limit formula
STOPPING_COMFORT_BRAKE_FACTOR = 0.9            # cp multiplies comfort_brake by this while actively braking (STOPPING only)

LEAD_CLOSE_TO_STOP_LINE_M = 4.0                # cancel an active stop if a real lead is this much closer than the stop line
# TUNED (was 2.0, cp's real default): in the boundary case where a real lead's distance is
# just outside this margin, the state machine does NOT hand off to normal lead-following, so the
# MPC ends up targeting the (unbuffered) virtual stop-line obstacle instead of the real lead's
# own properly-buffered obstacle distance -- see get_stopped_equivalence_factor() in long_mpc.py,
# which adds ~0 extra buffer for a stationary lead. The final gap to a real lead in that boundary
# case converges to roughly this margin value, so raising it from 2.0m to 4.0m raises the
# worst-case final stopped gap from ~2m to ~4m. This does not change the *entry* gate (still
# blocked by any detected lead, regardless of distance) or make cancellation trigger on distant/
# irrelevant leads elsewhere in radar range -- it only widens how close a real lead near the
# stop line needs to be before the module hands off to normal lead-following.
STOP_DISTANCE_RELATCH_MIN_M = 10.0             # cp: only re-latch actual_stop_distance from the model while still >10m out

STOPPED_SPEED_MS = 0.3                         # "fully stopped" threshold
STOPPED_GRACE_FRAMES = int(0.5 / DT_MDL)       # 10 frames = 0.5s: minimum time in STOPPED before a green light is allowed to release the *state* (cp: stopping_count)
STARTING_SUPPRESS_FRAMES = int(10.0 / DT_MDL)  # 200 frames = 10s: suppress new entries after gas is pressed mid-brake
PARAM_POLL_FRAMES = int(1.0 / DT_MDL)          # 20 frames = 1s, UI Params polling interval

DISTANCE_ADJUST_MIN_M = -5
DISTANCE_ADJUST_MAX_M = 5

# cp's single `TrafficStopDistanceAdjust` Params value conflates two different things into one
# number: a large, roughly-fixed physical correction for the model's stop-line distance being
# measured from the camera/device position (set back from the actual front bumper on the
# vehicle) rather than from the bumper itself, AND a small personal fine-tune on top of that.
# cp's real shipped default is raw Params value -150 (see params_keys.h), divided by 100 in
# carrot_functions.py -> -1.5m, with a UI range of -6.00..+6.00m -- i.e. a wide range centered
# on an already-large, unintuitive negative default, needed specifically because the two
# purposes were never separated. That -150 default is NOT the same as the placeholder `2.5` seen
# in carrot_functions.py's __init__: that 2.5 is dead code (the real Params-reading line right
# next to it is commented out) and gets overwritten by the real value on the next _params_update().
#
# Here the two are split: TRAFFIC_STOP_CAMERA_TO_FRONT_M is applied unconditionally as a fixed
# baseline (not user-facing), and TrafficStopDistanceAdjust (still Params-backed, still -5..+5m,
# default 0) is a genuine small fine-tune on top of that baseline, so a user who never touches
# the toggle gets cp's real-world-tuned stopping behavior, and the UI slider means exactly what
# it looks like it means instead of requiring the user to first cancel out an unexplained -1.5m.
#
# CAVEAT: -1.5m is cp's own default assumption for wherever cp's target hardware is mounted
# relative to that vehicle's front bumper. This is inherently install/vehicle-specific, not a
# universal constant -- it has not been independently re-derived or verified against sp260820's
# own calibration pipeline or hardware mounting positions. Treat it as a reasonable ported
# starting point, not a proven-correct value for every car/install this fork supports.
TRAFFIC_STOP_CAMERA_TO_FRONT_M = -1.5

GREEN_CONFIRM_SEC = 0.2                        # green needs ~4 frames to confirm; red is single-frame (no debounce -- ported as-is from cp)

STOP_SIGN_MAX_SPEED_KPH = 82.0
STOP_SIGN_DISTANCE_BP_KPH = (60.0, 80.0)
STOP_SIGN_DISTANCE_LIMIT_M = (120.0, 150.0)
STOP_SIGN_LATERAL_GUARD_M = 5.0                # known gap on wide/high-speed turns -- see porting spec section 8.1

_MODEL_V_FILTER_LEN = 10                       # cp: vFilter
_STOP_X_MEDIAN_LEN = 3                         # cp: xStopFilter
_STOP_X_AVG_LEN = 15                           # cp: xStopFilter2


class TrafficLightState(IntEnum):
  OFF = 0
  RED = 1
  GREEN = 2


class TrafficStopState(IntEnum):
  CRUISE = 0
  STOPPING = 1
  STOPPED = 2


@dataclass
class TrafficStopResult:
  stop_dist_m: float | None       # None => no active virtual obstacle this frame
  v_cruise_limited: float | None  # None => no additional cruise-speed limiting this frame


def _median(values: list[float]) -> float:
  s = sorted(values)
  n = len(s)
  mid = n // 2
  if n % 2:
    return s[mid]
  return (s[mid - 1] + s[mid]) / 2.0


class TrafficStopController:
  def __init__(self, params=None):
    self._params = params or Params()

    self._frame = 0
    self._enabled = False
    self._distance_adjust_m = 0.0

    self._state = TrafficStopState.CRUISE
    self._stopped_grace_frames = 0
    self._gas_suppress_frames = 0

    self._stop_sign_count = 0
    self._start_sign_count = 0

    # FIXED (verified against cp source): cp's vFilter/xStopFilter/xStopFilter2 run every
    # frame unconditionally and are never cleared -- they are NOT reset on release/disable,
    # so these histories persist across separate stop events (only the `_stop_x_rl` rate-limiter
    # output, and the accumulator below, are reset). Keeping the raw-model filters continuously
    # warm means a fresh red-light entry starts from an already-settled number instead of a
    # median/average of just 1-2 samples.
    self._model_v_hist: deque = deque(maxlen=_MODEL_V_FILTER_LEN)
    self._stop_x_median_hist: deque = deque(maxlen=_STOP_X_MEDIAN_LEN)
    self._stop_x_avg_hist: deque = deque(maxlen=_STOP_X_AVG_LEN)
    self._stop_x_rl: float | None = None

    self._reference_speed_kph: float = 0.0
    self._actual_stop_distance: float = 0.0

  def _reset(self):
    """Reset the per-stop-event state back to cruise / no-obstacle. Called on disable and on release.
    Deliberately does NOT clear _model_v_hist / _stop_x_median_hist / _stop_x_avg_hist -- see note
    in __init__."""
    self._state = TrafficStopState.CRUISE
    self._stopped_grace_frames = 0
    self._stop_sign_count = 0
    self._start_sign_count = 0
    self._reference_speed_kph = 0.0
    self._actual_stop_distance = 0.0

  def _poll_params(self):
    if self._frame % PARAM_POLL_FRAMES == 0:
      self._enabled = self._params.get_bool("TrafficStopEnabled")
      raw = self._params.get("TrafficStopDistanceAdjust", return_default=True)
      self._distance_adjust_m = float(clip(int(raw or 0), DISTANCE_ADJUST_MIN_M, DISTANCE_ADJUST_MAX_M))
    self._frame += 1

  def _check_model_stopping(self, v_cruise: float, model_v_traj, v_ego: float, a_ego: float,
                             model_x_end: float, model_y_traj, d_rel: float) -> TrafficLightState:
    v_ego_kph = v_ego * 3.6

    self._model_v_hist.append(model_v_traj[-1])
    model_v = sum(self._model_v_hist) / len(self._model_v_hist)

    start_sign = model_v > 5.0 or model_v > (model_v_traj[0] + 2)

    if v_ego_kph < 1.0:
      stop_sign = model_x_end < 20.0 and model_v < 10.0
    elif v_ego_kph < STOP_SIGN_MAX_SPEED_KPH:
      max_detect_dist = interp(model_v_traj[0] * 3.6, STOP_SIGN_DISTANCE_BP_KPH, STOP_SIGN_DISTANCE_LIMIT_M)
      stop_sign = (
        model_x_end < d_rel - 3.0 and
        model_x_end < max_detect_dist and
        (model_v < 3.0 or model_v < model_v_traj[0] * 0.7) and
        abs(model_y_traj[-1]) < STOP_SIGN_LATERAL_GUARD_M
      )
      # avoid false trigger from ordinary cruise deceleration (e.g. speed camera) unless already stopping
      if v_cruise != 0 and self._state == TrafficStopState.CRUISE and a_ego < -1.0:
        stop_sign = False
    else:
      stop_sign = False  # do not evaluate above 82 kph

    self._stop_sign_count = self._stop_sign_count + 1 if stop_sign else 0
    self._start_sign_count = self._start_sign_count + 1 if (start_sign and not stop_sign) else 0

    if self._stop_sign_count * DT_MDL > 0.0:
      return TrafficLightState.RED
    if self._start_sign_count * DT_MDL > GREEN_CONFIRM_SEC:
      return TrafficLightState.GREEN
    return TrafficLightState.OFF

  def _update_stop_model_x(self, raw_x: float, v_ego: float) -> tuple[float, float]:
    """Continuous median(3) + moving-average(15) filter, then a rate limiter that only caps
    the *closing* rate (jumps out immediately when the model reports "farther away"). Runs
    every frame regardless of state -- see the note in __init__.
    Returns (stop_model_x_raw, stop_model_x_rl): the filtered-but-not-rate-limited value (used
    for the close-lead check) and the filtered-and-rate-limited value (used for (re-)latching
    actual_stop_distance).
    """
    self._stop_x_median_hist.append(raw_x)
    med = _median(list(self._stop_x_median_hist))
    self._stop_x_avg_hist.append(med)
    stop_model_x_raw = sum(self._stop_x_avg_hist) / len(self._stop_x_avg_hist)

    if self._stop_x_rl is None:
      self._stop_x_rl = stop_model_x_raw
    else:
      max_close = v_ego * DT_MDL + 0.5
      if stop_model_x_raw > self._stop_x_rl:
        self._stop_x_rl = stop_model_x_raw  # getting farther away: jump immediately
      else:
        self._stop_x_rl = max(self._stop_x_rl - max_close, stop_model_x_raw)  # getting closer: rate-limited

    return stop_model_x_raw, self._stop_x_rl

  def update(self, model_v2, car_state, radar_state, v_ego: float, a_ego: float, v_cruise: float) -> TrafficStopResult:
    self._poll_params()

    if not self._enabled:
      self._reset()
      return TrafficStopResult(stop_dist_m=None, v_cruise_limited=None)

    model_v_traj = model_v2.velocity.x
    model_y_traj = model_v2.position.y
    model_x_traj = model_v2.position.x
    if len(model_x_traj) < abs(STOP_MODEL_IDX) or len(model_v_traj) == 0:
      # incomplete model output this frame -- inject nothing, keep prior latched state as-is
      return TrafficStopResult(stop_dist_m=None, v_cruise_limited=None)

    model_x_end = model_x_traj[-1]

    lead = radar_state.leadOne
    lead_present = lead.present
    d_rel = lead.dRel if lead_present else NO_STOP_DISTANCE_M

    steering_angle_deg = car_state.steeringAngleDeg
    gas_pressed = car_state.gasPressed
    left_blinker = car_state.leftBlinker

    # continuous filtering -- runs every frame no matter what state we're in
    stop_model_x_raw, stop_model_x_rl = self._update_stop_model_x(model_x_traj[STOP_MODEL_IDX], v_ego)

    traffic_state = self._check_model_stopping(v_cruise, model_v_traj, v_ego, a_ego, model_x_end, model_y_traj, d_rel)

    # FIXED (verified against cp source): cp only arms the 10s suppression window from a gas
    # press *while actively braking toward a stop* (XState.e2eStop) -- not on every gas press.
    # The original port armed it on any gas press at all, which in ordinary stop-and-go city
    # driving would keep the feature permanently suppressed.
    if gas_pressed and self._state == TrafficStopState.STOPPING:
      self._gas_suppress_frames = STARTING_SUPPRESS_FRAMES
    elif self._gas_suppress_frames > 0:
      self._gas_suppress_frames -= 1

    # FIXED (verified against cp source): cp's close-lead check for CANCELLING an active stop
    # compares against stop_model_x_raw (filtered, not rate-limited) with a 2m margin. The gate
    # for entering a NEW stop is different and simpler -- see below.
    lead_closer_than_stop = lead_present and (d_rel - stop_model_x_raw) < LEAD_CLOSE_TO_STOP_LINE_M

    # --- state machine (see porting spec section 6, corrected against cp source) ---
    if self._state == TrafficStopState.CRUISE:
      # FIXED (verified against cp source): entry is blocked by *any* detected lead at all
      # (cp: `if lead_detected: xState = XState.lead`), not specifically a lead closer than the
      # stop line. If a real car is already there, the MPC's own lead-following naturally
      # produces the same stop; a redundant virtual obstacle is unnecessary.
      entry_allowed = is_traffic_stop_entry_allowed(steering_angle_deg)
      if (not lead_present and traffic_state == TrafficLightState.RED and
          entry_allowed and self._gas_suppress_frames == 0):
        self._state = TrafficStopState.STOPPING
        self._reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, None)
        self._actual_stop_distance = get_virtual_traffic_stop_distance(stop_model_x_rl, self._reference_speed_kph)

    elif self._state == TrafficStopState.STOPPING:
      if gas_pressed:
        self._state = TrafficStopState.CRUISE
      elif lead_closer_than_stop:
        self._state = TrafficStopState.CRUISE
      elif traffic_state == TrafficLightState.GREEN:
        self._state = TrafficStopState.CRUISE
      else:
        # still red (or off): keep braking. Reduced comfort_brake while actively approaching.
        self._reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, self._reference_speed_kph)
        candidate = get_virtual_traffic_stop_distance(stop_model_x_rl, self._reference_speed_kph)
        # only re-latch from the model while still far out; trust the dead-reckoned countdown
        # for the final approach so last-meters model noise can't cause a jerk
        if candidate > STOP_DISTANCE_RELATCH_MIN_M:
          self._actual_stop_distance = candidate
        if v_ego < STOPPED_SPEED_MS:
          self._state = TrafficStopState.STOPPED
          self._stopped_grace_frames = STOPPED_GRACE_FRAMES

    elif self._state == TrafficStopState.STOPPED:
      if gas_pressed:
        self._state = TrafficStopState.CRUISE
      elif lead_closer_than_stop:
        self._state = TrafficStopState.CRUISE
      else:
        if self._stopped_grace_frames == 0:
          if traffic_state == TrafficLightState.GREEN and not left_blinker:
            self._state = TrafficStopState.CRUISE
        self._stopped_grace_frames = max(0, self._stopped_grace_frames - 1)

    # Instantaneous *obstacle* release: off/green clears the output THIS FRAME even if `_state`
    # is still formally STOPPING/STOPPED (e.g. still inside the post-stop grace window, or a
    # held left blinker). Ported from cp on purpose, including the frame-level flicker this can
    # cause -- see spec section 8.2. Crucially this must NOT also force `_state` back to CRUISE:
    # in cp, this override only zeroes the local stop_model_x/actual_stop_distance for this
    # frame's distance math -- self.xState is untouched by it. The state machine block above is
    # the only thing allowed to change `_state`.
    if self._state == TrafficStopState.CRUISE:
      self._actual_stop_distance = 0.0
      self._reference_speed_kph = 0.0
      self._stopped_grace_frames = 0
      return TrafficStopResult(stop_dist_m=None, v_cruise_limited=None)

    # FIXED (verified against cp source): cp dead-reckons actual_stop_distance down by
    # v_ego*DT_MDL every single frame (not just while stopping), then clears it on release.
    self._actual_stop_distance = max(0.0, self._actual_stop_distance - v_ego * DT_MDL)

    instant_obstacle_release = traffic_state in (TrafficLightState.OFF, TrafficLightState.GREEN)
    if instant_obstacle_release:
      self._actual_stop_distance = 0.0
      return TrafficStopResult(stop_dist_m=None, v_cruise_limited=None)

    # FIXED (verified against cp source): while actively stopping, cp resyncs the rate-limiter's
    # internal state to the raw filtered value every frame (`self._stop_x_rl = stop_model_x_raw`),
    # which effectively hands authority for smoothing over to `_actual_stop_distance`'s
    # dead-reckoning + guarded re-latch instead. The rate limiter's own asymmetric "cap the
    # closing speed" behavior only actually matters while NOT stopping (i.e. it stays warmed up
    # and ready for the moment a new red is detected).
    self._stop_x_rl = stop_model_x_raw

    stop_model_x_contribution = 0.0 if self._actual_stop_distance > 0.0 else stop_model_x_rl
    stop_dist = max(0.0, stop_model_x_contribution + self._actual_stop_distance)
    # FIXED (verified against cp source): the adjust offset is applied exactly once. The original
    # port applied it here AND again in long_mpc.py, double-counting it.
    # The fixed camera-to-front-bumper baseline is always applied here, in addition to (not
    # instead of) the small UI fine-tune -- see the TRAFFIC_STOP_CAMERA_TO_FRONT_M comment above.
    stop_dist = get_traffic_stop_obstacle_distance(stop_dist, TRAFFIC_STOP_CAMERA_TO_FRONT_M + self._distance_adjust_m)

    # FIXED (verified against cp source): cp hard-forces v_cruise to 0 while fully stopped
    # (XState.e2eStopped), rather than relying on the soft sqrt formula the whole time.
    if self._state == TrafficStopState.STOPPED:
      v_cruise_limited = 0.0
    else:
      v_cruise_limited = None
      if stop_dist < 300.0:
        comfort_brake = DEFAULT_COMFORT_BRAKE * STOPPING_COMFORT_BRAKE_FACTOR
        stop_dist_soft = max(stop_dist - 1.0, 0.0)
        v_cruise_limited = (2 * comfort_brake * stop_dist_soft) ** 0.5
        v_cruise_limited = min(v_cruise_limited, v_ego)  # monotonic guard: never request more speed than current

    return TrafficStopResult(stop_dist_m=stop_dist, v_cruise_limited=v_cruise_limited)
