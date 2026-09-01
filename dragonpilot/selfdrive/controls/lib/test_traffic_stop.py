import pytest

from dragonpilot.selfdrive.controls.lib.traffic_stop import (
  get_traffic_stop_obstacle_distance,
  get_traffic_stop_reference_speed,
  get_virtual_traffic_stop_distance,
  is_traffic_stop_entry_allowed,
)


# --- get_virtual_traffic_stop_distance: values lifted verbatim from cp's official
#     selfdrive/carrot/tests/test_traffic_stop.py (see traffic_stop_complete_guide_v2.md 第3節) ---
@pytest.mark.parametrize("model_distance, v_ego_kph, expected", [
  (100.0, 0.0, 100.0),
  (15.0, 100.0, 13.65),
  (110.0, 62.0, 89.54),
  (40.0, 62.0, 34.048),
  (20.0, 62.0, 18.512),
])
def test_get_virtual_traffic_stop_distance_matches_cp(model_distance, v_ego_kph, expected):
  assert get_virtual_traffic_stop_distance(model_distance, v_ego_kph) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize("stop_distance, distance_adjust, expected", [
  (100.0, -1.5, 98.5),
  (1.0, -1.5, 0.0),    # clamped at 0, never negative
  (0.0, -2.0, 0.0),
])
def test_get_traffic_stop_obstacle_distance_matches_cp(stop_distance, distance_adjust, expected):
  assert get_traffic_stop_obstacle_distance(stop_distance, distance_adjust) == pytest.approx(expected, abs=1e-3)


def test_get_traffic_stop_obstacle_distance_applied_once_ui_delta_is_exact():
  # regression for cp bug 8.3: the UI distance_adjust delta between two settings
  # must be exact, or it's being applied more than once somewhere in the pipeline.
  base = get_traffic_stop_obstacle_distance(50.0, 0.0)
  plus5 = get_traffic_stop_obstacle_distance(50.0, 5.0)
  assert plus5 - base == pytest.approx(5.0, abs=1e-6)


def test_entry_steering_limit():
  assert is_traffic_stop_entry_allowed(0.0)
  assert is_traffic_stop_entry_allowed(49.9)
  assert not is_traffic_stop_entry_allowed(50.0)
  assert not is_traffic_stop_entry_allowed(-65.0)


def test_reference_speed_latches_to_max_only():
  # monotonic non-decreasing: it only ever latches UP during a stop event
  assert get_traffic_stop_reference_speed(30.0, None) == 30.0
  assert get_traffic_stop_reference_speed(20.0, 30.0) == 30.0  # dip doesn't lower it
  assert get_traffic_stop_reference_speed(45.0, 30.0) == 45.0  # new peak does raise it


def test_virtual_stop_distance_fades_back_to_full_ratio_near_stop_line():
  # within the final 50m the ratio must fade back toward 1.0 regardless of
  # approach speed, so the car actually stops at the correct location (cp table 4)
  near = get_virtual_traffic_stop_distance(1.0, 100.0)
  assert near == pytest.approx(1.0, abs=0.05)


def test_virtual_stop_distance_never_negative():
  assert get_virtual_traffic_stop_distance(0.0, 50.0) == 0.0


# --- MPC obstacle-array construction (selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py) ---
# Kept in this file for now since it's tightly coupled to TrafficStopController.stop_dist_m's
# contract (None == no active stop). Requires the real long_mpc.py module (needs a compiled
# acados solver to import) - skipped automatically if that import fails, e.g. in CI/dev
# environments without the full acados build.
def test_traffic_stop_obstacle_array_feeds_mpc():
  long_mpc = pytest.importorskip(
    "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc",
    reason="requires the compiled acados solver extension")
  import numpy as np

  disabled = long_mpc.build_traffic_stop_obstacle(None)
  assert len(disabled) == long_mpc.N + 1
  assert (disabled == long_mpc.TRAFFIC_STOP_OBSTACLE_DISABLED_M).all()

  active = long_mpc.build_traffic_stop_obstacle(12.3)
  assert len(active) == long_mpc.N + 1
  assert (active == 12.3).all()

  # disabled sentinel must never spuriously win against a realistic cruise obstacle
  cruise_like = np.full(long_mpc.N + 1, 40.0)
  x_obstacles = np.column_stack([cruise_like, disabled])
  assert np.argmin(x_obstacles[0]) == 0

  # an active, closer stop must correctly become the binding obstacle
  x_obstacles2 = np.column_stack([cruise_like, active])
  assert np.argmin(x_obstacles2[0]) == 1
  assert np.min(x_obstacles2, axis=1)[0] == pytest.approx(12.3)


# --- lead-cancel margin (state machine level, needs the real Params/DT_MDL/V_CRUISE_MAX
# import chain - skipped automatically without the compiled extension, same as above) ---
#
# cp's own carrot_functions.py deliberately uses a distance-threshold cancel condition here
# (not "any lead cancels") - see lead_cancel_margin_guide.md. An earlier revision of this
# module briefly "fixed" this into an unconditional cancel, which is NOT what cp does and
# introduces its own new failure mode (an unrelated nearby vehicle - different lane, outside
# the path - could cancel an in-progress stop). These tests lock in the correct, restored
# threshold-based behavior at CANCEL_LEAD_MARGIN_M's boundary.
def test_lead_cancel_margin_boundary():
  ts = pytest.importorskip(
    "dragonpilot.selfdrive.controls.lib.traffic_stop",
    reason="requires the compiled Params extension")
  import types

  def model_v2(x, y, v):
    return types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y), velocity=types.SimpleNamespace(x=v))

  def car_state():
    return types.SimpleNamespace(steeringAngleDeg=0.0, gasPressed=False, leftBlinker=False)

  def radar_state(d_rel):
    return types.SimpleNamespace(leadOne=types.SimpleNamespace(status=True, dRel=d_rel))

  def stopping_ctrl(stop_model_x_rl):
    ctrl = ts.TrafficStopController()
    ctrl.is_enabled = True
    ctrl.state = ts.STOPPING
    ctrl.stop_model_x_rl = stop_model_x_rl
    ctrl.stop_model_x_raw = stop_model_x_rl
    ctrl.actual_stop_distance = 0.0
    ctrl.reference_speed_kph = 20.0
    return ctrl

  model_x = [20.0] * 33
  model_v = [2.0] * 33
  model_y = [0.0] * 33

  # lead just inside the margin -> must cede control back to normal lead-follow
  ctrl_in = stopping_ctrl(20.0)
  ctrl_in.update(model_v2(model_x, model_y, model_v), car_state(), radar_state(20.0 + 3.0), 5.0, -0.5, 15.0)
  assert ctrl_in.state == ts.CRUISE
  assert ctrl_in.stop_dist_m is None

  # lead beyond the margin -> module correctly keeps managing its own stop (cp's actual design)
  ctrl_out = stopping_ctrl(20.0)
  ctrl_out.update(model_v2(model_x, model_y, model_v), car_state(), radar_state(20.0 + 6.0), 5.0, -0.5, 15.0)
  assert ctrl_out.state == ts.STOPPING
  assert ctrl_out.stop_dist_m is not None
