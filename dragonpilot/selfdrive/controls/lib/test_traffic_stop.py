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
