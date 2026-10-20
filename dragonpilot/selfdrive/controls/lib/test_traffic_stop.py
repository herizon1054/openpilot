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
  # 18 km/h、距線約 18 m 尚未到必須煞車點：MPC 障礙物被閘門擋下，但 AEM 用的停止線距離仍在
  assert ctrl_out.near_stop_dist_m is not None


# --- MPC 障礙物閘門 + 鎖存（dp0111pre 整合修正，cp 原版沒有這個閘門）---
def _gate_setup():
  ts = pytest.importorskip(
    "dragonpilot.selfdrive.controls.lib.traffic_stop",
    reason="requires the compiled Params extension")
  import types

  def step(ctrl, d, v):
    xs = [d * i / 32 for i in range(33)]
    vs = [v] + [max(v * (1 - i / 32), 0.3) for i in range(1, 33)]
    ctrl.update(types.SimpleNamespace(position=types.SimpleNamespace(x=xs, y=[0.0] * 33),
                                      velocity=types.SimpleNamespace(x=vs)),
                types.SimpleNamespace(steeringAngleDeg=0.0, gasPressed=False, leftBlinker=False),
                types.SimpleNamespace(leadOne=types.SimpleNamespace(status=False, dRel=0.0)),
                v, 0.0, 30.0)
  return ts, step


def test_obstacle_gated_until_must_brake():
  ts, step = _gate_setup()
  ctrl = ts.TrafficStopController()
  ctrl.is_enabled = True
  # 30 km/h、距線 60 m：還不需要煞車 -> 不給 MPC 障礙物，但 AEM 距離存在
  for _ in range(20):
    step(ctrl, 60.0, 30 / 3.6)
  assert ctrl.state == ts.STOPPING
  assert ctrl.stop_dist_m is None
  assert ctrl.near_stop_dist_m is not None


def test_obstacle_latched_after_must_brake():
  ts, step = _gate_setup()
  ctrl = ts.TrafficStopController()
  ctrl.is_enabled = True
  for _ in range(20):
    step(ctrl, 60.0, 50 / 3.6)
  # 50 km/h 接近到 30 m：已達必須煞車點 -> 障礙物啟用
  for _ in range(20):
    step(ctrl, 30.0, 50 / 3.6)
  assert ctrl.stop_dist_m is not None
  # 開始煞車後車速大降（低於 v_release），障礙物仍須保持，不可被放掉
  for _ in range(5):
    step(ctrl, 25.0, 10 / 3.6)
  assert ctrl.state == ts.STOPPING
  assert ctrl.stop_dist_m is not None


def test_user_distance_adjust_sign_matches_settings_description():
  # 設定說明：正值 = 停更後面（障礙物更近），負值 = 停更前面（障礙物更遠）
  ts, step = _gate_setup()
  dists = {}
  for adj in (-2.0, 0.0, 2.0):
    ctrl = ts.TrafficStopController()
    ctrl.is_enabled = True
    ctrl.distance_adjust_m = adj
    ctrl._poll_frame = -10_000   # 測試期間不重新讀 Params 覆蓋 distance_adjust_m
    for _ in range(20):
      step(ctrl, 60.0, 30 / 3.6)
    dists[adj] = ctrl.near_stop_dist_m
  assert dists[2.0] == pytest.approx(dists[0.0] - 2.0, abs=1e-3)
  assert dists[-2.0] == pytest.approx(dists[0.0] + 2.0, abs=1e-3)


def test_obstacle_dead_reckons_when_model_flickers_off():
  # M5：STOPPING 中模型短暫沒判紅燈（OFF）時，障礙物不可凍結（否則停止線跟著車跑），
  # 也不可拿掉（否則 MPC 往前衝），而是以車速往前扣距離。
  ts, step = _gate_setup()
  import types

  def step_off(ctrl, v):
    # 模型終點 200 m 外（超出偵測距離 -> 非停車），終點速度 3 m/s（< 5，不構成起步）-> OFF
    xs = [200.0 * i / 32 for i in range(33)]
    vs = [v] + [3.0] * 32
    ctrl.update(types.SimpleNamespace(position=types.SimpleNamespace(x=xs, y=[0.0] * 33),
                                      velocity=types.SimpleNamespace(x=vs)),
                types.SimpleNamespace(steeringAngleDeg=0.0, gasPressed=False, leftBlinker=False),
                types.SimpleNamespace(leadOne=types.SimpleNamespace(status=False, dRel=0.0)),
                v, 0.0, 30.0)

  ctrl = ts.TrafficStopController()
  ctrl.is_enabled = True
  ctrl._poll_frame = -10_000
  for _ in range(20):
    step(ctrl, 30.0, 50 / 3.6)          # 50 km/h、距線 30 m：已鎖存障礙物
  assert ctrl.state == ts.STOPPING and ctrl.stop_dist_m is not None
  v = 30 / 3.6
  before = ctrl.stop_dist_m
  for _ in range(4):                    # 4 幀 OFF（未達 GREEN 確認，狀態不變）
    step_off(ctrl, v)
  assert ctrl.state == ts.STOPPING
  assert ctrl.stop_dist_m is not None                                   # 沒被拿掉
  assert ctrl.stop_dist_m == pytest.approx(before - 4 * v * ts.DT_MDL, abs=1e-3)  # 有往前扣
  assert ctrl.near_stop_dist_m == pytest.approx(ctrl.stop_dist_m, abs=1e-6)
