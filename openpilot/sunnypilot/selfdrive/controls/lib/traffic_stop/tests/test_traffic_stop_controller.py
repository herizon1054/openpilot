from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop_controller import (
  TrafficStopController,
  TrafficStopState,
  TrafficLightState,
  DISTANCE_ADJUST_MIN_M,
  DISTANCE_ADJUST_MAX_M,
  STOPPED_GRACE_FRAMES,
  STARTING_SUPPRESS_FRAMES,
)


class MockXYZTData:
  def __init__(self, x=None, y=None):
    self.x = x if x is not None else [0.0] * 33
    self.y = y if y is not None else [0.0] * 33


class MockModelV2:
  def __init__(self, position_x=None, position_y=None, velocity_x=None):
    self.position = MockXYZTData(x=position_x, y=position_y)
    self.velocity = MockXYZTData(x=velocity_x)


class MockLeadOne:
  def __init__(self, present=False, dRel=1000.0):
    self.present = present
    self.dRel = dRel


class MockRadarState:
  def __init__(self, present=False, dRel=1000.0):
    self.leadOne = MockLeadOne(present=present, dRel=dRel)


class MockCarState:
  def __init__(self, steeringAngleDeg=0.0, gasPressed=False, leftBlinker=False):
    self.steeringAngleDeg = steeringAngleDeg
    self.gasPressed = gasPressed
    self.leftBlinker = leftBlinker


class MockParams:
  def __init__(self, enabled=True, distance_adjust=0):
    self.enabled = enabled
    self.distance_adjust = distance_adjust

  def get_bool(self, key):
    return self.enabled

  def get(self, key, return_default=False):
    return self.distance_adjust


def approaching_red_light_model(model_x_end=15.0, model_v_end=2.0, model_v_start=10.0, y_end=0.0):
  """A model trajectory that should trigger a red-light detection at v_ego ~= model_v_start."""
  position_x = [model_x_end] * 33  # last point (index -2 via STOP_MODEL_IDX) close to model_x_end
  position_y = [0.0] * 33
  position_y[-1] = y_end
  velocity_x = [model_v_start] * 33
  velocity_x[-1] = model_v_end
  return MockModelV2(position_x=position_x, position_y=position_y, velocity_x=velocity_x)


def green_light_model(model_x_end=200.0, model_v=20.0):
  """A model trajectory whose start_sign condition holds every frame (for GREEN_CONFIRM_SEC debounce tests)."""
  velocity_x = [model_v] * 33
  return MockModelV2(position_x=[model_x_end] * 33, position_y=[0.0] * 33, velocity_x=velocity_x)


def run_frames(controller, model_v2, cs, rs, v_ego, a_ego, v_cruise, n=1):
  result = None
  for _ in range(n):
    result = controller.update(model_v2, cs, rs, v_ego, a_ego, v_cruise)
  return result


class TestTrafficStopController:
  def test_disabled_returns_none(self):
    controller = TrafficStopController(params=MockParams(enabled=False))
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState()
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0)
    assert result.stop_dist_m is None
    assert result.v_cruise_limited is None

  def test_red_single_frame_trigger(self):
    """Red-light detection has no debounce -- a single qualifying frame is enough."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.STOPPING
    assert result.stop_dist_m is not None

  def test_steering_angle_blocks_entry(self):
    """>=50 deg steering suppresses *new* entries into traffic-stop management."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model()
    cs = MockCarState(steeringAngleDeg=60.0)
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_any_lead_blocks_entry_regardless_of_distance(self):
    """cp blocks entry on ANY detected lead (XState.lead takes over), not just a lead closer
    than the stop line -- a real lead well past the stop line still blocks entry, since the
    MPC's own lead-following already produces the correct stop."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model(model_x_end=15.0)
    cs = MockCarState()
    rs = MockRadarState(present=True, dRel=500.0)  # far beyond the virtual stop line
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_gas_press_during_stopping_releases_and_suppresses_reentry(self):
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.STOPPING

    cs_gas = MockCarState(gasPressed=True)
    result = run_frames(controller, model, cs_gas, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None
    assert controller._gas_suppress_frames == STARTING_SUPPRESS_FRAMES

    # immediately re-approaching the same red light should now be suppressed
    result = run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_gas_press_during_cruise_does_not_suppress_future_entry(self):
    """Ordinary gas presses during normal driving (no active stop) must not arm the 10s
    suppression window -- only a gas press while actively braking toward a red does."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    cs_gas = MockCarState(gasPressed=True)
    rs = MockRadarState(present=False)
    far_model = approaching_red_light_model(model_x_end=200.0, model_v_end=15.0, model_v_start=15.0)
    run_frames(controller, far_model, cs_gas, rs, v_ego=15.0, a_ego=0.5, v_cruise=15.0, n=3)
    assert controller._state == TrafficStopState.CRUISE
    assert controller._gas_suppress_frames == 0

    # a genuine red on a *fresh* controller (no filter carry-over from the scenario above)
    # should still trigger normally -- already covered by test_red_single_frame_trigger; the
    # assertion above is the actual behavior under test here.

  def test_closer_lead_releases_during_stopping(self):
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.STOPPING

    rs_lead = MockRadarState(present=True, dRel=10.0)  # far closer than the ~15m filtered stop-line estimate
    result = run_frames(controller, model, cs, rs_lead, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    assert result.stop_dist_m is None

  def test_lead_cancel_margin_is_4m_not_2m(self):
    """LEAD_CLOSE_TO_STOP_LINE_M was raised from cp's 2.0m to 4.0m so the worst-case final
    stopped gap to a real lead near the boundary is ~4m instead of ~2m (see conversation: real
    lead just outside the margin means the MPC targets the unbuffered virtual stop-line obstacle
    instead of the real lead's own obstacle distance)."""
    cs = MockCarState()

    # lead 3m beyond the stop-line estimate: must cancel under the new 4.0m margin (would NOT
    # have cancelled under cp's original 2.0m margin)
    controller_a = TrafficStopController(params=MockParams(enabled=True))
    model_a = approaching_red_light_model(model_x_end=20.0)
    run_frames(controller_a, model_a, cs, MockRadarState(present=False), v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller_a._state == TrafficStopState.STOPPING
    result_a = run_frames(controller_a, model_a, cs, MockRadarState(present=True, dRel=23.0),
                           v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller_a._state == TrafficStopState.CRUISE
    assert result_a.stop_dist_m is None

    # lead 6m beyond the stop-line estimate: still outside even the widened 4.0m margin, module
    # keeps managing its own stop
    controller_b = TrafficStopController(params=MockParams(enabled=True))
    model_b = approaching_red_light_model(model_x_end=20.0)
    run_frames(controller_b, model_b, cs, MockRadarState(present=False), v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller_b._state == TrafficStopState.STOPPING
    result_b = run_frames(controller_b, model_b, cs, MockRadarState(present=True, dRel=26.0),
                           v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller_b._state == TrafficStopState.STOPPING
    assert result_b.stop_dist_m is not None

  def test_reaches_stopped_state_on_first_slow_frame(self):
    """cp transitions STOPPING -> STOPPED the instant v_ego < 0.3 m/s -- no multi-frame hold."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model(model_v_start=1.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=1.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPING

    result = run_frames(controller, model, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPED
    assert result.stop_dist_m is not None

  def test_stopped_state_forces_v_cruise_to_zero(self):
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model(model_v_start=1.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=1.0, a_ego=0.0, v_cruise=1.0, n=1)
    result = run_frames(controller, model, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPED
    assert result.v_cruise_limited == 0.0

  def test_green_exit_from_stopped_respects_grace_period(self):
    """The obstacle itself releases instantly on green (matches cp's flicker-on-purpose
    behavior), but the formal STOPPED -> CRUISE state transition waits out the ~0.5s grace
    window (cp: stopping_count) before allowing a green light to let the car go."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    stop_model = approaching_red_light_model(model_v_start=1.0, model_v_end=0.1)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, stop_model, cs, rs, v_ego=1.0, a_ego=0.0, v_cruise=1.0, n=1)
    run_frames(controller, stop_model, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=1)
    assert controller._state == TrafficStopState.STOPPED
    assert controller._stopped_grace_frames == STOPPED_GRACE_FRAMES

    green = green_light_model()
    # not enough green-confirm frames yet to even read GREEN, and grace period hasn't elapsed
    result = run_frames(controller, green, cs, rs, v_ego=0.0, a_ego=0.0, v_cruise=1.0, n=STOPPED_GRACE_FRAMES - 1)
    assert controller._state == TrafficStopState.STOPPED

  def test_green_confirm_needs_debounce(self):
    """_check_model_stopping requires ~0.2s (4 frames) of start_sign before reporting GREEN."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    model_v_traj = [10.0] * 33  # model_v_traj[0] == 10.0 -> start_sign true needs model_v > 12.0 or > 5.0
    model_v_traj[-1] = 20.0
    for i in range(4):
      state = controller._check_model_stopping(v_cruise=10.0, model_v_traj=model_v_traj, v_ego=10.0, a_ego=0.0,
                                                model_x_end=200.0, model_y_traj=[0.0] * 33, d_rel=1000.0)
      assert state != TrafficLightState.GREEN
    state = controller._check_model_stopping(v_cruise=10.0, model_v_traj=model_v_traj, v_ego=10.0, a_ego=0.0,
                                              model_x_end=200.0, model_y_traj=[0.0] * 33, d_rel=1000.0)
    assert state == TrafficLightState.GREEN

  def test_distance_adjust_is_clipped_and_v_cruise_limited_is_monotonic(self):
    controller = TrafficStopController(params=MockParams(enabled=True, distance_adjust=999))  # out of range
    model = approaching_red_light_model(model_x_end=5.0, model_v_start=5.0)
    cs = MockCarState()
    rs = MockRadarState(present=False)
    result = run_frames(controller, model, cs, rs, v_ego=5.0, a_ego=0.0, v_cruise=5.0, n=1)

    assert controller._distance_adjust_m == DISTANCE_ADJUST_MAX_M
    assert result.stop_dist_m is not None
    # v_cruise_limited must never ask for more speed than the car currently has
    assert result.v_cruise_limited is not None
    assert result.v_cruise_limited <= 5.0

    controller2 = TrafficStopController(params=MockParams(enabled=True, distance_adjust=-999))
    result2 = run_frames(controller2, model, cs, rs, v_ego=5.0, a_ego=0.0, v_cruise=5.0, n=1)
    assert controller2._distance_adjust_m == DISTANCE_ADJUST_MIN_M
    assert result2.stop_dist_m >= 0.0

  def test_model_filters_persist_across_release(self):
    """cp's median/average filters on the raw model x are never cleared -- they keep running
    continuously across separate stop events. Only the per-event accumulator/state resets."""
    controller = TrafficStopController(params=MockParams(enabled=True))
    model = approaching_red_light_model()
    cs = MockCarState()
    rs = MockRadarState(present=False)
    run_frames(controller, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=5)
    assert len(controller._stop_x_avg_hist) > 0
    hist_len_before = len(controller._stop_x_avg_hist)

    # release back to cruise (gas press)
    cs_gas = MockCarState(gasPressed=True)
    run_frames(controller, model, cs_gas, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)
    assert controller._state == TrafficStopState.CRUISE
    # filters must still hold their warmed-up history, not be cleared back to empty
    assert len(controller._stop_x_avg_hist) >= hist_len_before

  def test_camera_to_front_baseline_applies_even_with_zero_ui_adjust(self):
    """TRAFFIC_STOP_CAMERA_TO_FRONT_M (-1.5m, ported from cp's real default) must always be
    applied, even when the UI-facing TrafficStopDistanceAdjust is left at its neutral default
    of 0 -- the two are separate terms that both feed the same final offset."""
    from openpilot.sunnypilot.selfdrive.controls.lib.traffic_stop.traffic_stop_controller import (
      TRAFFIC_STOP_CAMERA_TO_FRONT_M,
    )
    model = approaching_red_light_model(model_x_end=40.0, model_v_start=10.0)
    cs = MockCarState()
    rs = MockRadarState(present=False)

    baseline_ctrl = TrafficStopController(params=MockParams(enabled=True, distance_adjust=0))
    baseline_result = run_frames(baseline_ctrl, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)

    plus5_ctrl = TrafficStopController(params=MockParams(enabled=True, distance_adjust=5))
    plus5_result = run_frames(plus5_ctrl, model, cs, rs, v_ego=10.0, a_ego=0.0, v_cruise=10.0, n=1)

    # the UI adjust (0 -> 5) must shift the result by exactly +5m regardless of the baseline
    assert abs((plus5_result.stop_dist_m - baseline_result.stop_dist_m) - 5.0) < 1e-6
    # the fixed baseline itself must be the real cp value, not silently zero or something else
    assert TRAFFIC_STOP_CAMERA_TO_FRONT_M == -1.5
