"""
Traffic light / stop sign virtual stop-line obstacle.

Ported from the carrot (cp) fork's traffic-stop mechanism into openpilot-dptest.
Pure-function layer is numerically verified against cp's official
`selfdrive/carrot/tests/test_traffic_stop.py`; the state machine is a
line-for-line port of cp's `carrot_functions.py` state transitions.

Integration divergence from cp (intentional, see traffic_stop_complete_guide_v2.md
section 9): cp disables this obstacle entirely in blended/e2e mode
(`if mode == 'blended': stop_x = 1000.0`). This port keeps the obstacle active
unconditionally across all longitudinal modes - dp's mode switching is not
trusted to always avoid blended mode approaching a red light. The final
accel candidate selection in selfdrive/controls/lib/longitudinal_planner.py
still takes min(mpc, e2e) in blended mode, so the e2e model's own stopping
judgement is never suppressed - this module (both the MPC obstacle and the
v_cruise floor below) only ever makes the mpc candidate *more* conservative,
never less; it can't remove a safety margin the e2e side would have applied.

Not ported in this pass (flagged for follow-up, see guide section 8.13):
the mode-switch "taper" fix for the e2e-candidate handoff blip. dp's outer
longitudinal_planner.py does take min(mpc, e2e) in blended mode, so the same
class of first-accelerate-then-brake blip is theoretically possible here too
if a mode switch happens while this module is actively braking. Not fixed in
this port - track separately if it reproduces on dp.

Integration is two layers, both active (redundant by design, same pattern cp
itself uses across lead0/lead1/cruise):

1. Primary - a real MPC obstacle. `self.stop_dist_m` is read by the outer
   selfdrive/controls/lib/longitudinal_planner.py and passed as
   `traffic_stop_obstacle_m` into LongitudinalMpc.update(), which adds it as a
   4th column in long_mpc.py's `x_obstacles` stack (see
   `build_traffic_stop_obstacle()`), right alongside lead0/lead1/cruise. This
   is what actually gives the smooth, precisely-timed brake curve - the
   solver treats the stop line like a parked car and plans the whole jerk-
   limited deceleration profile against it, the same way it already does for
   a real lead.
2. Secondary/redundant - LongitudinalPlannerDP.update_targets() (in
   longitudinal_planner.py) also adds `self.output_v_target` to the `targets`
   v/a-target min-selection dict alongside cruise/dtsc, so a soft v_cruise
   ceiling is enforced even on any path that doesn't consult the MPC's own
   obstacle list.
"""
from collections import deque

import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX

# ── state machine states ────────────────────────────────────────────────
CRUISE, STOPPING, STOPPED = 0, 1, 2
OFF, RED, GREEN = 0, 1, 2

# ── constants (traffic_stop_complete_guide_v2.md 第4節，逐項核對 cp 原始碼) ──
TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0

TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH = (0.0, 100.0)
TRAFFIC_STOP_DISTANCE_RATIO = (1.0, 0.7)
TRAFFIC_STOP_DISTANCE_FADE_BP_M = (0.0, 50.0)

COMFORT_BRAKE_BASE = 2.4          # m/s^2 - not 2.5, cp table 8.1
STOPPING_BRAKE_DISCOUNT = 0.9     # only applied while STOPPING, not STOPPED (8.2)

STOPPED_SPEED_MS = 0.3            # single-frame trigger, no debounce (8.6)
STOPPED_GRACE_FRAMES = round(0.5 / DT_MDL)       # 0.5s green cooldown after STOPPED
GREEN_CONFIRM_FRAMES = round(0.2 / DT_MDL)       # red has no debounce, green needs 4 frames @ 20Hz
GAS_SUPPRESS_FRAMES = round(10.0 / DT_MDL)       # 10s re-detect suppression after gas override

RECALIBRATE_MIN_DISTANCE_M = 10.0  # only re-lock the dead-reckoning accumulator above this

# 前車「取消」判斷門檻。cp 原始碼（selfdrive/carrot/carrot_functions.py）這裡固定是
# 2.0，且刻意跟「進入」判斷（只要有任何前車就阻擋，不比距離）用不同條件——這是 cp
# 的既有設計，不是本次移植的邊界情況。上一輪我誤把這個不對稱當成本模組自己引入的
# 安全性缺陷，改成「只要偵測到任何前車就無條件取消」，這其實引入了新的副作用：正在
# 進行中的停等，會因為雷達範圍內任何跟停止線無關的車（隔壁車道、路徑外）而被整個
# 取消掉。現在改回維持 cp 原本「距離門檻」判斷的架構，只把門檻值調大——原本 2.0m 太
# 緊會讓最終跟車距離卡在 2m 附近，調大到 4.0m 讓邊界情況下的最終距離更寬鬆，同時保留
# 「前車距離要夠接近虛擬停止點才視為同一個相關物體」這個判斷精神。實際最終跟車距離
# 還會疊加 long_mpc.py 的 get_safe_obstacle_distance()／LEAD_DANGER_FACTOR 這層跟
# 障礙物來源無關的舒適距離軟性懲罰項，兩層一起作用，需要路測驗證確切數字。
CANCEL_LEAD_MARGIN_M = 4.0

MEDIAN_WINDOW = 3
MOVING_AVG_WINDOW = 15
MODEL_V_WINDOW = 10

RATE_LIMIT_CLOSING_MARGIN_M = 0.5  # only limits approach speed; retreat is instant

STOP_SIGN_MAX_SPEED_KPH = 82.0
STOP_SIGN_DETECT_DIST_BP_KPH = (60.0, 80.0)
STOP_SIGN_DETECT_DIST_M = (120.0, 150.0)
STOP_SIGN_LATERAL_TOLERANCE_M = 5.0

# cp's default is a single unlabeled -1.5m physical (camera-to-bumper) offset
# mixed into the user-adjustable param. Per guide section 6/8.12 we split it:
# this constant is the fixed physical correction (not user-editable), and
# dp_lon_traffic_stop_distance_adjust_m (settings ITEMS, +/-5m, default 0) is
# the small user preference layered on top via get_traffic_stop_obstacle_distance().
CAMERA_TO_FRONT_M = -1.5

PARAM_ENABLED = "dp_lon_traffic_stop"
PARAM_DISTANCE_ADJUST = "dp_lon_traffic_stop_distance_adjust_m"
PARAM_POLL_FRAMES = round(1.0 / DT_MDL)  # 1s / 20 frames, per guide section 2a


# ── pure functions (verified against cp's official test_traffic_stop.py) ──

def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """Large steering angle => this is a turn, not an approach to a stop. Only
  blocks *new* entries into STOPPING; does not affect an already-active stop."""
  return abs(steering_angle_deg) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_reference_speed(v_ego_kph: float, previous_reference_kph: float | None) -> float:
  """Latches the highest v_ego seen during this stop event. Monotonic non-decreasing."""
  return max(0.0, v_ego_kph, previous_reference_kph or 0.0)


def get_virtual_traffic_stop_distance(model_distance: float, v_ego_kph: float) -> float:
  """Faster approach speed pulls the braking-start ratio in (100% @ 0kph, 70% @
  100kph), but the ratio fades back to 100% within the final 50m so the car
  actually stops at the right place regardless of approach speed."""
  distance_ratio = np.interp(v_ego_kph, TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH, TRAFFIC_STOP_DISTANCE_RATIO)
  applied_ratio = np.interp(model_distance, TRAFFIC_STOP_DISTANCE_FADE_BP_M, [1.0, distance_ratio])
  return max(0.0, model_distance * applied_ratio)


def get_traffic_stop_obstacle_distance(stop_distance: float, distance_adjust: float) -> float:
  """Apply the (fixed physical + user) distance correction. Call exactly once
  per pipeline pass - cp bug 8.3 was this being applied twice or not at all."""
  return max(0.0, stop_distance + distance_adjust)


def check_model_stopping(hist: deque, stop_sign_count: int, start_sign_count: int, state: int,
                          v_cruise: float, model_v_traj, v_ego: float, a_ego: float,
                          model_x_end: float, model_y_end: float, d_rel: float) -> tuple[int, int, int]:
  """Returns (traffic_state, new_stop_sign_count, new_start_sign_count)."""
  v_ego_kph = v_ego * 3.6

  hist.append(model_v_traj[-1])
  model_v = sum(hist) / len(hist)

  start_sign = model_v > 5.0 or model_v > (model_v_traj[0] + 2)

  if v_ego_kph < 1.0:
    stop_sign = model_x_end < 20.0 and model_v < 10.0
  elif v_ego_kph < STOP_SIGN_MAX_SPEED_KPH:
    max_detect_dist = np.interp(model_v_traj[0] * 3.6, STOP_SIGN_DETECT_DIST_BP_KPH, STOP_SIGN_DETECT_DIST_M)
    stop_sign = (
      model_x_end < d_rel - 3.0 and
      model_x_end < max_detect_dist and
      (model_v < 3.0 or model_v < model_v_traj[0] * 0.7) and
      abs(model_y_end) < STOP_SIGN_LATERAL_TOLERANCE_M
    )
    if v_cruise != 0 and state == CRUISE and a_ego < -1.0:
      stop_sign = False
  else:
    stop_sign = False

  stop_sign_count = stop_sign_count + 1 if stop_sign else 0
  start_sign_count = start_sign_count + 1 if (start_sign and not stop_sign) else 0

  if stop_sign_count * DT_MDL > 0.0:
    return RED, stop_sign_count, start_sign_count
  if start_sign_count * DT_MDL > 0.2:
    return GREEN, stop_sign_count, start_sign_count
  return OFF, stop_sign_count, start_sign_count


class TrafficStopController:
  """Stateful controller. Call update() once per planner cycle (20Hz)."""

  def __init__(self):
    self.params = Params()
    self.is_enabled = self.params.get_bool(PARAM_ENABLED)
    # dp: 這支 fork 的 Params class（common/params_pyx.pyx）只有 get()/get_bool()，
    # 沒有 get_int()——get() 會依 params_keys.h 裡登記的 ParamKeyType 自動轉型，
    # INT 類型的 key 會直接回傳 python int，不需要另外轉型或給預設值。
    self.distance_adjust_m = float(self.params.get(PARAM_DISTANCE_ADJUST))
    self._poll_frame = 0

    self.state = CRUISE
    self.stop_sign_count = 0
    self.start_sign_count = 0
    self.model_v_hist: deque = deque(maxlen=MODEL_V_WINDOW)

    # median(3) -> moving-average(15), never cleared across stop events (8.9)
    self._median_hist: deque = deque(maxlen=MEDIAN_WINDOW)
    self._avg_hist: deque = deque(maxlen=MOVING_AVG_WINDOW)
    self.stop_model_x_raw = 0.0
    self.stop_model_x_rl = 0.0

    self.reference_speed_kph = 0.0
    self.actual_stop_distance = 0.0
    self.gas_suppress_frames = 0
    self.stopped_grace_frames = 0

    self.stop_dist_m: float | None = None
    self.output_v_target = V_CRUISE_MAX
    self.output_a_target = 0.0

  def _poll_params(self):
    self._poll_frame += 1
    if self._poll_frame >= PARAM_POLL_FRAMES:
      self._poll_frame = 0
      self.is_enabled = self.params.get_bool(PARAM_ENABLED)
      adj = self.params.get(PARAM_DISTANCE_ADJUST)
      self.distance_adjust_m = float(np.clip(adj, -5, 5))

  def _reset(self):
    self.state = CRUISE
    self.stop_sign_count = 0
    self.start_sign_count = 0
    self.actual_stop_distance = 0.0
    self.reference_speed_kph = 0.0
    self.stopped_grace_frames = 0
    self.stop_dist_m = None
    self.output_v_target = V_CRUISE_MAX
    self.output_a_target = 0.0

  def _update_stop_model_x(self, raw_x: float, v_ego: float) -> tuple[float, float]:
    self._median_hist.append(raw_x)
    median_val = float(np.median(self._median_hist))
    self._avg_hist.append(median_val)
    stop_model_x_raw = float(np.mean(self._avg_hist))

    max_step = v_ego * DT_MDL + RATE_LIMIT_CLOSING_MARGIN_M
    if stop_model_x_raw < self.stop_model_x_rl:
      stop_model_x_rl = max(stop_model_x_raw, self.stop_model_x_rl - max_step)
    else:
      stop_model_x_rl = stop_model_x_raw  # retreating: no limit
    return stop_model_x_raw, stop_model_x_rl

  def update(self, model_v2, car_state, radar_state, v_ego: float, a_ego: float, v_cruise: float):
    self._poll_params()

    if not self.is_enabled:
      self._reset()
      return

    model_x_traj = model_v2.position.x
    model_y_traj = model_v2.position.y
    model_v_traj = model_v2.velocity.x
    if len(model_x_traj) < 2 or len(model_v_traj) == 0:
      self.output_v_target = V_CRUISE_MAX
      self.output_a_target = 0.0
      return
    model_x_end = model_x_traj[-1]

    lead = radar_state.leadOne
    lead_present = bool(lead.status)
    d_rel = lead.dRel if lead_present else 1000.0

    steering_angle_deg = car_state.steeringAngleDeg
    gas_pressed = car_state.gasPressed
    left_blinker = car_state.leftBlinker

    self.stop_model_x_raw, self.stop_model_x_rl = self._update_stop_model_x(model_x_traj[-2], v_ego)

    traffic_state, self.stop_sign_count, self.start_sign_count = check_model_stopping(
      self.model_v_hist, self.stop_sign_count, self.start_sign_count, self.state,
      v_cruise, model_v_traj, v_ego, a_ego, model_x_end, model_y_traj[-1], d_rel)

    if gas_pressed and self.state == STOPPING:
      self.gas_suppress_frames = GAS_SUPPRESS_FRAMES
    elif self.gas_suppress_frames > 0:
      self.gas_suppress_frames -= 1

    # dp 修正歷史：這裡曾經一度改成「只要偵測到任何前車就無條件取消」（lead_present）。
    # 後來確認 cp 原始碼本來就是用距離門檻判斷取消（CANCEL_LEAD_MARGIN_M，見上方常數
    # 註解），「無條件取消」並非 cp 設計、且會被雷達範圍內任何不相關的車誤觸發，已經
    # 改回維持 cp 原本的門檻架構，只調大門檻值。
    lead_cancels = lead_present and (d_rel - self.stop_model_x_raw) < CANCEL_LEAD_MARGIN_M

    if self.state == CRUISE:
      entry_allowed = is_traffic_stop_entry_allowed(steering_angle_deg)
      if (not lead_present and traffic_state == RED and
          entry_allowed and self.gas_suppress_frames == 0):
        self.state = STOPPING
        self.reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, None)
        self.actual_stop_distance = get_virtual_traffic_stop_distance(
          self.stop_model_x_rl, self.reference_speed_kph)

    elif self.state == STOPPING:
      if gas_pressed:
        self.state = CRUISE
      elif lead_cancels:
        self.state = CRUISE
      elif traffic_state == GREEN:
        self.state = CRUISE
      else:
        self.reference_speed_kph = get_traffic_stop_reference_speed(v_ego * 3.6, self.reference_speed_kph)
        candidate = get_virtual_traffic_stop_distance(self.stop_model_x_rl, self.reference_speed_kph)
        if candidate > RECALIBRATE_MIN_DISTANCE_M:
          self.actual_stop_distance = candidate
        if v_ego < STOPPED_SPEED_MS:
          self.state = STOPPED
          self.stopped_grace_frames = STOPPED_GRACE_FRAMES

    elif self.state == STOPPED:
      if gas_pressed:
        self.state = CRUISE
      elif lead_cancels:
        self.state = CRUISE
      else:
        if self.stopped_grace_frames == 0:
          if traffic_state == GREEN and not left_blinker:
            self.state = CRUISE
        self.stopped_grace_frames = max(0, self.stopped_grace_frames - 1)

    # obstacle release is independent of the state machine transition above -
    # must not share the same reset path (8.7)
    if self.state == CRUISE:
      self.actual_stop_distance = 0.0
      self.reference_speed_kph = 0.0
      self.stopped_grace_frames = 0
      self.stop_dist_m = None
      self.output_v_target = V_CRUISE_MAX
      self.output_a_target = 0.0
      return

    self.actual_stop_distance = max(0.0, self.actual_stop_distance - v_ego * DT_MDL)

    if traffic_state in (OFF, GREEN):
      self.actual_stop_distance = 0.0
      self.output_v_target = V_CRUISE_MAX
      self.output_a_target = 0.0
      return  # state does NOT revert here - only via the transitions above

    self.stop_model_x_rl = self.stop_model_x_raw  # actively stopping: force sync every frame

    contribution = 0.0 if self.actual_stop_distance > 0.0 else self.stop_model_x_rl
    stop_dist = max(0.0, contribution + self.actual_stop_distance)
    stop_dist = get_traffic_stop_obstacle_distance(stop_dist, CAMERA_TO_FRONT_M + self.distance_adjust_m)
    self.stop_dist_m = stop_dist

    if self.state == STOPPED:
      self.output_v_target = 0.0
      self.output_a_target = 0.0
    else:
      self.output_v_target = v_ego
      self.output_a_target = -COMFORT_BRAKE_BASE * STOPPING_BRAKE_DISCOUNT
      if stop_dist < 300.0:
        comfort_brake = COMFORT_BRAKE_BASE * STOPPING_BRAKE_DISCOUNT
        v_limited = (2 * comfort_brake * max(stop_dist - 1.0, 0.0)) ** 0.5
        self.output_v_target = min(v_limited, v_ego)
