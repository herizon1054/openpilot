"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
Adapted for dragonpilot (Metric Version - Updated Limits)
"""
from enum import IntEnum
from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params

# 最大值改為 30 km/h
LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS

# dp fork divergence：除了方向盤出力，還要求方向盤實際轉角超過這個門檻
# （跟 steeringTorque 同一套正負號慣例：正=向左，負=向右）才算確認。
STEERING_ANGLE_CONFIRM_DEG = 20.0


class LaneTurnState(IntEnum):
  # dp fork divergence：新增的三段式狀態機，讓 LTD 的出力確認方式
  # 跟 desire_helper.py 裡 LCA 的 preLaneChange -> laneChangeStarting
  # 完全對應，而不是打方向燈就自動觸發。
  off = 0        # 沒有在等待，也沒有在轉彎
  pending = 1    # 方向燈已打開、條件符合，等待駕駛出力確認（等同 LCA 的 preLaneChange）
  confirmed = 2  # 駕駛已出力確認，持續下 turn desire（等同 LCA 的 laneChangeStarting）


class LaneTurnController:
  def __init__(self):
    self.turn_desire = log.Desire.none
    self.turn_state = LaneTurnState.off
    self.turn_direction = None  # 'left' / 'right' / None，記錄目前 pending/confirmed 的方向
    self.params = Params()
    self.param_read_counter = 0
    self.enabled = False
    # 預設值改為 20 km/h
    self.lane_turn_value = 20.0 * CV.KPH_TO_MS
    self.read_params()

  def read_params(self):
    self.enabled = self.params.get_bool("dp_lane_turn_desire")
    val = self.params.get("dp_lane_turn_value")
    value = float(val if val is not None else 20.0)
    self.lane_turn_value = min(float(LANE_CHANGE_SPEED_MIN), value * CV.KPH_TO_MS)

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool,
                        v_ego: float, steering_pressed: bool, steering_torque: float, steering_angle_deg: float,
                        lane_line_probs: list[float] = None) -> None:
    """
    lane_line_probs: 預期傳入前方 5 秒內車道線的機率列表。
                     (若 Openpilot modelV2 原始輸出為 10 秒 / 33 個點，
                      請在呼叫端傳入前半段，例如 lane_line_probs[:16])

    dp fork divergence：跟 desire_helper.py 裡 LCA 的作動方式對齊——
    打方向燈只會進入 pending（等待）狀態，實際送出 turnLeft/turnRight
    desire 前，需要駕駛在方向盤上主動出力確認（steeringPressed +
    對應方向的 steeringTorque 正負號）。在此之上再加一道：方向盤實際
    轉角也要超過 ±STEERING_ANGLE_CONFIRM_DEG（預設 10 度），兩個條件
    都成立才會進入 confirmed 狀態——單純出力但輪子還沒真的轉過去不算數，
    避免只是手扶著方向盤、還沒真的打算轉時就被誤判成確認。
    這樣即使 road edge / 盲區偵測沒抓到真正的邊緣，駕駛仍握有最後一道確認關卡。
    """
    # 運作條件：前方路線 5 秒內車道線不可以「全部」大於 0.5
    # (只要有任何一個點的機率低於或等於 0.5，條件即成立，允許運作)
    lane_condition_met = True
    if lane_line_probs:
        lane_condition_met = not all(prob > 0.5 for prob in lane_line_probs)

    # 綜合判斷條件（不含出力確認，出力確認在下面狀態機另外處理）
    turn_allowed = (v_ego < self.lane_turn_value) and lane_condition_met

    direction = None
    if left_blinker and not right_blinker:
      direction = 'left'
    elif right_blinker and not left_blinker:
      direction = 'right'

    blindspot_detected = (blindspot_left and direction == 'left') or (blindspot_right and direction == 'right')

    # 跟 desire_helper.py 的 torque_applied 判斷方式完全一樣：
    # 需要駕駛正在出力（steeringPressed），且出力方向要跟轉彎方向一致
    # （正 torque 對應向左，負 torque 對應向右）。
    torque_applied = steering_pressed and (
      (steering_torque > 0 and direction == 'left') or
      (steering_torque < 0 and direction == 'right')
    )

    # dp fork divergence：額外要求方向盤實際轉角超過門檻（同樣正=左、負=右），
    # 跟 torque_applied 是 AND 的關係，兩個都成立才算駕駛真的確認要轉彎。
    angle_applied = (
      (steering_angle_deg > STEERING_ANGLE_CONFIRM_DEG and direction == 'left') or
      (steering_angle_deg < -STEERING_ANGLE_CONFIRM_DEG and direction == 'right')
    )
    confirm_applied = torque_applied and angle_applied

    if self.turn_state == LaneTurnState.off:
      if direction is not None and turn_allowed and not blindspot_detected:
        self.turn_state = LaneTurnState.pending
        self.turn_direction = direction

    elif self.turn_state == LaneTurnState.pending:
      if direction != self.turn_direction or not turn_allowed:
        # 方向燈關掉、換邊、或不再符合轉彎條件（例如車速超過門檻）就取消
        self.turn_state = LaneTurnState.off
        self.turn_direction = None
      elif not blindspot_detected and confirm_applied:
        # 邊緣/盲區已經解除，且駕駛出力+轉角都確認 -> 才真的進入 confirmed
        self.turn_state = LaneTurnState.confirmed
      # 邊緣/盲區還在偵測到的情況下，即使出力也先不確認，繼續停在 pending 等待

    elif self.turn_state == LaneTurnState.confirmed:
      if direction != self.turn_direction or not turn_allowed:
        # 跟 LCA 的 laneChangeStarting 一樣，一旦確認開始執行，
        # 不會因為之後又偵測到盲區/邊緣而中途取消，只有方向燈關掉
        # 或條件不再成立（例如超速）才會結束。
        self.turn_state = LaneTurnState.off
        self.turn_direction = None

    if self.turn_state == LaneTurnState.confirmed:
      self.turn_desire = log.Desire.turnLeft if self.turn_direction == 'left' else log.Desire.turnRight
    else:
      self.turn_desire = log.Desire.none

  def get_turn_desire(self):
    if not self.enabled:
      return log.Desire.none
    return self.turn_desire
