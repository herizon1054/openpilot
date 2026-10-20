"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
Adapted for dragonpilot (Metric Version - Updated Limits)

dp(L3 修正)：
  原本 update_lane_turn() 的「前方車道線不可全部 > 0.5」條件，呼叫端（desire_helper.py）
  從未傳入 lane_line_probs，條件恆成立，結果是低於門檻車速時「只要打方向燈就送 turnLeft/turnRight」，
  塞車換道、停等紅燈打燈都會觸發，而且優先於換道 desire。
  現在：
    - 呼叫端傳入本車道左右線機率 [laneLineProbs[1], laneLineProbs[2]]。
    - 任一條 <= LANE_WEAK_PROB 視為「已進入路口／車道線消失」，才允許送轉彎 desire；
      兩條都清楚時代表仍在一般車道上，交給換道邏輯。
    - 鎖存：一旦送出轉彎 desire，只要同側方向燈持續、車速仍在門檻內、該側無盲點，就維持，
      避免車道線機率在門檻附近跳動造成 desire 逐幀開關。
    - 路緣偵測（blindspot_left/right 參數內含 road edge）維持原本作為阻擋條件，未改動。
    - 未傳入 lane_line_probs（None 或長度不足）時維持舊行為（條件成立），避免其他呼叫端行為改變。
"""
from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params

# 最大值改為 30 km/h
LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS

# dp(L3): 本車道線「變弱」門檻，沿用原設計的 0.5
LANE_WEAK_PROB = 0.5


class LaneTurnController:
  def __init__(self):
    self.turn_desire = log.Desire.none
    self.params = Params()
    self.param_read_counter = 0
    self.enabled = False
    # 預設值改為 20 km/h
    self.lane_turn_value = 20.0 * CV.KPH_TO_MS
    self._latched = log.Desire.none   # dp(L3): 目前鎖存中的轉彎方向
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

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool, left_blinker: bool, right_blinker: bool, v_ego: float, lane_line_probs: list[float] = None) -> None:
    """
    lane_line_probs：本車道左右線機率 [laneLineProbs[1], laneLineProbs[2]]（dp L3）。
                     任一條 <= LANE_WEAK_PROB 即視為已進入路口，允許送轉彎 desire。
                     None 或長度不足時視為條件成立（舊行為）。
    """
    if lane_line_probs is not None and len(lane_line_probs) >= 2:
      lane_weak = min(lane_line_probs[0], lane_line_probs[1]) <= LANE_WEAK_PROB
    else:
      lane_weak = True

    speed_ok = v_ego < self.lane_turn_value
    want_left = left_blinker and not right_blinker and speed_ok and not blindspot_left
    want_right = right_blinker and not left_blinker and speed_ok and not blindspot_right

    if want_left and (lane_weak or self._latched == log.Desire.turnLeft):
      self.turn_desire = log.Desire.turnLeft
    elif want_right and (lane_weak or self._latched == log.Desire.turnRight):
      self.turn_desire = log.Desire.turnRight
    else:
      self.turn_desire = log.Desire.none

    # 鎖存只在功能開啟時累積，關閉時清除（get_turn_desire 關閉時回傳 none）
    self._latched = self.turn_desire if self.enabled else log.Desire.none

  def get_turn_desire(self):
    if not self.enabled:
      return log.Desire.none
    return self.turn_desire
