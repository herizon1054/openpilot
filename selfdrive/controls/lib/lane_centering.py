# 移植自 StarPilot (firestar5683/StarPilot) 的車道置中 (Lane Centering) 控制器。
#
# 來源比對基準：
#   - StarPilot: selfdrive/controls/lib/lane_centering.py
#   - 參考 PR（sunnypilot 分支上的等價移植）：
#       dsa302010/openpilot@96bc5a77dfbbe5df7bb05f34ac11557c86a3d0b8
#       "controls: port confidence-gated lane centering from StarPilot"
#
# 本檔案為「純演算法」層，刻意保持與 cp（StarPilot）100% 邏輯一致、不做任何
# openpilot-dplcc 專屬修改，方便未來 StarPilot 上游更新時直接比對/重新移植。
# dplcc 專屬的 dp_ 參數讀取、啟用條件等 glue 邏輯，請見：
#   dragonpilot/selfdrive/controls/lib/dp_lane_centering.py
#
# 演算法概述：
#   利用 modelV2 的左右車道線 (laneLines[1], laneLines[2]) 與模型路徑
#   (position)，在依車速決定的前視距離 (lookahead) 上，計算「車道中心」與
#   「模型路徑」的橫向誤差，轉換成一個曲率修正量，疊加到原本的
#   model_curvature 上。修正量會：
#     - 要求兩側車道線機率/標準差達到信心門檻，否則直接回退為 0（不介入）
#     - 進入/離開時以一階低通 (smooth_value) 平滑，避免方向盤突兀跳動
#     - 可設定車道內的置中偏移量 (offset)，並依車道寬度自動限縮到安全範圍
#     - 可依「端到端 (e2e) 模型路徑」的信心程度，讓模型路徑逐漸取得主導權
#       （e2e_authority 越高、模型路徑標準差越小，車道置中修正量會被壓低）
#     - 變換車道 (laneChangeState != off) 或方向燈閃爍時可暫停/淡出介入
from cereal import log
import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value


_MIN_V_EGO = 5.0
_MIN_LANE_PROB = 0.6
_MAX_LANE_STD = 0.3
_MIN_LANE_WIDTH = 2.6
_MAX_LANE_WIDTH = 4.8
_MAX_OFFSET = 0.3
_MIN_CENTER_TO_LINE = 1.1
_MAX_RAW_CORRECTION = 0.004
_MAX_GAIN = 0.30
_VISUAL_CORRECTION_EPSILON = 1e-6
_SMOOTH_TAU = 0.4
_SIGNAL_RELEASE_TAU = 0.20
_CONFIDENCE_RELEASE_TAU = 0.20
_CENTER_ERROR_DEADBAND = 0.08

_E2E_MAX_PATH_STD = 0.35
_E2E_BREAK_IN_START = 0.15
_E2E_BREAK_IN_FULL = 0.50


class LaneCenteringController:
  def __init__(self) -> None:
    self._correction = 0.0

  def reset(self) -> None:
    self._correction = 0.0

  def update(self, model_curvature, model_v2, v_ego, enabled, offset, e2e_authority, lat_active, model_valid,
             pause_on_signal=False, turn_signal_active=False, driver_override=False) -> float:
    model_curvature = float(model_curvature)

    try:
      v_ego = float(v_ego)
      offset = float(offset)
      e2e_authority = float(e2e_authority)
    except (TypeError, ValueError):
      self.reset()
      return model_curvature

    if not np.isfinite([v_ego, offset, e2e_authority]).all():
      self.reset()
      return model_curvature

    if not model_valid or not enabled or not lat_active or v_ego < _MIN_V_EGO:
      self.reset()
      return model_curvature

    if driver_override:
      self.reset()
      return model_curvature

    if pause_on_signal and turn_signal_active:
      self._correction = float(smooth_value(0.0, self._correction, _SIGNAL_RELEASE_TAU, dt=DT_CTRL))
      return model_curvature + self._correction

    try:
      if model_v2.meta.laneChangeState != log.LaneChangeState.off:
        self.reset()
        return model_curvature
    except (AttributeError, TypeError, ValueError):
      self.reset()
      return model_curvature

    valid, raw_correction = self._raw_correction(
      model_v2,
      v_ego,
      float(np.clip(offset, -_MAX_OFFSET, _MAX_OFFSET)),
      float(np.clip(e2e_authority, 0.0, 1.0)),
    )
    if not valid:
      self._correction = float(smooth_value(0.0, self._correction, _CONFIDENCE_RELEASE_TAU, dt=DT_CTRL))
      return model_curvature + self._correction

    target = float(np.clip(raw_correction, -_MAX_RAW_CORRECTION, _MAX_RAW_CORRECTION)) * _MAX_GAIN
    self._correction = float(smooth_value(target, self._correction, _SMOOTH_TAU, dt=DT_CTRL))
    return model_curvature + self._correction

  @staticmethod
  def _valid_path(x, y) -> bool:
    return x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0)

  @staticmethod
  def _covers(x, distance: float) -> bool:
    return bool(x[0] <= distance <= x[-1])

  @staticmethod
  def _raw_correction(model_v2, v_ego: float, offset: float, e2e_authority: float) -> tuple[bool, float]:
    try:
      lane_lines = model_v2.laneLines
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      stds = np.asarray(model_v2.laneLineStds, dtype=float)
      if len(lane_lines) < 3 or probs.size < 3 or stds.size < 3:
        return False, 0.0
      if not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all():
        return False, 0.0
      if np.any(probs[[1, 2]] < _MIN_LANE_PROB) or np.any(probs[[1, 2]] > 1.0):
        return False, 0.0
      if np.any(stds[[1, 2]] < 0.0) or np.any(stds[[1, 2]] > _MAX_LANE_STD):
        return False, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      if not (LaneCenteringController._valid_path(left_x, left_y) and
              LaneCenteringController._valid_path(right_x, right_y) and
              LaneCenteringController._valid_path(pos_x, pos_y)):
        return False, 0.0

      lookahead = float(np.clip(v_ego, 8.0, 35.0))
      if not all(LaneCenteringController._covers(x, lookahead) for x in (left_x, right_x, pos_x)):
        return False, 0.0

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left
      if not _MIN_LANE_WIDTH <= width <= _MAX_LANE_WIDTH:
        return False, 0.0

      max_safe_offset = min(_MAX_OFFSET, max(0.0, width * 0.5 - _MIN_CENTER_TO_LINE))
      target_y = 0.5 * (left + right) + float(np.clip(offset, -max_safe_offset, max_safe_offset))
      model_y = float(np.interp(lookahead, pos_x, pos_y))
      error = target_y - model_y
      error_abs = abs(error)
      if error_abs <= _CENTER_ERROR_DEADBAND:
        error = 0.0
      else:
        error = np.copysign(error_abs - _CENTER_ERROR_DEADBAND, error)

      try:
        pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)
        if LaneCenteringController._valid_path(pos_x, pos_y_std):
          path_std = float(np.interp(lookahead, pos_x, pos_y_std))
          if 0.0 <= path_std <= _E2E_MAX_PATH_STD:
            break_in = np.clip(
              (error_abs - _E2E_BREAK_IN_START) / (_E2E_BREAK_IN_FULL - _E2E_BREAK_IN_START),
              0.0,
              1.0,
            )
            error *= 1.0 - e2e_authority * float(break_in)
      except (AttributeError, TypeError, ValueError):
        pass

      return True, float(2.0 * error / lookahead ** 2)
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0


def get_raw_lane_centering_correction(model_v2, v_ego: float, offset: float,
                                      e2e_authority: float) -> tuple[bool, float]:
  """回傳未經 LaneCenteringController 濾波的瞬時車道置中修正量。"""
  return LaneCenteringController._raw_correction(model_v2, v_ego, offset, e2e_authority)


def get_lane_centering_visual_direction(model_v2, v_ego: float, offset: float, e2e_authority: float,
                                        enabled: bool, lat_active: bool, pause_on_signal: bool = False,
                                        turn_signal_active: bool = False,
                                        applied_correction: float | None = None) -> int:
  """回傳 1 表示向右修正、-1 表示向左修正、0 表示目前沒有修正在作用中。

  目前 openpilot-dplcc 尚未接上任何畫面渲染（車道線高亮）邏輯使用這個函式，
  純粹隨核心演算法一併移植過來，供未來若要做路面視覺化時使用。"""
  if not enabled or not lat_active or (pause_on_signal and turn_signal_active):
    return 0

  try:
    v_ego = float(v_ego)
    offset = float(offset)
    e2e_authority = float(e2e_authority)
    if not np.isfinite([v_ego, offset, e2e_authority]).all() or v_ego < _MIN_V_EGO:
      return 0
    if model_v2.meta.laneChangeState != log.LaneChangeState.off:
      return 0
  except (AttributeError, TypeError, ValueError):
    return 0

  valid, correction = get_raw_lane_centering_correction(
    model_v2,
    v_ego,
    float(np.clip(offset, -_MAX_OFFSET, _MAX_OFFSET)),
    float(np.clip(e2e_authority, 0.0, 1.0)),
  )
  if not valid or not np.isfinite(correction):
    return 0
  if applied_correction is not None and np.isfinite(applied_correction) and \
      abs(applied_correction) > _VISUAL_CORRECTION_EPSILON:
    correction = float(applied_correction)
  if abs(correction) <= _VISUAL_CORRECTION_EPSILON:
    return 0
  return 1 if correction > 0.0 else -1
