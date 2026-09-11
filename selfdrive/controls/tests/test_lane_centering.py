# 移植自 StarPilot (firestar5683/StarPilot)
# selfdrive/controls/tests/test_lane_centering.py
#
# 對應核心演算法：openpilot.selfdrive.controls.lib.lane_centering
# 這裡只測試「純演算法」層本身，dp_lane_centering.py 的參數讀取/預設值
# glue 邏輯測試請見 dragonpilot/selfdrive/controls/tests/test_dp_lane_centering.py
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController


_V_EGO = 20.0
_XS = np.linspace(0.0, 50.0, 52)


def _path(y, y_std=0.1):
  return SimpleNamespace(
    x=_XS.copy(),
    y=np.full_like(_XS, float(y)),
    yStd=np.full_like(_XS, float(y_std)),
  )


def _model(left=-1.8, right=1.8, model_y=0.0, lane_prob=0.9, lane_std=0.1, path_std=0.1, lane_change=0):
  return SimpleNamespace(
    laneLines=[_path(0.0), _path(left), _path(right), _path(0.0)],
    laneLineProbs=[0.0, lane_prob, lane_prob, 0.0],
    laneLineStds=[0.0, lane_std, lane_std, 0.0],
    position=_path(model_y, path_std),
    meta=SimpleNamespace(laneChangeState=lane_change),
  )


def _update(controller, model, *, offset=0.0, authority=1.0, enabled=True, active=True, valid=True, speed=_V_EGO,
            pause_on_signal=False, turn_signal_active=False):
  return controller.update(0.0, model, speed, enabled, offset, authority, active, valid,
                           pause_on_signal, turn_signal_active)


def _converge(model, *, offset=0.0, authority=1.0, speed=_V_EGO):
  controller = LaneCenteringController()
  output = 0.0
  for _ in range(300):
    output = _update(controller, model, offset=offset, authority=authority, speed=speed)
  return controller, output


@pytest.mark.parametrize(
  "kwargs",
  [
    {"enabled": False},
    {"active": False},
    {"valid": False},
    {"speed": 4.0},  # < _MIN_V_EGO (15 km/h ≈ 4.1667 m/s)
  ],
)
def test_hard_gates_are_noop(kwargs):
  assert _update(LaneCenteringController(), _model(left=-1.5, right=2.1), **kwargs) == 0.0


def test_lane_change_is_noop():
  assert _update(LaneCenteringController(), _model(left=-1.5, right=2.1, lane_change=1)) == 0.0


def test_turn_signal_fades_lane_centering_correction():
  model = _model(left=-1.5, right=2.1)
  controller, centered = _converge(model, authority=0.0)
  fading = _update(controller, model, authority=0.0, pause_on_signal=True, turn_signal_active=True)
  assert 0.0 < fading < centered

  for _ in range(300):
    fading = _update(controller, model, authority=0.0, pause_on_signal=True, turn_signal_active=True)
  assert abs(fading) < 1e-6


def test_turn_signal_pause_can_be_disabled():
  model = _model(left=-1.5, right=2.1)
  _, output = _converge(model, authority=0.0)
  controller, _ = _converge(model, authority=0.0)
  signaled = _update(controller, model, authority=0.0, turn_signal_active=True)
  assert signaled == pytest.approx(output, abs=1e-7)


@pytest.mark.parametrize(
  "field,value",
  [
    ("prob", np.nan),
    ("prob", 1.1),
    ("std", np.nan),
    ("std", -0.1),
  ],
)
def test_invalid_lane_confidence_is_rejected(field, value):
  model = _model(left=-1.5, right=2.1)
  values = model.laneLineProbs if field == "prob" else model.laneLineStds
  values[1] = value
  assert _update(LaneCenteringController(), model) == 0.0


def test_input_must_cover_lookahead():
  model = _model(left=-1.5, right=2.1)
  model.laneLines[1].x = model.laneLines[1].x[:10]
  model.laneLines[1].y = model.laneLines[1].y[:10]
  assert _update(LaneCenteringController(), model) == 0.0


def test_lane_center_error_steers_toward_center():
  _, right = _converge(_model(left=-1.5, right=2.1), authority=0.0)
  _, left = _converge(_model(left=-2.1, right=1.5), authority=0.0)
  assert right > 0.0
  assert left < 0.0


def test_small_center_error_does_not_chatter():
  _, output = _converge(_model(left=-1.75, right=1.85), authority=0.0)
  assert output == 0.0


def test_offset_direction():
  _, right = _converge(_model(), offset=0.2, authority=0.0)
  _, left = _converge(_model(), offset=-0.2, authority=0.0)
  assert right > 0.0
  assert left < 0.0


def test_offset_is_reduced_in_narrow_lane():
  narrow = _model(left=-1.3, right=1.3)
  _, at_safe_limit = _converge(narrow, offset=0.2, authority=0.0)
  _, above_safe_limit = _converge(narrow, offset=0.3, authority=0.0)
  assert np.isclose(at_safe_limit, above_safe_limit)


def test_confident_e2e_path_can_fully_break_in():
  model = _model(left=-1.0, right=2.6, model_y=0.0, path_std=0.1)
  _, lane_authority = _converge(model, authority=0.0)
  _, e2e_authority = _converge(model, authority=1.0)
  assert lane_authority > 0.0
  assert abs(e2e_authority) < 1e-9


def test_uncertain_e2e_path_does_not_break_in():
  model = _model(left=-1.0, right=2.6, model_y=0.0, path_std=0.6)
  _, output = _converge(model, authority=1.0)
  assert output > 0.0


def test_e2e_authority_blends_lane_correction():
  model = _model(left=-1.2, right=2.4, model_y=0.0, path_std=0.1)
  _, lane_only = _converge(model, authority=0.0)
  _, blended = _converge(model, authority=0.5)
  _, e2e = _converge(model, authority=1.0)
  assert lane_only > blended > e2e >= 0.0


def test_confident_e2e_authority_starts_before_large_offset():
  model = _model(left=-1.7, right=2.1, model_y=0.0, path_std=0.1)
  _, lane_only = _converge(model, authority=0.0)
  _, e2e = _converge(model, authority=1.0)
  assert lane_only > e2e > 0.0


def test_confidence_loss_drops_filtered_correction():
  controller, output = _converge(_model(left=-1.5, right=2.1), authority=0.0)
  assert output > 0.0
  fading = _update(controller, _model(left=-1.5, right=2.1, lane_prob=0.2), authority=0.0)
  assert 0.0 < fading < output

  for _ in range(300):
    fading = _update(controller, _model(left=-1.5, right=2.1, lane_prob=0.2), authority=0.0)
  assert abs(fading) < 1e-6


def test_correction_is_smoothed_and_capped():
  controller = LaneCenteringController()
  model = _model(left=0.0, right=3.0, path_std=0.6)
  first = _update(controller, model, authority=0.0)
  _, steady = _converge(model, authority=0.0)
  assert 0.0 < first < steady
  assert np.isclose(steady, 0.004 * 0.30, atol=1e-6)


# --- 以下對應與 cp 的三個刻意分歧（見 lane_centering.py 檔頭說明） ---

def test_min_v_ego_is_15kph_not_18kph():
  model = _model(left=-1.5, right=2.1)
  just_below = 15.0 / 3.6 - 0.05  # 略低於新門檻
  just_above = 15.0 / 3.6 + 0.05  # 略高於新門檻，且仍低於舊的 18 km/h 門檻
  assert _update(LaneCenteringController(), model, speed=just_below) == 0.0
  assert _update(LaneCenteringController(), model, speed=just_above) != 0.0


def test_e2e_confidence_weight_ramps_smoothly_instead_of_cliff():
  model = _model(left=-1.0, right=2.6, model_y=0.0)

  # path_std 在 ramp start (0.25) 以下：信心權重滿分，效果等同舊版「門檻內」
  model.position = _path(0.0, y_std=0.2)
  _, full_confidence = _converge(model, authority=1.0)
  assert abs(full_confidence) < 1e-9

  # path_std 在 ramp start (0.25) 與 max (0.35) 中間：應該是介於「完全折抵」
  # 與「完全不折抵」之間的部分值，而不是兩個極端之一
  model.position = _path(0.0, y_std=0.30)
  _, mid_confidence = _converge(model, authority=1.0)
  _, lane_only = _converge(model, authority=0.0)
  assert 0.0 < mid_confidence < lane_only

  # path_std 超過 max (0.35)：信心權重為 0，等同舊版「門檻外」，e2e 完全不折抵
  model.position = _path(0.0, y_std=0.6)
  _, no_confidence = _converge(model, authority=1.0)
  assert no_confidence == pytest.approx(lane_only, rel=1e-6)


def test_e2e_speed_weight_ramps_smoothly_with_vehicle_speed():
  model = _model(left=-1.9, right=1.9, model_y=0.20, path_std=0.1)

  # 剛超過 _MIN_V_EGO（15 km/h），仍遠低於 60 km/h 的滿分車速門檻，
  # e2e 折抵應該只有一小部分生效（同一車速下比較 authority=0 vs 1，
  # 才不會被「lookahead 隨車速改變」這個既有的、跟本次修改無關的效果干擾）
  low_speed = 15.0 / 3.6 + 0.1
  _, lane_only_low = _converge(model, authority=0.0, speed=low_speed)
  _, low_speed_e2e = _converge(model, authority=1.0, speed=low_speed)
  assert abs(lane_only_low) > abs(low_speed_e2e) > 0.0
  low_speed_offload_ratio = 1.0 - abs(low_speed_e2e) / abs(lane_only_low)

  # 60 km/h（含）以上：車速權重滿分
  high_speed = 60.0 / 3.6 + 1.0
  _, lane_only_high = _converge(model, authority=0.0, speed=high_speed)
  _, high_speed_e2e = _converge(model, authority=1.0, speed=high_speed)
  assert abs(lane_only_high) > abs(high_speed_e2e) > 0.0
  high_speed_offload_ratio = 1.0 - abs(high_speed_e2e) / abs(lane_only_high)

  # 車速越快，e2e 折抵掉的比例越高，是單調遞增、沒有斷崖的平滑變化
  assert 0.0 < low_speed_offload_ratio < high_speed_offload_ratio < 1.0


def test_avoidance_does_not_falsely_suppress_cold_start():
  # reset 之後（含全新 controller）的第一次呼叫，應該直接把當下修正量當成
  # 基準值，不能被誤判成「突然」而被壓低——否則車道置中每次剛啟用都會慢半拍
  model = _model(left=-1.5, right=2.1)
  controller = LaneCenteringController()
  first_with_avoidance = _update(controller, model, authority=0.0)

  reference = LaneCenteringController()
  reference._raw_correction_ema = 10.0  # 故意塞一個跟真實值差很多的基準值
  first_with_stale_ema = _update(reference, model, authority=0.0)

  # 冷啟動（基準值為 None）不應該被壓低，應該跟「基準值剛好等於當下值」一樣
  assert first_with_avoidance != 0.0
  assert first_with_stale_ema == 0.0 or abs(first_with_stale_ema) < abs(first_with_avoidance)


def test_sudden_swap_is_suppressed_more_than_gradual_smoothing_alone():
  import openpilot.selfdrive.controls.lib.lane_centering as lc_module

  model_a = _model(left=-1.5, right=2.1)
  model_b = _model(left=-2.1, right=1.5)  # 車道線左右對調，修正方向瞬間反過來

  def _switch_and_settle(jump_span):
    original = lc_module._AVOIDANCE_JUMP_SPAN
    lc_module._AVOIDANCE_JUMP_SPAN = jump_span
    try:
      controller = LaneCenteringController()
      for _ in range(300):
        _update(controller, model_a, authority=0.0)
      output = 0.0
      for _ in range(40):
        output = _update(controller, model_b, authority=0.0)
      return output
    finally:
      lc_module._AVOIDANCE_JUMP_SPAN = original

  # 正常門檻：突然反向的修正量會被避讓機制壓低
  with_avoidance = _switch_and_settle(lc_module._AVOIDANCE_JUMP_SPAN)
  # 門檻放到超大，等於避讓機制永遠不觸發，只剩下原本就有的 _SMOOTH_TAU 平滑，當對照組
  without_avoidance = _switch_and_settle(1e6)

  assert abs(with_avoidance) < abs(without_avoidance)


def test_persistent_deviation_eventually_overrides_avoidance_suppression():
  model_a = _model(left=-1.5, right=2.1)
  model_b = _model(left=-2.1, right=1.5)

  controller = LaneCenteringController()
  for _ in range(300):
    _update(controller, model_a, authority=0.0)
  # 切換後跑夠久（遠超過 _AVOIDANCE_EMA_TAU），代表這是持續存在的長期偏移，
  # 而不是短暫避讓，車道置中應該恢復介入、逐漸修正回新的車道線位置
  output = 0.0
  for _ in range(1500):
    output = _update(controller, model_b, authority=0.0)

  _, steady_b = _converge(model_b, authority=0.0)
  assert output == pytest.approx(steady_b, rel=0.05)
