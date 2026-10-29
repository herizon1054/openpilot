import types

import pytest

from openpilot.common.realtime import DT_MDL
from dragonpilot.selfdrive.controls.lib import aem as aem_mod
from dragonpilot.selfdrive.controls.lib.aem import (
  AEM, CONFIRM_TIME_S, MIN_DWELL_TIME_S, BLINKER_OFF_HOLD_S, NEAR_STOP_STEER_RELEASE_S,
  NEAR_STOP_ENTER_M, NEAR_STOP_EXIT_M, BASE_THROTTLE_LOW_SPEED_VALUE, BASE_THROTTLE_HIGH_SPEED_VALUE,
)

# v17 AEM 測試：mode 由「接近停止線（方向盤 <= 20°）> 方向燈 > 車速」決定（見 aem.py 檔頭）。
#   方向燈亮（含熄滅後 BLINKER_OFF_HOLD_S）-> 一般模式
#   否則：<= 20 km/h 立即實驗模式、20~30 維持、>= 30 一般模式（防彈跳）

CONFIRM_FRAMES = int(round((CONFIRM_TIME_S + MIN_DWELL_TIME_S) / DT_MDL)) + 2
MODEL = types.SimpleNamespace()


def kph(v):
  return v / 3.6


def run(aem, frames, v_kph, blinker=False, stop_dist_m=None, steer=0.0):
  for _ in range(frames):
    aem.update_states(MODEL, radar_msg=None, v_ego=kph(v_kph), stop_dist_m=stop_dist_m, blinker=blinker,
                      steer_angle_deg=steer)
  return aem


# --- 車速（未打方向燈）---

@pytest.mark.parametrize("v_kph", [0.0, 15.0, 19.9])
def test_low_speed_forces_blended_immediately(v_kph):
  assert run(AEM(), 1, v_kph).get_mode('acc') == 'blended'


def test_high_speed_is_acc():
  assert run(AEM(), 1, 40.0).get_mode('blended') == 'acc'


def test_speed_up_switches_to_acc_after_30_with_debounce():
  aem = run(AEM(), 10, 15.0)
  run(aem, 2, 40.0)
  assert aem.get_mode('acc') == 'blended'            # 防彈跳期間維持
  run(aem, CONFIRM_FRAMES, 40.0)
  assert aem.get_mode('acc') == 'acc'


def test_transition_band_20_30_holds_previous_mode():
  low = run(AEM(), 10, 15.0)
  assert run(low, 100, 25.0).get_mode('acc') == 'blended'
  high = run(AEM(), 10, 40.0)
  assert run(high, 100, 25.0).get_mode('acc') == 'acc'


# --- 方向燈 ---

def test_blinker_blocks_experimental_at_low_speed():
  aem = run(AEM(), 5, 10.0, blinker=True)
  assert aem.blinker_active
  assert aem.get_mode('blended') == 'acc'


def test_blinker_on_switches_out_of_experimental_same_frame():
  aem = run(AEM(), 10, 10.0)
  assert aem.get_mode('acc') == 'blended'
  assert run(aem, 1, 10.0, blinker=True).get_mode('blended') == 'acc'


def test_blinker_off_needs_hold_before_experimental():
  hold = int(round(BLINKER_OFF_HOLD_S / DT_MDL))
  aem = run(AEM(), 10, 10.0, blinker=True)
  assert run(aem, hold - 2, 10.0).get_mode('blended') == 'acc'
  assert run(aem, 3, 10.0).get_mode('acc') == 'blended'


def test_flashing_blinker_signal_does_not_flicker():
  # 部分車款方向燈訊號隨燈閃（約 0.4 s 亮 / 0.4 s 滅）：全程維持 acc
  aem = run(AEM(), 10, 10.0)
  out = []
  for _ in range(10):
    for on in (True, False):
      run(aem, 8, 10.0, blinker=on)
      out.append(aem.get_mode('blended'))
  assert set(out) == {'acc'}


@pytest.mark.parametrize("v_kph", [25.0, 31.0])
def test_blinker_release_above_20_keeps_acc_without_blip(v_kph):
  # 低速打燈轉彎、加速出彎，解除時已 > 20 km/h：延續 acc，不閃 blended
  aem = run(AEM(), 20, 10.0, blinker=True)
  out = []
  for _ in range(100):
    run(aem, 1, v_kph)
    out.append(aem.get_mode('blended'))
  assert set(out) == {'acc'}


def test_blinker_at_high_speed_is_acc_anyway():
  assert run(AEM(), 5, 50.0, blinker=True).get_mode('blended') == 'acc'


# --- 接近停止線（不受定速閘門影響）---

def test_near_stop_forces_blended_at_city_speed():
  aem = run(AEM(), 100, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  assert aem.near_stop_active
  assert aem.get_mode('acc') == 'blended'


def test_near_stop_has_priority_over_blinker():
  aem = run(AEM(), 100, 15.0, blinker=True, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  assert aem.get_mode('acc') == 'blended'


@pytest.mark.parametrize("steer", [21.0, -21.0, 90.0])
def test_near_stop_cancelled_by_steer_over_20(steer):
  # 50 km/h、接近停止線：方向盤 > 20° 不強制 blended，交給車速判斷（50 km/h -> acc）
  aem = run(AEM(), 100, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0, steer=steer)
  assert aem.near_stop_active and aem.near_stop_steer_cancel
  assert aem.get_mode('blended') == 'acc'


def test_near_stop_steer_20_or_less_still_blended():
  aem = run(AEM(), 100, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0, steer=20.0)
  assert aem.get_mode('acc') == 'blended'


def test_near_stop_steer_cancel_same_frame_and_release_hold():
  hold = int(round(NEAR_STOP_STEER_RELEASE_S / DT_MDL))
  aem = run(AEM(), 100, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  assert aem.get_mode('acc') == 'blended'
  assert run(aem, 1, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0, steer=45.0).get_mode('blended') == 'acc'
  assert run(aem, hold - 2, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0, steer=5.0).get_mode('blended') == 'acc'
  assert run(aem, 3, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0, steer=5.0).get_mode('acc') == 'blended'


def test_steer_does_not_affect_speed_rule():
  # 方向盤角度只影響接近停止線；低速未打燈仍是實驗模式
  assert run(AEM(), 5, 10.0, steer=90.0).get_mode('acc') == 'blended'


def test_near_stop_speed_gate_blocks_highway():
  aem = run(AEM(), 100, 80.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  assert aem.near_stop_active
  assert aem.get_mode('blended') == 'acc'


def test_near_stop_none_is_not_near():
  aem = run(AEM(), 100, 50.0, stop_dist_m=None)
  assert not aem.near_stop_active
  assert aem.get_mode('blended') == 'acc'


def test_near_stop_hysteresis():
  aem = run(AEM(), 100, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  run(aem, 100, 50.0, stop_dist_m=(NEAR_STOP_ENTER_M + NEAR_STOP_EXIT_M) / 2)
  assert aem.near_stop_active
  run(aem, 100, 50.0, stop_dist_m=NEAR_STOP_EXIT_M + 5.0)
  assert not aem.near_stop_active


# --- 基礎節流門檻 ---

def test_base_throttle_threshold_by_speed():
  assert run(AEM(), 100, 5.0).base_throttle_threshold == BASE_THROTTLE_LOW_SPEED_VALUE
  assert run(AEM(), 100, 40.0).base_throttle_threshold == BASE_THROTTLE_HIGH_SPEED_VALUE


# --- 實驗模式開關（刻意分歧 2）---

def test_experimental_toggle_ignored_by_default():
  assert run(AEM(), 1, 15.0).get_mode('acc') == 'blended'


def test_experimental_off_forces_acc_when_flag_set(monkeypatch):
  monkeypatch.setattr(aem_mod, 'AEM_EXPERIMENTAL_OFF_FORCES_ACC', True)
  aem = run(AEM(), 1, 15.0)
  assert aem.get_mode('acc') == 'acc'
  assert aem.get_mode('blended') == 'blended'


if __name__ == '__main__':
  raise SystemExit(pytest.main([__file__, '-q']))
