import types

import pytest

from openpilot.common.realtime import DT_MDL
from dragonpilot.selfdrive.controls.lib import aem as aem_mod
from dragonpilot.selfdrive.controls.lib.aem import (
  AEM, CONFIRM_TIME_S, CURVE_LAT_ACCEL_ENTER, CURVE_LAT_ACCEL_EXIT,
  NEAR_STOP_ENTER_M, NEAR_STOP_EXIT_M, BASE_THROTTLE_LOW_SPEED_VALUE, BASE_THROTTLE_HIGH_SPEED_VALUE,
)

# v11 AEM 測試：mode 由「過彎 > 接近停止線 > 車速」決定（見 aem.py 檔頭）。
# 取代 v0 以 gasPressProbs 機率門檻選 mode 的測試。

CONFIRM_FRAMES = int(round(CONFIRM_TIME_S / DT_MDL)) + 1   # 防彈跳需要的幀數（含餘裕 1 幀）


def model(yaw_rate=0.0, have_orientation=True):
  z = [yaw_rate] if have_orientation else []
  return types.SimpleNamespace(orientationRate=types.SimpleNamespace(z=z))


def kph(v):
  return v / 3.6


def run(aem, frames, v_kph, yaw_rate=0.0, stop_dist_m=None):
  for _ in range(frames):
    aem.update_states(model(yaw_rate), radar_msg=None, v_ego=kph(v_kph), stop_dist_m=stop_dist_m)
  return aem


def yaw_for_lat_accel(a_y, v_kph):
  return a_y / kph(v_kph)


# --- 車速 ---

def test_low_speed_forces_blended_immediately():
  aem = run(AEM(), 1, 15.0)
  assert aem.get_mode('acc') == 'blended'


def test_high_speed_is_acc():
  aem = run(AEM(), 1, 40.0)
  assert aem.get_mode('blended') == 'acc'


def test_speed_up_from_low_switches_to_acc_after_confirm():
  aem = run(AEM(), 10, 15.0)
  run(aem, 2, 40.0)
  assert aem.get_mode('acc') == 'blended'           # 防彈跳期間維持
  run(aem, CONFIRM_FRAMES, 40.0)
  assert aem.get_mode('acc') == 'acc'


def test_transition_band_holds_previous_mode():
  low = run(AEM(), 10, 15.0)
  run(low, 100, 25.0)
  assert low.get_mode('acc') == 'blended'           # 由低速上來：維持實驗模式

  high = run(AEM(), 10, 40.0)
  run(high, 100, 25.0)
  assert high.get_mode('acc') == 'acc'              # 由高速下來：維持 acc


# --- 過彎保護 ---

def test_curve_forces_acc_even_at_low_speed():
  v = 18.0
  aem = run(AEM(), 100, v, yaw_rate=yaw_for_lat_accel(CURVE_LAT_ACCEL_ENTER + 0.5, v))
  assert aem.get_mode('blended') == 'acc'


def test_curve_releases_with_hysteresis():
  v = 18.0
  aem = run(AEM(), 100, v, yaw_rate=yaw_for_lat_accel(CURVE_LAT_ACCEL_ENTER + 0.5, v))
  # 介於解除與進入門檻之間：維持過彎保護
  run(aem, 100, v, yaw_rate=yaw_for_lat_accel((CURVE_LAT_ACCEL_ENTER + CURVE_LAT_ACCEL_EXIT) / 2, v))
  assert aem.get_mode('blended') == 'acc'
  # 低於解除門檻：解除
  run(aem, 100, v, yaw_rate=0.0)
  assert aem.get_mode('acc') == 'blended'


def test_missing_orientation_rate_treated_as_straight():
  aem = AEM()
  aem.update_states(model(have_orientation=False), radar_msg=None, v_ego=kph(15.0))
  assert aem.get_mode('acc') == 'blended'


# --- 接近停止線 ---

def test_near_stop_forces_blended_at_city_speed():
  aem = run(AEM(), 100, 50.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  assert aem.near_stop_active
  assert aem.get_mode('acc') == 'blended'


def test_near_stop_speed_gate_blocks_highway():
  aem = run(AEM(), 100, 80.0, stop_dist_m=NEAR_STOP_ENTER_M - 5.0)
  assert aem.near_stop_active                        # 節流門檻覆寫仍生效
  assert aem.get_mode('blended') == 'acc'            # 但不切 mode


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
  aem = run(AEM(), 1, 15.0)
  assert aem.get_mode('acc') == 'blended'


def test_experimental_off_forces_acc_when_flag_set(monkeypatch):
  monkeypatch.setattr(aem_mod, 'AEM_EXPERIMENTAL_OFF_FORCES_ACC', True)
  aem = run(AEM(), 1, 15.0)
  assert aem.get_mode('acc') == 'acc'
  assert aem.get_mode('blended') == 'blended'


if __name__ == '__main__':
  raise SystemExit(pytest.main([__file__, '-q']))
