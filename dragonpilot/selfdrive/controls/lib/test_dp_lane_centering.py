"""dp_lane_centering.py 的參數讀取/預設值 glue 邏輯測試。

核心置中演算法本身的測試在
openpilot/selfdrive/controls/tests/test_lane_centering.py，這裡只驗證
DpLaneCentering 對 dp_ 參數的讀取、型別轉換與預設值處理是否正確，因此用一個
假的 Params 物件取代真正的 Params()，避免依賴磁碟上的 params 後端。
"""
import types

from dragonpilot.selfdrive.controls.lib.dp_lane_centering import DpLaneCentering


class _FakeParams:
  def __init__(self, values=None):
    self._values = dict(values or {})

  def get_bool(self, key, block=False):
    return bool(self._values.get(key, False))

  def get(self, key, block=False, return_default=False):
    return self._values.get(key)

  def put(self, key, val, block=False):
    self._values[key] = val

  def put_bool(self, key, val, block=False):
    self._values[key] = val


def _dp_lc(values=None):
  dp_lc = DpLaneCentering.__new__(DpLaneCentering)
  dp_lc._params = _FakeParams(values)
  dp_lc._controller = types.SimpleNamespace(reset=lambda: None)
  dp_lc._last_params_read = 0.0
  dp_lc.enabled = False
  dp_lc._offset = 0.0
  dp_lc._e2e_authority = 1.0
  dp_lc._pause_on_signal = True
  dp_lc._read_params(force=True)
  return dp_lc


def test_defaults_when_unset():
  dp_lc = _dp_lc()
  assert dp_lc.enabled is False
  # 車道內置中偏移量是純代碼內常數（見 _LANE_CENTER_OFFSET），不從 params 讀取，固定為 0
  assert dp_lc._offset == 0.0
  # e2e authority 預設 75%
  assert dp_lc._e2e_authority == 0.75
  # 沒被使用者寫入過時，pause-on-signal 的預設值是「開」
  assert dp_lc._pause_on_signal is True


def test_offset_is_a_code_constant_and_ignores_params():
  # 就算 params 裡曾經留有舊的 dp_lane_centering_offset 值，也完全不會被讀取
  dp_lc = _dp_lc({"dp_lane_centering": True, "dp_lane_centering_offset": "0.2"})
  assert dp_lc.enabled is True
  assert dp_lc._offset == 0.0


def test_e2e_authority_percent_is_scaled_and_clamped():
  dp_lc = _dp_lc({"dp_lane_centering_e2e_authority": "25"})
  assert dp_lc._e2e_authority == 0.25

  dp_lc = _dp_lc({"dp_lane_centering_e2e_authority": "150"})
  assert dp_lc._e2e_authority == 1.0

  dp_lc = _dp_lc({"dp_lane_centering_e2e_authority": "-10"})
  assert dp_lc._e2e_authority == 0.0


def test_pause_on_signal_can_be_explicitly_disabled():
  dp_lc = _dp_lc({"dp_lane_centering_pause_on_signal": "0"})
  assert dp_lc._pause_on_signal is False


def test_disabled_update_returns_model_curvature_unchanged():
  dp_lc = _dp_lc({"dp_lane_centering": False})
  assert dp_lc.update(0.05, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True) == 0.05
