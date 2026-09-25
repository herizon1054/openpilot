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
    val = self._values.get(key, False)
    if isinstance(val, bool):
      return val
    if isinstance(val, str):
      return val in ("1", "true", "True")
    return bool(val)

  def get(self, key, block=False, return_default=False):
    return self._values.get(key)

  def put(self, key, val, block=False):
    self._values[key] = val

  def put_bool(self, key, val, block=False):
    self._values[key] = val


class _FakeController:
  """記錄 reset()／update() 被呼叫的次數，用來驗證下降緣觸發的邊界行為，
  不需要真的算車道置中邏輯。"""
  def __init__(self):
    self.reset_count = 0
    self.update_count = 0

  def reset(self):
    self.reset_count += 1

  def update(self, model_curvature, model_v2, v_ego, enabled, offset, e2e_authority, lat_active, model_valid,
             pause_on_signal=False, turn_signal_active=False, driver_override=False):
    self.update_count += 1
    return model_curvature + 1.0  # 隨便回傳一個跟輸入不同的值，方便測試分辨有沒有真的呼叫到這裡


def _dp_lc(values=None):
  dp_lc = DpLaneCentering.__new__(DpLaneCentering)
  dp_lc._params = _FakeParams(values)
  dp_lc._controller = types.SimpleNamespace(reset=lambda: None)
  dp_lc._last_params_read = 0.0
  dp_lc.enabled = False
  dp_lc._offset = 0.0
  dp_lc._e2e_authority = 1.0
  dp_lc._pause_on_signal = False
  dp_lc._was_enabled = False
  dp_lc._read_params(force=True)
  return dp_lc


def test_defaults_when_unset():
  dp_lc = _dp_lc()
  assert dp_lc.enabled is False
  # 車道內置中偏移量是純代碼內常數（見 _LANE_CENTER_OFFSET），不從 params 讀取，固定為 0
  assert dp_lc._offset == 0.0
  # e2e authority 預設 85%
  assert dp_lc._e2e_authority == 0.85
  # 打方向燈暫停車道置中輔助，預設關閉
  assert dp_lc._pause_on_signal is False


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


def test_pause_on_signal_can_be_explicitly_enabled():
  dp_lc = _dp_lc({"dp_lane_centering_pause_on_signal": "1"})
  assert dp_lc._pause_on_signal is True


def test_disabled_update_returns_model_curvature_unchanged():
  dp_lc = _dp_lc({"dp_lane_centering": False})
  assert dp_lc.update(0.05, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True) == 0.05


def test_controller_reset_only_on_enabled_to_disabled_edge():
  # 對應 dp_lane_centering_reset_fix_說明.md 第 5 節的驗證序列：
  # reset() 應該只在「啟用→停用」的下降緣被呼叫一次，不會在持續停用期間
  # 每幀重複呼叫，也不會在從未啟用過就關閉、或重新啟用時被多按一次。
  dp_lc = _dp_lc({"dp_lane_centering": False})
  fake = _FakeController()
  dp_lc._controller = fake

  # 從未啟用時關閉 → 不需要 reset（控制器本來就是乾淨的）
  dp_lc.update(0.0, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  assert fake.reset_count == 0
  assert fake.update_count == 0

  # 啟用 → 正常呼叫 update()，不觸發 reset
  dp_lc.enabled = True
  dp_lc.update(0.0, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  assert fake.reset_count == 0
  assert fake.update_count == 1

  # 持續啟用一幀 → 一樣不觸發 reset
  dp_lc.update(0.0, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  assert fake.reset_count == 0
  assert fake.update_count == 2

  # 停用（下降緣）→ 剛好觸發一次 reset，輸出立即變回 model_curvature
  dp_lc.enabled = False
  out = dp_lc.update(0.42, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  assert fake.reset_count == 1
  assert fake.update_count == 2
  assert out == 0.42

  # 持續停用兩幀 → 不會重複呼叫 reset
  dp_lc.update(0.0, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  dp_lc.update(0.0, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  assert fake.reset_count == 1

  # 重新啟用 → 不會多按一次 reset，正常恢復呼叫 update()
  dp_lc.enabled = True
  dp_lc.update(0.0, model_v2=None, v_ego=20.0, lat_active=True, model_valid=True)
  assert fake.reset_count == 1
  assert fake.update_count == 3
