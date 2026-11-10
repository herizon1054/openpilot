"""opview 實驗模式開關（serverd /api/experimental_mode）的條件判斷。"""
from dragonpilot.dashy.serverd import experimental_mode_status, handle_experimental_mode


class FakeParams:
  def __init__(self, **kw):
    self.store = dict(kw)

  def get_bool(self, key):
    return bool(self.store.get(key))

  def put_bool(self, key, value):
    self.store[key] = bool(value)


def test_bad_body():
  p = FakeParams()
  assert handle_experimental_mode(p, None, True)[0] == 400
  assert handle_experimental_mode(p, {"enabled": "yes"}, True)[0] == 400
  assert "ExperimentalMode" not in p.store


def test_enable_requires_long_and_confirmation():
  p = FakeParams(ExperimentalModeConfirmed=True)
  status, body = handle_experimental_mode(p, {"enabled": True}, None)
  assert status == 409 and "發動" in body["error"]
  status, body = handle_experimental_mode(p, {"enabled": True}, False)
  assert status == 409 and "縱向" in body["error"]
  assert "ExperimentalMode" not in p.store

  p = FakeParams()
  status, body = handle_experimental_mode(p, {"enabled": True}, True)
  assert status == 409 and "確認" in body["error"]
  assert "ExperimentalModeConfirmed" not in p.store  # 不代替使用者確認


def test_enable_and_disable():
  p = FakeParams(ExperimentalModeConfirmed=True)
  assert handle_experimental_mode(p, {"enabled": True}, True) == (200, {"experimental_mode": True})
  assert experimental_mode_status(p, True) == {"experimental_mode": True, "available": True, "confirmed": True}
  # 關閉一律允許（即使縱向不可用 / 未確認）
  p = FakeParams(ExperimentalMode=True)
  assert handle_experimental_mode(p, {"enabled": False}, None) == (200, {"experimental_mode": False})
