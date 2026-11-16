"""opview Auto Hold 按鈕（serverd /api/auto_hold）的條件判斷。"""
from dragonpilot.dashy.serverd import auto_hold_status, handle_auto_hold


class FakeParams:
  def __init__(self, **kw):
    self.store = dict(kw)

  def get(self, key):
    return self.store.get(key)

  def get_bool(self, key):
    return bool(self.store.get(key))

  def put_bool(self, key, value):
    self.store[key] = bool(value)


def test_bad_body():
  p = FakeParams(OpviewAutoHoldState=1)
  assert handle_auto_hold(p, None)[0] == 400
  assert handle_auto_hold(p, {"paused": 1})[0] == 400
  assert "OpviewAutoHoldPaused" not in p.store


def test_pause_requires_available():
  p = FakeParams()
  status, body = handle_auto_hold(p, {"paused": True})
  assert status == 409 and "未啟用" in body["error"]
  assert "OpviewAutoHoldPaused" not in p.store


def test_resume_always_allowed():
  p = FakeParams(OpviewAutoHoldPaused=True)
  status, body = handle_auto_hold(p, {"paused": False})
  assert status == 200 and body["paused"] is False


def test_pause_and_status_bits():
  p = FakeParams(OpviewAutoHoldState=1 | 4)
  assert auto_hold_status(p) == {"paused": False, "available": True, "holding": True}
  status, body = handle_auto_hold(p, {"paused": True})
  assert status == 200 and body["paused"] is True and body["available"] is True
