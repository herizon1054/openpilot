"""Hardware-free regression checks. Run with python3 tools/jetlink/test_port.py."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_node(path, name, env, cls=None):
  tree = ast.parse((ROOT / path).read_text())
  body = tree.body if cls is None else next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls).body
  node = next(n for n in body if getattr(n, 'name', None) == name)
  node.decorator_list = []
  module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), node], type_ignores=[])
  exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), env)
  return env[name]


class PortTests(unittest.TestCase):
  def test_old_and_new_memoryview_api(self):
    view = load_node('jetlink_repo/jetlink/openpilot/warp.py', '_zero_copy_view', {})
    data = memoryview(bytearray(b'abc'))
    class Old:
      def as_memoryview(self, force_zero_copy=False):
        assert force_zero_copy
        return data
    class New:
      def as_memoryview(self, force_zero_copy=False, no_sync=False):
        assert force_zero_copy and no_sync
        return data
    self.assertIs(view(Old()), data)
    self.assertIs(view(New()), data)
    class Broken:
      def as_memoryview(self, **kwargs):
        raise RuntimeError('GPU failed')
    with self.assertRaises(RuntimeError): view(Broken())

  def test_model_status_valid(self):
    tree = ast.parse((ROOT / 'selfdrive/modeld/modeld.py').read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'new_message' and n.args and isinstance(n.args[0], ast.Constant)
             and n.args[0].value == 'modelExt']
    self.assertEqual(len(calls), 1)
    self.assertTrue(any(k.arg == 'valid' and k.value.value is True for k in calls[0].keywords))

  def test_external_action_head(self):
    env = dict(log=NS(ModelDataV2=NS(Action=lambda **kw: NS(**kw))),
               smooth_value=lambda value, old, seconds: value, LONG_SMOOTH_SECONDS=.2,
               LAT_SMOOTH_SECONDS=.2, MIN_LAT_CONTROL_SPEED=.3)
    action = load_node('selfdrive/modeld/modeld.py', 'get_action_from_model_jetlink', env)
    prev = NS(desiredAcceleration=0, desiredCurvature=.02)
    r = action({'action': np.array([[2., -.5]])}, prev, .4, .6, 10.)
    self.assertAlmostEqual(r.desiredCurvature, .02)
    self.assertEqual(r.desiredAcceleration, -.5)
    self.assertFalse(r.shouldStop)
    self.assertTrue(action({'action': np.array([[0., -.5]])}, prev, .4, .6, 0.).shouldStop)

  def test_control_gate_including_paused_alka_and_invalid_messages(self):
    env = {'IN_CONTROL': ('carState', 'carControl', 'controlsStateExt', 'carStateExt'), '_alka': lambda: True, 'SWAP_WHILE_PARKED': True, 'PARKED_GEARS': frozenset({'park', 'neutral', 'reverse'})}
    load_node('dragonpilot/jetlink_adapter/__init__.py', 'parked', env)
    gate = load_node('dragonpilot/jetlink_adapter/__init__.py', 'in_control', env)
    class SM(dict):
      healthy = True
      def all_alive(self, _): return self.healthy
      def all_valid(self, _): return self.healthy
    sm = SM(carState=NS(gearShifter='drive'), carControl=NS(enabled=False, latActive=False), controlsStateExt=NS(alkaActive=False), carStateExt=NS(lkasOn=False))
    self.assertFalse(gate(sm))
    sm['carStateExt'].lkasOn = True
    self.assertTrue(gate(sm))
    for gear in ('park', 'neutral', 'reverse'):
      sm['carState'].gearShifter = gear
      self.assertFalse(gate(sm))
      for service, field in [('carControl','enabled'), ('carControl','latActive'), ('controlsStateExt','alkaActive')]:
        setattr(sm[service], field, True)
        self.assertTrue(gate(sm))
        setattr(sm[service], field, False)
      sm.healthy = False
      self.assertTrue(gate(sm))
      sm.healthy = True
    for gear in ('drive', 'sport', 'low', 'unknown'):
      sm['carState'].gearShifter = gear
      self.assertTrue(gate(sm))
    sm['carState'].gearShifter = 'park'
    env['SWAP_WHILE_PARKED'] = False
    self.assertTrue(gate(sm))
    sm['carStateExt'].lkasOn = False
    sm.healthy = False
    self.assertTrue(gate(sm))
    sm.healthy = True
    sm['carControl'].latActive = True
    self.assertTrue(gate(sm))

  def test_waiting_keepalive_retries_and_reset(self):
    import threading
    from unittest.mock import Mock
    env = dict(time=NS(monotonic=lambda:100.), KEEPALIVE_PERIOD=10., PING_TIMEOUT=2., LOST='lost',
               KEEPALIVE_RETRY_DELAY=1., KEEPALIVE_RETRY_DELAY_MAX=5., QUICK_RETRIES=3)
    ping = load_node('jetlink_repo/jetlink/openpilot/joining.py', '_keep_alive', env, 'JoiningModelState')
    state = NS(_stop=threading.Event(), _rejoin=Mock(), _lock=threading.Lock(), _log=Mock(),
               _keepalive_lost=0, _failures=7, _note_link_loss=Mock(), _back_off=Mock(), _say_leaving=Mock())
    for expected in [1., 1., 1., 5., 5.]:
      client = Mock()
      client.ping.side_effect = TimeoutError('ping timed out')
      state._joined = (client, None)
      state._available = True
      state._rejoin.wait.return_value = False
      ping(state)
      self.assertEqual(state._rejoin_at, 100.+expected)
      self.assertEqual(state._failures, 7)
      self.assertFalse(state._available)
      self.assertIsNone(state._joined)
      client.close.assert_called_once()
    state._back_off.assert_not_called()
    client = Mock()
    state._joined = (client, None)
    state._rejoin.wait.side_effect = [False, True]
    ping(state)
    self.assertEqual(state._keepalive_lost, 0)
    self.assertIs(state._joined[0], client)
    client.close.assert_not_called()

  def test_model_switch_events_and_stale_state(self):
    names = ['bigModelAvailable','bigModelLinkLost','bigModelLoading','bigModelReady']
    env = {'jetlink_adapter': NS(OWNER='jetlinkd'), 'OFFER_TICKS':300, 'HANDBACK_TICKS':500,
           'SWITCHING_TICKS':100, 'AcceleratorState':NS(ready='ready', none='none'),
           'EventName':NS(**{n:n for n in names})}
    cls = load_node('dragonpilot/selfdrive/selfdrived/accelerator_events.py', 'AcceleratorEvents', env)
    class SM(dict): pass
    sm = SM(modelExt=NS(bigModel=False, acceleratorState='ready'))
    sm.seen = {'modelExt':True,'modelV2':True};sm.alive=sm.seen.copy();sm.valid={'modelExt':True}
    state=cls();events=set();state.update(sm,True,events)
    self.assertIn('bigModelAvailable',events)
    sm['modelExt'].bigModel=True;sm['modelExt'].acceleratorState='running'
    events=set();state.update(sm,False,events)
    self.assertIn('bigModelLoading',events)
    for _ in range(100):
      events=set();state.update(sm,False,events)
    self.assertIn('bigModelReady',events)
    sm.valid['modelExt']=False;events=set();state.update(sm,True,events)
    self.assertIn('bigModelLinkLost',events)

  def test_tici_hud_draw_path(self):
    calls=[]
    rl=NS(Rectangle=lambda *a:a, Vector2=lambda *a:a,
          draw_rectangle_rounded=lambda *a:None, draw_texture_ex=lambda *a:calls.append(a),
          draw_text_ex=lambda *a:None, Color=lambda *a:a)
    env=dict(jetlink_tr=lambda text: text, rl=rl, ui_state=NS(jetlink_state='active'), JetlinkState=NS(LOADING='loading'),
             jetlink_ui=NS(tint=lambda v:v,STATE_TEXT={'active':'大模型'},STATE_COLOR={'active':None}),
             JETLINK_BADGE_W=120,JETLINK_BADGE_H=89,JETLINK_LABEL_SIZE=40,
             COLORS=NS(BLACK_TRANSLUCENT=None),measure_text_cached=lambda *a:NS(x=80))
    draw=load_node('selfdrive/ui/onroad/hud_renderer.py','_draw_jetlink_badge',env,'HudRenderer')
    draw(NS(_jetlink_icons=NS(for_state=lambda state:(NS(width=120),1.)),_font_semi_bold=None),1800,300)
    self.assertEqual(len(calls),1)

  def test_mici_hud_draw_path(self):
    import math
    calls=[]
    env=dict(math=math, rl=NS(get_time=lambda:1.,Vector2=lambda *a:a,Color=lambda *a:a,draw_texture_ex=lambda *a:calls.append(a)),
             ui_state=NS(sm=NS(recv_frame={'selfdriveState':20}),started_frame=10,jetlink_state='active'),
             JetlinkState=NS(LOADING='loading',UNCOMPILED='uncompiled',FAILED='failed',ACTIVE='active',WAITING='waiting'),SET_SPEED_PERSISTENCE=5)
    draw=load_node('selfdrive/ui/mici/onroad/hud_renderer.py','_draw_model_source',env,'HudRenderer')
    icon=NS(width=60,height=44)
    fake=NS(_txt_jetlink_green=icon,_jetlink_icon=None,_jetlink_alpha_filter=NS(update=lambda v:1.))
    draw(fake,NS(x=0,y=0,width=476,height=240))
    self.assertEqual(len(calls),1)
    x,y=calls[0][1];self.assertGreaterEqual(x,0);self.assertLessEqual(x+60,476);self.assertLessEqual(y+44,240)


if __name__ == '__main__': unittest.main(verbosity=2)
