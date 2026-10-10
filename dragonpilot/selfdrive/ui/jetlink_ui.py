"""
dp - jetlink: what the panels and the onroad/sidebar icons show of the link.

Ported from zoompilot's selfdrive/ui/sunnypilot/accelerator_link.py and the
jetlink parts of model_info.py / layouts/settings/models.py. Everything here
reads ui_state.jetlink (the snapshot the 5 Hz params pass takes) and
ui_state.jetlink_state, never the link itself.

The icons are comma's chestnut icons (commaai/openpilot
selfdrive/assets/icons_mici/chestnut*.png), shipped here as
dragonpilot/selfdrive/assets/icons/jetlink*.png: zoompilot uses the same
icons for the link.
"""
from dragonpilot.system.ui.lib.multilang import tr, tr_noop

import math
from typing import Union

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, JetlinkState
from openpilot.system.ui.lib.application import gui_app
from dragonpilot.jetlink_adapter import KEYS, MODES

LINK_MODES = MODES
LINK_PARAM = KEYS.link
LINK_MODE_TITLES = {"off": tr_noop('Off'), "usb": "USB", "ios": "iOS"}

ICON_DIR = "../../dragonpilot/selfdrive/assets/icons"

# the UI fonts are bitmaps holding ASCII, a few symbols and the characters of the
# translation files: a character outside them (an em dash, a middle dot) draws as "?"
def none_text():
  return tr("None")

# one short word per state, for the sidebar card and the onroad badge
STATE_TEXT = {
  JetlinkState.DISCONNECTED: tr_noop('Disconnected'),
  JetlinkState.UNCOMPILED: tr_noop('Not ready'),
  JetlinkState.READY: tr_noop('Ready'),
  JetlinkState.LOADING: tr_noop('Connecting'),
  JetlinkState.ACTIVE: tr_noop('Large model'),
  JetlinkState.FAILED: tr_noop('Failed'),
  JetlinkState.WAITING: tr_noop('Waiting to switch'),
}

GREEN = rl.Color(128, 216, 166, 255)
YELLOW = rl.Color(218, 202, 37, 255)
ORANGE = rl.Color(255, 140, 40, 255)
GREY = rl.Color(166, 166, 166, 255)

STATE_COLOR = {
  JetlinkState.DISCONNECTED: GREY,
  JetlinkState.UNCOMPILED: ORANGE,
  JetlinkState.READY: GREEN,
  JetlinkState.LOADING: YELLOW,
  JetlinkState.ACTIVE: GREEN,
  JetlinkState.FAILED: ORANGE,
  JetlinkState.WAITING: GREEN,
}


def link_mode() -> str:
  """The setting as stored, one of LINK_MODES: shown even when jetlink cannot
  run, so a link that will not start can be turned off."""
  try:
    index = int(ui_state.params.get(LINK_PARAM) or 0)
  except (TypeError, ValueError):
    index = 0
  return LINK_MODES[index] if 0 <= index < len(LINK_MODES) else "off"


def link_toggle_meaningful() -> bool:
  """Offered wherever jetlink is present, except beside a USB GPU, which runs
  the big model itself; and while the setting is on, whatever else."""
  if ui_state.usbgpu:
    return link_mode() != "off"
  return ui_state.jetlink is not None or link_mode() != "off"


def link_status() -> str:
  """One line: what is on the comma's USB-C port right now. jetlink knows a
  Jetson or a phone and the transport says which; below that only the CC pin
  speaks (a cable with a host behind it, not what the host is)."""
  jetlink = ui_state.jetlink
  if jetlink is None:
    return ""
  if jetlink.present:
    return tr('Jetlink connected: {v0}.').format(v0=jetlink.transport)
  if jetlink.port is None:
    return ""
  return tr('No device on the USB port.') if jetlink.port == "empty" else tr('Device on USB port (not responding to Jetlink yet).')


def progress() -> tuple[str, float, str] | None:
  """(stage, 0..1, message) while jetlink is working, else None. The message
  names the cable once jetlink counts enough link drops to blame it."""
  jetlink = ui_state.jetlink
  p = jetlink.progress if jetlink is not None else None
  if not p:
    return None
  stage = str(p.get('stage', ''))
  if stage in ('', 'ready'):
    return None
  msg = str(p.get('msg', ''))
  if drops := p.get('drops'):
    hint = tr('Check the cable or app') if jetlink.mode == 'ios' else tr('Check the cable')
    msg = tr('{v0}, {v1} ({v2} disconnects)').format(v0=msg, v1=hint, v2=drops)
  return stage, float(p.get('frac', 0.0)), msg


def _alka_on() -> bool:
  try:
    return ui_state.params.get_bool("dp_lat_alka")
  except Exception:
    return False

def status_note(model_name: str | None = None) -> str:
  """The failover story (zoompilot's Model Status note, jetlink half). `model_name`:
  the model that will drive when the caller knows better than the snapshot (following
  the far end, it is the far end's loaded model, not the comma's default)."""
  view = ui_state.jetlink_view
  if view is None:
    return ""
  big_name = (model_name if model_name and model_name != none_text() else None) or view.model or tr('Large model')
  state = ui_state.jetlink_state
  if state in (JetlinkState.FAILED, JetlinkState.UNCOMPILED):
    if view.reason:
      return tr('Large model unavailable: {v0}. Using the small model.').format(v0=view.reason)
    return tr('Large model unavailable. Using the small model.')
  if state == JetlinkState.WAITING:
    if _alka_on():
      return tr('{model} is ready. With ALKA on, cancelling cruise will not switch: turn cruise main off for about 1 second, or stop and shift to P/N/R.').format(model=big_name)
    return tr('{model} is ready. Cancel cruise to switch, then re-enable after about 1 second.').format(model=big_name)
  if state == JetlinkState.LOADING:
    return tr('Using the small model until the large model is ready.')
  if not view.ready:
    if view.standin:
      return tr('Using {v0} until {v1} is ready.').format(v0=view.standin, v1=big_name)
    return tr('Using {v0} once Jetlink is ready.').format(v0=big_name)
  return tr('Using {v0}; automatically fall back to the small model if disconnected, then switch back after recovery.').format(v0=big_name)


class JetlinkIcons:
  """The three icon textures at one size, and how a state draws."""

  def __init__(self, width: int, height: int):
    self.green = gui_app.texture(f"{ICON_DIR}/jetlink_green.png", width, height)
    self.default = gui_app.texture(f"{ICON_DIR}/jetlink.png", width, height)
    self.orange = gui_app.texture(f"{ICON_DIR}/jetlink_orange.png", width, height)

  def for_state(self, state: JetlinkState) -> tuple[Union[rl.Texture, None], float]:
    """(texture, opacity) for a state; None while disconnected. Loading pulses,
    waiting is a steady dim green, failed is orange, ready/active solid green."""
    if state == JetlinkState.DISCONNECTED:
      return None, 0.0
    if state == JetlinkState.LOADING:
      return self.default, 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    if state == JetlinkState.WAITING:
      return self.green, 0.5
    if state in (JetlinkState.UNCOMPILED, JetlinkState.FAILED):
      return self.orange, 1.0
    return self.green, 1.0


def tint(opacity: float) -> rl.Color:
  return rl.Color(255, 255, 255, int(255 * max(0.0, min(1.0, opacity))))

# Event strings are published in English and translated by both alert renderers.
JETLINK_ALERT_TEXT = (
  tr_noop('Switching large model'),
  tr_noop('Large model active'),
  tr_noop('Large model ready'),
  tr_noop('Turn cruise main off, then re-enable to switch'),
  tr_noop('Large model disconnected'),
  tr_noop('Using the small model'),
)

# ALKA-dependent model-ready event descriptions.
JETLINK_ALKA_ALERT_TEXT = (
  tr_noop('ALKA on: turn cruise main off or shift to P to switch'),
  tr_noop('Cancel cruise to switch'),
)
