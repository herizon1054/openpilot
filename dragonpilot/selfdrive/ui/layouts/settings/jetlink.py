"""
dp - jetlink settings panel (tici).

Ported from the jetlink parts of zoompilot's sunnypilot models panel
(selfdrive/ui/sunnypilot/layouts/settings/models.py): the Jetlink setting
(Off / USB / iOS, offroad only, turns ADB off) and the failover note.

dp has no sunnypilot model manager. Its big-model menu is built from the same
sunnypilot chestnut catalog the Jetlink phone/Mac apps list, and from what the
far end reports in its hello (the model it has loaded and the ones it has
built). The default is to follow the far end: whatever is picked on the phone
drives. The live connection rows are dp's: zoompilot folds them into the
descriptions.
"""
from dragonpilot.system.ui.lib.multilang import tr, tr_noop

import threading
import time

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import multiple_button_item, text_item, toggle_item, button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller
from dragonpilot import jetlink_adapter
from dragonpilot.selfdrive.ui import jetlink_ui
from dragonpilot.selfdrive.ui.jetlink_ui import LINK_MODES, LINK_MODE_TITLES, LINK_PARAM, none_text

DESCRIPTION = (tr_noop('Run the large model on a device connected to comma USB-C (USB for Jetson / Linux PC / Mac; iOS for iPhone). Enabling disables ADB. Change modes only while offroad. If the link drops or lags, comma falls back to its small model.'))

PROGRESS_STAGES = {"connect": tr_noop('Connecting'), "fetch": tr_noop('Downloading'), "upload": tr_noop('Uploading'), "build": tr_noop('Building'), "failed": tr_noop('Failed')}


def _bool_text(v: bool) -> str:
  return tr('Yes') if v else tr('No')


class JetlinkLayout(Widget):
  def __init__(self):
    super().__init__()
    self._last_description = None
    # what the far end reported, read at most once a second (files and params, not per frame)
    self._far = {'loaded': none_text(), 'built': "", 'model': none_text(), 'following': True, 'at': 0.0}

    self._link_item = multiple_button_item(
      "Jetlink", lambda: tr(DESCRIPTION),
      buttons=[lambda m=m: tr(LINK_MODE_TITLES[m]) for m in LINK_MODES],
      selected_index=LINK_MODES.index(jetlink_ui.link_mode()),
      button_width=250, callback=self._on_link_mode)

    self._state_item = text_item(lambda: tr('Connection status'), lambda: tr(jetlink_ui.STATE_TEXT[ui_state.jetlink_state]),
                                 description=lambda: jetlink_ui.status_note(self._far['model']))
    self._transport_item = text_item(lambda: tr('Transport'), self._transport)
    self._present_item = text_item(lambda: tr('External device online'), lambda: _bool_text(bool(ui_state.jetlink and ui_state.jetlink.present)))
    self._port_item = text_item(lambda: tr('USB port'), self._port)
    self._model_item = text_item(lambda: tr('Driving model'), self._model)
    self._ready_item = text_item(lambda: tr('External engine built'), lambda: _bool_text(bool(ui_state.jetlink and ui_state.jetlink.ready)))
    self._progress_item = text_item(lambda: tr('Progress'), self._progress)
    self._reason_item = text_item(lambda: tr('Unavailable reason'), lambda: (ui_state.jetlink.reason if ui_state.jetlink and ui_state.jetlink.reason else none_text()))
    self._adb_item = text_item("ADB", lambda: tr('Disabled by Jetlink') if ui_state.adb_blocked else (tr('On') if ui_state.params.get_bool("AdbEnabled") else tr('Off')))

    # an iPhone on a direct cable is asked to charge from the comma; off by default, some lose the link once powered
    self._charge_item = toggle_item(lambda: tr('Power iPhone from comma'),
                                    lambda: tr('Power a directly connected iPhone from comma. Off by default: some phones disconnect when powered.'),
                                    initial_state=ui_state.params.get_bool(jetlink_ui.KEYS.charge_phone),
                                    callback=lambda v: ui_state.params.put_bool(jetlink_ui.KEYS.charge_phone, bool(v)),
                                    enabled=ui_state.is_offroad)

    # dp: the big-model menu (follow the far end, or a catalog model) and the far end's own models
    self._menu: list[dict] = []
    self._menu_labels: dict[str, str | None] = {}
    self._refreshing = False
    self._refresh_result = lambda: ""
    self._source_item = button_item(lambda: tr('Large model source'), lambda: tr('Select'), description=self._source_description, callback=self._open_source_dialog)
    self._far_end_item = text_item(lambda: tr('External device model'), self._far_end_loaded, description=self._far_end_built)
    self._refresh_item = button_item(lambda: tr('Refresh model list'), lambda: tr('Updating...') if self._refreshing else tr('Refresh'),
                                     description=lambda: self._refresh_result() or tr('Fetch the sunnypilot model list (same as the Jetlink app) to identify models on the phone.'),
                                     callback=self._refresh_catalog, enabled=lambda: not self._refreshing)

    self._items = [self._link_item, self._state_item, self._transport_item, self._present_item, self._port_item,
                   self._source_item, self._far_end_item, self._model_item, self._ready_item, self._progress_item,
                   self._refresh_item, self._reason_item, self._charge_item, self._adb_item]
    self._scroller = Scroller(self._items, line_separator=True, spacing=0)

  # -- values -------------------------------------------------------------------

  @staticmethod
  def _transport() -> str:
    j = ui_state.jetlink
    return j.transport if j is not None else none_text()

  @staticmethod
  def _port() -> str:
    j = ui_state.jetlink
    if j is None or j.port is None:
      return none_text()
    return tr('No device') if j.port == "empty" else tr('Device present')

  def _model(self) -> str:
    """The model that will drive: the far end's loaded one when following, else the comma's pick."""
    return self._far['model']

  @staticmethod
  def _progress() -> str:
    p = jetlink_ui.progress()
    if p is None:
      return none_text()
    stage, frac, msg = p
    label = tr(PROGRESS_STAGES.get(stage, stage))
    pct = f" {frac * 100:.0f}%" if 0.0 < frac < 1.0 else ""
    return f"{label}{pct} {msg}".strip()

  # -- the model menu -------------------------------------------------------------

  @staticmethod
  def _following_now() -> bool:
    return jetlink_adapter.follows_far_end(ui_state.params.get(jetlink_adapter.KEYS.big_model))

  def _following(self) -> bool:
    return self._far['following']

  def _refresh_far_end(self) -> None:
    now = time.monotonic()
    if now - self._far['at'] < 1.0:
      return
    loaded, built = jetlink_adapter.far_end_models()
    menu = jetlink_adapter.model_menu(ui_state.params)
    by_sha = {r['sha256']: r['name'] for r in menu if r['sha256']}
    name = (lambda s: by_sha.get(s, s[:16]) if s else None)
    following = self._following_now()
    j = ui_state.jetlink
    if following and loaded:
      model = name(loaded)
    elif j is not None:
      model = j.active_model or j.model or j.default_model or none_text()
    else:
      model = none_text()
    self._far = {'loaded': name(loaded) or tr('Not reported'),
                 'built': (tr('Built on external device: ') + "、".join(name(b) for b in built)) if built else tr('The external device has not reported built models (shown after connecting).'),
                 'model': model, 'following': following, 'at': now}

  def _source_description(self) -> str:
    if self._far['following']:
      return tr('Follow the external device: use the model selected in the phone (or Jetson / Mac) app. No separate download on comma.')
    slot = ui_state.params.get(jetlink_adapter.KEYS.big_model) or {}
    return tr('Selected on comma: {v0}. If missing on the external device, comma downloads and uploads it for building while parked.').format(v0=slot.get('displayName') or slot.get('ref', '')[:10])

  def _far_end_loaded(self) -> str:
    return self._far['loaded']

  def _far_end_built(self) -> str:
    return self._far['built']

  def _open_source_dialog(self):
    self._menu = jetlink_adapter.model_menu(ui_state.params)
    loaded, _ = jetlink_adapter.far_end_models()
    follow_label = tr('Follow external device (current: {v0})').format(v0=jetlink_adapter.model_name(loaded, ui_state.params) or tr('Unknown'))
    self._menu_labels = {follow_label: None}
    for row in self._menu:
      notes = [n for n, on in ((tr('Loaded'), row['loaded']), (tr('Built on external device'), row['built']), (tr('Downloaded on comma'), row['downloaded'])) if on]
      label = row['name'] + (f"（{'、'.join(notes)}）" if notes else "")
      self._menu_labels[label] = row['ref']
    slot = ui_state.params.get(jetlink_adapter.KEYS.big_model)
    current = follow_label if self._following_now() else next((k for k, v in self._menu_labels.items() if v == (slot or {}).get('ref')), "")
    dialog = MultiOptionDialog(tr('Select large model'), list(self._menu_labels.keys()), current,
                               callback=lambda result: self._on_source_selected(result, dialog))
    gui_app.push_widget(dialog)

  def _on_source_selected(self, result, dialog):
    if result != DialogResult.CONFIRM or dialog.selection not in self._menu_labels:
      return
    if not ui_state.is_offroad():
      return  # the far end switches engines only parked, as the link setting does
    jetlink_adapter.pick_model(self._menu_labels[dialog.selection], ui_state.params)
    self._far['at'] = 0.0

  def _refresh_catalog(self):
    if self._refreshing:
      return
    self._refreshing = True
    self._refresh_result = lambda: tr('Updating...')

    def work():
      try:
        n = jetlink_adapter.refresh_catalog(ui_state.params)
        self._refresh_result = lambda n=n: tr('Updated: {v0} large models.').format(v0=n)
      except Exception as e:
        self._refresh_result = lambda error=str(e): tr('Refresh failed: {v0}').format(v0=error)
      finally:
        self._refreshing = False

    threading.Thread(target=work, daemon=True).start()

  # -- events -------------------------------------------------------------------

  def _on_link_mode(self, index: int):
    if not ui_state.is_offroad():
      # the gadget changes only once the car is parked; put the buttons back
      self._link_item.action_item.set_selected_button(LINK_MODES.index(jetlink_ui.link_mode()))
      return
    ui_state.params.put(LINK_PARAM, int(index))
    if LINK_MODES[index] != "off" and ui_state.params.get_bool("AdbEnabled"):
      ui_state.params.put_bool("AdbEnabled", False)

  def _update_state(self):
    self._refresh_far_end()
    j = ui_state.jetlink
    mode = jetlink_ui.link_mode()
    on = mode != "off"
    self._link_item.action_item.set_selected_button(LINK_MODES.index(mode))
    self._link_item.action_item.set_enabled(ui_state.is_offroad())
    for item in (self._state_item, self._transport_item, self._present_item, self._port_item,
                 self._model_item, self._ready_item, self._progress_item, self._source_item, self._far_end_item, self._refresh_item):
      item.set_visible(on or (j is not None and j.present))
    self._source_item.action_item.set_enabled(ui_state.is_offroad())
    self._reason_item.set_visible(bool(j is not None and j.reason))
    self._charge_item.set_visible(mode == "ios")
    self._charge_item.action_item.set_state(ui_state.params.get_bool(jetlink_ui.KEYS.charge_phone))

    status = jetlink_ui.link_status()
    description = f"{tr(DESCRIPTION)} {status}".strip()
    if j is None and not on:
      description += tr(' (Jetlink package not found on this device)')
    if description != self._last_description:
      self._last_description = description
      self._link_item.set_description(description)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    ui_state.update_params()

  def _render(self, rect):
    self._scroller.render(rect)
