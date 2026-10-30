from dragonpilot.system.ui.lib.multilang import tr as jetlink_tr
from cereal import log
from dragonpilot import jetlink_adapter
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog, BigConfirmationDialog

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class JetlinkModeToggle(BigMultiParamToggle):
  """Keep parameter values stable; localize only the displayed option."""

  def _draw_content(self, btn_y: float):
    label = jetlink_tr("Off") if self.value == "off" else {"usb": "USB", "ios": "iOS"}.get(self.value, self.value)
    self._sub_label.set_text(label)
    super()._draw_content(btn_y)


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable openpilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)
    # dp - jetlink: zoompilot's mici AcceleratorLinkToggle, as a plain param toggle (off / usb / ios); parked only
    self._jetlink_toggle = JetlinkModeToggle("Jetlink", "JetlinkLink", ["off", "usb", "ios"])
    self._jetlink_toggle.set_enabled(lambda: ui_state.is_offroad())

    self._delete_jetlink = BigButton(jetlink_tr('Delete Jetlink downloads'), jetlink_tr('comma cache only'))
    self._delete_jetlink.set_click_callback(self._confirm_delete_jetlink)
    self._delete_jetlink.set_enabled(lambda: ui_state.is_offroad() and jetlink_adapter.can_delete_downloads(ui_state.params))

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      enable_openpilot,
      self._jetlink_toggle,
      self._delete_jetlink,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _confirm_delete_jetlink(self):
    if not ui_state.is_offroad() or not jetlink_adapter.can_delete_downloads(ui_state.params):
      return
    try:
      count, size = jetlink_adapter.downloaded_cache_info()
    except Exception as e:
      gui_app.push_widget(BigDialog('Jetlink', jetlink_tr(str(e))))
      return
    if not count:
      gui_app.push_widget(BigDialog('Jetlink', jetlink_tr('No downloaded Jetlink models.')))
      return
    title = jetlink_tr('Slide to delete comma cache\n{count} files / {size} MiB').format(count=count, size=size // (1024 * 1024))
    gui_app.push_widget(BigConfirmationDialog(title,
      gui_app.texture('icons_mici/settings/network/new/trash.png', 54, 64), self._delete_jetlink_cache, red=True))

  def _delete_jetlink_cache(self):
    if not ui_state.is_offroad():
      gui_app.push_widget(BigDialog('Jetlink', jetlink_tr('Park and turn Jetlink off before deleting models.')))
      return
    try:
      count, size = jetlink_adapter.delete_downloaded_models(ui_state.params)
      text = jetlink_tr('Deleted {count} files; freed {size} MiB.').format(count=count, size=size // (1024 * 1024))
    except Exception as e:
      text = jetlink_tr('Delete failed: {reason}').format(reason=jetlink_tr(str(e)))
    gui_app.push_widget(BigDialog('Jetlink', text))

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    self._jetlink_toggle._load_value()  # dp - jetlink

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
