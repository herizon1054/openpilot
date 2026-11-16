import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.sunnypilot.onroad.radar_ui.elements import RadarData


class RadarModelWrapper:
  def __init__(self, model_renderer):
    self._model = model_renderer

  def map_to_screen(self, dRel, yRel, z=0.0):
    try:
      screen_pt = self._model._map_to_screen(dRel, yRel, z)
    except Exception as e:
      return None
    return screen_pt

  def map(self, x, y, z=0.0):
    offset = self._model._path_offset_z
    return self.map_to_screen(x, y, z + offset)


class RadarUiRenderer(Widget):
  RADAR_UI_OFF = 0
  RADAR_UI_POINT = 1
  RADAR_UI_POINTINFO = 2

  def __init__(self, model_renderer):
    super().__init__()
    self.is_metric = False
    self.radar_ui_mode = 0
    self.label_size = 40
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

    self.model = RadarModelWrapper(model_renderer)
    self.radar = RadarData()

  def _update_state(self) -> None:
    self.radar_ui_mode = ui_state.radar_ui
    self.is_metric = ui_state.is_metric

    sm = ui_state.sm
    if sm.updated['radarTracks']:
      radarpoint = sm['radarTracks'].points
      vEgo = sm['carState'].vEgo
      self.radar.Points = self.radar.update(radarpoint)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self.radar_ui_mode:
      return

    # Draw OFF
    if self.radar_ui_mode == 0:
      return

    # Draw Points
    if self.radar_ui_mode > 0:
      self._draw_radar_ui_point(rect)

  def _draw_radar_ui_point(self, rect: rl.Rectangle) -> None:
    if self.model is None or not self.radar.Points:
      return

    for point in self.radar.Points:
      # 轉換成螢幕座標
      screen_pt = self.model.map(point.dRel, point.yRel)
      if screen_pt is None:
        continue

      # 取得螢幕座標
      x = int(screen_pt[0])
      y = int(screen_pt[1])

      if self.radar_ui_mode > 0:
        rl.draw_circle(x, y, self.label_size / 2, point.color)

      if self.radar_ui_mode > 1:
        # 顯示資訊：ID, dRel, yRel, vRel
        text = f"ID:{point.trackId}\n" + f"d:{point.dRel:.1f} m\n" + f"y:{point.yRel:.1f} m\n" + f"v:{point.vRel:.1f} m/s"

        size = measure_text_cached(self._font_bold, text, self.label_size, 0)
        text_width = size.x  # 寬度
        text_height = size.y  # 高度

        # 偏移到圓點正下方、水平置中
        text_x = x - text_width // 2
        text_y = y + self.label_size / 2  # 6 是圓點半徑

        # 畫背景
        # rl.draw_rectangle_rounded(rl.Rectangle(text_x, text_y, text_width + 4, text_height + 4), 0.2, 8, point.color)
        # draw_font_ex 需要傳入 font、文字、位置、大小、顏色
        rl.draw_text_ex(self._font_bold, text, rl.Vector2(text_x, text_y), self.label_size, 0, rl.WHITE)
