import pyray as rl

from collections import deque
import time

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state

HISTORY_SEC = 10.0  # ← 可調整的時間窗（秒）
MAX_POINTS = 500  # 保險用（避免爆記憶體）


class TrendRenderer(Widget):
  def __init__(self):
    super().__init__()
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

    self.last_a_ego = 0.0
    self.last_accel_cmd = 0.0
    self.last_accel_out = 0.0
    self.ts = deque(maxlen=MAX_POINTS)
    self.a_ego = deque(maxlen=MAX_POINTS)
    self.accel_cmd = deque(maxlen=MAX_POINTS)
    self.accel_out = deque(maxlen=MAX_POINTS)

  def _update_state(self) -> None:
    sm = ui_state.sm
    # 更新 latest value（有更新才換）
    updated = False
    if sm.updated['carState']:
      self.last_a_ego = sm['carState'].aEgo
      updated = True

    if sm.updated['carControl']:
      self.last_accel_cmd = sm['carControl'].actuators.accel
      updated = True

    if sm.updated['carOutput']:
      self.last_accel_out = sm['carOutput'].actuatorsOutput.accel
      updated = True

    # 這一幀什麼都不做
    if not updated:
      return

    now = time.monotonic()
    # 同步 append
    self.ts.append(now)
    self.a_ego.append(self.last_a_ego)
    self.accel_cmd.append(self.last_accel_cmd)
    self.accel_out.append(self.last_accel_out)
    self.trim_history(now)

  def trim_history(self, now):
    while self.ts and (now - self.ts[0]) > HISTORY_SEC:
      self.ts.popleft()
      self.a_ego.popleft()
      self.accel_cmd.popleft()
      self.accel_out.popleft()

  def _render(self, rect: rl.Rectangle) -> None:
    # === 計算受限畫布 ===
    is_wide = rect.width >= 2000
    draw_w = 600 if is_wide else 460
    draw_h = 210
    draw_x = int(rect.x + rect.width - rect.width // 4 - draw_w // 2)
    draw_y = int(rect.y + 40)

    # === 背景 + 外框 ===
    rl.draw_rectangle(draw_x, draw_y, draw_w, draw_h, rl.Color(30, 30, 30, 180))
    rl.draw_rectangle_lines(draw_x, draw_y, draw_w, draw_h, rl.WHITE)

    # === 標題（正中上方） ===
    title_text = tr("Accel")
    title_size = 16
    centered_title_x = draw_x + draw_w // 2 - (len(title_text) * title_size // 4)
    rl.draw_text_ex(self._font_bold, title_text, rl.Vector2(centered_title_x, draw_y + 5), title_size, 0, rl.WHITE)

    # === Y 軸刻度（虛線 + 數值在虛線下方） ===
    v_min = -3.0
    v_max = 3.0
    num_ticks = 7
    tick_color = rl.WHITE
    tick_offset = 2  # 數值在虛線下方的偏移
    inset = 10  # 虛線往內縮

    for i in range(num_ticks):
      v = v_min + i * (v_max - v_min) / (num_ticks - 1)
      if v == v_min or v == v_max:
        continue  # 不顯示最小最大刻度
      y = int(draw_y + draw_h - (v - v_min) / (v_max - v_min) * draw_h)
      y = max(draw_y + 2, min(draw_y + draw_h - 12, y))

      # 虛線
      dash_length = 5
      gap_length = 3
      x_start = draw_x + inset
      x_end = draw_x + draw_w - inset
      x = x_start
      while x < x_end:
        rl.draw_line(x, y, min(x + dash_length, x_end), y, tick_color)
        x += dash_length + gap_length

      # 刻度數值在虛線下方
      text_y = min(y + tick_offset, draw_y + draw_h - 12)
      rl.draw_text_ex(self._font_semi_bold, f"{v:.1f}", rl.Vector2(draw_x + inset, text_y), 12, 0, tick_color)

    # === 畫線工具函式 ===
    def map_x(t):
      if len(self.ts) < 2:
        return draw_x
      t0 = self.ts[0]
      t1 = self.ts[-1]
      return int(draw_x + (t - t0) / (t1 - t0) * draw_w)

    def map_y(v):
      v = max(v_min, min(v_max, v))
      return int(draw_y + draw_h - (v - v_min) / (v_max - v_min) * draw_h)

    def draw_series(values, color):
      prev_x = None
      prev_y = None
      for t, v in zip(self.ts, values):
        x = map_x(t)
        y = map_y(v)
        x = max(draw_x, min(draw_x + draw_w, x))
        y = max(draw_y, min(draw_y + draw_h, y))
        if prev_x is not None:
          rl.draw_line(prev_x, prev_y, x, y, color)
        prev_x = x
        prev_y = y

    # === 畫三條線 ===
    draw_series(self.accel_cmd, rl.YELLOW)
    draw_series(self.accel_out, rl.BLUE)
    draw_series(self.a_ego, rl.GREEN)

    # === 右下角 legend ===
    padding = 4  # 左右內距
    font_size = 24  # 字體大小變數
    line_height = int(font_size * 1.2)

    legend_items = [
      ("Accel_EGO", self.a_ego[-1] if self.a_ego else 0.0, rl.GREEN),
      ("Accel_CMD", self.accel_cmd[-1] if self.accel_cmd else 0.0, rl.YELLOW),
      ("Accel_OUT", self.accel_out[-1] if self.accel_out else 0.0, rl.BLUE),
    ]

    # 計算 legend 寬高自適應
    name_width = max(measure_text_cached(self._font_semi_bold, f"{name}: ", font_size).x for name, _, _ in legend_items) + 5
    value_width = max(measure_text_cached(self._font_semi_bold, f"{+9.99} m/s²", font_size).x for _ in legend_items) + 5
    # 注意這裡 value_width 用了最大可能長度 + 單位，避免跳動

    box_width = int(name_width + value_width + padding * 2)
    box_height = int(line_height * len(legend_items) + padding)

    # 背景半透明方塊
    legend_x = int(draw_x + draw_w - box_width - 10)
    legend_y = int(draw_y + draw_h - box_height - 5)
    rl.draw_rectangle(legend_x, legend_y, box_width, box_height, rl.Color(30, 30, 30, 180))

    # 畫文字
    for i, (name, val, color) in enumerate(legend_items):
      y_pos = legend_y + i * line_height

      # 左側名稱靠左（固定寬度容器，不受右側數值影響）
      rl.draw_text_ex(self._font_semi_bold, f"{name}:", rl.Vector2(legend_x + padding, y_pos), font_size, 0, color)

      # 右側數值靠右 + 單位（使用固定最大寬度 value_width）
      val_text = f"{val:+5.2f} m/s²"
      val_x = legend_x + box_width - padding - measure_text_cached(self._font_semi_bold, val_text, font_size).x
      rl.draw_text_ex(self._font_semi_bold, val_text, rl.Vector2(val_x, y_pos), font_size, 0, color)
