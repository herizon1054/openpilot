import math
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached


class TdxUiRenderer(Widget):

  def __init__(self):
    super().__init__()
    # 設定字體大小與字型，補全粗體與半粗體資源宣告避免屬性遺失
    self.label_size = 80
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)

    # --- TDX 路況預警變數 ---
    self.tdx_speed: int = -1
    self.tdx_next_speed: int = -1
    self.tdx_status: str = "unknown"
    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    self.frame_count: int = 0

  def _update_state(self) -> None:
    self.frame_count += 1
    sm = ui_state.sm

    if sm.updated['tdx']:
      tdx_msg = sm['tdx']
      self.tdx_speed = tdx_msg.trafficStatus.speed
      self.tdx_next_speed = tdx_msg.trafficStatus.nextSpeed
      self.tdx_status = str(tdx_msg.trafficStatus.status)
      self.tdx_event_active = tdx_msg.roadEvent.isActive
      self.tdx_event_desc = tdx_msg.roadEvent.description

  def _render(self, rect: rl.Rectangle) -> None:
    """強制繪製 TDX 即時路況(前方車速)與事件跑馬燈"""

    bg_padding_x = 45
    bg_padding_y = 20

    if self.tdx_status == "freeFlow":
      speed_color = rl.Color(128, 216, 166, 255)
    elif self.tdx_status == "heavyTraffic":
      speed_color = rl.Color(255, 204, 0, 255)
    elif self.tdx_status == "severeCongestion":
      speed_color = rl.Color(255, 100, 100, 255)
    else:
      speed_color = rl.WHITE

    # ==========================================
    # 前方車速: 置中顯示
    # ==========================================
    if self.tdx_next_speed > 0:
      speed_text = f"前方車速: {self.tdx_next_speed} km/h"
      tdx_speed_font_size = self.label_size * 1.2
      speed_size = measure_text_cached(self._font_bold, speed_text, tdx_speed_font_size)

      # 將 Y 軸改為中間位置 (目前速度下方)，並置中顯示 X 軸
      top_y = rect.y + (rect.height / 2) - 150
      speed_x = rect.x + rect.width / 2 - speed_size.x / 2

      bg_rect = rl.Rectangle(
        speed_x - bg_padding_x, top_y - bg_padding_y,
        speed_size.x + bg_padding_x * 2, speed_size.y + bg_padding_y * 2,
      )
      rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 160))
      rl.draw_text_ex(self._font_bold, speed_text, rl.Vector2(speed_x, top_y), tdx_speed_font_size, 0, speed_color)

    # ==========================================
    # 事件跑馬燈: 單向滾動 (從頭到尾)
    # ==========================================
    if self.tdx_event_active and len(self.tdx_event_desc) > 2:
      # SP 專有的底部狀態列留白高度
      bottom_offset = 30

      tdx_event_font_size = self.label_size
      max_text_width = rect.width - 200

      text = self.tdx_event_desc
      text_size = measure_text_cached(self._font_bold, text, tdx_event_font_size)
      text_width = text_size.x
      line_height = text_size.y

      display_width = min(text_width, max_text_width)

      event_bg_height = line_height + bg_padding_y * 2

      # 往下貼齊，保留間距避免與 SP 底部 Developer UI 重疊
      event_y = rect.y + rect.height - bottom_offset - event_bg_height - 10

      event_x = rect.x + rect.width / 2 - display_width / 2
      event_bg_rect = rl.Rectangle(
        event_x - bg_padding_x, event_y,
        display_width + bg_padding_x * 2, event_bg_height,
      )

      # 呼吸燈閃爍警告背景
      alpha = 130 + int(50 * math.sin(self.frame_count * 0.25))
      rl.draw_rectangle_rounded(event_bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))

      draw_y = event_y + bg_padding_y

      if text_width > max_text_width:
        # 文字超長 -> 裁切 + 單向跑馬燈 (從頭到尾，再重頭)
        rl.begin_scissor_mode(int(event_bg_rect.x), int(event_bg_rect.y), int(event_bg_rect.width), int(event_bg_rect.height))

        extra_width = text_width - max_text_width
        scroll_speed = 80.0  # 每秒移動像素數

        # 20Hz 影格換算：捲動幀數與停留幀數 (2.0 秒 = 40 幀)
        scroll_frames = max(1, int((extra_width / scroll_speed) * 20))
        pause_start_frames = 40  # 在最左邊停留 40 幀 (2 秒)
        pause_end_frames = 40    # 在最右邊停留 40 幀 (2 秒)

        # 單次循環總幀數
        cycle_frames = pause_start_frames + scroll_frames + pause_end_frames
        cycle_frame = self.frame_count % cycle_frames

        if cycle_frame < pause_start_frames:
          # 階段一：停在最左側
          offset = 0.0
        elif cycle_frame < pause_start_frames + scroll_frames:
          # 階段二：向左捲動 (計算當前幀進度百分比)
          progress = (cycle_frame - pause_start_frames) / scroll_frames
          offset = extra_width * progress
        else:
          # 階段三：停在最右側
          offset = extra_width

        draw_x = event_x - offset
        rl.draw_text_ex(self._font_bold, text, rl.Vector2(draw_x, draw_y), tdx_event_font_size, 0, rl.WHITE)

        rl.end_scissor_mode()
      else:
        rl.draw_text_ex(self._font_bold, text, rl.Vector2(event_x, draw_y), tdx_event_font_size, 0, rl.WHITE)