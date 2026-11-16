"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import re
import time

import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

# ==========================================
# 移植自 openpilot-dp0111pre selfdrive/ui/mici/onroad/hud_renderer.py
#   - 兩側方向燈 / 盲區邊條（_update_dp_indicator_side_state、_draw_edge_warnings）
#   - TDX 路況事件橫幅（_draw_tdx_info）
# 未移植：前車距離倒三角（_draw_lead_info）、紅綠燈減速中 banner（需要 dp 的 longitudinalPlanDP）
# ==========================================

# --- 測試模式總開關 (True: 開啟 UI 測試假數據 / False: 讀取真實車輛數據) ---
DEBUG_TEST_UI = False

# --- 方向燈與盲區閃爍頻率設定（與 dp 相同）---
DP_INDICATOR_BLINK_RATE_FAST = int(gui_app.target_fps * 0.25)
DP_INDICATOR_BLINK_RATE_STD = int(gui_app.target_fps * 0.5)
DP_INDICATOR_COLOR_BSM = rl.Color(255, 204, 0, 220)      # 盲區黃色
DP_INDICATOR_COLOR_BLINKER = rl.Color(0, 255, 0, 220)    # 方向燈綠色
DP_EDGE_BAR_WIDTH = 20
DP_EDGE_GAP = 2

# --- TDX ---
# dp 與 sp 的 tdx 資料結構不同：
#   dp：tdx.events[]（eventType: UInt8、isCurrentSection: Bool），UI 以 eventType 對照中文標籤
#   sp：tdx.roadEvent（isActive、description、distance、sectionId），description 由 tdx.py 組成：
#       「目前:<type>|<說明>/<type>|<說明>…」或「前方:…」（目前 / 前方每 10 秒輪替，同 dp）
#   → 從 description 解析出 type，再套用 dp 的標籤，顯示結果與 dp 相同（例：「前方:交通事故施工事件」）。
TDX_EVENT_TYPE_LABEL = {
  1: '交通事故', 2: '施工事件', 3: '壅塞事件',
  4: '道路管制', 5: '天氣異常', 8: '其他異常',
}
TDX_UNKNOWN_LABEL = '[其他]'
_TDX_TYPE_RE = re.compile(r'(?:^|/)(\d+)\|')
# sp 的 tdx 在 services.py 是頻率 0（on-demand），SubMaster 的 alive 永遠為 True，不能像 dp 用 sm.alive['tdx'] 判斷。
# tdx.py 每秒送一次（CALC_INTERVAL = 1.0）；GPS 失效 / 隧道 / 沒有路段資料時不送 → 超過這個秒數沒收到就隱藏。
TDX_STALE_S = 3.0


def format_tdx_event(description: str) -> str:
  """sp roadEvent.description → dp 格式的顯示文字。解析不到類型時退回原始說明。"""
  loc, sep, rest = description.partition(':')
  if not sep:
    return description
  labels: list[str] = []
  for t in _TDX_TYPE_RE.findall(rest):
    label = TDX_EVENT_TYPE_LABEL.get(int(t), TDX_UNKNOWN_LABEL)
    if label not in labels:
      labels.append(label)
  return f"{loc}:{''.join(labels)}" if labels else description


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()

    self._font_bold_dp: rl.Font = gui_app.font(FontWeight.BOLD)

    # --- TDX ---
    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    # --- 邊緣閃爍狀態變數 ---
    self._dp_indicator_show_left = False
    self._dp_indicator_show_right = False
    self._dp_indicator_count_left = 0
    self._dp_indicator_count_right = 0
    self._dp_indicator_color_left = rl.Color(0, 0, 0, 0)
    self._dp_indicator_color_right = rl.Color(0, 0, 0, 0)

  @staticmethod
  def _update_dp_indicator_side_state(blinker_state, bsm_state, show_prev, count_prev):
    """處理單邊的閃爍與顏色邏輯（同 dp）"""
    show = show_prev
    count = count_prev
    color = rl.Color(0, 0, 0, 0)

    if not blinker_state and not bsm_state:
      show = False
      count = 0
    else:
      count += 1

    if bsm_state and blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_FAST == 0 else show
      color = DP_INDICATOR_COLOR_BSM
    elif blinker_state:
      show = not show if count % DP_INDICATOR_BLINK_RATE_STD == 0 else show
      color = DP_INDICATOR_COLOR_BLINKER
    elif bsm_state:
      show = True
      color = DP_INDICATOR_COLOR_BSM
    else:
      show = False

    return show, count, color

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.tdx_event_active = False
      self.tdx_event_desc = ""
      self._dp_indicator_show_left = False
      self._dp_indicator_show_right = False
      self._dp_indicator_count_left = 0
      self._dp_indicator_count_right = 0
      return

    # --- 讀取 TDX 狀態：只採用本次上路後收到、且 TDX_STALE_S 內有更新的資料 ---
    tdx_fresh = (sm.recv_frame['tdx'] >= ui_state.started_frame and
                 time.monotonic() - sm.recv_time['tdx'] < TDX_STALE_S)
    if tdx_fresh:
      try:
        road_event = sm['tdx'].roadEvent
        self.tdx_event_active = bool(road_event.isActive) and len(road_event.description) > 0
        self.tdx_event_desc = format_tdx_event(road_event.description) if self.tdx_event_active else ""
      except Exception:
        self.tdx_event_active = False
        self.tdx_event_desc = ""
    else:
      self.tdx_event_active = False
      self.tdx_event_desc = ""

    car_state = sm['carState']
    if DEBUG_TEST_UI:
      # 自定義左右邊條測試狀態 (預設：左方向燈 / 右盲區)
      left_blinker, left_bsm, right_blinker, right_bsm = True, False, False, True
      self.tdx_event_active = True
      self.tdx_event_desc = format_tdx_event("前方:1|測試事故/2|測試施工/1|測試事故")
    else:
      left_blinker, left_bsm = car_state.leftBlinker, car_state.leftBlindspot
      right_blinker, right_bsm = car_state.rightBlinker, car_state.rightBlindspot

    # --- 更新兩側方向燈與盲區閃爍狀態 ---
    self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_color_left = \
      self._update_dp_indicator_side_state(left_blinker, left_bsm, self._dp_indicator_show_left, self._dp_indicator_count_left)
    self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_color_right = \
      self._update_dp_indicator_side_state(right_blinker, right_bsm, self._dp_indicator_show_right, self._dp_indicator_count_right)

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)

    # TDX 警告（畫面中央）
    self._draw_tdx_info(rect)

    # --- 最後繪製：自帶雙閃爍頻率的方向燈與盲區邊條，確保圖層在最上方 ---
    self._draw_edge_warnings(rect)

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected

  def _draw_edge_warnings(self, rect: rl.Rectangle) -> None:
    """繪製兩側方向燈與盲區警示（圓角、垂直置中後上移 20px，同 dp）"""
    bar_height = int(rect.height * 0.60)
    y_pos = int(rect.y + (rect.height - bar_height) / 2) - 20

    if self._dp_indicator_show_left:
      left_rect = rl.Rectangle(int(rect.x), y_pos, DP_EDGE_BAR_WIDTH, bar_height)
      rl.draw_rectangle_rounded(left_rect, 0.75, 20, self._dp_indicator_color_left)

    if self._dp_indicator_show_right:
      right_rect = rl.Rectangle(int(rect.x + rect.width - DP_EDGE_BAR_WIDTH), y_pos, DP_EDGE_BAR_WIDTH, bar_height)
      rl.draw_rectangle_rounded(right_rect, 0.75, 20, self._dp_indicator_color_right)

  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """TDX 路況警告：單向循環跑馬燈，寬度貼齊兩側方向燈 / 盲區邊條，字體 70（同 dp）"""
    if not self.tdx_event_active or not self.tdx_event_desc:
      return

    font_size = 70
    text_size = measure_text_cached(self._font_bold_dp, self.tdx_event_desc, font_size)

    # 背景框橫跨左右邊條的內部空間
    bg_width = rect.width - (DP_EDGE_BAR_WIDTH * 2) - (DP_EDGE_GAP * 2)
    bg_x = rect.x + DP_EDGE_BAR_WIDTH + DP_EDGE_GAP

    bg_padding_y = 15
    bg_height = text_size.y + bg_padding_y * 2

    # 垂直位置：螢幕中央偏上 20px
    pos_y = rect.y + (rect.height - text_size.y) / 2 - 20
    bg_y = pos_y - bg_padding_y
    bg_rect = rl.Rectangle(bg_x, bg_y, bg_width, bg_height)

    rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 180))
    alpha = 150 + int(60 * math.sin(rl.get_time() * 5))
    rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))

    text_padding = 20
    max_text_width = bg_width - (text_padding * 2)

    if text_size.x > max_text_width:
      rl.begin_scissor_mode(int(bg_rect.x), int(bg_rect.y), int(bg_rect.width), int(bg_rect.height))

      extra_width = text_size.x - max_text_width
      scroll_speed = 80.0
      scroll_duration = extra_width / scroll_speed

      # 單向跑馬燈：起點停 2 秒 → 向左滑 → 終點停 2 秒 → 歸零
      start_pause = 2.0
      end_pause = 2.0
      total_cycle = start_pause + scroll_duration + end_pause
      cycle_time = rl.get_time() % total_cycle

      if cycle_time < start_pause:
        offset = 0.0
      elif cycle_time < start_pause + scroll_duration:
        offset = extra_width * (cycle_time - start_pause) / scroll_duration
      else:
        offset = extra_width

      draw_x = bg_x + text_padding - offset
      rl.draw_text_ex(self._font_bold_dp, self.tdx_event_desc, rl.Vector2(draw_x, pos_y), font_size, 0, rl.WHITE)
      rl.end_scissor_mode()
    else:
      draw_x = bg_x + (bg_width - text_size.x) / 2
      rl.draw_text_ex(self._font_bold_dp, self.tdx_event_desc, rl.Vector2(draw_x, pos_y), font_size, 0, rl.WHITE)
