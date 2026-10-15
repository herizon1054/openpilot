import math
import shutil
import threading
import time
import pyray as rl
from dataclasses import dataclass
from cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
#from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'
Congestion = custom.Tdx.Congestion
TRAFFIC_STOP_IDX = int(custom.LongitudinalPlanDP.LongitudinalPlanSource.trafficStop)

# dp - 紅綠燈減速中 banner（基準尺寸 x1.5 再縮 25%）：中央車速下方的高度、靠左對齊；TDX 照常顯示，
# 與 TDX「前方車速」重疊時橫幅自動下移到其下方
TS_HOLD_TIME = 0.5          # 秒；braking 訊號短暫掉落時維持顯示，避免閃爍
TS_FONT_SIZE = 108
TS_BANNER_HEIGHT = 169
TS_TOP_OFFSET = 5           # 相對 header_height 的位移（km/h 單位文字底緣約在 header 內）
TS_PADDING_X = 45
TS_ICON_TEXT_GAP = 27
TS_LIGHT_RADIUS = 27
TS_LIGHT_GAP = 14
TS_BORDER = 7
# 左緣：content rect 左側即 30px 寬的方向燈 / 盲區邊條（augmented_road_view UI_BORDER_SIZE），
# 再留 20px（>= 10px 間隔）→ 距螢幕左緣 50px
TS_LEFT_MARGIN = 20
TS_TDX_GAP_Y = 10           # 與 TDX 前方車速框的垂直間隔

# DP Perf Constants (字體恢復為 50)
PERF_FONT_SIZE = 50
PERF_PADDING = 14
PERF_MARGIN_BOTTOM = 0   # 設定為 0 以貼齊底部
PERF_ITEM_GAP = 50
PERF_BG_COLOR = rl.Color(0, 0, 0, 160)


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    
    # 前車距離字串與數值變數
    self.lead_dist: str = "-"
    self.lead_dist_raw: float = 0.0

    # dp: 右下角資訊列 - 方向盤角度（取代原本的駕駛/自動控制狀態文字）
    self.steering_angle_deg: float = 0.0

    # --- 紅綠燈減速中 ---
    self.ts_braking: bool = False
    self._ts_last_braking_t: float = -1e9

    # --- TDX 路況預警變數 ---
    self.tdx_next_speed: int = -1
    self.tdx_congestion = Congestion.unknown
    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    #self._torque_bar = TorqueBar(scale=4.0)

    # Lincoln perf overlay init
    self._perf_font = gui_app.font(FontWeight.MEDIUM)
    self._perf_stats: dict[str, str] = {"cpu_temp": "-", "mem_usage": "-", "disk_free": "-"}
    self._perf_lock = threading.Lock()
    self._perf_running = True
    self._perf_thread = threading.Thread(target=self._perf_update_loop, daemon=True)
    self._perf_thread.start()

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self.lead_dist = "-"
      self.lead_dist_raw = 0.0
      self.steering_angle_deg = 0.0

      # 重置 TDX 變數，避免殘留上次熄火前的資料
      self._reset_tdx()
      self.ts_braking = False
      self._ts_last_braking_t = -1e9
      return

    self._update_traffic_stop(sm)

    controls_state = sm['controlsState']
    car_state = sm['carState']
    radar_state = sm['radarState']

    # 紀錄真實距離並格式化字串
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    # --- 讀取 TDX 狀態 ---
    # 只採用本次上路後收到、且仍在更新中的資料（tdxd 1Hz；GPS 失效 / 隧道 / 啟動中不會發布）
    if sm.recv_frame['tdx'] >= ui_state.started_frame and sm.alive['tdx']:
      tdx = sm['tdx']
      self.tdx_next_speed = tdx.nextSpeed
      self.tdx_congestion = tdx.congestion
      events = tdx.events
      self.tdx_event_active = len(events) > 0
      if self.tdx_event_active:
        loc_part = "目前" if events[0].isCurrentSection else "前方"
        self.tdx_event_desc = f"{loc_part}: {' / '.join(str(e.description) for e in events)}"
      else:
        self.tdx_event_desc = ""
    else:
      self._reset_tdx()

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    # dp: 右下角資訊列 - 方向盤角度
    self.steering_angle_deg = car_state.steeringAngleDeg

  def _update_traffic_stop(self, sm) -> None:
    """紅綠燈減速中：讀 longitudinalPlanDP.targets[trafficStop].braking，且 openpilot 正在控制縱向"""
    braking = False
    if sm.recv_frame['longitudinalPlanDP'] >= ui_state.started_frame and sm.alive['longitudinalPlanDP']:
      targets = sm['longitudinalPlanDP'].targets
      if len(targets) > TRAFFIC_STOP_IDX:
        braking = bool(targets[TRAFFIC_STOP_IDX].braking) and sm['carControl'].longActive

    now = rl.get_time()
    if braking:
      self._ts_last_braking_t = now
    self.ts_braking = (now - self._ts_last_braking_t) < TS_HOLD_TIME

  def _reset_tdx(self) -> None:
    self.tdx_next_speed = -1
    self.tdx_congestion = Congestion.unknown
    self.tdx_event_active = False
    self.tdx_event_desc = ""

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)

    # --- TDX 雙行顯示資訊 ---
    self._draw_tdx_info(rect)

    # --- 紅綠燈減速中（靠左；與 TDX 前方車速重疊時自動下移，畫在 TDX 之後）---
    if self.ts_braking:
      self._draw_traffic_stop(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    #if ui_state.sm['controlsState'].lateralControlState.which() != 'angleState':
      #self._torque_bar.render(rect)

    # Draw performance info at bottom
    self._draw_performance_info(rect)

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed

  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """繪製高公局即時路況與事件 (車速維持原位，事件貼齊下方單行跑馬燈)"""
    if self.tdx_next_speed <= 0 and not self.tdx_event_active:
      return

    bg_padding_x = 45
    bg_padding_y = 20

    # 1. 決定車速的文字顏色
    if self.tdx_congestion == Congestion.smooth:
      speed_color = rl.Color(128, 216, 166, 255)
    elif self.tdx_congestion == Congestion.slow:
      speed_color = rl.Color(255, 204, 0, 255)
    elif self.tdx_congestion == Congestion.congested:
      speed_color = rl.Color(255, 100, 100, 255)
    else:
      speed_color = rl.WHITE

    # ==========================================
    # 第一行: 車速 (維持在原位：頂部列下方，僅顯示前方車速)
    # ==========================================
    if 0 < self.tdx_next_speed <= 150:
      speed_text = f"前方車速: {self.tdx_next_speed} km/h"
      tdx_speed_font_size = 90
      speed_size = measure_text_cached(self._font_semi_bold, speed_text, tdx_speed_font_size)
      
      # 恢復原本的固定 Y 軸位置
      top_y = rect.y + UI_CONFIG.header_height + 25 
      speed_x = rect.x + rect.width / 2 - speed_size.x / 2
      
      bg_rect = rl.Rectangle(speed_x - bg_padding_x, top_y - bg_padding_y, speed_size.x + bg_padding_x * 2, speed_size.y + bg_padding_y * 2)
      rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 160))
      
      rl.draw_text_ex(self._font_semi_bold, speed_text, rl.Vector2(speed_x, top_y), tdx_speed_font_size, 0, speed_color)

    # ==========================================
    # 第二行: 事件 (貼齊下方 Performance 列，單行跑馬燈)
    # ==========================================
    if self.tdx_event_active and self.tdx_event_desc:
      # 計算 Performance 列的位置以決定事件區底部
      perf_bar_height = PERF_FONT_SIZE + 2 * PERF_PADDING
      perf_bar_y = rect.y + rect.height - perf_bar_height - PERF_MARGIN_BOTTOM

      tdx_event_font_size = 75
      max_text_width = rect.width - 200

      text = self.tdx_event_desc
      text_size = measure_text_cached(self._font_semi_bold, text, tdx_event_font_size)
      text_width = text_size.x
      line_height = text_size.y

      display_width = min(text_width, max_text_width)

      event_bg_height = line_height + bg_padding_y * 2
      event_y = perf_bar_y - event_bg_height - 15  # 留 15px 間距避免太黏

      event_x = rect.x + rect.width / 2 - display_width / 2
      event_bg_rect = rl.Rectangle(
          event_x - bg_padding_x,
          event_y,
          display_width + bg_padding_x * 2,
          event_bg_height
      )

      # 呼吸燈閃爍警告背景
      alpha = 130 + int(50 * math.sin(time.time() * 5))
      rl.draw_rectangle_rounded(event_bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))

      draw_y = event_y + bg_padding_y

      if text_width > max_text_width:
        # 文字超長 -> 裁切 + 跑馬燈來回捲動
        rl.begin_scissor_mode(int(event_bg_rect.x), int(event_bg_rect.y), int(event_bg_rect.width), int(event_bg_rect.height))

        extra_width = text_width - max_text_width
        scroll_speed = 80.0     # 跑馬燈捲動速度 (像素/秒)
        scroll_duration = extra_width / scroll_speed
        pause_duration = 2.0    # 在兩側邊界停留的時間 (秒)

        cycle_time = time.time() % ((scroll_duration + pause_duration) * 2)

        # 四階段: 左停 -> 往左捲 -> 右停 -> 往右捲回
        if cycle_time < pause_duration:
          offset = 0.0
        elif cycle_time < pause_duration + scroll_duration:
          progress = (cycle_time - pause_duration) / scroll_duration
          offset = extra_width * progress
        elif cycle_time < pause_duration * 2 + scroll_duration:
          offset = extra_width
        else:
          progress = (cycle_time - pause_duration * 2 - scroll_duration) / scroll_duration
          offset = extra_width * (1 - progress)

        draw_x = event_x - offset
        rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(draw_x, draw_y), tdx_event_font_size, 0, rl.WHITE)

        rl.end_scissor_mode()
      else:
        # 文字沒超長 -> 直接單行置中顯示
        rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(event_x, draw_y), tdx_event_font_size, 0, rl.WHITE)

  def _tdx_speed_rect(self, rect: rl.Rectangle):
    """TDX「前方車速」背景框位置；計算方式與 _draw_tdx_info 第一行相同（字 90、padding 45/20）。未顯示時回傳 None"""
    if not (0 < self.tdx_next_speed <= 150):
      return None
    speed_text = f"前方車速: {self.tdx_next_speed} km/h"
    size = measure_text_cached(self._font_semi_bold, speed_text, 90)
    top_y = rect.y + UI_CONFIG.header_height + 25
    speed_x = rect.x + rect.width / 2 - size.x / 2
    return rl.Rectangle(speed_x - 45, top_y - 20, size.x + 45 * 2, size.y + 20 * 2)

  def _draw_traffic_stop(self, rect: rl.Rectangle) -> None:
    """中央車速下方的高度、靠左（讓開左側方向燈邊條）：紅綠燈圖示 + 「紅綠燈減速中」
    與 TDX「前方車速」框重疊時，下移到該框下方 TS_TDX_GAP_Y"""
    text = "紅綠燈減速中"
    text_size = measure_text_cached(self._font_bold, text, TS_FONT_SIZE)
    icon_w = TS_LIGHT_RADIUS * 6 + TS_LIGHT_GAP * 4
    icon_h = TS_LIGHT_RADIUS * 2 + TS_LIGHT_GAP * 2
    banner_w = TS_PADDING_X * 2 + icon_w + TS_ICON_TEXT_GAP + text_size.x

    x = rect.x + TS_LEFT_MARGIN
    y = rect.y + UI_CONFIG.header_height + TS_TOP_OFFSET

    tdx_rect = self._tdx_speed_rect(rect)
    if tdx_rect is not None and rl.check_collision_recs(rl.Rectangle(x, y, banner_w, TS_BANNER_HEIGHT), tdx_rect):
      y = tdx_rect.y + tdx_rect.height + TS_TDX_GAP_Y

    banner = rl.Rectangle(x, y, banner_w, TS_BANNER_HEIGHT)
    rl.draw_rectangle_rounded(banner, 0.3, 10, rl.Color(0, 0, 0, 190))
    border_alpha = 170 + int(85 * math.sin(rl.get_time() * 6))
    rl.draw_rectangle_rounded_lines_ex(banner, 0.3, 10, TS_BORDER, rl.Color(255, 70, 70, border_alpha))

    # 橫式紅綠燈：外框 + 紅（呼吸閃爍）/ 黃 / 綠（暗）
    icon_x = x + TS_PADDING_X
    icon_y = y + (TS_BANNER_HEIGHT - icon_h) / 2
    rl.draw_rectangle_rounded(rl.Rectangle(icon_x, icon_y, icon_w, icon_h), 0.5, 8, rl.Color(30, 30, 30, 240))
    cy = int(icon_y + icon_h / 2)
    red_alpha = 180 + int(75 * math.sin(rl.get_time() * 6))
    light_colors = (rl.Color(255, 60, 60, red_alpha), rl.Color(90, 75, 20, 255), rl.Color(20, 70, 35, 255))
    for i, color in enumerate(light_colors):
      cx = int(icon_x + TS_LIGHT_GAP + TS_LIGHT_RADIUS + i * (TS_LIGHT_RADIUS * 2 + TS_LIGHT_GAP))
      rl.draw_circle(cx, cy, TS_LIGHT_RADIUS, color)

    text_x = icon_x + icon_w + TS_ICON_TEXT_GAP
    text_y = y + (TS_BANNER_HEIGHT - text_size.y) / 2
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(text_x, text_y), TS_FONT_SIZE, 0, rl.WHITE)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE

    max_text = tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)

  def _draw_performance_info(self, rect: rl.Rectangle) -> None:
    if rect.width <= 0 or rect.height <= 0:
      return

    stats = self._get_perf_stats()
    steering_angle_text = f"{tr('Steering Angle')} {self.steering_angle_deg:.0f}°"

    lead_dist = self.lead_dist
    cpu_temp = stats.get("cpu_temp", "-")
    mem_usage = stats.get("mem_usage", "-")
    disk_free = stats.get("disk_free", "-")

    items = [
      f"{tr('Lead Dist')} {lead_dist}",
      f"{tr('CPU Temp')} {cpu_temp}",
      f"{tr('Memory')} {mem_usage}",
      f"{tr('Disk Free')} {disk_free}",
      f"{steering_angle_text}",
    ]

    measurements = [measure_text_cached(self._perf_font, text, PERF_FONT_SIZE) for text in items]

    bar_width = max(rect.width - 20, 0)
    bar_height = PERF_FONT_SIZE + 2 * PERF_PADDING

    bar_x = rect.x + (rect.width - bar_width) / 2
    bar_y = rect.y + rect.height - bar_height - PERF_MARGIN_BOTTOM
    minimum_y = rect.y + PERF_MARGIN_BOTTOM
    if bar_y < minimum_y:
      bar_y = minimum_y

    rl.draw_rectangle_rounded(
      rl.Rectangle(bar_x, bar_y, bar_width, bar_height),
      0.2,
      8,
      PERF_BG_COLOR,
    )

    slot_width = bar_width / len(items)
    text_y = bar_y + PERF_PADDING

    for i, (text, measurement) in enumerate(zip(items, measurements)):
      cursor_x = bar_x + (i * slot_width) + (slot_width - measurement.x) / 2
      text_color = rl.WHITE
      if i == 0 and self.lead_dist != "-" and self.lead_dist_raw < 15.0:
        text_color = rl.Color(255, 188, 0, 200)

      rl.draw_text_ex(self._perf_font, text, rl.Vector2(cursor_x, text_y), PERF_FONT_SIZE, 0, text_color)

  def _get_perf_stats(self) -> dict[str, str]:
    with self._perf_lock:
      return dict(self._perf_stats)

  def _perf_update_loop(self) -> None:
    time.sleep(10)
    while self._perf_running:
      stats = {
        "cpu_temp": self._read_cpu_temp(),
        "mem_usage": self._read_mem_usage(),
        "disk_free": self._read_disk_free(),
      }
      with self._perf_lock:
        self._perf_stats.update(stats)
      for _ in range(10):
        if not self._perf_running:
          return
        time.sleep(0.1)

  @staticmethod
  def _read_cpu_temp() -> str:
    path = "/sys/class/thermal/thermal_zone0/temp"
    try:
      with open(path) as f:
        temp_c = int(f.read().strip()) / 1000.0
        return f"{temp_c:.0f}°C"
    except Exception:
      return "-"

  @staticmethod
  def _read_mem_usage() -> str:
    try:
      total_kb = None
      available_kb = None
      with open("/proc/meminfo") as f:
        for line in f:
          if line.startswith("MemTotal:"):
            total_kb = float(line.split()[1])
          elif line.startswith("MemAvailable:"):
            available_kb = float(line.split()[1])
          if total_kb is not None and available_kb is not None:
            break
      if total_kb and available_kb:
        used_pct = (total_kb - available_kb) / total_kb * 100.0
        used_pct = min(max(used_pct, 0.0), 100.0)
        return f"{used_pct:.0f}%"
    except Exception:
      pass
    return "-"

  @staticmethod
  def _read_disk_free() -> str:
    try:
      usage = shutil.disk_usage("/data")
      free_gb = usage.free / (1024 ** 3)
      if free_gb >= 1.0:
        return f"{free_gb:.1f}G" 
      free_mb = usage.free / (1024 ** 2)
      return f"{free_mb:.0f}MB"  
    except Exception:
      return "-"
