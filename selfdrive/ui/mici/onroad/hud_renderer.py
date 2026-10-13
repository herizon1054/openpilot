import math
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds

# --- 測試模式總開關 (True: 開啟 UI 測試假數據 / False: 讀取真實車輛數據) ---
DEBUG_TEST_UI = False

# --- 方向燈與盲區閃爍頻率設定 ---
DP_INDICATOR_BLINK_RATE_FAST = int(gui_app.target_fps * 0.25)
DP_INDICATOR_BLINK_RATE_STD = int(gui_app.target_fps * 0.5)
DP_INDICATOR_COLOR_BSM = rl.Color(255, 204, 0, 220)      # 盲區黃色
DP_INDICATOR_COLOR_BLINKER = rl.Color(0, 255, 0, 220)    # 方向燈綠色


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._turn_intent_rotation_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, self._turn_intent_rotation_filter.x, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])
    if left or right:
      # pre lane change
      if not self._pre:
        self._turn_intent_rotation_filter.x = self.FADE_IN_ANGLE if left else -self.FADE_IN_ANGLE

      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
      self._turn_intent_rotation_filter.update(0)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out and rotate away
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

      if self._turn_intent_direction == 0:
        # unknown. missed pre frame?
        self._turn_intent_rotation_filter.update(0)
      else:
        self._turn_intent_rotation_filter.update(self._turn_intent_direction * self.FADE_IN_ANGLE)
    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)
      self._turn_intent_rotation_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    self.tdx_event_active: bool = False
    self.tdx_event_desc: str = ""

    # --- 前車距離變數 ---
    self.lead_dist: str = "-"
    self.lead_dist_raw: float = 0.0

    # --- 邊緣閃爍狀態變數 ---
    self._dp_indicator_show_left = False
    self._dp_indicator_show_right = False
    self._dp_indicator_count_left = 0
    self._dp_indicator_count_right = 0
    self._dp_indicator_color_left = rl.Color(0, 0, 0, 0)
    self._dp_indicator_color_right = rl.Color(0, 0, 0, 0)

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._turn_intent = TurnIntent()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 9, 44)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_dp_indicator_side_state(self, blinker_state, bsm_state, show_prev, count_prev):
    """處理單邊的閃爍與顏色邏輯"""
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
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      self.tdx_event_active = False
      self.tdx_event_desc = ""
      self.lead_dist = "-"
      self.lead_dist_raw = 0.0
      
      self._dp_indicator_show_left = False
      self._dp_indicator_show_right = False
      return

    # --- 讀取雷達狀態 ---
    radar_state = sm['radarState']
    if radar_state.leadOne.status:
      self.lead_dist_raw = radar_state.leadOne.dRel
      self.lead_dist = f"{self.lead_dist_raw:.0f}m"
    else:
      self.lead_dist_raw = 0.0
      self.lead_dist = "-"

    # --- 讀取 TDX 狀態 (背景接收事件) ---
    # 只採用本次上路後收到、且仍在更新中的資料（tdxd 1Hz；GPS 失效 / 隧道 / 啟動中不會發布）
    if sm.recv_frame['tdx'] >= ui_state.started_frame and sm.alive['tdx']:
      try:
        events = sm['tdx'].events
        self.tdx_event_active = len(events) > 0

        EVENT_TYPE_LABEL = {
            1: '交通事故', 2: '施工事件', 3: '壅塞事件',
            4: '道路管制', 5: '天氣異常', 8: '其他異常'
        }

        if self.tdx_event_active:
            # "目前" 與 "前方" 都顯示
            loc_part = "目前" if events[0].isCurrentSection else "前方"

            unique_labels = []
            for evt in events:
                label = EVENT_TYPE_LABEL.get(evt.eventType, '[其他]')
                if label not in unique_labels:
                    unique_labels.append(label)

            self.tdx_event_desc = f"{loc_part}:{''.join(unique_labels)}"
        else:
            self.tdx_event_desc = ""

      except Exception:
        pass
    else:
      self.tdx_event_active = False
      self.tdx_event_desc = ""

    controls_state = sm['controlsState']
    car_state = sm['carState']

    # --- UI 測試模式覆寫區塊 ---
    if DEBUG_TEST_UI:
      self.lead_dist_raw = 105.0
      self.lead_dist = "105m"
      # 若要測試無鎖定前車，可將上方兩行改為：
      # self.lead_dist_raw = 0.0
      # self.lead_dist = "-"
      
      # 自定義左右邊條測試狀態 (這裡預設：左方向燈 / 右盲區)
      test_left_blinker = True
      test_left_bsm = False
      test_right_blinker = False
      test_right_bsm = True
    else:
      test_left_blinker = car_state.leftBlinker
      test_left_bsm = car_state.leftBlindspot
      test_right_blinker = car_state.rightBlinker
      test_right_bsm = car_state.rightBlindspot

    # --- 更新兩側方向燈與盲區閃爍狀態 ---
    self._dp_indicator_show_left, self._dp_indicator_count_left, self._dp_indicator_color_left = \
      self._update_dp_indicator_side_state(test_left_blinker, test_left_bsm,
                                           self._dp_indicator_show_left, self._dp_indicator_count_left)
    
    self._dp_indicator_show_right, self._dp_indicator_count_right, self._dp_indicator_color_right = \
      self._update_dp_indicator_side_state(test_right_blinker, test_right_bsm,
                                           self._dp_indicator_show_right, self._dp_indicator_count_right)

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    
    if self.is_cruise_set:
      self._draw_set_speed(rect)

    # 顯示置中的動態立體倒三角形與前車距離
    self._draw_lead_info(rect)
    
    # 繪製 TDX 警告
    self._draw_tdx_info(rect)

    # --- 最後繪製：自帶雙閃爍頻率的方向燈與盲區邊條 ---
    # 確保圖層順序在最上方，不被裁切
    self._draw_edge_warnings(rect)

  def _draw_edge_warnings(self, rect: rl.Rectangle) -> None:
    """繪製兩側方向燈與盲區警示 (加入圓角效果並垂直置中)"""
    bar_width = 20  
    bar_height = int(rect.height * 0.60) 
    
    # 將 Y 軸座標改為垂直置中，並向上微調 20px
    y_pos = int(rect.y + (rect.height - bar_height) / 2) - 20 

    if self._dp_indicator_show_left:
      left_rect = rl.Rectangle(int(rect.x), y_pos, bar_width, bar_height)
      # 圓角參數設回 0.75
      rl.draw_rectangle_rounded(left_rect, 0.75, 20, self._dp_indicator_color_left)

    if self._dp_indicator_show_right:
      right_rect = rl.Rectangle(int(rect.x + rect.width - bar_width), y_pos, bar_width, bar_height)
      # 圓角參數設回 0.75
      rl.draw_rectangle_rounded(right_rect, 0.75, 20, self._dp_indicator_color_right)

  def _draw_lead_info(self, rect: rl.Rectangle) -> None:
    """繪製置中的圓角立體「倒三角形」與前車距離"""
    
    # 當無鎖定前車的時候，直接不顯示圖示與數字
    if self.lead_dist == "-":
      return

    # 尺寸設定：高度 40，寬度 45
    bar_h = 40.0
    bar_w = 45.0
    
    # 垂直位置與水平置中
    pos_y = int(rect.y + rect.height - 39)
    bar_y = pos_y - bar_h / 2
    bar_x = rect.x + (rect.width - bar_w) / 2
    
    dist_text = self.lead_dist
    dist_font_size = 40
    dist_size = measure_text_cached(self._font_bold, dist_text, dist_font_size)
    
    # 距離數據顯示在圖形的左邊 (保持 15px 間距)
    text_x = bar_x - dist_size.x - 15  
    text_y = pos_y - dist_size.y / 2
        
    dist_color = rl.WHITE
    
    # 判斷是否處於警告狀態
    is_warning = (self.lead_dist_raw < 15.0)

    if is_warning:
      # 同步 TDX 頻道的呼吸燈頻率與透明度
      alpha = 150 + int(60 * math.sin(time.time() * 5))
      center_color = rl.Color(255, 100, 100, alpha) 
      edge_color = rl.Color(180, 0, 0, alpha)       
      dist_color = rl.Color(255, 100, 100, 255)
    else:
      # 安全狀態：綠色恆亮
      center_color = rl.Color(150, 255, 150, 255) 
      edge_color = rl.Color(0, 180, 0, 255)       
      dist_color = rl.Color(128, 216, 166, 255)
        
    # 繪製文字陰影與主體
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x + 2, text_y + 2), dist_font_size, 0, rl.Color(0, 0, 0, 150))
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x, text_y), dist_font_size, 0, dist_color)

    # --- 定義繪製圓角三角形的內部演算法 (避免半透明圖層重疊加深) ---
    def draw_rounded_triangle_poly(x, y, w, h, r, scale, color):
        cx = x + w / 2
        # 因為是倒三角形，視覺形心偏上方
        cy = y + h / 3
        
        # 內縮後的原始角點中心 (改為倒三角形：左上 -> 底部中心 -> 右上)
        # 確保順序為逆時針 (Counter-Clockwise)
        v1 = rl.Vector2(x + r, y + r)                  # 左上
        v2 = rl.Vector2(x + w / 2, y + h - r)          # 底部中心
        v3 = rl.Vector2(x + w - r, y + r)              # 右上
        
        # 依據 scale 比例推算實際點位
        c1 = rl.Vector2(cx + (v1.x - cx) * scale, cy + (v1.y - cy) * scale)
        c2 = rl.Vector2(cx + (v2.x - cx) * scale, cy + (v2.y - cy) * scale)
        c3 = rl.Vector2(cx + (v3.x - cx) * scale, cy + (v3.y - cy) * scale)
        scaled_r = r * scale

        # 計算兩點之間的垂直法向量
        def get_normal(pA, pB):
            dx = pB.x - pA.x
            dy = pB.y - pA.y
            length = math.hypot(dx, dy)
            return (-dy / length, dx / length) if length > 0 else (0, -1)

        n12 = get_normal(c1, c2)
        n23 = get_normal(c2, c3)
        n31 = get_normal(c3, c1)

        points = []
        
        # 計算弧線點的輪廓
        def add_arc(center, n_start, n_end):
            angle_start = math.atan2(n_start[1], n_start[0])
            angle_end = math.atan2(n_end[1], n_end[0])
            
            # 強制確保角度為逆時針方向 (Pyray 螢幕座標 Y 軸向下，角度應為遞減)
            while angle_end > angle_start:
                angle_end -= math.pi * 2
                
            steps = 6 # 圓角的平滑度 (分 6 段)
            for i in range(steps + 1):
                t = i / steps
                a = angle_start + (angle_end - angle_start) * t
                points.append(rl.Vector2(center.x + math.cos(a) * scaled_r, center.y + math.sin(a) * scaled_r))

        # 依序加入左上、底部中心、右下三個圓角的弧線
        add_arc(c1, n31, n12)
        add_arc(c2, n12, n23)
        add_arc(c3, n23, n31)

        # 透過中心點向輪廓畫出放射狀三角形 (Triangle Fan)，完美填充且不重疊
        center_pt = rl.Vector2(cx, cy)
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            rl.draw_triangle(center_pt, p1, p2, color)

    # --- 依序繪製底層、邊緣、中心 ---
    base_r = 6.0 # 控制三角形圓角平滑度的半徑參數
    
    # 背景黑底 (放大約 1.15 倍形成外圍邊框)
    draw_rounded_triangle_poly(bar_x, bar_y, bar_w, bar_h, base_r, 1.15, rl.Color(0, 0, 0, 180))
    # 底部暗色邊緣 (原始大小)
    draw_rounded_triangle_poly(bar_x, bar_y, bar_w, bar_h, base_r, 1.0, edge_color)
    # 頂部亮色中心 (縮小 0.7 倍製造立體感)
    draw_rounded_triangle_poly(bar_x, bar_y, bar_w, bar_h, base_r, 0.7, center_color)

  def _draw_tdx_info(self, rect: rl.Rectangle) -> None:
    """TDX 路況警告：單向循環跑馬燈，寬度貼齊兩側 BSM 與方向燈，字體 70"""
    if not self.tdx_event_active or not self.tdx_event_desc:
      return

    font_size = 70
    text_size = measure_text_cached(self._font_bold, self.tdx_event_desc, font_size)
    
    # 這裡呼應 BSM 邊緣條的設定
    bar_width = 20  
    gap = 2  # 距離兩側邊條的 2px 安全微調間距

    # 計算背景框的固定寬度與位置 (完美橫跨左右邊條的內部空間)
    bg_width = rect.width - (bar_width * 2) - (gap * 2)
    bg_x = rect.x + bar_width + gap
    
    bg_padding_y = 15
    bg_height = text_size.y + bg_padding_y * 2
    
    # 垂直位置：螢幕中央偏下
    pos_y = rect.y + (rect.height - text_size.y) / 2 - 20
    bg_y = pos_y - bg_padding_y

    bg_rect = rl.Rectangle(bg_x, bg_y, bg_width, bg_height)
    
    # 畫背景
    rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 180))
    alpha = 150 + int(60 * math.sin(time.time() * 5))
    rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(220, 50, 50, alpha))
    
    # 處理文字跑馬燈 (文字顯示的有效範圍)
    text_padding = 20
    max_text_width = bg_width - (text_padding * 2)
    is_overflow = text_size.x > max_text_width

    if is_overflow:
      rl.begin_scissor_mode(int(bg_rect.x), int(bg_rect.y), int(bg_rect.width), int(bg_rect.height))

      extra_width = text_size.x - max_text_width
      scroll_speed = 80.0     
      scroll_duration = extra_width / scroll_speed
      
      # --- 單向跑馬燈時間設定 ---
      start_pause = 2.0  # 停在起點的時間 (秒)
      end_pause = 2.0    # 停在終點的時間 (秒)
      total_cycle = start_pause + scroll_duration + end_pause

      cycle_time = time.time() % total_cycle

      if cycle_time < start_pause:
        # 第一階段：停在開頭
        offset = 0.0
      elif cycle_time < start_pause + scroll_duration:
        # 第二階段：單向向左滑動
        progress = (cycle_time - start_pause) / scroll_duration
        offset = extra_width * progress
      else:
        # 第三階段：停在結尾，時間一到直接歸零重新開始
        offset = extra_width

      draw_x = bg_x + text_padding - offset
      rl.draw_text_ex(self._font_bold, self.tdx_event_desc, rl.Vector2(draw_x, pos_y), font_size, 0, rl.WHITE)

      rl.end_scissor_mode()
    else:
      # 文字沒超過時，在整條橫幅中置中
      draw_x = bg_x + (bg_width - text_size.x) / 2
      rl.draw_text_ex(self._font_bold, self.tdx_event_desc, rl.Vector2(draw_x, pos_y), font_size, 0, rl.WHITE)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED and not ui_state.dp_alka_active:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y

    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    max_text = tr("MAX")
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + 25, y + FONT_SIZES.set_speed - 7 + 4),
      FONT_SIZES.max_speed,
      0,
      max_color,
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
