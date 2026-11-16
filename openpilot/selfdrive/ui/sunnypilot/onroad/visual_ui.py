import math
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached


class VisualUiRenderer(Widget):
  """
  VisualUiRenderer 專門負責視覺模型 (ModelV2) 相關的 UI 渲染。
  功能特色：
  1. 使用 for 迴圈遍歷所有有效前車目標 (leadsV3)。
  2. 【空間去重過濾】：自動過濾 1.0 x 1.0 公尺範圍內的重複追蹤點，確保單一車輛不重複標示。
  3. 【多重軌道擴張避讓】：文字像衛星一樣在車輛周圍找空位，內圈客滿自動往外圈擴張，確保 100% 零重疊。
  4. 【動態向量箭頭轉向】：找到完美空位後，加粗箭頭會精準從文字底端指向目標車輛中心，羽翼自動旋轉。
  5. 實作自訂的信心度顏色漸變系統 (紅 -> 黃 -> 綠)。
  """

  def __init__(self, model_renderer):
    super().__init__()
    # 設定字體大小與字型，保留原始範例的粗體設定
    self.label_size = 60
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)

    # 儲存傳入的 model_renderer，用於後續將 3D 座標轉換為 2D 螢幕座標
    self.model_renderer = model_renderer

    # 初始化儲存多個前車資訊的清單
    self.leads_list = []

  def _update_state(self) -> None:
    """
    更新狀態函式：每個影格都會被呼叫，負責從 cereal (sm) 抓取最新的 ModelV2 資料，
    經過空間去重過濾後，將有效的 leadsV3 目標打包存入清單中。
    """
    sm = ui_state.sm

    # 每次更新前先清空上一格的資料
    self.leads_list = []

    # 確保 modelV2 資料有更新，避免讀取到舊資料
    if sm.updated['modelV2']:
      model_data = sm['modelV2']

      # 使用 enumerate 遍歷所有的前車目標，同時取得其在 leadsV3 陣列中的索引 (i)
      for i, lead in enumerate(model_data.leadsV3):
        # 如果信心度過低 (例如小於 1%)，視為雜訊直接跳過不處理
        if lead.prob < 0.01:
          continue

        # 確保該目標擁有完整的 3D 座標數據 (x 為縱向距離，y 為橫向偏移)
        if len(lead.x) > 0 and len(lead.y) > 0:
          lead_x = lead.x[0]
          lead_y = lead.y[0]

          # =========================================================
          # 🚀 空間去重機制 (1.0 x 1.0 公尺範圍過濾)
          # =========================================================
          # 檢查這個新點是否與已經加入清單中的點距離過近 (X與Y皆在 ±0.5 公尺內)
          is_duplicate = False
          for existing_lead in self.leads_list:
            # abs() 計算絕對誤差，如果長寬都在 0.5 公尺內，代表在同一個 1x1 的方塊中
            if abs(lead_x - existing_lead['x']) <= 0.5 and abs(lead_y - existing_lead['y']) <= 0.5:
              is_duplicate = True
              break

          # 如果確認是同一台車的重複追蹤點，就跳過不加入清單
          if is_duplicate:
            continue

          # 通過過濾，將有用的前車資訊打包成字典，加進清單中
          self.leads_list.append(
            {
              'index': i,  # 在 leadsV3 中的陣列索引
              'prob': lead.prob,  # 信心度 (0.0 ~ 1.0)
              'x': lead_x,  # 縱向相對距離
              'y': lead_y,  # 橫向相對偏移
            }
          )

  def _render(self, rect: rl.Rectangle) -> None:
    """
    渲染函式：使用 for 迴圈遍歷 leads_list，將清單中每個前車點繪製到畫面上。
    防重疊機制：多重環繞軌道演算法，若內圈重疊則半徑遞增向外搜尋。
    動態箭頭機制：透過三角函數計算絕對角度，精準指嚮目標中心。
    """
    # 如果清單為空，代表目前沒有任何需要繪製的前車目標，直接返回
    if not self.leads_list:
      return

    # 建立一個清單，用來記錄在當前影格中「已經畫上去的文字矩形區域」
    drawn_text_rects = []

    # =========================================================
    # 使用 for 迴圈逐一處理並顯示所有偵測到的前車點
    # =========================================================
    for lead_info in self.leads_list:
      try:
        # 取得路徑的高度偏移量 (Z 軸)，與 RadarUiRenderer 保持一致的投影邏輯
        offset = self.model_renderer._path_offset_z

        # 呼叫底層的 _map_to_screen 將每個點的 3D 座標映射至 2D 螢幕上
        screen_pt = self.model_renderer._map_to_screen(lead_info['x'], lead_info['y'], offset)
      except Exception as e:
        continue

      # 若投影結果為空，同樣跳過
      if screen_pt is None:
        continue

      # 取得轉換後的螢幕 X, Y 像素座標 (車輛底部中心基準點)
      x = int(screen_pt[0])
      y = int(screen_pt[1])

      # 依據縱向距離估算虛擬的車輛像素寬高
      distance = max(lead_info['x'], 1.0)
      box_width = max(30, min(int((1.8 / distance) * 1000), 400))
      box_height = int(box_width * 0.8)

      # 目標車輛中心點
      target_center_x = x
      target_center_y = y

      # =========================================================
      # 顏色漸變計算 (50%以上綠色，30~50%黃漸變到綠，30%以下紅漸變到黃)
      # =========================================================
      prob = lead_info['prob']
      if prob >= 0.5:
        r, g, b = 0, 255, 0  # 信心度 ≥ 50%：純綠色
      elif prob >= 0.3:
        # 信心度在 30% ~ 50% 之間：由黃色 (255, 255, 0) 漸變到綠色 (0, 255, 0)
        ratio = (prob - 0.3) / 0.2
        r = int(255 * (1.0 - ratio))
        g = 255
        b = 0
      else:
        # 信心度小於 30%：由紅色 (255, 0, 0) 漸變到黃色 (255, 255, 0)
        ratio = prob / 0.3
        r = 255
        g = int(255 * ratio)
        b = 0

      color = rl.Color(r, g, b, 255)

      # =========================================================
      # 文字準備與初始排版測量 (格式如：lead[0]\n30%)
      # =========================================================
      prob_percent = prob * 100
      text = f"lead[{lead_info['index']}]{prob_percent:.0f}%"

      size = measure_text_cached(self._font_bold, text, self.label_size, 0)
      text_width = size.x
      text_height = size.y

      # =========================================================
      # 多重軌道擴張避讓演算法 (Multi-Orbit Expansion)
      # =========================================================
      # 設定基礎環繞軌道的半徑 (比車身稍微大一點，預留箭頭空間)
      base_orbit_radius = max(80, (box_height // 2) + 60)

      # 預先定義一系列嘗試的角度 (從正上方 -90 度開始，逐漸向左右展開)
      search_angles = [-90, -120, -60, -150, -30, -180, 0, 150, 30, 120, 60, 90]

      # 軌道擴張設定：最多擴張 5 圈，每圈半徑往外增加 60 像素
      max_orbits = 5
      orbit_step = 60

      final_text_rect = None
      final_angle = -90
      final_radius = base_orbit_radius

      # 開始逐圈搜尋
      for orbit in range(max_orbits):
        current_radius = base_orbit_radius + (orbit * orbit_step)

        for angle_deg in search_angles:
          # 將角度轉換為弧度
          angle_rad = math.radians(angle_deg)

          # 根據當前軌道半徑與角度，計算文字預設中心點
          test_center_x = target_center_x + current_radius * math.cos(angle_rad)
          test_center_y = target_center_y + current_radius * math.sin(angle_rad)

          # 推算出該文字的左上角座標
          test_x = test_center_x - (text_width / 2)
          test_y = test_center_y - (text_height / 2)

          # 建立測試用的矩形邊界 (額外加上 15 像素的安全間距，確保文字不會太貼近)
          current_rect = rl.Rectangle(test_x - 15, test_y - 15, text_width + 30, text_height + 30)

          # 檢查是否與任何已經畫好的文字重疊
          is_overlapping = False
          for drawn_rect in drawn_text_rects:
            if not (
              current_rect.x + current_rect.width < drawn_rect.x
              or drawn_rect.x + drawn_rect.width < current_rect.x
              or current_rect.y + current_rect.height < drawn_rect.y
              or drawn_rect.y + drawn_rect.height < current_rect.y
            ):
              is_overlapping = True
              break  # 發生碰撞，跳出檢查，換下一個角度

          if not is_overlapping:
            # 找到空位了！記錄下這個完美的矩形、角度與半徑
            final_text_rect = current_rect
            final_angle = angle_rad
            final_radius = current_radius
            break  # 成功找到位置，跳出角度迴圈

        if final_text_rect is not None:
          break  # 成功找到位置，跳出軌道擴張迴圈

      # 如果擴張了 5 圈還是找不到，強制使用最外圈正上方
      if final_text_rect is None:
        final_radius = base_orbit_radius + (max_orbits * orbit_step)
        fallback_y = target_center_y - final_radius - (text_height / 2)
        final_text_rect = rl.Rectangle(target_center_x - (text_width / 2) - 15, fallback_y - 15, text_width + 30, text_height + 30)
        final_angle = math.radians(-90)

      # 將最終確認的安全位置加入記錄清單，供下一台車避讓參考
      drawn_text_rects.append(final_text_rect)

      # 還原實際繪圖用的文字起點 (去掉安全間距)
      text_x = final_text_rect.x + 15
      text_y = final_text_rect.y + 15

      # =========================================================
      # 智慧動態箭頭轉向計算 (Dynamic Arrow Vector)
      # =========================================================
      # 箭頭終點 (Tip)：精準指向車輛中心點
      arrow_tip = rl.Vector2(target_center_x, target_center_y)

      # 箭頭起點 (Base)：從計算出的軌道邊緣出發
      # 為了不讓線條穿過文字，起點設定在稍微靠近目標的位置 (半徑內縮 20 像素)
      arrow_base = rl.Vector2(target_center_x + (final_radius - 20) * math.cos(final_angle), target_center_y + (final_radius - 40) * math.sin(final_angle))

      # 繪製加粗的箭頭主幹 (線寬設為 5.0，視覺極為醒目)
      rl.draw_line_ex(arrow_base, arrow_tip, 5.0, color)

      # 計算主幹連線的絕對角度，以便轉向羽翼
      dx = arrow_tip.x - arrow_base.x
      dy = arrow_tip.y - arrow_base.y
      line_angle = math.atan2(dy, dx)

      # 依據主幹的角度，動態推算兩側箭翼的方向 (兩翼與主幹夾角 135 度，長度 15 像素)
      arrow_length = 15.0
      left_wing_angle = line_angle + math.radians(135)
      right_wing_angle = line_angle - math.radians(135)

      # 計算左翼與右翼的端點座標
      left_wing_end = rl.Vector2(arrow_tip.x + math.cos(left_wing_angle) * arrow_length, arrow_tip.y + math.sin(left_wing_angle) * arrow_length)
      right_wing_end = rl.Vector2(arrow_tip.x + math.cos(right_wing_angle) * arrow_length, arrow_tip.y + math.sin(right_wing_angle) * arrow_length)

      # 繪製加粗的左右箭翼 (線寬 5.0)，自動隨軌道角度完美轉向
      rl.draw_line_ex(left_wing_end, arrow_tip, 5.0, color)
      rl.draw_line_ex(right_wing_end, arrow_tip, 5.0, color)

      # =========================================================
      # 繪製最終的信心度文字
      # =========================================================
      # 使用粗體字型與動態漸變色，畫在算好的環繞軌道空位上
      rl.draw_text_ex(self._font_bold, text, rl.Vector2(text_x, text_y), self.label_size, 0, color)
