import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached


class SystemMonitorRenderer(Widget):
  def __init__(self):
    super().__init__()
    # 字體設定
    self.label_size = 26
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)


  def _update_state(self) -> None:
    """
    從 ui_state 更新硬體數據
    """
    sm = ui_state.sm
    if sm.updated['deviceState']:
      device_state = sm['deviceState']

      # CPU 平均使用率
      cpu_usages = device_state.cpuUsagePercent
      cpu_usage = sum(cpu_usages) / len(cpu_usages) if len(cpu_usages) > 0 else 0.0

      # GPU 與 RAM 使用率
      gpu_usage = device_state.gpuUsagePercent
      mem_usage = device_state.memoryUsagePercent

      # 最高溫
      max_temp = device_state.maxTempC

      # 風扇轉速百分比
      fan_speed_pct = int(device_state.fanSpeedPercentDesired)

      # 更新左側要顯示的硬體文字
      self.hw_text = f"CPU: {cpu_usage:.0f}%  |  GPU: {gpu_usage}%  |  RAM: {mem_usage}%  |  Max Temp: {max_temp:.1f}°C  |  Fan: {fan_speed_pct}%"

  def _render(self, rect: rl.Rectangle) -> None:
    """
    執行畫面渲染 (直接由 Openpilot UI 引擎呼叫並傳入螢幕範圍 rect)
    """
    # 如果還沒抓到資料，可以選擇不畫，或畫預設文字
    if not self.hw_text:
      return

    # --- 設定顏色 ---
    # 這裡使用白色，如果您需要半透明，可以使用 rl.Color(255, 255, 255, 200)
    text_color = rl.WHITE

    # --- 座標計算 ---
    text_size = measure_text_cached(self._font_bold, self.hw_text, self.label_size, 0)
    bottom_y = rect.height - text_size.y

    # 左側硬體資訊 X 軸：固定靠左 (例如距離左邊緣 40 像素)
    hw_x = 80

    # --- 繪製文字 ---
    rl.draw_text_ex(self._font_bold, self.hw_text, rl.Vector2(hw_x, bottom_y), self.label_size, 0, text_color)
