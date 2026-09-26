"""
Copyright (c) 2026, Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, and/or sublicense,
for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
- Commercial use (e.g. use in a product, service, or activity intended to
  generate revenue) is prohibited without explicit written permission from
  the copyright holder.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

# 速度門檻常數 (km/h 轉換為 m/s)
APM_DEPARTURE_SPEED = 20 * 1000 / 3600   # 5 km/h：起步激烈模式上限

# 完全靜止門檻
V_EGO_STANDSTILL = 0.01                  # 低於 0.01 m/s 視為完全靜止，才準備起步


class APM:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    
    # 用來記憶車輛狀態 (僅保留起步狀態)
    self.is_departing = False       

    # 初始化開關狀態
    self._enabled = self.params.get_bool("dp_lon_apm")

  def update(self, sm=None):
    """依照系統物理週期更新 Params，參照 accel_controller"""
    self.frame += 1
    # 利用 DT_MDL 換算真實物理時間，精準每 10.0 秒讀取一次 Params
    if self.frame % int(10.0 / DT_MDL) == 0:
      self._enabled = self.params.get_bool("dp_lon_apm")

  def is_enabled(self) -> bool:
    return self._enabled

  def get_personality(self, v_ego, has_lead, v_lead, a_lead, d_lead, personality, t_follow_relaxed=1.75):
    # 如果模組未啟用，直接攔截並回傳原廠設定風格，不消耗任何計算資源
    if not self._enabled:
      return personality
      
    # --- 1. 起步狀態更新 (唯一保留的場景) ---
    # 利用狀態機的遲滯特性。減速過程中 v_ego 在 0.01 ~ 1.38 m/s 之間時，
    # 既不會觸發 is_departing = True，也不會強制設為 False，而是保留原本的狀態。
    if v_ego < V_EGO_STANDSTILL:
      self.is_departing = True
    elif v_ego >= APM_DEPARTURE_SPEED:
      self.is_departing = False

    # --- 2. 決定最終輸出的模式 ---
    # 當處於起步狀態且車速低於 5 km/h 時，強制輸出積極模式，確保輕快起步
    if self.is_departing and v_ego < APM_DEPARTURE_SPEED:
      return log.LongitudinalPersonality.aggressive

    # 只要不是在起步階段，一律原封不動回傳系統設定的性格 (通常是 Standard)，
    # 將行進間與煞停的減速邏輯完全交給 accel_controller 處理。
    return personality

