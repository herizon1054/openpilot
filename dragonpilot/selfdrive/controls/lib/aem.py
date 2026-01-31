"""
AEM (Automatic Experimental Mode) - Anti-Ghost Braking Version
功能：
1. 紅燈減速、綠燈快速恢復 ACC
2. [新增] 針對無號誌斑馬線的抗干擾機制 (Debounce)
3. [新增] 平滑化急迫度數值，配合 High Slew Rate Planner
"""

import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

# ==============================================================================
#                               CONFIG (參數設定區)
# ==============================================================================
class Config:
    # --- 速度定義 ---
    # 超過此速度 (70km/h) 忽略紅綠燈，避免高速公路誤判
    HIGHWAY_SPEED  = 70.0

    # --- 靈敏度曲線 (KPH) ---
    SENSITIVITY_BP   = [0.,  50., 80., 110.]
    SENSITIVITY_VALS = [1.0, 1.0, 0.85, 0.4]

    # --- 減速模型 (M/S 對應 距離) ---
    SLOW_DOWN_BP   = [0.,  5.,   10.,  15.,  20.,  25.,   30.]
    SLOW_DOWN_DIST = [5.,  25.,  50.,  75.,  100., 130.,  160.]

    # --- 模式定義 ---
    MODE_ACC = 'acc'
    MODE_BLENDED = 'blended'

# ==============================================================================
#                         UTILITY CLASSES (工具類別)
# ==============================================================================

class SmoothKalmanFilter:
  """簡化的濾波器，僅保留核心平滑運算"""
  def __init__(self, initial_value=0.0):
    self.x = initial_value
    self.P = 1.0
    self.R = 0.2
    self.Q = 0.01
    self.initialized = False

  def add_data(self, measurement):
    if not self.initialized:
      self.x = measurement
      self.initialized = True
      return

    # 標準卡爾曼更新
    self.P = self.P + self.Q
    K = self.P / (self.P + self.R)
    
    # 混合平滑因子 (固定為優化後的 0.85 效果)
    smoothing_factor = 0.85
    effective_K = K * (1.0 - smoothing_factor) + smoothing_factor * 0.1
    
    self.x = self.x + effective_K * (measurement - self.x)
    self.P = (1 - effective_K) * self.P

  def get_value(self):
    return self.x if self.initialized else 0.0

class ModeTransitionManager:
  """模式切換管理器"""
  def __init__(self):
    self.current_mode = Config.MODE_ACC
    self.mode_confidence = {Config.MODE_ACC: 1.0, Config.MODE_BLENDED: 0.0}

  def request_mode(self, mode, confidence=1.0):
    # [綠燈快速恢復邏輯]
    # 如果請求 ACC 且信心很高 (代表綠燈)，加速信心回升
    step = 0.2 if (mode == Config.MODE_ACC and confidence >= 0.9) else 0.05

    # 平滑增加目標模式的信心值
    target_conf = min(1.0, self.mode_confidence[mode] + step * confidence)
    self.mode_confidence[mode] = target_conf

    # 降低其他模式的信心值
    for m in self.mode_confidence:
      if m != mode:
        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - step)

    # 門檻判斷 (遲滯比較)
    threshold = 0.75 if mode != self.current_mode else 0.4
    if self.mode_confidence[mode] > threshold:
        self.current_mode = mode

  def update(self):
    # 自然衰減 Blended 模式信心
    self.mode_confidence[Config.MODE_BLENDED] *= 0.95
    self.mode_confidence[Config.MODE_ACC] = 1.0 - self.mode_confidence[Config.MODE_BLENDED]

  def get_mode(self):
    return self.current_mode

# ==============================================================================
#                               CORE LOGIC (核心邏輯)
# ==============================================================================

class AEM:
    def __init__(self):
        self._mode_manager = ModeTransitionManager()
        self._slow_down_filter = SmoothKalmanFilter()
        self._urgency = 0.0
        
        # [MOD] 新增：高急迫度計數器 (Debounce Counter)
        # 用來確認訊號是否穩定，避免斑馬線一瞬間的誤判
        self._high_urgency_counter = 0

    def get_mode(self, current_mode_str):
        return self._mode_manager.get_mode()

    def update_states(self, model_msg, radar_msg, v_ego):
        """主邏輯更新"""
        # 資料完整性檢查
        if len(model_msg.position.z) != ModelConstants.IDX_N:
            return

        v_kph = v_ego * 3.6
        # 直接取最後一點的距離
        model_end_dist = model_msg.position.z[ModelConstants.IDX_N - 1]

        # 1. 計算紅綠燈減速邏輯
        self._calculate_slow_down(model_end_dist, v_ego, v_kph)

        # 2. 決策與模式切換 [重點修改區域]
        TRIGGER_THRESHOLD = 0.45
        
        # 設定連續確認幀數 (5幀約等於 0.25秒)
        # 數值越小：反應越快，但容易被斑馬線誤導
        # 數值越大：越抗干擾，但紅燈反應變慢
        CONFIRMATION_FRAMES = 5 

        if self._urgency > TRIGGER_THRESHOLD:
            self._high_urgency_counter += 1
        else:
            self._high_urgency_counter = 0

        # [MOD] 邏輯變更：只有當連續計數超過設定值，才真的切換模式
        if self._high_urgency_counter >= CONFIRMATION_FRAMES:
            # 確認是穩定的紅燈/路口訊號 -> 切換 Blended
            self._mode_manager.request_mode(Config.MODE_BLENDED, confidence=min(1.0, self._urgency))
        else:
            # 訊號不穩定(可能是斑馬線誤判) 或 綠燈 -> 保持 ACC
            # confidence=0.9 確保在斑馬線誤判消失後能瞬間恢復
            self._mode_manager.request_mode(Config.MODE_ACC, confidence=0.9)

        # 3. 更新管理器狀態
        self._mode_manager.update()

    def _calculate_slow_down(self, model_end_dist, v_ego, v_kph):
        """
        核心功能：計算急迫度 (Urgency)
        """
        # 取得預期煞停距離 (加入 1.1 倍緩衝)
        base_expected = np.interp(v_ego, Config.SLOW_DOWN_BP, Config.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, Config.SENSITIVITY_BP, Config.SENSITIVITY_VALS)
        expected_distance = base_expected * sensitivity * 1.1

        # --- 綠燈/路徑通暢檢測 ---
        # 如果模型看的距離比預期煞停距離還遠，代表是綠燈或無障礙
        if model_end_dist > expected_distance:
            self._slow_down_filter.x = 0.0 # 強制歸零 (秒起步關鍵)
            self._urgency = 0.0
            self._high_urgency_counter = 0 # [MOD] 同步歸零計數器
            return

        # --- 紅燈/減速計算 ---
        # 計算距離缺口比例
        shortage_ratio = (expected_distance - model_end_dist) / max(1.0, expected_distance)
        
        # 指數曲線放大急迫度
        raw_urgency = np.clip((shortage_ratio ** 1.5) * 2.5, 0.0, 1.2)

        # 高速抑制 (防止路牌誤判)
        if v_kph > Config.HIGHWAY_SPEED:
             raw_urgency = min(raw_urgency, 0.4)

        # 濾波器更新
        # [MOD] 移除原本的 "if raw_urgency > current + 0.25" 跳躍邏輯
        # 強制所有數據經過平滑濾波，消除瞬間雜訊
        # 因為您在 Planner 的 Slew Rate 已經很高(0.15)，這裡不需要跳過濾波也能煞得住
        self._slow_down_filter.add_data(raw_urgency)

        self._urgency = self._slow_down_filter.get_value()
