"""
AEM (Automatic Experimental Mode) - Anti-Ghost Braking Version
功能：
1. 紅燈減速、綠燈快速恢復 ACC
2. 針對無號誌斑馬線的抗干擾機制 (Debounce)
3. 平滑化急迫度數值，配合 High Slew Rate Planner
4. [新增] 高速抑制遲滯邏輯 (70開啟，低於65才關閉)
5. [新增] 穩定前車檢測邏輯
"""

import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

# ==============================================================================
#                               CONFIG (參數設定區)
# ==============================================================================
class Config:
    # --- 速度定義 ---
    # [修改] 加入遲滯區間：超過 70km/h 開啟高速抑制，降至 65km/h 以下才解除
    HIGHWAY_SPEED_ON  = 70.0
    HIGHWAY_SPEED_OFF = 65.0

    # --- 靈敏度曲線 (KPH) ---
    SENSITIVITY_BP   = [0.,  50., 80., 110.]
    SENSITIVITY_VALS = [1.0, 1.0, 0.85, 0.4]

    # --- 減速模型 (M/S 對應 距離) ---
    SLOW_DOWN_BP   = [0.,  5.,   10.,  15.,  20.,  25.,   30.]
    SLOW_DOWN_DIST = [5.,  25.,  50.,  75.,  100., 130.,  160.]

    # --- 模式定義 ---
    MODE_ACC = 'acc'
    MODE_BLENDED = 'blended'
    
    # --- 新增：穩定前車檢測參數 ---
    LEAD_STABLE_TIME_SEC = 1.0        # 前車需穩定存在1秒
    LEAD_STABLE_FRAMES = 20           # 對應20幀（20Hz）
    LEAD_STABLE_DIST_VAR = 3.0        # 前車距離變化允許的最大波動 (米/幀)
    LEAD_MIN_DIST = 2.0               # 最小前車距離 (太近視為不穩定)
    
    # --- 實驗模式退出機制 ---
    EXIT_TRIGGER_THRESHOLD = 0.45     # 急迫度低於此值開始退出
    EXIT_DEBOUNCE_FRAMES = 5          # 需要連續低於閾值5幀才完全退出

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
    self.low_urgency_counter = 0  # 用於退出的計數器

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

  def update(self, urgency_val):
    """更新模式管理器狀態，加入退出邏輯"""
    # 檢查是否需要退出Blended模式
    if self.current_mode == Config.MODE_BLENDED:
        if urgency_val < Config.EXIT_TRIGGER_THRESHOLD:
            self.low_urgency_counter += 1
        else:
            self.low_urgency_counter = 0
        
        # 如果急迫度連續過低，快速退出Blended
        if self.low_urgency_counter >= Config.EXIT_DEBOUNCE_FRAMES:
            self.mode_confidence[Config.MODE_BLENDED] *= 0.7
    else:
        self.low_urgency_counter = 0
    
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
        
        # 高急迫度計數器 (Debounce Counter)
        self._high_urgency_counter = 0
        
        # [新增] 紀錄目前是否處於高速抑制狀態
        self._highway_suppression_active = False
        
        # ============== 新增：穩定前車檢測 ==============
        self._prevent_experiment_mode = False
        self._experiment_blocked_reason = ''
        
        # 前車追蹤變數
        self._lead_stable_counter = 0
        self._lead_unstable_counter = 0
        self._lead_confidence = 0.0
        self._last_lead_dRel = float('inf')
        self._last_lead_vRel = 0.0
        self._current_lead_id = None
        self._lead_same_target_frames = 0
        self._lead_detected_frames = 0
        
        # 實驗模式狀態追蹤
        self._experiment_mode_active = False
        self._experiment_enter_counter = 0
        self._last_lead_dist_when_entered = float('inf')
        
        # 歷史數據追蹤
        self._lead_distance_history = []
        self._max_history_frames = 30  # 保存1.5秒歷史

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

        # ============== 1. 穩定前車檢測 ==============
        self._update_lead_stability(radar_msg, v_ego)
        
        # ============== 2. 檢查是否可以進入實驗模式 ==============
        can_enter_experiment = self._can_enter_experiment_mode(model_msg, radar_msg, v_ego, v_kph)
        
        # 如果無法進入實驗模式，直接返回ACC
        if not can_enter_experiment:
            # 重置進入實驗模式的計數器
            self._high_urgency_counter = 0
            self._urgency = 0.0
            self._slow_down_filter.x = 0.0
            
            # 強制回到ACC模式
            self._mode_manager.request_mode(Config.MODE_ACC, confidence=1.0)
            self._mode_manager.update(self._urgency)
            
            # 更新實驗模式狀態
            if self._experiment_mode_active:
                self._experiment_mode_active = False
                self._experiment_enter_counter = 0
                self._last_lead_dist_when_entered = float('inf')
            
            return

        # ============== 3. 計算紅綠燈減速邏輯 ==============
        self._calculate_slow_down(model_end_dist, v_ego, v_kph)

        # ============== 4. 決策與模式切換 ==============
        TRIGGER_THRESHOLD = 0.45
        
        # 設定連續確認幀數 (5幀約等於 0.25秒)
        CONFIRMATION_FRAMES = 5 

        if self._urgency > TRIGGER_THRESHOLD:
            self._high_urgency_counter += 1
        else:
            self._high_urgency_counter = 0

        # 只有當連續計數超過設定值，才真的切換模式
        if self._high_urgency_counter >= CONFIRMATION_FRAMES:
            # 確認是穩定的紅燈/路口訊號 -> 切換 Blended
            self._mode_manager.request_mode(Config.MODE_BLENDED, confidence=min(1.0, self._urgency))
            
            # 記錄進入實驗模式
            if not self._experiment_mode_active:
                self._experiment_mode_active = True
                self._experiment_enter_counter += 1
                
                # 記錄進入時的車況
                if radar_msg and radar_msg.leadOne and radar_msg.leadOne.status:
                    self._last_lead_dist_when_entered = radar_msg.leadOne.dRel
        else:
            # 訊號不穩定 或 綠燈 -> 保持 ACC
            self._mode_manager.request_mode(Config.MODE_ACC, confidence=0.9)
            
            # 更新實驗模式狀態
            if self._experiment_mode_active:
                self._experiment_mode_active = False
                self._experiment_enter_counter = 0
                self._last_lead_dist_when_entered = float('inf')

        # ============== 5. 更新管理器狀態 ==============
        self._mode_manager.update(self._urgency)

    def _update_lead_stability(self, radar_msg, v_ego):
        """
        更新前車穩定性檢測
        
        檢測前車是否為同一目標且穩定存在超過1秒
        """
        # 檢查是否有前車
        has_lead = False
        current_lead_dRel = float('inf')
        current_lead_vRel = 0.0
        
        if radar_msg and radar_msg.leadOne and radar_msg.leadOne.status:
            has_lead = True
            current_lead_dRel = radar_msg.leadOne.dRel
            current_lead_vRel = radar_msg.leadOne.vRel
            
            # 檢查距離是否合理
            if current_lead_dRel < Config.LEAD_MIN_DIST:
                # 距離太近，視為不穩定
                has_lead = False
        
        if has_lead:
            self._lead_detected_frames += 1
            
            # 檢查是否為同一目標
            distance_variance = abs(current_lead_dRel - self._last_lead_dRel)
            speed_variance = abs(current_lead_vRel - self._last_lead_vRel)
            
            # 判斷是否為同一目標
            is_same_target = (distance_variance < Config.LEAD_STABLE_DIST_VAR and 
                             speed_variance < 2.0)  # 速度變化小於2m/s
            
            if is_same_target:
                self._lead_same_target_frames += 1
                self._lead_stable_counter = min(
                    Config.LEAD_STABLE_FRAMES, 
                    self._lead_stable_counter + 1
                )
                self._lead_unstable_counter = max(0, self._lead_unstable_counter - 1)
            else:
                # 目標變換，重置計數
                self._lead_same_target_frames = 1
                self._lead_stable_counter = 0
                self._lead_unstable_counter += 1
            
            # 更新穩定信心度
            if self._lead_same_target_frames >= Config.LEAD_STABLE_FRAMES:
                self._lead_confidence = min(1.0, self._lead_stable_counter / Config.LEAD_STABLE_FRAMES)
            else:
                self._lead_confidence = 0.0
            
            # 更新歷史數據
            self._lead_distance_history.append(current_lead_dRel)
            if len(self._lead_distance_history) > Config.LEAD_STABLE_FRAMES:
                self._lead_distance_history.pop(0)
            
            # 保存當前數據
            self._last_lead_dRel = current_lead_dRel
            self._last_lead_vRel = current_lead_vRel
            
        else:
            # 無前車
            self._lead_detected_frames = 0
            self._lead_same_target_frames = 0
            self._lead_stable_counter = max(0, self._lead_stable_counter - 1)
            self._lead_unstable_counter += 1
            self._lead_confidence = 0.0
            self._last_lead_dRel = float('inf')
            self._last_lead_vRel = 0.0
            self._lead_distance_history = []

    def _can_enter_experiment_mode(self, model_msg, radar_msg, v_ego, v_kph):
        """
        檢查是否可以進入實驗模式
        
        返回True的條件：
        1. 無前車
        2. 有前車但距離很遠（不影響AEM）
        3. 有前車但不穩定（可能是側方車輛，即將消失）
        
        返回False的條件：
        1. 有穩定前車（穩定存在>1秒）且距離 < AEM停車距離
        """
        # 檢查是否有前車
        has_lead = False
        lead_dRel = float('inf')
        
        if radar_msg and radar_msg.leadOne and radar_msg.leadOne.status:
            has_lead = True
            lead_dRel = radar_msg.leadOne.dRel
            
            # 檢查距離是否合理
            if lead_dRel < Config.LEAD_MIN_DIST:
                has_lead = False
        
        # 情況1：無前車 → 允許實驗模式
        if not has_lead:
            return True
        
        # 計算實驗模型停車距離
        experiment_stop_dist = self._calculate_experiment_stop_distance(model_msg, v_ego, v_kph)
        
        # 情況2：有前車但不穩定（可能是側方車輛）→ 允許實驗模式
        if self._lead_confidence < 0.8:  # 穩定信心度低於80%
            return True
        
        # 情況3：穩定前車，但距離很遠 → 允許實驗模式
        if lead_dRel > experiment_stop_dist * 1.5:  # 前車距離比AEM停車距離遠50%以上
            return True
        
        # 情況4：穩定前車，且距離很近 → 檢查是否應該阻止
        if lead_dRel < experiment_stop_dist:
            # 檢查前車是否穩定存在超過1秒
            if (self._lead_same_target_frames >= Config.LEAD_STABLE_FRAMES and 
                self._lead_confidence >= 0.8):
                # 穩定前車且距離太近，阻止實驗模式
                self._experiment_blocked_reason = (
                    f'穩定前車({lead_dRel:.1f}m) < AEM停車距離({experiment_stop_dist:.1f}m), '
                    f'穩定幀:{self._lead_same_target_frames}, '
                    f'信心度:{self._lead_confidence:.2f}'
                )
                return False
        
        # 其他情況都允許
        return True

    def _calculate_experiment_stop_distance(self, model_msg, v_ego, v_kph):
        """
        計算實驗模型（AEM）得出的目標停車位置
        """
        # 使用原有的基礎預期距離計算
        base_expected = np.interp(v_ego, Config.SLOW_DOWN_BP, Config.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, Config.SENSITIVITY_BP, Config.SENSITIVITY_VALS)
        
        # 實驗模型的目標停車距離
        experiment_stop_dist = base_expected * sensitivity * 1.1
        
        return experiment_stop_dist

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
            self._high_urgency_counter = 0 # 同步歸零計數器
            return

        # --- 紅燈/減速計算 ---
        # 計算距離缺口比例
        shortage_ratio = (expected_distance - model_end_dist) / max(1.0, expected_distance)
        
        # 指數曲線放大急迫度
        raw_urgency = np.clip((shortage_ratio ** 1.5) * 2.5, 0.0, 1.2)

        # --- [修改] 高速抑制 (70開 65關 遲滯邏輯) ---
        if v_kph > Config.HIGHWAY_SPEED_ON:
            self._highway_suppression_active = True
        elif v_kph < Config.HIGHWAY_SPEED_OFF:
            self._highway_suppression_active = False

        # 根據狀態決定是否壓抑 urgency
        if self._highway_suppression_active:
             raw_urgency = min(raw_urgency, 0.4)

        # 濾波器更新
        self._slow_down_filter.add_data(raw_urgency)
        self._urgency = self._slow_down_filter.get_value()
