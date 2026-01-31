import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

class AEM:
    def __init__(self):
        # --- Config (參數設定) ---
        self.HIGHWAY_SPEED = 70.0  # 超過此速度(kph)忽略紅燈
        
        # 靈敏度與距離定義 (使用完美復刻原版曲線)
        self.SENSITIVITY_BP = [0., 50., 80., 110.]
        self.SENSITIVITY_VALS = [1.0, 1.0, 0.85, 0.4]

        self.SLOW_DOWN_BP = [0., 5., 20., 25., 30.]
        self.SLOW_DOWN_DIST = [5., 25., 100., 130., 160.]

        # --- State (狀態) ---
        self.mode = 'acc'
        self.blended_conf = 0.0
        self.urgency_val = 0.0
        self.high_urgency_counter = 0

    # [FIX] 修正：增加 ignored_arg 參數，解決 TypeError 報錯
    # 這樣就算外部傳入參數，這裡也會接收並忽略，不會導致崩潰
    def get_mode(self, ignored_arg=None):
        return self.mode

    def update_states(self, model_msg, radar_msg, v_ego):
        # 1. 資料檢核
        if len(model_msg.position.z) != ModelConstants.IDX_N:
            return

        # 2. 基礎運算
        v_kph = v_ego * 3.6
        model_end_dist = model_msg.position.z[-1]

        # 3. 計算預期煞停距離
        base_expected = np.interp(v_ego, self.SLOW_DOWN_BP, self.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, self.SENSITIVITY_BP, self.SENSITIVITY_VALS)
        expected_dist = base_expected * sensitivity * 1.1

        # 4. 計算急迫度
        raw_urgency = 0.0
        if model_end_dist < expected_dist:
            shortage_ratio = (expected_dist - model_end_dist) / max(1.0, expected_dist)
            raw_urgency = np.clip((shortage_ratio ** 1.5) * 2.5, 0.0, 1.2)
            
            if v_kph > self.HIGHWAY_SPEED:
                raw_urgency = min(raw_urgency, 0.4)

        # 5. 濾波器 (綠燈加速歸零)
        alpha = 0.5 if raw_urgency == 0.0 else 0.15
        self.urgency_val = (self.urgency_val * (1 - alpha)) + (raw_urgency * alpha)

        # 6. 決策邏輯 (Debounce & Mode Switch)
        TRIGGER_THRESHOLD = 0.45
        CONFIRMATION_FRAMES = 5

        if self.urgency_val > TRIGGER_THRESHOLD:
            self.high_urgency_counter += 1
        else:
            self.high_urgency_counter = 0

        if self.high_urgency_counter >= CONFIRMATION_FRAMES:
            self.blended_conf = min(1.0, self.blended_conf + 0.1)
        else:
            self.blended_conf = max(0.0, self.blended_conf - 0.2)

        threshold = 0.4 if self.mode == 'blended' else 0.75
        self.mode = 'blended' if self.blended_conf > threshold else 'acc'
