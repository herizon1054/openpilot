import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

class AEM:
    def __init__(self):
        # --- Config (參數區) ---
        self.HIGHWAY_SPEED = 70.0  # 超過此速度(kph)忽略紅燈，防高速誤判
        
        # 速度(kph) 對應 靈敏度係數
        self.SENSITIVITY_BP = [0., 50., 80., 110.]
        self.SENSITIVITY_VALS = [1.0, 1.0, 0.85, 0.4]

        # 速度(m/s) 對應 預期煞停距離(m)
        self.SLOW_DOWN_BP = [0., 5., 25., 30.]
        self.SLOW_DOWN_DIST = [5., 25., 130., 160.] # 簡化了中間的線性插值點

        # --- State (狀態區) ---
        self.mode = 'acc'
        self.blended_conf = 0.0  # Blended 模式的信心值 (0.0~1.0)
        self.urgency_val = 0.0   # 平滑後的急迫度
        self.high_urgency_counter = 0 # 抗干擾計數器

    def get_mode(self):
        return self.mode

    def update_states(self, model_msg, radar_msg, v_ego):
        """
        主邏輯：輸入模型數據，輸出模式決策
        """
        # 1. 資料檢核
        if len(model_msg.position.z) != ModelConstants.IDX_N:
            return

        # 2. 基礎數據準備
        v_kph = v_ego * 3.6
        model_end_dist = model_msg.position.z[-1] # 直接取最後一點 (IDX_N - 1)

        # 3. 計算急迫度 (Urgency)
        # 取得預期煞停距離 (加入 1.1 倍緩衝)
        base_expected = np.interp(v_ego, self.SLOW_DOWN_BP, self.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, self.SENSITIVITY_BP, self.SENSITIVITY_VALS)
        expected_dist = base_expected * sensitivity * 1.1

        raw_urgency = 0.0
        
        # 如果模型預測距離 < 預期煞停距離，代表需要減速 (紅燈/路口/障礙)
        if model_end_dist < expected_dist:
            shortage_ratio = (expected_dist - model_end_dist) / max(1.0, expected_dist)
            # 指數放大急迫度，並限制最大值
            raw_urgency = np.clip((shortage_ratio ** 1.5) * 2.5, 0.0, 1.2)
            
            # 高速抑制 (防止路牌誤判)
            if v_kph > self.HIGHWAY_SPEED:
                raw_urgency = min(raw_urgency, 0.4)

        # 4. 濾波器 (取代原本複雜的 Kalman Filter)
        # 使用指數移動平均 (EMA)，alpha=0.15 相當於原本的平滑效果
        # 若 raw_urgency 為 0 (綠燈瞬間)，給予較大的 alpha (0.5) 以便快速歸零
        alpha = 0.5 if raw_urgency == 0.0 else 0.15
        self.urgency_val = (self.urgency_val * (1 - alpha)) + (raw_urgency * alpha)

        # 5. 決策邏輯 (Debounce & Mode Switch)
        TRIGGER_THRESHOLD = 0.45
        CONFIRMATION_FRAMES = 5  # 連續 5 幀才觸發 (約 0.25秒)

        # 計數器邏輯
        if self.urgency_val > TRIGGER_THRESHOLD:
            self.high_urgency_counter += 1
        else:
            self.high_urgency_counter = 0

        # 模式切換邏輯
        target_mode = 'acc'
        
        # 只有當訊號「強」且「穩定(計數達標)」才切換至 Blended
        if self.high_urgency_counter >= CONFIRMATION_FRAMES:
            # 增加 Blended 信心
            self.blended_conf = min(1.0, self.blended_conf + 0.1)
        else:
            # 綠燈或訊號不穩，快速衰退 (綠燈起步關鍵)
            # 這裡用 0.2 的衰退率，讓起步反應比原本更快
            self.blended_conf = max(0.0, self.blended_conf - 0.2)

        # 遲滯門檻 (Hysteresis) 避免在邊界跳動
        # 如果當前是 blended，降到 0.4 以下才切回 acc
        # 如果當前是 acc，升到 0.75 以上才切入 blended
        threshold = 0.4 if self.mode == 'blended' else 0.75
        
        if self.blended_conf > threshold:
            self.mode = 'blended'
        else:
            self.mode = 'acc'
