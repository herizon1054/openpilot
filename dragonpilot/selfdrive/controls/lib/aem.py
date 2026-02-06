import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

class AEM:
    def __init__(self):
        # --- Config (參數設定) ---
        self.HIGHWAY_SPEED = 70.0  # 超過此速度(kph)忽略紅燈/彎道減速
        
        # [關鍵設定] 速度區間：35 km/h ~ 1 km/h
        # 目的：中低速塞車時使用實驗模式(靈敏)，快煞停時(<1)切回ACC(柔順)
        self.EXPERIMENTAL_MAX_SPEED = 35.0 
        self.EXPERIMENTAL_MIN_SPEED = 1.0  
        
        self.TTC_THRESHOLD = 1.5           # 前車 TTC 觸發閾值 (秒)
        
        # 靈敏度與距離定義 (視覺模型用)
        self.SENSITIVITY_BP = [0., 50., 80., 110.]
        self.SENSITIVITY_VALS = [1.0, 1.0, 0.85, 0.4]

        self.SLOW_DOWN_BP = [0., 5., 20., 25., 30.]
        self.SLOW_DOWN_DIST = [5., 25., 100., 130., 160.]

        # --- State (狀態) ---
        self.mode = 'acc'
        self.blended_conf = 0.0
        self.urgency_val = 0.0
        self.high_urgency_counter = 0

    # 為了相容性，保留 ignored_arg 參數
    def get_mode(self, ignored_arg=None):
        return self.mode

    def update_states(self, model_msg, radar_msg, v_ego):
        # 1. 資料檢核
        if len(model_msg.position.z) != ModelConstants.IDX_N:
            return

        # 2. 基礎運算
        v_kph = v_ego * 3.6
        model_end_dist = model_msg.position.z[-1]

        # 3. 計算預期煞停距離 (視覺模型部分：紅燈/彎道)
        base_expected = np.interp(v_ego, self.SLOW_DOWN_BP, self.SLOW_DOWN_DIST)
        sensitivity = np.interp(v_kph, self.SENSITIVITY_BP, self.SENSITIVITY_VALS)
        expected_dist = base_expected * sensitivity * 1.1

        # 4. 計算急迫度 (視覺模型)
        raw_urgency = 0.0
        if model_end_dist < expected_dist:
            shortage_ratio = (expected_dist - model_end_dist) / max(1.0, expected_dist)
            raw_urgency = np.clip((shortage_ratio ** 1.5) * 2.5, 0.0, 1.2)
            
            if v_kph > self.HIGHWAY_SPEED:
                raw_urgency = min(raw_urgency, 0.4)

        # ----------------------------------------------------------------
        # 5. 混合式 TTC 介入 (僅在 35 ~ 1 km/h 區間運作)
        # ----------------------------------------------------------------
        if self.EXPERIMENTAL_MIN_SPEED < v_kph < self.EXPERIMENTAL_MAX_SPEED:
            
            lead_d = None
            lead_v = None
            
            # A. 雷達優先 (準確度最高)
            if radar_msg is not None and hasattr(radar_msg, 'leadOne') and radar_msg.leadOne.status:
                lead_d = radar_msg.leadOne.dRel
                lead_v = radar_msg.leadOne.vRel
            
            # B. 視覺備援 (無雷達或雷達丟失時使用)
            elif hasattr(model_msg, 'leadsV3') and len(model_msg.leadsV3) > 0:
                lead_vision = model_msg.leadsV3[0]
                # [抗噪] 視覺信心度 > 0.5 才採信
                if lead_vision.prob > 0.5:
                    lead_d = lead_vision.x[0]
                    lead_v = lead_vision.v[0]

            # C. TTC 計算
            # 確保有前車數據，且正在接近中 (相對速度 < -0.1 m/s)
            if lead_d is not None and lead_v < -0.1:
                ttc = -lead_d / lead_v
                
                # 如果即將碰撞時間小於閾值 (1.5秒)，強制拉高急迫度
                if ttc < self.TTC_THRESHOLD:
                    raw_urgency = max(raw_urgency, 1.2)
        
        # ----------------------------------------------------------------
        # [關鍵保護] 6. 強制 ACC 歸零邏輯
        # 當速度 < 1.0 km/h，無視所有視覺/雷達警報，強制切回 ACC 進行柔順煞停
        # 這是為了避免視覺模型在極低速時因為距離過近而卡在實驗模式
        # ----------------------------------------------------------------
        if v_kph < self.EXPERIMENTAL_MIN_SPEED:
            raw_urgency = 0.0

        # 7. 濾波器 (平滑化數值)
        alpha = 0.5 if raw_urgency == 0.0 else 0.15
        self.urgency_val = (self.urgency_val * (1 - alpha)) + (raw_urgency * alpha)

        # 8. 決策邏輯 (Debounce 防抖動)
        TRIGGER_THRESHOLD = 0.45
        CONFIRMATION_FRAMES = 4

        if self.urgency_val > TRIGGER_THRESHOLD:
            self.high_urgency_counter += 1
        else:
            self.high_urgency_counter = 0

        # 持續滿足條件 5 幀後才切換模式
        if self.high_urgency_counter >= CONFIRMATION_FRAMES:
            self.blended_conf = min(1.0, self.blended_conf + 0.1)
        else:
            self.blended_conf = max(0.0, self.blended_conf - 0.2)

        threshold = 0.4 if self.mode == 'blended' else 0.75
        self.mode = 'blended' if self.blended_conf > threshold else 'acc'
