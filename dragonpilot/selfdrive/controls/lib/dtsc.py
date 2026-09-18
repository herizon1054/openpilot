"""
Dynamic Turn Speed Controller (DTSC) - v29 出彎狀態機重新設計版
基於 v28 台灣路況強化版修改，v28 基於 v27 物理重生版 (Golden Right Foot Hybrid)。

=== v29 變更紀錄 (相對於 v28 的刻意分歧，供其他 fork 開發者參照) ===
1. [修正既有漏洞] _smooth_yaw_rate() 原本的中位數濾波迴圈為
   range(1, n-1)，頭尾兩點（index 0 與 -1）完全沒被濾波。index 0
   正是「現在」這個時間點，也是出彎判斷唯一使用的輸入，等於出彎判斷
   吃的正好是唯一沒被雜訊防護覆蓋到的點。v29 補上邊界點的平均濾波。
2. [重新設計] 出彎節流閥從「二元判斷 + 固定值 0.8」改為連續、逐幀
   重新評估的比例控制：記錄本次彎道的曲率峰值 peak_curvature_active，
   每一幀用「現在曲率 / 峰值曲率」算出比例（做 EMA 平滑），線性映射到
   EXIT_CEILING_MIN(彎心，接近收油) ~ EXIT_CEILING_FULL(接近出彎，
   0.8) 之間，取代原本的一次性二元切換。
3. [新增] 出彎確認 debounce（STRAIGHT_RATIO_THRESHOLD /
   STRAIGHT_CONFIRM_FRAMES）：比例連續多幀低於門檻才判定「真的出彎
   完成」，此時完全解除節流閥限制，把控制權交還上游，而非永遠卡在
   0.8 上限。
   ⚠️ 這套比例仍然只反映曲率的相對變化，不代表車輛此時處於安全狀態的
   絕對保證；EXIT_CEILING_FULL/MIN 的實際數值同樣是初步方向性設計，
   未經實測調校，需以路測 log 迭代驗證。

=== v28 變更紀錄 (相對於 v27 的刻意分歧，供其他 fork 開發者參照) ===
1. [新增] _smooth_yaw_rate()：對 horizon 上的 yaw_rate 陣列套用 window=3
   中位數濾波，消除模型在單一 frame（常見於下坡接彎的相機 pitch 角瞬變）
   產生的孤立曲率尖峰。這是「防單點雜訊」的第一道防線，在資料源頭處理。
2. [新增] EMERGENCY_CONFIRM_FRAMES 時序確認機制：原本只要當前這一幀判定
   「臨界彎距 ≤ MIN_CURVE_DISTANCE」就立刻解除 -2.0 舒適上限、放行到
   -3.0，單幀雜訊會直接造成一次重踩。v28 改為要求該判定連續
   EMERGENCY_CONFIRM_FRAMES 個規劃週期都成立才真正升級為 EMERGENCY，
   未達確認次數前一律鎖在 COMFORT 的 -2.0 上限內。真實存在的近距離彎道
   會持續被偵測到，不受影響；單幀誤判會被過濾掉。
   ⚠️ 已知限制：目前 get_mpc_constraints() 沒有坡度/pitch 資料輸入
   （longitudinal_planner.py 呼叫時只傳入 model_msg/v_ego/base_a_min/
   base_a_max），因此本版本無法對下坡做真正的重力分量補償，此為架構
   限制而非本次修正遺漏，若要做需額外從 liveLocationKalman 等服務接入
   坡度資訊並重新設計 decel_by_distance 的物理模型。
3. [調整] LAT_LIMIT_V 表格中高速段（15~30 m/s，約 54~108km/h）數值上調，
   低速段（5~10 m/s，市區路口/巷弄）維持原值不變。此調整為初步方向性
   修正，非精確驗證值，建議以實際路測 log（suggested_speed vs 實際限速/
   彎道行為）持續迭代。

原 v27 特色：
1. 融合 v27 MPC 陣列規劃與 10 秒低頻 UI 開關檢查，架構最現代化。
2. 融合 Candy 版「老司機黃金右腳」：引進加速度低通濾波 (LPF) 與速度階梯爬升。
3. 採用「出彎實體壓制」+「狀態死咬 (Hysteresis Recovery)」雙重出彎防護。
4. 保留 v27 防變道急煞濾波參數，完美免疫高速公路變換車道的幽靈急煞。
5. 移除 HTD，保留 0.95 靜態安全緩衝係數以優化休旅車高重心側傾體感。
"""

import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

# =============================
# [一、核心參數設定]
# =============================
MODEL_T_IDXS = ModelConstants.T_IDXS
DT_MPC = 0.05  

FILE_LOG_ENABLED = False

LAT_LIMIT_BP = [5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0]
# [v28 調整] 中高速段（15~30 m/s）數值上調，低速段（5~10 m/s）維持不變。
# 原值：[1.9, 1.9, 2.0,  2.3,  2.4,  2.4,  2.5,  2.6,  2.7]
LAT_LIMIT_V  = [1.9, 1.9, 2.1,  2.4,  2.4,  2.5,  2.6,  2.6,  2.7]

DECEL_BP = np.array([1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0])
DECEL_V  = np.array([0.0, -0.1, -0.3, -0.5, -0.8, -1.2, -1.4, -1.6, -2.0])

MAX_COMFORT_DECEL = -2.0       
EMERGENCY_DECEL   = -3.0       
MIN_CURVE_DISTANCE = 10.0      
MAX_EXIT_ACCEL = 0.8  # [優化] 出彎最大加速度從 0.6 放寬至 0.8，改善動力銜接         

# [v28 新增] EMERGENCY 時序確認所需的連續規劃週期數（防單點雜訊核心參數）。
# 3 幀 * DT_MPC(0.05s) ≈ 150ms 延遲，用來過濾單幀雜訊尖峰；
# 真實存在的近距離彎道會持續被偵測到，不受此延遲影響安全性。
EMERGENCY_CONFIRM_FRAMES = 3

# ==========================================================
# [v29 新增：出彎節流閥狀態機參數]
# ==========================================================
EXIT_CEILING_MIN = 0.3   # 仍在彎心附近（曲率比例接近峰值）時的節流閥上限，近似收油滑行
EXIT_CEILING_FULL = MAX_EXIT_ACCEL  # 曲率比例趨近 0（快出彎）時的節流閥上限，沿用原 0.8
EXIT_RATIO_LPF_ALPHA = 0.2   # 「現在曲率/峰值曲率」比例的 EMA 平滑係數，避免逐幀跳動造成油門忽大忽小
STRAIGHT_RATIO_THRESHOLD = 0.15   # 比例低於此值視為候選「已出彎」
STRAIGHT_CONFIRM_FRAMES = 3       # 連續多少幀候選成立才真正判定出彎完成、解除節流閥限制

# ==========================================================
# [二、防變道急煞與狀態平滑參數 (保留 v27 免疫幽靈急煞設定)]
# ==========================================================
LPF_ALPHA = 0.15                
LPF_RESET_TIME = 2.0           
LPF_RESET_LAT_ACC_THRESHOLD = 0.3 
PERSISTENCE_MIN_FRAC = 0.6     
CURVATURE_MIN_FOR_PERSIST = 0.01 
SHORT_DIST_IGNORE = 3.5        
SCCV_ABORT_PRED_LAT_ACC_TH = 0.7 
FUTURE_CURVE_THRESHOLD = 0.015 
HYSTERESIS_TIME = 0.5          

# =============================
# 工具函式
# =============================
def clamp(x, low, high):
    return max(low, min(high, x))

def interp_clamped(x, bp, fp):
    if x <= bp[0]: return fp[0]
    if x >= bp[-1]: return fp[-1]
    return float(np.interp(x, bp, fp))

def write_file_log(msg):
    if not FILE_LOG_ENABLED: return
    try:
        with open("/data/media/0/dtsc_log.txt", "a") as f:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            f.write(f"[{timestamp}] {msg}\n")
    except Exception:
        pass

# =============================
# DTSC 主類別
# =============================
class DTSC:
    def __init__(self, aggressiveness=1.0, **kwargs):
        self.aggressiveness = clamp(aggressiveness, 0.5, 1.8)
        self.active = False
        self.hysteresis_timer = 0.0
        self.filtered_lat_limits = None
        self.suggested_speed = V_CRUISE_MAX
        self.lpf_reset_timer = 0.0
        self.last_log_time = 0.0
        
        # 補回靜態安全緩衝係數 (0.95 提供 5% 的體感降速緩衝)
        self.safety_speed_factor = 0.95
        
        # [Candy 融合] 黃金右腳狀態變數
        self.smoothed_a_target = 0.0  
        self.output_v_target = V_CRUISE_MAX
        self.output_a_target = 0.0 

        # [v28 新增] EMERGENCY 時序確認計數器（防單點雜訊）
        self.emergency_confirm_counter = 0

        # [v29 新增] 出彎節流閥狀態機變數
        self.peak_curvature_active = 0.0   # 本次彎道事件中觀測到的曲率峰值
        self.exit_ratio_smoothed = 0.0     # 「現在曲率/峰值曲率」的 EMA 平滑值
        self.straight_confirm_counter = 0  # 連續判定「已出彎」的幀數

        self.params = Params()
        self.is_enabled = self.params.get_bool("dp_lon_dtsc")
        self.toggle_check_timer = 0.0
        
        cloudlog.info(f"DTSC (v29 Exit State Machine): 初始化完成. Aggressiveness={self.aggressiveness:.2f}")

    def set_aggressiveness(self, value):
        self.aggressiveness = clamp(value, 0.5, 1.8)

    def _is_model_valid(self, model_msg):
        try:
            return (len(model_msg.position.x) == ModelConstants.IDX_N and
                    len(model_msg.velocity.x) == ModelConstants.IDX_N and
                    len(model_msg.orientationRate.z) == ModelConstants.IDX_N)
        except Exception:
            return False

    def _smooth_yaw_rate(self, yaw_arr):
        """
        [v28 新增：防單點雜訊]
        對 yaw_rate 陣列做 window=3 的中位數濾波，消除模型在單一 frame
        （例如緩降坡接彎道的相機 pitch 角瞬變）產生的孤立曲率尖峰。
        真實存在的彎道曲率會延續在相鄰多個時間點，中位數濾波不會將其抹除，
        僅對「僅單一時間點異常」的雜訊有效。
        """
        n = len(yaw_arr)
        if n < 3:
            return yaw_arr
        smoothed = yaw_arr.copy()
        for i in range(1, n - 1):
            smoothed[i] = np.median(yaw_arr[i - 1:i + 2])
        # [v29 修正] 邊界點（尤其 index 0，即「現在」這個時間點，出彎判斷
        # 直接使用此點）原本完全未濾波，改用僅有的兩個相鄰點取平均
        # （邊界只有 2 點可用，中位數對 2 點無意義，故用平均）。
        smoothed[0] = (yaw_arr[0] + yaw_arr[1]) / 2.0
        smoothed[-1] = (yaw_arr[-2] + yaw_arr[-1]) / 2.0
        return smoothed

    def _compute_model_arrays(self, model_msg):
        v_arr = np.array(model_msg.velocity.x)
        pos_x_arr = np.array(model_msg.position.x)
        pos_y_arr = np.array(model_msg.position.y) 
        yaw_arr = np.array(model_msg.orientationRate.z)

        v_pred = np.interp(T_IDXS_MPC, MODEL_T_IDXS, v_arr)
        pos_x = np.interp(T_IDXS_MPC, MODEL_T_IDXS, pos_x_arr)
        pos_y = np.interp(T_IDXS_MPC, MODEL_T_IDXS, pos_y_arr) 
        yaw = np.interp(T_IDXS_MPC, MODEL_T_IDXS, yaw_arr)
        yaw = self._smooth_yaw_rate(yaw)  # [v28] 防單點雜訊濾波

        rel_pos = pos_x - pos_x[0]
        rel_pos = np.maximum(rel_pos, 0.0)
        return v_pred, rel_pos, yaw, pos_y

    def _compute_safe_speeds(self, v_pred, yaw_rates):
        raw_lat_limits = np.interp(v_pred, LAT_LIMIT_BP, LAT_LIMIT_V) * self.aggressiveness

        if self.filtered_lat_limits is None:
            self.filtered_lat_limits = raw_lat_limits
        else:
            self.filtered_lat_limits = (LPF_ALPHA * raw_lat_limits) + \
                                       ((1.0 - LPF_ALPHA) * self.filtered_lat_limits)

        current_lat_limits = np.maximum(self.filtered_lat_limits, 1.0)
        v_clip = np.clip(v_pred, 1.0, 100.0)
        curvatures = np.abs(yaw_rates / v_clip)
        
        # 套用 0.95 安全係數，讓彎道極限略微收斂
        safe_speeds = np.sqrt(current_lat_limits / (curvatures + 1e-6)) * self.safety_speed_factor
        return safe_speeds, curvatures

    def _compute_sp_decel(self, predicted_lat_acc_max):
        if predicted_lat_acc_max <= DECEL_BP[0]: return 0.0
        final_decel = interp_clamped(predicted_lat_acc_max, DECEL_BP, DECEL_V)
        return clamp(final_decel, EMERGENCY_DECEL, 0.0)

    def _compute_dtsc_decel(self, v_ego, v_pred, rel_pos, safe_speeds):
        speed_excess = v_pred - safe_speeds
        if np.all(speed_excess <= 0.0):
            self.emergency_confirm_counter = 0  # [v28] 無超速候選時歸零確認計數
            return 0.0, None, None

        critical_idx = int(np.argmax(speed_excess))
        critical_rel_dist = max(rel_pos[critical_idx], 1.0)

        decel_by_distance = (safe_speeds[critical_idx] ** 2 - v_ego ** 2) / (2.0 * critical_rel_dist)
        decel_by_distance = min(decel_by_distance, 0.0)

        is_emergency_candidate = critical_rel_dist <= MIN_CURVE_DISTANCE

        # ==========================================================
        # [v28 新增：EMERGENCY 時序確認機制 — 防單點雜訊第二道防線]
        # 近距離急彎候選需連續 EMERGENCY_CONFIRM_FRAMES 個規劃週期都成立，
        # 才真正解除 -2.0 舒適上限、授權到 -3.0。單幀雜訊尖峰無法連續
        # 重現，會被擋在 COMFORT 分支內；真實存在的近距離彎道會持續被
        # 偵測到，僅多付出約 EMERGENCY_CONFIRM_FRAMES * DT_MPC 秒的反應延遲。
        # ==========================================================
        if is_emergency_candidate:
            self.emergency_confirm_counter = min(self.emergency_confirm_counter + 1, EMERGENCY_CONFIRM_FRAMES)
        else:
            self.emergency_confirm_counter = 0

        if is_emergency_candidate and self.emergency_confirm_counter >= EMERGENCY_CONFIRM_FRAMES:
            mode = 'EMERGENCY'
        else:
            mode = 'COMFORT'
            decel_by_distance = max(decel_by_distance, MAX_COMFORT_DECEL)
        return decel_by_distance, critical_idx, mode

    def get_suggested_speed(self):
        return self.suggested_speed

    def get_mpc_constraints(self, model_msg, v_ego, base_a_min, base_a_max, **kwargs):
        # 10 秒檢查一次 UI 開關
        self.toggle_check_timer += DT_MPC
        if self.toggle_check_timer >= 10.0:
            self.is_enabled = self.params.get_bool("dp_lon_dtsc")
            self.toggle_check_timer = 0.0

        horizon_len = len(T_IDXS_MPC)
        a_min = np.ones(horizon_len) * (base_a_min if np.isscalar(base_a_min) else base_a_min[0])
        a_max = np.array(base_a_max) if not np.isscalar(base_a_max) else np.ones(horizon_len) * base_a_max

        if not self.is_enabled:
            self.active = False
            self.suggested_speed = V_CRUISE_MAX
            self.output_v_target = V_CRUISE_MAX
            self.output_a_target = a_max[0]
            self.smoothed_a_target = 0.0
            self.emergency_confirm_counter = 0
            self.peak_curvature_active = 0.0
            self.exit_ratio_smoothed = 0.0
            self.straight_confirm_counter = 0
            return a_min, a_max

        if not self._is_model_valid(model_msg):
            self.filtered_lat_limits = None 
            self.lpf_reset_timer = 0

            # [修復] 若上一輪仍在主動煞車，本輪 model 無效時先延續上一輪的減速度上限，
            # 避免 model 短暫抖動的那一幀完全放飛油門，造成「放→收」的頓挫；
            # 同時把 hysteresis_timer / output_v_target / output_a_target 一併歸零，
            # 避免 model 恢復後殘留的 hysteresis 狀態導致不必要的二次煞車。
            if self.active and self.smoothed_a_target < 0:
                for i in range(horizon_len):
                    a_max[i] = min(a_max[i], self.smoothed_a_target)
                    if a_max[i] < a_min[i]:
                        a_min[i] = a_max[i] - 0.05

            self.active = False
            self.hysteresis_timer = 0.0
            self.smoothed_a_target = 0.0
            self.output_v_target = V_CRUISE_MAX
            self.output_a_target = 0.0
            self.emergency_confirm_counter = 0
            self.peak_curvature_active = 0.0
            self.exit_ratio_smoothed = 0.0
            self.straight_confirm_counter = 0
            return a_min, a_max

        v_pred, rel_pos, yaw_rates, pred_y = self._compute_model_arrays(model_msg)
        predicted_lat_accels = np.abs(v_pred * yaw_rates)
        predicted_lat_acc_max = float(np.max(predicted_lat_accels))

        if predicted_lat_acc_max < LPF_RESET_LAT_ACC_THRESHOLD:
             self.lpf_reset_timer += DT_MPC
             if self.lpf_reset_timer > LPF_RESET_TIME:
                 self.filtered_lat_limits = None
                 self.lpf_reset_timer = 0
        else:
             self.lpf_reset_timer = 0

        safe_speeds, curvatures = self._compute_safe_speeds(v_pred, yaw_rates)
        raw_suggested_speed = float(np.min(safe_speeds)) if len(safe_speeds) > 0 else V_CRUISE_MAX

        sp_decel = self._compute_sp_decel(predicted_lat_acc_max) 
        dt_decel, critical_idx, dt_mode = self._compute_dtsc_decel(v_ego, v_pred, rel_pos, safe_speeds)

        speed_excess = v_pred - safe_speeds
        mask_curve = curvatures > CURVATURE_MIN_FOR_PERSIST
        mask_speed = speed_excess > 0.01
        mask = np.logical_and(mask_speed, mask_curve)
        persistence_ok = (float(np.sum(mask)) / len(mask)) >= PERSISTENCE_MIN_FRAC if len(mask) > 0 else False
        
        # [修復] 防變道急煞核心判斷，修正 999 漏洞確保無危險時正常套用短距離忽略
        critical_dist = rel_pos[critical_idx] if critical_idx is not None else 0.0

        if predicted_lat_acc_max < SCCV_ABORT_PRED_LAT_ACC_TH:
            dt_decel = sp_decel = 0.0
            dt_mode = None
            raw_suggested_speed = V_CRUISE_MAX 
        elif not persistence_ok and critical_dist < SHORT_DIST_IGNORE:
            dt_decel = sp_decel = 0.0
            dt_mode = None
            raw_suggested_speed = V_CRUISE_MAX

        # [優化：線性過渡] 車速 10~21 km/h (3.0 ~ 6.0 m/s) 間漸進套用 DTSC，避免切換瞬間頓挫
        if 3.0 <= v_ego < 6.0:
            fade_factor = (v_ego - 3.0) / 3.0  
            dt_decel *= fade_factor
            sp_decel *= fade_factor

        final_required_decel = dt_decel if dt_mode == "EMERGENCY" else min(sp_decel, dt_decel)
        final_required_decel = clamp(final_required_decel, EMERGENCY_DECEL, 0.0)

        # [核心修復] 低速防卡死機制 (根除起步死鎖)：
        # 當車速極低時，強制清空所有狀態，防止死咬機制 (hysteresis) 卡在 True 導致系統封殺油門
        if v_ego < 3.0:
            self.active = False
            self.hysteresis_timer = 0.0
            self.output_v_target = V_CRUISE_MAX
            self.smoothed_a_target = 0.0
            final_required_decel = 0.0
            raw_suggested_speed = V_CRUISE_MAX
            self.emergency_confirm_counter = 0
            self.peak_curvature_active = 0.0
            self.exit_ratio_smoothed = 0.0
            self.straight_confirm_counter = 0

        # ==========================================================
        # [Candy 融合：狀態死咬 (Hysteresis Recovery)]
        # ==========================================================
        is_recovering = False
        if self.active:
            # 判斷是否在恢復期：煞車還沒放平，或是目標速度還沒爬完
            brake_recovering = self.smoothed_a_target < -0.05
            speed_recovering = self.output_v_target < (raw_suggested_speed - 0.5)
            is_recovering = brake_recovering or speed_recovering

        if final_required_decel < -0.1:
            self.hysteresis_timer = HYSTERESIS_TIME
            self.active = True
        else:
            if self.hysteresis_timer > 0 or (self.active and is_recovering):
                if self.hysteresis_timer > 0:
                    self.hysteresis_timer -= DT_MPC
                self.active = True
                final_required_decel = 0.0  # 進入平滑釋放模式
            else:
                self.active = False
                self.hysteresis_timer = 0

        # ==========================================================
        # [Candy 融合：黃金右腳濾波 (速度 + 加速度雙重平滑)]
        # ==========================================================
        if self.active:
            # A. 加速度平滑 (LPF)
            # [優化] 降低濾波係數 (0.15 -> 0.08) 讓煞車踩放更像人類的緩踩緩放
            raw_a_target = float(clamp(final_required_decel, EMERGENCY_DECEL, MAX_EXIT_ACCEL))
            alpha_a = 0.08 
            self.smoothed_a_target = (1.0 - alpha_a) * self.smoothed_a_target + (alpha_a * raw_a_target)

            # B. 速度階梯爬升 (Staircase)
            if raw_suggested_speed < self.output_v_target:
                # 遇到更急的彎，瞬間下拉以保證安全
                self.output_v_target = raw_suggested_speed
            else:
                # 出彎時，每秒最多爬升 2.0 m/s，營造順暢推背感
                max_dv_per_step = 2.0 * DT_MPC
                self.output_v_target = min(self.output_v_target + max_dv_per_step, raw_suggested_speed)
        else:
            self.smoothed_a_target = 0.0
            self.output_v_target = V_CRUISE_MAX
            # [v29] 非 active 狀態時，出彎狀態機一併歸零，避免殘留峰值曲率
            # 影響下一次真正進彎事件的比例計算基準。
            self.peak_curvature_active = 0.0
            self.exit_ratio_smoothed = 0.0
            self.straight_confirm_counter = 0

        self.suggested_speed = self.output_v_target
        self.output_a_target = self.smoothed_a_target  # 供 Planner 判斷煞車狀態

        # ==========================================================
        # [輸出至 MPC 陣列 (將平滑後的極限餵給大腦)]
        # ==========================================================
        if self.active:
            pass_decel = self.smoothed_a_target if self.smoothed_a_target < 0 else 0.0
            # [修復] critical_idx 為 None (代表已無需煞車超速) 時，改用 0.0 而非 np.max(rel_pos)，
            # 否則會讓 rel_pos[i] <= critical_distance 對整個 horizon 恆成立，導致下方
            # 出彎節流閥壓制永遠進不了 else 分支而失效。
            critical_distance = rel_pos[critical_idx] if critical_idx is not None else 0.0
            critical_distance = max(critical_distance, 1e-3)

            # ==========================================================
            # [v29 重新設計：連續式出彎節流閥狀態機]
            # 取代原本「方向盤是否回正」的二元判斷 + 固定 0.8 上限。
            # 邏輯：記錄本次彎道曲率峰值，逐幀用「現在曲率/峰值曲率」算出
            # 比例（越接近 1 代表還在彎心，越接近 0 代表快出彎），做 EMA
            # 平滑後線性映射到節流閥上限；比例連續多幀低於門檻才判定真正
            # 出彎完成，此時完全解除限制、把控制權交還上游。
            # ==========================================================
            current_curvature = float(curvatures[0])
            self.peak_curvature_active = max(self.peak_curvature_active, current_curvature)
            raw_ratio = current_curvature / max(self.peak_curvature_active, 1e-4)
            raw_ratio = clamp(raw_ratio, 0.0, 1.0)
            self.exit_ratio_smoothed = (EXIT_RATIO_LPF_ALPHA * raw_ratio) + \
                                        ((1.0 - EXIT_RATIO_LPF_ALPHA) * self.exit_ratio_smoothed)

            if self.exit_ratio_smoothed < STRAIGHT_RATIO_THRESHOLD:
                self.straight_confirm_counter = min(self.straight_confirm_counter + 1, STRAIGHT_CONFIRM_FRAMES)
            else:
                self.straight_confirm_counter = 0

            curve_confirmed_straight = self.straight_confirm_counter >= STRAIGHT_CONFIRM_FRAMES
            # 比例 1(彎心) -> 上限 EXIT_CEILING_MIN；比例 0(快出彎) -> 上限 EXIT_CEILING_FULL
            exit_accel_ceiling = interp_clamped(self.exit_ratio_smoothed, [0.0, 1.0],
                                                 [EXIT_CEILING_FULL, EXIT_CEILING_MIN])

            for i in range(horizon_len):
                if rel_pos[i] <= critical_distance + 1e-6:
                    if pass_decel < 0:
                        a_max[i] = min(a_max[i], pass_decel)
                else:
                    if not curve_confirmed_straight:
                        a_max[i] = min(a_max[i], exit_accel_ceiling)
                    # 已確認出彎完成：不再施加任何節流閥限制，控制權交還上游

        for i in range(horizon_len):
            if a_max[i] < a_min[i]:
                a_min[i] = a_max[i] - 0.05

        return a_min, a_max

    def update(self, *args, **kwargs):
        # 保留空的 update 方法，防止 planner 副程式定期呼叫時引發 AttributeError
        pass
