"""
Dynamic Turn Speed Controller (DTSC) - v27 物理重生版 (Golden Right Foot Hybrid)
特色：
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
LAT_LIMIT_V  = [1.9, 1.9, 2.0,  2.3,  2.3,  2.3,  2.3,  2.4,  2.5]

DECEL_BP = np.array([1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0])
DECEL_V  = np.array([0.0, -0.1, -0.3, -0.5, -0.8, -1.2, -1.4, -1.6, -2.0])

MAX_COMFORT_DECEL = -2.0       
EMERGENCY_DECEL   = -3.0       
MIN_CURVE_DISTANCE = 10.0      
MAX_EXIT_ACCEL = 0.8  # [優化] 出彎最大加速度從 0.6 放寬至 0.8，改善動力銜接         

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

        self.params = Params()
        self.is_enabled = self.params.get_bool("dp_lon_dtsc")
        self.toggle_check_timer = 0.0
        
        cloudlog.info(f"DTSC (v27 Golden Right Foot Hybrid): 初始化完成. Aggressiveness={self.aggressiveness:.2f}")

    def set_aggressiveness(self, value):
        self.aggressiveness = clamp(value, 0.5, 1.8)

    def _is_model_valid(self, model_msg):
        try:
            return (len(model_msg.position.x) == ModelConstants.IDX_N and
                    len(model_msg.velocity.x) == ModelConstants.IDX_N and
                    len(model_msg.orientationRate.z) == ModelConstants.IDX_N)
        except Exception:
            return False

    def _compute_model_arrays(self, model_msg):
        v_arr = np.array(model_msg.velocity.x)
        pos_x_arr = np.array(model_msg.position.x)
        pos_y_arr = np.array(model_msg.position.y) 
        yaw_arr = np.array(model_msg.orientationRate.z)

        v_pred = np.interp(T_IDXS_MPC, MODEL_T_IDXS, v_arr)
        pos_x = np.interp(T_IDXS_MPC, MODEL_T_IDXS, pos_x_arr)
        pos_y = np.interp(T_IDXS_MPC, MODEL_T_IDXS, pos_y_arr) 
        yaw = np.interp(T_IDXS_MPC, MODEL_T_IDXS, yaw_arr)

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
        if np.all(speed_excess <= 0.0): return 0.0, None, None

        critical_idx = int(np.argmax(speed_excess))
        critical_rel_dist = max(rel_pos[critical_idx], 1.0)

        decel_by_distance = (safe_speeds[critical_idx] ** 2 - v_ego ** 2) / (2.0 * critical_rel_dist)
        decel_by_distance = min(decel_by_distance, 0.0)

        if critical_rel_dist <= MIN_CURVE_DISTANCE:
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

        self.suggested_speed = self.output_v_target
        self.output_a_target = self.smoothed_a_target  # 供 Planner 判斷煞車狀態

        # ==========================================================
        # [輸出至 MPC 陣列 (將平滑後的極限餵給大腦)]
        # ==========================================================
        if self.active:
            pass_decel = self.smoothed_a_target if self.smoothed_a_target < 0 else 0.0
            # [修復] critical_idx 為 None (代表已無需煞車超速) 時，改用 0.0 而非 np.max(rel_pos)，
            # 否則會讓 rel_pos[i] <= critical_distance 對整個 horizon 恆成立，導致下方
            # is_physically_in_curve 的出彎油門壓制永遠進不了 else 分支而失效。
            critical_distance = rel_pos[critical_idx] if critical_idx is not None else 0.0
            critical_distance = max(critical_distance, 1e-3)

            # 實體車身檢測：方向盤尚未回正，車身還在明顯彎中
            # [修復] 門檻改為隨 aggressiveness 縮放，避免高攻擊性設定下出彎油門解放過慢
            curve_exit_threshold = 1.0 * self.aggressiveness
            is_physically_in_curve = predicted_lat_accels[0] > curve_exit_threshold

            for i in range(horizon_len):
                if rel_pos[i] <= critical_distance + 1e-6:
                    if pass_decel < 0:
                        a_max[i] = min(a_max[i], pass_decel)
                else:
                    if is_physically_in_curve:
                        a_max[i] = min(a_max[i], MAX_EXIT_ACCEL)

        for i in range(horizon_len):
            if a_max[i] < a_min[i]:
                a_min[i] = a_max[i] - 0.05

        return a_min, a_max

    def update(self, *args, **kwargs):
        # 保留空的 update 方法，防止 planner 副程式定期呼叫時引發 AttributeError
        pass
