"""
Dynamic Turn Speed Controller (DTSC) - Final Production Edition
版本: v25 Final (High Res BP)
日期: 2026-01-13
"""

import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.common.swaglog import cloudlog

# =============================
# [一、核心參數設定]
# =============================
MODEL_T_IDXS = ModelConstants.T_IDXS
DT_MPC = 0.05  # MPC 控制週期 (0.05秒)

# --- Smart Log 設定 ---
FILE_LOG_ENABLED = False

# --- 安全係數 ---
SAFETY_SPEED_FACTOR = 0.95

# --- 1. 側向加速度極限表 (LAT_LIMIT) [G值天花板核心] ---
# 用途：定義車輛在不同速度下，允許的最大側向加速度 (即過彎的"G值天花板")
# 邏輯：速度越快，為了安全與舒適，允許的 G 值通常會稍微提高或保持穩定，但不能無限大。

# [速度節點] 單位: m/s (已換算為 km/h)
# 5.0=18km/h, 10.0=36km/h, 15.0=54km/h, 20.0=72km/h, 25.0=90km/h, 30.0=108km/h
LAT_LIMIT_BP = [5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0]

# [G值天花板] 單位: m/s² (除以 9.8 約等於 G力)
# 1.6 m/s² ≈ 0.16G (舒適)
# 2.0 m/s² ≈ 0.20G (一般過彎)
# 2.3 m/s² ≈ 0.23G (稍微激進)
LAT_LIMIT_V  = [1.9, 1.9, 2.0,  2.0,  2.2,  2.3,  2.3,  2.4,  2.5]

# --- 2. 5段式漸進減速邏輯 (DECEL) ---
# 用途：當預測的側向 G 值超過多少時，需要減速多少
# BP: 預測的側向加速度 (m/s²)
# V : 對應的減速度 (m/s²), 負值代表煞車
DECEL_BP = np.array([1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0])
DECEL_V  = np.array([0.0, -0.2, -0.5, -0.9, -1.2, -1.5, -1.8, -2.1, -2.4])
# --- 減速度與遲滯限制 ---
MAX_COMFORT_DECEL = -2.0       # 最大舒適減速度 (-2.0 m/s²)
EMERGENCY_DECEL   = -3.0       # 緊急減速極限
MIN_CURVE_DISTANCE = 10.0      # 彎道距離小於 10m 視為緊急
MAX_EXIT_ACCEL = 0.4           # 出彎最大加速度

# ==========================================================
# [3. 智慧備援機制參數 (Smart Backup v25)]
# ==========================================================
BACKUP_LAT_G_TH = 1.0          # [觸發閾值] 當預測側向 G > 1.0 m/s² (約0.12G) 時進入監控
BACKUP_BASE_DECEL = -0.5       # 備援基礎減速

# --- [A. 切西瓜 (Inner Cut)] ---
# 車輛往彎道內側偏移 (壓線)
INNER_DEV_TH = 2.0             # 容許內側偏移量 (公尺)
INNER_GAIN_PER_10CM = 0.04     # 每偏 10cm 增加的煞車力道
INNER_MAX_DECEL = -0.5         # 內側最大修正減速

# --- [B. 外拋 (Outer Wide)] ---
# 車輛往彎道外側偏移 (推頭) -> 危險度較高，參數較嚴格
OUTER_DEV_TH = 0.1             # 容許外側偏移量 (僅 10cm)
OUTER_GAIN_PER_10CM = 0.4      # 每偏 10cm 增加的煞車力道 (很強)
OUTER_MAX_DECEL = -3.5         # 外拋最大修正減速 (強力煞車)

# --- 濾波器與防抖動參數 ---
LPF_ALPHA = 0.15                
LPF_RESET_TIME = 2.0           
LPF_RESET_LAT_ACC_THRESHOLD = 0.3 

# --- 誤判防護 ---
PERSISTENCE_MIN_FRAC = 0.6     
CURVATURE_MIN_FOR_PERSIST = 0.01 
SHORT_DIST_IGNORE = 3.5        
SCCV_ABORT_PRED_LAT_ACC_TH = 0.7 
FUTURE_CURVE_THRESHOLD = 0.015 
HYSTERESIS_TIME = 1.5          

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
        self.suggested_speed = 255.0
        self.lpf_reset_timer = 0.0
        self.last_log_time = 0.0
        cloudlog.info(f"DTSC (v25 High Res): 初始化完成. Aggressiveness={self.aggressiveness:.2f}")

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
        # 根據當前車速查表，找出對應的"G值天花板" (LAT_LIMIT_V)
        # 並乘上 aggressiveness (激進係數) 進行調整
        raw_lat_limits = np.interp(v_pred, LAT_LIMIT_BP, LAT_LIMIT_V) * self.aggressiveness

        if self.filtered_lat_limits is None:
            self.filtered_lat_limits = raw_lat_limits
        else:
            self.filtered_lat_limits = (LPF_ALPHA * raw_lat_limits) + \
                                       ((1.0 - LPF_ALPHA) * self.filtered_lat_limits)

        current_lat_limits = np.maximum(self.filtered_lat_limits, 1.0)
        v_clip = np.clip(v_pred, 1.0, 100.0)
        curvatures = np.abs(yaw_rates / v_clip)
        
        # 核心公式：V_max = sqrt( a_lat_max / curvature )
        # 計算出在 G 值天花板限制下的最高安全速度
        safe_speeds = np.sqrt(current_lat_limits / (curvatures + 1e-6)) * SAFETY_SPEED_FACTOR
        return safe_speeds, curvatures

    def _compute_sp_decel(self, predicted_lat_acc_max):
        if predicted_lat_acc_max <= DECEL_BP[0]: return 0.0
        final_decel = interp_clamped(predicted_lat_acc_max, DECEL_BP, DECEL_V)
        return clamp(final_decel, EMERGENCY_DECEL, 0.0)

    def _compute_dtsc_decel(self, v_ego, v_pred, rel_pos, safe_speeds):
        speed_excess = v_pred - safe_speeds
        if np.all(speed_excess <= 0.0): return 0.0, None, None

        critical_idx = int(np.argmax(speed_excess))
        critical_rel_dist = max(rel_pos[critical_idx], 1e-3)

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
        """DTSC 核心入口"""
        horizon_len = len(T_IDXS_MPC)
        a_min = np.ones(horizon_len) * (base_a_min if np.isscalar(base_a_min) else base_a_min[0])
        a_max = np.array(base_a_max) if not np.isscalar(base_a_max) else np.ones(horizon_len) * base_a_max

        if not self._is_model_valid(model_msg):
            self.filtered_lat_limits = None 
            self.lpf_reset_timer = 0
            self.active = False
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
        if len(safe_speeds) > 0:
            self.suggested_speed = float(np.min(safe_speeds))

        sp_decel = self._compute_sp_decel(predicted_lat_acc_max) 
        dt_decel, critical_idx, dt_mode = self._compute_dtsc_decel(v_ego, v_pred, rel_pos, safe_speeds)

        speed_excess = v_pred - safe_speeds
        mask_curve = curvatures > CURVATURE_MIN_FOR_PERSIST
        mask_speed = speed_excess > 0.01
        mask = np.logical_and(mask_speed, mask_curve)
        persistence_ok = (float(np.sum(mask)) / len(mask)) >= PERSISTENCE_MIN_FRAC if len(mask) > 0 else False
        critical_dist = rel_pos[critical_idx] if critical_idx is not None else 999.0

        if predicted_lat_acc_max < SCCV_ABORT_PRED_LAT_ACC_TH:
            dt_decel = sp_decel = 0.0
            dt_mode = None
            self.suggested_speed = 255.0 
        elif not persistence_ok and critical_dist < SHORT_DIST_IGNORE:
            dt_decel = sp_decel = 0.0
            dt_mode = None
            self.suggested_speed = 255.0

        final_required_decel = dt_decel if dt_mode == "EMERGENCY" else min(sp_decel, dt_decel)
        final_required_decel = clamp(final_required_decel, EMERGENCY_DECEL, 0.0)

        # ==========================================================
        # [8. 智慧備援機制 (Smart Backup v25)]
        # ==========================================================
        check_idx = 5 
        current_y = pred_y[check_idx]      
        current_yaw = yaw_rates[check_idx] 
        current_lane_deviation = abs(current_y)
        backup_triggered = False

        # (1) 判斷偏移性質
        is_cutting_corner = (current_y * current_yaw) > 0

        # (2) 設定參數
        if is_cutting_corner:
            actual_dev_th = INNER_DEV_TH        
            gain_per_10cm = INNER_GAIN_PER_10CM 
            max_backup_decel = INNER_MAX_DECEL  
            dev_type_str = "切西瓜(Inner)"
        else:
            actual_dev_th = OUTER_DEV_TH        
            gain_per_10cm = OUTER_GAIN_PER_10CM 
            max_backup_decel = OUTER_MAX_DECEL  
            dev_type_str = "外拋(Outer)⚠️"

        # (3) 觸發與計算
        if predicted_lat_acc_max > BACKUP_LAT_G_TH and current_lane_deviation > actual_dev_th:
            excess_dev_m = current_lane_deviation - actual_dev_th
            excess_units = excess_dev_m * 10.0
            extra_brake = excess_units * gain_per_10cm
            backup_required = BACKUP_BASE_DECEL - extra_brake
            backup_required = max(backup_required, max_backup_decel)

            if backup_required < final_required_decel:
                final_required_decel = backup_required
                backup_triggered = True

                if FILE_LOG_ENABLED and (time.monotonic() - self.last_log_time > 0.2):
                    log_msg = f"備援觸發[{dev_type_str}]: G={predicted_lat_acc_max:.2f}, Dev={current_lane_deviation:.2f}m, 單位數={excess_units:.1f}, Req={backup_required:.2f}"
                    write_file_log(log_msg)
                    self.last_log_time = time.monotonic()
        # ==========================================================

        # 9. 遲滯邏輯
        if final_required_decel < -0.1 or backup_triggered:
            self.hysteresis_timer = HYSTERESIS_TIME
            self.active = True
        else:
            if self.hysteresis_timer > 0:
                self.hysteresis_timer -= DT_MPC
                self.active = True
            else:
                self.active = False
                self.hysteresis_timer = 0

        # 10. 應用 MPC 限制
        if self.active:
            pass_decel = final_required_decel if final_required_decel < 0 else 0.0
            critical_distance = rel_pos[critical_idx] if critical_idx is not None else np.max(rel_pos)
            critical_distance = max(critical_distance, 1e-3)

            if FILE_LOG_ENABLED and not backup_triggered:
                current_time = time.monotonic()
                if (current_time - self.last_log_time > 1.0) and (pass_decel < -0.2):
                    log_msg = f"DTSC介入: 車速={v_ego*3.6:.0f} | 減速G={pass_decel:.2f} | 建議速={self.suggested_speed*3.6:.0f}"
                    write_file_log(log_msg)
                    self.last_log_time = current_time

            has_future_curve = any(
                rel_pos[i] > critical_distance and curvatures[i] > FUTURE_CURVE_THRESHOLD
                for i in range(horizon_len)
            )

            for i in range(horizon_len):
                if rel_pos[i] <= critical_distance + 1e-6:
                    if pass_decel < 0:
                        a_max[i] = min(a_max[i], pass_decel)
                else:
                    if has_future_curve:
                        a_max[i] = min(a_max[i], MAX_EXIT_ACCEL)

        for i in range(horizon_len):
            if a_max[i] < a_min[i]:
                a_min[i] = a_max[i] - 0.05

        return a_min, a_max
