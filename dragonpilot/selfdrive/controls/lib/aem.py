"""
Copyright (c) 2025, Rick Lan

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

# Dynamic Experimental Mode（原 AEM / Adaptive Experimental Mode）
# 依「車速」、「側向加速度（過彎程度）」與「接近模型停止線」動態決定縱向控制模式：
#   * 'blended' = 實驗模式 (e2e)
#   * 'acc'     = 一般模式
#
# ════════════════════════════════════════════════════════════════════════════
# ⚠️ 與原廠 AEM 的刻意分歧（intentional divergence，非 upstream 行為）
# ════════════════════════════════════════════════════════════════════════════
#   1. 判斷依據不同：原廠 AEM 只看 modelV2.meta.disengagePredictions.gasPressProbs[1]
#      的機率門檻（0.6 → acc / 0.4 → blended），且每一幀直接拿原始值比較。實測這個值
#      每 50ms 可在 0.06~0.58 之間跳動，會讓 mode 本身逐幀在 acc/blended 之間橫跳。
#      本版本整段移除該機率邏輯，改以車速 + 側向加速度 + 接近停止線判斷，全部經過
#      濾波／遲滯／防彈跳。update_states 因此不使用 radar_msg（僅保留參數維持介面相容）。
#
#   2. 實驗模式開關（selfdriveState.experimentalMode）預設「不」影響結果：
#      原廠 AEM 把開關當成預設值，只在機率門檻成立時偏移；本版本在 AEM 啟用時，
#      get_mode() 預設完全忽略傳入的 mode，車速／彎道／停止線三個條件全權決定 mode，
#      使用者在 UI 切換實驗模式開關不會有任何效果。
#      若希望「關閉實驗模式開關 = 強制一般模式、AEM 只在開關打開時才運作」，
#      把 AEM_EXPERIMENTAL_OFF_FORCES_ACC 改為 True（見常數區）。
#
# ════════════════════════════════════════════════════════════════════════════
# 判斷優先權（由高到低，見 get_mode()）
# ════════════════════════════════════════════════════════════════════════════
#   0. （僅 AEM_EXPERIMENTAL_OFF_FORCES_ACC = True 時）實驗模式開關關閉 -> acc
#   1. 過彎保護：濾波後側向加速度 > CURVE_LAT_ACCEL_ENTER（1.76 m/s²）強制 acc，
#      低於 CURVE_LAT_ACCEL_EXIT（1.30 m/s²）才解除。
#      a_y = |v_ego * yaw_rate|，yaw_rate 取自 modelV2.orientationRate.z[0]（與 dtsc.py 同源）。
#      ⚠️ 刻意不用 carState.yawRate：Toyota 等多數品牌 carstate.py 從未賦值，恆為 0.0，
#      已用 TOYOTA_COROLLA_TSS2 實際路測 rlog 驗證（carState.yawRate 全程 0.0，
#      modelV2.orientationRate.z[0] 全程非零）。
#      ⚠️ 已知行為：a_y = v²/R，20 km/h 半徑 15 m 的一般路口轉彎約 2.06 m/s²，
#      會超過 1.76 觸發本保護，因此低速路口轉彎途中可能由 blended 切到 acc。
#   2. 接近停止線 + 車速閘門：near_stop_active 為 True（距停止線 <= NEAR_STOP_ENTER_M）
#      且車速閘門開放（<= 60 km/h 開、>= 70 km/h 關、中間維持前一狀態）時強制 blended。
#      車速閘門避免快速道路等高速情境遠遠看到停止線就被拉進實驗模式。
#   3. 車速（兩層）：
#        v_ego <= 20 km/h -> 無條件、立即強制實驗模式，不經防彈跳，起步／低速無空窗期
#        20 < v_ego < 30 km/h -> 過渡帶，維持前一狀態
#        v_ego >= 30 km/h -> 一般模式（含防彈跳）
#   防彈跳：所有切換條件需連續成立 CONFIRM_TIME_S（0.5 s）才生效；車速模式另外要求距離
#   上次切換至少 MIN_DWELL_TIME_S（2 s）。過彎、停止線、節流門檻不套用最短維持時間，
#   避免安全保護被延遲。
#
# 另外暴露兩個「只影響節流門檻、不影響 get_mode()」的屬性給 longitudinal_planner.py：
#   * near_stop_active        -> 呼叫端改用較保守的 ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP
#   * base_throttle_threshold -> 依車速動態決定的 blended 基礎節流門檻
#
# 側向加速度門檻參考（同 fork 的 dtsc.py）：
#   DECEL_BP = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0]
#   DECEL_V  = [0.0, -0.1, -0.3, -0.5, -0.8, -1.2, -1.4, -1.6, -2.0]（MAX_COMFORT_DECEL = -2.0）
#   1.76 m/s² 約落在 DTSC 減速度 -0.8 附近，屬於中等彎道；更輕微的彎道留在實驗模式自行處理。
#   ⚠️ 1.76/1.30 為依需求調整的方向性數值，非針對特定車款的實測結論，建議依路測 log 微調。
#
# ════════════════════════════════════════════════════════════════════════════
# 變更紀錄
# ════════════════════════════════════════════════════════════════════════════
#   v1  方向盤角度 > 60° 判斷過彎（易被路口大角度低 G 情境誤觸發）。
#   v2  改用 a_y = |v_ego * yawRate|。
#   v3  過彎門檻上移至 2.0/2.6 m/s²。
#   v4  [bug fix] yawRate 改取 modelV2.orientationRate.z[0]（carState.yawRate 在 Toyota 恆為 0）。
#   v5  過彎門檻下修為 1.96/1.50 m/s²（之後再依需求調為現值 1.76/1.30）。
#   v6  方向燈覆寫，後續版本已整段移除，方向燈目前完全不影響 AEM。
#   v7  新增 near_stop_active（只影響節流門檻）。
#   v9  新增 base_throttle_threshold（依車速動態切換 blended 基礎節流門檻）。
#   v10 接近停止線時 mode 也強制 blended（加車速閘門 60/70 km/h）。
#   v11 （本版）
#       - [bug fix] NEAR_STOP_ENTER_M/EXIT_M 由 5/15 m 改為 40/50 m。5 m 加上 0.5 s 防彈跳
#         與 stop_dist_m 晚一幀，觸發時車已幾乎到線，且此時車速通常已 <= 20 km/h、本來就是
#         實驗模式，導致 near-stop 的 mode 覆寫與節流覆寫實際上都不會產生作用。
#       - 修正所有與常數值不一致的註解／docstring。
#       - 新增 AEM_EXPERIMENTAL_OFF_FORCES_ACC，並把「忽略實驗模式開關」明確列為刻意分歧；
#         預設 False，行為與 v10 相同。

from openpilot.common.realtime import DT_MDL

# 實驗模式開關處理（見檔頭「刻意分歧 2」）
#   False（預設）：AEM 啟用時忽略開關，mode 完全由 AEM 決定（與 v10 行為相同）
#   True         ：開關關閉時一律回傳 'acc'，AEM 只在開關打開時運作
AEM_EXPERIMENTAL_OFF_FORCES_ACC = False

# 車速門檻
SPEED_FORCE_EXPERIMENTAL_KPH = 20.0   # <= 這個值：無條件、立即強制實驗模式，不經防彈跳
SPEED_TO_EXPERIMENTAL = 20.0 / 3.6    # 遲滯開關下緣。<= 20 km/h 已被上面的強制層接管，
                                       # 此分支理論上不會被用到，僅作防禦保留
SPEED_TO_NORMAL       = 30.0 / 3.6    # >= 30 km/h：切換為一般模式（含防彈跳）
# 20~30 km/h 為過渡帶，維持前一狀態

# 過彎判斷門檻（m/s²），含遲滯
CURVE_LAT_ACCEL_ENTER = 1.76   # 濾波後 a_y > 此值 -> 進入過彎保護（強制 acc）
CURVE_LAT_ACCEL_EXIT  = 1.30   # 濾波後 a_y <= 此值 -> 解除過彎保護

# 側向加速度低通濾波係數（風格與 dtsc.py / ocm.py 的 LPF_ALPHA 一致）
LAT_ACCEL_LPF_ALPHA = 0.2

# 防彈跳與最短維持時間
CONFIRM_TIME_S   = 0.5   # 任一切換條件需連續成立這麼久（秒）才生效
MIN_DWELL_TIME_S = 2.0   # 車速模式切換後至少維持這麼久（秒）才允許下一次切換

# 接近模型停止線門檻（m），含遲滯
# 距離來源為 traffic_stop.py 的 stop_dist_m（已經過 median + moving-average 平滑與物理
# 偏移修正），不是原始 model_msg.position.x。stop_dist_m 為 None 明確視為「不接近」。
NEAR_STOP_ENTER_M = 40.0   # 距離 <= 40 m -> 進入「接近停止線」狀態
NEAR_STOP_EXIT_M  = 50.0   # 距離 > 50 m -> 解除（10 m 遲滯緩衝；50 m 對應 traffic_stop.py
                            # TRAFFIC_STOP_DISTANCE_FADE_BP_M 的上限值）

# 接近停止線時，是否連 mode 也一併強制 blended 的車速閘門（km/h）
STOP_MODE_SPEED_ENTER_KPH = 60.0   # <= 60 km/h -> 允許「接近停止線」強制切 mode
STOP_MODE_SPEED_EXIT_KPH  = 70.0   # >= 70 km/h -> 不允許；60~70 km/h 維持前一狀態

# 基礎節流門檻依車速動態切換（只影響 blended 模式的 allow_throttle，不影響 get_mode()）
# 10~20 km/h 為過渡帶，維持前一狀態。四個常數彼此獨立，可個別調整。
# ⚠️ 門檻 <= 0.0 的語意是「此車速區間 blended 模式永不進入 coast（allow_throttle 恆為
#    True）」，因為 throttle_prob 不可能小於 0；這是刻意設定，不是 bug。
#    門檻 0.1 時，搭配 planner 的遲滯，解除門檻為 0.025，實務上也極少進入 coast。
#    若希望 blended 在一般行駛時仍會適度收油，需要調高這兩個值（例如 0.2 / 0.1）。
BASE_THROTTLE_LOW_SPEED_KPH    = 10.0   # 車速 <= 10 km/h -> 低速狀態
BASE_THROTTLE_HIGH_SPEED_KPH   = 20.0   # 車速 >= 20 km/h -> 高速狀態
BASE_THROTTLE_LOW_SPEED_VALUE  = 0.1    # 低速狀態的節流門檻（越低加速意願越積極）
BASE_THROTTLE_HIGH_SPEED_VALUE = 0.0    # 高速狀態的節流門檻（0.0 = 不 coast，見上方說明）


class AEM:
  def __init__(self):
    # 車速邏輯狀態
    self._speed_mode = 'normal'            # 'experimental' / 'normal'
    self._speed_pending = self._speed_mode
    self._speed_confirm_t = 0.0
    self._speed_dwell_t = MIN_DWELL_TIME_S  # 開機後第一次車速判斷可立即生效

    # 過彎覆寫狀態
    self._curve_active = False
    self._curve_pending = False
    self._curve_confirm_t = 0.0
    self._lat_accel_filtered = 0.0

    # 接近停止線狀態
    self._near_stop_active = False
    self._near_stop_pending = False
    self._near_stop_confirm_t = 0.0

    # 接近停止線的車速閘門狀態
    self._stop_mode_speed_ok = True
    self._stop_mode_speed_pending = self._stop_mode_speed_ok
    self._stop_mode_speed_confirm_t = 0.0

    # 基礎節流門檻的車速狀態
    self._base_throttle_state = 'low'   # 'low' / 'high'
    self._base_throttle_pending = self._base_throttle_state
    self._base_throttle_confirm_t = 0.0

  @property
  def near_stop_active(self):
    """是否處於「接近模型停止線」狀態（距離 <= NEAR_STOP_ENTER_M，> NEAR_STOP_EXIT_M 解除）。
    供 longitudinal_planner.py 決定是否改用 ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP。
    本屬性本身不影響 get_mode()；get_mode() 是否因此切成 blended，還要另外通過
    STOP_MODE_SPEED_ENTER_KPH/EXIT_KPH 車速閘門。"""
    return self._near_stop_active

  @property
  def base_throttle_threshold(self):
    """依車速決定的 blended 基礎節流門檻：低速狀態（<= BASE_THROTTLE_LOW_SPEED_KPH）回傳
    BASE_THROTTLE_LOW_SPEED_VALUE，高速狀態（>= BASE_THROTTLE_HIGH_SPEED_KPH）回傳
    BASE_THROTTLE_HIGH_SPEED_VALUE，中間維持前一狀態。
    有接近停止線覆寫時，呼叫端應取兩者中較保守（較高）的值。"""
    return (BASE_THROTTLE_LOW_SPEED_VALUE if self._base_throttle_state == 'low'
            else BASE_THROTTLE_HIGH_SPEED_VALUE)

  def update_states(self, model_msg, radar_msg, v_ego, stop_dist_m=None):
    yaw_rate = model_msg.orientationRate.z[0] if len(model_msg.orientationRate.z) else 0.0
    self._speed_dwell_t += DT_MDL
    self._update_speed_mode(v_ego)
    self._update_curve_override(v_ego, yaw_rate)
    self._update_near_stop(stop_dist_m)
    self._update_stop_mode_speed_gate(v_ego)
    self._update_base_throttle(v_ego)

  def _update_speed_mode(self, v_ego):
    if v_ego * 3.6 <= SPEED_FORCE_EXPERIMENTAL_KPH:
      # 無條件強制層：狀態、pending、計時全部同步，之後進入 20~30 遲滯區間時是乾淨起點
      self._speed_mode = 'experimental'
      self._speed_pending = 'experimental'
      self._speed_confirm_t = 0.0
      self._speed_dwell_t = MIN_DWELL_TIME_S
      return

    if v_ego <= SPEED_TO_EXPERIMENTAL:
      candidate = 'experimental'
    elif v_ego >= SPEED_TO_NORMAL:
      candidate = 'normal'
    else:
      candidate = self._speed_mode   # 20~30 km/h 過渡帶：維持前一狀態

    if candidate == self._speed_pending:
      self._speed_confirm_t += DT_MDL
    else:
      self._speed_pending = candidate
      self._speed_confirm_t = 0.0

    if (self._speed_pending != self._speed_mode
        and self._speed_confirm_t >= CONFIRM_TIME_S
        and self._speed_dwell_t >= MIN_DWELL_TIME_S):
      self._speed_mode = self._speed_pending
      self._speed_dwell_t = 0.0

  def _update_curve_override(self, v_ego, yaw_rate):
    raw_lat_accel = abs(v_ego * yaw_rate)
    self._lat_accel_filtered = (LAT_ACCEL_LPF_ALPHA * raw_lat_accel
                                 + (1.0 - LAT_ACCEL_LPF_ALPHA) * self._lat_accel_filtered)

    # 進入用 CURVE_LAT_ACCEL_ENTER，解除用 CURVE_LAT_ACCEL_EXIT，避免卡在臨界值附近反覆切換
    threshold = CURVE_LAT_ACCEL_EXIT if self._curve_active else CURVE_LAT_ACCEL_ENTER
    candidate = self._lat_accel_filtered > threshold

    if candidate == self._curve_pending:
      self._curve_confirm_t += DT_MDL
    else:
      self._curve_pending = candidate
      self._curve_confirm_t = 0.0

    if self._curve_pending != self._curve_active and self._curve_confirm_t >= CONFIRM_TIME_S:
      self._curve_active = self._curve_pending

  def _update_near_stop(self, stop_dist_m):
    # None = traffic_stop 目前沒有主動停等（功能關閉或沒偵測到紅燈／停止標誌），
    # 必須視為「不接近」，不能當成距離 0
    if stop_dist_m is None:
      candidate = False
    else:
      threshold = NEAR_STOP_EXIT_M if self._near_stop_active else NEAR_STOP_ENTER_M
      candidate = stop_dist_m <= threshold

    if candidate == self._near_stop_pending:
      self._near_stop_confirm_t += DT_MDL
    else:
      self._near_stop_pending = candidate
      self._near_stop_confirm_t = 0.0

    # 不套用 MIN_DWELL_TIME_S，沒有理由延遲生效或延遲解除
    if self._near_stop_pending != self._near_stop_active and self._near_stop_confirm_t >= CONFIRM_TIME_S:
      self._near_stop_active = self._near_stop_pending

  def _update_stop_mode_speed_gate(self, v_ego):
    v_kph = v_ego * 3.6
    if v_kph <= STOP_MODE_SPEED_ENTER_KPH:
      candidate = True
    elif v_kph >= STOP_MODE_SPEED_EXIT_KPH:
      candidate = False
    else:
      candidate = self._stop_mode_speed_ok   # 60~70 km/h 過渡帶：維持前一狀態

    if candidate == self._stop_mode_speed_pending:
      self._stop_mode_speed_confirm_t += DT_MDL
    else:
      self._stop_mode_speed_pending = candidate
      self._stop_mode_speed_confirm_t = 0.0

    if (self._stop_mode_speed_pending != self._stop_mode_speed_ok
        and self._stop_mode_speed_confirm_t >= CONFIRM_TIME_S):
      self._stop_mode_speed_ok = self._stop_mode_speed_pending

  def _update_base_throttle(self, v_ego):
    v_kph = v_ego * 3.6
    if v_kph <= BASE_THROTTLE_LOW_SPEED_KPH:
      candidate = 'low'
    elif v_kph >= BASE_THROTTLE_HIGH_SPEED_KPH:
      candidate = 'high'
    else:
      candidate = self._base_throttle_state   # 10~20 km/h 過渡帶：維持前一狀態

    if candidate == self._base_throttle_pending:
      self._base_throttle_confirm_t += DT_MDL
    else:
      self._base_throttle_pending = candidate
      self._base_throttle_confirm_t = 0.0

    # 不套用 MIN_DWELL_TIME_S：只影響節流保守程度，沒有理由延遲
    if self._base_throttle_pending != self._base_throttle_state and self._base_throttle_confirm_t >= CONFIRM_TIME_S:
      self._base_throttle_state = self._base_throttle_pending

  def get_mode(self, mode):
    # mode 為 selfdriveState.experimentalMode 對應的預設值（'blended' / 'acc'）。
    # 預設忽略（刻意分歧 2）；AEM_EXPERIMENTAL_OFF_FORCES_ACC = True 時，開關關閉一律 acc。
    if AEM_EXPERIMENTAL_OFF_FORCES_ACC and mode == 'acc':
      return 'acc'
    if self._curve_active:
      return 'acc'
    if self._near_stop_active and self._stop_mode_speed_ok:
      return 'blended'
    return 'blended' if self._speed_mode == 'experimental' else 'acc'
