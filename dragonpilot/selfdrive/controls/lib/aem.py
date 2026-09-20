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
# 依「車速」與「側向加速度（過彎程度）」動態切換 experimentalMode 對應的縱向控制模式：
#   * 'blended' = 實驗模式 (e2e)
#   * 'acc'     = 一般模式
#
# 判斷優先權（由高到低）：
#   1. 過彎減速保護：側向加速度 |a_y| 超過 CURVE_LAT_ACCEL_ENTER 時強制切為一般模式 (acc)，
#      避免在彎道中使用行為較不可預期的 e2e 縱向控制。
#      a_y = |v_ego * yawRate|（carState.yawRate 為車輛最佳估計橫擺角速度，rad/s），
#      比方向盤角度更能反映實際過彎的側向 G，且已內含車速的影響（同角度在不同車速
#      下的側向 G 差異很大，純角度門檻會太武斷）。
#   2. 車速雙門檻 + 遲滯區間：
#        v_ego <= SPEED_TO_EXPERIMENTAL (80 km/h) -> 切換為實驗模式 (blended)
#        v_ego >= SPEED_TO_NORMAL       (90 km/h) -> 切換為一般模式 (acc)
#      80~90 km/h 之間視為「過渡帶」，維持前一狀態、不切換，避免在單一門檻附近來回抖動。
#   3. 防彈跳 (debounce)：任何切換條件都必須連續成立 CONFIRM_TIME_S 秒才會真正生效。
#      車速模式的切換另外要求距離上一次切換至少 MIN_DWELL_TIME_S 秒（過彎的強制/解除
#      不受此最短間隔限制，確保過彎安全保護不會被延遲觸發）。
#      這是本次要求的「過渡」機制：避免感測雜訊或臨界值附近的抖動造成縱向目標
#      （加速度）忽然跳動，導致突然減速或加速。
#
# 側向加速度門檻參考來源（同一 fork 內的 dtsc.py，已依實測調校，非憑空訂值）：
#   SCCV_ABORT_PRED_LAT_ACC_TH = 0.7 m/s²                     -> DTSC 認定連彎道都算不上的雜訊下限
#   DECEL_BP = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.3, 2.6, 3.0]  -> DTSC 側向加速度 -> 減速度的對照表
#   DECEL_V  = [0.0, -0.1, -0.3, -0.5, -0.8, -1.2, -1.4, -1.6, -2.0]（MAX_COMFORT_DECEL = -2.0）
#   換言之 DTSC 本身從 ~1.0 m/s²（≈0.1G）就開始輕微收油，是漸進式介入，不是一個單一開關；
#   1.0~2.3 m/s² 這段只需要 -0.1 ~ -1.4 m/s² 的輕度到中度減速，交給實驗模式(e2e)自行處理
#   多半已經足夠；直到 2.6 m/s² 附近，DTSC 才需要 -1.6 m/s²，逼近舒適上限 -2.0，
#   這才是真正「大彎道」的量級。
#
#   依使用者需求「輕微彎道交給實驗模式過彎，大彎道才交給一般模式 + DTSC」，
#   本模組把門檻訂在 DECEL_BP 的中後段，只在彎道嚴重到 DTSC 需要中重度煞車時才切手，
#   而非跟 DTSC 一起在 ~1.0 m/s² 就介入：
#     CURVE_LAT_ACCEL_ENTER = 2.6 m/s²（≈0.27G，對應 DTSC 減速度 -1.6，接近舒適上限）
#     CURVE_LAT_ACCEL_EXIT  = 2.0 m/s²（≈0.20G，對應 DTSC 減速度 -1.2，仍在中段，形成遲滯緩衝）
#   1.0~2.0 m/s² 這段的輕微彎道，維持在實驗模式，由 blended/e2e 自行處理過彎減速。
#   ⚠️ 2.6/2.0 m/s² 為方向性建議值，非針對特定車款的實測結論，建議依路測 log
#   （尤其是切手當下 blended 是否來得及減速、DTSC 介入時模式是否已經正確切到 acc）微調。
#
# 與原廠設計的差異（intentional divergence，非 upstream 行為，依使用者要求記錄於此）：
#   原始 AEM 完全依 modelV2.meta.disengagePredictions.gasPressProbs 的機率門檻 (0.4/0.6)
#   來決定 blended/acc；本版本改以「車速 + 側向加速度」為主要判斷依據，
#   原本的 gasPressProb 機率門檻邏輯已整段移除，update_states 因此不再使用
#   model_msg / radar_msg 的內容（僅保留參數以維持呼叫端介面相容）。
#
# v2 變更紀錄（相對於 v1 的刻意分歧）：
#   v1 使用方向盤角度 (steeringAngleDeg > 60°) 判斷過彎，未考慮車速造成的側向 G 差異，
#   且容易被市區路口轉彎、迴轉等大角度但低側向 G 的情境誤觸發。v2 改用
#   a_y = |v_ego * yawRate| 取代方向盤角度，同一套遲滯 + 防彈跳機制沿用不變。
#
# v3 變更紀錄（相對於 v2 的刻意分歧）：
#   v2 的 2.0/1.5 m/s² 門檻其實比 DTSC 自己開始介入的 ~1.0 m/s² 高出不少，但仍會讓
#   DTSC 剛開始輕度收油（1.0~2.0 m/s²）的「輕微彎道」就被 AEM 切去一般模式。
#   v3 依需求把整組門檻上移到 DECEL_BP 中後段（2.0/2.6 m/s²），讓輕微彎道留在實驗
#   模式自行處理，只有側向 G 逼近 DTSC 舒適上限的大彎道才交給一般模式 + DTSC。

from openpilot.common.realtime import DT_MDL

# 車速門檻（km/h 換算為 m/s），80~90 km/h 為遲滯 / 過渡帶
SPEED_TO_EXPERIMENTAL = 80.0 / 3.6   # 車速 <= 80 km/h -> 切換為實驗模式 (blended)
SPEED_TO_NORMAL       = 90.0 / 3.6   # 車速 >= 90 km/h -> 切換為一般模式 (acc)

# 過彎判斷門檻：側向加速度 a_y = |v_ego * yawRate|（m/s²），含遲滯避免臨界值抖動
# 只在「大彎道」才切手，輕微彎道交給實驗模式自行處理（詳見上方 DECEL_BP/DECEL_V 對照說明）
CURVE_LAT_ACCEL_ENTER = 2.6   # 進入過彎保護（約 0.27G，對應 DTSC 減速度 -1.6，接近舒適上限）
CURVE_LAT_ACCEL_EXIT  = 2.0   # 解除過彎保護（約 0.20G，對應 DTSC 減速度 -1.2，低於進入門檻避免抖動）

# 側向加速度的低通濾波係數，濾除單幀雜訊尖峰（風格與 dtsc.py / ocm.py 的 LPF_ALPHA 一致）
LAT_ACCEL_LPF_ALPHA = 0.2

# 防彈跳與最短維持時間
CONFIRM_TIME_S   = 0.5   # 任一切換條件需連續成立這麼久（秒）才生效，濾除瞬間雜訊
MIN_DWELL_TIME_S = 2.0   # 車速模式切換後至少維持這麼久（秒）才允許下一次切換


class AEM:
  def __init__(self):
    # 車速邏輯狀態
    self._speed_mode = 'normal'            # 目前生效的車速模式：'experimental' / 'normal'
    self._speed_pending = self._speed_mode
    self._speed_confirm_t = 0.0
    self._speed_dwell_t = MIN_DWELL_TIME_S  # 讓開機後第一次車速判斷可以立即生效

    # 過彎覆寫狀態
    self._curve_active = False
    self._curve_pending = False
    self._curve_confirm_t = 0.0
    self._lat_accel_filtered = 0.0

  def update_states(self, model_msg, radar_msg, v_ego, yaw_rate=0.0):
    self._speed_dwell_t += DT_MDL
    self._update_speed_mode(v_ego)
    self._update_curve_override(v_ego, yaw_rate)

  def _update_speed_mode(self, v_ego):
    if v_ego <= SPEED_TO_EXPERIMENTAL:
      candidate = 'experimental'
    elif v_ego >= SPEED_TO_NORMAL:
      candidate = 'normal'
    else:
      candidate = self._speed_mode   # 80~90 km/h 過渡帶：維持前一狀態，不切換

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

    # 進入用 2.0 m/s²，解除用 1.5 m/s²，避免側向加速度剛好卡在臨界值附近反覆切換
    threshold = CURVE_LAT_ACCEL_EXIT if self._curve_active else CURVE_LAT_ACCEL_ENTER
    candidate = self._lat_accel_filtered > threshold

    if candidate == self._curve_pending:
      self._curve_confirm_t += DT_MDL
    else:
      self._curve_pending = candidate
      self._curve_confirm_t = 0.0

    if self._curve_pending != self._curve_active and self._curve_confirm_t >= CONFIRM_TIME_S:
      self._curve_active = self._curve_pending

  def get_mode(self, mode):
    if self._curve_active:
      return 'acc'
    return 'blended' if self._speed_mode == 'experimental' else 'acc'
