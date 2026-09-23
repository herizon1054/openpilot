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
#      a_y = |v_ego * yaw_rate|，yaw_rate 取自 modelV2.orientationRate.z[0]（視覺模型預測
#      的橫擺角速度，rad/s，與 dtsc.py 同一來源）。
#      ⚠️ 這裡刻意不用 carState.yawRate：openpilot 各品牌 carstate.py 只有 Ford、PSA
#      會實際賦值，包含 Toyota 在內的其餘品牌從未設定該欄位，數值恆為 0.0（capnp 預設值）。
#      本 fork 主力車款 Corolla TSS2（Toyota）即屬未賦值品牌，若採用 carState.yawRate，
#      過彎保護在這台車上會完全失效且沒有任何錯誤訊息——已用實際路測 rlog 驗證
#      carState.yawRate 全程恆為 0.0，而 modelV2.orientationRate.z[0] 全程有非零值，
#      因此改用模型訊號，與 dtsc.py 的資料來源一致，可跨品牌使用。
#   2. 車速雙門檻 + 遲滯區間：
#        v_ego <= SPEED_TO_EXPERIMENTAL (20 km/h) -> 切換為實驗模式 (blended)
#        v_ego >= SPEED_TO_NORMAL       (30 km/h) -> 切換為一般模式 (acc)
#      20~30 km/h 之間視為「過渡帶」，維持前一狀態、不切換，避免在單一門檻附近來回抖動。
#   3. 接近模型停止線 + 車速閘門：near_stop_active 為 True 且車速通過
#      STOP_MODE_SPEED_ENTER_KPH/EXIT_KPH 這組閘門（<=60km/h 開放、>=70km/h 關閉，
#      60~70km/h 過渡帶維持前一狀態）時，強制切為實驗模式 (blended)。車速閘門的用意是
#      避免在快速道路等高速情境遠遠看到停止線就被拉進實驗模式，只在市區低速接近停止線
#      時才讓 mode 跟著切換。優先權低於過彎保護、高於車速雙門檻。
#   4. 防彈跳 (debounce)：任何切換條件都必須連續成立 CONFIRM_TIME_S 秒才會真正生效。
#      車速模式的切換另外要求距離上一次切換至少 MIN_DWELL_TIME_S 秒（過彎的強制/解除
#      不受此最短間隔限制，確保安全保護不會被延遲觸發）。
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
#   本模組把門檻訂在 DECEL_BP 的中段，在彎道已經需要中等力道煞車時才切手，
#   而非跟 DTSC 一起在 ~1.0 m/s² 就介入：
#     CURVE_LAT_ACCEL_ENTER = 1.76 m/s²（依需求調整，低於原本 1.96 的中等彎道估算值）
#     CURVE_LAT_ACCEL_EXIT  = 1.30 m/s²（依需求調整，低於進入門檻，形成遲滯緩衝）
#   1.0~1.5 m/s² 這段的輕微彎道，維持在實驗模式，由 blended/e2e 自行處理過彎減速。
#   ⚠️ 1.96/1.50 m/s² 為方向性建議值，非針對特定車款的實測結論，建議依路測 log
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

# v4 變更紀錄（相對於 v3 的重大修正，非單純刻意分歧，屬於 bug fix）：
#   v1~v3 皆用 carState.yawRate 計算側向加速度，經由實際路測 rlog（TOYOTA_COROLLA_TSS2）
#   驗證發現：Toyota（以及除 Ford/PSA 外的絕大多數品牌）carstate.py 從未對 ret.yawRate
#   賦值，該欄位在這些車上恆為 0.0，導致過彎保護完全失效、且無任何錯誤或警告。
#   v4 改用 modelV2.orientationRate.z[0]（與 dtsc.py 相同的資料來源），已用同一份
#   rlog 驗證全程有非零值。update_states 介面因此變回只需要 model_msg / radar_msg /
#   v_ego 三個參數，不再需要外部傳入 yaw_rate 或 steering_angle_deg。
#
# v5 變更紀錄（相對於 v4 的門檻調整）：
#   v3/v4 把進入門檻上修到 2.6 m/s²（≈0.27G，對應 DTSC 減速度 -1.6，接近舒適上限），
#   只在彎道逼近 DTSC 舒適上限時才切手。v5 依需求下修回 0.2G（1.96 m/s²），對應 DTSC
#   減速度約 -1.0~-1.2，屬於中等彎道即切手，比 v3/v4 更早把控制權交給一般模式 + DTSC。
#   解除門檻同步下修為 1.50 m/s²（≈0.15G），維持約 0.46 m/s² 的遲滯緩衝。
#
# v6 曾新增方向燈覆寫（打方向燈強制切為實驗模式）與對應的節流門檻覆寫，依需求已
# 於後續版本整段移除（含 blinker_on 參數、blinker_active 屬性），方向燈目前完全不影響
# AEM 的判斷。
#
# v7 變更紀錄（相對於 v6 的功能新增）：
#   新增「接近模型停止線」狀態：距離 <= 50m（含遲滯，60m 解除）時，near_stop_active
#   屬性回傳 True。⚠️ 這一項刻意只影響呼叫端的節流門檻選擇，不寫進 get_mode()、
#   不影響 blended/acc 判斷——接不接近停止線，跟該不該用 e2e 縱向控制是兩個問題，
#   AEM 原本的職責是選 mode，這裡只是額外暴露一個狀態供節流門檻參考，避免職責混在一起。
#   50m 沿用同一 fork 的 traffic_stop.py 既有常數 TRAFFIC_STOP_DISTANCE_FADE_BP_M 的
#   上限值（該模組本來就把「接近停止線」的物理意義定在 50m 內），不是另外憑空訂的數字。
#   距離來源必須是 traffic_stop.py 算出來的 stop_dist_m（已經過 median+moving-average
#   平滑與物理偏移修正），不是原始 model_msg.position.x，否則會繼承模型單幀跳動的雜訊。
#   stop_dist_m 為 None（目前沒有主動停等）明確視為「不接近」，不可誤判為距離 0。
#
# v9 變更紀錄（相對於 v8 的功能新增）：
#   新增「基礎節流門檻依車速動態切換」：車速 <= 60km/h 用較保守的 0.2，車速 >= 70km/h
#   用較積極的 0.1，60~70km/h 為過渡帶維持前一狀態（門檻數字後續依需求調整為 50/60km/h，
#   見常數區與 v9 之後的變更）。透過 base_throttle_threshold 屬性
#   暴露給呼叫端，取代原本寫死在 longitudinal_planner.py 裡的固定值。跟 near_stop_active
#   一樣只影響節流門檻，不寫進 get_mode()。四個常數（車速門檻×2、節流值×2）彼此獨立，
#   個別調整互不影響；longitudinal_planner.py 的 ALLOW_THROTTLE_THRESHOLD_E2E（AEM 停用
#   時的後備值）也是完全獨立的常數，跟這裡的門檻值互不牽動。

# v10 變更紀錄（相對於 v9 的功能新增）：
#   新增「接近停止線時 mode 也強制切為實驗模式」：near_stop_active 為 True 且車速通過
#   STOP_MODE_SPEED_ENTER_KPH(60)/EXIT_KPH(70) 這組獨立的車速閘門時，get_mode() 強制
#   回傳 'blended'。車速閘門是為了避免高速情境（例如快速道路遠遠看到停止線）也被拉進
#   實驗模式，只在市區低速（<=60km/h，直到 >=70km/h 才解除）接近停止線時才讓 mode
#   跟著切換。優先權排在過彎保護之後、車速雙門檻之前。near_stop_active 本身（節流門檻
#   覆寫）不受影響，車速閘門只決定要不要「額外」讓 mode 也跟著切，兩者是獨立判斷。

# 車速門檻（km/h 換算為 m/s），20~30 km/h 為遲滯 / 過渡帶
SPEED_TO_EXPERIMENTAL = 20.0 / 3.6   # 車速 <= 20 km/h -> 切換為實驗模式 (blended)
SPEED_TO_NORMAL       = 30.0 / 3.6   # 車速 >= 30 km/h -> 切換為一般模式 (acc)

# 過彎判斷門檻：側向加速度 a_y = |v_ego * yaw_rate|（m/s²，yaw_rate 取自 modelV2），含遲滯避免臨界值抖動
# 只在「大彎道」才切手，輕微彎道交給實驗模式自行處理（詳見上方 DECEL_BP/DECEL_V 對照說明）
CURVE_LAT_ACCEL_ENTER = 1.76   # 進入過彎保護
CURVE_LAT_ACCEL_EXIT  = 1.30   # 解除過彎保護

# 側向加速度的低通濾波係數，濾除單幀雜訊尖峰（風格與 dtsc.py / ocm.py 的 LPF_ALPHA 一致）
LAT_ACCEL_LPF_ALPHA = 0.2

# 防彈跳與最短維持時間
CONFIRM_TIME_S   = 0.5   # 任一切換條件需連續成立這麼久（秒）才生效，濾除瞬間雜訊
MIN_DWELL_TIME_S = 2.0   # 車速模式切換後至少維持這麼久（秒）才允許下一次切換

# 距離模型停止線的節流保守化門檻（m），含遲滯避免臨界值抖動
NEAR_STOP_ENTER_M = 5.0   # 距離 <= 40m 進入「接近停止線」狀態（沿用 traffic_stop.py 自己的
                           # TRAFFIC_STOP_DISTANCE_FADE_BP_M 上限值，非另外憑空訂的數字）
NEAR_STOP_EXIT_M  = 15.0   # 距離 > 50m 才解除，形成 10m 遲滯緩衝，避免在 50m 附近來回抖動

# 接近停止線時，是否連 mode 也一併強制切為實驗模式，額外用車速做一次限制：
# 車速 <= 60km/h 才允許這個覆寫生效，車速 >= 70km/h 就算正接近停止線也不觸發，
# 60~70km/h 為過渡帶維持前一狀態。避免在高速（例如快速道路匝道遠遠看到停止線）時
# 也被拉進實驗模式；只在市區低速接近停止線的情境下才讓 mode 跟著切換。
STOP_MODE_SPEED_ENTER_KPH = 60.0   # 車速 <= 60km/h -> 允許「接近停止線」強制切 mode
STOP_MODE_SPEED_EXIT_KPH  = 70.0   # 車速 >= 70km/h -> 不允許，就算接近停止線也不切 mode

# 基礎節流門檻依車速動態切換（km/h），供呼叫端在沒有接近停止線覆寫時使用。
# 50~60 km/h 為過渡帶，維持前一狀態不切換，緩衝寬度比照車速模式門檻的遲滯設計，
# 避免車速在邊界附近小幅波動時頻繁切換。
# ⚠️ 以下四個常數彼此獨立，個別調整互不影響：車速門檻（KPH）決定「什麼時候切換」，
# 節流值（VALUE）決定「切換後用多保守/多積極的門檻」，兩兩之間可以任意分開調整。
BASE_THROTTLE_LOW_SPEED_KPH    = 10.0   # 車速 <= 這個值 -> 判定為低速，套用 BASE_THROTTLE_LOW_SPEED_VALUE
BASE_THROTTLE_HIGH_SPEED_KPH   = 20.0   # 車速 >= 這個值 -> 判定為高速，套用 BASE_THROTTLE_HIGH_SPEED_VALUE
BASE_THROTTLE_LOW_SPEED_VALUE  = 0.1    # 低速（<=50km/h）時使用的節流門檻，越低代表加速意願越積極
BASE_THROTTLE_HIGH_SPEED_VALUE = 0.0    # 高速（>=60km/h）時使用的節流門檻，越低代表加速意願越積極


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

    # 接近停止線狀態（節流門檻覆寫用，見 near_stop_active）
    self._near_stop_active = False
    self._near_stop_pending = False
    self._near_stop_confirm_t = 0.0

    # 接近停止線的車速閘門狀態（get_mode() 是否要跟著切換用，見 STOP_MODE_SPEED_*）
    self._stop_mode_speed_ok = True
    self._stop_mode_speed_pending = self._stop_mode_speed_ok
    self._stop_mode_speed_confirm_t = 0.0

    # 基礎節流門檻的車速狀態（只影響節流門檻，不影響 get_mode()）
    self._base_throttle_state = 'low'   # 'low' -> 0.2, 'high' -> 0.1
    self._base_throttle_pending = self._base_throttle_state
    self._base_throttle_confirm_t = 0.0

  @property
  def near_stop_active(self):
    """目前是否處於「接近模型停止線」狀態（距離 <= 50m，含遲滯），供呼叫端讀取，
    用來決定是否改用較保守的節流門檻（longitudinal_planner.py 的
    ALLOW_THROTTLE_THRESHOLD_E2E_NEAR_STOP）。這個屬性本身不影響 get_mode()；
    get_mode() 是否因為接近停止線而切成實驗模式，還要另外通過車速閘門
    （STOP_MODE_SPEED_ENTER_KPH/EXIT_KPH），見 get_mode() 內部判斷。"""
    return self._near_stop_active

  @property
  def base_throttle_threshold(self):
    """依車速動態決定的基礎節流門檻（車速 <= 50km/h 為 0.2，>= 60km/h 為 0.1，中間維持
    前一狀態）。供呼叫端在沒有接近停止線覆寫時使用；有覆寫時呼叫端應取兩者中
    較保守（較高）的值，而不是直接覆蓋掉這個車速判斷。"""
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
    if v_ego <= SPEED_TO_EXPERIMENTAL:
      candidate = 'experimental'
    elif v_ego >= SPEED_TO_NORMAL:
      candidate = 'normal'
    else:
      candidate = self._speed_mode   # 50~60 km/h 過渡帶：維持前一狀態，不切換

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

  def _update_near_stop(self, stop_dist_m):
    # stop_dist_m 為 None 代表 traffic_stop 目前沒有主動停等中（功能關閉、或沒偵測到
    # 紅燈/停止標誌），必須明確視為「不接近」，不能當成距離 0 處理，否則會誤判成
    # 永遠接近停止線
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

    # 同樣不套用 MIN_DWELL_TIME_S：這只影響節流保守程度，沒有理由延遲生效或延遲解除
    if self._near_stop_pending != self._near_stop_active and self._near_stop_confirm_t >= CONFIRM_TIME_S:
      self._near_stop_active = self._near_stop_pending

  def _update_stop_mode_speed_gate(self, v_ego):
    v_kph = v_ego * 3.6
    if v_kph <= STOP_MODE_SPEED_ENTER_KPH:
      candidate = True
    elif v_kph >= STOP_MODE_SPEED_EXIT_KPH:
      candidate = False
    else:
      candidate = self._stop_mode_speed_ok   # 60~70 km/h 過渡帶：維持前一狀態，不切換

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
      candidate = self._base_throttle_state   # 50~60 km/h 過渡帶：維持前一狀態，不切換

    if candidate == self._base_throttle_pending:
      self._base_throttle_confirm_t += DT_MDL
    else:
      self._base_throttle_pending = candidate
      self._base_throttle_confirm_t = 0.0

    # 不套用 MIN_DWELL_TIME_S：這只影響節流保守程度（不像 mode 切換那樣需要避免頻繁
    # 改變控制策略），沒有理由延遲生效或延遲解除
    if self._base_throttle_pending != self._base_throttle_state and self._base_throttle_confirm_t >= CONFIRM_TIME_S:
      self._base_throttle_state = self._base_throttle_pending

  def get_mode(self, mode):
    if self._curve_active:
      return 'acc'
    if self._near_stop_active and self._stop_mode_speed_ok:
      return 'blended'
    return 'blended' if self._speed_mode == 'experimental' else 'acc'
