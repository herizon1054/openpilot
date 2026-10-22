"""
PathDeviationMonitor - 自適應路徑偏離監控器 (V8.6.2 Heading Error Edition)
Copyright (c) 2024-2026 DragonPilot Contributors.

====================================================================================
[系統架構與完整功能總覽 - 教科書級說明]
====================================================================================
本模組的核心目標：「全時監控實車動態，確保物理軌跡貼合神經網路的期望路徑。
若發生物理偏離（如轉向不足、打滑，或緊急閃避未打燈），系統將主動介入給予平滑且
安全的減速，將車身拉回安全軌跡。」

為達到市區安靜、高速安全、彎道全時守護的量產級要求，本模組具備以下 7 大核心機制：

1. 【全時物理監控 (Full-Time Safeguard)】
   - 捨棄傳統的「方向盤扭力介入判斷」。因為在高速彎道中，駕駛常會不自覺地出力對抗，
     若此時暫停防護，容易在彎心發生危險。
   - 僅認可「方向燈」為合法跨線意圖。只要未打燈，系統便全時監控軌跡落差。

2. 【一階航向角對決 (Heading Error vs Neural) - V8.6.2 核心進化】
   - 痛點解決：傳統基於「二階位移 (y = 0.5*k*x^2)」的計算方式，在面對真實道路的
     Clothoid (歐拉螺旋線) 緩和曲線入彎時，會因為嚴重的數學滯後性而產生「幽靈轉向不足」。
   - 革命性升級：徹底放棄橫向位移 (y) 對決，改採時間零滯後的「航向角誤差 (Heading Error)」。
     直接比對「模型預期車頭指向」與「底盤真實車頭指向」。

3. 【極短物理擂台 (Micro-Lookahead)】
   - 配合航向角對決特性，徹底拔除舊有的長視距機制。將物理比對擂台強制鎖死在
     0.4 秒 (最大 8 公尺) 的極短距離內。
   - 確保系統抓取的是「當下最真實的瞬態角度」，徹底消除因視距拉長產生的幾何發散。

4. 【等比例空間分割法 (Proportional Compression) - V8.5 核心升級】
   - 解決痛點：傳統固定查表法在高速公路上，因為「視距拉長導致的幾何誤差放大」，
     容易頻繁觸發神經質點煞與斷崖式頓挫。
   - 運作原理：依據車速動態推延「死區（不介入範圍）」，並將剩下的活動空間按照
     4:3:2 的黃金比例「等比例壓縮」。
   - 達成效益：徹底移除 0.95 等硬編碼門檻，不論死區在哪裡，系統永遠使用完美一致
     的斜率進行「平滑砍半 -> 降至 1/4 -> 0.00 極限煞停」，達成真正的無縫線性介入。

5. 【整體感知信心度 (Confidence Averaging)】
   - 雙線時取左右信心度平均值，單線時取單側。標線模糊時等比例降低減速懲罰，
     防止系統在惡劣路況下產生「幽靈煞車」。

6. 【入彎寬容期與動態死區 (Grace Window & Dynamic Deadzone)】
   - 系統能自動偵測「曲率變化率 (Yaw Jerk)」。在入彎初期 (曲率陡增)，給予高達 0.25m
     的遲滯寬容度，完美吸收 EPS 轉向馬達死區與車輪側滑角 (Slip Angle) 建立的時間差。
   - 依車速線性調整容忍下限，高速容忍度大、市區容忍度小。

7. 【極限舒適減速器 (Rate Limiter - Decel Only)】
   - 所有緊急煞車請求都會被鉗制在最大 0.25G 的線性遞增範圍內。
   - 這不僅保護乘客舒適度，更模擬了資深駕駛的「帶煞入彎 (Trail Braking)」技巧。
   - 加速恢復徹底交還給下層 Openpilot 原生縱向 MPC 處理，避免雙重控制疊床架屋。
====================================================================================
"""

import numpy as np
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase


class PathDeviationMonitor(TargetsBase):
  """路徑偏離監控器核心邏輯 - V8.6.2 航向角對決版"""

  # ==========================================
  # 基礎常數與系統邊界設定
  # ==========================================
  MIN_ACTIVE_SPEED = 1.0  # 系統最低作動車速 (m/s)，約等於 3.6 km/h，防止靜止時誤觸發
  MIN_LANE_PROB = 0.3  # 車道線可信度最低門檻 (30%)，低於此值視為無標線
  EMA_ALPHA = 0.20  # 指數移動平均濾波係數，Alpha=0.2 提供約 0.3s 的時間平滑窗口
  MAX_DECEL_RATE = 2.5 * DT_MDL  # 降速斜率限制：2.5 m/s^2 (約 0.25G)，DT_MDL 為單幀時間

  def __init__(self, CP, mpc):
    super().__init__(CP, mpc)

    # 物理濾波：用於平滑 IMU / 方向盤推算出的真實車輛曲率
    self.curvature_lp = 0.0
    self.prev_curvature_lp = 0.0  # 用於偵測曲率變化率 (Yaw Jerk)，建立入彎寬容期

    # 決策濾波：最終輸出的偏離比例 (Ratio)，經過 EMA 平滑後才作為查表依據
    self.ratio_ema = 0.0

    # 系統狀態控制
    self.cooldown_timer = 0  # 方向燈取消後的冷卻計時器
    self.last_action_state = False  # 記錄上一幀是否處於介入煞車狀態
    self.last_v_target = V_CRUISE_MAX  # 記錄上一幀的目標車速 (用於平滑減速計算)

    # ==========================================
    # 等比例空間分割法 - 查值表定義
    # ==========================================
    # 這裡定義的是 Y 軸 (目標車速的保留比例)。
    # 配合 X 軸 (偏離率) 的 4:3:2 空間分割，實現完美的線性煞車。
    # 1.00 = 不減速
    # 0.50 = 減速至一半 (對應 4/9 空間結束)
    # 0.25 = 減速至四分之一 (對應 7/9 空間結束)
    # 0.00 = 要求極限煞停 (對應 9/9 空間結束)
    self.SPEED_FACTOR_V = [1.00, 0.50, 0.25, 0.00]

    self.debug_log = True
    self.log_counter = 0

  def _get_speed_factor(self, ratio_ema, v_ego):
    """
    【核心演算法：動態死區與等比例空間分割法 (Proportional Compression)】

    設計理念：
    為了解決高速行駛時「預測距離平方效應」所放大的微小幾何誤差，我們必須採用動態死區。
    但如果只是簡單地將死區推延（例如從 10% 延後到 35%），而繼續使用舊有的固定查表法，
    會導致剩下的介入空間被嚴重擠壓 (Squeeze)。
    例如：剩下 65% 的空間卻要完成 100% 的煞車力道分配，會造成高速時煞車力道異常突兀。

    解法：我們採用「4:3:2 等比例分割」，將扣除死區後「剩餘的活動空間」進行動態切分，
    確保不論車速多少、死區在哪裡，各速域的煞車斜率都能保持一致的線性體感。
    """

    # ---------------------------------------------------------
    # 1. 計算動態死區 (Dynamic Deadzone)
    # ---------------------------------------------------------
    # 市區 (10 m/s = 36 km/h) 容忍 10% 的偏移，此寬容度用於應對駕駛微調方向盤或閃避機車。
    # 高速 (30 m/s = 108 km/h) 容忍 35% 的偏移，此寬容度用於抵消視距拉長後產生的物理放大誤差。
    dynamic_deadzone = float(np.interp(v_ego, [10.0, 15.0, 30.0], [0.10, 0.15, 0.20]))

    # 若目前的偏離率未跨越死區，完全不介入 (維持 100% 車速)，並直接返回
    if ratio_ema <= dynamic_deadzone:
      return 1.0, dynamic_deadzone

    # ---------------------------------------------------------
    # 2. 啟動「等比例空間分割法 (Proportional Compression)」
    # ---------------------------------------------------------
    # 步驟 A：算出剩下的「實際活動空間」(Active Range)
    # 舉例：若高速死區為 35% (0.35)，則剩下的介入空間為 65% (0.65)。
    active_range = 1.0 - dynamic_deadzone

    # 步驟 B：將剩餘空間分成 9 份 (依據黃金比例 4 + 3 + 2 = 9 份)
    # 第一階段 (砍半區)：佔剩餘空間的 4/9
    bp1 = dynamic_deadzone + active_range * (4.0 / 9.0)

    # 第二階段 (降至1/4區)：佔剩餘空間的 7/9 (累積了 4/9 + 3/9)
    bp2 = dynamic_deadzone + active_range * (7.0 / 9.0)

    # 第三階段 (極限煞停區)：佔滿剩餘空間 (直接推至 1.00)
    # 動態產生完美平移的查表曲線 (X 軸斷點)
    current_bp = [dynamic_deadzone, bp1, bp2, 1.00]

    # 步驟 C：透過線性插值 (np.interp) 求出對應的車速縮放比例
    speed_factor = float(np.interp(ratio_ema, current_bp, self.SPEED_FACTOR_V))

    return speed_factor, dynamic_deadzone

  def update_target(self, sm, v_ego, a_ego, v_cruise):
    car_state = sm['carState']
    model_v2 = sm['modelV2']

    # 基準狀態初始化
    desired_v_target = v_cruise  # 預設目標車速為巡航設定值
    is_valid = True  # 標記當前防護條件是否成立
    is_deviating = False  # 標記是否觸發物理偏離減速
    log_reason = "正常巡航"  # 用於終端機輸出的狀態說明
    log_y_actual = 0.0
    path_y = 0.0

    # 為 Debug 變數設定預設值
    compare_dist = 0.0
    heading_error = 0.0
    actual_deviation = 0.0
    is_entering_curve = False

    # ===========================================================
    # 步驟 1: 物理曲率計算 (含 Sensor Fallback 感測器備援機制)
    # ===========================================================
    if sm.valid['carState']:
      raw_yaw = car_state.yawRate

      # 【備援機制】檢查 YawRate 是否失效
      # 若 YawRate 極小且方向盤有明顯轉動，判定為硬體無支援，改用阿克曼幾何推算
      if abs(raw_yaw) < 1e-6 and abs(car_state.steeringAngleDeg) > 0.1:
        # 計算實際車輪轉角 = 方向盤轉角 / 轉向比
        actual_wheel_angle = np.radians(car_state.steeringAngleDeg) / max(self.CP.steerRatio, 1.0)
        # 逆推橫擺率 = (車速 * 車輪轉角) / 軸距
        raw_yaw = (v_ego * actual_wheel_angle) / max(self.CP.wheelbase, 1.0)

      # 【物理 LPF】計算當下實體軌跡的曲率 (Curvature = YawRate / Velocity)
      # 進行低通濾波 (0.2 / 0.8) 以抹平因為路面顛簸或方向盤微小抖動造成的雜訊
      self.curvature_lp = 0.2 * (raw_yaw / max(v_ego, 1.0)) + 0.8 * self.curvature_lp

    # ---------------------------------------------------------
    # 車道線信心度融合 (Lane Confidence Fusion)
    # ---------------------------------------------------------
    if sm.valid['modelV2'] and len(model_v2.laneLineProbs) >= 4:
      prob_l = model_v2.laneLineProbs[1]
      prob_r = model_v2.laneLineProbs[2]
    else:
      prob_l = 0.0
      prob_r = 0.0

    # 判定單側車道線是否達到可用門檻
    has_left = prob_l >= self.MIN_LANE_PROB
    has_right = prob_r >= self.MIN_LANE_PROB

    # 【信心度融合】雙線取平均防幻覺；單線保留 100% 信任維持防護力道
    if has_left and has_right:
      # 兩側標線皆清晰：取平均值，可有效防止單側標線因為反光或接縫造成的瞬間跳動 (幻覺)
      lane_conf = (prob_l + prob_r) * 0.5
    elif has_left:
      # 僅左側清晰：完全信任左側，確保在右側標線消失的路段依然保有完整的防護力道
      lane_conf = prob_l
    elif has_right:
      # 僅右側清晰：完全信任右側
      lane_conf = prob_r
    else:
      # 雙線皆低於門檻或完全消失：直接歸零，後續將觸發防護力道衰減機制
      lane_conf = 0.0

    # ---------------------------------------------------------
    # 系統狀態屏蔽邏輯 (防誤判守門員)
    # ---------------------------------------------------------
    if car_state.leftBlinker or car_state.rightBlinker:
      # 駕駛打方向燈，視為合法變換車道意圖，立即解除防護
      is_valid, log_reason = False, "方向燈介入中"
      # 給予 2 秒的冷卻時間，並重置軌跡記憶
      self.cooldown_timer, self.ratio_ema = int(2.0 / DT_MDL), 0.0
    elif self.cooldown_timer > 0:
      is_valid, log_reason = False, f"冷卻中 ({self.cooldown_timer} 幀)"
      self.cooldown_timer -= 1
      self.ratio_ema = 0.0
    elif v_ego < self.MIN_ACTIVE_SPEED:
      # 車速過低不作動，防止起步或停車時的無效計算
      is_valid, log_reason = False, "車速過低"
      self.ratio_ema = 0.0
    elif lane_conf < self.MIN_LANE_PROB:
      # 模型已經完全看不到線，不具備參考價值
      is_valid, log_reason = False, "雙線皆失"
      # 將累積的偏離率緩慢衰減，避免瞬間解除防護造成的頓挫
      self.ratio_ema *= 0.8

    # ===========================================================
    # 步驟 2: 物理擂台與真實航向角偏離計算 (V8.6.2 Heading Error Edition)
    # ===========================================================
    if is_valid:
      model_x = np.array(model_v2.position.x)

      # ---------------------------------------------------------
      # 【V8.6.2 升級 ①：物理擂台極限短視距】
      # (已徹底捨棄舊版基於位移的長視距，因為航向角對決不需要累積空間距離)
      # 配合 Clothoid 緩和曲線特性，將對決視距壓制在 0.4 秒，並設定 8m 絕對上限。
      # 在極短距離內，能徹底消除道路曲率非線性遞增造成的數學發散。
      # ---------------------------------------------------------
      physical_time_margin = 0.4
      compare_dist = max(5.0, min(v_ego * physical_time_margin, 8.0))

      # 防呆：確保比較距離不會超過模型實際輸出的最遠點
      if len(model_x) > 0:
        compare_dist = min(compare_dist, float(model_x[-1]))

      path_y = float(np.interp(compare_dist, model_x, model_v2.position.y))

      # ---------------------------------------------------------
      # 【V8.6.2 升級 ②：真實物理航向角對決 (Heading Error vs Neural)】
      # 解決傳統二階積分 (y = 0.5 * k * x^2) 遇到多項式 (y ≈ x^3) 必定判定的偏離誤判。
      # 降維至一階航向角 (Heading) 進行對決，時間零滯後。
      # ---------------------------------------------------------
      # 1. 估算模型預期航向 (Model Heading)：利用 compare_dist 前後 1m 的 Y 座標差分求出導數 dy/dx
      y_front = float(np.interp(compare_dist + 1.0, model_x, model_v2.position.y))
      y_back = float(np.interp(compare_dist - 1.0, model_x, model_v2.position.y))
      model_heading = (y_front - y_back) / 2.0  # 小角度近似 tan(θ) ≈ θ

      # 2. 計算車輛物理預期航向 (Vehicle Heading)：Yaw = 曲率 * 距離
      vehicle_heading = self.curvature_lp * compare_dist

      # 3. 取航向角絕對誤差，並將角度投影回橫向位移 (m)，以無縫相容舊版的空間分割邏輯
      heading_error = abs(model_heading - vehicle_heading)
      actual_deviation = compare_dist * heading_error

      # 為了維持 Log 輸出相容，保留舊有公式產生一組 log_y_actual 顯示用
      log_y_actual = 0.5 * self.curvature_lp * (compare_dist**2)

      # ---------------------------------------------------------
      # 偏離率 (Ratio) 正規化計算
      # ---------------------------------------------------------
      # 改用 compare_dist 取出左右車道線的 Y 軸位置
      left_line_y = float(np.interp(compare_dist, model_x, model_v2.laneLines[1].y))
      right_line_y = float(np.interp(compare_dist, model_x, model_v2.laneLines[2].y))

      # 取較近的一條線作為「可用安全空間 (Safety Margin)」。
      safety_margin = max(0.3, min(abs(left_line_y - path_y), abs(right_line_y - path_y)))

      # ---------------------------------------------------------
      # 【V8.6.2 升級 ③：入彎寬容期 (Grace Window) 與遲滯容錯】
      # ---------------------------------------------------------
      # 偵測是否正處於入彎階段 (曲率絕對值正在明顯增加)
      is_entering_curve = abs(self.curvature_lp) > abs(self.prev_curvature_lp) + 0.0002
      self.prev_curvature_lp = self.curvature_lp

      # 針對 EPS 轉向馬達死區、車輪側滑角 (Slip Angle) 的建立延遲，
      # 若處於入彎初期，給予高達 25 公分 (0.25m) 的動態寬容度；平穩期維持 15 公分。
      base_tolerance = 0.25 if is_entering_curve else 0.15
      effective_deviation = max(0.0, actual_deviation - base_tolerance)

      # 計算偏離懲罰：改用扣除硬體死區後的 (有效偏離量 / 安全空間) * 信心度
      scaled_ratio = min(1.0, effective_deviation / safety_margin) * lane_conf
      self.ratio_ema = (1.0 - self.EMA_ALPHA) * self.ratio_ema + self.EMA_ALPHA * scaled_ratio

      # =========================================================
      # 步驟 3: 動態 EMA 死區與目標車速縮放
      # =========================================================
      speed_factor, dynamic_deadzone = self._get_speed_factor(self.ratio_ema, v_ego)

      if self.ratio_ema > dynamic_deadzone and speed_factor < 1.0:
        desired_v_target = v_cruise * speed_factor
        is_deviating = True
        log_reason = f"偏離減速 (EMA:{self.ratio_ema * 100:.0f}%, 死區:{dynamic_deadzone * 100:.0f}%)"
      else:
        log_reason = "處於死區寬容" if self.ratio_ema <= dynamic_deadzone else "縮放因子未觸發"

    # ===========================================================
    # 步驟 4: 極限舒適減速器 (Rate Limiter - 僅限減速)
    # ===========================================================
    base_v = self.last_v_target if self.last_action_state else v_cruise
    min_allowed_v = base_v - self.MAX_DECEL_RATE
    final_v = float(max(desired_v_target, min_allowed_v))

    # 動態閉環安全防護：若駕駛在系統介入時手動調降 v_cruise，
    # 確保輸出的目標車速不會大於設定值，防止出現非預期的加速暴衝現象。
    final_v = min(final_v, v_cruise)

    # ===========================================================
    # 步驟 5: 狀態更新與輸出
    # ===========================================================
    self.action = is_deviating
    self.v_target = final_v
    self.a_target = a_ego
    self.last_v_target = self.v_target
    self.last_action_state = self.action

    if self.debug_log:
      self._print_log(log_reason, log_y_actual, path_y, compare_dist, heading_error, actual_deviation, is_entering_curve)
    return super().update_target(sm, v_ego, a_ego, v_cruise)

  def _print_log(self, reason, y_actual, path_y, compare_dist, heading_error, actual_deviation, is_entering_curve):
    """
    智慧型 Log 輸出機制 (V8.6.2 航向角除錯增強版)
    防洗版設計：介入時每幀輸出，巡航時每 60 幀 (約 1 秒) 輸出一次心跳封包。
    """
    self.log_counter += 1
    if self.action or self.log_counter >= 60:
      state_str = "🛑 [介入]" if self.action else "✅ [巡航]"
      curve_state = "入彎" if is_entering_curve else "平穩"

      # [新增] 加入 距(compare_dist)、角差(heading_error)、偏(actual_deviation) 與 態(入彎寬容期狀態)
      log_msg = (
        f"[PDM V8.6.2] {state_str} {reason} | 目標車速:{self.v_target * 3.6:.1f}km/h | "
        f"對決視距:{compare_dist:.1f}m | 航向角誤差:{heading_error:.4f}rad | 實際偏離量:{actual_deviation:.2f}m | "
        f"物理Y:{y_actual:.2f} | 模型Y:{path_y:.2f} | 彎道狀態:{curve_state} | EMA:{self.ratio_ema * 100:.0f}%"
      )
      print(log_msg)
      cloudlog.debug(log_msg)
      self.log_counter = 0
