import numpy as np
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.targetsbase import TargetsBase

class LeadDepartureSmoother(TargetsBase):
  """
  LeadDepartureSmoother - 前車駛離與跟車確定性幾何拋物線軌跡生成器 (V1.7.1 動態起步優化版)
  Copyright (c) 2026 DragonPilot Contributors.

  ====================================================================================
  [控制學終極思想：單點速率限制與底盤 MPC 職責解耦 - 教科書級說明]
  ====================================================================================
  本模組貫徹「極簡控制哲學」，回歸縱向控制的核心本質：本模組的神聖職責是強行控管與修飾「傳遞
  給 MPC Planner 的巡航目標車速上限」，確保該車速指令在任何極端場景下都是「穩定上升與下降的」。

  本模組徹底砍掉所有繁瑣、越權且疊床架屋的緊急煞車判斷與安全不感帶分叉，實現完美的職責解耦：
  - LDS 模組：100% 專注於利用「等加速度幾何運動學拋物線」規劃最舒適、光滑、穩定不跳動的速度極限。
  - 底盤 MPC：100% 全權負責實車的安全防線、車距保持與應對突發急煞。安全交給 MPC，快適交給 LDS。

  1. 【空間換速度 ── 徹底消除加速再減速的拉風琴效應】
     - 當前方有車時，依據剩餘物理車距 (dRel) 主動向下光滑收斂。在接近前車的最後 2 秒鐘，
       目標速指令線會自發性拉平，無超調 (0% Overshoot) 地黏在前車速度上，免除點煞震盪。

  2. 【物理幾何牆 ── 100% 屏蔽前方手排車低速換檔高頻晃動】
     - 核心計算完全不使用相對速度 vRel，唯獨跟隨前車物理距離 (dRel)。
     - 手排車換檔踩離合器失速的 0.3 秒內，兩車實際物理距離幾乎光滑不變，故輸出穩如泰山。

  3. 【自適應動態加速牆 ── 完美解決低速起步緩慢、保留高速快適】
     - 升級優化方案二。引入隨自車速 (v_ego) 動態查表插值的加速速率限制器。
     - 低速起步時放寬增益至 1.2 m/s² 確保輕快不滯後；高速時收緊至 0.5 m/s² 維持細緻平穩。
     - 搭配 COMFORT_DECEL_DOWN 減速牆，一體化管控穩定上升與下降，徹底消滅任何斷崖速度突波。
  ====================================================================================
  """

  # ==============================================================================
  # 全域調整參數宣告區 (類別常數) —— 實車測試微調看這裡！
  # ==============================================================================
  SCA_DECEL = 0.38          # 恆定舒適幾何加速度/減速度 (m/s²)。數值越小，接近前車的滑行過程越長越溫柔平滑
  SCA_SAFE_DIST = 11.5      # 幾何對齊安全車距邊界 (meter)。當在此車距時，自車目標速與前車速完美完成光滑重合
  V_TARGET_FLOOR = 1.39     # 巡航目標速度保底下限 (m/s)，等於 5.0 km/h。防止指令沉入 0 死區導致起步反應滯後
  COMFORT_DECEL_DOWN = 1.0  # 巡航速度穩定下降斜率牆 (m/s²)。慢車突然切入插隊時，限制單幀指令暴跌，實現優雅收油滑行

  # ------------------------------------------------------------------------------
  # 方案二：自適應動態起步加速查表參數 (分段對齊時速 0km/h, 20km/h, 60km/h)
  # ------------------------------------------------------------------------------
  V_ACCEL_BP = [0.0, 10, 16]  # 自車時速中斷點廊道 (單位: m/s，對應 0, 36, 55 km/h)
  V_ACCEL_V = [1.2, 0.8, 0.5]      # 對應允許的動態舒適加速斜率牆 (單位: m/s²)。低速大推力，高速極絲滑

  def __init__(self, CP, mpc):
    super().__init__(CP, mpc)

    # 智慧型日誌控制
    self.debug_log = False
    self.log_counter = 0

  def update_target(self, sm, v_ego, a_ego, v_cruise):
    """
    核心確定性幾何與單點動態速率限制控制迴圈
    """
    # 提取雷達狀態與第一順位主前車 (leadOne)
    radar_state = sm['radarState']
    lead_one = radar_state.leadOne

    # 【動態全時熱啟動防護】
    # 若當下系統處於未介入狀態，內部的目標車速暫存器必須全時與實車當前最真實的速度 (v_ego) 保持 100% 同步。
    # 這能確保當前方前車突然駛離、系統觸發介入的瞬間，軌跡生成器能從當下最精準的底盤物理起點優雅出發。
    if not self.action:
      self.v_target = v_ego

    # ===========================================================
    # 步驟 1: 幾何與定速目標基本判定 ── 全面重構去冗餘 (單一分支流)
    # ===========================================================
    if lead_one.present:
      # 有車時：底盤對齊速度 (v_floor) 錨定在前車真實速度與定速保底線的最大值
      v_floor = max(self.V_TARGET_FLOOR, float(lead_one.vLead))

      # 計算剩餘物理車距前導量。max(0.0) 確保即便因極端貼車跌入安全距離內，根號內部依然大於等於 0 絕對不噴 Bug
      distance_headroom = max(0.0, float(lead_one.dRel) - self.SCA_SAFE_DIST)

      # 萬流歸宗：標準運動學幾何拋物線方程。純幾何演進，低速晃動在數學結構上被徹底抹平
      raw_v_target = np.sqrt(v_floor**2 + 2 * self.SCA_DECEL * distance_headroom)
    else:
      # 前方完全淨空：目標速度直接指向駕駛人設定的巡航車速
      raw_v_target = v_cruise

    # 最大安全防禦牆：無論幾何拋物線數值如何噴發，計算出的原始目標速度絕對不得超越駕駛設定的巡航上限
    raw_v_target = min(raw_v_target, v_cruise)

    # ===========================================================
    # 步驟 2: 判定是否啟動控制權介入 (Action Decision) ── 狀態機維持不變
    # ===========================================================
    if self.action:
      self.action = (raw_v_target < v_cruise) or (self.v_target < v_cruise - 0.05)
    else:
      self.action = (raw_v_target < v_cruise - 0.05)

    # ===========================================================
    # 步驟 3: 萬流歸宗 ── 雙向速率限幅牆，只管控「穩定上升與下降」
    # ===========================================================
    if self.action:
      # 計算當前幀目標速度與上一幀指令的速差餘額
      v_diff = raw_v_target - self.v_target

      if v_diff > 0:
        # 情境一：速度需要增長 (起步出發、前車加速拉開、前車駛離失去目標緩加速)
        # 實作優化方案二：透過 np.interp 根據當前自車速度動態解算出最優的舒適加速斜率牆
        comfort_accel_up = float(np.interp(v_ego, self.V_ACCEL_BP, self.V_ACCEL_V))

        # 限制每幀最大允許的增速步進量，低速起步輕快飽滿，中高速線性收斂，確保車速溫柔且穩定地上升
        self.v_target += min(v_diff, comfort_accel_up * DT_MDL)
      elif v_diff < 0:
        # 情境二：速度需要縮減 (高速接近慢車、突發慢車強行插隊切入)
        # 限制每幀最大允許的減速步進量，不允許 any 斷崖式的跳變，強制雕刻出優雅的收油滑行軌跡
        self.v_target += max(v_diff, -self.COMFORT_DECEL_DOWN * DT_MDL)

      # 貫徹最高指令與職責解耦：我們完全不控制、不修改加速度。全時回傳當前實車加速度 a_ego 歸還給 MPC 全權解算安全車距
      self.a_target = a_ego

      # 雙重防線：限制在合理的實車安全物理區間內
      self.v_target = max(0.0, min(self.v_target, v_cruise))
    else:
      # 終極優化：當前方完全淨空且已圓滿回歸定速，未介入狀態下讓目標速度上限與駕駛設定的 v_cruise 完美貼合
      # 徹底消除高頻開關介入時因為 v_target 重置為 V_CRUISE_MAX 所產生的單幀幽靈加速度突波與拉扯頓挫感
      self.v_target = v_cruise
      self.a_target = a_ego

    # ===========================================================
    # 步驟 4: 狀態更新與 Log 記錄
    # ===========================================================
    if self.debug_log:
      self._print_log(lead_one.status, raw_v_target, lead_one.vLead, lead_one.dRel, v_ego)

    return super().update_target(sm, v_ego, a_ego, v_cruise)

  def _print_log(self, lead_status, raw_v_target, v_lead, d_rel, v_ego):
    """
    優雅的除錯日誌輸出機制
    """
    self.log_counter += 1
    if self.action or self.log_counter >= 60:
      state_str = "🛑 [幾何控制中]" if self.action else "✅ [穩態巡航/跟車]"
      lead_str = f"有車 (前車速:{v_lead * CV.MS_TO_KPH :.1f}km/h, 車距:{d_rel:.1f}m)" if lead_status else "無車 (前方淨空)"

      # 動態即時查表以供 Log 記錄除錯
      curr_accel_wall = float(np.interp(v_ego, self.V_ACCEL_BP, self.V_ACCEL_V)) if self.action else 0.0

      log_msg = (
        f"[LDS V1.7.1 動態優化版] {state_str} 前方狀態:{lead_str} | "
        f"幾何原始速:{raw_v_target * CV.MS_TO_KPH:.1f}km/h | 最終輸出速:{self.v_target * CV.MS_TO_KPH:.1f}km/h | "
        f"當前加速牆上限:{curr_accel_wall:.2f}m/s² | 實車當前加速度:{self.a_target:.3f}m/s²"
      )
      print(log_msg)
      cloudlog.debug(log_msg)
      self.log_counter = 0
