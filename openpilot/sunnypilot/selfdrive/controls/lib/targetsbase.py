from abc import ABC, abstractmethod

from openpilot.cereal import messaging, custom
from openpilot.common.params import Params

from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX

# 匯入 cloudlog 用於系統日誌記錄
from openpilot.common.swaglog import cloudlog


# 定義縱向控制計畫的來源 (對應 DP 的自定義結構)
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class TargetsBase(ABC):
  """
  目標控制基底類別 (TargetsBase)

  這是一個抽象基底類別 (Abstract Base Class)，用於定義 openpilot 縱向控制
  中各種目標控制器 (例如：DTSC, PDM 等) 的標準介面與基礎屬性。
  所有的目標子類別都應該繼承此類別，並根據需求覆寫 (override) 其中的方法。
  """

  def __init__(self, CP: structs.CarParams, mpc):
    """
    初始化基底類別的通用狀態與變數。

    參數:
      CP (structs.CarParams): 車輛參數，包含車輛的硬體設定與支援功能資訊。
      mpc: 模型預測控制器 (Model Predictive Controller) 實例。
    """
    self.classname = self.__class__.__name__
    self.params = Params()  # 用於讀取 openpilot 的系統參數或使用者設定 (如 UI toggles)
    self.frame = 0  # 迴圈計數器，用於控制特定邏輯 (如讀取參數) 的執行頻率

    self.CP = CP  # 車輛參數，提供子類別使用
    self.mpc = mpc  # 模型預測控制器，提供子類別使用

    # 檢查當前車輛是否支援 openpilot 的縱向控制
    self.available = CP.openpilotLongitudinalControl

    # --- 狀態控制變數 ---
    self.enable = False  # 總開關：此功能是否被使用者啟用 (例如：從 Params 讀取設定)
    self.action = False  # 介入狀態：此功能當下是否達到作動條件並正在介入 (Active)
    self.braking = False  # 煞車狀態：此功能當下是否要求車輛進行減速

    # --- 歷史狀態追蹤變數 (用於偵測狀態變更並觸發 Log) ---
    # 在初始化時，將歷史狀態設為與初始狀態相同，避免一啟動就誤觸發變更紀錄
    self._prev_available = self.available
    self._prev_enable = self.enable
    self._prev_action = self.action

    # --- 內部計算目標值存放區 ---
    self.v_target = V_CRUISE_MAX  # 子類別計算出來的內部目標車速 (m/s)
    self.a_target = 0.0  # 子類別計算出來的內部 MPC 曲率規劃起點 (m/s^2)

    # --- 最終輸出目標值存放區 ---
    self.output_v_target = V_CRUISE_MAX  # 準備交給規劃器 (Planner) 的最終目標車速
    self.output_a_target = 0.0  # 準備交給規劃器 (Planner) 的 MPC 曲率規劃起點

  def update(self, sm: messaging.SubMaster) -> None:
    """
    基礎更新迴圈。
    負責管控特定方法 (如參數更新) 的執行頻率。

    參數:
      sm (messaging.SubMaster): 包含來自各個 socket 的最新數據狀態。
    """

    # 只有當車輛硬體支援 (CP.openpilotLongitudinalControl)、巡航系統目前處於啟用狀態時，才視為可用 (available)
    cruise_enabled = sm['carState'].cruiseState.enabled
    self.available = self.CP.openpilotLongitudinalControl and cruise_enabled

    self.frame += 1
    # 依據模型預測的週期 (DT_MDL) 來限制參數更新的頻率，以節省系統效能。
    # 若 DT_MDL 為 0.05 (20Hz)，則相當於每 1 秒更新一次 Params，避免頻繁讀寫硬碟。
    if self.frame % int(1.0 / DT_MDL) == 0:
      self.update_params()

  def update_params(self):
    self.enable = self.params.get_bool(self.classname)
    """
    更新目標參數。

    [抽象方法]
    子類別必須實作此方法，通常用於從 self.params 讀取開關狀態，並更新 self.enable。
    """

  @abstractmethod
  def update_target(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float):
    """
    更新並決定最終的目標輸出數值，同時監控運作狀態的變更以輸出日誌。

    [抽象方法]
    此處提供了預設的輸出仲裁邏輯：
    1. 如果功能啟用 (enable) 且正在作動 (action)，則輸出內部計算的 v_target 與 a_target。
    2. 否則，輸出不限制狀態 (最高巡航車速 V_CRUISE_MAX 與當前加速度 a_ego 作為規劃起點)。

    注意：由於標記為 @abstractmethod，子類別必須覆寫此方法。若子類別想直接套用
    這段預設的判斷與 Log 邏輯，需要在其覆寫的方法最後呼叫 `super().update_target(...)`。

    參數:
      sm (messaging.SubMaster): 系統狀態數據。
      v_ego (float): 當前自車速度 (m/s)。
      a_ego (float): 當前自車加速度 (m/s^2)。
      v_cruise (float): 當前設定的巡航定速 (m/s)。

    回傳:
      tuple[float, float]: (最終輸出目標車速 output_v_target, 最終輸出之 MPC 曲率規劃起點 output_a_target)
    """
    # ==========================================
    # 輸出目標仲裁邏輯
    # ==========================================
    # 只有在功能支援 (available)、已開啟 (enable)，且確定要介入 (action) 時，才輸出子類別的計算目標
    if self.available and self.enable and self.action:
      self.output_v_target = max(0.0, min(self.v_target, V_CRUISE_MAX))
      self.output_a_target = self.a_target
    else:
      # 否則 (未開啟、不支援、或未達到介入條件)，一律交還控制權
      # 速度給予最大寬容值，加速度則帶入 a_ego 作為下游 MPC 的曲率規劃起點
      self.output_v_target = V_CRUISE_MAX
      self.output_a_target = a_ego
    # 自動判斷是否在煞車
    self.braking = self.action and self.output_a_target < 0.0

    # ==========================================
    # 狀態變更偵測與 Log 輸出邏輯
    # ==========================================
    # 比對當前狀態與上一幀的歷史狀態，只要有任何一個發生改變，就觸發記錄
    if self.available != self._prev_available or self.enable != self._prev_enable or self.action != self._prev_action:
      # 動態取得當前正在執行的子類別名稱 (例如: "DTSC" 或 "PathDeviationMonitor")
      # 這使得所有繼承 TargetsBase 的子類別都能自動擁有正確名稱的 Log，無須重複撰寫
      child_class_name = self.__class__.__name__

      # 將布林值狀態格式化為易讀的字串
      state_str = f"available:{self.available}, enable:{self.enable}, action:{self.action}, braking:{self.braking}"

      # 組合最終的 Log 訊息。速度與 MPC 起點保留兩位小數點，讓排版更整齊。
      # 格式: [繼承者的物件名稱] [狀態] [self.output_v_target] [self.output_a_target]
      log_msg = f"[{child_class_name}] [{state_str}] [{self.output_v_target:.2f}] [{self.output_a_target:.2f}]"

      # 印出到終端機畫面，方便開發時即時查看
      print(log_msg)

      # 使用 cloudlog.info 記錄到系統日誌中
      # 提示：若要在終端機即時看到 info 層級日誌，請確保啟動前執行 `export LOGPRINT=info` (或 debug)
      cloudlog.info(log_msg)

      # 狀態記錄完畢後，更新歷史狀態變數，等待下一次的狀態反轉
      self._prev_available = self.available
      self._prev_enable = self.enable
      self._prev_action = self.action

    # 回傳最終決定的目標速度與 MPC 曲率規劃起點給上層 Planner
    return self.output_v_target, self.output_a_target

  def write_to_msg(self, target_msg) -> None:
    """
    將內部的狀態優雅地寫入 Cap'n Proto 的 Target 結構中。
    此方法供上層的 Planner 呼叫。
    """
    target_msg.available = bool(self.available)
    target_msg.enable = bool(self.enable)
    target_msg.action = bool(self.action)
    target_msg.braking = bool(self.braking)
    target_msg.vTarget = float(self.v_target)
    target_msg.aTarget = float(self.a_target)
    target_msg.outputVtarget = float(self.output_v_target)
    target_msg.outputAtarget = float(self.output_a_target)
