"""
dp: 移植自開發者 Candy0707 對 sunnypilot 的實際修改（已上車驗證）：
    https://github.com/Candy0707/openpilot/commit/547751461052ea2bf39bc438785d6717bc7a6e31
    (sunnypilot/selfdrive/controls/lib/latcontrol_dynamic.py)

僅供 opendbc/car/toyota CAR.TOYOTA_COROLLA_TSS2 使用：
同時運算 Torque 與 Angle(LTA) 兩顆控制器（熱備援），依車速與方向盤角度/角速度遲滯切換兩者的主控權，
避免在打彎過程中觸發切換造成不連續。切換僅在「安全直行狀態」下才會發生。

與原始 commit 的差異（因 dpagel controlsd/LatControl 架構不同而調整，非功能性差異，
已跟原始邏輯逐行核對一致）：
  - dpagel 的 LatControl 系列建構子簽章為 (CP, CI, dt)，沒有 CP_SP（sunnypilot 多了 CarParamsSP）。
  - dpagel 的 LatControl.update() 沒有 calibrated_pose 參數。

已知已依您指示改回、與最初 sunnypilot 原始碼（.rar/zip 快照）一致的地方：
  - 切回 Torque 時，以及 reset() 時，會額外呼叫 self.torque_ctrl.pid.reset() 清空 PID 積分項。
    這點與開發者 GitHub commit（547751461052ea2bf39bc438785d6717bc7a6e31，未清積分）不同，
    是您明確要求比照最初提供的 sunnypilot 原始碼所做的選擇。

已知額外補上、與原始 commit 不同的一處（請見 update_live_torque_params）：
  - 原始 commit 未定義 update_live_torque_params()，而 controlsd.py 在 lateralTuning 為
    torque 調校時會無條件呼叫 self.LaC.update_live_torque_params(...)；LatControlDynamic 與其
    基底類別 LatControl 皆無此方法，一旦 liveTorqueParameters.useParams 轉為 True 會導致
    AttributeError 使 controlsd 崩潰。此處補上轉發給 torque_ctrl，避免此崩潰風險。
    如需與原始 commit 完全一致（不補這個方法），請告知。
"""

from cereal import car
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque


class LatControlDynamic(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    # 同時初始化兩個控制器
    self.angle_ctrl = LatControlAngle(CP, CI, dt)
    self.torque_ctrl = LatControlTorque(CP, CI, dt)

    # 預設使用 CP (CarParams) 讀出來的設定值
    self.use_angle = (CP.steerControlType == car.CarParams.SteerControlType.angle)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # dp: 補上轉發，避免 controlsd 呼叫時 AttributeError（原始 commit 沒有這個方法，見檔頭說明）
    self.torque_ctrl.update_live_torque_params(latAccelFactor, latAccelOffset, friction)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    # 定義「安全直行狀態」：方向盤打角小於 10 度，且轉動速率小於 5 度/秒
    is_safe_to_switch = abs(CS.steeringAngleDeg) < 10.0 and abs(CS.steeringRateDeg) < 5.0

    # 1. 判斷主控權與遲滯區間，並且鎖死過彎時的切換
    if CS.vEgo > 8.0 and not self.use_angle and is_safe_to_switch:
      self.use_angle = True
      self.angle_ctrl.reset()  # 確保角度控制器狀態乾淨  22ms

    elif CS.vEgo < 4.0 and self.use_angle and is_safe_to_switch:
      self.use_angle = False
      self.torque_ctrl.reset()  # 確保扭矩控制器狀態乾淨 16ms
      if hasattr(self.torque_ctrl, 'pid'):
        self.torque_ctrl.pid.reset()  # 徹底清除積分

    # 2. Angle 控制器永遠運算 (幾何計算，無風險)
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature,
                                               curvature_limited, lat_delay)

    # 3. Torque 控制器永遠運算 (熱備援)
    # 關鍵防護：如果當前是 Angle 主控，強制觸發 steer_limited_by_safety 來凍結 Torque 的 PID 積分
    torque_is_frozen = True if self.use_angle else steer_limited_by_safety
    t_steer, _, t_log = self.torque_ctrl.update(active, CS, VM, params, torque_is_frozen, desired_curvature,
                                                curvature_limited, lat_delay)

    # 4. 雙輸出合併：回傳 (扭矩輸出, 角度輸出, 當前主控的Log)
    if self.use_angle:
      return t_steer, a_steer, a_log
    else:
      return t_steer, a_steer, t_log

  def reset(self):
    super().reset()
    self.angle_ctrl.reset()
    self.torque_ctrl.reset()
    if hasattr(self.torque_ctrl, 'pid'):
      self.torque_ctrl.pid.reset()
