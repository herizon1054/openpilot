from opendbc.car import structs
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque

class LatControlDynamic(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    # 同時初始化兩個控制器
    self.angle_ctrl = LatControlAngle(CP, CP_SP, CI, dt)
    self.torque_ctrl = LatControlTorque(CP, CP_SP, CI, dt)

    # 預設使用 CP 讀出來的設定值
    self.use_angle = (CP.steerControlType == structs.CarParams.SteerControlType.angle)

  # ==========================================
  # 🌟 新增：Sunnypilot Torque 專屬代理轉發 (Proxy)
  # ==========================================
  @property
  def extension(self):
    # 當外層要求 extension 時，直接把 torque_ctrl 的 extension 交出去
    return self.torque_ctrl.extension

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, frictionCoefficient):
    # 將即時參數更新指令轉發給內部的 torque_ctrl
    # 注意：sptest/openpilot-sptest 已將 LatControlTorque.update_live_torque_params
    # 重新命名為 update_torque_parameters（與 spcandy 不同），此處已同步改名，
    # 否則 controlsd.py 呼叫 self.LaC.update_torque_parameters(...) 時會拋出 AttributeError。
    if hasattr(self.torque_ctrl, 'update_torque_parameters'):
      self.torque_ctrl.update_torque_parameters(latAccelFactor, latAccelOffset, frictionCoefficient)
  # ==========================================

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # 定義「安全直行狀態」：方向盤打角小於 10 度，且轉動速率小於 5 度/秒
    is_safe_to_switch = abs(CS.steeringAngleDeg) < 10.0 and abs(CS.steeringRateDeg) < 5.0

    # 1. 判斷主控權與遲滯區間，並且鎖死過彎時的切換
    if CS.vEgo > 10.0 and not self.use_angle and is_safe_to_switch:
      self.use_angle = True
      self.angle_ctrl.reset()  # 確保角度控制器狀態乾淨 22ms

    elif CS.vEgo < 8.0 and self.use_angle and is_safe_to_switch:
      self.use_angle = False
      self.torque_ctrl.reset() # 確保扭矩控制器狀態乾淨 16ms
      if hasattr(self.torque_ctrl, 'pid'):
        self.torque_ctrl.pid.reset() # 徹底清除積分

    # 2. Angle 控制器永遠運算 (幾何計算，無風險)
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

    # 3. Torque 控制器永遠運算 (熱備援)
    # 已改回與 spcandy 完全一致的寫法：torque_is_frozen 只會傳給
    # LatControlTorque.update() 的 steer_limited_by_safety 參數，內部只用來
    # 決定 freeze_integrator（只凍結 PID 積分項的累加，不會讓整個輸出停在
    # 舊值——比例項與前饋項每一幀都還是照當下的誤差重新計算），所以這一段
    # 跟「高速失去置中」實際上沒有因果關係，是我先前錯誤的推測，已改回原樣。
    torque_is_frozen = True if self.use_angle else steer_limited_by_safety
    t_steer, _, t_log = self.torque_ctrl.update(active, CS, VM, params, torque_is_frozen, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

    # 4. 雙輸出合併：回傳 (扭矩輸出, 角度輸出, 當前主控的Log)
    # 注意：controlsd.py 的 publish() 已經加上 hasattr(self.LaC, 'use_angle') 的
    # 動態判斷分支，會依照 self.use_angle 動態選擇要把這裡回傳的 log 寫進
    # cs.lateralControlState 的 angleState 還是 torqueState 分支，兩邊型別
    # 保證一致，不會再發生 capnp union 型別不符的崩潰。
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
