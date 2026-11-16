from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as LatControlTorqueV1
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

# ==========================================
# 🛡️ 配合 Panda (toyota.h) 限制與台灣路況的切換參數
# ==========================================
# 1. 車速門檻固定不動 (1 m/s = 3.6 km/h)
ANGLE_ENABLE_SPEED = 22.0        # 固定 > 22.0 m/s (79.2 km/h) 才允許切入 Angle
TORQUE_FALLBACK_SPEED = 16.0     # 固定 < 16.0 m/s (57.6 km/h) 切回 Torque
PANDA_SAFE_FALLBACK_SPEED = 12.5 # < 12.5 m/s (45.0 km/h) 強制保底切回 Torque (無視打角大小)

# 2. 高速切入 Angle 條件 (維持原本嚴格標準：確保高速直線無感切換)
TO_ANGLE_MAX_DEG = 10.0          # 打角 < 10.0 度
TO_ANGLE_MAX_RATE = 5.0          # 轉速 < 5.0 度/秒

# 3. 低速切回 Torque 條件 (對照 toyota.h 預留安全裕度)
# 在 16.0 m/s 時，Panda LTA 速率上限為 21.75 度/秒，設為 19.0 度/秒預留 2.75 度/秒緩衝
TO_TORQUE_MAX_DEG = 25.0         # 打角放寬至 < 25.0 度 (涵蓋台灣交流道減速與一般彎道)
TO_TORQUE_MAX_RATE = 19.0        # 轉速 < 19.0 度/秒 (預留安全餘裕，確保撞牆前順利切回)

# 在 12.5 m/s 時，Panda LTA 速率上限為 24.375 度/秒，設為 22.0 度/秒預留緩衝
FORCE_TORQUE_MAX_RATE = 22.0     # 低於 45 km/h 時，只要轉速 < 22.0 度/秒即強制退回 Torque


class LatControlDynamic(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.params = Params()

    # 1. 初始化角度控制器
    self.angle_ctrl = LatControlAngle(CP, CP_SP, CI, dt)

    # 2. 讀取 sunnypilot UI 的 Torque 版本設定 (預設使用 V0，支援 UI 切換 V1)
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_version = self.params.get("TorqueControlTune")

    try:
      is_v1 = (float(torque_version) == 1.0) if torque_version is not None else False
    except (ValueError, TypeError):
      is_v1 = False

    if enforce_torque_control and is_v1:
      self.torque_ctrl = LatControlTorqueV1(CP, CP_SP, CI, dt)
    else:
      self.torque_ctrl = LatControlTorqueV0(CP, CP_SP, CI, dt)

    # 預設使用 CP 讀出來的設定值
    self.use_angle = (CP.steerControlType == structs.CarParams.SteerControlType.angle)

  # ==========================================
  # 🌟 Sunnypilot Torque 專屬代理轉發 (Proxy)
  # ==========================================
  @property
  def extension(self):
    # 當外層要求 extension 時，直接把 torque_ctrl 的 extension 交出去
    return self.torque_ctrl.extension

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, frictionCoefficient):
    # 將即時參數更新指令轉發給內部的 torque_ctrl
    self.torque_ctrl.update_torque_parameters(latAccelFactor, latAccelOffset, frictionCoefficient)
  # ==========================================

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    abs_angle = abs(CS.steeringAngleDeg)
    abs_rate = abs(CS.steeringRateDeg)

    # 高速切入 Angle：維持嚴格直行條件 (< 10度 且 < 5度/秒)
    can_switch_to_angle = abs_angle < TO_ANGLE_MAX_DEG and abs_rate < TO_ANGLE_MAX_RATE

    # 低速切回 Torque：放寬條件 (< 25度 且 < 19度/秒)，或車速低於 12.5 m/s 保底切換 (< 22度/秒)
    can_switch_to_torque = (abs_angle < TO_TORQUE_MAX_DEG and abs_rate < TO_TORQUE_MAX_RATE) or \
                           (CS.vEgo < PANDA_SAFE_FALLBACK_SPEED and abs_rate < FORCE_TORQUE_MAX_RATE)

    # 1. 判斷主控權與遲滯區間 (車速嚴格鎖定 > 22.0 與 < 16.0)
    if CS.vEgo > ANGLE_ENABLE_SPEED and not self.use_angle and can_switch_to_angle:
      self.use_angle = True
      self.angle_ctrl.reset()  # 確保角度控制器狀態乾淨

    elif CS.vEgo < TORQUE_FALLBACK_SPEED and self.use_angle and can_switch_to_torque:
      self.use_angle = False
      self.torque_ctrl.reset() # 確保扭矩控制器狀態乾淨
      if hasattr(self.torque_ctrl, 'pid'):
        self.torque_ctrl.pid.reset() # 徹底清除積分

    # 2. Angle 控制器永遠運算 (幾何計算，無風險)
    _, a_steer, a_log = self.angle_ctrl.update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

    # 3. Torque 控制器永遠運算 (熱備援)
    torque_is_frozen = True if self.use_angle else steer_limited_by_safety
    t_steer, _, t_log = self.torque_ctrl.update(active, CS, VM, params, torque_is_frozen, desired_curvature, calibrated_pose, curvature_limited, lat_delay)

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
