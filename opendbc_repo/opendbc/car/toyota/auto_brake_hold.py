"""
dp - 豐田自動煞車維持 (Auto Brake Hold)

Ported from sunnypilot-candy (opendbc/car/toyota/carcontroller.py: create_auto_brake_hold_messages),
which is in turn based on AlexandreSato/openpilot's automatic brakehold.

原理：
  TSS2 ADAS 相機 (bus 2) 持續送出 PRE_COLLISION_2 (0x344)。
  停車且未踩油門時，panda 會擋掉相機的原始 0x344（fwd hook），改由 openpilot 轉送；
  需要保持時，把 DSS1GDRV 寫成 -2.0 m/s^2 並拉高 PBRTRGR，借用 PCS 煞車把車留在原地。
  車輪一轉動或踩下油門，panda 立刻恢復轉送相機原始訊息並拒絕 openpilot 的 0x344。

三種模式（dp_toyota_auto_brake_hold）：
  0 = OFF   : 關閉
  1 = ON    : 在 D 檔即可作動
  2 = SPEED : 在 D 檔且本次 D 檔期間車速曾超過 5 m/s (18 km/h) 才作動（離開 D 檔即重置）

狀態機（與 candy 相同）：
  IDLE      --(允許保持)-->                       COUNTING
  COUNTING  --(持續允許 BRAKE_HOLD_TIMER 個呼叫)-->  ACTIVE
  ACTIVE    --(再次踩下煞車的上升緣)-->            COUNTING（解除並重新計時）
  任何狀態  --(不再允許保持)-->                    IDLE

允許保持 = D 檔 且 靜止 且 未踩油門 且 ACC 未啟用 且 (非 SPEED 模式 或 已達速度門檻)
"""
from opendbc.car import structs
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import ToyotaFlags

GearShifter = structs.CarState.GearShifter

BRAKE_HOLD_IDLE = 0
BRAKE_HOLD_COUNTING = 1
BRAKE_HOLD_ACTIVE = 2

# 呼叫頻率為 50Hz（carcontroller 每 2 個 frame 呼叫一次），100 次 = 2 秒，與 candy 相同
BRAKE_HOLD_TIMER = 100
# SPEED 模式的速度門檻 (m/s)，5 m/s = 18 km/h，與 candy 相同
BRAKE_HOLD_SPEED_THRESHOLD = 5.


class AutoBrakeHold:
  def __init__(self, CP):
    self.enabled = bool(CP.flags & ToyotaFlags.AUTO_BRAKE_HOLD.value)
    self.speed_mode = bool(CP.flags & ToyotaFlags.AUTO_BRAKE_HOLD_SPEED.value)

    self.state = BRAKE_HOLD_IDLE
    self.counter = 0
    self.active = False
    self._prev_brake_pressed = False
    self._speed_gear_lock = False

  def _update_state(self, CS) -> None:
    gear = CS.out.gearShifter == GearShifter.drive

    if self.speed_mode:
      # 檔位 D 鎖定邏輯：離開 D 檔即解除；在 D 檔且車速 > 5 m/s (18 km/h) 後才啟用
      if not gear:
        self._speed_gear_lock = False
      elif CS.out.vEgo > BRAKE_HOLD_SPEED_THRESHOLD:
        self._speed_gear_lock = True

    standstill_ok = gear and CS.out.standstill and not CS.out.gasPressed
    speed_enabled = self._speed_gear_lock if self.speed_mode else True
    cruise_disabled = not CS.out.cruiseState.enabled and speed_enabled
    brake_hold_allowed = standstill_ok and cruise_disabled

    brake_pressed = CS.out.brakePressed
    brake_pressed_edge = brake_pressed and not self._prev_brake_pressed

    if brake_hold_allowed:
      if self.state == BRAKE_HOLD_IDLE:
        # 允許保持 -> 開始計時
        self.state = BRAKE_HOLD_COUNTING
        self.counter = 0

      elif self.state == BRAKE_HOLD_COUNTING:
        # 持續允許 -> 計時，超過門檻進入保持
        self.counter += 1
        if self.counter > BRAKE_HOLD_TIMER:
          self.state = BRAKE_HOLD_ACTIVE

      elif self.state == BRAKE_HOLD_ACTIVE:
        # 再次踩下煞車 -> 解除並重新計時
        if brake_pressed_edge:
          self.state = BRAKE_HOLD_COUNTING
          self.counter = 0

    else:
      # 不允許保持 -> 無條件清除
      self.state = BRAKE_HOLD_IDLE
      self.counter = 0

    self.active = self.state == BRAKE_HOLD_ACTIVE
    self._prev_brake_pressed = brake_pressed

  def update(self, packer, frame, CS) -> list:
    """每 2 個 frame (50Hz) 呼叫一次。回傳要送出的 CAN 訊息（0 或 1 筆）。"""
    if not self.enabled:
      return []

    # 尚未收到相機 PRE_COLLISION_2 或已過期：不轉送、不保持（狀態機重置，避免恢復時直接跳到 ACTIVE）
    if not CS.pre_collision_2_valid:
      self.state = BRAKE_HOLD_IDLE
      self.counter = 0
      self.active = False
      self._prev_brake_pressed = CS.out.brakePressed
      CS.auto_brake_hold_active = False
      return []

    self._update_state(CS)

    # 與 candy 的差異：只在 panda 會接受的條件（靜止且未踩油門）下才送出。
    # 其餘時間 panda 正在轉送相機原始 0x344，openpilot 送了也只會被 panda 拒絕（徒增 safetyTxBlocked 計數）
    if not CS.out.standstill or CS.out.gasPressed:
      CS.auto_brake_hold_active = False
      return []

    # 回報給 card -> carStateExt.autoBrakeHoldActive -> selfdrived 的 autoBrakeHold 事件
    # （只有真的送出 -2.0 保持指令時才算作動中）
    CS.auto_brake_hold_active = self.active

    return [toyotacan.create_brake_hold_command(packer, frame, CS.pre_collision_2, self.active)]
