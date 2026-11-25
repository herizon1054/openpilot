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

洩壓（與 candy 不同，見 BRAKE_HOLD_REFRESH_* 說明）：
  保持中每約 5 秒放開 PBRTRGR 40 ms（同 candy ac4dd42），並以 BRAKE_FORCE 下降確認 ECU 有接受，沒接受就重送。

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

# 保持中 PBRTRGR 洩壓脈衝（update 以 50 Hz 呼叫，以下單位皆為「訊息數」）
# candy 原本用 frame % 730 < 727（50 Hz 下實際只放開 1 個訊息 / 20 ms）；candy/opendbc ac4dd42 已改為
# (frame // 2) % 252 < 250（約 5 秒放開 40 ms），這裡採用相同的長度與週期。
# 與 candy 不同（刻意）：以進入保持為起點計時、用 BRAKE_FORCE 確認 ECU 有洩壓（沒有就重送）、BRAKE_LIGHTS_ACC 熄滅時補洩壓。
# rlog 實證（COROLLA TSS2，sptest）：
#   - ECU 接受洩壓時 BRAKE(0xA6).BRAKE_FORCE 會短暫下降（2720 → 2280 N）
#   - 單一訊息的脈衝偶爾被 ECU 漏收（力量沒下降）
#   - 距上一次被接受的洩壓滿 10 秒，ESP 自行放掉 PCS 煞車，儀表跳「碰撞預防系統故障」→ 溜車
BRAKE_HOLD_REFRESH_PERIOD = 252   # 約 5 秒洩壓一次（同 candy ac4dd42；以進入保持 / 上次洩壓為起點），ECU 上限約 10 秒
BRAKE_HOLD_REFRESH_CUT = 2        # 放開 2 個訊息（40 ms），同 candy ac4dd42「5000ms ON / 40ms OFF」
BRAKE_HOLD_VERIFY_WINDOW = 10     # 脈衝結束後 200 ms 內要看到 BRAKE_FORCE 下降
BRAKE_HOLD_RETRY_DELAY = 25       # 沒看到下降 → 0.5 秒後重送
BRAKE_HOLD_MAX_RETRY = 5          # 連續重送上限，超過回到定期洩壓（BRAKE_FORCE 不反應的車型不會一直洩壓）
BRAKE_HOLD_DIP_N = 200.           # BRAKE_FORCE 至少下降 200 N 才算 ECU 有接受
BRAKE_HOLD_MIN_FORCE = 1000.      # 脈衝前保持力低於此值不做確認
BRAKE_HOLD_REARM_DELAY = 50       # 補救：進入保持 / 洩壓後 1 秒才開始檢查 BRAKE_LIGHTS_ACC


class AutoBrakeHold:
  def __init__(self, CP):
    self.enabled = bool(CP.flags & ToyotaFlags.AUTO_BRAKE_HOLD.value)
    self.speed_mode = bool(CP.flags & ToyotaFlags.AUTO_BRAKE_HOLD_SPEED.value)

    self.state = BRAKE_HOLD_IDLE
    self.counter = 0
    self.active = False
    self._prev_brake_pressed = False
    self._speed_gear_lock = False
    self._reset_refresh()

  def _reset_refresh(self) -> None:
    self._msgs = 0            # 進入保持 / 上次洩壓後經過的訊息數
    self._cut_left = 0        # 洩壓脈衝剩餘訊息數
    self._verify_left = 0     # 洩壓確認視窗剩餘訊息數
    self._ref_force = 0.      # 脈衝前的 BRAKE_FORCE
    self._dip_seen = False    # 本次脈衝已看到 BRAKE_FORCE 下降
    self._retries = 0         # 連續重送次數
    self._lights_seen = False # 本次保持中 ESP 曾回報 BRAKE_LIGHTS_ACC=1

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
          self._reset_refresh()

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

    release_pulse = self._refresh(CS) if self.active else False
    return [toyotacan.create_brake_hold_command(packer, frame, CS.pre_collision_2, self.active, release_pulse)]

  def _refresh(self, CS) -> bool:
    """保持中的 PBRTRGR 洩壓脈衝，回傳 True = 這個訊息 PBRTRGR 送 0。
    1. 定期：滿 BRAKE_HOLD_REFRESH_PERIOD 個訊息 → 放開 BRAKE_HOLD_REFRESH_CUT 個訊息。
    2. 確認：脈衝開始到結束後 BRAKE_HOLD_VERIFY_WINDOW 內 BRAKE_FORCE 須下降 BRAKE_HOLD_DIP_N，
       沒看到 → ECU 漏收，BRAKE_HOLD_RETRY_DELAY 後重送。
    3. 補救：本次保持中 BRAKE_LIGHTS_ACC 亮過後熄掉（ECU 已放掉）→ 立刻再送一次。"""
    force_min = CS.brake_force_min   # carstate 兩次讀取之間（100 Hz）所有 BRAKE_FORCE 的最小值
    force = CS.brake_force
    CS.brake_force_min = float('inf')
    can_verify = force is not None

    def dip_check():
      if not self._dip_seen and force_min <= self._ref_force - BRAKE_HOLD_DIP_N:
        self._dip_seen = True

    self._msgs += 1

    # 脈衝進行中
    if self._cut_left > 0:
      dip_check()
      self._cut_left -= 1
      if self._cut_left == 0:
        self._msgs = 0
        self._verify_left = BRAKE_HOLD_VERIFY_WINDOW if can_verify else 0
      return True

    # 脈衝後確認視窗
    if self._verify_left > 0:
      dip_check()
      self._verify_left -= 1
      if self._dip_seen:
        self._verify_left = 0
        self._retries = 0
      elif self._verify_left == 0 and self._ref_force >= BRAKE_HOLD_MIN_FORCE and self._retries < BRAKE_HOLD_MAX_RETRY:
        # ECU 沒接受這次洩壓 → 0.5 秒後重送
        self._retries += 1
        self._msgs = BRAKE_HOLD_REFRESH_PERIOD - BRAKE_HOLD_RETRY_DELAY

    lights = CS.brake_lights_acc
    if lights:
      self._lights_seen = True
    dropped = (not lights) and self._lights_seen and self._msgs >= BRAKE_HOLD_REARM_DELAY

    if self._msgs >= BRAKE_HOLD_REFRESH_PERIOD or dropped:
      if dropped:
        self._lights_seen = False  # 等 ECU 重新亮燈才再次啟用補救
      self._ref_force = force if force is not None else 0.
      self._dip_seen = False
      self._verify_left = 0
      self._cut_left = BRAKE_HOLD_REFRESH_CUT - 1
      if self._cut_left == 0:
        self._msgs = 0
        self._verify_left = BRAKE_HOLD_VERIFY_WINDOW if can_verify else 0
      return True
    return False
