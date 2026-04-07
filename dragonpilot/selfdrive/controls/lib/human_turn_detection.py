import time
from enum import Enum, auto

from openpilot.common.params import Params


PARAM_REFRESH_SEC = 2.0
MIN_SPEED_MS = 0.1
# [安全鎖 4] 最高車速限制，約 35 km/h。超過此速度的 90 度大轉向屬危險動作，HTD 拒絕作動
MAX_SPEED_MS = 9.72


class HTDState(Enum):
  INACTIVE = auto()
  MANUAL_TURN = auto()
  RAMPING = auto()


class HumanTurnDetection:
  def __init__(self) -> None:
    self._params = Params()
    self._last_params_read = 0.0
    
    # --- 由 UI 控制的變數 ---
    self._enabled = True
    self._angle_threshold_deg = 90.0

    # --- 以下為寫死的系統參數，不再從 Params 讀取 ---
    # 統一釋放角度為 20 度
    self._angle_release_deg = 20.0
    self._torque_start_nm = 2.0
    self._torque_release_nm = 0.6
    # [新增] 安全接管角度鎖：當角度大於此值時，即使秒數倒數完畢也拒絕恢復自動駕駛
    self._resume_angle_lock_deg = 60.0

    self._state: HTDState = HTDState.INACTIVE
    self._state_change_time = 0.0

    # 紀錄帶有正負號的原始數值，用於方向一致性判斷
    self._last_angle_raw = 0.0
    self._last_torque_raw = 0.0

    self._last_angle = 0.0
    self._last_torque = 0.0
    self._last_pressed = False
    self._trigger_start_time = 0.0

    # 紀錄這一次彎道的最大角度
    self._max_turn_angle = 0.0
    # 準備接管時的動態等待秒數
    self._dynamic_delay = 0.5

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now
    
    # 僅讀取 UI 上有開放設定的參數，避免其他未註冊的參數引發 Exception
    self._enabled = self._params.get_bool("dp_htd_enabled")
    self._angle_threshold_deg = self._get_float("dp_htd_turn_angle_threshold", 90.0)

  # [修正] 增加 reason 參數，避免原版傳入字串時引發 TypeError 崩潰
  def _transition(self, new_state: HTDState, reason: str = "") -> None:
    if new_state == self._state:
      return
    self._state = new_state
    self._state_change_time = time.monotonic()

  def update(
    self, lat_active: bool, cruise_enabled: bool, steering_angle_deg: float, steering_torque_nm: float, v_ego: float, steering_pressed: bool = False
  ) -> tuple[bool, HTDState]:
    self._read_params()

    # 儲存帶正負號的原始數值
    self._last_angle_raw = steering_angle_deg
    self._last_torque_raw = steering_torque_nm

    # 儲存絕對值用於門檻比對
    self._last_angle = abs(steering_angle_deg)
    self._last_torque = abs(steering_torque_nm)
    self._last_pressed = steering_pressed

    # ==========================================
    # [新增] 定速巡航鎖：當定速巡航啟用時，強制讓 HTD 暫停作動
    # ==========================================
    if cruise_enabled:
      if self._state != HTDState.INACTIVE:
        self._transition(HTDState.INACTIVE, "cruise_active")
      self._trigger_start_time = 0.0
      return True, self._state

    # [安全鎖 4] 超速時強制失效
    if not self._enabled or not lat_active or v_ego < MIN_SPEED_MS or v_ego > MAX_SPEED_MS:
      if self._state != HTDState.INACTIVE:
        self._transition(HTDState.INACTIVE, "disabled_or_speed_limit")
      self._trigger_start_time = 0.0
      return True, self._state

    if self._state == HTDState.INACTIVE:
      if self._should_trigger():
        self._max_turn_angle = self._last_angle  # 初始化最大角度
        self._transition(HTDState.MANUAL_TURN, "trigger")
        return False, self._state
      return True, self._state

    if self._state == HTDState.MANUAL_TURN:
      # 持續更新最大角度
      self._max_turn_angle = max(self._max_turn_angle, self._last_angle)

      if self._should_release():
        # 計算動態延遲：根據最大角度計算 (270度 -> 1.0秒)
        calculated_delay = self._max_turn_angle / 270.0
        # 嚴格限制：最少 0.5 秒，最多 1.0 秒
        self._dynamic_delay = max(0.5, min(calculated_delay, 1.0))

        self._transition(HTDState.RAMPING, "release")
      return False, self._state

    # RAMPING 狀態 (安全緩衝區)
    if self._should_trigger():
      self._transition(HTDState.MANUAL_TURN, "retrigger")
      return False, self._state

    # 確保等待時間達到動態秒數 (絕對不會低於 0.5 秒)，避免 Panda 扭力突波報錯
    if time.monotonic() - self._state_change_time >= self._dynamic_delay:
      # [新增安全鎖 5] 接管角度鎖：若方向盤角度仍大於安全閥值 (預設 60 度)，拒絕恢復自動駕駛
      if self._last_angle > self._resume_angle_lock_deg:
        # 維持在 RAMPING 狀態，持續回傳 False 不接管，直到角度回正進入安全範圍
        return False, self._state

      self._max_turn_angle = 0.0  # 徹底恢復前，重置角度紀錄
      self._transition(HTDState.INACTIVE, "resume")
      return True, self._state

    return False, self._state

  def _should_trigger(self) -> bool:
    direction_match = (self._last_angle_raw * self._last_torque_raw) > 0

    condition_met = self._last_pressed and direction_match and self._last_torque >= self._torque_start_nm and self._last_angle >= self._angle_threshold_deg

    if condition_met:
      if self._trigger_start_time == 0.0:
        self._trigger_start_time = time.monotonic()
      elif time.monotonic() - self._trigger_start_time >= 0.2:
        return True
    else:
      self._trigger_start_time = 0.0

    return False

  def _should_release(self) -> bool:
    # [軌道 A] 完美回正：物理方向盤已經被轉回 20 度內，且扭力小
    perfect_return = self._last_torque <= self._torque_release_nm and self._last_angle <= self._angle_release_deg

    # [軌道 B] 放開方向盤
    hands_off = not self._last_pressed

    # 只要滿足任一條件，就同意進入 RAMPING 緩衝倒數
    release_condition = perfect_return or hands_off

    if release_condition:
      self._trigger_start_time = 0.0

    return release_condition

  def _get_float(self, key: str, default: float) -> float:
    try:
      val = self._params.get(key)
      if val is None:
        return default
      return float(val)
    except Exception:
      return default
