"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom
import numpy as np
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

# 定義加速度個性化設定的列舉型態 (Eco, Normal, Sport)
AccelPersonality = custom.LongitudinalPlanDP.AccelerationPersonality
ACCEL_PERSONALITY_OPTIONS = [AccelPersonality.eco, AccelPersonality.normal, AccelPersonality.sport]

# ==============================================================================
# 加速度與減速度設定檔 (Profiles & Breakpoints)
# ==============================================================================

# 加速度上限的中斷點 (車速, 單位: m/s)
A_MAX_BP = [0.0,  0.5,  1.0,  4.0,   6.0,  9.0,  11.0, 16.0,  20.0, 25.0, 30.0, 55.0]
# 各種個性化設定下的最大加速度值 (對應 A_MAX_BP)
A_MAX_V = {
  AccelPersonality.eco:       [1.00, 0.60, 1.00, 1.40,  1.20, 1.00, 0.80, 0.60,  0.50, 0.40, 0.12, 0.08],
  AccelPersonality.normal:    [1.60, 0.80, 1.20, 1.80,  1.40, 1.20, 1.00, 0.80,  0.70, 0.60, 0.24, 0.10],
  AccelPersonality.sport:     [2.00, 1.00, 1.40, 2.00,  1.60, 1.40, 1.20, 1.00,  0.90, 0.80, 0.36, 0.12],
}

# 滑行阻力 (Coast Drag) 的中斷點 (車速, 單位: m/s)
COAST_DRAG_BP = [0.0, 10.0, 25.0, 40.0]
# 各種個性化設定下的滑行減速度值 (對應 COAST_DRAG_BP，模擬放開油門時的自然減速)
COAST_DRAG_V = {
  AccelPersonality.eco:    [-0.03, -0.05, -0.08, -0.12],
  AccelPersonality.normal: [-0.04, -0.07, -0.12, -0.18],
  AccelPersonality.sport:  [-0.06, -0.10, -0.18, -0.28],
}

# 煞車底線 (A_MIN Floor) 的中斷點 (車速, 單位: m/s)
A_MIN_FLOOR_BP =    [3., 4.5, 7.,  9., 25]
# 各種個性化設定下的最大允許減速度值 (對應 A_MIN_FLOOR_BP)
A_MIN_FLOOR_V = {
  AccelPersonality.eco:    [-0.16, -0.25, -0.35, -0.48, -2.0],
  AccelPersonality.normal: [-0.17, -0.27, -0.37, -0.50, -2.0],
  AccelPersonality.sport:  [-0.18, -0.29, -0.39, -0.52, -2.0],
}

# ==============================================================================
# 控制模型常數設定
# ==============================================================================
DEFICIT_TO_FLOOR = 8.5  # 當車速低於巡航速度在此範圍內時，逐漸過渡到煞車底線
COAST_DEADBAND = 0.5    # 巡航死區 (m/s)，在此速差範圍內優先進入滑行狀態以維持車速穩定
RAMP_OFF_RANGE = 3.0    # 接近巡航速度時，加速度上限開始線性遞減的緩衝範圍 (m/s)

# 非對稱變化率限制 (Rate Limiting)
A_MIN_TIGHTEN_RATE = 1.5  # 煞車加重時的變化率上限 (m/s³，對應原本的 MAX_DECEL_INCREASE_RATE)
A_MIN_RELAX_RATE = 0.6    # 煞車放鬆時的變化率上限 (m/s³，對應原本的 MAX_DECEL_DECREASE_RATE)
A_MAX_RATE = 1.2          # 加速度上限的變化率 (m/s³)

# 動態安全廊道間距 (Dynamic Safety Corridor Gap)
# 確保最小加速度永遠嚴格小於最大加速度，防止解算器崩潰 (Solver Crash)
MIN_MAX_GAP = 0.05

# 參數重新讀取的幀數間隔 (每秒更新一次 Params)
PARAM_REFRESH_FRAMES = max(1, int(1.0 / DT_MDL))

# ==============================================================================
# 條件式雷達滑行常數設定 (Early Coast)
# ==============================================================================
EARLY_COAST_TRIGGER_FRAMES = max(1, int(0.5 / DT_MDL))  # 進入觸發所需的連續持續時間 (約 0.5 秒)
EARLY_COAST_RELEASE_FRAMES = max(1, int(0.5 / DT_MDL))  # 解除觸發所需的連續持續時間 (約 0.5 秒，避免單幀雜訊瞬間解除)
EARLY_COAST_MIN_DIST = 10.0                             # 最小觸發距離 (公尺)
EARLY_COAST_MAX_DIST = 70.0                             # 最大觸發距離 (公尺，低速時的基準值)

# 最大觸發距離依車速動態延伸的中斷點 (車速, 單位: m/s)
# 60 km/h (16.7 m/s) 以下維持 70m；100 km/h (27.8 m/s) 以上放寬到 100m；中間線性內插
EARLY_COAST_MAX_DIST_BP = [16.7, 27.8]
EARLY_COAST_MAX_DIST_V = [EARLY_COAST_MAX_DIST, 100.0]

# vRel 低通濾波係數 (風格與 dtsc.py / aem.py 的 LPF_ALPHA 一致)
# 純視覺來源 (radard.py 的 get_RadarState_from_vision，無 Kalman 平滑) 的 vRel 逐幀雜訊較大，
# 濾波後再拿去跟 v_rel_thresh / 0.0 比較，避免距離不穩定造成觸發狀態反覆橫跳
EARLY_COAST_VREL_LPF_ALPHA = 0.2


class AccelPersonalityController:
  """
  升級版縱向加減速控制器 (新版模型 tn)
  引入巡航速度 (v_cruise) 依賴性，將減速邏輯區分為滑行阻力與煞車底線，
  並透過接近緩和與巡航死區控制，大幅提升車輛在接近目標車速時的平穩度與舒適性。
  """
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self._first = True  # 標記是否為首次執行，用於初始化加減速狀態

    # 從系統參數讀取當前的個性化設定與啟用狀態
    val = self.params.get('AccelPersonality')
    self._personality = val if val is not None else AccelPersonality.normal
    self._enabled = self.params.get_bool('AccelPersonalityEnabled')

    # 初始化巡航速度與當前加減速限制值
    self._v_cruise = 0.0
    self._a_min = -0.05
    self._a_max = 1.50

    # 效能優化快取機制 (Caching)
    self._cache_v: float | None = None
    self._cache_v_cruise: float | None = None
    self._cache_a_min = self._a_min
    self._cache_a_max = self._a_max

    # 狀態標記：是否觸發提早滑行與連續逼近幀數
    self._force_early_coast = False
    self._approach_frames = 0
    self._release_frames = 0     # 解除觸發的連續幀計數器 (debounce)
    self._vrel_filtered = 0.0    # 低通濾波後的 vRel，降低純視覺來源的逐幀雜訊

  def update(self, sm=None):
    """
    更新控制器狀態，讀取最新車輛巡航速度並定期刷新系統參數
    """
    self.frame += 1
    # 每個 cycle 重設快取
    self._cache_v = None
    self._cache_v_cruise = None

    if sm is not None:
      try:
        # 取得巡航速度
        self._v_cruise = float(sm['carState'].vCruise) * CV.KPH_TO_MS

        # 取得前車狀態
        if 'radarState' in sm:
          lead_one = sm['radarState'].leadOne
          v_ego = float(sm['carState'].vEgo)

          # ==============================================================================
          # 條件式雷達滑行邏輯 (依車速動態延伸的 10m~70/100m 區間 + 進出雙向 Debounce)
          # ==============================================================================
          if lead_one.status:
            # 根據車速動態計算逼近速度閾值
            v_rel_thresh = float(np.interp(v_ego, [16.0, 22.0], [0.5, 1.0]))

            # 根據車速動態計算最大觸發距離 (60km/h 以下 70m，100km/h 以上 100m)
            early_coast_max_dist = float(np.interp(v_ego, EARLY_COAST_MAX_DIST_BP, EARLY_COAST_MAX_DIST_V))

            # 0. 對 vRel 做低通濾波，降低純視覺來源 (無雷達 Track 的 Kalman 平滑) 的逐幀雜訊，
            # 避免跟車距離不穩定時，濾波前的原始 vRel 在門檻附近反覆橫跳
            self._vrel_filtered = (EARLY_COAST_VREL_LPF_ALPHA * float(lead_one.vRel)
                                    + (1.0 - EARLY_COAST_VREL_LPF_ALPHA) * self._vrel_filtered)

            # 1. 判斷是否為「持續逼近」且在有效範圍內 (10m ~ 依車速動態延伸的上限)
            if self._vrel_filtered < -v_rel_thresh and EARLY_COAST_MIN_DIST < lead_one.dRel < early_coast_max_dist:
              self._approach_frames += 1
            else:
              # 若未達逼近閾值或超出距離範圍，中斷連續計數
              self._approach_frames = 0

            # 2. 觸發條件：必須連續確認前車逼近達設定時間 (約 0.5s)
            if self._approach_frames >= EARLY_COAST_TRIGGER_FRAMES:
              self._force_early_coast = True
              self._release_frames = 0  # 重新觸發後，解除計數器歸零

            # 3. 解除條件：前車不再逼近 (濾波後 vRel >= 0)，同樣改為連續 0.5s 才解除，
            # 避免濾波後的 vRel 在 0 附近瞬間跨線就立刻解除，形成對稱的進出 debounce
            if self._vrel_filtered >= 0.0:
              self._release_frames += 1
            else:
              self._release_frames = 0

            if self._release_frames >= EARLY_COAST_RELEASE_FRAMES:
              self._force_early_coast = False
              self._approach_frames = 0
              self._release_frames = 0

          else:
            # 4. Fail-safe: 未鎖定前車，立即重設所有滑行狀態 (不套用濾波與計數器)
            self._force_early_coast = False
            self._approach_frames = 0
            self._release_frames = 0
            self._vrel_filtered = 0.0
          # ==============================================================================

      except Exception:
        # ==============================================================================
        # 5. 例外處理 Fail-safe: 防止出現錯誤時狀態卡死在 True
        # ==============================================================================
        self._force_early_coast = False
        self._approach_frames = 0
        self._release_frames = 0
        self._vrel_filtered = 0.0

    # 定期刷新外部參數，避免每幀讀取 Params 造成 I/O 負擔
    if self.frame % PARAM_REFRESH_FRAMES == 0:
      val = self.params.get('AccelPersonality')
      self._personality = val if val is not None else AccelPersonality.normal
      self._enabled = self.params.get_bool('AccelPersonalityEnabled')

  @property
  def accel_personality(self) -> int:
    return self._personality

  def get_accel_personality(self) -> int:
    return int(self._personality)

  def set_accel_personality(self, personality: int):
    if personality in ACCEL_PERSONALITY_OPTIONS:
      self._personality = personality
      self.params.put('AccelPersonality', personality)

  def cycle_accel_personality(self) -> int:
    idx = ACCEL_PERSONALITY_OPTIONS.index(self._personality) if self._personality in ACCEL_PERSONALITY_OPTIONS else 0
    nxt = ACCEL_PERSONALITY_OPTIONS[(idx + 1) % len(ACCEL_PERSONALITY_OPTIONS)]
    self.set_accel_personality(nxt)
    return int(nxt)

  def is_enabled(self) -> bool:
    return self._enabled

  def set_enabled(self, enabled: bool):
    self._enabled = bool(enabled)
    self.params.put_bool('AccelPersonalityEnabled', self._enabled)

  def toggle_enabled(self) -> bool:
    self.set_enabled(not self._enabled)
    return self._enabled

  def reset(self, personality: int | None = None):
    if personality is None or personality not in ACCEL_PERSONALITY_OPTIONS:
      personality = AccelPersonality.normal
    self._personality = personality
    self.params.put('AccelPersonality', self._personality)
    self.frame = 0
    self._first = True
    self._a_min = -0.05
    self._a_max = 1.50
    self._cache_v = None
    self._cache_v_cruise = None
    self._force_early_coast = False
    self._approach_frames = 0
    self._release_frames = 0
    self._vrel_filtered = 0.0

  def get_accel_limits(self, v_ego: float) -> tuple[float, float]:
    v_ego = max(0.0, v_ego)
    if (self._cache_v is not None
        and abs(self._cache_v - v_ego) < 0.01
        and self._cache_v_cruise == self._v_cruise):
      return self._cache_a_min, self._cache_a_max

    self._cache_a_min, self._cache_a_max = self._step(v_ego)
    self._cache_v = v_ego
    self._cache_v_cruise = self._v_cruise
    return self._cache_a_min, self._cache_a_max

  def get_min_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[0]

  def get_max_accel(self, v_ego: float) -> float:
    return self.get_accel_limits(v_ego)[1]

  def _ramp_off(self, v_ego: float) -> float:
    if self._v_cruise <= 0.0:
      return 1.0
    return float(np.clip((self._v_cruise - v_ego) / RAMP_OFF_RANGE, 0.0, 1.0))

  def _target_max(self, v_ego: float) -> float:
    base = float(np.interp(v_ego, A_MAX_BP, A_MAX_V[self._personality]))
    return base * self._ramp_off(v_ego)

  def _target_min(self, v_ego: float) -> float:
    coast = float(np.interp(v_ego, COAST_DRAG_BP, COAST_DRAG_V[self._personality]))
    if self._v_cruise <= 0.0 or v_ego >= self._v_cruise:
      return coast

    floor = float(np.interp(v_ego, A_MIN_FLOOR_BP, A_MIN_FLOOR_V[self._personality]))
    deficit = self._v_cruise - v_ego
    t = float(np.clip(deficit / DEFICIT_TO_FLOOR, 0.0, 1.0)) ** 1.5
    return coast + t * (floor - coast)

  def _apply_coast_deadband(self, v_ego: float, t_min: float, t_max: float) -> tuple[float, float]:
    if self._v_cruise <= 0.0 or abs(v_ego - self._v_cruise) >= COAST_DEADBAND:
      return t_min, t_max
    coast = float(np.interp(v_ego, COAST_DRAG_BP, COAST_DRAG_V[self._personality]))
    return coast, max(0.05, t_max * 0.25)

  def _rate_limit(self, last: float, target: float, rate_down: float, rate_up: float) -> float:
    rate = rate_up if target > last else rate_down
    step = rate * DT_MDL
    return float(np.clip(target, last - step, last + step))

  def _step(self, v_ego: float) -> tuple[float, float]:
    t_max = self._target_max(v_ego)
    t_min = self._target_min(v_ego)

    t_min, t_max = self._apply_coast_deadband(v_ego, t_min, t_max)

    # ==============================================================================
    # 提早滑行攔截邏輯 (配置於死區修正後)
    # ==============================================================================
    if self._force_early_coast:
        # 取消補油，將加速上限限制在不大於 0.0 的狀態
        # 不主動要求負加速度，讓車輛透過自然阻力滑行
        t_max = min(t_max, 0.0)
    # ==============================================================================

    if self._first:
      self._a_min, self._a_max = t_min, t_max
      self._first = False
      return self._a_min, self._a_max

    new_min = self._rate_limit(self._a_min, t_min, rate_down=A_MIN_TIGHTEN_RATE, rate_up=A_MIN_RELAX_RATE)
    new_max = self._rate_limit(self._a_max, t_max, rate_down=A_MAX_RATE, rate_up=A_MAX_RATE)

    new_min = min(new_min, new_max - MIN_MAX_GAP)

    self._a_min, self._a_max = new_min, new_max
    return self._a_min, self._a_max
