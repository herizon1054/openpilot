"""
dplcc 專屬的車道置中 (Lane Centering) 功能 glue 層。

核心演算法完全來自 StarPilot 的移植版本（見
openpilot.selfdrive.controls.lib.lane_centering.LaneCenteringController），
本檔案只負責：
  1. 依照本 fork 既有的 dp_ 慣例，定期（PARAM_REFRESH_SEC）從 Params 讀取
     使用者在設定頁（dragonpilot/settings/min-feat.lat.lane_centering.py）
     調整的數值。
  2. 在 controlsd 每一幀呼叫 update() 時，組出核心演算法需要的參數，回傳
     修正後的 desired curvature。

參數對照（皆由 dragonpilot/settings/min-feat.lat.lane_centering.py 註冊，
車道內置中偏移量除外，見下方說明）：
  dp_lane_centering                 BOOL   總開關
  dp_lane_centering_e2e_authority   INT    端到端模型路徑可覆蓋車道置中的
                                            程度，單位 %（0~100）
  dp_lane_centering_pause_on_signal BOOL   打方向燈時是否淡出車道置中修正

車道內置中偏移量（原本規劃的 dp_lane_centering_offset）改為
「純代碼內設定」，不開放使用者從 UI 調整：寫死在下方的
`_LANE_CENTER_OFFSET` 常數，目前固定為 0.0（不偏移，置中目標點就是車道
正中央）。若之後要調整，直接改這個常數即可，不需要碰 UI/params。
"""
import time

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController

PARAM_REFRESH_SEC = 2.0

# 車道內置中偏移量：純代碼內設定，不對應任何 params/UI 項目。
# 單位公尺，負值偏左、正值偏右，核心演算法內部會依車道寬度自動限縮到安全範圍。
_LANE_CENTER_OFFSET = 0.0

_DEFAULT_E2E_AUTHORITY_PCT = 75
_DEFAULT_PAUSE_ON_SIGNAL = True


class DpLaneCentering:
  def __init__(self) -> None:
    self._params = Params()
    self._controller = LaneCenteringController()
    self._last_params_read = 0.0

    self.enabled = False
    self._offset = _LANE_CENTER_OFFSET
    self._e2e_authority = _DEFAULT_E2E_AUTHORITY_PCT / 100.0
    self._pause_on_signal = _DEFAULT_PAUSE_ON_SIGNAL

    self._read_params(force=True)

  def _get_int(self, key: str, default: int) -> int:
    try:
      val = self._params.get(key)
      return int(float(val)) if val is not None else default
    except (TypeError, ValueError):
      return default

  def _read_params(self, force: bool = False) -> None:
    now = time.monotonic()
    if not force and now - self._last_params_read < PARAM_REFRESH_SEC:
      return
    self._last_params_read = now

    self.enabled = self._params.get_bool("dp_lane_centering")
    # 車道內置中偏移量固定用代碼內常數，不從 params 讀取
    self._offset = _LANE_CENTER_OFFSET
    e2e_authority_pct = self._get_int("dp_lane_centering_e2e_authority", _DEFAULT_E2E_AUTHORITY_PCT)
    self._e2e_authority = max(0.0, min(100.0, e2e_authority_pct)) / 100.0
    # 這個參數預設為「開」，所以跟其他預設為關的布林開關不同，get_bool()
    # 在還沒被使用者寫入前一律回傳 False，必須另外處理未設定時的預設值
    raw_pause = self._params.get("dp_lane_centering_pause_on_signal")
    self._pause_on_signal = _DEFAULT_PAUSE_ON_SIGNAL if raw_pause is None else bool(int(raw_pause))

  def reset(self) -> None:
    self._controller.reset()

  def update(self, model_curvature: float, model_v2, v_ego: float, lat_active: bool, model_valid: bool,
             turn_signal_active: bool = False, driver_override: bool = False) -> float:
    self._read_params()

    if not self.enabled:
      return model_curvature

    return self._controller.update(
      model_curvature,
      model_v2,
      v_ego,
      self.enabled,
      self._offset,
      self._e2e_authority,
      lat_active,
      model_valid,
      self._pause_on_signal,
      turn_signal_active,
      driver_override,
    )
