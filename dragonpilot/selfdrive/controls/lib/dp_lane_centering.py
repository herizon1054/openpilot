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
                                            （預設關閉）

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


class DpLaneCentering:
  def __init__(self) -> None:
    self._params = Params()
    self._controller = LaneCenteringController()
    self._last_params_read = 0.0

    self.enabled = False
    self._offset = _LANE_CENTER_OFFSET
    self._e2e_authority = _DEFAULT_E2E_AUTHORITY_PCT / 100.0
    self._pause_on_signal = False
    # 追蹤「上一次呼叫 update() 時 self.enabled 的狀態」，只用來偵測「使用者從設定頁
    # 把 dp_lane_centering 關掉」這個下降緣（enabled: True -> False），跟 controlsd.py
    # 在 latActive 變 False 時呼叫的 reset() 是兩條獨立的觸發路徑，互不影響（見 update()
    # 內的說明）。
    self._was_enabled = False

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
    # 預設關閉，跟 dp_lane_centering 一樣是「預設為關」的布林開關，
    # 未設定時 get_bool() 本來就回傳 False，不需要額外處理預設值
    self._pause_on_signal = self._params.get_bool("dp_lane_centering_pause_on_signal")

  def reset(self) -> None:
    self._controller.reset()

  def update(self, model_curvature: float, model_v2, v_ego: float, lat_active: bool, model_valid: bool,
             turn_signal_active: bool = False, driver_override: bool = False) -> float:
    self._read_params()

    if not self.enabled:
      # dp: 剛從「啟用」切換為「停用」的那一幀，把核心演算法的內部狀態重置乾淨。
      # 這裡回傳 model_curvature 本身不受影響（停用時本來就不會去讀
      # self._controller 的任何狀態），純粹是為了「下次重新啟用」時不要沿用這次
      # 停用前的殘留狀態：
      #   - self._correction：若不重置，重新啟用後 smooth_value() 會拿這個殘留值
      #     當起點去平滑逼近新的目標值，而不是從 0 開始，等於重新啟用瞬間會有一段
      #     跟停用前修正量相關、但跟停用期間路況完全無關的過渡量。
      #   - self._raw_correction_ema（避讓判斷的慢速基準值）：若不重置，重新啟用後
      #     第一幀會拿「停用前、可能是很久以前、很不同路況下」的舊基準值去跟當下的
      #     raw_correction 比較，容易被誤判成「突發避讓」而錯誤地把修正量壓低，直到
      #     基準值花時間追上來為止。
      #   - self._distance_since_reactivation：若不重置，這個值在停用期間仍會是
      #     停用前累積的舊值（很可能已經 >= _REACTIVATION_OBSERVE_DISTANCE），導致
      #     重新啟用時直接跳過第 6 點說明的「重新啟用觀察期」保護——而這個保護機制
      #     原本要防的就是「剛恢復啟用、車道線關聯還沒穩定」這個情境，跟這裡的
      #     使用場景（使用者手動切換開關）完全吻合，不重置等於讓保護機制形同虛設。
      # 只在下降緣觸發一次（而非停用期間每幀都呼叫），避免停用期間不必要的重複呼叫。
      if self._was_enabled:
        self._controller.reset()
      self._was_enabled = False
      return model_curvature

    self._was_enabled = True
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
