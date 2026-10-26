#!/usr/bin/env python3
"""
dragonpilot radard 擴充（radard_ext）：在原廠 radard 的前車選擇上，加入以下機制。

  1. 提早鎖定（Early Lock）：每個雷達目標依「與視覺前車的吻合程度」累積 EMA 信心度，
     信心度高時降低視覺機率門檻（0.5 → 最低 0.3），讓雷達目標更早被配對成前車。
  2. 橫向閘門：落在本車預測路徑走廊外、且與視覺前車不一致的雷達目標不參與配對（去抖動）。
  3. 前車選擇穩定化：鎖定黏著、更遠目標需連續確認、Toyota 重複雷達點（雙 trackId）不互換、
     aLeadK 變化率限制、短暫斷流續命。
  4. 雷達主導救援：本車道內連續 1 秒穩定移動的雷達目標，視覺機率只需 >= 0.1 即可成為前車。
  5. 雷達切入預測：以 alpha-beta 濾波估計目標相對路徑的橫向偏移與速度，預測即將切入本車道時
     提早採用為前車（只處理 leadOne）。
  6. 聯結車／大型車防護（v9.6~）：多點大型車須越過模型車道線才算切入候選。

以 monkey patch 取代 radard.Track 與 radard.get_lead，RadarDExt 由 radard.main() 建立。
只在 radard.DP_RADARD_EXT_BRANDS（Toyota）載入；其他品牌不 import 本模組，維持原廠前車選擇。

版本紀錄（切入預測）：
  v9.2   啟用車速 36 → 20 km/h（20~36 km/h 確認 6 幀）；radard.py 的換道判斷不看方向燈。
  v9.3   同車一致性（大車多反射點）、停滯釋放；停用切入狀態跨 ID 繼承。
  v9.4   1 秒淨橫移條件（擋單一反射點大型車的橫向抖動）。
  v9.6   多點大型車改以模型車道線判斷切入；車道線信心度 < 0.5 時退回本車路徑 + 濾波車道寬；
         模型路徑跳動保護。小車維持 v9.4。
  v9.6.2 大型車身分補強：ID 黏著、重複點繼承、只看移動中的點。
  v9.6.3 雷達主導救援在車道線可靠時，目標須已越過車道線（路徑偏移時不會救援旁車道聯結車）。
  v9.6.4 品牌閘門（radard.py）：只有 Toyota 載入 radard_ext；其他品牌使用原廠 get_lead / Track。
         所有門檻只用 Toyota TSS2 log 驗證，其他品牌雷達的點數、measured 語意、橫向尺度不同（見 radard.py）。
  v9.6.5 雷達主導救援、切入預測的啟用車速提高到 > 30 km/h（避開市區低速路口轉彎）；
         已採用中的目標沿用原車速下限（救援 10.8 km/h、切入 20 km/h），本車減速時前車不會突然消失。
  v9.6.6 切入預測一律以越過車道線為標準（不再只限多點大型車）：雷達點越過目標側車道線才算候選、
         退回線外 0.1m 即釋放；車道線信心度 < 0.5 時退回本車路徑 + 濾波車道寬（越過推估線 0.3m）。
         單一反射點的聯結車／大車在旁車道橫向抖動、路徑漂移不再觸發切入。代價：真切入要等壓過線才採用。
  v9.6.7 切入／大型車車道線門檻提前 0.1m：雷達點壓線（線外 0.1m 內）即為候選，釋放與本車路徑模式同步
         提前 0.1m。續命例外：視覺前車比鎖定中的雷達前車近 HOLD_BYPASS_DD 以上且視覺機率高時不續命，
         當幀換成視覺前車（log 032 藍車：遠方前車續命讓換到視覺前車晚 0.25 秒）。

與原廠（cp）的刻意分歧：
  - 低速覆寫：現有雷達前車與最近目標為同一物體時不替換（原廠只要更近就替換，重複點會每幀互換）。
  - 雷達前車的 aLeadK 有變化率限制；切入／救援前車的 modelProb 填 0，不套用視覺推得的 aLeadTau。
  - 其餘分歧見各參數區說明。
"""
import capnp
import numpy as np
from collections import deque
from typing import Any

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import (
  KalmanParams, Track, RadarD, match_vision_to_track, get_RadarState_from_vision, RADAR_TO_CAMERA,
)


# ==============================================================================
# 參數：提早鎖定（EMA 信心度）
# ==============================================================================
FUZZY_BOUNDS = [0.5, 1.5]           # 橫向/速度誤差 (m, m/s)：0.5 內滿分 1.0，1.5 以上歸零
FUZZY_D_BOUNDS_PCT = [0.05, 0.12]   # 距離誤差的滿分/歸零門檻隨距離放大（下限仍為 FUZZY_BOUNDS）。
                                    # 視覺測距在中遠距離誤差常超過 1.5m，固定門檻會讓 30m 外 EMA 無法累積

ALPHA_BASE = 0.2                    # EMA 上升學習率
ALPHA_DOWN = 0.1                    # EMA 下降／失效衰減學習率
BRAKE_THRES_RANGE = [-3.0, -1.2]    # 目標減速度 (m/s²) → 威脅倍率 MULT_RANGE
MULT_RANGE = [1.2, 1.0]
CUTIN_DIST_LIMIT = 40.0             # 橫向切入威脅倍率的最大評估距離 (m)
DYNAMIC_SPEED_PCT = 0.2             # 「移動中」判定：絕對速度 > max(1.0, 此比例 × 本車速)

CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 慢速目標保護：本車速 → 視覺機率門檻
CAM_PROB_RANGE = [0.5, 0.3]
STATIC_EMA_CAP = 0.6                # 慢速目標視覺機率未達門檻時的 EMA 上限

EMA_VAL_RANGE = [0.4, 0.8]          # 最高 EMA 信心度 → 視覺機率門檻
PROB_THRES_RANGE = [0.5, 0.3]
RELEASE_FRAMES = 5                  # 目標短暫失效時 EMA 凍結（不衰減）的幀數

# 比照原廠 match_vision_to_track() 的 vel_sane 擇一放行：(速度誤差 < 10) OR (v_ego + vRel > 3)。
# v_ego + vRel 是目標的絕對速度：目標本身在移動就不是靜止雜訊。此處要求連續 3 幀成立才放行。
VEL_SANE_FALLBACK_SPEED = 3.0       # m/s
VEL_SANE_CONFIRM_FRAMES = 3
VEL_SANE_FALLBACK_SCORE = 1.0       # 放行後 score_v 的下限

# ==============================================================================
# 參數：橫向閘門
# ==============================================================================
# 排除條件需同時成立（去抖動 GATE_DEBOUNCE_FRAMES）：
#   (a) 走廊外：|yRel − 路徑y(d)| > LANE_CORRIDOR_HALF_WIDTH（近距離加寬），且
#   (b) 與視覺前車不一致：橫向差 > LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN，或速度差過大。
# 7 段 log：同一物體雷達/視覺橫向差在 20~40m 有 20% 超過 2.0m，不能只看視覺 y；走廊內一律不排除。
LANE_WIDTH_FALLBACK = 1.5           # m
LANE_HYSTERESIS_MARGIN = 0.5        # m（兩者合計 2.0m 為「與視覺前車橫向不一致」門檻）
LANE_CORRIDOR_HALF_WIDTH = 1.75     # m
# Toyota 雷達近距離常打到車尾角落（<10m 時雷達/視覺橫向差中位數約 1.0m）：
# 距離 <= 8m 走廊半寬加 1.0m，15m 以上不加。
LANE_CORRIDOR_NEAR_BP = [8.0, 15.0]
LANE_CORRIDOR_NEAR_EXTRA = [1.0, 0.0]
LANE_GATE_DV_MIN = 2.0              # m/s，速度不一致門檻 = max(此值, LANE_GATE_DV_PCT × |視覺前車速度|)
LANE_GATE_DV_PCT = 0.25
# 旁車切入時橫向正好跨越走廊邊界，無狀態閘門會每幀翻轉（log 92102 45.2s）。
# 新目標第一幀直接採用判定，之後需連續 N 幀相反才切換；進出同一組條件。
GATE_DEBOUNCE_FRAMES = 3
# 路口轉彎時模型路徑在直行/轉彎間跳動：同一距離處路徑 y 一幀變化超過此值，該幀視為未出界。
GATE_PATH_JUMP_LIMIT = 1.0          # m

# ==============================================================================
# 參數：前車選擇（續命、黏著、重複點、aLeadK）
# ==============================================================================
SELECT_HOLDOVER_FRAMES = 3          # 配對失敗時沿用上一個雷達前車的幀數
# 鎖定黏著（只套用在移動中的鎖定目標）：前車在兩台車之間跳動時，原廠 dist_sane 時過時不過。
#   - 鎖定目標仍在雷達清單與走廊內、且沒有更近的純視覺前車：續命延長為 STICKY_HOLD_FRAMES。
#   - 換成「更遠」的目標需連續 SWITCH_FARTHER_CONFIRM_FRAMES 幀；換成「更近」的（切入車）立即切換。
STICKY_HOLD_FRAMES = 10             # 0.5 秒
# v9.6.7 續命例外：配對失敗時，若純視覺前車比上一個雷達前車近 HOLD_BYPASS_DD 以上、且視覺機率
# >= HOLD_BYPASS_MIN_PROB，不續命，當幀改用視覺前車（模型已換到較近的切入車，但視覺測距誤差大、
# 雷達配對不到時，續命會讓前車停留在遠方車）。原廠沒有續命，此情況等同回到原廠行為。
HOLD_BYPASS_DD = 10.0               # m
HOLD_BYPASS_MIN_PROB = 0.7
SWITCH_FARTHER_CONFIRM_FRAMES = 5   # 0.25 秒
# Toyota TSS2 雷達常把同一台車回報成兩個 trackId（log 92462：60 秒內互換 279 次）。
# 三項差距都在下列範圍內視為同一物體（log 中重複點最大差距：1.13m / 0.60m / 0.90 m/s）。
SAME_OBJ_MAX_DD = 1.5               # m
SAME_OBJ_MAX_DY = 1.0               # m
SAME_OBJ_MAX_DV = 1.5               # m/s
# 同一物體的雷達前車 aLeadK 每幀最大變化量；換成不同物體時不限制（見 _build_lead_dict）。
MAX_ALEADK_DELTA_PER_FRAME = 1.0    # m/s²
# 視覺加速度雙重驗證：視覺前車機率夠高且預測急煞時，依後續加速度決定 aLeadTau。
MODEL_TAU_MIN_PROB = 0.5
MODEL_TAU_BRAKE_A = -0.5            # m/s²
MODEL_TAU_SUSTAINED = 0.5           # 視覺確認急煞持續
MODEL_TAU_SPURIOUS = 3.0            # 視覺預測即將回升

# ==============================================================================
# 參數：模型路徑（modelV2.position）
# ==============================================================================
# 車速低於此值時 position.x 擠在 0 附近、不單調：橫向閘門改為假設直行，救援與切入預測停用。
MODEL_PATH_MIN_SPEED = 3.0          # m/s
MODEL_PATH_MIN_LENGTH = 5.0         # m

# ==============================================================================
# 參數：雷達主導救援（只處理 leadOne）
# ==============================================================================
# 雷達目標連續 RADAR_RESCUE_CONFIRM_FRAMES 幀同時符合：真實量測、向前移動、在模型路徑走廊內、
# 距離範圍內、方向盤角度小、模型路徑有效 → 成為救援候選。任一幀不符即歸零。
# 採用：原始（未濾波）視覺機率 >= RADAR_RESCUE_MIN_PROB，且沒有前車或比現有前車更近。
# 救援前車不享有續命，條件不成立當幀撤銷。
RADAR_RESCUE_MIN_PROB = 0.1
RADAR_RESCUE_CORRIDOR = 1.0         # m，進入條件
RADAR_RESCUE_CORRIDOR_HOLD = 1.5    # m，已在救援中的目標的維持條件（避免遠距離橫向雜訊鎖一下就放掉）
RADAR_RESCUE_CONFIRM_FRAMES = int(1.0 / DT_MDL)   # 連續 1 秒
RADAR_RESCUE_MIN_DIST = 5.0         # m
RADAR_RESCUE_MAX_DIST = 120.0       # m
RADAR_RESCUE_MIN_SPEED = 3.0        # m/s，絕對速度下限 = max(此值, PCT × 本車速)
RADAR_RESCUE_MIN_SPEED_PCT = 0.2
RADAR_RESCUE_MAX_ANGLE = 25.0       # deg
# v9.6.5：本車速 > 30 km/h 才開始累積救援幀數（市區低速路口轉彎不救援）。已在救援中的目標不受此限
# （仍需模型路徑有效，即 > MODEL_PATH_MIN_SPEED）：跟著救援前車減速到 30 km/h 以下時不會當幀撤銷。
RADAR_RESCUE_MIN_V_EGO = 30.0 / 3.6 # m/s
# v9.6.3：目標側車道線信心度 >= LANE_LINE_MIN_PROB 時，救援目標須已越過車道線（雷達點在本車道內）。
# 救援走廊以模型路徑為中心，路徑往旁車偏時，旁車道聯結車側面點會落入走廊（虛擬測試：路徑右偏
# 0.85m，線外 0.5m 的聯結車被救援採用，−2.7 m/s²）。8 段實車 log 的救援目標越線距離皆 >= 0.00m。
RADAR_RESCUE_LINE_ENTER = 0.0       # m，進入：雷達點越過車道線
RADAR_RESCUE_LINE_HOLD = -0.2       # m，維持：已在救援中的目標允許壓線雜訊

# ==============================================================================
# 參數：雷達切入預測（只處理 leadOne）
# ==============================================================================
# 背景（log 032/033，約 105 km/h）：藍車從右車道切入，雷達 57.5s 已看到它壓線，但模型前車仍指向
# 57m 外的遠車，直到 59.4s 才鎖到藍車（駕駛 57.89s 已踩煞車）。
# 採用條件：連續 CUTIN_CONFIRM_FRAMES 幀成立、同車一致、1 秒淨橫移足夠，且沒有前車或比現有前車
# 更近（且不是同一物體）。切入前車不享有續命。
# v9.6.5：新候選需本車速 > 30 km/h（市區低速路口轉彎不預測切入）；已採用中的切入目標維持到
# CUTIN_HOLD_MIN_V_EGO，本車為切入車減速時不會當幀釋放。
CUTIN_MIN_V_EGO = 30.0 / 3.6        # m/s
CUTIN_HOLD_MIN_V_EGO = 20.0 / 3.6   # m/s
CUTIN_LOW_SPEED = 36.0 / 3.6        # m/s，低於此車速（機車多、反射點跳動大）確認幀數提高
CUTIN_CONFIRM_FRAMES = 4            # 0.2 秒
CUTIN_CONFIRM_FRAMES_LOW = 6        # 0.3 秒
CUTIN_MIN_DIST = 3.0                # m
CUTIN_MAX_DIST_BP = [10.0, 30.0]    # 本車速 m/s → 偵測距離上限 m（約 1.6~2 秒車距）
CUTIN_MAX_DIST_V = [20.0, 50.0]
CUTIN_CORRIDOR = 1.0                # m，預測要進入的走廊半寬
CUTIN_OFF_MAX = 2.5                 # m，小車候選偏移上限（半車道 1.75 + 半車寬 0.85）
CUTIN_HOLD_OFF = 2.5                # m，小車已採用後的維持偏移上限
CUTIN_VY_MIN = 0.3                  # m/s
CUTIN_VY_MAX = 2.5                  # m/s
CUTIN_T_ENTER = 1.5                 # s，預測在此秒數內進入走廊
CUTIN_CLOSING_VREL = -0.5           # 縱向接近：vRel < 此值，或距離 < 本車速 × CUTIN_CLOSE_HEADWAY
CUTIN_CLOSE_HEADWAY = 1.0           # s
CUTIN_ALPHA = 0.3                   # alpha-beta 濾波：位置增益
CUTIN_BETA = 0.05                   # alpha-beta 濾波：速度增益
CUTIN_PATH_JUMP_LIMIT = 0.5         # m/幀，路徑跳動時重設濾波（CUTIN_BETA/DT_MDL = 1.0，跳 1m 會注入 1 m/s）
CUTIN_MEASURED_MAX_AGE = 2          # 幀，容許的連續非實測（外推）幀數
CUTIN_PENDING_MAX_FRAMES = 50       # 停滯釋放：採用後 2.5 秒仍未進入走廊即釋放
# 同車一致性：大車最強反射點會沿車身滑動，看起來像在往內移；同車其他反射點也須朝路徑移動。
CUTIN_GROUP_DD = 15.0               # m
CUTIN_GROUP_DY = 1.5                # m
CUTIN_GROUP_DV = 1.0                # m/s
CUTIN_GROUP_VY_RATIO = 0.4
CUTIN_GROUP_VY_MIN = 0.15           # m/s
# 1 秒淨橫移：真切入一路往內，抖動一下往內一下往外（藍車 0.95m vs 大型車抖動 −0.06~0.10m）。
CUTIN_NET_SHIFT_FRAMES = 20         # 1.0 秒
CUTIN_NET_SHIFT_MIN = 0.5           # m
CUTIN_NET_SHIFT_MIN_FRAMES = 10     # 追蹤不滿 1 秒的新雷達點至少觀察 0.5 秒
CUTIN_NET_SHIFT_REVERSAL = 0.15     # m，新雷達點觀察期間允許的往外回彈

# ==============================================================================
# 參數：多點大型車與車道線（v9.6~）
# ==============================================================================
# 聯結車在旁邊車道並行時，側面多個反射點落在小車候選帶內，反射點滑動或路徑漂移會被誤判為切入。
# 多點大型車改以目標側模型車道線判斷：雷達點須越過車道線才算候選，退回線外即釋放。
# 車道線信心度不足時退回本車路徑 ± 濾波車道寬/2（本車在車道內的偏移未知，候選多扣餘裕）。
# 限制：從後方遠距離只看得到車尾點時無法判定為大型車，沿用小車規則。
LARGE_VEH_GROUP_DD = 25.0           # m，判定時的縱向搜尋範圍（涵蓋超長聯結車）
LARGE_VEH_CHAIN_GAP = 4.0           # m，鏈上相鄰反射點最大間距
LARGE_VEH_MIN_SPAN = 5.0            # m，鏈長門檻
LARGE_VEH_HOLD_FRAMES = 20          # 1.0 秒遲滯
LANE_LINE_MIN_PROB = 0.5            # 目標側車道線信心度下限
LARGE_LINE_ENTER = -0.1             # m，車道線模式候選：雷達點壓線（線外 0.1m 內）（v9.6.6：0.0 越線）
LARGE_LINE_RELEASE = -0.2           # m，車道線模式釋放：退回線外 0.2m（v9.6.6：-0.1）
LARGE_PATH_ENTER = 0.2              # m，本車路徑模式候選：越過推估車道線 0.2m（v9.6.6：0.3）
LARGE_PATH_RELEASE = -0.1           # m，本車路徑模式釋放：退回推估車道線外 0.1m（v9.6.6：0.0）
LANE_WIDTH_DEFAULT = 3.5            # m
LANE_WIDTH_MIN = 3.0                # m
LANE_WIDTH_MAX = 4.0                # m
LANE_WIDTH_X = 8.0                  # m，量測車道寬的位置（模型座標）
LANE_WIDTH_ALPHA = 0.05             # 每幀濾波係數（約 1 秒時間常數）
# v9.6.6：所有切入目標（不分大小車）都套用上方的車道線規則（LARGE_LINE_* / LARGE_PATH_*）。
# False = 回到 v9.6.5（只有多點大型車用車道線，小車用 CUTIN_OFF_MAX / CUTIN_HOLD_OFF 偏移帶）。
CUTIN_LINE_CROSS_ALL = True

# ==============================================================================
# 模組狀態（跨幀）
# ==============================================================================
_LANE_WIDTH = {'w': LANE_WIDTH_DEFAULT}   # 濾波後車道寬
_LOW_SPEED_LAST = {'track': None}         # 上一幀低速覆寫選到的雷達目標
# 每個前車槽位（0 = leadOne，1 = leadTwo）的選擇狀態，直接快取 Track 物件：
#   track/absent：目前前車與連續配對失敗幀數；rescue/cutin：前車來自救援/切入（皆不續命）；
#   last_aLeadK/last_aLeadK_track：aLeadK 變化率限制基準；cutin_track：上一幀的切入前車；
#   switch_id/switch_cnt：換成更遠目標的連續確認。
_LEAD_STATE_CACHE = {
  i: {'track': None, 'absent': 0, 'last_aLeadK': None, 'last_aLeadK_track': None, 'rescue': False,
      'cutin': False, 'cutin_track': None, 'switch_id': None, 'switch_cnt': 0}
  for i in (0, 1)
}


# ==============================================================================
# 共用函數
# ==============================================================================
def _is_same_object(a, b) -> bool:
  # 兩個雷達目標是否為同一實體物體的重複回報
  return (abs(a.dRel - b.dRel) <= SAME_OBJ_MAX_DD and
          abs(a.yRel - b.yRel) <= SAME_OBJ_MAX_DY and
          abs(a.vRel - b.vRel) <= SAME_OBJ_MAX_DV)


def get_model_lead_tau(lead_msg, lead_prob: float) -> float | None:
  if lead_prob < MODEL_TAU_MIN_PROB or len(lead_msg.a) < 2:
    return None
  a0 = float(lead_msg.a[0])
  a1 = float(lead_msg.a[1])
  if a0 > MODEL_TAU_BRAKE_A:
    return None
  if a1 < 0.5 * a0:
    return MODEL_TAU_SUSTAINED
  if a1 > 0.1 * a0:
    return MODEL_TAU_SPURIOUS
  return None


class _ModelPath:
  # 模型規劃路徑（modelV2.position）：模型座標 y 向右為正，取負號轉成 radar 座標（左正）；
  # x 由攝影機起算，以 dRel + RADAR_TO_CAMERA 查表。無效時（低速、資料異常）假設直行。
  def __init__(self, path_x, path_y, v_ego: float):
    self.valid = False
    self.px = self.py = None
    if path_x is not None and path_y is not None and len(path_x) >= 2 and len(path_x) == len(path_y):
      self.px = np.asarray(path_x, dtype=float)
      self.py = np.asarray(path_y, dtype=float)
      if (v_ego > MODEL_PATH_MIN_SPEED and bool(np.all(np.diff(self.px) > 0)) and
          (self.px[-1] - self.px[0]) > MODEL_PATH_MIN_LENGTH):
        self.valid = True

  def y_at(self, d: float) -> float:
    if self.valid:
      return -float(np.interp(d + RADAR_TO_CAMERA, self.px, self.py))
    return 0.0


def _build_lane_ctx(lane_lines, lane_line_probs):
  # 整理 laneLines[1]（本車左線）、[2]（本車右線）與信心度；兩側信心度都足夠時更新濾波車道寬。
  # 資料不完整時回傳 None（大型車一律用本車路徑模式）。
  if lane_lines is None or lane_line_probs is None or len(lane_lines) < 3 or len(lane_line_probs) < 3:
    return None
  ctx = {}
  for key, idx in (('left', 1), ('right', 2)):
    x, y = lane_lines[idx]
    if len(x) < 2 or len(x) != len(y):
      return None
    xa = np.asarray(x, dtype=float)
    if not bool(np.all(np.diff(xa) > 0)):
      return None
    ctx[key] = (xa, np.asarray(y, dtype=float), float(lane_line_probs[idx]))
  l_x, l_y, l_p = ctx['left']
  r_x, r_y, r_p = ctx['right']
  if l_p >= LANE_LINE_MIN_PROB and r_p >= LANE_LINE_MIN_PROB:
    # 模型座標 y 向右為正：車道寬 = 右線 y − 左線 y；明顯錯誤的量測不納入
    w = float(np.interp(LANE_WIDTH_X, r_x, r_y) - np.interp(LANE_WIDTH_X, l_x, l_y))
    if LANE_WIDTH_MIN - 0.5 < w < LANE_WIDTH_MAX + 0.5:
      w = float(np.clip(w, LANE_WIDTH_MIN, LANE_WIDTH_MAX))
      _LANE_WIDTH['w'] += LANE_WIDTH_ALPHA * (w - _LANE_WIDTH['w'])
  return ctx


def _lane_intrusion(track, off: float, lane_ctx) -> tuple[float, str]:
  # 回傳（雷達點越過目標側車道線的距離，模式）；正值 = 已在本車道內。目標側由 off 符號決定。
  right_side = off < 0.0
  if lane_ctx is not None:
    x, y, p = lane_ctx['right'] if right_side else lane_ctx['left']
    if p >= LANE_LINE_MIN_PROB:
      line_y = -float(np.interp(track.dRel + RADAR_TO_CAMERA, x, y))
      intr = (track.yRel - line_y) if right_side else (line_y - track.yRel)
      return intr, 'line'
  return _LANE_WIDTH['w'] / 2.0 - abs(off), 'path'


def _update_large_vehicle_flags(tracks, v_ego: float) -> None:
  # 每幀呼叫一次（leadOne）。以每個移動中的雷達點為中心，找同車點（橫向/速度同 CUTIN_GROUP_*，
  # 縱向 LARGE_VEH_GROUP_DD 內）沿縱向串鏈，鏈長 >= LARGE_VEH_MIN_SPAN 即為大型車。
  #   - 只有移動中的點參與（排除護欄、門架等靜止物串成的鏈）。
  #   - ID 曾判定為大型車即維持到該 ID 消失：誤判需要 ID 穩定追蹤 >= 0.5 秒，綁 ID 足夠。
  #   - 與大型車點為同一物體的重複點同幀繼承（log 033：重複點位在 |off| 2.25~2.6m，最易誤判）。
  v_min = max(3.0, DYNAMIC_SPEED_PCT * v_ego)
  items = list(tracks.values())
  moving = [o for o in items if (o.vRel + v_ego) > v_min]
  for t in items:
    large_now = False
    if (t.vRel + v_ego) > v_min:
      ds = sorted(o.dRel for o in moving
                  if abs(o.dRel - t.dRel) < LARGE_VEH_GROUP_DD and abs(o.yRel - t.yRel) < CUTIN_GROUP_DY and
                  abs(o.vRel - t.vRel) < CUTIN_GROUP_DV)
      i = min(range(len(ds)), key=lambda k: abs(ds[k] - t.dRel))
      lo = hi = i
      while lo > 0 and ds[lo] - ds[lo - 1] <= LARGE_VEH_CHAIN_GAP:
        lo -= 1
      while hi < len(ds) - 1 and ds[hi + 1] - ds[hi] <= LARGE_VEH_CHAIN_GAP:
        hi += 1
      large_now = (ds[hi] - ds[lo]) >= LARGE_VEH_MIN_SPAN
    t.large_hold = LARGE_VEH_HOLD_FRAMES if large_now else max(t.large_hold - 1, 0)
    if large_now:
      t.large_sticky = True
    t.is_large = t.large_hold > 0 or t.large_sticky
  # 重複點繼承：以本幀已判定的大型車點為來源，只傳一層（不做遞移擴散）
  sources = [o for o in items if o.is_large]
  for t in items:
    if not t.is_large and any(_is_same_object(t, o) for o in sources):
      t.is_large = True
      t.large_sticky = True


def _cutin_confirm_frames(v_ego: float) -> int:
  return CUTIN_CONFIRM_FRAMES_LOW if v_ego < CUTIN_LOW_SPEED else CUTIN_CONFIRM_FRAMES


def _cutin_net_shift_ok(cand) -> bool:
  # 過去 1 秒（未濾波偏移）往本車路徑的淨位移 >= CUTIN_NET_SHIFT_MIN。
  # 追蹤不滿 1 秒的新雷達點：至少 CUTIN_NET_SHIFT_MIN_FRAMES 幀，淨位移同樣要夠，且過程中
  # 往外回彈不超過 CUTIN_NET_SHIFT_REVERSAL。
  h = cand.cutin_raw_hist
  n = len(h)
  if n < CUTIN_NET_SHIFT_MIN_FRAMES + 1:
    return False
  now = h[-1]
  direction = -1.0 if now > 0 else 1.0       # 往路徑（偏移絕對值變小）的方向
  if (now - h[0]) * direction < CUTIN_NET_SHIFT_MIN:
    return False
  if n >= CUTIN_NET_SHIFT_FRAMES + 1:
    return True
  best = h[0] * direction
  for x in list(h)[1:]:
    p = x * direction
    if best - p > CUTIN_NET_SHIFT_REVERSAL:
      return False
    best = max(best, p)
  return True


def _cutin_group_consistent(cand, tracks) -> bool:
  # 同車其他反射點也必須朝本車路徑移動；沒有同車點時視為一致
  sibs = [o for o in tracks.values()
          if o is not cand and o.cutin_off is not None and
          abs(o.dRel - cand.dRel) < CUTIN_GROUP_DD and abs(o.yRel - cand.yRel) < CUTIN_GROUP_DY and
          abs(o.vRel - cand.vRel) < CUTIN_GROUP_DV]
  if len(sibs) == 0:
    return True
  direction = -1.0 if cand.cutin_off > 0 else 1.0     # 朝路徑移動的方向
  toward = sorted(o.cutin_vy * direction for o in sibs)
  med = toward[len(toward) // 2]
  return med >= max(CUTIN_GROUP_VY_MIN, CUTIN_GROUP_VY_RATIO * abs(cand.cutin_vy))


# ==============================================================================
# 雷達目標
# ==============================================================================
class TrackDP(Track):
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    # 提早鎖定 / 橫向閘門（依前車槽位分開）
    self.ema_confidence = {0: 0.4, 1: 0.4}
    self.holdover_frames = {0: 0, 1: 0}         # EMA 凍結剩餘幀數
    self.moving_streak = {0: 0, 1: 0}           # 連續「移動目標」幀數（vel_sane 放行）
    self.is_out_of_lane = {0: False, 1: False}
    self.gate_initialized = {0: False, 1: False}
    self.gate_flip_cnt = {0: 0, 1: 0}           # 連續與目前閘門狀態相反的幀數
    self.gate_last_path_y = {0: None, 1: None}  # 上一幀此目標距離處的走廊中心
    # 雷達主導救援
    self.radar_rescue_frames = 0
    # 切入預測
    self.cutin_off = None                       # 濾波後相對本車路徑的橫向偏移（左正）
    self.cutin_vy = 0.0                         # 濾波後橫向速度（m/s，左正）
    self.cutin_raw_hist = deque(maxlen=CUTIN_NET_SHIFT_FRAMES + 1)   # 未濾波偏移歷史
    self.cutin_last_path_y = None               # 上一幀此目標距離處的路徑 y（跳動保護）
    self.cutin_frames = 0                       # 連續符合候選條件的幀數
    self.cutin_active = False                   # 已被採用為切入前車
    self.cutin_active_frames = 0                # 採用後經過的幀數
    self.cutin_entered = False                  # 採用後是否已進入 ±CUTIN_CORRIDOR
    self.cutin_measured_age = 0                 # 距離最近一次真實雷達量測的幀數
    self.cutin_diag_armed = False               # 候選成立診斷 log 只記一次
    # 多點大型車 / 車道線
    self.large_hold = 0
    self.large_sticky = False
    self.is_large = False
    self.cutin_is_large = False                 # 以下三項僅供診斷 log
    self.cutin_line_mode = 'path'               # 'line' = 車道線模式，'path' = 本車路徑模式
    self.cutin_line_intr = None                 # 越過車道線的距離（正值 = 已在本車道內）

  # ---------- 提早鎖定 / 橫向閘門 ----------
  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float,
                          is_turning: bool = False, path_y: float = 0.0):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # 轉彎時要求真實量測（避免旁側車道外推點誤判切入本車道）；直行時放行，避免漏拍拖慢反應。
    is_out = self._update_lane_gate(lead_idx, vision_y, vision_v, v_ego, path_y)
    is_lateral_far = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    is_invalid = (is_turning and not self.measured) or is_lateral_far or is_out

    fuzzy_score = 0.0
    if not is_invalid:
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego, lead_idx)
      is_invalid = fuzzy_score == 0.0

    if is_invalid:
      if self.holdover_frames[lead_idx] > 0:
        self.holdover_frames[lead_idx] -= 1     # 寬限期內 EMA 維持不變
      else:
        self.ema_confidence[lead_idx] = (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
      return

    self.holdover_frames[lead_idx] = RELEASE_FRAMES
    alpha = self._calculate_threat_multipliers(v_ego) if fuzzy_score > 0.5 else ALPHA_DOWN
    new_ema = alpha * fuzzy_score + (1 - alpha) * self.ema_confidence[lead_idx]
    self.ema_confidence[lead_idx] = self._apply_slow_protection(v_ego, lead_prob, new_ema)

  def _update_lane_gate(self, lead_idx: int, vision_y: float, vision_v: float, v_ego: float, path_y: float) -> bool:
    last_path_y = self.gate_last_path_y[lead_idx]
    self.gate_last_path_y[lead_idx] = path_y
    path_unstable = last_path_y is not None and abs(path_y - last_path_y) > GATE_PATH_JUMP_LIMIT
    corridor_half = LANE_CORRIDOR_HALF_WIDTH + float(np.interp(self.dRel, LANE_CORRIDOR_NEAR_BP, LANE_CORRIDOR_NEAR_EXTRA))
    outside_corridor = abs(self.yRel - path_y) > corridor_half
    lateral_mismatch = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    speed_mismatch = abs((self.vRel + v_ego) - vision_v) > max(LANE_GATE_DV_MIN, LANE_GATE_DV_PCT * abs(vision_v))
    raw_out = (not path_unstable) and outside_corridor and (lateral_mismatch or speed_mismatch)

    if not self.gate_initialized[lead_idx]:
      # 新目標：第一幀直接採用當下判定（例如一出現就在路邊的靜止物，立即排除）
      self.is_out_of_lane[lead_idx] = raw_out
      self.gate_initialized[lead_idx] = True
      self.gate_flip_cnt[lead_idx] = 0
    elif raw_out != self.is_out_of_lane[lead_idx]:
      self.gate_flip_cnt[lead_idx] += 1
      if self.gate_flip_cnt[lead_idx] >= GATE_DEBOUNCE_FRAMES:
        self.is_out_of_lane[lead_idx] = raw_out
        self.gate_flip_cnt[lead_idx] = 0
    else:
      self.gate_flip_cnt[lead_idx] = 0
    return self.is_out_of_lane[lead_idx]

  def _moving_target_confirmed(self, lead_idx: int, v_ego: float) -> bool:
    # 原廠 vel_sane 的 (v_ego + vRel > 3)：目標本身在移動。需連續 VEL_SANE_CONFIRM_FRAMES 幀成立。
    if (v_ego + self.vRel) > VEL_SANE_FALLBACK_SPEED:
      self.moving_streak[lead_idx] = min(self.moving_streak[lead_idx] + 1, VEL_SANE_CONFIRM_FRAMES)
    else:
      self.moving_streak[lead_idx] = 0
    return self.moving_streak[lead_idx] >= VEL_SANE_CONFIRM_FRAMES

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float,
                             lead_idx: int) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    d_bounds = [max(FUZZY_BOUNDS[0], FUZZY_D_BOUNDS_PCT[0] * offset_vision_dist),
                max(FUZZY_BOUNDS[1], FUZZY_D_BOUNDS_PCT[1] * offset_vision_dist)]
    score_d = float(np.interp(err_d, d_bounds, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))
    if self._moving_target_confirmed(lead_idx, v_ego):
      score_v = max(score_v, VEL_SANE_FALLBACK_SCORE)
    return score_d * score_y * score_v

  def _calculate_threat_multipliers(self, v_ego: float) -> float:
    brake_mult = float(np.interp(self.aLeadK, BRAKE_THRES_RANGE, MULT_RANGE))
    cutin_mult = 1.0
    if self.dRel < CUTIN_DIST_LIMIT and abs(self.yRel) > 1.0:
      v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)
      cutin_mult = float(np.interp(self.vRel, [-v_limit, v_limit], MULT_RANGE))
    return min(1.0, ALPHA_BASE * brake_mult * cutin_mult)

  def _apply_slow_protection(self, v_ego: float, cam_prob: float, current_ema: float) -> float:
    if abs(self.vRel + v_ego) < max(1.0, DYNAMIC_SPEED_PCT * v_ego):
      if cam_prob < float(np.interp(v_ego, CAM_PROB_SPEED_RANGE, CAM_PROB_RANGE)):
        return min(current_ema, STATIC_EMA_CAP)
    return current_ema

  # ---------- 雷達主導救援 ----------
  def update_radar_rescue(self, v_ego: float, path_valid: bool, path_y: float, steering_angle_deg: float,
                          is_active: bool = False, line_intr: float | None = None) -> None:
    # 每幀呼叫一次（leadOne）。is_active：此目標是上一幀的救援前車，走廊用維持條件。
    # line_intr：目標側車道線可靠時的越線距離（None = 車道線不可靠，不檢查）。
    corridor = RADAR_RESCUE_CORRIDOR_HOLD if is_active else RADAR_RESCUE_CORRIDOR
    line_ok = line_intr is None or line_intr >= (RADAR_RESCUE_LINE_HOLD if is_active else RADAR_RESCUE_LINE_ENTER)
    v_ok = is_active or v_ego > RADAR_RESCUE_MIN_V_EGO
    eligible = (path_valid and v_ok and bool(self.measured) and line_ok and
                RADAR_RESCUE_MIN_DIST < self.dRel < RADAR_RESCUE_MAX_DIST and
                abs(self.yRel - path_y) < corridor and
                (self.vRel + v_ego) > max(RADAR_RESCUE_MIN_SPEED, RADAR_RESCUE_MIN_SPEED_PCT * v_ego) and
                abs(steering_angle_deg) < RADAR_RESCUE_MAX_ANGLE)
    self.radar_rescue_frames = self.radar_rescue_frames + 1 if eligible else 0

  # ---------- 切入預測 ----------
  def update_cutin(self, v_ego: float, path_valid: bool, path_y: float, gated: bool,
                   is_large: bool = False, lane_ctx=None) -> None:
    # 每幀呼叫一次（leadOne）。gated：本車轉彎或變換車道中，暫停判定。
    if bool(self.measured):
      self.cutin_measured_age = 0
    else:
      self.cutin_measured_age = min(self.cutin_measured_age + 1, CUTIN_MEASURED_MAX_AGE + 1)
    fresh = self.cutin_measured_age <= CUTIN_MEASURED_MAX_AGE
    self.cutin_is_large = is_large

    if not path_valid:
      if self.cutin_active or self.cutin_frames > 0:
        cloudlog.debug(
          f"[RadarD_CutinReset_DP] id={self.identifier} reason=path_invalid " +
          f"d={self.dRel:.1f} y={self.yRel:.2f} age={self.cutin_measured_age}"
        )
      self.cutin_off = None
      self.cutin_vy = 0.0
      self.cutin_frames = 0
      self.cutin_active = False
      self.cutin_diag_armed = False
      self.cutin_raw_hist.clear()
      self.cutin_last_path_y = None
      return

    off, vy = self._update_cutin_filter(path_y, is_large)
    line_rule = is_large or CUTIN_LINE_CROSS_ALL     # 車道線規則（v9.6.6 起所有目標）
    intr, mode = _lane_intrusion(self, off, lane_ctx)
    self.cutin_line_intr = intr
    self.cutin_line_mode = mode

    v_abs = self.vRel + v_ego
    max_d = float(np.interp(v_ego, CUTIN_MAX_DIST_BP, CUTIN_MAX_DIST_V))
    base_ok = (fresh and not gated and v_ego > (CUTIN_HOLD_MIN_V_EGO if self.cutin_active else CUTIN_MIN_V_EGO) and
               CUTIN_MIN_DIST < self.dRel < max_d and
               v_abs > max(3.0, DYNAMIC_SPEED_PCT * v_ego) and
               (self.vRel < CUTIN_CLOSING_VREL or self.dRel < CUTIN_CLOSE_HEADWAY * v_ego))

    if self.cutin_active:
      self._update_cutin_active(off, vy, intr, mode, base_ok, fresh, gated, line_rule)
    else:
      self._update_cutin_candidate(off, vy, intr, mode, base_ok, line_rule, v_ego)

  def _update_cutin_filter(self, path_y: float, is_large: bool) -> tuple[float, float]:
    # 相對本車路徑的橫向偏移 alpha-beta 濾波。路徑一幀跳動超過 CUTIN_PATH_JUMP_LIMIT 時偏移基準
    # 已改變：重設濾波與淨橫移歷史（已採用中的目標不因此釋放，vy 歸零後重新估計）。
    path_jump = self.cutin_last_path_y is not None and abs(path_y - self.cutin_last_path_y) > CUTIN_PATH_JUMP_LIMIT
    self.cutin_last_path_y = path_y

    meas = self.yRel - path_y
    if path_jump:
      if self.cutin_active or self.cutin_frames > 0:
        cloudlog.debug(
          f"[RadarD_CutinPathJump_DP] id={self.identifier} d={self.dRel:.1f} y={self.yRel:.2f} " +
          f"offNew={meas:.2f} vyPrev={self.cutin_vy:.2f} active={self.cutin_active} " +
          f"frames={self.cutin_frames} large={is_large}"
        )
      self.cutin_raw_hist.clear()
      self.cutin_off = None
      self.cutin_frames = 0
      self.cutin_diag_armed = False

    self.cutin_raw_hist.append(meas)
    if self.cutin_off is None:
      self.cutin_off = meas
      self.cutin_vy = 0.0
    else:
      pred = self.cutin_off + self.cutin_vy * DT_MDL
      res = meas - pred
      self.cutin_off = pred + CUTIN_ALPHA * res
      self.cutin_vy = self.cutin_vy + (CUTIN_BETA / DT_MDL) * res
    return self.cutin_off, self.cutin_vy

  def _update_cutin_active(self, off: float, vy: float, intr: float, mode: str, base_ok: bool, fresh: bool,
                           gated: bool, line_rule: bool) -> None:
    # 已採用：仍在維持範圍內（車道線規則：仍在車道線內；偏移帶規則：|off| < CUTIN_HOLD_OFF）且未遠離就維持；
    # 採用後 CUTIN_PENDING_MAX_FRAMES 幀內仍未進入走廊（壓線不進來）即停滯釋放。
    if line_rule:
      release_thr = LARGE_LINE_RELEASE if mode == 'line' else LARGE_PATH_RELEASE
      within = intr >= release_thr
    else:
      within = abs(off) < CUTIN_HOLD_OFF
    moving_away = (off * vy > 0.0) and abs(vy) > CUTIN_VY_MIN and abs(off) > CUTIN_CORRIDOR
    self.cutin_active_frames += 1
    if abs(off) <= CUTIN_CORRIDOR:
      self.cutin_entered = True
    stalled = (not self.cutin_entered) and self.cutin_active_frames > CUTIN_PENDING_MAX_FRAMES
    if not stalled and base_ok and within and not moving_away:
      return

    if stalled:
      reason = 'stalled'
    elif not fresh:
      reason = 'stale'
    elif gated:
      reason = 'gated'
    elif moving_away:
      reason = 'moving_away'
    elif not within:
      reason = 'line_out' if line_rule else 'off'
    else:
      reason = 'base_condition'
    cloudlog.debug(
      f"[RadarD_CutinRelease_DP] id={self.identifier} reason={reason} " +
      f"d={self.dRel:.1f} off={off:.2f} vy={vy:.2f} vRel={self.vRel:.2f} " +
      f"aLeadK={self.aLeadK:.2f} aLeadTau={float(self.aLeadTau.x):.2f} age={self.cutin_measured_age} " +
      f"large={self.cutin_is_large} mode={mode} intr={intr:.2f} activeFrames={self.cutin_active_frames}"
    )
    self.cutin_active = False
    self.cutin_frames = 0
    self.cutin_diag_armed = False
    self.cutin_active_frames = 0
    self.cutin_entered = False

  def _update_cutin_candidate(self, off: float, vy: float, intr: float, mode: str, base_ok: bool, line_rule: bool,
                              v_ego: float) -> None:
    # 未採用：預測即將進入走廊（朝路徑移動、橫向速度合理、CUTIN_T_ENTER 內進入）的連續幀數。
    # 車道線規則：須已越過車道線（本車路徑模式多扣餘裕）；偏移帶規則：偏移須 < CUTIN_OFF_MAX。
    self.cutin_active_frames = 0
    self.cutin_entered = False
    if line_rule:
      enter_thr = LARGE_LINE_ENTER if mode == 'line' else LARGE_PATH_ENTER
      lateral_ok = CUTIN_CORRIDOR < abs(off) and intr >= enter_thr
    else:
      lateral_ok = CUTIN_CORRIDOR < abs(off) < CUTIN_OFF_MAX
    toward = off * vy < 0.0
    hit = (base_ok and lateral_ok and toward and
           CUTIN_VY_MIN < abs(vy) < CUTIN_VY_MAX and
           (abs(off) - CUTIN_CORRIDOR) / abs(vy) < CUTIN_T_ENTER)
    self.cutin_frames = self.cutin_frames + 1 if hit else 0

    if self.cutin_frames >= _cutin_confirm_frames(v_ego) and not self.cutin_diag_armed:
      t_enter = (abs(off) - CUTIN_CORRIDOR) / max(abs(vy), 1e-3)
      cloudlog.debug(
        f"[RadarD_CutinArmed_DP] id={self.identifier} d={self.dRel:.1f} y={self.yRel:.2f} " +
        f"off={off:.2f} vy={vy:.2f} tEnter={t_enter:.2f}s vRel={self.vRel:.2f} " +
        f"vLead={self.vLead:.2f} aLeadK={self.aLeadK:.2f} aLeadTau={float(self.aLeadTau.x):.2f} " +
        f"frames={self.cutin_frames} age={self.cutin_measured_age} large={self.cutin_is_large} mode={mode} intr={intr:.2f}"
      )
      self.cutin_diag_armed = True
    elif self.cutin_frames == 0:
      self.cutin_diag_armed = False


# ==============================================================================
# 前車選擇（取代 radard.get_lead）
# ==============================================================================
def _update_lead_one_states(tracks, v_ego: float, ready: bool, path: _ModelPath, steering_angle_deg: float,
                            cutin_gated: bool, lane_lines, lane_line_probs, rescue_prob: float):
  # leadOne 專屬的每幀更新：救援目標換 ID 繼承、大型車判定、救援/切入狀態，回傳救援候選（或 None）。
  cache0 = _LEAD_STATE_CACHE[0]
  active_rescue = cache0['track'] if cache0['rescue'] else None
  # 救援目標的雷達點消失、改用新 ID 回報同一台車時（遠距離常見），新 ID 繼承連續幀數。
  # （切入狀態刻意不跨 ID 繼承：大車反射點交換 ID 時會接續錯誤狀態。）
  if active_rescue is not None and tracks.get(active_rescue.identifier) is not active_rescue:
    heirs = [t for t in tracks.values() if _is_same_object(active_rescue, t)]
    if len(heirs) > 0:
      heir = min(heirs, key=lambda t: abs(t.dRel - active_rescue.dRel) + abs(t.yRel - active_rescue.yRel))
      heir.radar_rescue_frames = max(heir.radar_rescue_frames, active_rescue.radar_rescue_frames)
      cache0['track'] = heir
      active_rescue = heir
    else:
      active_rescue = None

  if len(tracks) > 0:
    _update_large_vehicle_flags(tracks, v_ego)
  lane_ctx = _build_lane_ctx(lane_lines, lane_line_probs)

  for track in tracks.values():
    path_y = path.y_at(track.dRel)
    intr, mode = _lane_intrusion(track, track.yRel - path_y, lane_ctx)
    track.update_radar_rescue(v_ego, path.valid, path_y, steering_angle_deg,
                              is_active=track is active_rescue, line_intr=intr if mode == 'line' else None)
    track.update_cutin(v_ego, path.valid, path.y_at(track.dRel), cutin_gated,
                       is_large=track.is_large, lane_ctx=lane_ctx)

  rescue_candidates = [t for t in tracks.values() if t.radar_rescue_frames >= RADAR_RESCUE_CONFIRM_FRAMES]
  if not (ready and rescue_prob >= RADAR_RESCUE_MIN_PROB and len(rescue_candidates) > 0):
    return None
  rescue_track = min(rescue_candidates, key=lambda t: t.dRel)
  # 上一幀的救援目標仍是候選、且與最近候選為同一物體（重複點）時沿用
  prev_rescue = cache0['track'] if cache0['rescue'] else None
  if (prev_rescue is not None and prev_rescue is not rescue_track and
      any(t is prev_rescue for t in rescue_candidates) and _is_same_object(prev_rescue, rescue_track)):
    rescue_track = prev_rescue
  return rescue_track


def _match_with_stickiness(cache, matched_track, tracks, v_ego: float, lead_idx: int, path: _ModelPath):
  # 重複點沿用與鎖定黏著。回傳（配對結果, 目前鎖定的雷達目標, 鎖定目標是否仍在走廊內）。
  # 新配對結果和目前鎖定目標是同一物體時，沿用目前鎖定目標
  prev_track = cache['track']
  if (matched_track is not None and prev_track is not None and matched_track is not prev_track and
      tracks.get(prev_track.identifier) is prev_track and _is_same_object(prev_track, matched_track)):
    matched_track = prev_track

  locked = cache['track'] if (cache['track'] is not None and not cache['rescue'] and not cache['cutin']) else None
  locked_in_path = (locked is not None and tracks.get(locked.identifier) is locked and
                    (locked.vRel + v_ego) > max(1.0, DYNAMIC_SPEED_PCT * v_ego) and
                    not locked.is_out_of_lane[lead_idx] and
                    abs(locked.yRel - path.y_at(locked.dRel)) <= LANE_CORRIDOR_HALF_WIDTH)

  # 換成更遠的目標需連續確認；換成更近的目標（切入車）立即切換
  if matched_track is not None and locked_in_path and matched_track is not locked and matched_track.dRel > locked.dRel:
    if cache['switch_id'] == matched_track.identifier:
      cache['switch_cnt'] += 1
    else:
      cache['switch_id'] = matched_track.identifier
      cache['switch_cnt'] = 1
    if cache['switch_cnt'] < SWITCH_FARTHER_CONFIRM_FRAMES:
      matched_track = locked
  else:
    cache['switch_id'] = None
    cache['switch_cnt'] = 0
  return matched_track, locked, locked_in_path


def _clear_lead_cache(cache, clear_aleadk: bool) -> None:
  cache['track'] = None
  cache['absent'] = 0
  if clear_aleadk:
    cache['last_aLeadK'] = None
  cache['rescue'] = False
  cache['cutin'] = False


def _set_lead_cache(cache, track, rescue: bool = False, cutin: bool = False) -> None:
  cache['track'] = track
  cache['absent'] = 0
  cache['rescue'] = rescue
  cache['cutin'] = cutin


def _arbitrate_cutin(cache, cutin_prev, tracks, v_ego: float, selected_track, vision_lead):
  # 切入預測仲裁（leadOne）。候選 = 已採用中的切入目標，或連續成立確認幀數、同車一致、淨橫移足夠的
  # 新目標，取最近者；沒有前車、或比現有前車更近且不是同一物體才採用。
  # 回傳（selected_track, vision_lead, is_cutin）。
  is_cutin = False
  cutin_cands = [t for t in tracks.values() if t.cutin_active or
                 (t.cutin_frames >= _cutin_confirm_frames(v_ego) and _cutin_group_consistent(t, tracks) and
                  _cutin_net_shift_ok(t))]
  if len(cutin_cands) > 0:
    cutin_track = min(cutin_cands, key=lambda t: t.dRel)
    # 同一台車的多個反射點都成為候選時，沿用上一幀的前車點
    if (cutin_prev is not None and cutin_prev is not cutin_track and any(t is cutin_prev for t in cutin_cands) and
        _is_same_object(cutin_prev, cutin_track)):
      cutin_track = cutin_prev
    if selected_track is not None:
      cur_d = selected_track.dRel
      same = selected_track is cutin_track or _is_same_object(selected_track, cutin_track)
    elif vision_lead is not None:
      cur_d = vision_lead['dRel']
      same = False
    else:
      cur_d = float('inf')
      same = False

    if same:
      cutin_track.cutin_active = True     # 已由其他路徑選中同一物體：保持狀態，交由原路徑輸出
    elif cutin_track.dRel < cur_d:
      cutin_track.cutin_active = True
      selected_track = cutin_track
      vision_lead = None
      is_cutin = True
      _set_lead_cache(cache, cutin_track, cutin=True)
      cache['cutin_track'] = cutin_track
      if cutin_prev is not cutin_track:
        off = cutin_track.cutin_off if cutin_track.cutin_off is not None else 0.0
        vy = cutin_track.cutin_vy
        t_enter = ((abs(off) - CUTIN_CORRIDOR) / max(abs(vy), 1e-3)) if abs(off) > CUTIN_CORRIDOR else 0.0
        intr = cutin_track.cutin_line_intr if cutin_track.cutin_line_intr is not None else float('nan')
        cloudlog.debug(
          f"[RadarD_CutinSelect_DP] id={cutin_track.identifier} d={cutin_track.dRel:.1f} " +
          f"y={cutin_track.yRel:.2f} off={off:.2f} vy={vy:.2f} tEnter={t_enter:.2f}s " +
          f"vRel={cutin_track.vRel:.2f} vLead={cutin_track.vLead:.2f} " +
          f"aLeadK={cutin_track.aLeadK:.2f} aLeadTau={float(cutin_track.aLeadTau.x):.2f} " +
          f"frames={cutin_track.cutin_frames} age={cutin_track.cutin_measured_age} oldLeadD={cur_d:.1f} " +
          f"large={cutin_track.cutin_is_large} mode={cutin_track.cutin_line_mode} " +
          f"intr={intr:.2f}"
        )
    else:
      cutin_track.cutin_active = False

  # 未被輸出（也不是輸出前車的重複點）的切入目標一律取消採用
  for t in tracks.values():
    if t.cutin_active and t is not selected_track and not (selected_track is not None and _is_same_object(selected_track, t)):
      t.cutin_active = False
  return selected_track, vision_lead, is_cutin


def _build_lead_dict(cache, selected_track, lead_msg, lead_prob: float, is_cutin: bool, is_radar_rescue: bool,
                     lead_idx: int, current_prob_thres: float) -> dict[str, Any]:
  # 雷達前車輸出：切入前車 modelProb 填 0（視覺機率描述的是另一台車，也避免觸發 FCW）。
  lead_dict = selected_track.get_RadarState(0.0 if is_cutin else lead_prob)

  # aLeadK 變化率限制（凍結續命中、或剛恢復配對跳到最新卡曼值時，避免被誤判成前車急減速）。
  # 只限制 aLeadK；換成不同物體（例如切入車取代原前車）時不限制，直接採用新目標的 aLeadK。
  last_ref = cache['last_aLeadK_track']
  if last_ref is not None and last_ref is not selected_track and not _is_same_object(last_ref, selected_track):
    cache['last_aLeadK'] = None
  if cache['last_aLeadK'] is not None:
    delta = float(np.clip(lead_dict['aLeadK'] - cache['last_aLeadK'], -MAX_ALEADK_DELTA_PER_FRAME, MAX_ALEADK_DELTA_PER_FRAME))
    lead_dict['aLeadK'] = float(cache['last_aLeadK'] + delta)
  cache['last_aLeadK'] = float(lead_dict['aLeadK'])
  cache['last_aLeadK_track'] = selected_track

  # 視覺加速度雙重驗證阻尼（切入／救援前車可能不是視覺前車，不套用）
  model_tau = get_model_lead_tau(lead_msg, lead_prob) if not (is_cutin or is_radar_rescue) else None
  if model_tau is not None:
    lead_dict['aLeadTau'] = model_tau

  if not is_radar_rescue and current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
    cloudlog.debug(
      f"[RadarD_EarlyLock_DP] 提早鎖定/續命成功！目標 {lead_idx} | " +
      f"相機機率: {lead_prob:.2f} (動態門檻: {current_prob_thres:.2f})"
    )
  return lead_dict


def _apply_low_speed_override(lead_dict: dict[str, Any], tracks, v_ego: float) -> dict[str, Any]:
  # 原廠低速覆寫（v_ego < 4 m/s，正前方 25m 內的雷達點，不需視覺確認）。
  # 刻意分歧：上一幀選到的目標與最近目標為同一物體時沿用；現有雷達前車與最近目標為同一物體時不替換。
  low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
  low_speed_used = None
  if len(low_speed_tracks) > 0:
    closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
    last_ls = _LOW_SPEED_LAST['track']
    if (last_ls is not None and last_ls is not closest_track and
        any(c is last_ls for c in low_speed_tracks) and _is_same_object(last_ls, closest_track)):
      closest_track = last_ls
    cur_track = tracks.get(lead_dict['radarTrackId']) if (lead_dict['status'] and lead_dict.get('radar')) else None
    same_as_current = cur_track is not None and _is_same_object(cur_track, closest_track)
    if not same_as_current and ((not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel'])):
      lead_dict = closest_track.get_RadarState()
      low_speed_used = closest_track
  _LOW_SPEED_LAST['track'] = low_speed_used
  return lead_dict


def get_lead_ext(
  v_ego: float,
  ready: bool,
  tracks: dict[int, TrackDP],
  lead_msg: capnp._DynamicStructReader,
  model_v_ego: float,
  lead_prob: float,
  is_turning: bool = False,
  steering_angle_deg: float = 0.0,
  low_speed_override: bool = True,
  raw_lead_prob: float | None = None,
  lane_change: bool = False,
  path_x: list[float] | None = None,
  path_y: list[float] | None = None,
  lane_lines: list | None = None,
  lane_line_probs: list[float] | None = None,
) -> dict[str, Any]:
  """
  取代 radard.get_lead。前 6 個參數與原廠相同；radard.py 以關鍵字參數傳入以下擴充參數：
    is_turning          方向盤角度/角速度超過門檻：要求真實量測、暫停切入預測
    steering_angle_deg  雷達主導救援的方向盤角度條件
    raw_lead_prob       未濾波的視覺機率（救援只看原始值）；未提供時用濾波後的 lead_prob
    lane_change         本車變換車道中（模型 laneChangeState 非 off）：暫停切入預測
    path_x / path_y     modelV2.position：橫向閘門、鎖定黏著、救援與切入預測的走廊
    lane_lines / lane_line_probs  modelV2.laneLines 的 (x, y) 與信心度：多點大型車的車道線判斷
  low_speed_override=True 表示 leadOne（救援、切入預測與低速覆寫只處理 leadOne）。
  """
  lead_idx = 0 if low_speed_override else 1
  cache = _LEAD_STATE_CACHE[lead_idx]
  path = _ModelPath(path_x, path_y, v_ego)

  # 1. 提早鎖定：更新每個雷達目標的 EMA 信心度與橫向閘門，決定視覺機率門檻
  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob, is_turning, path_y=path.y_at(track.dRel))
  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane[lead_idx] and v.ema_confidence[lead_idx] > 0.0}
  max_ema_confidence = max((t.ema_confidence[lead_idx] for t in valid_tracks.values()), default=0.0)
  normal_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))
  current_prob_thres = normal_thres   # 救援生效時同步降到 RADAR_RESCUE_MIN_PROB（僅影響 EarlyLock 診斷）

  # 2. leadOne：救援 / 切入 / 大型車狀態
  rescue_prob = lead_prob if raw_lead_prob is None else raw_lead_prob
  rescue_track = None
  if lead_idx == 0:
    rescue_track = _update_lead_one_states(tracks, v_ego, ready, path, steering_angle_deg, is_turning or lane_change,
                                           lane_lines, lane_line_probs, rescue_prob)

  # 3. 視覺配對 + 重複點沿用 + 鎖定黏著
  matched_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    matched_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)
  matched_track, locked, locked_in_path = _match_with_stickiness(cache, matched_track, tracks, v_ego, lead_idx, path)

  # 4. 純視覺後備候選（門檻維持 normal_thres，不跟著救援降到 0.1）與續命
  vision_cand = None
  if matched_track is None and ready and lead_prob > normal_thres:
    vision_cand = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)
  hold_limit = SELECT_HOLDOVER_FRAMES
  if locked_in_path and (vision_cand is None or vision_cand['dRel'] >= locked.dRel):
    hold_limit = STICKY_HOLD_FRAMES
  # 續命例外（v9.6.7）：視覺前車明顯較近且機率高時不續命
  if (matched_track is None and vision_cand is not None and cache['track'] is not None and
      lead_prob >= HOLD_BYPASS_MIN_PROB and vision_cand['dRel'] < cache['track'].dRel - HOLD_BYPASS_DD):
    hold_limit = 0
    cloudlog.debug(
      f"[RadarD_HoldBypass_DP] 目標 {lead_idx} | 雷達前車 {cache['track'].identifier} d={cache['track'].dRel:.1f} → " +
      f"視覺前車 d={vision_cand['dRel']:.1f} 機率 {lead_prob:.2f}"
    )
  held_track = None
  if matched_track is None and cache['track'] is not None and cache['absent'] + 1 <= hold_limit:
    held_track = cache['track']
  held_is_rescue = held_track is not None and (cache['rescue'] or cache['cutin'])   # 救援/切入前車不續命

  # 5. 雷達主導救援採用：沒有前車，或比現有前車更近且不是同一物體
  if matched_track is not None:
    existing_d = matched_track.dRel
  elif held_track is not None and not held_is_rescue:
    existing_d = held_track.dRel
  elif vision_cand is not None:
    existing_d = vision_cand['dRel']
  else:
    existing_d = float('inf')
  existing_track = matched_track if matched_track is not None else (held_track if not held_is_rescue else None)
  is_radar_rescue = (rescue_track is not None and rescue_track is not matched_track and
                     rescue_track.dRel < existing_d and
                     not (existing_track is not None and _is_same_object(existing_track, rescue_track)))

  # 6. 決定前車（救援 > 配對 > 續命 > 純視覺）
  vision_lead = None
  if is_radar_rescue:
    selected_track = rescue_track
    _set_lead_cache(cache, rescue_track, rescue=True)
    current_prob_thres = min(current_prob_thres, RADAR_RESCUE_MIN_PROB)
    cloudlog.debug(
      f"[RadarD_RadarRescue_DP] 雷達主導救援！目標 {lead_idx} | 雷達 {rescue_track.identifier} " +
      f"d={rescue_track.dRel:.1f} y={rescue_track.yRel:.2f} v={rescue_track.vRel + v_ego:.1f} | " +
      f"連續 {rescue_track.radar_rescue_frames} 幀 | 相機機率(原始/濾波): {rescue_prob:.2f}/{lead_prob:.2f} " +
      f"(原門檻: {normal_thres:.2f} → {current_prob_thres:.2f}) | 原前車距離: {existing_d:.1f}"
    )
  elif matched_track is not None:
    selected_track = matched_track
    _set_lead_cache(cache, matched_track)
  else:
    selected_track = None
    if held_is_rescue:
      _clear_lead_cache(cache, clear_aleadk=True)      # 救援/切入前車條件不成立當幀撤銷
    elif cache['track'] is not None:
      cache['absent'] += 1
      if cache['absent'] <= hold_limit:
        selected_track = cache['track']                # 續命：沿用上一刻的前車物件
      else:
        _clear_lead_cache(cache, clear_aleadk=True)    # 前車真正消失，重置 aLeadK 基準
    if selected_track is None:
      vision_lead = vision_cand

  # 7. 切入預測仲裁（leadOne）
  is_cutin = False
  cutin_prev = cache['cutin_track']
  cache['cutin_track'] = None
  if lead_idx == 0 and ready:
    selected_track, vision_lead, is_cutin = _arbitrate_cutin(cache, cutin_prev, tracks, v_ego, selected_track, vision_lead)

  # 8. 輸出
  lead_dict = {'status': False}
  if selected_track is not None:
    lead_dict = _build_lead_dict(cache, selected_track, lead_msg, lead_prob, is_cutin, is_radar_rescue,
                                 lead_idx, current_prob_thres)
  elif vision_lead is not None:
    lead_dict = vision_lead
    cache['last_aLeadK'] = None   # 純視覺後備不經過雷達物件，重置 aLeadK 基準

  if low_speed_override:
    lead_dict = _apply_low_speed_override(lead_dict, tracks, v_ego)
  return lead_dict


# ==============================================================================
# Monkey patch 與 RadarD 擴充
# ==============================================================================
radard.Track = TrackDP
radard.get_lead = get_lead_ext


class RadarDExt(RadarD):
  """dragonpilot 版 RadarD：介面與 RadarD(delay) 相同，Track / get_lead 已由上方替換。"""
