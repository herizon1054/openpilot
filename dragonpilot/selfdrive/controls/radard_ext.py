#!/usr/bin/env python3
import capnp
import numpy as np
from collections import deque
from typing import Any
from cereal import messaging, car

# ==============================================================================
# 1. 引入整個 radard 模組進行 Monkey Patch (動態替換)
# ==============================================================================
from openpilot.selfdrive.controls import radard

# 2. 正常引入我們需要的元件與原始函數 (移除 DP 中不存在的 structs 與客製函數)
from openpilot.selfdrive.controls.radard import (
    KalmanParams, Track, RadarD, match_vision_to_track,
    get_RadarState_from_vision, RADAR_TO_CAMERA
)

# 3. 引入 cloudlog 用於記錄我們自訂的提早鎖定事件
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL

# ==============================================================================
# 提早鎖定 (Early Lock) 擴充模組參數設定
# ==============================================================================
LANE_WIDTH_FALLBACK = 1.5           # 預測車道基準單側半寬 (m)
LANE_HYSTERESIS_MARGIN = 0.5        # 邊界外的遲滯容錯預度 (m)
FUZZY_BOUNDS = [0.5, 1.5]           # 物理誤差 (m 或 m/s): 0.5 以內給滿分 1.0，大於 1.5 總分歸零

ALPHA_BASE = 0.2                    # 常規上升學習率
ALPHA_DOWN = 0.1                    # 常規下降與短路過濾時的衰減學習率

BRAKE_THRES_RANGE = [-3.0, -1.2]    # 急煞觸發區間 (m/s²)
MULT_RANGE = [1.2, 1.0]             # 對應威脅倍率
CUTIN_DIST_LIMIT = 40.0             # 評估切入威脅的最大縱向有效距離 (m)
DYNAMIC_SPEED_PCT = 0.2             # 動態相對速度閥值比例

CAM_PROB_SPEED_RANGE = [10.0, 25.0] # 動態相機門檻車速區間
CAM_PROB_RANGE = [0.5, 0.3]         # 動態相機審查門檻
STATIC_EMA_CAP = 0.6                # 目標未達審查門檻時的 EMA 天花板

EMA_VAL_RANGE = [0.4, 0.8]          # 本地 EMA 信心度 X 軸
PROB_THRES_RANGE = [0.5, 0.3]       # 映射出對應的「視覺提早放行門檻」 Y 軸

RELEASE_FRAMES = 5                  # 目標短暫丟失或出界時的 EMA 續命凍結幀數
SELECT_HOLDOVER_FRAMES = 3          # 雷達硬體斷流時，強制維持上一幀鎖定的幀數

# dp(切入閃爍修正): 鎖定黏著。視覺前車在兩台車之間跳動時（切入/切出、前方有兩台車），
# 原廠配對的 dist_sane（誤差 < max(25%, 5m)）時過時不過，前車在雷達目標、純視覺、另一台
# 雷達目標之間來回切換。
#   以下兩項只套用在「移動中」的鎖定目標（絕對速度 > max(1.0, DYNAMIC_SPEED_PCT * v_ego)），
#   靜止目標維持原本行為，避免把路邊/路口的靜止物黏住更久。
#   (1) 已鎖定的雷達目標仍在雷達清單中、仍在本車走廊內、且沒有更近的純視覺前車時，
#       配對失敗可續命 STICKY_HOLD_FRAMES 幀（原本 3 幀）。
#   (2) 配對結果換成「更遠」的另一個目標，而原鎖定目標仍在走廊內時，需連續
#       SWITCH_FARTHER_CONFIRM_FRAMES 幀都選到新目標才切換。換成「更近」的目標
#       （切入車）一律立即切換，不延遲。
STICKY_HOLD_FRAMES = 10             # 0.5 秒
SWITCH_FARTHER_CONFIRM_FRAMES = 5   # 0.25 秒

# dp(重複雷達點閃爍修正): Toyota TSS2 雷達常把同一台車回報成兩個 trackId（例如 log 92462
# 的 17685/18307），兩點距離差中位數 0.02m、橫向差 0.08m、速度差 0.05 m/s。原廠
# match_vision_to_track() 取機率最大者，兩點幾乎一樣，每幀依微小差異互換（92462 車上
# 實際 60 秒內換了 279 次），畫面上的鎖定點一閃一閃。這是原廠行為（原廠 radard 已不做
# 雷達點叢集），不是先前修改造成的。
# 修正：新選到的目標若和目前鎖定目標「是同一個物體」（三項差距都在下列範圍內），
# 沿用目前鎖定目標。log 中重複點的最大差距：距離 1.13m、橫向 0.60m、速度 0.90 m/s。
SAME_OBJ_MAX_DD = 1.5               # m
SAME_OBJ_MAX_DY = 1.0               # m
SAME_OBJ_MAX_DV = 1.5               # m/s

MODEL_TAU_MIN_PROB = 0.5            # 啟動驗證的最低視覺機率
MODEL_TAU_BRAKE_A = -0.5            # 啟動驗證的最低急煞門檻 (m/s²)
MODEL_TAU_SUSTAINED = 0.5           # 視覺確認急煞持續
MODEL_TAU_SPURIOUS = 3.0            # 視覺預測即將加速

# dp: 比照原廠 match_vision_to_track() 的 vel_sane 擇一寬鬆備援：
#   vel_sane = (誤差 < 10) OR (v_ego + vRel > 3)
# 注意：v_ego + vRel 是目標的「絕對速度」，不是接近速度。原廠的意思是「目標本身在移動
# （> 10.8 km/h）就不是靜止雜訊，速度比對直接放行」。此處行為與原廠相同（連續 3 幀
# 確認後 score_v 視為滿分），註解已更正；變數名沿用舊名以維持相容。
VEL_SANE_FALLBACK_SPEED = 3.0       # 原廠門檻：目標絕對速度 (v_ego + vRel) 超過此值視為移動目標
VEL_SANE_CONFIRM_FRAMES = 3         # 需連續幾幀都符合才真正啟用寬鬆備援
VEL_SANE_FALLBACK_SCORE = 1.0       # 啟用後 score_v 的下限，1.0 = 完全比照原廠「視為合理」的語意

# dp: 雷達主導救援（取代原本的兩段式持續性救援）
# 原本的救援用 valid_streak_frames（雷達與視覺吻合的連續幀數）當證據，並把門檻降到 0.4/0.3，
# 但持續計數本身依賴視覺位置吻合，視覺一不可靠就先中斷。10 段 log 重播中，因救援而
# 改變的前車選擇累計 0 幀。改為以「雷達本身的穩定性」當證據：
#   一個雷達目標連續 RADAR_RESCUE_CONFIRM_FRAMES 幀同時滿足下列條件，即成為救援候選：
#     - 真實量測（measured）
#     - 向前移動：絕對速度 > max(RADAR_RESCUE_MIN_SPEED, RADAR_RESCUE_MIN_SPEED_PCT * v_ego)，
#       排除靜止物（路邊違停、護欄、ETC 門架）與對向來車
#     - 位在模型路徑走廊內：|yRel - 模型路徑y(d)| < RADAR_RESCUE_CORRIDOR
#     - 距離在 [RADAR_RESCUE_MIN_DIST, RADAR_RESCUE_MAX_DIST]
#     - 方向盤角度 < RADAR_RESCUE_MAX_ANGLE（路口轉彎時關閉）
#     - 模型路徑有效（車速 > MODEL_PATH_MIN_SPEED）
#   任一幀不符合即歸零重新計數。
# 採用條件（僅 leadOne）：
#     - 視覺信心度 >= RADAR_RESCUE_MIN_PROB（總視覺信心度門檻下限），以「未濾波」的原始
#       leadsV3[0].prob 判斷（最嚴格版本）。radard.py 的 lead_prob 經非對稱濾波（上升瞬間到位、
#       下降每幀 alpha=0.2），濾波值恆 >= 原始值，單幀尖峰會被撐住約 5~6 幀；改用原始值後，
#       原始機率一低於 0.1 當幀就停止採用。
#     - 目前沒有前車，或救援候選比現有前車更近
# 10 段 log（走廊 ±1.0m）：純雷達規則觸發時，未曾落在車道線外 0.5m 以上的目標；
# 92104 14.5s（88m 外 21 km/h 慢車，視覺 0.2~0.3）與 92102 33.6s（92m 外 16 km/h，
# 視覺 0.14）為原邏輯遺漏、此規則可補回的本車道慢車。
RADAR_RESCUE_MIN_PROB = 0.1         # 總視覺信心度門檻下限
RADAR_RESCUE_CORRIDOR = 1.0         # 模型路徑左右各 1.0m（進入條件）
# dp(第八版): 進入與維持分開。已在救援中的目標，走廊放寬為 ±1.5m 才算出界，避免遠距離
# 雷達橫向雜訊在 1.0m 邊緣（log：1.05m、1.12m）造成救援「鎖一下就放掉」。
# 進入仍需在 ±1.0m 內連續 1 秒；其他條件（原始視覺機率、移動、量測、距離、角度）不放寬。
RADAR_RESCUE_CORRIDOR_HOLD = 1.5    # 模型路徑左右各 1.5m（維持條件）
RADAR_RESCUE_CONFIRM_FRAMES = int(1.0 / DT_MDL)   # 連續 1 秒
RADAR_RESCUE_MIN_DIST = 5.0
RADAR_RESCUE_MAX_DIST = 120.0
RADAR_RESCUE_MIN_SPEED = 3.0        # m/s
RADAR_RESCUE_MIN_SPEED_PCT = 0.2
RADAR_RESCUE_MAX_ANGLE = 25.0       # deg

# dp(log 驗證修正 1b-v2): 橫向排除必須同時滿足（第五版起加上去抖動，見 GATE_DEBOUNCE_FRAMES）：
#   (a) 落在本車預測路徑走廊外：|yRel - 路徑y(d)| > LANE_CORRIDOR_HALF_WIDTH，且
#   (b) 與視覺前車不一致：橫向差 > LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN (2.0m)，
#       或絕對速度差 > max(LANE_GATE_DV_MIN, LANE_GATE_DV_PCT * |視覺前車速度|)。
# 7 段 log 統計（同一物體的雷達/視覺配對 344 組）：雷達與視覺的橫向差在 20~40m 有 20% 超過
# 2.0m，所以不能只看視覺 y；舊版 1.5m 回歸門檻會把真前車永久鎖在出界狀態（92102 43.4s 起
# 連續 2.8 秒、92104 26.8s 起停等中）。走廊內的目標一律不因視覺 y 不一致而排除。
LANE_CORRIDOR_HALF_WIDTH = 1.75     # 半個車道寬 (m)
# 近距離雷達橫向偏差補償：Toyota 雷達在近距離常打到車尾角落，log 實測 <10m 時雷達與視覺
# 橫向差中位數約 1.0m（92104 26.8s 停等時，正前方 6m 的真前車 R1809 雷達 y≈-2.4m）。
# 距離 <= 8m 走廊半寬加 1.0m，15m 以上不加，中間線性內插。
LANE_CORRIDOR_NEAR_BP = [8.0, 15.0]
LANE_CORRIDOR_NEAR_EXTRA = [1.0, 0.0]
LANE_GATE_DV_MIN = 2.0              # m/s
# dp(切入閃爍修正): 橫向閘門去抖動。旁車切入時，目標橫向正好跨越走廊邊界、視覺 y 追趕落後，
# 無狀態閘門每幀翻轉，前車在「雷達目標 ↔ 純視覺」之間一閃一閃（log 92102 45.2s：閘門
# 在 5 幀內翻轉 4 次，連續 4 幀出界後超過續命 3 幀，前車跳成純視覺）。
# 改為：新目標第一幀直接採用當下判定；之後判定需連續 GATE_DEBOUNCE_FRAMES 幀相反才切換狀態。
# 進出使用同一組條件（不是 1.5m/2.0m 的不對稱遲滯），不會再發生「永久鎖在出界」的問題。
GATE_DEBOUNCE_FRAMES = 3
# 模型路徑不穩定保護：同一目標距離處的路徑 y 與上一幀相差超過此值（路口轉彎時模型在
# 直行/轉彎之間跳動，log 92102 45.2s 在 24m 處一幀內從 -21m 跳到 -35m 再到 -12m），
# 該幀無法判斷走廊，視為「未出界」（比照原廠不做橫向排除），並照常累計去抖動。
GATE_PATH_JUMP_LIMIT = 1.0          # m
LANE_GATE_DV_PCT = 0.25

# dp(第九版 / 9.1 穩定性補強): 雷達橫向速度切入預測（cut-in prediction）
# v9.6.2：大型車身分補強（依 7 段實車 log）——(a) 同一 ID 曾判定為大型車即維持到該 ID 消失；
#         (b) 與大型車點為同一物體的重複點（Toyota 雙 trackId）繼承大型車身分；
#         (c) 只有移動中的點參與大型車判定（排除護欄、門架等靜止物串成的鏈）。
# v9.6：聯結車在旁邊車道並行時的誤觸發防護——多點大型車改以模型車道線判斷（雷達點須越過車道線
#       才算候選、退回線外即釋放），車道線信心度 < 0.5 時退回本車路徑 + 濾波車道寬；
#       加入模型路徑跳動保護。小車（非多點大型車）維持 v9.4 判斷。詳見 v9.6 參數區說明。
# v9.4：在 v931 基礎上加入「1 秒淨橫移」條件（CUTIN_NET_SHIFT_*），擋單一反射點大型車的橫向抖動。
# v9.3（無切入跨 ID 繼承版）：保留多點一致性、停滯釋放與切入偵測；新 ID 重新確認。
# v9.2：啟用車速下限 36 → 20 km/h；量測新鮮度與切入診斷保留，跨 ID 繼承已移除
# （36 km/h 以下確認幀數 6）；radard.py 的換道判斷不再包含方向燈。
# 背景（log 032/033，約 105 km/h）：藍色小車從右車道切入，雷達 57.5s 已看到它壓線、以約
# 1 m/s 橫向速度切入，但模型 leadsV3[0] 仍指向 57m 外的遠車（prob 1.0），換 lead 時距離又
# 嚴重高估，原廠配對與 fuzzy 都判定失敗，直到 59.4s 才鎖到藍車，駕駛 57.89s 已踩煞車。
# 雷達主導救援（走廊 ±1.0m 內連續 1 秒）也要 59.25s 才進入走廊，救不到。
# 做法：對每個雷達目標以 alpha-beta 濾波估計「相對本車路徑的橫向偏移 off 與橫向速度 vy」，
# 預測即將進入走廊、且縱向正在接近時，連續成立 CUTIN_CONFIRM_FRAMES 幀即成為切入候選；
# 與救援共用仲裁：沒有前車、或比現有前車更近（且不是同一物體）才採用。不依賴視覺機率。
CUTIN_MIN_V_EGO = 20.0 / 3.6        # m/s（v9.2：20 km/h 以下不啟用，原為 36 km/h）
CUTIN_MIN_DIST = 3.0                # m
CUTIN_MAX_DIST_BP = [10.0, 30.0]    # 本車速 m/s
CUTIN_MAX_DIST_V = [20.0, 50.0]     # 偵測距離上限 m（約 1.6~2 秒車距）
CUTIN_CORRIDOR = 1.0                # 預測要進入的走廊半寬 m（與雷達主導救援進入條件相同）
CUTIN_OFF_MAX = 2.5                 # 候選偏移上限：約為目標車身內緣壓到車道線（半車道 1.75m + 半車寬 0.85m）
                                    # log：設 4.0 時 92431 36.3s、032/033 109.7s 各有 1 幀誤採用（大車反射點橫向跳動，
                                    # 偏移 2.7~3.5m），設 2.5 後消失，藍車仍在 57.65s 採用
CUTIN_VY_MIN = 0.3                  # 橫向速度下限 m/s（擋雜訊）
CUTIN_VY_MAX = 2.5                  # 橫向速度上限 m/s（擋大車多反射點跳動）
CUTIN_T_ENTER = 1.5                 # 預測在此秒數內進入走廊
CUTIN_CLOSING_VREL = -0.5           # 縱向接近（vRel < 此值）……
CUTIN_CLOSE_HEADWAY = 1.0           # ……或距離 < 本車速 × 此秒數（近距離同速切入）
CUTIN_CONFIRM_FRAMES = 4            # 連續成立幀數（0.2 秒）
# v9.2: 低速區間（20~36 km/h）機車多、雷達反射點小且橫向跳動大，確認幀數提高為 6 幀（0.3 秒）
CUTIN_LOW_SPEED = 36.0 / 3.6        # m/s
CUTIN_CONFIRM_FRAMES_LOW = 6


def _cutin_confirm_frames(v_ego: float) -> int:
  return CUTIN_CONFIRM_FRAMES_LOW if v_ego < CUTIN_LOW_SPEED else CUTIN_CONFIRM_FRAMES


def _cutin_net_shift_ok(cand) -> bool:
  # dp(v9.4): 過去 1 秒（未濾波偏移）往本車路徑的淨位移 >= CUTIN_NET_SHIFT_MIN
  # 追蹤不滿 1 秒的新雷達點（例如超車後立刻切入）：至少要有 CUTIN_NET_SHIFT_MIN_FRAMES 幀，
  # 淨位移同樣要 >= CUTIN_NET_SHIFT_MIN，且過程中不能有超過 CUTIN_NET_SHIFT_REVERSAL 的往外回彈
  # （抖動的特徵是一下往內一下往外）。
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
  # dp(v9.3): 同車其他反射點也必須朝本車路徑移動；沒有同車點時視為一致
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
CUTIN_HOLD_OFF = 2.5                # 已採用後，偏移 < 此值且未遠離就維持
CUTIN_ALPHA = 0.3                   # alpha-beta 濾波：位置增益
CUTIN_BETA = 0.05                   # alpha-beta 濾波：速度增益

# dp(v9.3): 大型車（聯結車、貨櫃車）壓線誤觸發防護
# 背景：大型車有多個雷達反射點，最強反射點會沿著車身「滑動」（接近時從車尾角落滑到側面），
# 單一點看起來像以 1~2 m/s 往本車道移動；同一台車的其他反射點卻沒有橫向移動。
# log 032/033 109.5s 聯結車：R13510 濾波橫向速度 1.8 m/s，同車 R13604/R13618 只有 −0.4~+0.1 m/s；
# 藍車真切入時兩個反射點（13304/13413）同時以 0.8~1.3 m/s 移動。
# (1) 同車一致性：新候選若有「同車其他反射點」，這些點也必須朝本車路徑移動（中位數 ≥ 候選的
#     CUTIN_GROUP_VY_RATIO 倍且 ≥ CUTIN_GROUP_VY_MIN），否則不採用。沒有同車點（一般小車單點）不受影響。
# (2) 停滯釋放：採用後若 CUTIN_PENDING_MAX_FRAMES 幀內仍未進入 ±CUTIN_CORRIDOR 走廊（壓線不進來），
#     就釋放，避免跟著一台壓線行駛的大車一直煞車。log 中真切入在採用後 1.1~1.5 秒進入走廊。
CUTIN_GROUP_DD = 15.0               # 同車反射點：縱向距離差上限 m（涵蓋聯結車車身長度）
CUTIN_GROUP_DY = 1.5                # 同車反射點：橫向距離差上限 m
CUTIN_GROUP_DV = 1.0                # 同車反射點：相對速度差上限 m/s
CUTIN_GROUP_VY_RATIO = 0.4
CUTIN_GROUP_VY_MIN = 0.15           # m/s
CUTIN_PENDING_MAX_FRAMES = 50       # 2.5 秒（CUTIN_T_ENTER 1.5 秒 + 1 秒餘裕）

# dp(v9.4): 1 秒淨橫移條件——擋「單一反射點」大型車的橫向抖動
# 背景（log 035，右車道大型廂型車 KEN-3302）：雷達只有一個反射點 R12241，橫向速度有 157 幀
# 看起來往內（最高 2.0 m/s），是反射點在車身上跳動；同車一致性沒有其他點可比對而失效。
# 把它往車道線平移 0.8~1.0 m 模擬壓線時，v9.2 / v931 都會誤接手 0.25~0.4 秒。
# 真切入是一路往內移動，抖動是一下往內一下往外，1 秒淨位移接近 0：
#   藍車 0.95 m、R14009 1.67 m；大型車抖動 −0.06 m、0.10 m。
# 新候選必須「過去 CUTIN_NET_SHIFT_FRAMES 幀（未濾波偏移）往內淨位移 ≥ CUTIN_NET_SHIFT_MIN」。
# 追蹤不滿 1 秒的新雷達點：至少觀察 0.5 秒，且淨位移 ≥ 0.5 m、過程中無往外回彈（見 _cutin_net_shift_ok）。
CUTIN_NET_SHIFT_FRAMES = 20         # 1.0 秒
CUTIN_NET_SHIFT_MIN = 0.5           # m
CUTIN_NET_SHIFT_MIN_FRAMES = 10     # 新雷達點至少觀察 0.5 秒
CUTIN_NET_SHIFT_REVERSAL = 0.15     # m，新雷達點觀察期間允許的往外回彈上限

# ==============================================================================
# dp(v9.6): 聯結車在旁邊車道並行時的切入誤觸發防護
# ==============================================================================
# 問題：聯結車在旁邊車道並行時，雷達在車身側面打出多個反射點；側面離本車路徑約 2.25m
#   （3.5m 車道中心 − 半車寬 1.25m），長時間落在小車設計的候選帶 1.0 < |off| < 2.5 內。
#   反射點沿車身滑動、或模型路徑漂移，都可能讓它看起來在往內移而被採用；採用後側面點一直在
#   CUTIN_HOLD_OFF 2.5 內，要等 2.5 秒停滯釋放，期間以近距離前車重煞。
# 做法（只套用在「多點大型車」，小車維持 v9.4）：
#   (1) 大型車判定（_update_large_vehicle_flags）：同車條件（橫向差 < CUTIN_GROUP_DY、速度差 <
#       CUTIN_GROUP_DV、縱向 LARGE_VEH_GROUP_DD 內）的點沿縱向串成鏈（相鄰間距 <= LARGE_VEH_CHAIN_GAP），
#       鏈長 >= LARGE_VEH_MIN_SPAN 即為大型車；1 秒遲滯。
#       限制：從後方遠距離只看得到車尾點（鏈長約 0）時判定不到，該情況維持 v9.4 行為。
#   (2) 車道線判斷：以目標所在側的模型車道線（右側 laneLines[2]、左側 laneLines[1]）計算
#       「雷達點越過車道線的距離」intr（正值 = 已在本車道內）。
#       候選：intr >= LARGE_LINE_ENTER（0：雷達點已越過車道線）。
#       維持：intr < LARGE_LINE_RELEASE（退回線外 0.1m）即釋放，不等停滯釋放。
#   (3) 車道線信心度 < LANE_LINE_MIN_PROB（0.5）時退回本車路徑：假設車道線在
#       路徑 ± 濾波車道寬/2。本車路徑不等於車道中心（本車在車道內的偏移未知），
#       候選多扣 LARGE_PATH_ENTER（0.3m）餘裕；釋放在 intr < LARGE_PATH_RELEASE（0：退回推估車道線外）。
#   (4) 濾波車道寬：兩側車道線信心度都 >= 0.5 時，取本車前方 LANE_WIDTH_X 處的 L1−R1，
#       一階濾波、限制在 [LANE_WIDTH_MIN, LANE_WIDTH_MAX]；尚無量測時用 LANE_WIDTH_DEFAULT。
#   (5) 路徑跳動保護（所有目標）：同一目標距離處的路徑 y 一幀變化 > CUTIN_PATH_JUMP_LIMIT 時，
#       重設偏移濾波與淨橫移歷史。原本 CUTIN_BETA/DT_MDL = 1.0，路徑跳 1m 會注入 1 m/s 假橫向速度。
LARGE_VEH_GROUP_DD = 25.0           # m，大型車判定的縱向搜尋範圍（涵蓋超長聯結車）
LARGE_VEH_CHAIN_GAP = 4.0           # m，鏈上相鄰反射點最大縱向間距
LARGE_VEH_MIN_SPAN = 5.0            # m，鏈長 >= 此值視為大型車
LARGE_VEH_HOLD_FRAMES = 20          # 1.0 秒判定遲滯
LANE_LINE_MIN_PROB = 0.5            # 目標側車道線信心度下限，低於此值退回本車路徑
LARGE_LINE_ENTER = 0.0              # m，車道線模式：雷達點越過車道線才算候選
LARGE_LINE_RELEASE = -0.1           # m，車道線模式：退回線外 0.1m 即釋放
LARGE_PATH_ENTER = 0.3              # m，本車路徑模式：越過推估車道線 0.3m 才算候選（本車偏移餘裕）
LARGE_PATH_RELEASE = 0.0            # m，本車路徑模式：退回推估車道線外即釋放
LANE_WIDTH_DEFAULT = 3.5            # m
LANE_WIDTH_MIN = 3.0                # m
LANE_WIDTH_MAX = 4.0                # m
LANE_WIDTH_X = 8.0                  # m，量測車道寬的位置（模型座標，攝影機前方）
LANE_WIDTH_ALPHA = 0.05             # 每幀濾波係數（約 1 秒時間常數）
CUTIN_PATH_JUMP_LIMIT = 0.5         # m / 幀

_LANE_WIDTH = {'w': LANE_WIDTH_DEFAULT}   # dp(v9.6): 濾波後車道寬


def _update_large_vehicle_flags(tracks, v_ego: float) -> None:
  # dp(v9.6): 每幀只呼叫一次（leadOne）。以每個雷達點為中心找同車點，沿縱向串鏈，鏈長判定大型車。
  # 一般小車的重複點縱向差 < 1.5m；同車道前後兩台同速小車（車尾點間距通常 > 4m）不會串成一條鏈。
  # dp(v9.6.2):
  #   (c) 只有移動中的點（絕對速度 > max(3, DYNAMIC_SPEED_PCT * v_ego)）參與判定。7 段 log 中被串鏈判為
  #       大型車的片段，多數是相對速度 ≈ −本車速的護欄、門架等靜止物；靜止物本來就不會成為切入候選，
  #       排除後大型車旗標只代表真正的移動大車，也避免診斷 log 混淆。
  #   (a) 同一 ID 曾被串鏈判定為大型車，就維持到該 ID 消失（large_sticky）。會造成切入誤判的一定是
  #       ID 穩定、連續追蹤 >= 0.5 秒的點（淨橫移條件），ID 跳動的點本來就不可能被採用，所以綁 ID 足夠。
  #   (b) 與大型車點為同一物體（_is_same_object）的重複點，同幀繼承大型車身分。log 033 聯結車：
  #       R13604 判為大型車，但其重複點 R13618/R13626/R13630（d、y、v 完全相同）沒有，而這些重複點
  #       位在 |off| 2.25~2.6m，正好在小車候選帶 CUTIN_OFF_MAX 2.5 邊緣，是最可能造成誤煞的點。
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
  # (b) 重複點繼承：以本幀已判定的大型車點為來源，只傳一層（不做遞移擴散）
  sources = [o for o in items if o.is_large]
  for t in items:
    if not t.is_large and any(_is_same_object(t, o) for o in sources):
      t.is_large = True
      t.large_sticky = True

# dp(9.1): 切入雷達量測 freshness。允許 Toyota 雷達短暫 2 幀（約 0.1s）非實測/外推，
# 超過後不再累積或維持 cut-in，避免長時間外推點單靠橫向估計成為切入前車。
CUTIN_MEASURED_MAX_AGE = 2           # 幀；0=本幀實測，1~2=短暫容忍，>2=失效

# dp(修正 4): 模型路徑（modelV2.position）預測。車速低於此值時 position.x 會擠在 0 附近、
# 不單調：橫向閘門改為假設直行，雷達主導救援停用。
MODEL_PATH_MIN_SPEED = 3.0          # m/s
MODEL_PATH_MIN_LENGTH = 5.0         # 模型路徑最短有效長度 (m)

# dp(修正 5): fuzzy 距離容差隨距離放大。原本固定 [0.5, 1.5] m，視覺測距在中遠距離
# 誤差常超過 1.5m，導致 EMA 在 ~30m 外無法累積。
# 比照原廠 dist_sane 精神（25% 或 5m），但取較保守的比例。
FUZZY_D_BOUNDS_PCT = [0.05, 0.12]   # 滿分 / 歸零 對應的距離比例

def _build_lane_ctx(lane_lines, lane_line_probs):
  # dp(v9.6): 整理 modelV2.laneLines[1]（本車左線）、[2]（本車右線）與信心度；
  # 兩側信心度都 >= LANE_LINE_MIN_PROB 時順便更新濾波車道寬。資料不完整時回傳 None（一律本車路徑模式）。
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
    # 模型座標 y 向右為正：車道寬 = 右線 y − 左線 y
    w = float(np.interp(LANE_WIDTH_X, r_x, r_y) - np.interp(LANE_WIDTH_X, l_x, l_y))
    if LANE_WIDTH_MIN - 0.5 < w < LANE_WIDTH_MAX + 0.5:   # 明顯錯誤的量測不納入
      w = float(np.clip(w, LANE_WIDTH_MIN, LANE_WIDTH_MAX))
      _LANE_WIDTH['w'] += LANE_WIDTH_ALPHA * (w - _LANE_WIDTH['w'])
  return ctx


def _lane_intrusion(track, off: float, lane_ctx) -> tuple[float, str]:
  # dp(v9.6): 回傳（雷達點越過目標側車道線的距離，模式）。正值 = 已在本車道內。
  # 目標側由 off 符號決定（off < 0 為右側）。radar 座標 y 向左為正，模型座標 y 向右為正，取負號轉換；
  # 模型 x 由攝影機起算，以 dRel + RADAR_TO_CAMERA 查表（與 _path_y_at 相同）。
  right_side = off < 0.0
  if lane_ctx is not None:
    x, y, p = lane_ctx['right'] if right_side else lane_ctx['left']
    if p >= LANE_LINE_MIN_PROB:
      line_y = -float(np.interp(track.dRel + RADAR_TO_CAMERA, x, y))
      intr = (track.yRel - line_y) if right_side else (line_y - track.yRel)
      return intr, 'line'
  # 本車路徑模式：假設車道線在路徑 ± 濾波車道寬/2
  return _LANE_WIDTH['w'] / 2.0 - abs(off), 'path'


def _is_same_object(a, b) -> bool:
  # dp: 兩個雷達目標是否為同一個實體物體的重複回報
  return (abs(a.dRel - b.dRel) <= SAME_OBJ_MAX_DD and
          abs(a.yRel - b.yRel) <= SAME_OBJ_MAX_DY and
          abs(a.vRel - b.vRel) <= SAME_OBJ_MAX_DV)


_LOW_SPEED_LAST = {'track': None}   # dp: 上一幀低速覆寫選到的雷達目標（重複點閃爍修正用）

# 全域快取：改回 Candy 版邏輯，直接快取 Track 物件本身
# dp: 額外加上 last_aLeadK，用來在「凍結中」跟「剛恢復匹配」兩種情況下，
# 都對輸出的 aLeadK 做變化率限制，避免瞬間跳動觸發幽靈煞車。
# dp(第七版): 加上 last_aLeadK_track，換成「不同物體」時不做變化率限制（見輸出段說明）。
_LEAD_STATE_CACHE = {
    0: {'track': None, 'absent': 0, 'last_aLeadK': None, 'last_aLeadK_track': None, 'rescue': False, 'cutin': False, 'cutin_track': None, 'switch_id': None, 'switch_cnt': 0},
    1: {'track': None, 'absent': 0, 'last_aLeadK': None, 'last_aLeadK_track': None, 'rescue': False, 'cutin': False, 'cutin_track': None, 'switch_id': None, 'switch_cnt': 0}
}
MAX_ALEADK_DELTA_PER_FRAME = 1.0    # aLeadK 每幀最大允許變化量 (m/s²)，可依實測調整


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


class TrackDP(Track):
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    super().__init__(identifier, v_lead, kalman_params)
    self.ema_confidence = {0: 0.4, 1: 0.4}
    self.holdover_frames = {0: 0, 1: 0}
    # dp(修正 1): 依 lead_idx 分開存。原本單一 bool 會被 leadOne/leadTwo 互相覆寫遲滯狀態。
    self.is_out_of_lane = {0: False, 1: False}
    self.gate_initialized = {0: False, 1: False}   # dp: 閘門是否已完成第一幀判定
    self.gate_flip_cnt = {0: 0, 1: 0}              # dp: 連續出現「與目前狀態相反」判定的幀數
    self.gate_last_path_y = {0: None, 1: None}     # dp: 上一幀此目標距離處的走廊中心
    self.closing_speed_streak = {0: 0, 1: 0}   # dp: 連續幾幀符合「正在快速接近」
    self.radar_rescue_frames = 0               # dp: 連續符合雷達主導救援條件的幀數（與 lead_idx 無關）
    self.cutin_off = None                      # dp(第九版): 濾波後相對本車路徑的橫向偏移（左正）
    self.cutin_raw_hist = deque(maxlen=CUTIN_NET_SHIFT_FRAMES + 1)   # dp(v9.4): 未濾波偏移歷史
    self.cutin_vy = 0.0                        # dp(第九版): 濾波後橫向速度（m/s，左正）
    self.cutin_frames = 0                      # dp(第九版): 連續符合切入預測的幀數
    self.cutin_active = False                  # dp(第九版): 已被採用為切入前車
    self.cutin_active_frames = 0               # dp(v9.3): 採用後經過的幀數
    self.cutin_entered = False                 # dp(v9.3): 採用後是否已進入 ±CUTIN_CORRIDOR 走廊
    self.cutin_measured_age = 0                # dp(9.1): 距離最近一次真實雷達量測的幀數
    self.cutin_diag_armed = False              # dp(9.1): 防止候選成立診斷重複洗 log
    self.cutin_last_path_y = None              # dp(v9.6): 上一幀此目標距離處的路徑 y（路徑跳動保護）
    self.large_hold = 0                        # dp(v9.6): 大型車判定遲滯計數
    self.is_large = False                      # dp(v9.6): 是否判定為多點大型車
    self.large_sticky = False                  # dp(v9.6.2): 此 ID 曾判定為大型車，維持到 ID 消失
    self.cutin_is_large = False                # dp(v9.6): 本幀切入判斷使用的大型車旗標（診斷用）
    self.cutin_line_mode = 'path'              # dp(v9.6): 'line' = 車道線模式，'path' = 本車路徑模式
    self.cutin_line_intr = None                # dp(v9.6): 雷達點越過車道線的距離（正值 = 已在本車道內）

  def _check_closing_speed_fallback(self, lead_idx: int, v_ego: float) -> bool:
    # 比照原廠 vel_sane 的 (v_ego + vRel > 3) 這個條件（目標絕對速度 > 3 m/s，即移動目標），
    # 但要求連續 N 幀都成立才啟用，避免單幀雷達雜訊造成的速度瞬間跳動誤觸發。
    is_closing_fast = (v_ego + self.vRel) > VEL_SANE_FALLBACK_SPEED
    if is_closing_fast:
      self.closing_speed_streak[lead_idx] = min(self.closing_speed_streak[lead_idx] + 1, VEL_SANE_CONFIRM_FRAMES)
    else:
      self.closing_speed_streak[lead_idx] = 0
    return self.closing_speed_streak[lead_idx] >= VEL_SANE_CONFIRM_FRAMES

  def _update_lane_gate(self, lead_idx: int, vision_y: float, vision_v: float, v_ego: float, path_y: float) -> bool:
    # dp(修正 1b-v2): 取代原本 _check_spatial_boundaries() 的遲滯鎖存。
    # 原本的遲滯邏輯實際上從未生效（>2.0m 時提早 return，跳過了設定出界的程式），
    # 補上之後又因 1.5m 回歸門檻，讓雷達/視覺橫向偏差 1.5~2.0m 的真前車被永久鎖在出界。
    # dp(切入閃爍修正): 加上去抖動（GATE_DEBOUNCE_FRAMES）與模型路徑跳動保護。
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

  def update_cutin(self, v_ego: float, path_valid: bool, path_y: float, gated: bool,
                   is_large: bool = False, lane_ctx=None) -> None:
    # dp(第九版): 每幀呼叫一次（只在 leadOne 那次）。gated=True 表示本車轉彎或變換車道中，暫停判定。
    # dp(9.1): 加入 radar measurement freshness；短暫漏拍最多容忍 CUTIN_MEASURED_MAX_AGE 幀。
    # dp(v9.6): is_large=True（多點大型車）時，候選與維持改以車道線判斷（見 v9.6 參數區）；
    #           lane_ctx 為 get_lead_ext 準備的車道線資料（None = 無車道線，一律用本車路徑模式）。
    if bool(self.measured):
      self.cutin_measured_age = 0
    else:
      self.cutin_measured_age = min(self.cutin_measured_age + 1, CUTIN_MEASURED_MAX_AGE + 1)
    fresh = self.cutin_measured_age <= CUTIN_MEASURED_MAX_AGE
    self.cutin_is_large = is_large

    if not path_valid:
      if self.cutin_active or self.cutin_frames > 0:
        cloudlog.debug(
          f"[RadarD_CutinReset_DP] id={self.identifier} reason=path_invalid "
          f"d={self.dRel:.1f} y={self.yRel:.2f} age={self.cutin_measured_age}"
        )
      self.cutin_off = None
      self.cutin_vy = 0.0
      self.cutin_frames = 0
      self.cutin_active = False
      self.cutin_diag_armed = False
      self.cutin_raw_hist.clear()                # dp(v9.4): 路徑無效時偏移基準改變，歷史一併清除
      self.cutin_last_path_y = None              # dp(v9.6)
      return

    # dp(v9.6): 路徑跳動保護——路徑在此距離處一幀跳動超過 CUTIN_PATH_JUMP_LIMIT 時，偏移基準已改變，
    # 重設濾波與淨橫移歷史，不把路徑跳動當成目標橫向移動。已採用中的目標不因此釋放，vy 歸零後重新估計。
    path_jump = self.cutin_last_path_y is not None and abs(path_y - self.cutin_last_path_y) > CUTIN_PATH_JUMP_LIMIT
    self.cutin_last_path_y = path_y

    meas = self.yRel - path_y
    if path_jump:
      if self.cutin_active or self.cutin_frames > 0:
        cloudlog.debug(
          f"[RadarD_CutinPathJump_DP] id={self.identifier} d={self.dRel:.1f} y={self.yRel:.2f} "
          f"offNew={meas:.2f} vyPrev={self.cutin_vy:.2f} active={self.cutin_active} "
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
    off, vy = self.cutin_off, self.cutin_vy

    # dp(v9.6): 雷達點越過車道線的距離（大型車用；小車僅記錄供診斷）
    intr, mode = _lane_intrusion(self, off, lane_ctx)
    self.cutin_line_intr = intr
    self.cutin_line_mode = mode

    v_abs = self.vRel + v_ego
    max_d = float(np.interp(v_ego, CUTIN_MAX_DIST_BP, CUTIN_MAX_DIST_V))
    base_ok = (fresh and not gated and v_ego > CUTIN_MIN_V_EGO and CUTIN_MIN_DIST < self.dRel < max_d and
               v_abs > max(3.0, DYNAMIC_SPEED_PCT * v_ego) and
               (self.vRel < CUTIN_CLOSING_VREL or self.dRel < CUTIN_CLOSE_HEADWAY * v_ego))
    toward = off * vy < 0.0

    if self.cutin_active:
      # 已採用：偏移仍在 CUTIN_HOLD_OFF 內、沒有明顯遠離路徑就維持（進入走廊後也維持）
      # dp(v9.6): 大型車改為「仍在車道線內（intr >= 釋放門檻）」才維持，退回線外即釋放
      if is_large:
        release_thr = LARGE_LINE_RELEASE if mode == 'line' else LARGE_PATH_RELEASE
        within = intr >= release_thr
      else:
        within = abs(off) < CUTIN_HOLD_OFF
      moving_away = (off * vy > 0.0) and abs(vy) > CUTIN_VY_MIN and abs(off) > CUTIN_CORRIDOR
      # dp(v9.3): 停滯釋放——採用後 CUTIN_PENDING_MAX_FRAMES 幀內仍未進入走廊就釋放
      self.cutin_active_frames += 1
      if abs(off) <= CUTIN_CORRIDOR:
        self.cutin_entered = True
      stalled = (not self.cutin_entered) and self.cutin_active_frames > CUTIN_PENDING_MAX_FRAMES
      if stalled or not (base_ok and within and not moving_away):
        if stalled:
          reason = 'stalled'
        elif not fresh:
          reason = 'stale'
        elif gated:
          reason = 'gated'
        elif moving_away:
          reason = 'moving_away'
        elif not within:
          reason = 'large_line_out' if is_large else 'off'
        else:
          reason = 'base_condition'
        cloudlog.debug(
          f"[RadarD_CutinRelease_DP] id={self.identifier} reason={reason} "
          f"d={self.dRel:.1f} off={off:.2f} vy={vy:.2f} vRel={self.vRel:.2f} "
          f"aLeadK={self.aLeadK:.2f} aLeadTau={float(self.aLeadTau.x):.2f} age={self.cutin_measured_age} "
          f"large={is_large} mode={mode} intr={intr:.2f} activeFrames={self.cutin_active_frames}"
        )
        self.cutin_active = False
        self.cutin_frames = 0
        self.cutin_diag_armed = False
        self.cutin_active_frames = 0
        self.cutin_entered = False
      return

    self.cutin_active_frames = 0               # dp(v9.3): 未採用狀態下歸零
    self.cutin_entered = False
    if is_large:
      # dp(v9.6): 大型車——雷達點須已越過車道線（本車路徑模式多扣餘裕），取代 CUTIN_OFF_MAX 上限
      enter_thr = LARGE_LINE_ENTER if mode == 'line' else LARGE_PATH_ENTER
      lateral_ok = CUTIN_CORRIDOR < abs(off) and intr >= enter_thr
    else:
      lateral_ok = CUTIN_CORRIDOR < abs(off) < CUTIN_OFF_MAX
    hit = (base_ok and lateral_ok and toward and
           CUTIN_VY_MIN < abs(vy) < CUTIN_VY_MAX and
           (abs(off) - CUTIN_CORRIDOR) / abs(vy) < CUTIN_T_ENTER)
    self.cutin_frames = self.cutin_frames + 1 if hit else 0

    if self.cutin_frames >= _cutin_confirm_frames(v_ego) and not self.cutin_diag_armed:
      t_enter = (abs(off) - CUTIN_CORRIDOR) / max(abs(vy), 1e-3)
      cloudlog.debug(
        f"[RadarD_CutinArmed_DP] id={self.identifier} d={self.dRel:.1f} y={self.yRel:.2f} "
        f"off={off:.2f} vy={vy:.2f} tEnter={t_enter:.2f}s vRel={self.vRel:.2f} "
        f"vLead={self.vLead:.2f} aLeadK={self.aLeadK:.2f} aLeadTau={float(self.aLeadTau.x):.2f} "
        f"frames={self.cutin_frames} age={self.cutin_measured_age} large={is_large} mode={mode} intr={intr:.2f}"
      )
      self.cutin_diag_armed = True
    elif self.cutin_frames == 0:
      self.cutin_diag_armed = False

  def update_radar_rescue(self, v_ego: float, path_valid: bool, path_y: float, steering_angle_deg: float,
                          is_active: bool = False) -> None:
    # dp: 每幀呼叫一次（只在 leadOne 那次呼叫時更新，避免一幀累加兩次）
    # is_active：此目標是上一幀的救援前車，走廊用維持條件 RADAR_RESCUE_CORRIDOR_HOLD
    corridor = RADAR_RESCUE_CORRIDOR_HOLD if is_active else RADAR_RESCUE_CORRIDOR
    eligible = (path_valid and bool(self.measured) and
                RADAR_RESCUE_MIN_DIST < self.dRel < RADAR_RESCUE_MAX_DIST and
                abs(self.yRel - path_y) < corridor and
                (self.vRel + v_ego) > max(RADAR_RESCUE_MIN_SPEED, RADAR_RESCUE_MIN_SPEED_PCT * v_ego) and
                abs(steering_angle_deg) < RADAR_RESCUE_MAX_ANGLE)
    self.radar_rescue_frames = self.radar_rescue_frames + 1 if eligible else 0

  def _calculate_fuzzy_score(self, offset_vision_dist: float, vision_y: float, vision_v: float, v_ego: float, lead_idx: int) -> float:
    err_d = abs(self.dRel - offset_vision_dist)
    err_y = abs(self.yRel - vision_y)
    err_v = abs((self.vRel + v_ego) - vision_v)

    d_bounds = [max(FUZZY_BOUNDS[0], FUZZY_D_BOUNDS_PCT[0] * offset_vision_dist),
                max(FUZZY_BOUNDS[1], FUZZY_D_BOUNDS_PCT[1] * offset_vision_dist)]
    score_d = float(np.interp(err_d, d_bounds, [1.0, 0.0]))
    score_y = float(np.interp(err_y, FUZZY_BOUNDS, [1.0, 0.0]))
    score_v = float(np.interp(err_v, FUZZY_BOUNDS, [1.0, 0.0]))

    # dp: 比照原廠 vel_sane 的擇一寬鬆備援——連續多幀確認目標在移動時，
    # 即使速度誤差略大，也不讓 score_v 把總分拉到 0。
    if self._check_closing_speed_fallback(lead_idx, v_ego):
      score_v = max(score_v, VEL_SANE_FALLBACK_SCORE)

    return score_d * score_y * score_v

  def _calculate_threat_multipliers(self, v_ego: float) -> float:
    brake_mult = float(np.interp(self.aLeadK, BRAKE_THRES_RANGE, MULT_RANGE))
    cutin_mult = 1.0
    
    if self.dRel < CUTIN_DIST_LIMIT and abs(self.yRel) > 1.0:
      v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)
      cutin_mult = float(np.interp(self.vRel, [-v_limit, v_limit], MULT_RANGE))

    final_alpha = ALPHA_BASE * brake_mult * cutin_mult
    return min(1.0, final_alpha)

  def _apply_slow_protection(self, v_ego: float, cam_prob: float, current_ema: float) -> float:
    abs_v_lead = abs(self.vRel + v_ego)
    dynamic_v_limit = max(1.0, DYNAMIC_SPEED_PCT * v_ego)

    if abs_v_lead < dynamic_v_limit:
      dynamic_cam_prob_thres = float(np.interp(v_ego, CAM_PROB_SPEED_RANGE, CAM_PROB_RANGE))
      if cam_prob < dynamic_cam_prob_thres:
        return min(current_ema, STATIC_EMA_CAP)

    return current_ema

  def process_track_logic(self, lead_idx: int, lead_msg: capnp._DynamicStructReader, v_ego: float, lead_prob: float, is_turning: bool = False,
                          path_y: float = 0.0):
    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
    vision_y = -lead_msg.y[0]
    vision_v = lead_msg.v[0]

    # dp: 「必須真實量測」這道門檻改成只在轉彎時生效——
    # 轉彎時保留保護，避免旁側車道目標因外推值誤判成切入本車道；
    # 直行/巡航時放行，避免正常雷達漏拍拖慢插隊車輛的信心度累積、反應變慢半拍。
    #
    # dp(log 驗證修正 1b-v2): 原本 is_out_of_lane 從未被設為 True（見 _update_lane_gate 說明），
    # valid_tracks 的橫向過濾等於不存在。log 實證：92216 55.0s 車速 46 km/h，右車道線外
    # 1.1~1.9m、80m 處的靜止物被選為前車；92101 22.1s 右側 5~7m 的靜止物被選為前車。
    is_out = self._update_lane_gate(lead_idx, vision_y, vision_v, v_ego, path_y)
    is_lateral_far = abs(self.yRel - vision_y) > (LANE_WIDTH_FALLBACK + LANE_HYSTERESIS_MARGIN)
    is_invalid = (is_turning and not self.measured) or is_lateral_far or is_out

    fuzzy_score = 0.0
    if not is_invalid:
      fuzzy_score = self._calculate_fuzzy_score(offset_vision_dist, vision_y, vision_v, v_ego, lead_idx)
      is_invalid = fuzzy_score == 0.0

    if is_invalid:
      if self.holdover_frames[lead_idx] > 0:
        self.holdover_frames[lead_idx] -= 1
        return   # 續命寬限期內的短暫失效，EMA 維持不變
      else:
        self.ema_confidence[lead_idx] = ALPHA_DOWN * 0.0 + (1 - ALPHA_DOWN) * self.ema_confidence[lead_idx]
        return

    self.holdover_frames[lead_idx] = RELEASE_FRAMES

    final_alpha_up = self._calculate_threat_multipliers(v_ego)
    target_ema = fuzzy_score
    alpha = final_alpha_up if fuzzy_score > 0.5 else ALPHA_DOWN
    
    new_ema = alpha * target_ema + (1 - alpha) * self.ema_confidence[lead_idx]
    new_ema = self._apply_slow_protection(v_ego, lead_prob, new_ema)

    self.ema_confidence[lead_idx] = new_ema


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
  DP 適配版：移除了 CP 與 CP_SP，純粹依靠 DP 的系統參數運作。
  新增 is_turning：由 radard.py 依方向盤角度/角速度判斷是否正在轉彎，
  轉出去給 process_track_logic 決定是否要求「必須真實量測」。
  steering_angle_deg：雷達主導救援的方向盤角度開關。
  raw_lead_prob：未濾波的原始 lead 機率，雷達主導救援的視覺信心度下限只看這個值；
  未提供時（例如測試）退回使用濾波後的 lead_prob。
  lane_change：本車正在變換車道（模型 laneChangeState 非 off），切入預測暫停（v9.2 起不看方向燈）。
  path_x/path_y：modelV2.position，供橫向閘門、鎖定黏著、雷達主導救援與切入預測的走廊判斷。
  lane_lines/lane_line_probs（v9.6）：modelV2.laneLines 每條線的 (x, y) 與 laneLineProbs，
  供多點大型車的切入判斷改用車道線；未提供時大型車一律用本車路徑模式。
  radard.py 一律以關鍵字參數傳入上述擴充參數，避免位置參數錯位。
  """
  lead_idx = 0 if low_speed_override else 1
  max_ema_confidence = 0.0

  # dp(修正 4): 優先使用模型規劃路徑（modelV2.position；模型座標 y 向右為正，取負號
  # 轉成 radar 座標 y 向左為正；x 由攝影機起算，故以 dRel + RADAR_TO_CAMERA 查表）。
  # 模型路徑能預見前方彎道，彎道入口「車頭還直、路已經在彎」時不會把彎道外側的
  # 路邊物體框進走廊。低速或路徑無效時，橫向閘門假設直行，雷達主導救援則停用。
  use_model_path = False
  px = py = None
  if path_x is not None and path_y is not None and len(path_x) >= 2 and len(path_x) == len(path_y):
    px = np.asarray(path_x, dtype=float)
    py = np.asarray(path_y, dtype=float)
    if v_ego > MODEL_PATH_MIN_SPEED and bool(np.all(np.diff(px) > 0)) and (px[-1] - px[0]) > MODEL_PATH_MIN_LENGTH:
      use_model_path = True

  def _path_y_at(d: float) -> float:
    # 走廊中心：模型路徑有效時用模型路徑，否則假設直行（低速/停等/路口大角度轉彎時，
    # 自行車模型的二次外推會失真，不適合拿來做排除）。
    if use_model_path:
      return -float(np.interp(d + RADAR_TO_CAMERA, px, py))
    return 0.0

  if ready:
    for track in tracks.values():
      track.process_track_logic(lead_idx, lead_msg, v_ego, lead_prob, is_turning, path_y=_path_y_at(track.dRel))

  valid_tracks = {k: v for k, v in tracks.items() if not v.is_out_of_lane[lead_idx] and v.ema_confidence[lead_idx] > 0.0}

  if len(valid_tracks) > 0:
    max_ema_confidence = max(track.ema_confidence[lead_idx] for track in valid_tracks.values())

  normal_thres = float(np.interp(max_ema_confidence, EMA_VAL_RANGE, PROB_THRES_RANGE))
  current_prob_thres = normal_thres   # 總視覺信心度門檻（雷達主導救援生效時會同步降到 RADAR_RESCUE_MIN_PROB）

  # dp: 雷達主導救援——只處理 leadOne，每幀更新一次所有雷達目標的連續幀數
  rescue_track = None
  rescue_prob = lead_prob if raw_lead_prob is None else raw_lead_prob
  if lead_idx == 0:
    cache0 = _LEAD_STATE_CACHE[0]
    active_rescue = cache0['track'] if cache0['rescue'] else None
    # dp(第八版): 救援目標的雷達點消失、雷達改用新 ID 回報同一台車時（遠距離常見，
    # log 92102 33.8s/35.4s），新 ID 繼承原本的連續幀數，不必重新等 1 秒。
    # 判定沿用重複點的同一物體條件（_is_same_object），拿消失前最後一幀的位置比對。
    if active_rescue is not None and tracks.get(active_rescue.identifier) is not active_rescue:
      heirs = [t for t in tracks.values() if _is_same_object(active_rescue, t)]
      if len(heirs) > 0:
        heir = min(heirs, key=lambda t: abs(t.dRel - active_rescue.dRel) + abs(t.yRel - active_rescue.yRel))
        heir.radar_rescue_frames = max(heir.radar_rescue_frames, active_rescue.radar_rescue_frames)
        cache0['track'] = heir
        active_rescue = heir
      else:
        active_rescue = None
    # v9.3（停用切入跨 ID 繼承）：新 Track 使用建構時的初始切入狀態，
    # 重新估算橫向速度與累積確認幀數，避免大車反射點交換 ID 時接續錯誤狀態。

    # dp(v9.6): 先依本幀所有雷達點判定多點大型車，並整理車道線（同時更新濾波車道寬）
    if len(tracks) > 0:
      _update_large_vehicle_flags(tracks, v_ego)
    lane_ctx = _build_lane_ctx(lane_lines, lane_line_probs)

    for track in tracks.values():
      track.update_radar_rescue(v_ego, use_model_path, _path_y_at(track.dRel), steering_angle_deg,
                                is_active=track is active_rescue)
      track.update_cutin(v_ego, use_model_path, _path_y_at(track.dRel), is_turning or lane_change,
                         is_large=track.is_large, lane_ctx=lane_ctx)

    rescue_candidates = [t for t in tracks.values() if t.radar_rescue_frames >= RADAR_RESCUE_CONFIRM_FRAMES]
    if ready and rescue_prob >= RADAR_RESCUE_MIN_PROB and len(rescue_candidates) > 0:
      rescue_track = min(rescue_candidates, key=lambda t: t.dRel)
      # dp(重複雷達點閃爍修正): 上一幀的救援目標仍是候選、且與最近候選為同一物體時沿用
      prev_rescue = _LEAD_STATE_CACHE[lead_idx]['track'] if _LEAD_STATE_CACHE[lead_idx]['rescue'] else None
      if (prev_rescue is not None and prev_rescue is not rescue_track and
          any(t is prev_rescue for t in rescue_candidates) and _is_same_object(prev_rescue, rescue_track)):
        rescue_track = prev_rescue

  matched_track = None
  if len(valid_tracks) > 0 and ready and lead_prob > current_prob_thres:
    matched_track = match_vision_to_track(v_ego, lead_msg, valid_tracks)

  # 狀態機記憶：還原 Candy 版的殭屍物件強制續命邏輯 (直接快取物件)
  # dp: 先判斷「這一幀若沒有配對成功，是否會由續命沿用上一個目標」，讓雷達主導救援能拿
  # 「現有前車」做距離比較。cache['rescue'] 標記快取中的目標是否來自雷達主導救援。
  cache = _LEAD_STATE_CACHE[lead_idx]

  # dp(重複雷達點閃爍修正): 新配對結果和目前鎖定目標是同一物體時，沿用目前鎖定目標
  prev_track = cache['track']
  if (matched_track is not None and prev_track is not None and matched_track is not prev_track and
      tracks.get(prev_track.identifier) is prev_track and _is_same_object(prev_track, matched_track)):
    matched_track = prev_track

  # dp(切入閃爍修正): 目前鎖定的雷達目標是否仍在雷達清單中、且仍在本車走廊內
  locked = cache['track'] if (cache['track'] is not None and not cache['rescue'] and not cache['cutin']) else None
  locked_in_path = (locked is not None and tracks.get(locked.identifier) is locked and
                    (locked.vRel + v_ego) > max(1.0, DYNAMIC_SPEED_PCT * v_ego) and
                    not locked.is_out_of_lane[lead_idx] and
                    abs(locked.yRel - _path_y_at(locked.dRel)) <= LANE_CORRIDOR_HALF_WIDTH)

  # dp(切入閃爍修正 2): 換成更遠的目標需連續確認；換成更近的目標（切入車）立即切換
  if matched_track is not None and locked_in_path and matched_track is not locked and matched_track.dRel > locked.dRel:
    if cache['switch_id'] == matched_track.identifier:
      cache['switch_cnt'] += 1
    else:
      cache['switch_id'] = matched_track.identifier
      cache['switch_cnt'] = 1
    if cache['switch_cnt'] < SWITCH_FARTHER_CONFIRM_FRAMES:
      matched_track = locked   # 確認期間維持原鎖定目標
  else:
    cache['switch_id'] = None
    cache['switch_cnt'] = 0

  # 純視覺後備前車候選：門檻刻意維持 normal_thres（最低 0.3），不跟著雷達主導救援降到 0.1。
  # 0.1 只允許用在「雷達已連續 1 秒確認的本車道移動目標」，不允許單獨由視覺產生前車。
  vision_cand = None
  if matched_track is None and ready and lead_prob > normal_thres:
    vision_cand = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  # dp(切入閃爍修正 1): 鎖定目標仍在走廊內、且沒有更近的純視覺前車時，延長續命
  hold_limit = SELECT_HOLDOVER_FRAMES
  if locked_in_path and (vision_cand is None or vision_cand['dRel'] >= locked.dRel):
    hold_limit = STICKY_HOLD_FRAMES

  held_track = None
  if matched_track is None and cache['track'] is not None and cache['absent'] + 1 <= hold_limit:
    held_track = cache['track']
  held_is_rescue = held_track is not None and (cache['rescue'] or cache['cutin'])   # 救援/切入前車皆不續命

  # 現有前車距離（不含來自救援的續命目標，救援目標每幀都要重新和現有前車比較）
  if matched_track is not None:
    existing_d = matched_track.dRel
  elif held_track is not None and not held_is_rescue:
    existing_d = held_track.dRel
  elif vision_cand is not None:
    existing_d = vision_cand['dRel']
  else:
    existing_d = float('inf')

  # dp: 雷達主導救援採用判斷——沒有前車，或救援候選比現有前車更近才採用
  # dp(重複雷達點閃爍修正): 救援候選若與現有雷達前車是同一物體（重複點），不算「更近的前車」
  existing_track = matched_track if matched_track is not None else (held_track if not held_is_rescue else None)
  is_radar_rescue = (rescue_track is not None and rescue_track is not matched_track and
                     rescue_track.dRel < existing_d and
                     not (existing_track is not None and _is_same_object(existing_track, rescue_track)))

  vision_lead = None
  if is_radar_rescue:
    selected_track = rescue_track
    cache['track'] = rescue_track
    cache['absent'] = 0
    cache['rescue'] = True
    cache['cutin'] = False
    # 同步總視覺信心度門檻：這一幀的前車是在 RADAR_RESCUE_MIN_PROB 門檻下被接受的
    current_prob_thres = min(current_prob_thres, RADAR_RESCUE_MIN_PROB)
    cloudlog.debug(
      f"[RadarD_RadarRescue_DP] 雷達主導救援！目標 {lead_idx} | 雷達 {rescue_track.identifier} "
      f"d={rescue_track.dRel:.1f} y={rescue_track.yRel:.2f} v={rescue_track.vRel + v_ego:.1f} | "
      f"連續 {rescue_track.radar_rescue_frames} 幀 | 相機機率(原始/濾波): {rescue_prob:.2f}/{lead_prob:.2f} "
      f"(原門檻: {normal_thres:.2f} → {current_prob_thres:.2f}) | 原前車距離: {existing_d:.1f}"
    )
  elif matched_track is not None:
    selected_track = matched_track
    cache['track'] = matched_track
    cache['absent'] = 0
    cache['rescue'] = False
    cache['cutin'] = False
  else:
    selected_track = None
    if held_is_rescue:
      # 最嚴格版本：來自雷達主導救援的目標不享有續命。救援條件（原始視覺機率 >= 0.1、
      # 雷達連續 1 秒、移動、走廊內、真實量測）任一項在這一幀不成立，救援前車當幀就撤銷，
      # 不用 SELECT_HOLDOVER_FRAMES 延長。
      cache['track'] = None
      cache['absent'] = 0
      cache['last_aLeadK'] = None
      cache['rescue'] = False
      cache['cutin'] = False
    elif cache['track'] is not None:
      cache['absent'] += 1
      if cache['absent'] <= hold_limit:
        selected_track = cache['track']  # 強制回傳上一刻的凍結物件，維持鎖定
      else:
        cache['track'] = None
        cache['absent'] = 0
        cache['last_aLeadK'] = None  # lead 真正消失，重置參考基準，避免下一個新目標被錯誤地拿舊值做限制
        cache['rescue'] = False
        cache['cutin'] = False
    if selected_track is None:
      vision_lead = vision_cand

  # dp(第九版): 切入預測仲裁（只處理 leadOne）。候選 = 已採用中的切入目標，或連續成立
  # CUTIN_CONFIRM_FRAMES 幀的新目標；取最近者。沒有前車、或比現有前車更近且不是同一物體才採用。
  is_cutin = False
  cutin_prev = cache['cutin_track']
  cache['cutin_track'] = None
  if lead_idx == 0 and ready:
    cutin_cands = [t for t in tracks.values() if t.cutin_active or
                   (t.cutin_frames >= _cutin_confirm_frames(v_ego) and _cutin_group_consistent(t, tracks) and
                    _cutin_net_shift_ok(t))]
    if len(cutin_cands) > 0:
      cutin_track = min(cutin_cands, key=lambda t: t.dRel)
      # 同一台車的多個反射點都成為候選時，沿用上一幀的前車點，避免在反射點之間互換
      prev_t = cutin_prev
      if (prev_t is not None and prev_t is not cutin_track and any(t is prev_t for t in cutin_cands) and
          _is_same_object(prev_t, cutin_track)):
        cutin_track = prev_t
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
        cutin_track.cutin_active = True   # 已由其他路徑選中同一物體：保持狀態，交由原路徑輸出
      elif cutin_track.dRel < cur_d:
        cutin_track.cutin_active = True
        selected_track = cutin_track
        vision_lead = None
        is_cutin = True
        cache['track'] = cutin_track
        cache['absent'] = 0
        cache['rescue'] = False
        cache['cutin'] = True             # 與救援相同：不享有續命，條件不成立當幀撤銷
        cache['cutin_track'] = cutin_track
        off = cutin_track.cutin_off if cutin_track.cutin_off is not None else 0.0
        vy = cutin_track.cutin_vy
        t_enter = ((abs(off) - CUTIN_CORRIDOR) / max(abs(vy), 1e-3)) if abs(off) > CUTIN_CORRIDOR else 0.0
        if cutin_prev is not cutin_track:
          cloudlog.debug(
            f"[RadarD_CutinSelect_DP] id={cutin_track.identifier} d={cutin_track.dRel:.1f} "
            f"y={cutin_track.yRel:.2f} off={off:.2f} vy={vy:.2f} tEnter={t_enter:.2f}s "
            f"vRel={cutin_track.vRel:.2f} vLead={cutin_track.vLead:.2f} "
            f"aLeadK={cutin_track.aLeadK:.2f} aLeadTau={float(cutin_track.aLeadTau.x):.2f} "
            f"frames={cutin_track.cutin_frames} age={cutin_track.cutin_measured_age} oldLeadD={cur_d:.1f} "
            f"large={cutin_track.cutin_is_large} mode={cutin_track.cutin_line_mode} "
            f"intr={cutin_track.cutin_line_intr if cutin_track.cutin_line_intr is not None else float('nan'):.2f}"
          )
      else:
        cutin_track.cutin_active = False
    for t in tracks.values():
      if t.cutin_active and t is not selected_track and not (selected_track is not None and _is_same_object(selected_track, t)):
        t.cutin_active = False

  lead_dict = {'status': False}
  if selected_track is not None:
    # dp(第九版): 切入前車的 modelProb 填 0（比照原廠低速覆寫）。此時 leadsV3[0].prob 描述的是
    # 另一台車，不能拿來當切入車的視覺確認，也避免 MPC 以此機率觸發 FCW。
    lead_dict = selected_track.get_RadarState(0.0 if is_cutin else lead_prob)

    # dp: 不管是「凍結續命中」還是「剛恢復匹配、瞬間跳到最新卡曼值」，
    # 都對 aLeadK 做變化率限制，避免瞬間跳動被誤判成前車突然減速（幽靈煞車）。
    # 只限制 aLeadK，dRel/yRel/vRel 不受影響，維持插隊偵測所需的位置即時性。
    # dp(第七版): 換成「不同物體」（例如切入車取代原前車）時不做限制，直接採用新目標的
    # aLeadK。原本會拿舊前車的 aLeadK 當基準、每幀最多變 1.0 m/s²，切入車若正在急煞，
    # 要好幾幀才追上真值。同一物體的重複雷達點之間仍照常限制。
    last_ref = cache['last_aLeadK_track']
    if last_ref is not None and last_ref is not selected_track and not _is_same_object(last_ref, selected_track):
      cache['last_aLeadK'] = None
    if cache['last_aLeadK'] is not None:
      raw_aLeadK = lead_dict['aLeadK']
      delta = float(np.clip(raw_aLeadK - cache['last_aLeadK'], -MAX_ALEADK_DELTA_PER_FRAME, MAX_ALEADK_DELTA_PER_FRAME))
      lead_dict['aLeadK'] = float(cache['last_aLeadK'] + delta)
    cache['last_aLeadK'] = float(lead_dict['aLeadK'])
    cache['last_aLeadK_track'] = selected_track

    # 視覺加速度雙重驗證阻尼
    # dp(第九版): 切入 / 雷達主導救援的前車與視覺前車可能不是同一台車，不套用以視覺加速度推得的 aLeadTau
    model_tau = get_model_lead_tau(lead_msg, lead_prob) if not (is_cutin or is_radar_rescue) else None
    if model_tau is not None:
      lead_dict['aLeadTau'] = model_tau

    if not is_radar_rescue and current_prob_thres < 0.5 and (0.5 >= lead_prob > current_prob_thres):
      cloudlog.debug(
        f"[RadarD_EarlyLock_DP] 提早鎖定/續命成功！目標 {lead_idx} | "
        f"相機機率: {lead_prob:.2f} (動態門檻: {current_prob_thres:.2f})"
      )

  elif vision_lead is not None:
    lead_dict = vision_lead
    _LEAD_STATE_CACHE[lead_idx]['last_aLeadK'] = None  # 純視覺後備路徑不經過雷達物件，重置參考基準


  # 原廠底線救援
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    low_speed_used = None
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      # dp(重複雷達點閃爍修正): 上一幀低速覆寫選到的目標仍在清單中、且與最近目標為同一物體時沿用
      last_ls = _LOW_SPEED_LAST['track']
      if (last_ls is not None and last_ls is not closest_track and
          any(c is last_ls for c in low_speed_tracks) and _is_same_object(last_ls, closest_track)):
        closest_track = last_ls
      # dp(重複雷達點閃爍修正): 現有雷達前車與最近目標為同一物體時，不因幾公分的差距而替換
      # （刻意偏離原廠：原廠只要更近就替換，同一物體的重複點會因此每幀互換）
      cur_track = tracks.get(lead_dict['radarTrackId']) if (lead_dict['status'] and lead_dict.get('radar')) else None
      same_as_current = cur_track is not None and _is_same_object(cur_track, closest_track)
      if not same_as_current and ((not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel'])):
        lead_dict = closest_track.get_RadarState()
        low_speed_used = closest_track
    _LOW_SPEED_LAST['track'] = low_speed_used

  return lead_dict


# ==============================================================================
# 雙重 Monkey Patching
# ==============================================================================
radard.Track = TrackDP
radard.get_lead = get_lead_ext


class RadarDExt(RadarD):
  """
  DP 版專屬：初始化參數對齊 DP 的單一 delay 參數（第七版起移除未使用的 steer_ratio/wheelbase，
  與最初版介面相同）。
  """
  def __init__(self, delay: float = 0.0):
    super().__init__(delay)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    super().update(sm, rr)
