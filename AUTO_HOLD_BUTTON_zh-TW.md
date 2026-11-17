# opview Auto Hold 按鈕（暫停 / 恢復 comma 端自動煞車維持）

2026-11-16 新增。適用分支：opview、openpilot-dplcc、openpilot-dp0111pre（dragonpilot 系）、openpilot-test（sunnypilot 系）。

## 功能

> **僅限豐田 TSS2，舊款油電專用**（沒有原廠 Auto Hold 的舊款油電車）。汽油車實測無法保持；非 TSS2 車款 comma 不會回報可用，按鈕不會出現。

直向 opview 在**儀表卡片正下方**（PortraitPanel；空間分配優先級最高，關閉儀表卡片時在最上方）顯示一顆 Auto Hold 按鈕，點一下切換 comma 端
豐田自動煞車維持的「暫停 / 恢復」。

| 按鈕 | 意義 | 點擊後 |
|---|---|---|
| 綠色「正在運作」＋手掌圖示 | 自動煞車維持作用中；正在煞住車時加粗白框，狀態字改為白底「煞車保持中」 | 暫停：保持中立即放開，之後不再保持 |
| 紅色「暫停」＋暫停圖示 | 已被 opview 暫停 | 恢復：停車滿 2 秒（原本的計時）才再保持 |

按鈕版面（高 56）：左側白底圓形圖示 → 小字「AUTO HOLD」＋大字狀態（正在運作 / 暫停；煞住車時為白底「煞車保持中」）→ 右側白框動作鈕「點擊暫停」/「點擊恢復」。
除了顏色，圖示形狀、狀態文字、白框都不同，色弱或陽光下也能分辨。

**顯示條件（全部成立才出現）**

1. 顯示設定 →「行車提醒」→「自動煞車維持按鈕」（副標「僅限 TSS2 舊款油電」）開啟（**預設關閉**）
2. 直向；已連線且資料未中斷（`_stale` 為 false）
3. comma 回報「可用」：豐田 TSS2、自動煞車維持設定已開啟、車輛符合條件、已發動；舊版 comma 不回報 → 不顯示
4. 車速 < 60 km/h（`carState.vEgo × 3.6`）
5. 非巡航狀態：`selfdriveState.enabled` 與 `carState.cruiseState.enabled` 皆為 false

按鈕出現時，先扣掉按鈕高度，儀表卡片 / 路況卡 / LINE / 停車面板再依剩餘高度重新分配（按鈕不會被擠掉）。

**按鈕顏色一律以 comma 回報的實際狀態為準**（webrtcd 附在 carState 的 `opviewAutoHold`），不是以 HTTP 回應為準。
**自動恢復**：暫停中車速超過 30 km/h，comma 端 card 自動清除暫停旗標（按鈕回到綠色）；
每次發動（onroad 開始）也會重置為「恢復」。自動恢復在 comma 端判斷，手機斷線時同樣有效。

## 資料流

```
opview 點擊 ──POST :5088/api/auto_hold {"paused": bool}──▶ serverd（dp）/ opviewd（sp）
                                                          │ 寫入 Params OpviewAutoHoldPaused
                                                          ▼
                        card.params_thread（10 Hz）讀 OpviewAutoHoldPaused
                          （暫停中且 CS.vEgo > 30 km/h → 寫回 false，自動恢復）
                          → card.opview_auto_hold_paused
                        card 主迴圈每次 CI.apply 前（100 Hz）：暫停中就把 carcontroller 的狀態機歸零
                          ├─ dp ：CI.CC.auto_brake_hold.state = IDLE、.counter = 0
                          └─ sp ：CI.CC._brake_hold_state = IDLE、._brake_hold_counter = 0
                             （狀態機 50 Hz 才跑一次，每次跑之前都被歸零 → 計數到不了門檻、不會保持；
                               保持中則下一次就放開。opendbc 完全不修改）
                          回報實際狀態（只在變化時寫入）Params OpviewAutoHoldState
                             bit0 可用、bit1 已暫停、bit2 保持中
                                                          │
                        webrtcd（每 0.2 秒最多讀一次）附在 carState 的 "opviewAutoHold":
                          {"available": bool, "paused": bool, "holding": bool}
                        另外 carState 精簡欄位加上 cruiseState.enabled
                                                          ▼
                        opview UIState.autoHoldAvailable / autoHoldPaused / autoHoldHolding / cruiseEnabled
```

## comma 端介面（dp serverd 與 sp opviewd 相同）

```
GET  http://<comma>:5088/api/auto_hold
  200 {"paused": bool, "available": bool, "holding": bool}

POST http://<comma>:5088/api/auto_hold
Content-Type: application/json
{"paused": true}        // 或 false
```

| 狀態 | 回應 | opview 顯示 |
|---|---|---|
| 200 | `{"paused", "available", "holding"}`（寫入後） | 「Auto Hold 已暫停」/「Auto Hold 已恢復」 |
| 400 | body 格式錯誤 | 直接顯示原因 |
| 409 | 暫停時 comma 回報不可用：`comma 端自動煞車維持未啟用（需開啟設定並發動車輛）` | 直接顯示原因 |
| 404 | — | 「comma 端沒有 Auto Hold 控制（請更新 comma 分支）」 |
| 逾時 / 連不到 | — | 「連不到 comma，Auto Hold 未切換」 |

- 恢復一律允許（即使不可用 / 已熄火）；暫停只在可用時允許。
- 逾時 5 秒，不重試。

## Params（新增）

| key | 型別 | 旗標 | 說明 |
|---|---|---|---|
| `OpviewAutoHoldPaused` | BOOL | CLEAR_ON_MANAGER_START \| CLEAR_ON_ONROAD_TRANSITION | opview 寫入的暫停旗標，每次發動重置 |
| `OpviewAutoHoldState` | INT | CLEAR_ON_MANAGER_START \| CLEAR_ON_ONROAD_TRANSITION \| CLEAR_ON_OFFROAD_TRANSITION | card 回報：bit0 可用、bit1 已暫停、bit2 保持中 |

「可用」的判定（card 另外明確檢查 `CP.brand == "toyota"`；dp 再檢查 `ToyotaFlags.TSS2`）：
- dp：`ToyotaFlags.AUTO_BRAKE_HOLD`（`dp_toyota_auto_brake_hold` ≠ 關閉，且 interface.py 判定合格：TSS2 相機 ACC、非 SecOC、openpilot 縱向）
- sp：`ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD`（`ToyotaEnableAutoHold` ≠ 0）且 `ToyotaFlags.TSS2`（與原本送出條件相同）

「保持中」的判定：
- dp：`CS.auto_brake_hold_active`（真的送出 -2.0 保持指令時才為 true，與 autoBrakeHold 事件相同）
- sp：`CC.brake_hold_active`（opendbc 原有屬性）

## 修改檔案

### openpilot-dplcc / openpilot-dp0111pre
- `common/params_keys.h`：新增兩個 Params（dp0111pre 與 dplcc 的此檔原本就有一行差異，**請各自覆蓋自己的版本**）
- `opendbc_repo/opendbc/car/toyota/auto_brake_hold.py`：**不修改**（壓縮檔內附原版，用來還原先前版本的修改）
- `selfdrive/car/card.py`：`opview_auto_hold_update()`（`params_thread` 每 0.1 秒讀參數、30 km/h 自動恢復）、`opview_auto_hold_apply()`（主迴圈 `CI.apply` 前歸零狀態機）
  - 不能用 `auto_brake_hold.enabled = False` 暫停：停車時 panda 已擋掉相機原始 0x344，openpilot 停止轉送會讓 PRE_COLLISION_2 中斷
- `system/webrtc/webrtcd.py`：carState 精簡欄位加 `cruiseState.enabled`；附加 `opviewAutoHold`
- `dragonpilot/dashy/serverd.py`：`GET/POST /api/auto_hold`
- `dragonpilot/dashy/tests/test_auto_hold.py`：端點條件測試（新檔）

### openpilot-test（sunnypilot）
- `openpilot/common/params_keys.h`：新增兩個 Params
- `opendbc_repo/opendbc/car/toyota/carcontroller.py`：**不修改**（壓縮檔內附原版，用來還原先前版本的修改）
- `openpilot/selfdrive/car/card.py`：`opview_auto_hold_available()`（`ToyotaFlagsSP.SP_AUTO_BRAKE_HOLD` 且 `ToyotaFlags.TSS2`）、`opview_auto_hold_update()`（`params_thread` 每 0.1 秒）、`opview_auto_hold_apply()`（主迴圈 `CI.apply` 前歸零狀態機）
- `openpilot/system/webrtc/webrtcd.py`：同 dp（只在 opview 連線時附加）
- `openpilot/sunnypilot/opview/opviewd.py`：`GET/POST /api/auto_hold`
- `openpilot/sunnypilot/opview/tests/test_opviewd.py`：新增 4 個測試

### opview
- `lib/services/comma_auto_hold_api.dart`（新檔）：`setCommaAutoHoldPaused()`
- `lib/selfdrive/ui/ui_state.dart`：`cruiseEnabled`、`autoHold*`、`autoHoldButtonEligible`
- `lib/services/viewer_settings.dart`：`autoHoldButton`（預設 false，`viewer_auto_hold_button`）
- `lib/selfdrive/ui/settings/viewer_settings_dialog.dart`：「行車提醒」新增開關與說明
- `lib/selfdrive/ui/onroad/portrait_panel.dart`：按鈕（最上方、先扣高度）
- `lib/selfdrive/ui/onroad/augmented_road_view.dart`：`_toggleAutoHold()`、invalidateKey、傳入 PortraitPanel
- `test/comma_auto_hold_api_test.dart`（新檔）

## 行為注意事項

- 暫停時若正在保持，煞車**立即放開**：油電車在 D 檔會開始蠕行，請先踩住煞車再按。
- 暫停後維持到按「恢復」、車速超過 30 km/h（自動恢復）或下次發動。
  因此停等時暫停 → 起步開走 → 超過 30 km/h 即自動恢復，下次停車又會正常保持。
- 只影響 openpilot 自己的自動煞車維持（PCS 借用），不影響原廠 Auto Hold（`brakeHoldActive`）。

## 驗證方式

1. `curl http://<comma>:5088/api/auto_hold` → 已發動且設定開啟時 `available: true`
2. 停車滿 2 秒 → opview 綠色按鈕加白框、狀態變白底「煞車保持中」
3. 按按鈕 → 約 0.3 秒內變紅色「暫停」、車輛放開
4. 再按 → 變綠色「正在運作」；停車滿 2 秒後再次「煞車保持中」
5. 暫停後開車超過 30 km/h → 按鈕自動變回綠色（`OpviewAutoHoldPaused` 被清除）
6. 開啟巡航或車速超過 60 km/h → 按鈕消失

## 變更紀錄

- 2026-11-16 b：暫停中車速超過 30 km/h 自動恢復。
- 2026-11-16 c：位置改到儀表卡片正下方。
- 2026-11-16 d：顏色維持紅 = 恢復、綠 = 暫停；加強辨識（圖示形狀、大字狀態、保持中白框、動作鈕），高度 52 → 56。
- 2026-11-16 e：顏色對調為綠 = 恢復 / 保持中、紅 = 暫停（版面與辨識設計同 d 版）。
- 2026-11-16 f：僅限豐田 TSS2（card 明確檢查品牌與 TSS2）；opview 設定標示「TSS2 舊款油電專用」。
- 2026-11-16 g：狀態字「恢復」改為「正在運作」；煞住車時改為白底「煞車保持中」，取消原本的「保持中」標籤。
- 2026-11-16 h：設定項目改名「自動煞車維持按鈕」＋副標「僅限 TSS2 舊款油電」（原標題中英混排、換三行）；圖示改綠色手掌；說明改為分段條列。
- 2026-11-17：設定標題固定換行為「自動煞車／維持按鈕」兩行（說明視窗標題仍為一行）。
