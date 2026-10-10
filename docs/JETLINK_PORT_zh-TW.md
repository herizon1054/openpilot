# Jetlink 移植說明：zoompilot → openpilot-dp0111pre

> 本文件源自 Claude 版本，包含該版本的歷史測試敘述，不代表本次重新執行。新分支的實際修改與本次驗證結果請見 `JETLINK_PREBUILD_PORT_zh-TW.md`。

> 來源：`zoompilot-develop__261008__7`（jetlink 套件 `jetlink_repo` v0.8.5，API 2）
> 目標：`openpilot-dp0111pre`（dragonpilot，upstream base `c0ab3550`，tinygrad pin `eecd4706`）
> 參考：`Zoompilot-modeld.zip`（`openpilot-dpjetlink/selfdrive/modeld`）
> 日期：2026-10-09

## 1. 功能概要

Jetlink 讓接在 comma USB-C 埠上的外部裝置（Jetson Orin Nano Super / Linux PC + NVIDIA / Apple silicon Mac / iPhone·iPad / Android 實驗版）運算 comma 的**大模型**，comma 本身保留：相機、校正、warp、parser、controlsd、panda、CAN。外部裝置只是「warp 後影像 + 少量純量進，18452 個 float 出」的純函式，不持有任何控制狀態。

- comma 端每幀做 warp（GPU，約 2 ms），把 393 KB 的 warped frame 經 USB（FunctionFS gadget）或 iPhone 的 USB 網路（CDC-NCM，192.168.60.1:5599）送出。
- 外部裝置執行 `run_policy`，歷史佇列留在外部裝置上（每幀只傳約 0.5 MB 而非 10 MB）。
- 每幀預算 50 ms；回覆逾時就「hold」（重播上一幀輸出），連續 hold 過多、或 modeld 掉幀比例超過 0.75%，就交回小模型。
- **小模型永遠先開**：外部裝置在背景連線，就緒後只在「沒有任何東西在控制」時才換上大模型；換上後第一秒為驗證期，禁止 engage。

## 2. 移植內容總覽

### 2.1 新增檔案

| 路徑 | 說明 |
| --- | --- |
| `jetlink_repo/jetlink/**` | jetlink 套件（comma 端，純 stdlib + numpy；tinygrad 只在 modeld / build 用）。**有 3 處 dp 相容修改**，見 §5 |
| `jetlink_repo/scripts/comma/jetlink-root.sh` | 所有 root 動作（USB gadget、PD 角色、UDC、VM sysctl、iOS 網路）。由 `sudo -n bash` 執行 |
| `jetlink_repo/LICENSE`、`README.md`、`pyproject.toml` | 上游授權（MIT）與版本 |
| `jetlink_repo/jetlink` | 唯一套件位置；啟動腳本、adapter 與建置流程設定匯入路徑，不建立根目錄 symlink |
| `dragonpilot/jetlink_adapter/__init__.py` | **轉接層**：實作 `jetlink.openpilot.interface.Openpilot`，並提供所有 hook（`should_run/status/reason/prepare/attach/in_control/request_shutdown/shutdown_pending`）。頂層只 import stdlib（jetlinkd 常駐約 10 MB） |
| `dragonpilot/jetlink_adapter/SConscript` | 建置期編譯 comma 端 warp JIT：`dragonpilot/jetlink_adapter/models/warp_<cam>_512x256_tinygrad.pkl`（tici 1928x1208、mici 1344x760 兩種都建） |
| `dragonpilot/selfdrive/selfdrived/accelerator_events.py` | 行車事件：大模型就緒提示、切換 no-entry、啟用提示、斷線警告；切換時的 settling |
| `dragonpilot/selfdrive/ui/jetlink_ui.py` | UI 共用：模式讀取、狀態文字/顏色、進度、failover 說明、圖示選擇 |
| `dragonpilot/selfdrive/ui/layouts/settings/jetlink.py` | tici 設定頁「Jetlink」 |
| `dragonpilot/selfdrive/assets/icons/jetlink{,_green,_orange}.png` | 圖示（comma 的 chestnut 圖示，zoompilot 同樣用它表示 Jetlink） |
| `docs/JETLINK_PORT_zh-TW.md` | 本文件 |

### 2.2 修改檔案

| 檔案 | 修改 |
| --- | --- |
| `common/params_keys.h` | `JetlinkLink`(INT, 0/1/2)、`JetlinkSpec`、`JetlinkModelPointers`、`JetlinkBigModel`、`JetlinkCatalog`、`JetlinkChargePhone`、`AcceleratorProgress`、`Offroad_AcceleratorUnavailable` |
| `cereal/custom.capnp` | `ModelExt` 新增 `acceleratorState @2`（enum none/joining/running/retrying/unavailable/ready）、`bigModel @3` |
| `cereal/log.capnp` | `OnroadEvent.EventName` 新增 `bigModelLoading @100`、`bigModelReady @101`、`bigModelAvailable @102`、`bigModelLinkLost @103` |
| `selfdrive/controls/controlsd.py` | 大模型切換後 1.2 s 內 ALKA 不起動 |
| `selfdrive/modeld/modeld.py` | 見 §3.1 |
| `selfdrive/modeld/SConscript` | 呼叫 `dragonpilot/jetlink_adapter/SConscript`，傳入 `CAMERA_CONFIGS` |
| `system/manager/process_config.py` | `RestartingPythonProcess`（含 crash backoff）與 `jetlinkd` 程序（`always_run and should_run`） |
| `system/hardware/hardwared.py` | 離線警示 `Offroad_AcceleratorUnavailable`；關機時先請外部裝置關機（最多等 25 s），期間 `not_powering_off` 阻擋上路 |
| `selfdrive/selfdrived/selfdrived.py` | `AcceleratorEvents`；`jetlinkd` 不計入 processNotRunning；切換 settling 期間壓住 commIssue / posenet / locationd 檢查 |
| `selfdrive/selfdrived/events.py` | 4 個新事件的警示（繁中） |
| `selfdrive/selfdrived/alerts_offroad.json` | `Offroad_AcceleratorUnavailable`：「Jetlink unavailable: %1」 |
| `selfdrive/ui/ui_state.py` | `JetlinkState` enum；`ui_state.jetlink`（5 Hz params 執行緒取 snapshot）、`jetlink_state`、`jetlink_view`、`adb_blocked`；訂閱 `modelExt` |
| `selfdrive/ui/layouts/sidebar.py` | Jetlink 開啟時：home/flag 按鈕位置改顯示 Jetlink 圖示，並多一張「JETLINK」狀態卡 |
| `selfdrive/ui/onroad/hud_renderer.py` | 行車畫面：實驗模式按鈕下方的 Jetlink 徽章（圖示 + 狀態字） |
| `selfdrive/ui/layouts/settings/settings.py` | 新增「Jetlink」面板；8 個面板時自動縮小導覽列行高 |
| `selfdrive/ui/layouts/settings/developer.py`、`mici/.../developer.py` | Jetlink 開啟時 ADB 開關變灰 |
| `selfdrive/ui/mici/layouts/home.py` | eGPU 圖示旁加入 Jetlink 圖示（連線中閃爍 / 綠 / 半透明綠 / 橘） |
| `selfdrive/ui/mici/onroad/hud_renderer.py` | zoompilot 的 `_draw_model_source` |
| `selfdrive/ui/mici/layouts/settings/toggles.py` | mici 的 jetlink off/usb/ios 開關 |
| `system/ui/widgets/icon_widget.py` | `set_opacity()`（圖示閃爍用） |
| `launch_chffrplus.sh` | 加入 `jetlink_repo` 至 `PYTHONPATH`，保留本分支硬體初始化 |

## 3. 執行流程

### 3.1 modeld（`selfdrive/modeld/modeld.py`）

1. `ModelState` 新增 `lat_delay`、`frame_drop_ratio`、`in_control`（joining model 每幀寫入）。
2. `run()` 簽名改為 `run(bufs, transforms, inputs, after_enqueue=None, *, prepare_only=False)`：jetlink 的 joining model 以第 4 個位置參數傳 `after_enqueue`；dp 的 `prepare_only` 改成 keyword。
3. `config_realtime_process(7, 54)` 之前先 `jetlink_adapter.prepare()`：tinygrad 的 device 執行緒必須在變成 SCHED_FIFO 前建立，否則會繼承 core 7 / FIFO 54 搶 frame loop。
4. 小模型載入後 `jetlink_adapter.attach()`；有 USB GPU（chestnut）時完全不接 jetlink。
5. 每幀：`inputs['action_t']`（大模型需要；dp 小模型忽略）、`model.in_control`、`model.frame_drop_ratio`；`handovers` 變化時重置掉幀濾波器（切換的停頓不算 lag）。
6. **掉幀路徑（dp 分歧）**：dp 在掉幀時只做 `prepare_only`（推進佇列、不發布）。小模型駕駛時維持原行為；**大模型駕駛時照常推論每一幀**（歷史在外部裝置，與 upstream 新版 modeld 相同）。
7. `modelExt.acceleratorState` / `modelExt.bigModel` 發布（zoompilot 為 `modelDataV2SP.acceleratorState` / `modelV2.big`）。

### 3.2 jetlinkd（gadget owner）

- `JetlinkLink != 0` 且沒有 chestnut 時由 manager 常駐（含行車中；owner 一離開 gadget 就會掉線）。
- 透過 `jetlink-root.sh` 建立 configfs gadget（VID 0x1209 / PID 0x0001），開 FunctionFS ep0、綁 UDC（`a600000.dwc3`）。iOS 模式多一個 CDC-NCM 介面 + dnsmasq。
- 熄火時發起 provisioning：從 comma 的 Git LFS 下載預設大模型 ONNX → USB 上傳 → 外部裝置建 TensorRT/CoreML 引擎（Jetson 約 2–5 分鐘）→ 寫入 `JetlinkSpec`（持久，避免每次點火重建）。
- 狀態檔：`/dev/shm/jetlink*`、`/dev/shm/jetlink/status.json`；日誌：`/data/log/jetlink-owner.log`。

### 3.3 行車切換

| 狀態 (`acceleratorState`) | 畫面 | 說明 |
| --- | --- | --- |
| joining / retrying | 圖示閃爍、「連線中」 | 小模型駕駛，背景連線 |
| ready | 半透明綠、「待切換」 | 已就緒，等待「無控制」窗口；控制中時跳「大模型已就緒／關閉巡航主開關後重新啟用即可切換」3 秒 |
| running（bigModel=true） | 綠色、「大模型」 | 切換後 1 秒 no-entry「大模型切換中」，之後提示「大模型已啟用」 |
| 斷線/延遲 | 橘色或回到閃爍 | 控制中則警告「大模型連線中斷／改用小模型駕駛」5 秒，**不解除**，小模型從重置的歷史接手 |

## 4. 連線移植重點（特別注意）

1. **實體連線**：外部裝置接 comma 的 **USB-C 埠**（裝置模式控制器 `a600000.dwc3`），需 **USB 3 資料線**；comma 與外部裝置**各自供電**（comma 無法供電給 Jetson）。iPhone/Android 走同一個埠；iPhone 直連時預設**不**由 comma 供電（`JetlinkChargePhone`，部分手機被供電後會斷線）。
2. **ADB 與 Jetlink 互斥**：AGNOS 的 ADB gadget（g1）佔用同一個 UDC，`jetlink-root.sh` 不會搶。開啟 Jetlink 時 UI 會自動把 `AdbEnabled` 關掉並讓 ADB 開關變灰（在 UI params 執行緒做，所以任何方式設定 `JetlinkLink` 都生效）。**SSH 不受影響**。
3. **只能熄火切換模式**：gadget 只在停車時重組；設定頁的 Off/USB/iOS 在行車中變灰。
4. **sudo**：jetlink 以 `sudo -n bash jetlink_repo/scripts/comma/jetlink-root.sh` 執行 root 動作，AGNOS 的 comma 使用者需免密碼 sudo（原廠即是）。
5. **ALKA 對應 MADS（v4：與 zoompilot 一對一）**：zoompilot 的「控制中」＝ openpilot 啟用或 MADS 啟用（含暫停：檔位、車門、安全帶、煞車保持、駐車煞車、原廠 LKAS 關），MADS 由 ACC 主開關關閉（或 MADS 按鈕）釋放。dp 的 `lkasOn` 在所有品牌都追蹤 ACC 主開關（opendbc 各 carstate 的 `lkas_on = cruiseState.available`，panda `toyota.h` 亦同），所以一對一對應為：openpilot 啟用、ALKA 轉向中（`latActive`/`alkaActive`），或「ALKA 開啟且 `lkasOn`」（ALKA 暫停也算）。關掉 ACC 主開關即開放切換，與 zoompilot 相同。ALKA 沒有獨立按鈕，主開關是唯一關閉方式（zoompilot 另可按 MADS 按鈕）。`controlsd` 在大模型換上後 1.2 秒內不讓 ALKA 由關轉開，對應 zoompilot 讓 MADS 在 bigModelLoading 期間停在暫停。訊號過期一律視為控制中。
6. **warp 是建置產物**：`scons` 會在 comma 上編好 warp pickle；沒有 warp 時 Jetlink 開啟會出現離線警示「Jetlink unavailable: no warp built for this camera」，行車只用小模型。
7. **大模型選單（v2 新增，從手機取得）**：預設「跟隨外部裝置」——手機（或 Jetson/Mac）App 上選的、目前載入的模型就是行車用的大模型；外部裝置每次 hello 都回報 `loaded`（載入中）與 `cached_models`（已建置）的 sha256，jetlinkd 記錄在 `/dev/shm/jetlink/server.json`。comma 端「更新模型清單」會抓與 App 相同的 sunnypilot chestnut 模型庫（實測 16 個大模型）並解析每個模型的 sha256，用來替手機上的模型命名；也可在 comma 上指定某個模型（外部裝置已建置就直接載入，沒有才由 comma 停車時下載上傳）。跟隨模式下 comma 不會下載任何模型。
8. **外部裝置版本**：comma 端是 jetlink **0.8.5（API 2）**。Jetson/Linux 的安裝腳本取 jetlink `main`，若協定版本跳號會變成「只有小模型」；請安裝與 0.8.5 相容的 jetlink-server（Mac/iPhone App 同理）。
9. **網路**：provisioning 需要 comma 熄火時有網路（下載 ONNX，約 0.8–1.75 GB，存在 `/data/media/0/models/jetlink/`）。

## 5. 與 zoompilot 的刻意差異（Intentional divergences）

| # | 項目 | zoompilot | dp0111pre 移植 | 原因 |
| --- | --- | --- | --- | --- |
| D1 | 模型管理 | sunnypilot model manager 的大模型槽與 catalog | dp 自有 `JetlinkBigModel`（未設＝跟隨外部裝置）與 `JetlinkCatalog`（從 sunnypilot 模型庫抓，未抓前內建 Cinque Terre V3）；`catalog_selector=19` | dp 無 model manager；模型選單改從手機回報 + 同一模型庫 |
| D2 | modeld 結構 | upstream 新版單一 `run_model` JIT | dp 兩段 JIT（`warp_enqueue` + `run_policy`）；`make_warp()` 由 dp 的 `make_frame_prepare` 組出無狀態 warp | dp 沒有獨立 warp |
| D3 | 狀態發布 | `modelDataV2SP.acceleratorState`、`modelV2.big` | `modelExt.acceleratorState`、`modelExt.bigModel` | dp 無 SP 結構；擴充既有 `ModelExt` |
| D4 | 事件 | `bigModelLoading` 原生，其餘 `OnroadEventSP` | 4 個都加在原生 `EventName`（@100–@103），文字繁中 | dp 無 OnroadEventSP |
| D5 | 控制判定 | MADS 啟用（含暫停）；主開關關閉釋放 | ALKA 開啟且 lkasOn（＝ACC 主開關）或轉向中；切換後 1.2 s 內 ALKA 不起動（§4-5） | v4 與 zoompilot 一對一；僅差 ALKA 無獨立按鈕 |
| D6 | 掉幀 | 每幀都跑 | 小模型沿用 dp `prepare_only`；大模型每幀跑 | 保持 dp 小模型行為 |
| D7 | 動作函式（v3 已比照） | 有 `action` 輸出時直接讀，否則用 plan；`should_stop` 看 v_ego | 大模型駕駛的幀改用 `get_action_from_model_jetlink`（zoompilot 函式照搬，4000 組隨機輸入與 zoompilot 原函式逐位元相同）；小模型仍用 dp 版。大模型幀**不套** `dp_lat_offset_cm`（zoompilot 沒有） | 比照 zoompilot |
| D8 | chestnut 偵測 | `deviceState.chestnutPresent` | sysfs 掃描同一組 USB ID；UI 以 `UsbGpuPresent` 讓位 | dp 無該欄位 |
| D9 | 模型存放 | `Paths.model_root()/jetlink` | `/data/media/0/models/jetlink` | dp 無 `model_root()` |
| D10 | jetlink `warp.py` | 原版 | 3 處相容：`CapturedJit.linear` vs `_linear`；`as_memoryview(no_sync=)` 不存在時退回；無 `tinygrad.engine.worker` 時略過。另：QCOM 的 graph-only 快速路徑失敗時改為重播整個 capture（多約 0.6 ms），而非拒絕連線 | dp 的 tinygrad `eecd4706`（2026-05-25） |
| D11 | 設定 UI | Models 面板內一列 | 獨立「Jetlink」面板 + 即時連線列 | dp 無 Models 面板 |
| D12 | 行車 UI（tici） | 只有側欄圖示 | 側欄圖示 + 狀態卡 + 行車 HUD 徽章 | 需求：行車介面顯示連線狀況 |
| D14 | jetlink 套件連線層 | 原版 | `lending.SERVER_FIELDS` 多記 `loaded`/`cached_models`；`link.open_link` 跟隨外部裝置載入的模型；`stand_in` 對無法得知大小的已載入模型以 0 詢問（伺服器以 sha 回答）；`provision` 在外部裝置已有模型時不下載、跟隨模式停車時不 provisioning；provisioning 先 hello 再決定：外部裝置已載入模型就只確認該模型（不換模型、不需網路），有模型但未載入就交給它自己選，完全沒有模型才給預設（v7，修正開機後第一次 provisioning 把手機模型換掉） | 從手機取得模型選單 |
| D13 | 參考 modeld 的手機前車融合 | — | **未移植** `apply_phone_leads`、`LeadPriorityGuard`、`jetlink.vision`、`jetlink_context` | 這些不在 zoompilot 原始碼中（屬自製手機前車方案），無法依原始碼移植 |

## 6. 驗證

離線（雲端容器，dp 釘選的 tinygrad `eecd4706`，CPU 後端）50 項全數通過；另以實網路抓取模型庫成功（16 個大模型，並解析出 Lebowski 的 sha256 與 1675 MB）：

- 轉接層對 `jetlink.openpilot.interface` 的 `StatusSide/WorkerSide/ModelSide/BuildSide/Openpilot` conformance：0 問題；API 版本 2 = 2。
- hook：link off 時 `should_run=False`、`prepare=False`、`attach=None`；link=usb 時 `enabled=True`，`reason()` 正確回報「no warp built for this camera」；預設模型名稱 Cinque Terre V3 Model。
- `in_control`：無控制/過期/啟用/ALKA 作動/ALKA+LKAS/僅 LKAS 六種情境。
- warp：以 dp `make_frame_prepare` 組成並由 jetlink 自己的 `compile_warp → Warps.load → Warp` 流程執行，輸入名稱 `['big_frame','big_tfm','frame','tfm']`，輸出 393216 bytes，**與 dp 原生 frame_prepare 逐位元相同（max diff 0）**。
- joining model 套在 dp 形狀的 ModelState：`after_enqueue` 以位置參數傳遞、寫入同步到小模型、`prepare_reset` 正確清空 dp 佇列。
- capnp schema 可編譯；所有修改 Python 檔可編譯，ruff F/E9 沒有新增問題（既有 5 項與原始樹相同）。

**未能在容器驗證（需實車/實機）**：QCOM 上的 IO-coherent 記憶體與 graph-only 快速路徑、FunctionFS gadget、PD 角色切換、與外部裝置的實際連線、大模型輸出經 dp `fill_model_msg` 的完整欄位。

## 7. 安裝與測試步驟

1. 覆蓋檔案（保留路徑），確認 `jetlink_repo/jetlink` 存在；無需建立根目錄 `jetlink`。
2. 重開機讓 scons 建置（會看到 `[JETLINK WARP] .../warp_1928x1208_512x256_tinygrad.pkl`）。
3. 外部裝置依 jetlink 文件安裝（Jetson：`docs/jetson.md`；Mac App；iPhone TestFlight）。
4. 熄火狀態：設定 → Jetlink → USB（iPhone 選 iOS）。ADB 會自動關閉。
5. 看側欄圖示：閃爍＝下載/建置中；綠＝就緒；橘＝看離線警示。
6. 上路：先小模型；關閉巡航主開關（ALKA 隨之關閉）→ 出現「大模型切換中」→「大模型已啟用」後再 engage。
7. 問題排查：`tail -n 200 /data/log/jetlink-owner.log`、`sudo /data/openpilot/jetlink_repo/scripts/comma/jetlink-root.sh check`、rlog 中搜尋 `jetlink:`。
8. 回復：設定 → Jetlink → 關閉。

## 8. 已知風險

- **QCOM 路徑未實機驗證**：若 warp 在 comma 上失敗，log 有「jetlink load failed」，modeld 安全地維持小模型。
- 跟隨模式依賴外部裝置回報的 `loaded`；手機若完全沒有任何模型，才回到預設模型（Cinque Terre V3），由 comma 停車時下載上傳。手機有模型但當下未載入（切換中）時 comma 不介入，下一輪再看。
- 大模型幀不套 dp 車道偏移（`dp_lat_offset_cm`）；切回小模型時偏移恢復，橫向位置可能在切換瞬間略有差異。
- `log.capnp` 的 EventName 新增序號 @100–@103；之後與 upstream 合併若 upstream 也新增事件，需調整序號。
