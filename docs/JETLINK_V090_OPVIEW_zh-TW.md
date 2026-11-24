# dp0111jetlink：Jetlink 更新到 v0.9.0（API 4）＋ 預設開啟並完整相容 opview

對象分支：herizon1054/openpilot `dp0111jetlink`（上傳的 openpilot-dp0111jetlink.zip，與 GitHub 上 84acc0c 相同）。
參考來源：
- Jetlink：zoompilot/jetlink `v0.9.0`（= 上傳的 jetlink-main.zip，`jetlink/` 套件逐位元相同）
- opview：opview v2.0.15 的 `OPVIEW_PORTING_zh-TW.md`、`docs/comma_reference/dragonpilot/webrtcd.py`、
  `docs/comma_reference/sunnypilot/webrtcd.py`；herizon1054/openpilot `dp0111pre`（已支援 opview 的 dp 分支）

---

## 1. Jetlink v0.8.10 → v0.9.0

### 1.1 上游改了什麼（影響 comma 端的部分）

| 項目 | v0.8.10 | v0.9.0 |
|---|---|---|
| jetlink.openpilot `API` | 3 | **4**（API 3 再保留一個小版本） |
| 大模型清單來源 | `jetlink/registry/catalog.py`：抓 sunnypilot chestnut catalog，再逐一向 comma LFS 解析 sha256 | **刪除 catalog.py**，改用 `jetlink/registry/index.py`：zoompilot/jetlink-model-index（每小時更新，直接帶每個模型的 oid / size） |
| comma 端模型索引 | 無 | `<model root>/jetlink/index.json`，離線也能命名、計算大小、解析 pick |
| 雙向 pick（v0.8.11–0.8.13） | 無 | hello 帶 comma 的 `pick`，伺服器（手機 / Jetson）回較新的 pick；伺服器較新的 pick 會直接驅動（`Models.adopt`） |
| 清空大模型欄位 | 視為沒選 | 視為「選了預設模型」（`slot.json` 記錄清空時間），會當成較新的 pick 送給伺服器 |
| 進度代碼 | — | 新增 `download`（伺服器自己下載模型） |
| hello 回覆欄位（lending.SERVER_FIELDS） | … `sleep_after` | 新增 `pick`、`fetches` |

### 1.2 做法

`jetlink_repo/` 換成 v0.9.0（只保留 comma 需要的 `jetlink/`、`LICENSE`、`pyproject.toml`、`README.md`、
`scripts/comma/jetlink-root.sh`，與原本相同的子集），dp 的修改用三方合併（base = v0.8.10、ours = 原分支、theirs = v0.9.0）重新套上。

| 檔案 | dp 修改 | 合併結果 |
|---|---|---|
| `jetlink/comma/owner.py`、`openpilot/settings.py` | 一律不替手機充電 | 自動合併 |
| `jetlink/openpilot/joining.py` | keepalive 斷線不累加失敗次數、快速重連 | 自動合併 |
| `jetlink/openpilot/models.py` | 下載加 `cache_lock`（配合刪除下載模型功能） | 自動合併 |
| `jetlink/comma/lending.py` | SERVER_FIELDS 多記 `loaded`、`cached_models` | 衝突 → 上游的 `pick`、`fetches` 與 dp 的兩個欄位**全部保留** |
| `jetlink/openpilot/link.py` | 跟隨外接裝置（follow far end） | 衝突 → 跟隨時用手機已載入的模型；不跟隨時走上游新流程（`adopt()` → `selected_model()` → `identity()`） |
| `jetlink/openpilot/provision.py` | 跟隨外接裝置（停車時不替手機下載 / 建模型） | 衝突 → 上游把 hello 移到最前面；dp 的跟隨判斷改放在 hello 之後、`adopt()` 之前，不再重複 hello；`run()` 的跟隨判斷放在上游新的「伺服器有新 pick 就問一次」條件之前 |
| `jetlink/openpilot/model_cache.py` | dp 新增（刪除下載模型） | 原樣保留（只動 `*.onnx`，不會刪到新的 `index.json` / `slot.json`） |

**新增的 dp 修改（與上游不同，刻意）**：`jetlink/openpilot/models.py` 的 `Models.default_chosen_at()`

- dp 的「大模型欄位空白 = 跟隨外接裝置」。v0.9.0 會把「清空欄位」當成「選了預設模型」，下一次 hello 就把
  Cinque Terre V3 當成比手機更新的 pick 送出，手機會換掉使用者載入的模型（就是 2026-10-10 手機 log 那個問題的新版）。
- 修改：`follow_far_end()` 成立時 `default_chosen_at()` 回 None → `hello_pick()` 回 None → 伺服器自己的 pick 維持。
- 已用替身測試確認：跟隨時清空 → `hello_pick() is None`；不跟隨時清空 → 照上游送出預設模型。

### 1.3 dp adapter（`dragonpilot/jetlink_adapter/__init__.py`）

- `API = 4`，`COMPATIBLE_APIS = {4, 3}`（`_bind()` 改用 `in`）。
- `refresh_catalog()`：`jetlink.registry.catalog` 已不存在，改用 `jetlink.openpilot.models.Models(Adapter()).big_catalog(19)`：
  抓模型索引、寫 jetlink 自己的 `index.json`、產生 dp 的 `JetlinkCatalog`；`JetlinkModelPointers` 直接由索引的
  oid / size 填入，不再對每個模型各打一次 LFS（「更新模型清單」按鈕變快）。
- `model_menu()`：pointers 參數缺某個模型時，從 jetlink 的 `index.json`（離線）補 sha256 / size。
- `_Absent`：補上 API 4 的 `lists_big_models()`、`big_catalog()`、`listed_ref()`（回答「連線關閉」）。
- `KEYS` 不變（`pointers` 在 v0.9.0 的 `Keys` 仍保留一個小版本）；`catalog_selector` 留著無害。
- dp 沒有 sunnypilot 模型管理器、沒有 `Keys.select`，所以 `adopt()` 不會改寫 `JetlinkBigModel`：
  手機上選的模型照樣驅動，comma 的選單仍以「跟隨外接裝置」顯示。

### 1.4 驗證

- jetlink v0.9.0 上游 Python 測試套用到合併後的套件：**1257 passed**，6 failed 全部是預期差異：
  - 3 項：dp 刻意多記 `loaded` / `cached_models`、刻意不充電
  - 3 項：容器沒有 Swift 原始碼 / 狀態頁 HTML（測試讀不到檔案）
- `tools/jetlink/test_port.py`（用 GitHub 上含 `selfdrive/modeld` 的完整分支跑）：7/8 通過。
  - 修正：`test_old_and_new_memoryview_api` 指向 `dragonpilot/jetlink_adapter/warp.py`（函式早就搬到那裡，原分支這項就失敗）
  - `test_mici_hud_draw_path` 原分支就失敗（測試替身少 `JetlinkState.DISCONNECTED`），與本次無關，未修改

### 1.5 注意

- **手機端 Jetlink App 也要更新到 v0.9.0**（上游：comma 與 App 更新順序不限）。
- 大模型清單第一次要按設定頁「更新模型清單」（需要網路，連 raw.githubusercontent.com），之後離線可用。
- 舊的 `jetlink_repo/jetlink/registry/catalog.py` 覆蓋後請刪除（v0.9.0 已沒有，留著不會被載入，但會混淆）。

---

## 2. opview：預設開啟 + 完整相容

原分支的 webrtcd 是比 dp0111pre 舊的版本（缺 orientationRate / leadsV3、欄位精簡、雷達點、eGPU、限速），
也沒有 :5088 的實驗模式端點、沒有 720p 串流。依 `OPVIEW_PORTING_zh-TW.md` 逐項處理：

| # | 項目 | 狀態 | 檔案 |
|---|---|---|---|
| A1 / A2 / A3 | 聽 0.0.0.0:5001、`/schema`、`initCamera` 請求格式 | dp 原生已有 | — |
| A4 | 開關 + manager 啟動 | **`dp_dev_opview` 預設改為 1** | `common/params_keys.h`、`dragonpilot/settings/min-feat.dev.opview.{yaml,py}` |
| A6 | `connectionReplaced` | 已有 | — |
| A8 | webrtcd / stream_encoderd 當掉不擋啟用 | **新增**（opview 預設開啟後更需要） | `selfdrive/selfdrived/selfdrived.py` |
| B1 | 串流 720p（tizi 1152×720、mici 1280×720、tici 上限 1440 寬） | **新增**（取自 dp0111pre） | `system/loggerd/loggerd.h` |
| B2 / B3 | 動態位元率、同連線換鏡頭 | 已有 | — |
| C1 | modelV2 精簡＋ `orientationRate.z[0]`、`leadsV3` | **新增** | `system/webrtc/webrtcd.py` |
| C2 / D4 | 其他 service 只送 opview 讀的欄位（含 `status` / `present`、`personality`） | **新增** | 同上 |
| C3 | 每個 service ≤ 33 Hz | **新增** | 同上 |
| C4 | 壅塞丟舊資料（`bufferedAmount > 256 KB`） | 已有 | — |
| D1 | `radarState.opviewTracks`＋**measured 退路**＋`opviewTracksSrc` 診斷 | **新增**（採 sp 寫法，非 dp0111pre 的 `if p.measured`） | 同上 |
| D3 | `deviceState` MEM / CPU / TEMP＋`opviewEgpu` | **新增，並改為反映 Jetlink**（見 2.1） | 同上 |
| D5 / E3 | Auto Hold | 端點與參數鍵已加；**本分支沒有豐田自動煞車維持**，狀態恆為「不可用」，opview 不顯示按鈕 | `serverd.py`、`params_keys.h` |
| E1 | `/api/init`、`/api/models`、`/api/models/select` | 已有（dashy serverd） | — |
| E2 | `GET/POST /api/experimental_mode` | **新增**（取自 dp0111pre，條件比照設定頁） | `dragonpilot/dashy/serverd.py` |
| F2 | 請求帶 `tdx` 時拿掉 | **新增** | `system/webrtc/webrtcd.py` |
| F3 | 區網禁止 `bridge_services_in`（403） | **新增**（dp0111pre 也沒有，刻意補上） | 同上 |
| — | `initCamera` 不是 road / wideRoad / driver → 400 | **新增**（移植指南 3.3） | 同上 |

### 2.1 `deviceState.opviewEgpu` 在 Jetlink 分支（與 dp0111pre 不同，刻意）

dp0111pre 只看 `UsbGpuPresent` / `UsbGpuCompiled`。本分支的大模型多半由 Jetlink 計算，所以：

- 有 USB GPU（chestnut）→ 與 dp0111pre 相同。
- 沒有 USB GPU、`JetlinkLink` ≠ 0 → 每秒讀一次 `jetlink_adapter.status()`（只讀檔案與參數，不碰 USB 連線）：

| opviewEgpu | 來源 |
|---|---|
| `present` | `Status.present`（外接裝置在 USB gadget 上） |
| `active` | `Status.active`（大模型正在驅動 modeld） |
| `loading` | `Status.running` 為 joining / retrying，或停車準備中（progress 階段不是 ready / failed），且尚未 active |
| `compiled` | `Status.runnable`（引擎已建好） |
| `source` | `"jetlink"` / `"usbgpu"`（新增欄位，舊版 opview 會忽略） |

jetlink 不存在或出錯時退回 USB GPU 的讀法，不丟例外。

### 2.2 預設開啟的意思

- `dp_dev_opview` 預設值改為 `"1"`：新裝置、或從未切過這個開關的裝置（參數檔不存在）開機即開啟。
- **已經存過 `0` 的裝置維持使用者的設定**（manager 只補不存在的參數），要開啟請到設定頁打開，或
  `echo -n 1 > /data/params/d/dp_dev_opview`。
- 開啟後行車中會多跑 webrtcd 與 stream_encoderd（多佔一組硬體編碼器與一些 CPU，與 Jetlink 同時使用時請留意溫度）。
  A8 已讓它們當掉不會擋住啟用。

### 2.3 測試

- `system/webrtc/tests/test_opview_compact.py`（dp0111pre 原有 4 項＋新增 6 項：雷達點 measured 退路、診斷、
  F3 的本機判斷、eGPU 的 Jetlink / USB GPU 兩種來源）
- `system/webrtc/tests/test_stream_session.py`：新增 tdx 不轉送
- `dragonpilot/dashy/tests/test_experimental_mode.py`、`test_auto_hold.py`（dp0111pre 原有）
- 容器內已用替身驗證：雷達點退路、eGPU 對照、`get_stream` 區網帶 `bridge_services_in` → 403、錯誤鏡頭 → 400、
  serverd 兩組測試 7 項通過、全樹 642 個 Python 檔語法解析通過。

---

## 3. 尚需實機確認

容器沒有 capnp 建置、msgq、tinygrad、QCOM 裝置與 USB/PD 硬體，以下只能上車確認：

1. scons 建置（`params_keys.h`、`loggerd.h` 有改，需重新編譯；不要用跳過建置的 prebuilt 標記）。
2. 停車：設定頁 Jetlink「更新模型清單」→ 清單出現、`/data/media/0/models/jetlink/index.json` 產生；
   owner log（`/data/log/jetlink-owner.log`）沒有 API / import 錯誤。
3. 跟隨外接裝置：手機 App 換模型 → comma 下次連線跟著換；comma 選單切回「跟隨」後，手機**不會**被換成 CTV3。
4. opview：熱點下連線 10 分鐘以上不卡；橫向 HUD 的 eGPU 欄在 Jetlink 連線 / 大模型啟用時正確；
   opview log「雷達點（60 秒）」有點數；實驗模式按鈕可切換。
5. comma 端驗證指令：
   ```bash
   cat /data/params/d/dp_dev_opview                       # 1
   ps aux | grep -E "webrtcd|encoderd --stream|serverd|jetlinkd" | grep -v grep
   curl -s "http://<comma IP>:5001/schema?services=carState" | head -c 200
   curl -s http://<comma IP>:5088/api/experimental_mode
   curl -s http://<comma IP>:5088/api/auto_hold          # available 恆為 false（本分支沒有 Auto Hold）
   ```

## 4. 修改檔案一覽

```
common/params_keys.h
docs/JETLINK_V090_OPVIEW_zh-TW.md                         （新增，本文件）
dragonpilot/dashy/serverd.py
dragonpilot/dashy/tests/__init__.py                       （新增）
dragonpilot/dashy/tests/test_auto_hold.py                 （新增）
dragonpilot/dashy/tests/test_experimental_mode.py         （新增）
dragonpilot/jetlink_adapter/__init__.py
dragonpilot/settings/min-feat.dev.opview.py
dragonpilot/settings/min-feat.dev.opview.yaml
jetlink_repo/**                                           （整個換成 v0.9.0 + dp 修改；刪除 jetlink/registry/catalog.py）
selfdrive/selfdrived/selfdrived.py
system/loggerd/loggerd.h
system/webrtc/webrtcd.py
system/webrtc/tests/test_opview_compact.py                （新增）
system/webrtc/tests/test_stream_session.py
tools/jetlink/test_port.py
```
