# Claude v10 合併至 pre-build 多國語言版

來源為本次上傳的完整 openpilot-dp0111pre-jetlink-final (3).zip，並與 jetlink-v10-changed-files.zip、增量 patch 核對。目標為上一份 dragonpilot-pre-build-jetlink-i18n.zip。

## 合併內容

1. 等待切換的連線若 keepalive ping 失敗，前 3 次在 1 秒後重試，第 4 次起在 5 秒後重試。成功 ping 會清除 keepalive 失敗計數。這個路徑不增加一般失敗次數；已切換模型後的一般故障退避仍保留。
2. ALKA 開啟且 ACC 主開關開啟時，P／N／R 檔不再單憑主開關阻擋切換。enabled／latActive／alkaActive 作動或控制判斷依賴訊息失效，仍禁止切換。
3. selfdrived 的切換事件判斷同步使用上述檔位條件。
4. 就緒警示改依 ALKA 狀態給出不同操作說明，並保留繁體／簡體翻譯。

## 與提供檔案的對照

- joining.py 與 Claude 完整檔逐位元一致。
- adapter 的 parked／in_control 函式 AST 與 Claude 一致；保留我們的 jetlink_repo 匯入路徑。
- 完整檔相對上次 Claude 檔還多了 _alka_on 與 big_model_available_alert；僅套用增量 patch 會遺漏其前置修改。本次兩者皆已合併。
- 不覆蓋新分支原有功能，不復原根目錄 jetlink symlink，不復原硬編碼繁體文字。
- 大模型就緒警示仍在訊息中使用英文來源字串，兩種 UI 在顯示時翻譯。

## 邊界與檢查

「parked」在這份修正中只代表 P／N／R 檔位，沒有車速門檻；不能將它解讀成程式已確認車輛完全靜止。本次保留 Claude 的判斷，未自行新增車速限制。

切換後的 ALKA_HOLD_SECONDS=1.2 保護與原有模型驗證期維持。

- 8 項移植測試通過，增加 P／N／R、D／未知檔位、控制作動、失效訊息、關閉例外開關，以及 keepalive 的 1／5 秒延遲與成功重置案例。
- 7 項翻譯測試通過，25 份 PO 編譯、更新流程及繁簡切換皆通過。
- 168 項 joining／warp／interface／status 回歸與 45 子測試通過。warp 測試沿用之前針對 Claude 完整 capture 回退修改的預期。
- Python 語法、完整／增量來源一致性及封裝內容檢查通過。

以上為離線測試，未驗證手機／Mac 實際握手、QCOM 推論及實車切換。前幾份移植說明為歷史紀錄，v10 差異以本文件為準。
