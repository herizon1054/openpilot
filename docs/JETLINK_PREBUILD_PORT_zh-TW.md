# dragonpilot-pre-build：依 Claude 已可用版本移植 Jetlink

本次以使用者提供的 dragonpilot-pre-build.zip 為目標，將 openpilot-dp0111pre-jetlink-final (2).zip 中的 Jetlink 功能合併。只修改 comma 端，未新增低功耗模式。

## 合併原則及差異

- jetlink_repo 內所有來源檔案與 Claude 版本逐位元一致，保留 tinygrad 記憶體 API、CapturedJit 及完整 capture 回退相容處理，以及手機模型跟隨與既有握手流程。沒有混入先前 v9 的握手修改。
- 本分支保留原有控制邏輯、扭矩條與硬體初始化；不導入 Claude 舊分支的 HTD／LCC／TDX 等無關改動。
- 全部 Jetlink 核心位於 jetlink_repo。啟動腳本、adapter 子程序環境及建置流程提供套件路徑，不需要根目錄 jetlink symlink。
- modelExt 發布明確設定 valid=True；本分支訊息建構預設 valid=False，未指定會讓訂閱者將模型狀態視為無效。
- 移植 tici 設定頁、側欄與行車徽章，以及 mici 設定切換、首頁圖示與行車模型狀態。

## 本次已執行驗證

1. 全樹 Python 語法解析、修改檔衝突標記檢查、啟動與 root helper 的 bash 語法檢查通過。
2. 實際匯入 adapter 後，jetlink 解析到本分支 jetlink_repo/jetlink。
3. 套用本分支核心的 joining／warp／interface／status 回歸測試：168 項與 45 子測試通過。其中上游「非預期 capture 應拒絕」測試改為驗證 Claude 的「完整 capture 回放」行為，並檢查傳入的 transforms；沒有略過該案例。
4. 包內 tools/jetlink/test_port.py：7 項通過，涵蓋新舊 memoryview API、modelExt 有效旗標、action head、控制切換條件、事件與兩種 HUD 繪製方法。可在有 numpy 的環境執行 python3 tools/jetlink/test_port.py。
5. HUD 測試使用繪圖替身，不代表螢幕實際顯示驗證。

## 尚需實機確認

容器沒有完整 tinygrad 子模組、QCOM 裝置、capnp 建置環境及 tici／mici。尚未完成裝置上的 scons、USB/PD 握手、實際大模型推論、行車切換及 HUD 視覺驗證。不能將離線測試結果視為實車驗證。

本包為完整分支，不是 Git 提交；保留原 ZIP 的子模組／LFS 狀態，沒有把原分支缺少的子模組與大型模型下載成實體檔。warp pickle 仍由裝置 scons 建置。更新時勿使用跳過建置的 prebuilt 標記。套用後先在停車狀態確認建置、手機幀數、owner log 與 modelExt 狀態，再檢查行車切換。

## 相對原目標修改的既有檔案

- `cereal/custom.capnp`
- `cereal/log.capnp`
- `common/params_keys.h`
- `launch_chffrplus.sh`
- `selfdrive/controls/controlsd.py`
- `selfdrive/modeld/SConscript`
- `selfdrive/modeld/modeld.py`
- `selfdrive/selfdrived/alerts_offroad.json`
- `selfdrive/selfdrived/events.py`
- `selfdrive/selfdrived/selfdrived.py`
- `selfdrive/ui/layouts/settings/developer.py`
- `selfdrive/ui/layouts/settings/settings.py`
- `selfdrive/ui/layouts/sidebar.py`
- `selfdrive/ui/mici/layouts/home.py`
- `selfdrive/ui/mici/layouts/settings/developer.py`
- `selfdrive/ui/mici/layouts/settings/toggles.py`
- `selfdrive/ui/mici/onroad/hud_renderer.py`
- `selfdrive/ui/onroad/hud_renderer.py`
- `selfdrive/ui/ui_state.py`
- `system/hardware/hardwared.py`
- `system/manager/process_config.py`
- `system/ui/widgets/icon_widget.py`
