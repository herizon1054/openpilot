# v10 字型缺字修正

照片中「源、擇、外、部、建、引、擎」等字變成問號，可由原字型產生器的字集選擇重現。

根因：UI 已直接使用 dragonpilot_*.po，但 process.py 仍只從 dragonpilot_*.mo 收集字元。OTF 有字元不代表產生的 FNT/PNG 圖集包含字元；之前僅檢查 OTF 字元表並不足夠。

修正 selfdrive/assets/fonts/process.py：直接讀取同一份 PO 收集字元，不依賴過時或不存在的 MO。
修正 selfdrive/ui/SConscript：追蹤兩個翻譯域的 PO 與 languages.json；依照生成器實際輸出的 OpFont 各語言及 Labels 圖集宣告目標，不再要求沒有生成的 OpFont.fnt 或 unifont.fnt。

保留 v10 所有連線修正及多國語言功能。新增 3 項字集／建置規則測試通過，7 項翻譯測試再次通過。未在本環境執行 raylib 光柵化或實機 UI。

更新後讓正常啟動建置執行，勿以 prebuilt 跳過。若要手動重建，請在停車時透過 SSH 執行：

```sh
cd /data/openpilot
python3 selfdrive/assets/fonts/process.py
```

確認沒有錯誤後重新啟動裝置，讓 UI 載入新的 FNT/PNG。只替換 PO 或 MO 不會修復已生成的缺字圖集。
