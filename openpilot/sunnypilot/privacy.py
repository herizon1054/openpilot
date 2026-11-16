"""
隱私模式（PrivacyMode，預設關閉）

裝置 ID：隱私模式不阻擋 sunnylink 取得裝置 ID。sunnylink 註冊需要 comma 的 DongleId
（sunnylink/api.py register_device：comma_dongle_id 為 None 就不註冊），所以 manager 照常呼叫 register()：
本機有 ID（DongleId 參數或 /persist/comma/dongle_id）時只讀本機；完全沒有時才連 comma 註冊一次（v2/pilotauth）。

開啟時：
  comma.ai
    - 裝置註冊照常（見上）：本機已有 ID 時不連 comma
    - 不執行 athenad（comma 遠端連線：遠端取檔、即時影像、讀取 GPS、SSH 代理…），已在跑的 daemon 會被終止
    - 不執行 uploader（行車紀錄 / 影片上傳到 comma）
    - 不執行 statsd（comma 統計資料）
    - CommaConnectApi 一律拒絕連線（UI 的 prime 狀態、firehose、行程統計都不再向 comma 查詢）
  sunnylink（保留設定網頁與備份）
    - 不執行 statsd_sp（車輛參數 / 設定統計）、sunnylink_uploader
    - sunnylinkd 不上傳檔案、不轉送 log、不上傳統計
    - sunnylinkd 拒絕遠端呼叫：取檔 / 上傳、即時影像、讀 cereal 訊息（GPS 等）、列目錄、SSH 代理、SIM 資訊
      （裝置公鑰 getPublicKey 不擋：是裝置身分的一部分，註冊時本來就送給 sunnylink）
    - getParams 不回傳位置相關參數；PrivacyMode 本身不能從 sunnylink 遠端關閉

  車內與錄影（每次發動時鎖定，行駛中切換於下次發動生效）
    - 照常錄道路 / 廣角 / qcamera 影片（只存在本機，不上傳）；不寫車內鏡頭影片（dcamera.hevc，
      即使 RecordFront 開啟）、不錄車內聲音（RecordAudio：qcamera 混音與 rawAudioData）
      → loggerd.h（cabin .record、qcam .include_audio）、loggerd.cc（record_audio）讀 PrivacyMode
    - 不執行 dmonitoringmodeld：車內鏡頭畫面不做任何分析
    - dmonitoringd 不讀車內鏡頭，並停用駕駛監控（wheeltouch_step 的 driver_engaged 固定為 True，
      awareness 永遠 1.0、不發任何警示；仍持續送出 driverMonitoringState，selfdrived 不會報通訊錯誤）
    - webrtcd 拒絕車內鏡頭串流（opview / sunnylink 都一樣）
  opview 用的 stream_encoderd 只編碼要求的道路鏡頭、不寫檔，不受影響。

comma 雲端 / sunnylink 部分切換後需重新開機才完全生效。
"""
import os
import signal

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

PRIVACY_PARAM = "PrivacyMode"

# 隱私模式仍放行的 comma.ai API：裝置註冊（只在本機沒有任何裝置 ID 時才呼叫；sunnylink 需要 DongleId）
PRIVACY_ALLOWED_ENDPOINTS = frozenset({"v2/pilotauth/"})

# manager 不啟動的程序
COMMA_CLOUD_PROCS = ("manage_athenad", "uploader", "statsd")
SUNNYLINK_TELEMETRY_PROCS = ("statsd_sp", "sunnylink_uploader")
PRIVACY_BLOCKED_PROCS = COMMA_CLOUD_PROCS + SUNNYLINK_TELEMETRY_PROCS

# sunnylinkd 拒絕的遠端方法（athenad 的方法透過共用 dispatcher 也會出現在 sunnylinkd）
SUNNYLINK_BLOCKED_METHODS = frozenset({
  "getMessage",          # 讀任何 cereal 訊息：gpsLocation、liveLocationKalman、carState…
  "startStream",         # WebRTC 即時影像
  "listDataDirectory",   # 列出行車紀錄 / 影片
  "uploadFileToUrl",     # 上傳行車紀錄 / 影片
  "uploadFilesToUrls",
  "listUploadQueue",
  "cancelUpload",
  "setRouteViewed",
  "startLocalProxy",     # 遠端 SSH 代理
  "setNavDestination",
  "getSimInfo",          # IMEI / ICCID
  "getSshAuthorizedKeys",
  "getGithubUsername",
  "takeSnapshot",        # 舊版 athenad 才有；保險起見一併列入
})

# sunnylinkd getParams 不回傳的參數（位置資訊）
SUNNYLINK_BLOCKED_READ_PARAMS = frozenset({
  "LastGPSPosition",
  "LastGPSPositionLLK",
  "LocationFilterInitialState",
})


_onroad_latch: bool | None = None


def privacy_mode_onroad(started: bool, params: Params) -> bool:
  """發動時鎖定一次隱私模式狀態，整趟行駛不變。
  行駛中切換開關不會讓 dmonitoringmodeld / dmonitoringd 中途不一致（例如 model 停了但 dmonitoringd 還在等車內鏡頭結果
  → selfdrived 通訊錯誤），下次發動才套用。"""
  global _onroad_latch
  if not started:
    _onroad_latch = None
    return privacy_mode(params)
  if _onroad_latch is None:
    _onroad_latch = privacy_mode(params)
  return _onroad_latch


def privacy_mode(params: Params | None = None) -> bool:
  try:
    return (params or Params()).get_bool(PRIVACY_PARAM)
  except Exception:
    # 參數讀不到時（例如 params_keys.h 尚未重新編譯）與預設值一致：關閉
    return False


def stop_athenad_daemon(params: Params) -> None:
  """manage_athenad 是 DaemonProcess：manager 重啟也不會停（stop() 是空的），這裡依 AthenadPid 終止。"""
  pid = params.get("AthenadPid")
  if pid is None:
    return
  try:
    pid = int(pid)
    with open(f"/proc/{pid}/cmdline") as f:
      if "athena" not in f.read():
        return
    try:
      os.killpg(pid, signal.SIGTERM)  # DaemonProcess 以 setpgrp 啟動，連同 athenad 子程序一起停
    except OSError:
      os.kill(pid, signal.SIGTERM)
    cloudlog.warning(f"privacy mode: stopped athenad daemon (pid {pid})")
  except (OSError, ValueError):
    pass
  finally:
    params.remove("AthenadPid")
