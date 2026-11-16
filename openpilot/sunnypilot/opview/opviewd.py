#!/usr/bin/env python3
"""
opviewd：給 opview（Android 手機 App）用的 comma 本機 HTTP 服務（port 5088）。

dragonpilot 系（dp0111pre）由 dashy serverd（:5088）提供車型讀寫；sunnypilot 沒有 dashy，
這裡只實作 opview 主畫面用到的三個端點，回應格式與 dp serverd 相同（opview comma_models_api.dart 不需分支判斷）：

  GET  /api/init            → {"branch": "sp", "opview": true, "isOffroad": bool}
                              （刻意不帶 dp_dev_dashy：opview 以 branch == "sp" 判斷為 sunnypilot，
                               裝置設定開 sunnylink 而不是 :5088 網頁）
  GET  /api/models          → {"model_list": {品牌: [車型名稱, …]}, "selected_model": "<車型名稱>" | "",
                               "apply_hint": "<設定後何時生效>"}
  POST /api/models/select   body {"selected_model": "<車型名稱>" | "[AUTO]" | ""}
  GET  /api/experimental_mode → {"experimental_mode": bool, "available": bool, "confirmed": bool}
  POST /api/experimental_mode   body {"enabled": bool} → 200 {"experimental_mode": bool} | 400 / 409 {"error": "<原因>"}
                              （opview 側欄第一顆按鈕；與 dp serverd 相同介面）

車型寫入 sunnypilot 的 CarPlatformBundle（與 設定 → 車輛 → 車型選擇 相同：{**car_list[name], "name": name}），
card.py 在每次 onroad 開始時讀取 → 不需重新開機，下次發動生效。

只在 OpviewEnabled 開啟時由 manager 啟動（offroad 也執行）。
"""
import argparse
import json
import logging
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections.abc import Callable
from typing import Any
from urllib.parse import urlparse

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params

OPVIEWD_PORT = 5088
AUTO_MODEL = "[AUTO]"
CAR_LIST_JSON = os.path.join(BASEDIR, "openpilot", "sunnypilot", "selfdrive", "car", "car_list.json")

HINT_OFFROAD = "下次發動（onroad）時生效，不需重新開機"
HINT_ONROAD = "行駛中變更：熄火後再次發動時生效"

logger = logging.getLogger("opviewd")


class CarList:
  """car_list.json（車型名稱 → {platform, make, model, year…}）；讀不到檔案時退回 CarList 參數。"""
  def __init__(self, params: Params, path: str = CAR_LIST_JSON):
    self.params = params
    self.path = path
    self._lock = threading.Lock()
    self._platforms: dict[str, dict] | None = None

  def platforms(self) -> dict[str, dict]:
    with self._lock:
      if self._platforms is None:
        data: Any = None
        try:
          with open(self.path) as f:
            data = json.load(f)
        except (OSError, ValueError):
          logger.exception("failed to read %s, falling back to CarList param", self.path)
          data = self.params.get("CarList")
        self._platforms = data if isinstance(data, dict) else {}
      return self._platforms

  def by_make(self) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for name, data in self.platforms().items():
      make = (data or {}).get("make") or "Other"
      out.setdefault(str(make), []).append(name)
    return {make: sorted(names) for make, names in sorted(out.items())}


def selected_model(params: Params) -> str:
  bundle = params.get("CarPlatformBundle")
  if isinstance(bundle, dict):
    return str(bundle.get("name") or bundle.get("platform") or "")
  return ""


def is_offroad(params: Params) -> bool:
  return params.get_bool("IsOffroad")


def handle_init(params: Params) -> tuple[int, dict]:
  return 200, {"branch": "sp", "opview": True, "isOffroad": is_offroad(params)}


def handle_models(params: Params, cars: CarList) -> tuple[int, dict]:
  return 200, {
    "model_list": cars.by_make(),
    "selected_model": selected_model(params),
    "apply_hint": HINT_OFFROAD if is_offroad(params) else HINT_ONROAD,
  }


def handle_select(params: Params, cars: CarList, payload: Any) -> tuple[int, dict]:
  if not isinstance(payload, dict):
    return 400, {"error": "bad request"}
  name = str(payload.get("selected_model") or "")
  if name in ("", AUTO_MODEL):
    params.remove("CarPlatformBundle")
    logger.warning("opviewd: CarPlatformBundle cleared (auto fingerprint)")
  else:
    data = cars.platforms().get(name)
    if data is None:
      return 400, {"error": "unknown_model", "message": f"unknown model {name!r}"}
    params.put("CarPlatformBundle", {**data, "name": name})
    logger.warning("opviewd: CarPlatformBundle set to %s (%s)", name, data.get("platform"))
  return 200, {"status": "success", "apply_hint": HINT_OFFROAD if is_offroad(params) else HINT_ONROAD}


# --- 實驗模式開關 ---
# 條件比照 openpilot 設定頁（selfdrive/ui/layouts/settings/toggles.py）：
#   - 需要 openpilot 縱向控制（ui_state.has_longitudinal_control 同邏輯：
#     alphaLongitudinalAvailable → AlphaLongitudinalEnabled，否則 CarParams.openpilotLongitudinalControl）
#   - 需要 ExperimentalModeConfirmed（設定頁的實驗模式說明確認；這裡不代替使用者確認）
#   - 關閉一律允許
# selfdrived 的 params_thread 每 0.1 秒重讀 ExperimentalMode，寫入後即生效（與原廠設定頁 / onroad 按鈕相同）。
def experimental_long_available(params: Params) -> bool | None:
  """openpilot 縱向控制是否可用；還沒有 CarParamsPersistent（從未發動）時回傳 None。"""
  cp_bytes = params.get("CarParamsPersistent")
  if not cp_bytes:
    return None
  from openpilot.cereal import messaging
  from opendbc.car.structs import car
  cp = messaging.log_from_bytes(cp_bytes, car.CarParams)
  if cp.alphaLongitudinalAvailable:
    return params.get_bool("AlphaLongitudinalEnabled")
  return bool(cp.openpilotLongitudinalControl)


def handle_experimental_status(params: Params, long_available: bool | None) -> tuple[int, dict]:
  return 200, {
    "experimental_mode": params.get_bool("ExperimentalMode"),
    "available": bool(long_available),
    "confirmed": params.get_bool("ExperimentalModeConfirmed"),
  }


def handle_experimental_mode(params: Params, payload: Any, long_available: bool | None) -> tuple[int, dict]:
  if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
    return 400, {"error": 'body 需為 {"enabled": true | false}'}
  enabled = payload["enabled"]
  if enabled:
    if long_available is None:
      return 409, {"error": "尚未取得車輛資訊，請先發動一次再開啟實驗模式"}
    if not long_available:
      return 409, {"error": "此車未使用 openpilot 縱向控制，無法開啟實驗模式"}
    if not params.get_bool("ExperimentalModeConfirmed"):
      return 409, {"error": "請先在 comma 設定頁開啟一次實驗模式並確認說明"}
  params.put_bool("ExperimentalMode", enabled)
  now = params.get_bool("ExperimentalMode")
  logger.warning("opviewd: ExperimentalMode=%s", now)
  return 200, {"experimental_mode": now}


class OpviewdHandler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"
  server: "OpviewdHTTPServer"

  def _send(self, status: int, obj: dict) -> None:
    body = json.dumps(obj).encode()
    self.send_response(status)
    self.send_header("Content-Type", "application/json; charset=utf-8")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    if self.command != "HEAD":
      self.wfile.write(body)

  def _read_json(self) -> Any:
    length = int(self.headers.get("Content-Length", 0))
    raw = self.rfile.read(length) if length else b""
    return json.loads(raw) if raw else None

  def _dispatch(self) -> None:
    path = urlparse(self.path).path.rstrip("/")
    params, cars = self.server.params, self.server.cars
    try:
      if self.command in ("GET", "HEAD") and path == "/api/init":
        result = handle_init(params)
      elif self.command in ("GET", "HEAD") and path == "/api/models":
        result = handle_models(params, cars)
      elif self.command == "POST" and path == "/api/models/select":
        try:
          payload = self._read_json()
        except ValueError:
          result = (400, {"error": "bad request"})
        else:
          result = handle_select(params, cars, payload)
      elif self.command in ("GET", "HEAD") and path == "/api/experimental_mode":
        result = handle_experimental_status(params, self.server.long_available(params))
      elif self.command == "POST" and path == "/api/experimental_mode":
        try:
          payload = self._read_json()
        except ValueError:
          payload = None
        result = handle_experimental_mode(params, payload, self.server.long_available(params))
      elif path in ("/api/init", "/api/models", "/api/models/select", "/api/experimental_mode"):
        result = (405, {"error": "method not allowed"})
      else:
        result = (404, {"error": "not found"})
    except Exception as e:
      logger.exception("opviewd: error handling %s %s", self.command, self.path)
      result = (500, {"error": "exception", "message": f"{type(e).__name__}: {e}"})
    self._send(*result)

  def do_GET(self) -> None:
    self._dispatch()

  def do_HEAD(self) -> None:
    self._dispatch()

  def do_POST(self) -> None:
    self._dispatch()

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    pass


class OpviewdHTTPServer(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True
  params: Params
  cars: CarList
  long_available: Callable[[Params], bool | None]


def make_server(host: str, port: int, params: Params | None = None, cars: CarList | None = None) -> OpviewdHTTPServer:
  server = OpviewdHTTPServer((host, port), OpviewdHandler)
  server.params = params or Params()
  server.cars = cars or CarList(server.params)
  server.long_available = experimental_long_available
  return server


def main():
  parser = argparse.ArgumentParser(description="opview helper daemon")
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=OPVIEWD_PORT, help="Port to listen on")
  args = parser.parse_args()

  logging.basicConfig(level=logging.INFO)
  server = make_server(args.host, args.port)
  logger.info("opviewd listening on %s:%d", args.host, args.port)
  server.serve_forever()


if __name__ == "__main__":
  main()
