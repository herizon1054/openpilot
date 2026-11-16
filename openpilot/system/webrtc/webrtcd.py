#!/usr/bin/env python3

from abc import abstractmethod
from collections.abc import Callable
import os
import socket
import time
import capnp
import ctypes
import gc
import argparse
import asyncio
import contextlib
import ipaddress
import json
import uuid
import logging
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from typing import Any

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.schema import generate_field
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.cereal import messaging, log
from openpilot.cereal.services import SERVICE_LIST
from openpilot.sunnypilot.privacy import privacy_mode

SESSION_TIMEOUT_SECONDS = 300

# --- opview ---
# opview（Android 手機 App）在區網直接連 webrtcd（:5001），請求格式沿用 dp0111pre 的新版 webrtcd：
#   {"sdp", "initCamera": "road", "bridge_services_out": [...], "bridge_services_in": []}
# sunnypilot 的 webrtcd 是 upstream 新版（libdatachannel、只聽 127.0.0.1、要求 cameras + enabled、
# session 300 秒逾時），以下為了讓 opview 能連上所做的相容處理，全部只作用在 opview session，
# sunnylink / athena 的 livestream（從 127.0.0.1 來）行為與 upstream 相同。
OPVIEW_PARAM = "OpviewEnabled"
OPVIEW_CAMERAS = ("road", "wideRoad", "driver")
# opview 使用的舊 service 名稱 → 這版 sunnypilot（upstream）改名後的名稱。
# capnp ordinal 與欄位都相同（log.capnp：extrinsicsCalibration @19 :ExtrinsicsCalibration、narrowRoadCameraState @2 :FrameData、
# cabinCameraState @70 :FrameData），只是名字不同，所以請求時換成新名稱、送出時 type 換回舊名稱，opview 不需修改。
# 只在舊名稱不存在、新名稱存在時才轉換（其他分支仍用舊名稱時不受影響）。
OPVIEW_SERVICE_ALIASES = {
  "liveCalibration": "extrinsicsCalibration",
  "roadCameraState": "narrowRoadCameraState",
  "driverCameraState": "cabinCameraState",
}


# opview 已內建 TDX（手機 GPS + TDX API），comma 的 tdx 不再傳給 opview：
# opview 的請求若帶 tdx（舊版 App）就從 bridge_services_out 拿掉，不訂閱、不轉送，也不會因此回 500。
# sunnylink / athena（upstream 格式）的請求不受影響。
OPVIEW_DROPPED_SERVICES = frozenset({"tdx"})


def resolve_opview_services(services: list[str]) -> list[str]:
  out: list[str] = []
  for s in services:
    if s not in SERVICE_LIST and OPVIEW_SERVICE_ALIASES.get(s) in SERVICE_LIST:
      s = OPVIEW_SERVICE_ALIASES[s]
    if s not in out:
      out.append(s)
  return out


def opview_outgoing_names(services: list[str]) -> dict[str, str]:
  """送給 opview 時的 type：新名稱 → opview 認得的舊名稱"""
  reverse = {new: old for old, new in OPVIEW_SERVICE_ALIASES.items() if old not in SERVICE_LIST}
  return {s: reverse[s] for s in services if s in reverse}


def _r3(values) -> list[float]:
  return [round(v, 3) for v in values]


def _xyz(xyzt) -> dict:
  return {"x": _r3(xyzt.x), "y": _r3(xyzt.y), "z": _r3(xyzt.z)}


def compact_model_v2(model) -> dict:
  """opview 用的 modelV2 精簡版（移植自 dp0111pre）：只送畫面用到的欄位，數值四捨五入到小數 3 位。
  完整 to_dict() 約 31 KB / 筆（20 Hz ≈ 5 Mbps），精簡後約 5.6 KB（≈ 0.9 Mbps）。
  opview 讀的欄位：position.x/y/z、laneLines[].x/y/z、laneLineProbs、roadEdges[].x/y/z、roadEdgeStds、acceleration.x、
  leadsV3[].prob/x/y（近距離警示）
  （見 opview lib/selfdrive/ui/ui_state.dart applyModelV2）。opview 之後若要用其他欄位，需在這裡一併加上。"""
  return {
    "frameId": model.frameId,
    "position": _xyz(model.position),
    "laneLines": [_xyz(line) for line in model.laneLines],
    "laneLineProbs": _r3(model.laneLineProbs),
    "roadEdges": [_xyz(e) for e in model.roadEdges],
    "roadEdgeStds": _r3(model.roadEdgeStds),
    "acceleration": {"x": _r3(model.acceleration.x)},
    # 純視覺前車（近距離警示用：opview 讀 leadsV3[0] 的 x[0] / y[0] / prob）
    "leadsV3": [{"prob": round(lead.prob, 3), "x": _r3(lead.x), "y": _r3(lead.y)} for lead in model.leadsV3],
  }


# opview 只讀這些欄位（opview lib/selfdrive/ui/ui_state.dart 的 apply* 函式）。其他欄位不送，
# 省下 to_dict() 產生整個巢狀 dict 與 json.dumps 的 CPU / 記憶體（實測 rlog 每筆 JSON：longitudinalPlan 2.0 KB、
# carState 1.6 KB、controlsState 1.6 KB、radarState 1.0 KB、roadCameraState 0.6 KB → 各約 0.05–0.3 KB）。
# key 是 opview 端的 service 名稱（改名前的舊名稱）；欄位不存在的分支就略過該欄位（與 to_dict() 沒有該欄位相同）。
# 表上沒有的 service 照舊整筆 to_dict()。modelV2 另由 compact_model_v2 處理。
# 有沒有前車：dragonpilot 舊 cereal 叫 status，openpilot / sunnypilot 新版改名 present（同 @11）。兩個都列，
# 不存在的那個會略過。只列 status 時新版分支 leadOne 沒有這個欄位，opview 判定「沒有前車」（鎖定框、車距都不見，2026-11-13）。
OPVIEW_LEAD_FIELDS = ("status", "present", "dRel", "yRel", "vRel", "vLead", "aLeadK", "modelProb", "radar")
OPVIEW_COMPACT_FIELDS: dict[str, dict[str, Any]] = {
  "carState": dict.fromkeys(("vEgo", "aEgo", "steeringAngleDeg", "leftBlinker", "rightBlinker", "leftBlindspot",
                             "rightBlindspot", "gearShifter", "standstill", "gasPressed", "vEgoCluster", "vCruiseCluster")),
  "selfdriveState": dict.fromkeys(("enabled", "experimentalMode", "alertText1", "alertText2", "alertSize",
                                   "alertStatus", "state")),
  "controlsState": dict.fromkeys(("vCruiseDEPRECATED",)),
  "radarState": {"leadOne": OPVIEW_LEAD_FIELDS, "leadTwo": OPVIEW_LEAD_FIELDS},
  "longitudinalPlan": dict.fromkeys(("allowThrottle",)),
  "deviceState": dict.fromkeys(("deviceType", "started")),
  "liveCalibration": dict.fromkeys(("calStatus", "rpyCalib", "wideFromDeviceEuler", "height")),
  "roadCameraState": dict.fromkeys(("sensor",)),
}


def _plain(value: Any) -> Any:
  """capnp 值 → JSON 可序列化的值（與 to_dict() 相同：enum 用名稱字串、list 轉 list）"""
  if value is None or isinstance(value, (bool, int, float, str)):
    return value
  if isinstance(value, (capnp._DynamicListReader, list, tuple)):
    return [_plain(v) for v in value]
  if isinstance(value, capnp._DynamicStructReader):
    return value.to_dict()
  return str(value)  # _DynamicEnum → 'drive'、'calibrated'…


def compact_fields(reader: Any, fields: dict[str, Any] | tuple[str, ...]) -> dict:
  """只取指定欄位；fields 的值是 tuple 時代表子結構只取那些欄位。不存在的欄位略過。"""
  out: dict[str, Any] = {}
  items = fields.items() if isinstance(fields, dict) else ((f, None) for f in fields)
  for name, sub in items:
    try:
      value = getattr(reader, name)
    except (AttributeError, capnp.KjException):
      continue
    out[name] = compact_fields(value, sub) if sub is not None else _plain(value)
  return out


class StreamRequestError(Exception):
  def __init__(self, status: int, error: str, message: str):
    super().__init__(message)
    self.status, self.error, self.message = status, error, message


def parse_stream_request(raw: Any, remote_is_local: bool = True, allow_driver_camera: bool = True) -> tuple[StreamRequestBody, bool]:
  """回傳 (body, is_opview)。
  - upstream 格式（sunnylink / athena：cameras + enabled）原樣交給 StreamRequestBody，行為不變。
  - opview 格式（initCamera，或明確帶 "client": "opview"）：
      initCamera → cameras=[initCamera]、enabled 預設 True、忽略不認得的欄位（同 dp0111pre）。
  - opview 請求帶的 tdx 一律拿掉（opview 已內建 TDX，見 OPVIEW_DROPPED_SERVICES）。
  - 不認得的 cereal service → 500（與 dp0111pre / stock webrtcd 一致）。
  - 區網（非 127.0.0.1）來的請求不允許 bridge_services_in：webrtcd 原本只聽 localhost，
    開放給 opview 後不能讓區網任何人往 comma 發 cereal 訊息（testJoystick、can…）。"""
  if not isinstance(raw, dict):
    raise StreamRequestError(400, "invalid_request", "request body must be a JSON object")

  is_opview = raw.get("client") == "opview" or ("initCamera" in raw and "cameras" not in raw)
  if is_opview:
    raw = dict(raw)
    camera = raw.get("initCamera") or "road"
    if "cameras" not in raw:
      raw["cameras"] = [camera]
    raw.setdefault("enabled", True)
    known = StreamRequestBody.__dataclass_fields__.keys()
    raw = {k: v for k, v in raw.items() if k in known}
    if isinstance(raw.get("bridge_services_out"), list):
      out = [svc for svc in raw["bridge_services_out"] if svc not in OPVIEW_DROPPED_SERVICES]
      raw["bridge_services_out"] = resolve_opview_services(out)

  try:
    body = StreamRequestBody(**raw)
  except TypeError as e:
    raise StreamRequestError(400, "invalid_request", str(e)) from e

  if is_opview:
    if len(body.cameras) != 1 or body.cameras[0] not in OPVIEW_CAMERAS:
      raise StreamRequestError(400, "invalid_camera", f"unsupported camera {body.cameras}")
    unknown = [s for s in (*body.bridge_services_out, *body.bridge_services_in) if s not in SERVICE_LIST]
    if unknown:
      # 500：與 dp0111pre / stock webrtcd（SubMaster KeyError）一致
      raise StreamRequestError(500, "unknown_service", f"unknown services {unknown}")

  if not allow_driver_camera and "driver" in body.cameras:
    # 隱私模式：不串流車內鏡頭（opview / sunnylink 皆同）
    raise StreamRequestError(403, "forbidden", "driver camera is disabled in privacy mode")

  if not remote_is_local and body.bridge_services_in:
    raise StreamRequestError(403, "forbidden", "bridge_services_in is only allowed from localhost")

  return body, is_opview



# ice candidate parser for logging
# --- 記憶體（openpilot-test：按巡航 commIssue 調查，mici 空閒記憶體常態約 20 MB） ---
# webrtcd 有多條執行緒（HTTP、asyncio、libdatachannel），glibc 預設最多開「CPU 核心數 × 8」個 malloc arena，
# 每個 arena 各自保留釋放後的空間，RSS 只增不減。
#   1. 啟動時 mallopt(M_ARENA_MAX, 2)：最多 2 個 arena。
#   2. 連線結束（沒有任何 session）後 gc.collect() + malloc_trim(0)：把 arena 裡空出來的頁還給系統。
# 與上游不同（刻意）：上游 webrtcd 沒有這兩項。非 glibc（macOS、musl）時兩者都不做事，回傳 False。
M_ARENA_MAX = -8  # glibc <malloc.h>
WEBRTCD_MALLOC_ARENAS = 2
MEMORY_RELEASE_DELAY_S = 1.0  # session 結束後等 libdatachannel / asyncio 收尾再 trim

_libc: ctypes.CDLL | None | bool = None


def _glibc() -> ctypes.CDLL | None:
  global _libc
  if _libc is None:
    try:
      lib = ctypes.CDLL("libc.so.6")
      lib.mallopt.argtypes = [ctypes.c_int, ctypes.c_int]
      lib.mallopt.restype = ctypes.c_int
      lib.malloc_trim.argtypes = [ctypes.c_size_t]
      lib.malloc_trim.restype = ctypes.c_int
      _libc = lib
    except (OSError, AttributeError):
      _libc = False
  return _libc or None


def limit_malloc_arenas(n: int = WEBRTCD_MALLOC_ARENAS) -> bool:
  """限制 malloc arena 數量；要在建立其他執行緒之前呼叫。成功回傳 True。"""
  lib = _glibc()
  return lib is not None and lib.mallopt(M_ARENA_MAX, n) == 1


def _rss_mb() -> float | None:
  try:
    with open("/proc/self/statm") as f:
      return int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1e6
  except (OSError, ValueError, IndexError):
    return None


def release_memory() -> tuple[bool, float | None, float | None]:
  """gc.collect() 後 malloc_trim(0)。回傳 (是否真的還了記憶體, trim 前 RSS MB, trim 後 RSS MB)。"""
  before = _rss_mb()
  gc.collect()
  lib = _glibc()
  released = bool(lib.malloc_trim(0)) if lib is not None else False
  return released, before, _rss_mb()


def schedule_memory_release(state: "ServerState") -> None:
  """連線結束後延遲一下再還記憶體；屆時若已有新連線就不做（避免和新 session 搶時間）。"""
  def release():
    if state.streams:
      return
    t0 = time.monotonic()
    released, before, after = release_memory()
    logging.getLogger("webrtcd").info(
      "session ended: malloc_trim released=%s, RSS %s -> %s MB (%.0f ms)", released,
      f"{before:.0f}" if before is not None else "?", f"{after:.0f}" if after is not None else "?",
      (time.monotonic() - t0) * 1000)

  asyncio.get_running_loop().call_later(MEMORY_RELEASE_DELAY_S, release)


def _ice_candidates(sdp: str) -> list[str]:
  return [line.removeprefix("a=") for line in sdp.splitlines() if line.startswith("a=candidate:")]

# socket trick: route lookup for 8.8.8.8 (nothing is sent or actually connected to)
# return the source interfaces IP which is the default interface of the device
def _default_route_ip() -> str | None:
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    s.connect(("8.8.8.8", 53))  # selects a route, sends nothing
    return s.getsockname()[0]
  except OSError:
    return None
  finally:
    s.close()

class AsyncTaskRunner:
  def __init__(self):
    self.task = None
    self.logger = logging.getLogger("webrtcd")

  def start(self):
    assert self.task is None
    self.task = asyncio.create_task(self.run())

  async def stop(self):
    if self.task is None:
      return
    task = self.task
    self.task = None
    if task.done():
      return
    task.cancel()
    with contextlib.suppress(asyncio.CancelledError):
      await task

  @abstractmethod
  async def run(self):
    pass


class CerealOutgoingMessageProxy(AsyncTaskRunner):
  # opview：data channel 是可靠、有序的，送出佇列沒有上限。網路（例如 iPhone 熱點）或 comma CPU 跟不上時，
  # 舊訊息會在佇列裡越積越多、照順序晚到。
  # 壅塞判斷只用 send() 的回傳值（libdatachannel：True = 立即送出，False = 進了緩衝佇列）。
  # 刻意「不」呼叫 buffered_amount()：實車 rlog 顯示呼叫它之後 webrtcd 事件迴圈卡死（連線後 CPU 不再增加、
  # 資料 0 筆、影像只有 1 張），約 12 秒後 SIGSEGV（exitCode -11）。上游 webrtcd / teleoprtc 也只用 is_open() 與 send()。
  # send() 回 False 後暫停 OPVIEW_BACKOFF_S（SubMaster 只留最新值，恢復後送的就是最新資料）。
  OPVIEW_BACKOFF_S = 0.2
  # opview 只在 modelV2（20 Hz）到達時重畫：carState、controlsState 這類 100 Hz 的 service 多送也看不到，
  # 每個 service 最多約 33 Hz（間隔 30 ms）。20 Hz 以下的 service（含 modelV2）不受影響。
  OPVIEW_MIN_SEND_INTERVAL = 0.03

  def __init__(self, services: list[str], enabled: bool = True, opview: bool = False):
    super().__init__()
    self.services = list(services)
    self.sm = messaging.SubMaster(self.services)
    self.channels = []
    self._enabled = enabled
    self.opview = opview
    self.out_names = opview_outgoing_names(self.services) if opview else {}
    self.skipped = 0
    self.congested_since: float | None = None
    self.backoff_until = 0.0
    self.last_sent: dict[str, float] = {}

  def add_channel(self, channel):
    self.channels.append(channel)

  def enable(self, enable: bool):
    self._enabled = enable

  def to_json(self, msg_content: Any):
    if isinstance(msg_content, capnp._DynamicStructReader):
      msg_dict = msg_content.to_dict()
    elif isinstance(msg_content, capnp._DynamicListReader):
      msg_dict = [self.to_json(msg) for msg in msg_content]
    elif isinstance(msg_content, bytes):
      msg_dict = msg_content.decode()
    else:
      msg_dict = msg_content

    return msg_dict

  def update(self):
    # this is blocking in async context...
    self.sm.update(0)
    if self.opview:
      self._update_opview()
      return
    for service, updated in self.sm.updated.items():
      if not updated:
        continue
      msg_dict = self.to_json(self.sm[service])
      mono_time, valid = self.sm.logMonoTime[service], self.sm.valid[service]
      outgoing_msg = {"type": service, "logMonoTime": mono_time, "valid": valid, "data": msg_dict}
      encoded_msg = json.dumps(outgoing_msg).encode()
      for channel in self.channels:
        if not channel.is_open():
          continue
        channel.send(encoded_msg)

  def _update_opview(self):
    open_channels = [ch for ch in self.channels if ch.is_open()]
    if not open_channels:
      return
    now = time.monotonic()
    n_updated = sum(1 for updated in self.sm.updated.values() if updated)
    if now < self.backoff_until:
      self.skipped += n_updated
      return
    if self.congested_since is not None:
      self.logger.warning("opview data channel recovered after %.1fs, dropped %d messages", now - self.congested_since, self.skipped)
      self.congested_since = None
      self.skipped = 0

    for service, updated in self.sm.updated.items():
      if not updated:
        continue
      if now - self.last_sent.get(service, 0.0) < self.OPVIEW_MIN_SEND_INTERVAL:
        continue
      self.last_sent[service] = now
      out_name = self.out_names.get(service, service)
      if service == "modelV2":
        msg_dict = compact_model_v2(self.sm[service])
      elif out_name in OPVIEW_COMPACT_FIELDS:
        msg_dict = compact_fields(self.sm[service], OPVIEW_COMPACT_FIELDS[out_name])
      else:
        msg_dict = self.to_json(self.sm[service])
      mono_time, valid = self.sm.logMonoTime[service], self.sm.valid[service]
      outgoing_msg = {"type": out_name, "logMonoTime": mono_time, "valid": valid, "data": msg_dict}
      encoded_msg = json.dumps(outgoing_msg).encode()
      sent_now = True
      for channel in open_channels:
        if channel.send(encoded_msg) is False:
          sent_now = False
      if not sent_now:
        # 已進入緩衝佇列：這一筆仍會送達，之後暫停一下讓佇列消化
        self.backoff_until = now + self.OPVIEW_BACKOFF_S
        if self.congested_since is None:
          self.congested_since = now
          self.logger.warning("opview data channel congested, backing off %.1fs", self.OPVIEW_BACKOFF_S)
        return

  async def run(self):
    while True:
      if not self._enabled:
        await asyncio.sleep(0.01)
        continue
      try:
        self.update()
      except Exception:
        self.logger.exception("Cereal outgoing proxy failure")
      await asyncio.sleep(0.01)


class CerealIncomingMessageProxy:
  def __init__(self, pm: messaging.PubMaster):
    self.pm = pm

  def send(self, message: bytes):
    msg_json = json.loads(message)
    msg_type, msg_data = msg_json["type"], msg_json["data"]
    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(msg_type, size=size)
    setattr(msg, msg_type, msg_data)
    self.pm.send(msg_type, msg)


class DynamicPubMaster(messaging.PubMaster):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.lock = asyncio.Lock()

  async def add_services_if_needed(self, services):
    async with self.lock:
      for service in services:
        if service not in self.sock:
          self.sock[service] = messaging.pub_sock(service)


class LivestreamBitrateController(AsyncTaskRunner):
  bitrates = [500_000, 1_500_000, int(os.environ.get("STREAM_BITRATE", 5_000_000))]
  label_to_bitrate = { "high": bitrates[2], "med": bitrates[1], "low": bitrates[0]}
  sample_interval = 0.2
  high_level = 0.1 # drop immediately
  med_level = 0.05 # drop after # of samples
  low_level = 0 # raise after # of samples
  down_samples = 5
  param_name = "LivestreamEncoderBitrate"

  def __init__(self, get_stats: Callable[[], dict[str, Any]], params: Params, enabled: bool = True):
    super().__init__()
    self.get_stats = get_stats
    self.params = params

    self.level = 2
    self._publish(self.bitrates[self.level])
    self.prev_stats: tuple[Any, ...] | None = None
    self.counter = 0
    self.up_samples = 5 # 1s
    self._auto = True
    self._enabled = enabled

  def enable(self, enable: bool):
    self._enabled = enable

  async def run(self):
    while True:
      await asyncio.sleep(self.sample_interval)
      if not self._enabled:
        continue
      if not self._auto:
        continue

      loss_rate = self._sample()
      if loss_rate is None:
        continue
      if loss_rate >= self.med_level and self.level > 0:
        self.counter += 1
        if self.counter >= self.down_samples or loss_rate >= self.high_level:
          self.level -= 1
          self.up_samples *= 2 # exponential backoff before raising again
          self.counter = 0
          self._publish(self.bitrates[self.level])
      elif loss_rate <= self.low_level and self.level < len(self.bitrates) - 1:
        self.counter -= 1
        if -self.counter >= self.up_samples:
          self.level += 1
          self.counter = 0
          self._publish(self.bitrates[self.level])

  def _sample(self) -> float | None:
    report = next(iter(self.get_stats().values()), None)
    if report is None:
      return None

    current = (report.ssrc, report.fraction_lost, report.packets_lost, report.highest_seq_no, report.jitter, report.lsr, report.dlsr)
    if self.prev_stats == current:
      return None
    self.prev_stats = current

    loss_rate = report.fraction_lost / 256
    return loss_rate

  def _publish(self, bitrate: float):
    self.params.put(self.param_name, bitrate)

  def set_quality(self, quality):
    if quality in self.label_to_bitrate:
      self._publish(self.label_to_bitrate[quality])
      self._auto = False
    elif quality == "auto":
      self._auto = True


class StreamSession:
  shared_pub_master = DynamicPubMaster([])

  def __init__(self, body: StreamRequestBody, is_opview: bool = False):
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from teleoprtc.builder import WebRTCAnswerBuilder

    self.identifier = str(uuid.uuid4())
    self.params = Params()
    self.is_opview = is_opview
    # opview：不綁定預設路由介面。comma 同時有 LTE 與 WiFi 時預設路由可能是 LTE，
    # 綁上去 ICE 只會產生 LTE 位址，手機熱點 / 區網的 opview 就連不到。
    bind_address = None if is_opview else _default_route_ip()
    builder = WebRTCAnswerBuilder(body.sdp, bind_address=bind_address)

    self.enabled = body.enabled
    self.video_tracks = []
    for camera in body.cameras:
      track = LiveStreamVideoStreamTrack(camera, self.enabled)
      self.video_tracks.append(track)
      builder.add_video_stream(camera, track)
    self.stream = builder.stream()

    self.is_body = "testJoystick" in body.bridge_services_in

    self.incoming_bridge: CerealIncomingMessageProxy | None = None
    self.incoming_bridge_services = body.bridge_services_in
    self.outgoing_bridge: CerealOutgoingMessageProxy | None = None
    self.bitrate_controller: LivestreamBitrateController | None = None
    if len(body.bridge_services_in) > 0:
      self.incoming_bridge = CerealIncomingMessageProxy(self.shared_pub_master)
    if len(body.bridge_services_out) > 0:
      self.outgoing_bridge = CerealOutgoingMessageProxy(body.bridge_services_out, self.enabled, opview=is_opview)
    self.bitrate_controller = LivestreamBitrateController(self.stream.get_receiver_report_stats, self.params, self.enabled)

    self.run_task: asyncio.Task | None = None
    self._cleanup_lock = asyncio.Lock()
    self._cleanup_done = False
    self.logger = logging.getLogger("webrtcd")
    cloudlog.warning(
      "New stream session (%s), video cameras %s, video enabled %s, incoming services %s, outgoing services %s, opview %s",
      self.identifier, [t.id for t in self.video_tracks], body.enabled, body.bridge_services_in, body.bridge_services_out, is_opview,
    )

  def start(self):
    self.run_task = asyncio.create_task(self.run())

  async def stop(self):
    if self.run_task is not None and not self.run_task.done() and self.run_task is not asyncio.current_task():
      self.run_task.cancel()
      with contextlib.suppress(asyncio.CancelledError):
        await self.run_task
    self.run_task = None
    await self.post_run_cleanup()

  async def get_answer(self):
    return await self.stream.start()

  def message_handler(self, message: bytes):
    try:
      payload = json.loads(message) if isinstance(message, (bytes, str)) else None
      if isinstance(payload, dict):
        msg_type = payload.get("type")

        match msg_type:
          case "livestreamCameraSwitch":
            # only needed for 1 track stream
            if len(self.video_tracks) == 1:
              self.video_tracks[0].switch_camera(payload["data"]["camera"])
          case "livestreamSettings":
            if self.bitrate_controller is not None:
              self.bitrate_controller.set_quality(payload["data"]["quality"])
          case "livestreamVideoEnable":
            enabled = payload["data"]["enabled"]
            self.enabled = enabled
            for track in self.video_tracks:
              track.enable(enabled)
            if self.outgoing_bridge is not None:
              self.outgoing_bridge.enable(enabled)
            if self.bitrate_controller is not None:
              self.bitrate_controller.enable(enabled)
            if not enabled:
              self.params.put("LivestreamRequestKeyframe", True)
          case "clockSync":
            pong = json.dumps({"type": "clockSync", "data": {
              "action": "pong", "browserSendTime": payload["data"]["browserSendTime"], "deviceTime": time.time() * 1000, # noqa: TID251
            }})
            self.stream.get_messaging_channel().send(pong)
          case "enableTimingSei":
            for track in self.video_tracks:
              track.timing_sei_enabled = bool(payload["data"]["enabled"])
          case _:
            if msg_type not in self.incoming_bridge_services:
              return
            if self.incoming_bridge is not None:
              self.incoming_bridge.send(message)
    except Exception:
      self.logger.exception("Cereal incoming proxy failure")

  async def run_normal_session(self):
    try:
      await asyncio.wait_for(self.stream.wait_for_disconnection(), timeout=SESSION_TIMEOUT_SECONDS)
    except TimeoutError:
      self.logger.warning("Stream session (%s) timed out after %d s", self.identifier, SESSION_TIMEOUT_SECONDS)
      try:
        self.stream.get_messaging_channel().send(json.dumps({"type": "disconnect", "data": "Session timed out"}))
      except Exception:
        pass

  async def run_body_session(self):
    await self.stream.wait_for_disconnection()

  async def run(self):
    try:
      self.params.put("LivestreamRequestKeyframe", True)

      # avoid datachannel race by adding messange_handler immediately
      self.stream.set_message_handler(self.message_handler)

      await asyncio.wait_for(self.stream.wait_for_connection(), timeout=15)
      if self.stream.has_messaging_channel():
        if self.incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self.incoming_bridge_services)
        if self.outgoing_bridge is not None:
          channel = self.stream.get_messaging_channel()
          self.outgoing_bridge.add_channel(channel)
          self.outgoing_bridge.start()
      if self.bitrate_controller is not None:
        self.bitrate_controller.start()

      with cloudlog.ctx(session_id=self.identifier):
        cloudlog.warning("webrtcd.session.connected")
      if self.is_body or self.is_opview:
        # opview 是整趟行車都開著的儀表，不套用 300 秒 session 逾時（同 body）
        await self.run_body_session()
      else:
        await self.run_normal_session()
      with cloudlog.ctx(session_id=self.identifier):
        cloudlog.warning("webrtcd.session.ended")
    except Exception:
      self.logger.exception("Stream session failure")
      with cloudlog.ctx(session_id=self.identifier):
        cloudlog.exception("webrtcd.session.exception")
    finally:
      await self.post_run_cleanup()

  async def post_run_cleanup(self):
    async with self._cleanup_lock:
      if self._cleanup_done:
        return
      self._cleanup_done = True
      self.params.put("LivestreamRequestKeyframe", False)
      if self.bitrate_controller is not None:
        await self.bitrate_controller.stop()
      if self.outgoing_bridge is not None:
        await self.outgoing_bridge.stop()
      for track in self.video_tracks:
        track.stop()
      self.video_tracks.clear()
      await self.stream.stop()


class ServerState:
  def __init__(self):
    self.streams: dict[str, StreamSession] = {}
    self.stream_lock = asyncio.Lock()
    self.teardown: asyncio.TimerHandle | None = None


# if nothing connects for 5 seconds, tear down livestreaming processes
def schedule_teardown(state: ServerState):
  if state.teardown is not None:
    state.teardown.cancel()

  def clear():
    if not state.streams:
      Params().put_bool("IsLiveStreaming", False)

  state.teardown = asyncio.get_running_loop().call_later(5.0, clear)


def _json_response(obj: Any, status: int = 200) -> tuple[int, bytes, str]:
  return (status, json.dumps(obj).encode(), "application/json; charset=utf-8")


def _text_response(text: str, status: int = 200) -> tuple[int, bytes, str]:
  return (status, text.encode(), "text/plain; charset=utf-8")


async def handle_get_stream(state: ServerState, raw_body: bytes, content_type: str,
                            remote_is_local: bool = True) -> tuple[int, bytes, str]:
  if content_type != "application/json":
    return _json_response({"error": "unsupported media type"}, status=415)

  stream_dict = state.streams
  try:
    body, is_opview = parse_stream_request(json.loads(raw_body), remote_is_local, allow_driver_camera=not privacy_mode())
  except json.JSONDecodeError:
    return _json_response({"error": "bad request"}, status=400)
  except StreamRequestError as e:
    logging.getLogger("webrtcd").warning("rejected stream request: %s", e.message)
    return _json_response({"error": e.error, "message": e.message}, status=e.status)

  async with state.stream_lock:
    # don't remove existing connection on prewarm request
    enabled = any(s.run_task and not s.run_task.done() and s.enabled for s in stream_dict.values())
    if enabled and not body.enabled:
      return _json_response({"error": "busy", "message": "someone else is connected."})

    for sid, s in list(stream_dict.items()):
      if s.run_task and not s.run_task.done():
        try:
          ch = s.stream.get_messaging_channel()
          # opview 認的是 connectionReplaced（收到後停止自動重連，避免兩台裝置互搶）；其他 client 維持 upstream 的 disconnect
          msg_type = "connectionReplaced" if s.is_opview else "disconnect"
          ch.send(json.dumps({"type": msg_type, "data": "Another device has connected, closing this session."}))
        except Exception:
          pass
      await s.stop()
      stream_dict.pop(sid, None)

    session = StreamSession(body, is_opview)
    stream_dict[session.identifier] = session
    try:
      answer = await asyncio.wait_for(session.get_answer(), timeout=30)
      cloudlog.event(
        "webrtcd.session.ice_candidates",
        session_id=session.identifier,
        offer_candidates=_ice_candidates(body.sdp),
        answer_candidates=_ice_candidates(answer.sdp),
      )
    except TimeoutError:
      await session.stop()
      stream_dict.pop(session.identifier, None)
      logging.getLogger("webrtcd").exception("Timed out creating stream answer")
      with cloudlog.ctx(session_id=session.identifier):
        cloudlog.warning("webrtcd.session.answer_timeout")
      raise
    except Exception:
      await session.stop()
      stream_dict.pop(session.identifier, None)
      logging.getLogger("webrtcd").exception("Failed to create stream answer")
      with cloudlog.ctx(session_id=session.identifier):
        cloudlog.exception("webrtcd.session.answer_exception")
      raise
    session.start()

    def remove_finished_session(_: asyncio.Task) -> None:
      stream_dict.pop(session.identifier, None)
      schedule_teardown(state)
      schedule_memory_release(state)

    session.run_task.add_done_callback(remove_finished_session)

  return _json_response({"sdp": answer.sdp, "type": answer.type})


async def handle_get_schema(state: ServerState, services_param: str) -> tuple[int, bytes, str]:
  services = services_param.split(",")
  services = [s for s in services if s]
  assert all(s in log.Event.schema.fields and not s.endswith("DEPRECATED") for s in services), "Invalid service name"
  schema_dict = {s: generate_field(log.Event.schema.fields[s]) for s in services}
  return _json_response(schema_dict)


async def handle_post_notify(state: ServerState, payload: Any) -> tuple[int, bytes, str]:
  for session in list(state.streams.values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps(payload))
    except Exception:
      continue

  return _text_response("OK")


async def on_shutdown(state: ServerState):
  for session in list(state.streams.values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps({"type": "disconnect", "data": "device streaming has been stopped."}))
    except Exception:
      pass
    await session.stop()
  state.streams.clear()


class WebrtcdHandler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  # path -> allowed methods (aiohttp registered POST /stream, POST /notify, GET /schema + its auto HEAD)
  _routes = {
    "/schema": ("GET", "HEAD"),
    "/stream": ("POST",),
    "/notify": ("POST",),
  }

  def _send(self, status: int, body: bytes, content_type: str) -> None:
    self.send_response(status)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    if self.command != "HEAD":
      self.wfile.write(body)

  def _read_body(self) -> bytes:
    length = int(self.headers.get("Content-Length", 0))
    return self.rfile.read(length) if length else b""

  def _run(self, coro) -> tuple[int, bytes, str]:
    return asyncio.run_coroutine_threadsafe(coro, self.server.loop).result()

  def _remote_is_local(self) -> bool:
    try:
      return ipaddress.ip_address(self.client_address[0]).is_loopback
    except ValueError:
      return False

  def _dispatch_request(self) -> None:
    parsed = urlparse(self.path)
    allowed = self._routes.get(parsed.path)
    remote_is_local = self._remote_is_local()

    try:
      if not remote_is_local and not self.server.params.get_bool(OPVIEW_PARAM):
        # 只有開啟 opview 時才接受區網連線；未開啟時與 upstream 一樣只服務 localhost（sunnylink / athena）
        result = _json_response({"error": "forbidden", "message": "LAN access requires opview to be enabled"}, status=403)
      elif allowed is None:
        result = _json_response({"error": "not found"}, status=404)
      elif self.command not in allowed:
        result = _json_response({"error": "method not allowed"}, status=405)
      elif parsed.path == "/schema":
        services = parse_qs(parsed.query).get("services", [""])[0]
        result = self._run(handle_get_schema(self.server.state, services))
      elif parsed.path == "/stream":
        result = self._run(handle_get_stream(self.server.state, self._read_body(), self.headers.get_content_type(), remote_is_local))
      else:  # /notify
        try:
          payload = json.loads(self._read_body())
        except Exception:
          result = _json_response({"error": "bad request"}, status=400)
        else:
          result = self._run(handle_post_notify(self.server.state, payload))
    except Exception as e:
      logging.getLogger("webrtcd").exception("Unhandled error handling %s", self.path)
      result = _json_response({"error": "exception", "message": f"{type(e).__name__}: {e}"}, status=500)

    self._send(*result)

  def do_GET(self) -> None:
    self._dispatch_request()

  def do_HEAD(self) -> None:
    self._dispatch_request()

  def do_POST(self) -> None:
    self._dispatch_request()

  def do_PUT(self) -> None:
    self._dispatch_request()

  def do_DELETE(self) -> None:
    self._dispatch_request()

  def do_PATCH(self) -> None:
    self._dispatch_request()

  def do_OPTIONS(self) -> None:
    self._dispatch_request()

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    # silence default access logging; errors are logged explicitly in _dispatch_request
    pass


class WebrtcdHTTPServer(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True
  state: ServerState
  loop: asyncio.AbstractEventLoop
  params: Params


async def _shutdown(server: WebrtcdHTTPServer, state: ServerState, loop: asyncio.AbstractEventLoop) -> None:
  # stop accepting new HTTP connections (blocks until serve_forever returns, so
  # run it off the loop) then tear down active stream sessions.
  await loop.run_in_executor(None, server.shutdown)
  await on_shutdown(state)
  loop.stop()


def prewarm_stream_session_imports() -> None:
  from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
  from teleoprtc.builder import WebRTCAnswerBuilder
  assert LiveStreamVideoStreamTrack
  assert WebRTCAnswerBuilder


def webrtcd_thread(host: str, port: int, arenas_limited: bool | None = None):
  logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler()])
  if arenas_limited is not None:
    logging.getLogger("webrtcd").info("malloc arena limit %d: %s", WEBRTCD_MALLOC_ARENAS,
                                      "on" if arenas_limited else "unavailable (not glibc)")
  prewarm_start = time.monotonic()
  prewarm_stream_session_imports()
  prewarm_end = time.monotonic()
  logging.getLogger("webrtcd").info(f"webrtc prewarm finished in {(prewarm_end - prewarm_start) * 1000} ms")

  loop = asyncio.new_event_loop()
  asyncio.set_event_loop(loop)
  state = ServerState()

  server = WebrtcdHTTPServer((host, port), WebrtcdHandler)
  server.state = state
  server.loop = loop
  server.params = Params()

  # serve HTTP on a daemon thread so the asyncio loop can own the main thread
  http_thread = threading.Thread(target=server.serve_forever, name="webrtcd-http", daemon=True)
  http_thread.start()

  shutting_down = False
  shutdown_task = None

  def request_shutdown() -> None:
    nonlocal shutting_down, shutdown_task
    if shutting_down:
      return
    shutting_down = True
    shutdown_task = loop.create_task(_shutdown(server, state, loop))

  for sig in (signal.SIGINT, signal.SIGTERM):
    loop.add_signal_handler(sig, request_shutdown)

  try:
    loop.run_forever()
  finally:
    server.server_close()
    loop.close()


def main():
  # 必須在 webrtcd_thread 建立 HTTP / asyncio / libdatachannel 執行緒之前
  arenas_limited = limit_malloc_arenas()
  parser = argparse.ArgumentParser(description="WebRTC daemon")
  # opview：手機在區網直接連 :5001，所以聽所有介面；非 localhost 的請求在 _dispatch_request 依 OpviewEnabled 擋下
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  args = parser.parse_args()

  webrtcd_thread(args.host, args.port, arenas_limited)


if __name__=="__main__":
  main()
