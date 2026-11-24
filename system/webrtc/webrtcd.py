#!/usr/bin/env python3

from abc import abstractmethod
import os
import time
import argparse
import asyncio
import contextlib
import json
import uuid
import logging
from dataclasses import dataclass, field
from typing import Any, TYPE_CHECKING

# aiortc and its dependencies have lots of internal warnings :(
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

import capnp
from aiohttp import web
if TYPE_CHECKING:
  from aiortc.rtcdatachannel import RTCDataChannel

from openpilot.system.webrtc.schema import generate_field
from openpilot.common.params import Params
from cereal import messaging, log
from cereal.services import SERVICE_LIST


class AsyncTaskRunner:
  def __init__(self):
    self.is_running = False
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


def _r3(values) -> list[float]:
  return [round(v, 3) for v in values]


def _xyz(xyzt) -> dict:
  return {"x": _r3(xyzt.x), "y": _r3(xyzt.y), "z": _r3(xyzt.z)}


def compact_model_v2(model) -> dict:
  """opview 用的 modelV2 精簡版：只送畫面用到的欄位，數值四捨五入到小數 3 位。
  完整 to_dict() 約 31 KB / 筆（20 Hz ≈ 5 Mbps），精簡後約 5.6 KB（≈ 0.9 Mbps），
  comma 端 json.dumps 與手機端 jsonDecode 的負擔也一起降低。
  opview 讀的欄位：position.x/y/z、laneLines[].x/y/z、laneLineProbs、roadEdges[].x/y/z、roadEdgeStds、acceleration.x、
  orientationRate.z[0]（opview v2.0.13 車型 3D 辨識：本車轉向角速度）、leadsV3[].prob/x/y（近距離警示）
  （見 opview lib/selfdrive/ui/ui_state.dart applyModelV2）。opview 之後若要用其他欄位，需在這裡一併加上。"""
  return {
    "frameId": model.frameId,
    "position": _xyz(model.position),
    "laneLines": [_xyz(l) for l in model.laneLines],
    "laneLineProbs": _r3(model.laneLineProbs),
    "roadEdges": [_xyz(e) for e in model.roadEdges],
    "roadEdgeStds": _r3(model.roadEdgeStds),
    "acceleration": {"x": _r3(model.acceleration.x)},
    # 本車轉向角速度（opview v2.0.13 車型 3D 辨識：追蹤目標速度扣掉本車轉向、連續車頭角度）：只送 t = 0 那一點
    "orientationRate": {"z": _r3([model.orientationRate.z[0]] if len(model.orientationRate.z) else [])},
    # 純視覺前車（近距離警示用：opview 讀 leadsV3[0] 的 x[0] / y[0] / prob）
    "leadsV3": [{"prob": round(lead.prob, 3), "x": _r3(lead.x), "y": _r3(lead.y)} for lead in model.leadsV3],
  }



# opview 只讀這些欄位（opview lib/selfdrive/ui/ui_state.dart 的 apply* 函式）。其他欄位不送，
# 省下 to_dict() 產生整個巢狀 dict 與 json.dumps 的 CPU / 記憶體（實測 rlog 每筆 JSON：longitudinalPlan 2.0 KB、
# carState 1.6 KB、controlsState 1.6 KB、radarState 1.0 KB、roadCameraState 0.6 KB → 各約 0.05–0.3 KB）。
# key 是 opview 端的 service 名稱（改名前的舊名稱）；欄位不存在的分支就略過該欄位（與 to_dict() 沒有該欄位相同）。
# 表上沒有的 service 照舊整筆 to_dict()（2026-11-13 從 openpilot-test 移植，dp 分支原本全部 to_dict()）。modelV2 另由 compact_model_v2 處理。
# 有沒有前車：dragonpilot 舊 cereal 叫 status，openpilot / sunnypilot 新版改名 present（同 @11）。兩個都列，
# 不存在的那個會略過。只列 status 時新版分支 leadOne 沒有這個欄位，opview 判定「沒有前車」（鎖定框、車距都不見，2026-11-13）。
OPVIEW_LEAD_FIELDS = ("status", "present", "dRel", "yRel", "vRel", "vLead", "aLeadK", "modelProb", "radar")
OPVIEW_COMPACT_FIELDS: dict[str, dict[str, Any]] = {
  "carState": dict.fromkeys(("vEgo", "aEgo", "steeringAngleDeg", "leftBlinker", "rightBlinker", "leftBlindspot",
                             "rightBlindspot", "gearShifter", "standstill", "gasPressed", "vEgoCluster", "vCruiseCluster")) |
              {"cruiseState": ("enabled",)},  # opview Auto Hold 按鈕：巡航中不顯示
  "selfdriveState": dict.fromkeys(("enabled", "experimentalMode", "alertText1", "alertText2", "alertSize",
                                   "alertStatus", "state", "personality")),  # personality：opview 橫向 HUD（吉米）
  "controlsState": dict.fromkeys(("vCruiseDEPRECATED",)),
  "radarState": {"leadOne": OPVIEW_LEAD_FIELDS, "leadTwo": OPVIEW_LEAD_FIELDS},
  "longitudinalPlan": dict.fromkeys(("allowThrottle",)),
  # memoryUsagePercent / cpuUsagePercent / maxTempC：opview 橫向 HUD（吉米）的 MEM / CPU / TEMP
  "deviceState": dict.fromkeys(("deviceType", "started", "memoryUsagePercent", "cpuUsagePercent", "maxTempC")),
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

# opview Auto Hold 按鈕：card 寫入的 OpviewAutoHoldState（bit0 可用、bit1 已暫停、bit2 保持中），
# 附在 carState 訊息的 "opviewAutoHold" 欄位送給 opview。Params 每 0.2 秒最多讀一次。
OPVIEW_AUTO_HOLD_READ_INTERVAL = 0.2


class OpviewAutoHoldReader:
  def __init__(self):
    self.params = Params()
    self.last_read = 0.0
    self.value: dict[str, bool] = {"available": False, "paused": False, "holding": False}

  def get(self) -> dict[str, bool]:
    now = time.monotonic()
    if now - self.last_read >= OPVIEW_AUTO_HOLD_READ_INTERVAL:
      self.last_read = now
      try:
        state = int(self.params.get("OpviewAutoHoldState") or 0)
      except Exception:
        state = 0
      self.value = {"available": bool(state & 1), "paused": bool(state & 2), "holding": bool(state & 4)}
    return self.value


# opview 橫向 HUD 的 eGPU 狀態：附在 deviceState 訊息的 "opviewEgpu" 欄位送給 opview。Params 每 1 秒最多讀一次。
#   {"present": 偵測到 eGPU, "active": 模型正在 eGPU 上跑, "loading": 大模型載入中, "compiled": 大模型已編譯}
# dragonpilot 系（dp0111pre）：modeld 啟動時寫入 UsbGpuPresent（偵測到 eGPU）、UsbGpuCompiled（大模型已編譯）；
#   兩者都成立時 modeld 用 eGPU 跑大模型（modeld.py：USBGPU = _present and _compiled）。
# dp0111jetlink（與 dp0111pre 不同，刻意）：大模型也可能由 Jetlink（USB-C 外接 Jetson / 手機 / Mac）計算。
#   沒有 USB GPU、JetlinkLink 開啟時改讀 jetlink 的狀態快照（jetlink_adapter.status()：只讀檔案與參數、不碰連線）：
#     present  = 外接裝置在 USB gadget 上（Status.present）
#     active   = 大模型正在驅動 modeld（Status.active，modeld 的 runtime 紀錄）
#     loading  = 連線 / 切換中（joining、retrying）或停車準備中（progress 階段不是 ready / failed）
#     compiled = 大模型引擎已建好（Status.runnable）
#   另附 "source": "jetlink" / "usbgpu"（舊版 opview 忽略不認得的欄位）。
# 參數不存在（其他分支）一律視為 False，不會丟例外。
OPVIEW_EGPU_READ_INTERVAL = 1.0
OPVIEW_JETLINK_LOADING_STATES = frozenset({"joining", "retrying"})


class OpviewEgpuReader:
  def __init__(self):
    self.params = Params()
    self.last_read = -OPVIEW_EGPU_READ_INTERVAL
    self.value: dict[str, bool] = {"present": False, "active": False, "loading": False, "compiled": False}

  def _get_bool(self, key: str) -> bool:
    try:
      return bool(self.params.get_bool(key))
    except Exception:
      return False

  def _link_on(self) -> bool:
    try:
      return int(self.params.get("JetlinkLink") or 0) != 0
    except Exception:
      return False

  @staticmethod
  def _jetlink() -> dict | None:
    """Jetlink 的狀態快照轉成 opviewEgpu；jetlink 不存在 / 出錯時 None（jetlink_adapter 的 hook 本身不丟例外）"""
    try:
      from dragonpilot import jetlink_adapter
      st = jetlink_adapter.status()
    except Exception:
      return None
    if st is None:
      return None
    stage = str((st.progress or {}).get("stage", "")) if isinstance(st.progress, dict) else ""
    loading = (st.running in OPVIEW_JETLINK_LOADING_STATES) or (stage not in ("", "ready", "failed"))
    active = bool(st.active)
    return {"present": bool(st.present), "active": active, "loading": bool(loading and not active),
            "compiled": bool(st.runnable), "source": "jetlink"}

  def get(self) -> dict:
    now = time.monotonic()
    if now - self.last_read >= OPVIEW_EGPU_READ_INTERVAL:
      self.last_read = now
      present = self._get_bool("UsbGpuPresent")
      compiled = self._get_bool("UsbGpuCompiled")
      value = None
      if not present and self._link_on():
        value = self._jetlink()
      if value is None:
        value = {"present": present, "active": present and compiled, "loading": False, "compiled": compiled,
                 "source": "usbgpu"}
      self.value = value
    return self.value


# opview 吉米三線道模型（v2.0.9）：雷達點雲精簡版，附在 radarState 訊息的 "opviewTracks" 欄位送給 opview。
# 用途：讓手機的相機車種辨識目標對到雷達點，40 m 內的位置改用雷達量到的距離 / 橫向。
# 只送 measured、前方 0–45 m 的點（opview 只要求 40 m 內準確，多 5 m 給配對容錯），每點 [trackId, dRel, yRel, vRel]
# 小數 1 位，10 Hz（radarState 20 Hz，隔一筆附一次）。實測 TSS2 rlog 每筆約 7 點、≤ 0.25 KB → ≤ 20 kbps，
# 約影像串流（5 Mbps）的 0.4%。舊版 opview 會忽略這個欄位；沒有 radarTracks / liveTracks 的分支不附。
OPVIEW_TRACKS_MAX_D = 45.0
OPVIEW_TRACKS_INTERVAL = 0.1
OPVIEW_TRACKS_STALE = 0.5


def _opview_measured(p) -> bool | None:
  """雷達點的 measured；新版 opendbc 把 measured 移進 deprecated 群組（直接讀會 AttributeError）→ None（不提供）"""
  try:
    return bool(p.measured)
  except Exception:
    return None


class OpviewTracksReader:
  def __init__(self):
    self.name: str | None = None
    self.sock = None
    try:
      for name in ("radarTracks", "liveTracks"):  # openpilot 新版改名為 radarTracks；dragonpilot 舊 cereal 為 liveTracks
        if name in SERVICE_LIST:
          self.name = name
          break
      if self.name is not None:
        self.sock = messaging.sub_sock(self.name, conflate=True)
    except Exception:
      self.sock = None
    self.last_attach = 0.0
    self.received_at = 0.0
    self.points: list[list[float]] = []
    # 診斷（比照 sp 參考 2026-11-20）：每秒附一次 opviewTracksSrc，opview log 可看出 comma 端有沒有收到雷達軌跡、過濾前後各幾點
    self.last_info = 0.0
    self.rx = 0     # 這一秒收到幾筆雷達軌跡訊息
    self.raw = 0    # 最近一筆過濾前的點數
    self.meas = 0   # 最近一筆 measured 的點數
    self.err = ""

  def info(self) -> dict | None:
    """每秒一次的診斷資訊；其他時候 None"""
    now = time.monotonic()
    if now - self.last_info < 1.0:
      return None
    self.last_info = now
    out = {"src": self.name or "", "rx": self.rx, "raw": self.raw, "meas": self.meas,
           "age": round(now - self.received_at, 1) if self.received_at > 0 else -1}
    if self.err:
      out["err"] = self.err[:80]
    self.rx = 0
    return out

  def get(self) -> list[list[float]] | None:
    """None = 這次不附（10 Hz 節流或沒有雷達點資料來源）"""
    if self.sock is None:
      return None
    now = time.monotonic()
    if now - self.last_attach < OPVIEW_TRACKS_INTERVAL:
      return None
    self.last_attach = now
    try:
      msg = messaging.recv_one_or_none(self.sock)
      if msg is not None:
        pts = getattr(msg, self.name).points
        self.rx += 1
        self.raw = len(pts)
        flags = [_opview_measured(p) for p in pts]
        self.meas = sum(1 for f in flags if f)
        # opview 移植指南 6.1：有些 opendbc 不設 measured（點本身已用 VALID / 分數篩過），
        # 整筆沒有任何 measured 時不用 measured 過濾，否則一個點都送不出去（dp0111pre 參考版只用 p.measured）
        use_meas = self.meas > 0
        self.points = [[int(p.trackId), round(p.dRel, 1), round(p.yRel, 1), round(p.vRel, 1)]
                       for p, f in zip(pts, flags)
                       if (f or not use_meas) and 0.0 < p.dRel <= OPVIEW_TRACKS_MAX_D]
        self.received_at = now
        self.err = ""
    except Exception as e:
      self.err = repr(e)
      self.points = []
    if now - self.received_at > OPVIEW_TRACKS_STALE:
      self.points = []
    return self.points


class CerealOutgoingMessageProxy(AsyncTaskRunner):
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

  # opview：data channel 是可靠、有序的，aiortc 的送出佇列沒有上限。
  # 網路（例如 iPhone 熱點）或 comma CPU 跟不上時，舊訊息會在佇列裡越積越多、照順序晚到，
  # 手機上就會看到「車停了還顯示 98 km/h」這種落後好幾秒到幾十秒的畫面。
  # 佇列超過上限時這一輪不送（SubMaster 只留最新值，下一輪送的就是最新資料），讓延遲維持在上限以內。
  MAX_BUFFERED_BYTES = 256 * 1024
  # opview 只在 modelV2（20 Hz）到達時重畫：carState、controlsState 這類 100 Hz 的 service 多送也看不到，
  # 每個 service 最多約 33 Hz（間隔 30 ms）。20 Hz 以下的 service（含 modelV2）不受影響。
  MIN_SEND_INTERVAL = 0.03

  def __init__(self, sm: messaging.SubMaster):
    super().__init__()
    self.sm = sm
    self.channels: list[RTCDataChannel] = []
    self.skipped = 0
    self.congested_since: float | None = None
    self.last_sent: dict[str, float] = {}
    self.auto_hold = OpviewAutoHoldReader()
    self.egpu = OpviewEgpuReader()
    self.tracks = OpviewTracksReader()

  def add_channel(self, channel: 'RTCDataChannel'):
    self.channels.append(channel)

  def update(self):
    # this is blocking in async context...
    self.sm.update(0)
    ready = [ch for ch in self.channels if ch.bufferedAmount <= self.MAX_BUFFERED_BYTES]
    n_updated = sum(1 for updated in self.sm.updated.values() if updated)
    if len(ready) < len(self.channels):
      if self.congested_since is None:
        self.congested_since = time.monotonic()
        buffered = max(ch.bufferedAmount for ch in self.channels)
        self.logger.warning("data channel congested (%d bytes queued), dropping stale messages", buffered)
      self.skipped += n_updated
    elif self.congested_since is not None:
      self.logger.warning("data channel recovered after %.1fs, dropped %d messages",
                          time.monotonic() - self.congested_since, self.skipped)
      self.congested_since = None
      self.skipped = 0
    if not ready:
      return
    now = time.monotonic()
    for service, updated in self.sm.updated.items():
      if not updated:
        continue
      if now - self.last_sent.get(service, 0.0) < self.MIN_SEND_INTERVAL:
        continue
      self.last_sent[service] = now
      if service == "modelV2":
        msg_dict = compact_model_v2(self.sm[service])
      elif service in OPVIEW_COMPACT_FIELDS:
        msg_dict = compact_fields(self.sm[service], OPVIEW_COMPACT_FIELDS[service])
        if service == "carState":
          msg_dict["opviewAutoHold"] = self.auto_hold.get()
        elif service == "deviceState":
          msg_dict["opviewEgpu"] = self.egpu.get()
        elif service == "radarState":
          tracks = self.tracks.get()
          if tracks is not None:
            msg_dict["opviewTracks"] = tracks
          info = self.tracks.info()
          if info is not None:
            msg_dict["opviewTracksSrc"] = info
      else:
        msg_dict = self.to_json(self.sm[service])
      mono_time, valid = self.sm.logMonoTime[service], self.sm.valid[service]
      outgoing_msg = {"type": service, "logMonoTime": mono_time, "valid": valid, "data": msg_dict}
      encoded_msg = json.dumps(outgoing_msg).encode()
      for channel in ready:
        channel.send(encoded_msg)

  async def run(self):
    from aiortc.exceptions import InvalidStateError

    while True:
      try:
        self.update()
      except InvalidStateError:
        self.logger.warning("Cereal outgoing proxy invalid state (connection closed)")
        break
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
  down_samples = 5 # 1s
  param_name = "LivestreamEncoderBitrate"

  def __init__(self, peer_connection: Any):
    super().__init__()
    self.pc = peer_connection
    self.params = Params()

    self.level = 0
    self.prev_lost, self.prev_sent = None, None
    self.counter = 0
    self.up_samples = 5 # 1s
    self._auto = True

  async def run(self):
    while True:
      await asyncio.sleep(self.sample_interval)
      if not self._auto:
        continue

      loss_rate = await self._sample()
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

  async def _sample(self) -> float | None:
    report = await self.pc.getStats()
    packets_lost = packets_sent = 0
    for s in report.values():
      if s.type == "remote-inbound-rtp":
        packets_lost += s.packetsLost
      elif s.type == "outbound-rtp":
        packets_sent += s.packetsSent

    if self.prev_lost is None:
      self.prev_lost, self.prev_sent = packets_lost, packets_sent
      return None
    lost_delta = max(0, packets_lost - self.prev_lost)
    sent_delta = max(0, packets_sent - self.prev_sent)
    self.prev_lost, self.prev_sent = packets_lost, packets_sent
    return lost_delta / sent_delta if sent_delta else 0.0

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

  def __init__(self, sdp: str, init_camera: str, incoming_services: list[str], outgoing_services: list[str], debug_mode: bool = False):
    from aiortc.mediastreams import VideoStreamTrack
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from teleoprtc import WebRTCAnswerBuilder

    builder = WebRTCAnswerBuilder(sdp)

    self.video_track = LiveStreamVideoStreamTrack(init_camera) if not debug_mode else VideoStreamTrack()
    builder.add_video_stream(init_camera, self.video_track)

    self.stream = builder.stream()
    self.identifier = str(uuid.uuid4())

    self.incoming_bridge: CerealIncomingMessageProxy | None = None
    self.incoming_bridge_services = incoming_services
    self.outgoing_bridge: CerealOutgoingMessageProxy | None = None
    self.bitrate_controller: LivestreamBitrateController | None = None
    if len(incoming_services) > 0:
      self.incoming_bridge = CerealIncomingMessageProxy(self.shared_pub_master)
    if len(outgoing_services) > 0:
      self.outgoing_bridge = CerealOutgoingMessageProxy(messaging.SubMaster(outgoing_services))
    self.bitrate_controller = LivestreamBitrateController(self.stream.peer_connection)

    self.run_task: asyncio.Task | None = None
    self._cleanup_lock = asyncio.Lock()
    self._cleanup_done = False
    self.logger = logging.getLogger("webrtcd")
    self.logger.info(
      "New stream session (%s), init camera %s, incoming services %s, outgoing services %s",
      self.identifier, init_camera, incoming_services, outgoing_services,
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
            self.video_track.switch_camera(payload["data"]["camera"])
          case "livestreamSettings":
            self.bitrate_controller.set_quality(payload["data"]["quality"])
          case "clockSync":
            pong = json.dumps({"type": "clockSync", "data": {
              "action": "pong", "browserSendTime": payload["data"]["browserSendTime"], "deviceTime": time.time() * 1000, # noqa: TID251
            }})
            self.stream.get_messaging_channel().send(pong)
          case "enableTimingSei":
            if hasattr(self.video_track, 'timing_sei_enabled'):
              self.video_track.timing_sei_enabled = bool(payload["data"]["enabled"])
          case _:
            if payload.get("type") not in self.incoming_bridge_services:
              return
            if self.incoming_bridge is not None:
              self.incoming_bridge.send(message)
    except Exception:
      self.logger.exception("Cereal incoming proxy failure")

  async def run(self):
    try:
      self.stream.set_message_handler(self.message_handler)
      await self.stream.wait_for_connection()
      if self.stream.has_messaging_channel():
        if self.incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self.incoming_bridge_services)
        if self.outgoing_bridge is not None:
          channel = self.stream.get_messaging_channel()
          self.outgoing_bridge.add_channel(channel)
          self.outgoing_bridge.start()
      self.bitrate_controller.start()

      self.logger.info("Stream session (%s) connected", self.identifier)

      await self.stream.wait_for_disconnection()

      self.logger.info("Stream session (%s) ended", self.identifier)
    except Exception:
      self.logger.exception("Stream session failure")
    finally:
      await self.post_run_cleanup()

  async def post_run_cleanup(self):
    async with self._cleanup_lock:
      if self._cleanup_done:
        return
      self._cleanup_done = True
      if self.bitrate_controller is not None:
        await self.bitrate_controller.stop()
      if self.outgoing_bridge is not None:
        await self.outgoing_bridge.stop()
      await self.stream.stop()


# opview 已內建 TDX（手機 GPS + TDX API），comma 的 tdx 不再傳給 opview：
# 請求若帶 tdx（舊版 opview）就從 bridge_services_out 拿掉，不訂閱、不轉送。tdx 只有 opview 會訂閱，
# 其他 client 不受影響。tdxd 本身照常執行（comma 端畫面仍可使用）。
DROPPED_OUTGOING_SERVICES = frozenset({"tdx"})


def drop_unforwarded_services(services: list[str]) -> list[str]:
  return [s for s in services if s not in DROPPED_OUTGOING_SERVICES]


VALID_INIT_CAMERAS = ("road", "wideRoad", "driver")
LOCAL_ADDRESSES = frozenset({"127.0.0.1", "::1", "localhost"})


def is_local_request(request) -> bool:
  remote = getattr(request, "remote", None) or ""
  return remote in LOCAL_ADDRESSES or remote.startswith("::ffff:127.")


@dataclass
class StreamRequestBody:
  sdp: str
  initCamera: str
  bridge_services_in: list[str] = field(default_factory=list)
  bridge_services_out: list[str] = field(default_factory=list)


async def get_stream(request: 'web.Request'):
  stream_dict, debug_mode = request.app['streams'], request.app['debug']
  raw_body = await request.json()
  # dp - 相容舊版請求（opview 舊版、其他舊工具）：舊格式送 "cameras": ["road"]，新格式為 "initCamera": "road"
  if "initCamera" not in raw_body and "cameras" in raw_body:
    cameras = raw_body.pop("cameras") or ["road"]
    raw_body["initCamera"] = cameras[0]
  # dp - 忽略不認得的欄位，避免 StreamRequestBody(**raw_body) 拋 TypeError（回 500）
  known = StreamRequestBody.__dataclass_fields__.keys()
  raw_body = {k: v for k, v in raw_body.items() if k in known}
  try:
    body = StreamRequestBody(**raw_body)
  except TypeError as e:
    raise web.HTTPBadRequest(text=json.dumps({"error": "invalid_request", "message": str(e)}),
                             content_type="application/json") from e
  body.bridge_services_out = drop_unforwarded_services(body.bridge_services_out)
  # dp - opview 移植指南 F3（安全）：webrtcd 聽 0.0.0.0，只允許本機帶 bridge_services_in，
  # 不能讓區網任何人往 comma 發 cereal 訊息（testJoystick、can…）。opview 一律送空陣列。
  if body.bridge_services_in and not is_local_request(request):
    raise web.HTTPForbidden(text=json.dumps({"error": "forbidden", "message": "bridge_services_in is only allowed from localhost"}),
                            content_type="application/json")
  # 移植指南 3.3：initCamera 只能是 road / wideRoad / driver
  if body.initCamera not in VALID_INIT_CAMERAS:
    raise web.HTTPBadRequest(text=json.dumps({"error": "invalid_camera", "message": f"unknown camera {body.initCamera!r}"}),
                             content_type="application/json")

  async with request.app['stream_lock']:
    # Fully disconnect any other active stream before starting the replacement.
    for sid, s in list(stream_dict.items()):
      if s.run_task and not s.run_task.done():
        try:
          ch = s.stream.get_messaging_channel()
          ch.send(json.dumps({"type": "connectionReplaced", "data": "Another device has connected, closing this session."}))
        except Exception:
          pass
      await s.stop()
      del stream_dict[sid]

    session = StreamSession(body.sdp, body.initCamera, body.bridge_services_in, body.bridge_services_out, debug_mode)
    try:
      answer = await session.get_answer()
    except ValueError as e:
      await session.stop()
      raise web.HTTPBadRequest(
        text=json.dumps({"error": "invalid_sdp", "message": str(e)}),
        content_type="application/json",
      ) from e
    except Exception:
      await session.stop()
      raise
    session.start()

    stream_dict[session.identifier] = session

  return web.json_response({"sdp": answer.sdp, "type": answer.type})


async def get_schema(request: 'web.Request'):
  services = request.query["services"].split(",")
  services = [s for s in services if s]
  assert all(s in log.Event.schema.fields and not s.endswith("DEPRECATED") for s in services), "Invalid service name"
  schema_dict = {s: generate_field(log.Event.schema.fields[s]) for s in services}
  return web.json_response(schema_dict)


async def post_notify(request: 'web.Request'):
  try:
    payload = await request.json()
  except Exception as e:
    raise web.HTTPBadRequest(text="Invalid JSON") from e

  for session in list(request.app.get('streams', {}).values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps(payload))
    except Exception:
      continue

  return web.Response(status=200, text="OK")


async def on_shutdown(app: 'web.Application'):
  for session in app['streams'].values():
    await session.stop()
  del app['streams']


def webrtcd_thread(host: str, port: int, debug: bool):
  logging.basicConfig(level=logging.CRITICAL, handlers=[logging.StreamHandler()])
  logging_level = logging.DEBUG if debug else logging.INFO
  logging.getLogger("WebRTCStream").setLevel(logging_level)
  logging.getLogger("webrtcd").setLevel(logging_level)

  app = web.Application()

  app['streams'] = dict()
  app['stream_lock'] = asyncio.Lock()
  app['debug'] = debug
  app.on_shutdown.append(on_shutdown)
  app.router.add_post("/stream", get_stream)
  app.router.add_post("/notify", post_notify)
  app.router.add_get("/schema", get_schema)

  web.run_app(app, host=host, port=port)


def main():
  parser = argparse.ArgumentParser(description="WebRTC daemon")
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  parser.add_argument("--debug", action="store_true", help="Enable debug mode")
  args = parser.parse_args()

  webrtcd_thread(args.host, args.port, args.debug)


if __name__=="__main__":
  main()
