#!/usr/bin/env python3
"""
dp - TDX 高公局路況預警 daemon (tdxd)

主執行緒：讀 GPS -> 路段匹配 -> 1Hz 發布 tdx。
背景執行緒：所有網路存取（線形圖資每日更新、LiveEvents / LiveTraffic 每 60 秒更新），不阻塞主迴圈。

線形圖資：
  - repo 內 freeway_shapes.json 為唯讀備援，執行時不再覆寫
  - 下載結果寫入 CACHE_SHAPES_PATH（/data 下），先驗證內容再以 tmp + os.replace 原子替換
  - 快取未滿 SHAPES_MAX_AGE_S 不重新下載

TLS：等系統時間有效後才連線並驗證憑證；若主機仍驗證失敗，記錄 warning 後對該主機改用不驗證連線。
"""
import json
import math
import os
import re
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from urllib.parse import urlparse

import requests
import urllib3

import cereal.messaging as messaging
from cereal import custom
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths

# 只在 TLS 驗證失敗而改用不驗證連線時才會用到，避免 log 被 InsecureRequestWarning 洗版
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

Congestion = custom.Tdx.Congestion

# ==========================================
# 設定
# ==========================================
SHAPES_URL = "https://tdx.transportdata.tw/api/basic/v2/Road/Traffic/SectionShape/Freeway?$format=JSON"
SHAPES_HEADERS = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) openpilot/tdxd'}
LIVE_HEADERS = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}

BUNDLED_SHAPES_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "freeway_shapes.json")
CACHE_DIR = os.path.join(Paths.comma_home(), "media", "0", "tdx") if PC else "/data/media/0/tdx"
CACHE_SHAPES_PATH = os.path.join(CACHE_DIR, "freeway_shapes.json")

SHAPES_MAX_AGE_S = 24 * 3600     # 快取一天內不重新下載
SHAPES_RETRY_INTERVAL_S = 600    # 下載失敗後重試間隔
SHAPES_MAX_ATTEMPTS = 3          # 每次啟動最多嘗試次數
SHAPES_TIMEOUT_S = 15
MIN_VALID_SECTIONS = 300         # 現行圖資 465 筆；低於此數視為異常內容，不採用

LIVE_UPDATE_INTERVAL_S = 60
LIVE_TIMEOUT_S = 5

TOPOLOGY_MAX_GAP_M = 100.0       # 前一段終點與下一段起點的最大距離（原 50m，大量相鄰路段接不上）
TOPOLOGY_MAX_ANGLE_DEG = 45.0    # 方位差上限，排除對向車道

AHEAD_DISTANCE_M = 3000.0
CALC_INTERVAL_S = 1.0
MAX_HORIZONTAL_ACCURACY = 50.0
SMOOTH_COUNT = 3
CURRENT_SECTION_MISS_LIMIT = 2


# ==========================================
# 網路工具
# ==========================================
_insecure_hosts: set[str] = set()


def _http_get(url, headers, timeout):
  """先以 TLS 驗證連線；該主機驗證失敗時記錄並改用不驗證連線（之後同主機直接走不驗證）。"""
  host = urlparse(url).hostname or ""
  if host not in _insecure_hosts:
    try:
      return requests.get(url, headers=headers, timeout=timeout)
    except requests.exceptions.SSLError as e:
      cloudlog.warning(f"tdxd: TLS 驗證失敗 ({host})，改用不驗證連線: {e}")
      _insecure_hosts.add(host)
  return requests.get(url, headers=headers, timeout=timeout, verify=False)


# ==========================================
# 線形圖資：驗證、讀取、下載
# ==========================================
def _validate_shapes(data):
  """回傳 SectionShapes list；內容不符預期時回傳 None。"""
  raw_shapes = data.get("SectionShapes") if isinstance(data, dict) else data
  if not isinstance(raw_shapes, list):
    return None
  valid = sum(1 for item in raw_shapes
              if isinstance(item, dict) and item.get('SectionID') and str(item.get('Geometry', '')).upper().startswith('LINESTRING'))
  if valid < MIN_VALID_SECTIONS:
    return None
  return raw_shapes


def _load_shapes_file(path):
  try:
    with open(path, encoding='utf-8') as f:
      return _validate_shapes(json.load(f))
  except Exception as e:
    cloudlog.warning(f"tdxd: 讀取圖資失敗 {path}: {e}")
    return None


def _cache_is_fresh():
  try:
    age = time.time() - os.path.getmtime(CACHE_SHAPES_PATH)  # noqa: TID251 與檔案 mtime 比較，需用牆鐘
  except OSError:
    return False
  return 0 <= age < SHAPES_MAX_AGE_S


def _atomic_write_json(path, data):
  os.makedirs(os.path.dirname(path), exist_ok=True)
  fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".tmp_", suffix=".json")
  try:
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
      json.dump(data, f, ensure_ascii=False)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_path, path)
  except BaseException:
    try:
      os.unlink(tmp_path)
    except OSError:
      pass
    raise


def download_shapes():
  """下載線形圖資，驗證通過才原子寫入快取。回傳 SectionShapes list 或 None。"""
  try:
    response = _http_get(SHAPES_URL, SHAPES_HEADERS, SHAPES_TIMEOUT_S)
    response.raise_for_status()
    shapes = _validate_shapes(response.json())
    if shapes is None:
      cloudlog.warning("tdxd: 下載的圖資內容不符預期，保留原有圖資")
      return None
    _atomic_write_json(CACHE_SHAPES_PATH, {"SectionShapes": shapes})
    cloudlog.info(f"tdxd: 圖資下載成功，共 {len(shapes)} 筆路段 -> {CACHE_SHAPES_PATH}")
    return shapes
  except Exception as e:
    cloudlog.warning(f"tdxd: 圖資下載失敗: {e}")
    return None


def load_initial_shapes():
  """啟動時優先使用快取，快取無效時用 repo 內建檔案。"""
  if os.path.exists(CACHE_SHAPES_PATH):
    shapes = _load_shapes_file(CACHE_SHAPES_PATH)
    if shapes is not None:
      return shapes
  shapes = _load_shapes_file(BUNDLED_SHAPES_PATH)
  return shapes if shapes is not None else []


# ==========================================
# 工具函數：WKT 解析、距離與方位角運算
# ==========================================
def _parse_wkt_linestring(wkt_str):
  inner = re.search(r'LINESTRING\s*\((.+)\)', wkt_str, re.IGNORECASE)
  if not inner:
    return []
  pairs = inner.group(1).split(',')
  return [tuple(map(float, p.strip().split())) for p in pairs]


def _segment_bearing(lon1, lat1, lon2, lat2):
  dy = lat2 - lat1
  dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
  return math.degrees(math.atan2(dx, dy)) % 360


def _project_to_segment(px, py, ax, ay, bx, by):
  """回傳 (距離平方, 投影參數 t∈[0,1])"""
  dx, dy = bx - ax, by - ay
  seg_len_sq = dx * dx + dy * dy
  if seg_len_sq == 0:
    return (px - ax) ** 2 + (py - ay) ** 2, 0.0
  t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
  proj_x = ax + t * dx
  proj_y = ay + t * dy
  return (px - proj_x) ** 2 + (py - proj_y) ** 2, t


def _point_to_segment_dist_sq_and_bearing(px, py, ax, ay, bx, by):
  dist_sq, _ = _project_to_segment(px, py, ax, ay, bx, by)
  if (bx - ax) == 0 and (by - ay) == 0:
    return dist_sq, 0.0
  return dist_sq, _segment_bearing(ax, ay, bx, by)


def _angle_diff(a, b):
  diff = abs(a - b) % 360
  return diff if diff <= 180 else 360 - diff


def _get_distance_meters(lon1, lat1, lon2, lat2):
  dx = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2)) * 111320.0
  dy = (lat2 - lat1) * 111320.0
  return math.sqrt(dx * dx + dy * dy)


def bearing_to_direction(bearing_deg):
  b = bearing_deg % 360
  if b < 45 or b >= 315:
    return '北向'
  elif b < 135:
    return '東向'
  elif b < 225:
    return '南向'
  else:
    return '西向'


def _parse_event_type(event_type):
  try:
    value = int(event_type)
  except (TypeError, ValueError):
    return 0
  return value if 0 <= value <= 255 else 0


# ==========================================
# 1. 核心圖資配對引擎 (向量內積防護版)
# ==========================================
class LocalMapMatcher:
  def __init__(self, raw_shapes):
    self.sections = {}
    self.spatial_grid = {}
    self.grid_size = 0.01
    self.topology_next = {}

    try:
      for item in raw_shapes:
        geom_wkt = item.get('Geometry')
        sec_id = item.get('SectionID')
        if not geom_wkt or not sec_id:
          continue

        coords = _parse_wkt_linestring(geom_wkt)
        if len(coords) < 2:
          continue

        segments = []
        total_length = 0.0
        for i in range(len(coords) - 1):
          p1, p2 = coords[i], coords[i + 1]
          dist = _get_distance_meters(p1[0], p1[1], p2[0], p2[1])
          # start_dist：此線段起點距路段起點的累計距離，用於計算車輛到路段終點的剩餘距離
          segments.append({'p1': p1, 'p2': p2, 'dist': dist, 'start_dist': total_length})
          total_length += dist

        self.sections[sec_id] = {
          'id': sec_id,
          'coords': coords,
          'segments': segments,
          'total_length': total_length,
          'start_pt': coords[0],
          'end_pt': coords[-1],
        }

        min_lon = min(p[0] for p in coords)
        max_lon = max(p[0] for p in coords)
        min_lat = min(p[1] for p in coords)
        max_lat = max(p[1] for p in coords)

        min_gx, max_gx = int(min_lon / self.grid_size), int(max_lon / self.grid_size)
        min_gy, max_gy = int(min_lat / self.grid_size), int(max_lat / self.grid_size)

        for gx in range(min_gx, max_gx + 1):
          for gy in range(min_gy, max_gy + 1):
            self.spatial_grid.setdefault((gy, gx), []).append(sec_id)

      # 建立拓樸：終點與下一段起點距離 < TOPOLOGY_MAX_GAP_M 且方位差 < TOPOLOGY_MAX_ANGLE_DEG
      for sec_a, data_a in self.sections.items():
        end_pt = data_a['end_pt']
        last_seg = data_a['segments'][-1]
        end_bearing = _segment_bearing(last_seg['p1'][0], last_seg['p1'][1], last_seg['p2'][0], last_seg['p2'][1])
        best_next = None
        min_gap = float('inf')

        for sec_b, data_b in self.sections.items():
          if sec_a == sec_b:
            continue
          start_pt = data_b['start_pt']
          gap = _get_distance_meters(end_pt[0], end_pt[1], start_pt[0], start_pt[1])

          if gap < TOPOLOGY_MAX_GAP_M and gap < min_gap:
            first_seg = data_b['segments'][0]
            start_bearing = _segment_bearing(first_seg['p1'][0], first_seg['p1'][1], first_seg['p2'][0], first_seg['p2'][1])
            if _angle_diff(end_bearing, start_bearing) < TOPOLOGY_MAX_ANGLE_DEG:
              best_next = sec_b
              min_gap = gap

        if best_next:
          self.topology_next[sec_a] = best_next

      cloudlog.info(f"tdxd: 圖資載入 {len(self.sections)} 筆，建立 {len(self.topology_next)} 筆道路連結")
    except Exception:
      cloudlog.exception("tdxd: 建立圖資索引失敗")

  def _get_candidates_from_grid(self, lat, lon, radius_km=3):
    candidates = set()
    gy_center, gx_center = int(lat / self.grid_size), int(lon / self.grid_size)
    span = math.ceil((radius_km / 111.0) / self.grid_size)

    for gy in range(gy_center - span, gy_center + span + 1):
      for gx in range(gx_center - span, gx_center + span + 1):
        candidates.update(self.spatial_grid.get((gy, gx), []))
    return candidates

  def find_current_section(self, lat, lon, threshold_meters=300, bearing_deg=None, direction_text=None):
    candidates = self._get_candidates_from_grid(lat, lon, radius_km=2)
    if not candidates:
      return None

    best_match = None
    min_dist = float('inf')

    # 針對 XML 事件：建立文字方向向量
    evt_vec = None
    if direction_text:
      if '北' in direction_text:
        evt_vec = (0, 1)
      elif '南' in direction_text:
        evt_vec = (0, -1)
      elif '東' in direction_text:
        evt_vec = (1, 0)
      elif '西' in direction_text:
        evt_vec = (-1, 0)

    for sec_id in candidates:
      sec_data = self.sections[sec_id]

      # 利用向量內積，排除對向車道
      if evt_vec:
        start_pt = sec_data['start_pt']
        end_pt = sec_data['end_pt']
        sec_vec_x = end_pt[0] - start_pt[0]
        sec_vec_y = end_pt[1] - start_pt[1]
        dot_product = sec_vec_x * evt_vec[0] + sec_vec_y * evt_vec[1]
        if dot_product < 0:
          continue

      for seg in sec_data['segments']:
        dist_sq, local_bearing = _point_to_segment_dist_sq_and_bearing(
          lon, lat, seg['p1'][0], seg['p1'][1], seg['p2'][0], seg['p2'][1])

        dist_meters = math.sqrt(dist_sq) * 111320.0

        if dist_meters < min_dist:
          # 嚴格維持 60 度，避免十字交流道上下層誤判
          if bearing_deg is not None and _angle_diff(bearing_deg, local_bearing) > 60:
            continue

          min_dist = dist_meters
          best_match = sec_id

    if best_match and min_dist < threshold_meters:
      return best_match
    return None

  def distance_to_section_end(self, sec_id, lat, lon):
    """車輛投影到路段上的位置，到該路段終點的剩餘距離 (m)。"""
    sec = self.sections.get(sec_id)
    if sec is None:
      return 0.0

    best_dist_sq = float('inf')
    along = 0.0
    for seg in sec['segments']:
      dist_sq, t = _project_to_segment(lon, lat, seg['p1'][0], seg['p1'][1], seg['p2'][0], seg['p2'][1])
      if dist_sq < best_dist_sq:
        best_dist_sq = dist_sq
        along = seg['start_dist'] + t * seg['dist']
    return max(0.0, sec['total_length'] - along)

  def find_ahead_section(self, current_sec_id, dist_to_end_m, target_distance_m=AHEAD_DISTANCE_M):
    """回傳車輛前方 target_distance_m 處所在的路段。

    原寫法以「目前路段全長」比較，長度 >= 3km 的路段（佔多數）會直接回傳目前路段；
    改為從車輛位置到目前路段終點的剩餘距離開始累加。
    """
    if current_sec_id not in self.sections:
      return None

    accumulated_dist = dist_to_end_m
    current = current_sec_id

    for _ in range(20):
      if accumulated_dist >= target_distance_m:
        return current
      next_sec = self.topology_next.get(current)
      if next_sec is None:
        return current
      current = next_sec
      accumulated_dist += self.sections[current]['total_length']

    return current


# ==========================================
# 2. 高公局 XML 雙核心抓取器
# ==========================================
class FreewayDataClient:
  def __init__(self):
    self.events_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveEvents.xml"
    self.traffic_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveTraffic.xml"
    # 由背景執行緒整批替換（不就地修改），主執行緒讀取時不需加鎖
    self.cached_events = []
    self.cached_speeds = {}

  def update_data(self, matcher):
    try:
      res_evt = _http_get(self.events_url, LIVE_HEADERS, LIVE_TIMEOUT_S)
      res_evt.encoding = 'utf-8'
      xml_evt = re.sub(r'\sxmlns="[^"]+"', '', res_evt.text, count=1)
      root_evt = ET.fromstring(xml_evt)

      new_events = []
      for event in root_evt.findall('.//LiveEvent'):
        event_type = (event.findtext('EventType') or '0').strip()
        desc = event.findtext('Description', '未知事件')
        positions = event.findtext('Positions')
        direction = (event.findtext('.//Direction') or '').strip()

        if positions and positions.startswith('POINT('):
          coords_str = positions.replace('POINT(', '').replace(')', '').strip().split()
          if len(coords_str) == 2:
            evt_lon, evt_lat = float(coords_str[0]), float(coords_str[1])
            # 帶入 direction_text，讓內積演算法判斷對錯車道
            sec_id = matcher.find_current_section(evt_lat, evt_lon, threshold_meters=1000, direction_text=direction)

            # 就算找不到 SectionID 也存起來供空間雷達掃描用
            new_events.append({
              'EventType': event_type,
              'Description': desc,
              'SectionID': str(sec_id).strip() if sec_id else "",
              'lat': evt_lat,
              'lon': evt_lon,
              'direction': direction,
            })
      self.cached_events = new_events

      res_spd = _http_get(self.traffic_url, LIVE_HEADERS, LIVE_TIMEOUT_S)
      res_spd.encoding = 'utf-8'
      xml_spd = re.sub(r' xmlns="[^"]+"', '', res_spd.text, count=1)
      xml_spd = re.sub(r' xmlns:xsi="[^"]+"', '', xml_spd, count=1)
      xml_spd = re.sub(r' xsi:[^=]+="[^"]+"', '', xml_spd)
      root_spd = ET.fromstring(xml_spd)

      new_speeds = {}
      for traffic in root_spd.findall('.//LiveTraffic'):
        link_id = (traffic.findtext('SectionID') or '').strip()
        speed_str = (traffic.findtext('TravelSpeed') or '').strip()
        if link_id and speed_str:
          spd = int(speed_str)
          if spd > 0:
            new_speeds[link_id] = spd
      self.cached_speeds = new_speeds
      cloudlog.debug(f"tdxd: 快取 {len(self.cached_events)} 筆事件, {len(self.cached_speeds)} 筆有效車速")
    except Exception as e:
      cloudlog.warning(f"tdxd: LiveEvents/LiveTraffic 更新失敗: {e}")


# ==========================================
# 3. 背景網路執行緒
# ==========================================
class NetworkWorker:
  def __init__(self, matcher):
    self.matcher = matcher  # 主執行緒讀取；更新圖資時整個物件替換
    self.client = FreewayDataClient()
    self._shapes_attempts = 0
    self._next_shapes_attempt = 0.0
    self._last_live_update = -float('inf')
    self._thread = threading.Thread(target=self._run, name="tdxd_net", daemon=True)

  def start(self):
    self._thread.start()

  def _maybe_update_shapes(self, now):
    if self._shapes_attempts >= SHAPES_MAX_ATTEMPTS or now < self._next_shapes_attempt or _cache_is_fresh():
      return
    self._shapes_attempts += 1
    shapes = download_shapes()
    if shapes is None:
      self._next_shapes_attempt = now + SHAPES_RETRY_INTERVAL_S
      return
    self._shapes_attempts = SHAPES_MAX_ATTEMPTS  # 本次啟動已成功，不再下載
    self.matcher = LocalMapMatcher(shapes)

  def _run(self):
    while True:
      try:
        # 時鐘未同步前 TLS 憑證驗證必定失敗，先等系統時間有效
        if not system_time_valid():
          time.sleep(5)
          continue

        # 即時路況優先，圖資下載（最長 SHAPES_TIMEOUT_S）排在後面
        now = time.monotonic()
        if now - self._last_live_update >= LIVE_UPDATE_INTERVAL_S:
          self._last_live_update = now
          self.client.update_data(self.matcher)

        self._maybe_update_shapes(time.monotonic())
      except Exception:
        cloudlog.exception("tdxd: 網路執行緒例外")
      time.sleep(1)


# ==========================================
# 4. 主循環
# ==========================================
def main():
  cloudlog.info("tdxd: 啟動 (空間雷達掃描版)")
  pm = messaging.PubMaster(['tdx'])

  worker = NetworkWorker(LocalMapMatcher(load_initial_shapes()))
  worker.start()

  sm = messaging.SubMaster(['liveGPS', 'gpsLocationExternal'],
                           ignore_alive=['liveGPS', 'gpsLocationExternal'])

  last_calc_time = 0.0
  ahead_section_history = []
  stable_ahead_section = None
  stable_current_section = None
  current_section_miss_count = 0
  last_log_state = None

  while True:
    sm.update()
    current_time = time.monotonic()

    gps_source = None
    is_gps_ready = False

    if sm.updated.get('liveGPS', False):
      live_gps = sm['liveGPS']
      if live_gps.gpsOK and (live_gps.horizontalAccuracy <= 0 or
                             live_gps.horizontalAccuracy <= MAX_HORIZONTAL_ACCURACY):
        lat = live_gps.latitude
        lon = live_gps.longitude
        bearing = live_gps.bearingDeg
        is_gps_ready = True
        gps_source = 'liveGPS'

    if not is_gps_ready and sm.updated.get('gpsLocationExternal', False):
      raw_gps = sm['gpsLocationExternal']
      acc = getattr(raw_gps, 'horizontalAccuracy', 0.0)
      if acc <= 0 or acc <= MAX_HORIZONTAL_ACCURACY:
        lat = raw_gps.latitude
        lon = raw_gps.longitude
        bearing = raw_gps.bearingDeg
        is_gps_ready = True
        gps_source = 'gpsLocationExternal'

    if not (is_gps_ready and (current_time - last_calc_time >= CALC_INTERVAL_S)):
      continue
    last_calc_time = current_time

    matcher = worker.matcher
    client = worker.client
    my_direction = bearing_to_direction(bearing)

    raw_current_section = matcher.find_current_section(lat, lon, threshold_meters=300, bearing_deg=bearing)

    if raw_current_section is not None:
      stable_current_section = raw_current_section
      current_section_miss_count = 0
    else:
      current_section_miss_count += 1
      if current_section_miss_count >= CURRENT_SECTION_MISS_LIMIT:
        stable_current_section = None

    # 圖資更新後舊 SectionID 可能已不存在
    if stable_current_section is not None and stable_current_section not in matcher.sections:
      stable_current_section = None

    if stable_current_section is not None:
      dist_to_end = matcher.distance_to_section_end(stable_current_section, lat, lon)
      raw_ahead_section = matcher.find_ahead_section(stable_current_section, dist_to_end)

      ahead_section_history.append(raw_ahead_section)
      if len(ahead_section_history) > SMOOTH_COUNT:
        ahead_section_history.pop(0)

      if len(ahead_section_history) == SMOOTH_COUNT and len(set(ahead_section_history)) == 1:
        stable_ahead_section = ahead_section_history[0]
    else:
      ahead_section_history.clear()
      stable_ahead_section = None

    # ----------------------------------------------------
    # 填寫 TDX 訊息
    # ----------------------------------------------------
    msg = messaging.new_message('tdx')
    tdx = msg.tdx

    curr_speed = client.cached_speeds.get(str(stable_current_section).strip(), -1) if stable_current_section else -1
    ahead_speed = client.cached_speeds.get(str(stable_ahead_section).strip(), -1) if stable_ahead_section else -1

    tdx.sectionId = str(stable_current_section) if stable_current_section else ""
    tdx.speed = int(curr_speed)
    tdx.nextSectionId = str(stable_ahead_section) if stable_ahead_section else ""
    tdx.nextSpeed = int(ahead_speed)

    if ahead_speed <= 0:
      tdx.congestion = Congestion.unknown
    elif ahead_speed >= 80:
      tdx.congestion = Congestion.smooth
    elif ahead_speed >= 40:
      tdx.congestion = Congestion.slow
    else:
      tdx.congestion = Congestion.congested

    # ----------------------------------------------------
    # 處理事件
    # ----------------------------------------------------
    current_events = []
    ahead_events = []

    for evt in client.cached_events:
      evt_sid = evt.get('SectionID', '')
      evt_item = (_parse_event_type(evt.get('EventType', '0')), evt['Description'])
      evt_lat = evt.get('lat')
      evt_lon = evt.get('lon')
      evt_dir = evt.get('direction', '')

      # 【距離 + 角度混合檢查】
      is_physically_ahead = True
      if evt_lat and evt_lon:
        dist_to_evt = _get_distance_meters(lon, lat, evt_lon, evt_lat)
        evt_bearing = _segment_bearing(lon, lat, evt_lon, evt_lat)
        angle_diff = _angle_diff(bearing, evt_bearing)

        # 距離 <= 300m 無視角度強制顯示（避免接近時 GPS 亂跳導致閃爍）
        if dist_to_evt <= 300:
          is_physically_ahead = True
        # 距離 > 300m 且角度 > 90 度（確實落在車身後方）才取消
        elif angle_diff > 90:
          is_physically_ahead = False

      # 1. 目前路段比對
      if stable_current_section and evt_sid == str(stable_current_section).strip():
        if is_physically_ahead:
          current_events.append(evt_item)
        continue

      # 2. 前方路段比對
      is_ahead = False

      # 方法A：拓樸推導
      if stable_ahead_section and evt_sid == str(stable_ahead_section).strip():
        if is_physically_ahead:
          is_ahead = True

      # 方法B：空間雷達
      elif evt_lat and evt_lon and stable_current_section is not None:
        if dist_to_evt <= 3000 and _angle_diff(bearing, evt_bearing) <= 60:
          if evt_dir and (my_direction[0] in evt_dir):
            is_ahead = True
          elif not evt_dir:
            is_ahead = True

      if is_ahead:
        ahead_events.append(evt_item)

    # 目前 / 前方事件每 10 秒輪替優先顯示
    show_current_first = int(current_time // 10) % 2 == 0
    if show_current_first:
      selected, is_current = (current_events, True) if current_events else (ahead_events, False)
    else:
      selected, is_current = (ahead_events, False) if ahead_events else (current_events, True)

    events = tdx.init('events', len(selected))
    for i, (event_type, desc) in enumerate(selected):
      events[i].eventType = event_type
      events[i].description = desc
      events[i].isCurrentSection = is_current

    pm.send('tdx', msg)

    # 只在狀態改變時記錄，避免每秒寫入 log
    log_state = (gps_source, my_direction, tdx.sectionId, tdx.nextSectionId, tdx.nextSpeed,
                 is_current if selected else None, tuple(selected))
    if log_state != last_log_state:
      last_log_state = log_state
      evt_loc = '目前' if is_current else '前方'
      cloudlog.debug(f"tdxd: gps={gps_source} 航向={bearing:.1f}({my_direction}) 目前={tdx.sectionId or '無'} " +
                     f"前方={tdx.nextSectionId or '無'} 前方車速={tdx.nextSpeed} 事件={evt_loc}:{len(selected)}")


if __name__ == "__main__":
  main()
