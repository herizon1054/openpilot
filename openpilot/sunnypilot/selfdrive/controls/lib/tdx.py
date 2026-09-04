#!/usr/bin/env python3
import time
import math
import json
import re
import xml.etree.ElementTree as ET
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.cereal import log
from openpilot.tools.lib.url_file import URLFile


# ==========================================
# 工具函數：開機自動下載 TDX 線形圖資 (使用內部 URLFile 連線池與 atomic_write 斷電防護)
# ==========================================
def download_freeway_shapes_guest():
  print(f"[系統] 偵測到開機啟動，準備透過訪客額度下載最新 TDX 圖資...")
  url = "https://tdx.transportdata.tw/api/basic/v2/Road/Traffic/SectionShape/Freeway?$format=JSON"
  headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) openpilot/tdx'}

  try:
    # 使用 openpilot 內部連線池 (自帶 5 次指數退避重試)
    pool = URLFile.pool_manager()
    response = pool.request("GET", url, headers=headers, timeout=3, retries=False)
    if response.status not in (200, 206):
      raise Exception(f"HTTP 狀態碼異常: {response.status}")

    data = json.loads(response.data.decode('utf-8'))

    Params().put("TdxFreewayShapes", data)

    print(f"[系統] ✅ 圖資下載成功！(共 {len(data)} 筆路段)")
    return

  except Exception as e:
    print(f"[警告] ⚠️ 圖資下載失敗 ({e})。將沿用本地舊有圖資檔案。")


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


def _point_to_segment_dist_sq_and_bearing(px, py, ax, ay, bx, by):
  dx, dy = bx - ax, by - ay
  seg_len_sq = dx * dx + dy * dy
  if seg_len_sq == 0:
    return (px - ax) ** 2 + (py - ay) ** 2, 0.0

  t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
  proj_x = ax + t * dx
  proj_y = ay + t * dy
  dist_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2
  bearing = _segment_bearing(ax, ay, bx, by)
  return dist_sq, bearing


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


# ==========================================
# 1. 核心圖資配對引擎 (向量內積防護版)
# ==========================================
class LocalMapMatcher:
  def __init__(self):
    self.sections = {}
    self.spatial_grid = {}
    self.grid_size = 0.01
    self.topology_next = {}
    self.load()

  def load(self) -> bool:
    """(重新)從 Params 讀取 TdxFreewayShapes 並建立空間網格與幾何拓樸。
    容錯處理：若圖資從未成功下載過（全新安裝、開機當下無網路且無本地快取），
    Params().get() 會回傳 None，直接 iterate 會讓整個 tdx 行程在啟動階段崩潰、
    反覆重啟。這裡改為安全地視為「暫無圖資」，回傳 False 讓呼叫端可以之後重試，
    而不是讓整個行程崩潰。
    回傳值：True 表示本次成功載入至少一筆路段，False 表示圖資暫時不可用。
    """
    self.sections = {}
    self.spatial_grid = {}
    self.topology_next = {}

    data = Params().get("TdxFreewayShapes")
    raw_shapes = data.get("SectionShapes", data) if isinstance(data, dict) else data

    if not isinstance(raw_shapes, list) or len(raw_shapes) == 0:
      print("[圖資] ⚠️ 尚無可用的 TDX 圖資（可能是開機當下下載失敗且無本地快取），"
            "TDX 功能將暫時停用，稍後會自動重試下載。")
      return False

    print("[圖資] 正在建立空間網格與幾何拓樸...")

    for item in raw_shapes:
      if not isinstance(item, dict):
        continue
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
        total_length += dist
        segments.append({'p1': p1, 'p2': p2, 'dist': dist})

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
          grid_key = (gy, gx)
          if grid_key not in self.spatial_grid:
            self.spatial_grid[grid_key] = []
          self.spatial_grid[grid_key].append(sec_id)

    # 建立拓樸
    for sec_a, data_a in self.sections.items():
      end_pt = data_a['end_pt']
      end_bearing = _segment_bearing(
        data_a['segments'][-1]['p1'][0], data_a['segments'][-1]['p1'][1], data_a['segments'][-1]['p2'][0], data_a['segments'][-1]['p2'][1]
      )
      best_next = None
      min_gap = float('inf')

      for sec_b, data_b in self.sections.items():
        if sec_a == sec_b:
          continue
        start_pt = data_b['start_pt']
        gap = _get_distance_meters(end_pt[0], end_pt[1], start_pt[0], start_pt[1])

        if gap < 50.0 and gap < min_gap:
          start_bearing = _segment_bearing(
            data_b['segments'][0]['p1'][0], data_b['segments'][0]['p1'][1], data_b['segments'][0]['p2'][0], data_b['segments'][0]['p2'][1]
          )
          if _angle_diff(end_bearing, start_bearing) < 45:
            best_next = sec_b
            min_gap = gap

      if best_next:
        self.topology_next[sec_a] = best_next

    print(f"[圖資] ✅ 載入 {len(self.sections)} 筆，建立 {len(self.topology_next)} 筆道路連結！")
    return len(self.sections) > 0

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

      # [核心修復] 利用向量內積，徹底解決錯綁對向車道的問題
      if evt_vec:
        start_pt = sec_data['start_pt']
        end_pt = sec_data['end_pt']
        sec_vec_x = end_pt[0] - start_pt[0]
        sec_vec_y = end_pt[1] - start_pt[1]
        dot_product = sec_vec_x * evt_vec[0] + sec_vec_y * evt_vec[1]
        if dot_product < 0:  # 內積為負，代表這是反向車道，直接剔除！
          continue

      for seg in sec_data['segments']:
        dist_sq, local_bearing = _point_to_segment_dist_sq_and_bearing(lon, lat, seg['p1'][0], seg['p1'][1], seg['p2'][0], seg['p2'][1])

        dist_meters = math.sqrt(dist_sq) * 111320.0

        if dist_meters < min_dist:
          # [車輛防護] 嚴格維持 60 度，避免十字交流道上下層誤判
          if bearing_deg is not None and _angle_diff(bearing_deg, local_bearing) > 60:
            continue

          min_dist = dist_meters
          best_match = sec_id

    if best_match and min_dist < threshold_meters:
      return best_match
    return None

  def find_ahead_section(self, current_sec_id, target_distance_m=3000):
    if current_sec_id not in self.sections:
      return None

    accumulated_dist = 0.0
    current = current_sec_id

    for _ in range(20):
      sec_len = self.sections[current]['total_length']
      if accumulated_dist + sec_len >= target_distance_m:
        return current

      accumulated_dist += sec_len
      if current in self.topology_next:
        current = self.topology_next[current]
      else:
        return current

    return current


# ==========================================
# 2. 高公局 XML 雙核心抓取器 (使用 URLFile.pool_manager 避免磁碟分塊快取)
# ==========================================
class FreewayDataClient:
  def __init__(self):
    self.events_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveEvents.xml"
    self.traffic_url = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveTraffic.xml"
    self.headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
    self.cached_events = []
    self.cached_speeds = {}

  def update_data(self, matcher):
    print("[網路] 🔄 背景更新高公局 LiveEvents & LiveTraffic...")
    try:
      pool = URLFile.pool_manager()

      # 抓取即時事件 XML (完整載入記憶體進行解碼)
      res_evt = pool.request("GET", self.events_url, headers=self.headers, timeout=5)
      if res_evt.status != 200:
        raise Exception(f"LiveEvents HTTP {res_evt.status}")

      xml_evt_text = res_evt.data.decode('utf-8')
      xml_evt = re.sub(r'\sxmlns="[^"]+"', '', xml_evt_text, count=1)
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

            # 加入 lat, lon, direction，且就算找不到完美的 SectionID 也存起來供雷達掃描用
            new_events.append(
              {
                'EventType': event_type,
                'Description': desc,
                'SectionID': str(sec_id).strip() if sec_id else "",
                'lat': evt_lat,
                'lon': evt_lon,
                'direction': direction,
              }
            )
      self.cached_events = new_events

      # 抓取即時車速 XML
      res_spd = pool.request("GET", self.traffic_url, headers=self.headers, timeout=5)
      if res_spd.status != 200:
        raise Exception(f"LiveTraffic HTTP {res_spd.status}")

      xml_spd_text = res_spd.data.decode('utf-8')
      xml_spd = re.sub(r' xmlns="[^"]+"', '', xml_spd_text, count=1)
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
      print(f"[網路] ✅ 快取 {len(self.cached_events)} 筆事件, {len(self.cached_speeds)} 筆有效車速。")
    except Exception as e:
      print(f"[網路] 更新失敗: {e}")


# ==========================================
# 3. 主循環
# ==========================================
def main():
  print("🚀 啟動 tdxd 路況發布系統 (空間雷達掃描版)...")
  pm = messaging.PubMaster(['tdx'])

  download_freeway_shapes_guest()

  matcher = LocalMapMatcher()
  client = FreewayDataClient()

  # 【修復 v2】主要定位來源改用 liveLocationKalman（GPS+IMU+輪速融合），
  # gpsLocationExternal（原始GPS）僅作為備案，解決國道高架橋/隧道下 GPS 斷訊或精度暴衝時
  # 完全停止發佈車速與事件的問題。
  sm = messaging.SubMaster(['gpsLocationExternal', 'liveLocationKalman'], ignore_alive=['gpsLocationExternal', 'liveLocationKalman'])

  MAX_HORIZONTAL_ACCURACY = 50.0
  MAX_DEAD_RECKONING_SEC = 20.0  # liveLocationKalman 失去真實GPS校正超過此秒數就不再信任其位置
  last_gps_ok_time = 0
  last_api_call = 0
  UPDATE_INTERVAL = 60

  last_calc_time = 0
  CALC_INTERVAL = 1.0

  ahead_section_history = []
  SMOOTH_COUNT = 3
  stable_ahead_section = None
  stable_current_section = None

  CURRENT_SECTION_MISS_LIMIT = 2
  current_section_miss_count = 0

  # 【修復】圖資自我修復重試機制：若開機時圖資載入失敗（例如開機當下無網路且
  # 無本地快取），不讓行程崩潰，而是每隔 SHAPES_RETRY_INTERVAL 秒重新嘗試下載
  # 一次，一旦成功即自動恢復正常運作，不需要重開機或手動重啟行程。
  SHAPES_RETRY_INTERVAL = 300  # 5 分鐘
  last_shapes_retry = time.time()

  # ==========================================
  # 測試點設定 (可自行開關) True or False
  # ==========================================
  TEST_MODE = False
  TEST_LAT = 25.126123
  TEST_LON = 121.697325
  TEST_BEARING = 180.0
  # ==========================================

  while True:
    if TEST_MODE:
      time.sleep(0.1)
    else:
      sm.update()

    current_time = time.time()

    if len(matcher.sections) == 0 and (current_time - last_shapes_retry) >= SHAPES_RETRY_INTERVAL:
      print("[圖資] 🔄 偵測到圖資暫無資料，重新嘗試下載...")
      download_freeway_shapes_guest()
      matcher.load()
      last_shapes_retry = current_time

    if current_time - last_api_call >= UPDATE_INTERVAL:
      client.update_data(matcher)
      last_api_call = current_time

    gps_source = None
    is_gps_ready = False

    if TEST_MODE:
      is_gps_ready = True
      lat, lon, bearing = TEST_LAT, TEST_LON, TEST_BEARING
      gps_source = 'TEST_MODE'
    else:
      # 【修復 v2】主要來源：liveLocationKalman（融合定位，隧道/橋下短暫失鎖仍可靠航位推算撐住）
      llk = sm['liveLocationKalman']
      # 驗證方式對齊 sunnypilot 官方 osm_map_data.py (OsmMapData.update_location)
      llk_valid = (
        sm.updated.get('liveLocationKalman', False)
        and llk.inputsOK
        and llk.posenetOK
        and llk.sensorsOK
        and llk.status == log.LiveLocationKalman.Status.valid
        and llk.positionGeodetic.valid
      )

      if llk_valid:
        if llk.gpsOK:
          last_gps_ok_time = current_time

        dead_reckoning_time = current_time - last_gps_ok_time
        if dead_reckoning_time < MAX_DEAD_RECKONING_SEC:
          lat, lon, _ = llk.positionGeodetic.value
          bearing = math.degrees(llk.calibratedOrientationNED.value[2]) % 360
          is_gps_ready = True
          gps_source = 'liveLocationKalman' if llk.gpsOK else 'liveLocationKalman(推算中)'

      # 備案：liveLocationKalman 不可用，或飄移太久失去真實GPS校正，改用原始 GPS 頂上
      if not is_gps_ready and sm.updated.get('gpsLocationExternal', False):
        raw_gps = sm['gpsLocationExternal']
        # 兼容不同版本，嘗試抓取 accuracy 或是 horizontalAccuracy
        acc = getattr(raw_gps, 'accuracy', getattr(raw_gps, 'horizontalAccuracy', 0.0))
        if acc <= 0 or acc <= MAX_HORIZONTAL_ACCURACY:
          lat = raw_gps.latitude
          lon = raw_gps.longitude
          bearing = raw_gps.bearingDeg
          is_gps_ready = True
          gps_source = 'gpsLocationExternal(備案)'

    if is_gps_ready and (current_time - last_calc_time >= CALC_INTERVAL):
      last_calc_time = current_time
      my_direction = bearing_to_direction(bearing)

      raw_current_section = matcher.find_current_section(lat, lon, threshold_meters=300, bearing_deg=bearing)

      if raw_current_section is not None:
        stable_current_section = raw_current_section
        current_section_miss_count = 0
      else:
        current_section_miss_count += 1
        if current_section_miss_count >= CURRENT_SECTION_MISS_LIMIT:
          stable_current_section = None

      raw_ahead_section = None
      if stable_current_section is not None:
        raw_ahead_section = matcher.find_ahead_section(stable_current_section, target_distance_m=3000)

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
      traffic = msg.tdx.init('trafficStatus')

      curr_speed = client.cached_speeds.get(str(stable_current_section).strip(), -1) if stable_current_section else -1
      ahead_speed = client.cached_speeds.get(str(stable_ahead_section).strip(), -1) if stable_ahead_section else -1

      traffic.sectionId = str(stable_current_section) if stable_current_section else ""
      traffic.speed = int(curr_speed)
      traffic.nextSectionId = str(stable_ahead_section) if stable_ahead_section else ""
      traffic.nextSpeed = int(ahead_speed)

      display_speed = ahead_speed

      # 對齊 custom.capnp 中 TrafficStatus.Status 列舉
      if display_speed > 0:
        if display_speed >= 80:
          traffic.status = "freeFlow"
        elif display_speed >= 40:
          traffic.status = "heavyTraffic"
        else:
          traffic.status = "severeCongestion"
      else:
        traffic.status = "unknown"

      # ----------------------------------------------------
      # 處理事件
      # ----------------------------------------------------
      current_event = None
      ahead_event = None
      current_event_sid = ""
      ahead_event_sid = ""
      current_event_dist = 0.0
      ahead_event_dist = 0.0

      for evt in client.cached_events:
        evt_sid = evt.get('SectionID', '')
        evt_desc = evt['Description']
        evt_type = evt.get('EventType', '0')
        evt_lat = evt.get('lat')
        evt_lon = evt.get('lon')
        evt_dir = evt.get('direction', '')

        formatted_evt = f"{evt_type}|{evt_desc}"

        # 【距離 + 角度混合檢查】
        dist_to_evt = 0.0
        is_physically_ahead = True
        if evt_lat and evt_lon:
          dist_to_evt = _get_distance_meters(lon, lat, evt_lon, evt_lat)
          evt_bearing = _segment_bearing(lon, lat, evt_lon, evt_lat)
          angle_diff = _angle_diff(bearing, evt_bearing)

          # 1. 核心保留區：只要距離小於 300 公尺，無視角度強制顯示（避免接近時 GPS 亂跳導致閃爍）
          if dist_to_evt <= 300:
            is_physically_ahead = True
          # 2. 駛離判定：距離大於 300 公尺，且角度大於 90 度 (確實落在車身後方)，才正式取消
          elif angle_diff > 90:
            is_physically_ahead = False

        # 1. 目前路段比對
        if stable_current_section and evt_sid == str(stable_current_section).strip():
          if not is_physically_ahead:
            continue

          current_event = (current_event + "/" + formatted_evt) if current_event else formatted_evt
          current_event_sid = evt_sid or str(stable_current_section).strip()
          current_event_dist = dist_to_evt
          continue

        # 2. 前方路段比對
        is_ahead = False

        # 方法A：拓樸推導
        if stable_ahead_section and evt_sid == str(stable_ahead_section).strip():
          if is_physically_ahead:
            is_ahead = True

        # 方法B：空間雷達
        elif evt_lat and evt_lon and stable_current_section is not None:
          if dist_to_evt <= 3000:
            if _angle_diff(bearing, evt_bearing) <= 60:
              if evt_dir and (my_direction[0] in evt_dir):
                is_ahead = True
              elif not evt_dir:
                is_ahead = True

        if is_ahead:
          ahead_event = (ahead_event + "/" + formatted_evt) if ahead_event else formatted_evt
          ahead_event_sid = evt_sid or (str(stable_ahead_section).strip() if stable_ahead_section else "")
          ahead_event_dist = dist_to_evt

      cycle_state = int(current_time // 10) % 2
      event = msg.tdx.init('roadEvent')
      event.isActive = False
      event.description = ""
      event.sectionId = ""
      event.distance = 0.0

      if cycle_state == 0:
        if current_event:
          event.description = f"目前:{current_event}"
          event.sectionId = current_event_sid
          event.distance = float(current_event_dist)
          event.isActive = True
        elif ahead_event:
          event.description = f"前方:{ahead_event}"
          event.sectionId = ahead_event_sid
          event.distance = float(ahead_event_dist)
          event.isActive = True
      else:
        if ahead_event:
          event.description = f"前方:{ahead_event}"
          event.sectionId = ahead_event_sid
          event.distance = float(ahead_event_dist)
          event.isActive = True
        elif current_event:
          event.description = f"目前:{current_event}"
          event.sectionId = current_event_sid
          event.distance = float(current_event_dist)
          event.isActive = True

      pm.send('tdx', msg)

      # ----------------------------------------------------
      # 輸出 log
      # ----------------------------------------------------
      print(f"GPS: {gps_source} | 航向: {bearing:.1f}°({my_direction})")
      print(f"路段判定 -> 目前: {stable_current_section or '無'} | 前方: {stable_ahead_section or '無'}")

      if traffic.nextSpeed > 0:
        print(f" => 前方車速顯示: {traffic.nextSpeed} km/h (狀態: {traffic.status})")
      if event.isActive:
        print(f" => 事件警告 [{event.sectionId} / {event.distance:.0f}m]: {event.description}")
      print("-" * 40)


if __name__ == "__main__":
  main()
