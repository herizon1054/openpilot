#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any, Tuple

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


_LEAD_ACCEL_TAU = 1.5
SPEED, ACCEL = 0, 1
V_EGO_STATIONARY = 4.

# ==========================================
# [自訂參數區] 
# ==========================================
STATIONARY_MAX_DIST = 90.0
STATIONARY_MIN_PROB = 0.4
BLIND_SPOT_PRIORITY_DIST = 23.0
BLIND_SPOT_HYSTERESIS_DIST = 25.0

# [防護機制] 低速大舵角抑制靜止車參數 (解決市區轉彎誤煞)
SUPPRESS_STATIONARY_SPEED = 30.0 / 3.6  # 嚴格限制在 30 km/h 以下 (轉換為 m/s)
# ==========================================

RADAR_TO_CENTER = 2.7
RADAR_TO_CAMERA = 1.52


class KalmanParams:
  def __init__(self, dt: float):
    assert dt > .01 and dt < .2
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    
    # 靜止目標信心度累加器 (防幽靈煞車核心)
    self.is_stopped_car_count = 0
    self.selected_count = 0

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float):
    self.dRel = d_rel
    self.yRel = y_rel
    self.vRel = v_rel
    self.vLead = v_lead
    self.measured = measured

    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1
    
    # 每幀預設扣 1 分，目標必須持續符合靜止條件才會加分
    self.is_stopped_car_count = max(0, self.is_stopped_car_count - 1)

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < BLIND_SPOT_HYSTERESIS_DIST)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    return f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track], path_x: list[float], path_y: list[float], current_prob_threshold: float, ignore_stationary: bool = False):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # 理智檢查: 距離與速度合理性
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  is_dynamic_target = dist_sane and vel_sane and (lead.prob > current_prob_threshold)
  
  model_x = track.dRel + RADAR_TO_CAMERA
  expected_yRel = -np.interp(model_x, path_x, path_y)
  
  # ==========================================
  # [防護機制] 30m 前方彎道預判與平滑極速收縮 (純幾何、無紅利)
  # ==========================================
  # 探測前方 30 公尺處的視覺軌跡橫向偏移
  curve_offset = abs(np.interp(30.0, path_x, path_y))
  
  # 橫向範圍 (防止抓到壓線車): 
  # [0.15, 0.25] 包含 15cm 的直路雜訊容許死區。
  # 一旦偏移突破 15cm，橫向寬容度會極速且平滑地從 1.0m 縮減至 0.4m
  dynamic_y_threshold = np.interp(curve_offset, [0.15, 0.25], [1.0, 0.4])
  y_sane_on_path = abs(track.yRel - expected_yRel) < dynamic_y_threshold
  
  # 縱向範圍 (防止掃到隔壁車道遠方物體): 
  # 採用相同 [0.15, 0.25] 區間，讓最遠偵測距離從 90m 平滑驟降到 50m
  dynamic_max_dist = np.interp(curve_offset, [0.15, 0.25], [STATIONARY_MAX_DIST, 50.0])
  # ==========================================
  
  v_absolute = track.vRel + v_ego
  is_physically_stationary = abs(v_absolute) < 2.0
  
  dynamic_stat_prob = np.interp(track.dRel, [30.0, 90.0], [0.5, 0.4])

  # 靜止目標必須通過所有嚴苛考驗 (包含上述的彎道動態範圍)
  is_stationary_target = (0.0 < track.dRel <= dynamic_max_dist) and is_physically_stationary and dist_sane and y_sane_on_path and (lead.prob > dynamic_stat_prob)

  # ==========================================
  # [防護機制] 低速大轉彎強制剔除靜止目標
  # ==========================================
  # 如果系統正處於市區轉彎抑制狀態，且目標絕對速度低於 2.0m/s，直接判斷為無效
  if ignore_stationary and is_physically_stationary:
      is_stationary_target = False
  # ==========================================

  is_valid_lead = is_dynamic_target or is_stationary_target

  # 如果完美符合靜止車條件，開始累積信心度分數 (+6)
  if is_valid_lead:
    track.is_stopped_car_count = min(track.is_stopped_car_count + 6, 50)

  best_track = None

  if is_dynamic_target:
    best_track = track # 動態車輛直接鎖定
  elif is_stationary_target and track.is_stopped_car_count >= 40: 
    best_track = track # 靜止車輛必須穩定追蹤達到 40 分 (約 0.4 秒連續不丟失) 才會鎖定

  for c in tracks.values():
    if best_track is not None and c is best_track:
      c.selected_count += 1
    else:
      c.selected_count = 0

  return best_track


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, path_x: list[float], path_y: list[float],
             low_speed_override: bool = True, is_locked: bool = False,
             current_prob_threshold: float = 0.5, ignore_stationary: bool = False) -> Tuple[dict[str, Any], bool]:
  
  gate_threshold = min(current_prob_threshold, STATIONARY_MIN_PROB)
  
  if len(tracks) > 0 and ready and lead_msg.prob > gate_threshold:
    best_valid_track = match_vision_to_track(v_ego, lead_msg, tracks, path_x, path_y, current_prob_threshold, ignore_stationary)
  else:
    best_valid_track = None

  fused_lead_dict = {'status': False}
  if best_valid_track is not None:
    fused_lead_dict = best_valid_track.get_RadarState(lead_msg.prob)
  elif ready and (lead_msg.prob > 0.5):
    fused_lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  lead_dict = fused_lead_dict  
  new_locked_state = is_locked 

  # ==========================================
  # [防護機制] 原廠低速盲區強迫煞車 (完全獨立)
  # ==========================================
  # 這裡不加入 not ignore_stationary，確保無論是否在轉彎，
  # 只要極近距離 (<25m) 有障礙物，系統依然能強制介入防止追撞。
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    
    if len(low_speed_tracks) > 0:
      closest_low_speed_track = min(low_speed_tracks, key=lambda c: c.dRel)
      blind_spot_dict = closest_low_speed_track.get_RadarState()
      
      if is_locked:
        if blind_spot_dict['dRel'] <= BLIND_SPOT_HYSTERESIS_DIST:
          lead_dict = blind_spot_dict 
          new_locked_state = True
        else:
          new_locked_state = False    
          
      else:
        if blind_spot_dict['dRel'] <= BLIND_SPOT_PRIORITY_DIST:
          lead_dict = blind_spot_dict 
          new_locked_state = True     
        else:
          if not fused_lead_dict['status'] or blind_spot_dict['dRel'] < fused_lead_dict.get('dRel', 1000.0):
            lead_dict = blind_spot_dict
            
    else:
      new_locked_state = False
  else:
    new_locked_state = False

  return lead_dict, new_locked_state


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False
    self.ready = False
    self.lead_one_locked = False 

    self.dynamic_prob_threshold = 0.5  
    self.low_prob_score = 0            

    # ==========================================
    # [新增變數] 防市區誤煞專用
    # ==========================================
    self.steering_angle = 0.0
    self.ignore_stationary_active = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.steering_angle = abs(sm['carState'].steeringAngleDeg) # 讀取方向盤絕對角度
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    # ==========================================
    # [防護機制] 遲滯邏輯 (Hysteresis) 過濾市區轉彎誤判
    # ==========================================
    if self.v_ego < SUPPRESS_STATIONARY_SPEED:  # 僅在車速低於 30 km/h 時啟動
        if self.steering_angle > 30.0:
            self.ignore_stationary_active = True   # 方向盤轉超過 30 度，啟動靜止車忽略
        elif self.steering_angle < 20.0:
            self.ignore_stationary_active = False  # 必須回正到 20 度內才恢復偵測，防反覆點頭
    else:
        self.ignore_stationary_active = False      # 車速 >= 30 km/h，強制全面恢復防護
    # ==========================================

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    for ids in ar_pts:
      rpt = ar_pts[ids]
      v_lead = rpt[2] + self.v_ego_hist[0]

      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].position.x) > 0:
      path_x = list(sm['modelV2'].position.x)
      path_y = list(sm['modelV2'].position.y)
    else:
      path_x = [0.0, 100.0]
      path_y = [0.0, 0.0]

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
      
    leads_v3 = sm['modelV2'].leadsV3

    if len(leads_v3) > 0:
      lead_prob = leads_v3[0].prob

      if 0.2 <= lead_prob < 0.5:
        self.low_prob_score = min(self.low_prob_score + 1, 120)
      elif lead_prob >= 0.5:
        self.low_prob_score = max(self.low_prob_score - 2, 0)
      else:
        self.low_prob_score = max(self.low_prob_score - 1, 0)

      if self.low_prob_score >= 100: 
        self.dynamic_prob_threshold = 0.45
      elif self.low_prob_score == 0: 
        self.dynamic_prob_threshold = 0.5

    if len(leads_v3) > 1:
      # 將轉彎忽略狀態 (ignore_stationary_active) 傳遞給主目標 leadOne
      self.radar_state.leadOne, self.lead_one_locked = get_lead(
          self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, path_x, path_y, 
          low_speed_override=True, is_locked=self.lead_one_locked,
          current_prob_threshold=self.dynamic_prob_threshold,
          ignore_stationary=self.ignore_stationary_active
      )
      
      # 副目標 leadTwo 通常不需要轉彎過濾
      self.radar_state.leadTwo, _ = get_lead(
          self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, path_x, path_y, 
          low_speed_override=False, is_locked=False,
          current_prob_threshold=0.5,
          ignore_stationary=False
      )

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None
    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()
    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)

if __name__ == "__main__":
  main()
