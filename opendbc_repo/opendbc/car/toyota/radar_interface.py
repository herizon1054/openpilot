#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import RadarData
from opendbc.car.toyota.values import DBC, TSS2_CAR
from opendbc.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(car_fingerprint):
  # 根據車型判斷雷達訊號的 ID 範圍
  if car_fingerprint in TSS2_CAR:
    RADAR_A_MSGS = list(range(0x180, 0x190))
    RADAR_B_MSGS = list(range(0x190, 0x1a0))
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)
  # 建立 CAN Parser，頻率設定為 20Hz
  messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))

  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.track_id = 0

    if CP.carFingerprint in TSS2_CAR:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)

        score = self.rcp.vl[ii+16]['SCORE']

        # -----------------------------------------------------------------
        # 【修正 2: 混合過濾邏輯 (Hybrid Logic)】
        # 目的：解決新版 DBC 精度過高導致的伸縮縫幽靈煞車
        # -----------------------------------------------------------------
        
        # 【當前設定】：輕度過濾 (0.05秒)
        # 設定為 1，代表需要連續偵測到 2 次才顯示。
        # 這是折衷方案，反應比 0.1秒快，但比 0秒穩定。
        min_valid_cnt = 1 

        # 【備註】：強力過濾 (0.10秒) 設定說明
        # 如果您發現 0.05秒 仍然會對伸縮縫偶發煞車，請將上方的數值改為 2
        # min_valid_cnt = 2  <-- 強力模式 (連續偵測 3 次)，過濾效果最強
        
        # 【跟車模式】：0 延遲
        # 判斷相對速度 (REL_SPEED)：
        # 如果 > -2 m/s (約 -7.2 km/h)，代表物體跟我們的速差很小，或者是正在遠離
        # 這通常意味著它是「正在行駛的前車」或「低速跟車」，此時取消所有過濾延遲
        if cpt['REL_SPEED'] > -2:
            min_valid_cnt = 0

        # 檢查是否顯示目標：
        # 1. 它是 VALID 的
        # 2. 或者：分數 > 50 且 距離 < 255 且 「持續次數超過我們設定的動態門檻」
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > min_valid_cnt):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = RadarData.RadarPoint()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          
          # 這裡的數值會由 CANParser 自動根據新 DBC 的 Scale (0.005) 轉換
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          # 釋放消失的目標 (Garbage Collection)
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
