#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any, Dict

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

# =====================
# Global constants
# =====================
_LEAD_ACCEL_TAU = 1.5
SPEED, ACCEL = 0, 1
V_EGO_STATIONARY = 4.0
RADAR_TO_CAMERA = 1.52

# =====================
# Config (final confirmed)
# =====================
CROSSING_LAT_SPEED_TH = 1.5
CROSSING_MAX_DIST = 25.0
STATIC_OBSTACLE_DIST = 50.0
STATIC_BRAKE_LOW = 0.3
STATIC_BRAKE_HIGH = 0.8
SPEED_LIMIT = 65.0 / 3.6

LANE_WIDTH_STATIC = 0.6
LANE_WIDTH_MOVING = 1.0

# =====================
# Kalman
# =====================
class KalmanParams:
    def __init__(self, dt: float):
        assert 0.01 < dt < 0.2
        self.A = [[1.0, dt], [0.0, 1.0]]
        self.C = [1.0, 0.0]
        dts = [i * 0.01 for i in range(1, 21)]
        K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,
              0.21372394, 0.22761098, 0.24069424, 0.253096, 0.26491023,
              0.27621103, 0.28705801, 0.29750003, 0.30757767, 0.31732515,
              0.32677158, 0.33594201, 0.34485814, 0.35353899, 0.36200124]
        K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364,
              0.28342219, 0.28144091, 0.27958406, 0.27783249, 0.27617149,
              0.27458948, 0.27307714, 0.27162685, 0.27023228, 0.26888809,
              0.26758976, 0.26633338, 0.26511557, 0.26393339, 0.26278425]
        self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
    def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
        self.identifier = identifier
        self.cnt = 0
        self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
        self.kf = KF1D([[v_lead], [0.0]], kalman_params.A, kalman_params.C, kalman_params.K)
        self._prev_yRel = None

    def update(self, d_rel, y_rel, v_rel, v_lead):
        self.dRel, self.yRel, self.vRel = d_rel, y_rel, v_rel
        self.vLead = v_lead

        if self.cnt > 0:
            self.kf.update(self.vLead)

        self.vLeadK = float(self.kf.x[SPEED][0])
        self.aLeadK = float(self.kf.x[ACCEL][0])
        self.cnt += 1

    def get_RadarState(self):
        return {
            "dRel": self.dRel, "yRel": self.yRel, "vRel": self.vRel,
            "vLead": self.vLead, "vLeadK": self.vLeadK,
            "aLeadK": self.aLeadK, "aLeadTau": float(self.aLeadTau.x),
            "status": True, "fcw": False,
            "modelProb": 0.0, "radar": True,
            "radarTrackId": self.identifier,
        }

    def potential_low_speed_lead(self, v_ego):
        is_static = abs(self.vRel + v_ego) < 0.5

        lane_limit = LANE_WIDTH_STATIC if is_static else LANE_WIDTH_MOVING

        return (
            abs(self.yRel) < lane_limit and
            v_ego < 2.5 and
            1.0 < self.dRel < 12.0 and
            not is_static
        )


def laplacian_pdf(x, mu, b):
    return math.exp(-abs(x - mu) / max(b, 1e-4))


def match_vision_to_track(v_ego, lead, tracks):
    offset = lead.x[0] - RADAR_TO_CAMERA
    if not tracks:
        return None

    def score(t):
        return (
            laplacian_pdf(t.dRel, offset, lead.xStd[0]) *
            laplacian_pdf(t.yRel, -lead.y[0], lead.yStd[0]) *
            laplacian_pdf(t.vRel + v_ego, lead.v[0], lead.vStd[0])
        )

    track = max(tracks.values(), key=score)

    # ✅ 静止 + 偏离车道 → 直接拒绝
    if abs(track.vRel + v_ego) < 0.5 and abs(track.yRel) > LANE_WIDTH_STATIC:
        return None

    dist_ok = abs(track.dRel - offset) < max(offset * 0.25, 5.0)
    vel_ok = abs(track.vRel + v_ego - lead.v[0]) < 10 or (track.vRel + v_ego > 3)

    return track if dist_ok and vel_ok else None


def get_RadarState_from_vision(lead_msg, v_ego, model_v_ego):
    v_rel = lead_msg.v[0] - model_v_ego
    return {
        "dRel": lead_msg.x[0] - RADAR_TO_CAMERA,
        "yRel": -lead_msg.y[0],
        "vRel": v_rel,
        "vLead": v_ego + v_rel,
        "vLeadK": v_ego + v_rel,
        "aLeadK": lead_msg.a[0],
        "aLeadTau": 0.3,
        "status": True, "fcw": False,
        "modelProb": lead_msg.prob, "radar": False,
        "radarTrackId": -1,
    }


def get_lead(v_ego, ready, tracks, lead_msg, model_v_ego):
    lead_dict = {'status': False}

    if ready and lead_msg.prob > 0.5:
        track = match_vision_to_track(v_ego, lead_msg, tracks)
    else:
        track = None

    if track is not None:
        # ✅ 高速静止车绝不当 lead
        if v_ego > 20.0 and abs(track.vRel + v_ego) < 0.5:
            track = None
        else:
            lead_dict = track.get_RadarState()

    if track is None and ready and lead_msg.prob > 0.5:
        lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

    if ready:
        low_speed = [t for t in tracks.values() if t.potential_low_speed_lead(v_ego)]
        if low_speed:
            closest = min(low_speed, key=lambda t: t.dRel)
            if (not lead_dict['status']) or (closest.dRel < lead_dict['dRel']):
                lead_dict = closest.get_RadarState()

    return lead_dict


class RadarD:
    def __init__(self, delay=0.0):
        self.tracks: Dict[int, Track] = {}
        self.kalman_params = KalmanParams(DT_MDL)

        self.v_ego = 0.0
        self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL)) + 1)
        self.last_v_ego_frame = -1

        self.radar_state = None
        self.radar_state_valid = False
        self.ready = False

        self.crossing_slowdown_factor = 1.0
        self.static_slowdown_factor = 1.0

    def _detect_crossing_and_static(self):
        crossing = False
        static_obs = []

        for t in self.tracks.values():
            if t._prev_yRel is None:
                t._prev_yRel = t.yRel
                continue

            lat_speed = abs(t.yRel - t._prev_yRel) / DT_MDL
            t._prev_yRel = t.yRel

            if (
                lat_speed > CROSSING_LAT_SPEED_TH and
                t.dRel < CROSSING_MAX_DIST and
                self.v_ego < SPEED_LIMIT
            ):
                crossing = True

            if (
                abs(t.vRel + self.v_ego) < 0.5 and
                t.dRel < STATIC_OBSTACLE_DIST and
                abs(t.yRel) < LANE_WIDTH_STATIC
            ):
                static_obs.append(t)

        if crossing:
            self.crossing_slowdown_factor = max(0.4, min(1.0, t.dRel / 15.0))
        else:
            self.crossing_slowdown_factor = 1.0

        if static_obs:
            closest = min(static_obs, key=lambda t: t.dRel)
            brake_min = STATIC_BRAKE_HIGH if self.v_ego > SPEED_LIMIT else STATIC_BRAKE_LOW
            self.static_slowdown_factor = max(brake_min, 1.0 - closest.dRel / STATIC_OBSTACLE_DIST)
        else:
            self.static_slowdown_factor = 1.0

    def update(self, sm, rr):
        self.ready = sm.seen['modelV2']
        self.v_ego = sm['carState'].vEgo
        self.v_ego_hist.append(self.v_ego)

        ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}
        for tid in list(self.tracks.keys()):
            if tid not in ar_pts:
                del self.tracks[tid]

        for tid, rpt in ar_pts.items():
            v_lead = rpt[2] + self.v_ego_hist[0]
            if tid not in self.tracks:
                self.tracks[tid] = Track(tid, v_lead, self.kalman_params)
            self.tracks[tid].update(rpt[0], rpt[1], rpt[2], v_lead)

        self._detect_crossing_and_static()

        self.radar_state_valid = sm.all_checks()
        self.radar_state = log.RadarState.new_message()
        self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
        self.radar_state.carStateMonoTime = sm.logMonoTime['carState']
        self.radar_state.radarErrors = rr.errors

        leads_v3 = sm['modelV2'].leadsV3
        model_v_ego = sm['modelV2'].velocity.x[0] if len(sm['modelV2'].velocity.x) else self.v_ego

        if len(leads_v3) > 1:
            self.radar_state.leadOne = get_lead(
                self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego
            )
            self.radar_state.leadTwo = get_lead(
                self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego
            )

    def publish(self, pm):
        msg = messaging.new_message("radarState")
        msg.valid = self.radar_state_valid
        msg.radarState = self.radar_state
        pm.send("radarState", msg)


def main():
    config_realtime_process(5, Priority.CTRL_LOW)
    cloudlog.info("radard waiting for CarParams")
    CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
    cloudlog.info("radard got CarParams")

    sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
    pm = messaging.PubMaster(['radarState'])

    RD = RadarD(CP.radarDelay)

    while True:
        sm.update()
        RD.update(sm, sm['liveTracks'])
        RD.publish(pm)


if __name__ == "__main__":
    main()
