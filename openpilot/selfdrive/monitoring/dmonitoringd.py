#!/usr/bin/env python3
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Ratekeeper, DT_DMON
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.monitoring.policy import DriverMonitoring, MonitoringPolicy
from openpilot.sunnypilot.privacy import privacy_mode
from opendbc.car.structs import car


def wheeltouch_step(DM: DriverMonitoring, sm) -> None:
  """隱私模式：不讀車內鏡頭（driverStateV2），且停用方向盤觸碰監控。
  將 driver_engaged 固定設為 True，使 awareness 永遠保持 1.0，不觸發任何警告事件。"""
  car_speed = sm['carState'].vEgo
  enabled = sm['selfdriveState'].enabled or sm['carControl'].latActive
  wrong_gear = sm['carState'].gearShifter not in (car.CarState.GearShifter.drive, car.CarState.GearShifter.low)
  lowspeed = car_speed < DM.settings._ALERT_MIN_SPEED
  driver_engaged = True
  DM._update_events(driver_engaged=driver_engaged, op_engaged=enabled, lowspeed=lowspeed, wrong_gear=wrong_gear)


def dmonitoringd_wheeltouch_thread(params: Params, pm: messaging.PubMaster) -> None:
  cloudlog.warning("dmonitoringd: privacy mode, driver camera and wheel-touch monitoring disabled")
  # 沒有 poll 時 SubMaster 會假設以最快 service 的頻率（carState 100 Hz）更新，
  # 但這個迴圈是 20 Hz → 頻率檢查失敗 → all_checks() 為 False → driverMonitoringState.valid=False
  # → selfdrived「driverMonitoringState 流程間的溝通問題」。明確指定迴圈頻率 20 Hz。
  sm = messaging.SubMaster(['carState', 'selfdriveState', 'carControl'], frequency=1. / DT_DMON)
  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))
  DM._set_policy(MonitoringPolicy.wheeltouch)
  rk = Ratekeeper(1 / DT_DMON, print_delay_threshold=None)
  frame = 0
  while True:
    sm.update(0)
    valid = sm.all_checks()
    if valid:
      wheeltouch_step(DM, sm)
    pm.send('driverMonitoringState', DM.get_state_packet(valid=valid))

    frame += 1
    if frame % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
    rk.keep_time()


def dmonitoringd_thread():
  config_realtime_process([0, 1, 2, 3], 5)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  # 隱私模式在 dmonitoringd 啟動（每次發動）時決定，與 process_config 的 dmonitoringmodeld 鎖定一致
  if privacy_mode(params):
    dmonitoringd_wheeltouch_thread(params, pm)
    return
  sm = messaging.SubMaster(['driverStateV2', 'extrinsicsCalibration', 'carState', 'selfdriveState', 'modelV2',
                            'carControl'], poll='driverStateV2')

  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))
  demo_mode=False

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      # iterate when model has new output
      continue

    valid = sm.all_checks()
    if demo_mode and sm.valid['driverStateV2']:
      DM.run_step(sm, demo=True)
    elif valid:
      DM.run_step(sm, demo=demo_mode)

    # publish
    dat = DM.get_state_packet(valid=valid)
    pm.send('driverMonitoringState', dat)

    # load live always-on toggle
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and not demo_mode and
     DM.wheelpos_offsetter.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_offsetter.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
