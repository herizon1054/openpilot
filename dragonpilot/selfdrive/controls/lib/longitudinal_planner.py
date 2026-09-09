"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX

from dragonpilot.selfdrive.controls.lib.dtsc import DTSC
from dragonpilot.selfdrive.controls.lib.accel_controller import AccelPersonalityController
from dragonpilot.selfdrive.controls.lib.traffic_stop import TrafficStopController
from opendbc.car.interfaces import ACCEL_MIN

LongitudinalPlanSource = custom.LongitudinalPlanDP.LongitudinalPlanSource


class LongitudinalPlannerDP:
  def __init__(self, CP: structs.CarParams, mpc):
    # self.acm = ACM()
    self.dtsc = DTSC() # 統一正名為 dtsc
    self.accel_controller = AccelPersonalityController() # 初始化 accel_controller
    self.traffic_stop = TrafficStopController() # 紅綠燈/停止標誌虛擬停止線
    self.source = LongitudinalPlanSource.cruise
    self.output_v_target = 0.0
    self.output_a_target = 0.0

  # accel_controller 限制最大加速度
  def get_accel_clip(self, v_ego: float, mode: str) -> list[float] | None:
    if mode == 'acc' and self.accel_controller.is_enabled():
      return [ACCEL_MIN, self.accel_controller.get_max_accel(v_ego)]
    return None

  # accel_controller 限制最小加速度
  def get_cruise_min_accel(self, v_ego: float) -> float | None:
    if self.accel_controller.is_enabled():
      return self.accel_controller.get_min_accel(v_ego)
    return None

  def update(self, sm: messaging.SubMaster) -> None:
    # 刷新各模組
    # self.acm.update(sm, self.CP)
    self.accel_controller.update(sm) # 每個 frame 刷新一次設定檔，已補上 sm 以讀取雷達與巡航車速資訊

    # 使用範例 Smart Cruise Control
    # self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

  # LongitudinalPlannerDP 核心邏輯：收集各來源目標並選出最保守的縱向控制目標
  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    # 缺少的參數可以 從sm 拿取
    CS = sm['carState']
    CC = sm['carControl']

    # [修改] 已經移除 self.dtsc.update(...) 呼叫，因為 DTSC 已純淨化，不再依賴外部動態刷新

    # 紅綠燈/停止標誌虛擬停止線：內部自行輪詢 Params 開關，關閉時 output_v_target
    # 維持 V_CRUISE_MAX，min() 選擇時自然不會勝出，故不需要額外的 dp_flags 閘門
    self.traffic_stop.update(sm['modelV2'], CS, sm['radarState'], v_ego, a_ego, v_cruise)

    # 控制參考來源
    # output_v_target = 目標巡航速度
    # output_a_target = 目標加速度
    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      # LongitudinalPlanSource.acm: (self.acm.output_v_target, self.acm.output_a_target),
      LongitudinalPlanSource.dtsc: (self.dtsc.output_v_target, self.dtsc.output_a_target),
      LongitudinalPlanSource.trafficStop: (self.traffic_stop.output_v_target, self.traffic_stop.output_a_target),
      # 使用範例 Smart Cruise Control
      # LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      # LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
    }

    # 選出最小的 target 來源
    self.source = min(targets, key=lambda k: targets[k][0])
    # 取出該 target 的速度與加速度
    self.output_v_target, self.output_a_target = targets[self.source]
    # 回傳結果
    return self.output_v_target, self.output_a_target

  # 發布 DP 專用 UI 資訊 (包含性格與目前控制來源)
  def publish_longitudinal_plan_dp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_dp_send = messaging.new_message('longitudinalPlanDP')

    plan_dp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanDP = plan_dp_send.longitudinalPlanDP
    longitudinalPlanDP.accelPersonality = self.accel_controller.get_accel_personality()
    longitudinalPlanDP.longitudinalPlanSource = self.source
    longitudinalPlanDP.vTarget = float(self.output_v_target)
    longitudinalPlanDP.aTarget = float(self.output_a_target)

    # 動態寫入 targets 列表邏輯
    source = LongitudinalPlanSource.schema.enumerants
    targets_list = longitudinalPlanDP.init('targets', len(source))

    for name, enum_value in source.items():
      controller = getattr(self, name, None)
      if controller is not None and hasattr(controller, 'write_to_msg'):
        idx = enum_value
        controller.write_to_msg(targets_list[idx])

    pm.send('longitudinalPlanDP', plan_dp_send)
