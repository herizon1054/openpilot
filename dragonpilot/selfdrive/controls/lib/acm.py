import time
import numpy as np
from cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  get_safe_obstacle_distance, get_stopped_equivalence_factor, get_T_FOLLOW
)

class ACM:
  def __init__(self):
    self.enabled = False
    self.active = False
    self.just_disabled = False
    self._active_prev = False

    # 状态变量
    self.personality = log.LongitudinalPersonality.standard
    self._dtsc_is_active = False
    self._is_e2e_mode = False

    # ---------- 可配置参数 ----------
    self.COASTING_DISABLE_SPEED_KPH = 75.0  # ✅ 滑行关闭速度阈值

    # 场景判断阈值
    self.DISTANCE_RATIO_VERY_FAR = 3.0
    self.DISTANCE_RATIO_FAR = 2.0
    self.DISTANCE_RATIO_SAFE = 1.5
    self.DISTANCE_RATIO_CLOSE = 1.2

    # 相对速度阈值 (m/s)
    self.REL_SPEED_FAST_APPROACH = 3.0
    self.REL_SPEED_SLOW_APPROACH = 1.5
    self.REL_SPEED_MATCHING = 0.5
    self.REL_SPEED_FALLING_BEHIND = -1.0

    # 策略参数
    self.ACCEL_BOOST_FACTOR = 1.3
    self.MAX_ACCEL_BOOST = 0.5
    self.MIN_COASTING_FACTOR = 0.3

    # 状态跟踪
    self.current_scenario = "NO_LEAD"
    self.current_action = {
      'should_coast': False,
      'coasting_factor': 0.0,
      'accel_boost': 0.0,
      'decel_adjust': 0.0,
      'reason': "初始化"
    }

  def calculate_safe_ratio(self, lead, v_ego, t_follow):
    if not lead or not lead.status:
      return float('inf')

    safe_distance = get_safe_obstacle_distance(v_ego, t_follow)
    if safe_distance < 0.1:
      return float('inf')

    lead_obstacle_dist = lead.dRel + get_stopped_equivalence_factor(lead.vLead)
    return lead_obstacle_dist / safe_distance

  def classify_scenario(self, lead, v_ego, a_ego, t_follow):
    if not lead or not lead.status:
      return "NO_LEAD", {}

    distance_ratio = self.calculate_safe_ratio(lead, v_ego, t_follow)
    relative_speed = v_ego - lead.vLead
    closing_speed = max(relative_speed, 0.1)
    ttc = lead.dRel / closing_speed

    scenario_info = {
      'distance_ratio': distance_ratio,
      'relative_speed': relative_speed,
      'ttc': ttc,
      'v_ego': v_ego,
      'v_lead': lead.vLead,
      'a_ego': a_ego,
      'a_lead': lead.aLeadK
    }

    # 距离分类
    if distance_ratio >= self.DISTANCE_RATIO_VERY_FAR:
      distance_category = "VERY_FAR"
    elif distance_ratio >= self.DISTANCE_RATIO_FAR:
      distance_category = "FAR"
    elif distance_ratio >= self.DISTANCE_RATIO_SAFE:
      distance_category = "SAFE"
    elif distance_ratio >= 1.0:
      distance_category = "CLOSE"
    else:
      distance_category = "TOO_CLOSE"

    # 相对速度分类
    if relative_speed >= self.REL_SPEED_FAST_APPROACH:
      speed_category = "FAST_APPROACH"
    elif relative_speed >= self.REL_SPEED_SLOW_APPROACH:
      speed_category = "SLOW_APPROACH"
    elif relative_speed >= -self.REL_SPEED_MATCHING:
      speed_category = "MATCHING"
    else:
      speed_category = "FALLING_BEHIND"

    # 场景决策
    if distance_category == "VERY_FAR":
      if speed_category in ("MATCHING", "FALLING_BEHIND"):
        scenario = "ACCELERATE_TO_CATCH_UP"
      elif speed_category == "SLOW_APPROACH":
        scenario = "MAINTAIN_SPEED"
      else:
        scenario = "GENTLE_DECEL"

    elif distance_category == "FAR":
      if speed_category in ("MATCHING", "SLOW_APPROACH"):
        scenario = "COASTING"
      elif speed_category == "FAST_APPROACH":
        scenario = "GENTLE_DECEL"
      else:
        scenario = "ACCELERATE_TO_CATCH_UP"

    elif distance_category == "SAFE":
      if speed_category == "MATCHING":
        scenario = "COASTING"
      elif speed_category == "SLOW_APPROACH":
        scenario = "GENTLE_COASTING"
      else:
        scenario = "NORMAL_FOLLOW"

    elif distance_category == "CLOSE":
      scenario = "NORMAL_FOLLOW"

    else:
      scenario = "SAFETY_OVERRIDE"

    return scenario, scenario_info

  def determine_action(self, scenario, scenario_info, a_ego):
    distance_ratio = scenario_info.get('distance_ratio', 0)
    relative_speed = scenario_info.get('relative_speed', 0)

    action = {
      'should_coast': False,
      'coasting_factor': 0.0,
      'accel_boost': 0.0,
      'decel_adjust': 0.0,
      'reason': scenario
    }

    if scenario == "ACCELERATE_TO_CATCH_UP":
      safe_far = max(self.DISTANCE_RATIO_VERY_FAR, 1e-3)
      boost_factor = min(
        self.ACCEL_BOOST_FACTOR * (distance_ratio / safe_far),
        1.0 + self.MAX_ACCEL_BOOST
      )
      action['accel_boost'] = boost_factor

    elif scenario == "MAINTAIN_SPEED":
      action['should_coast'] = True
      action['coasting_factor'] = 0.3

    elif scenario == "COASTING":
      distance_factor = min(1.0, max(0, (distance_ratio - self.DISTANCE_RATIO_SAFE) /
                           (self.DISTANCE_RATIO_FAR - self.DISTANCE_RATIO_SAFE)))
      speed_factor = 1.0 - min(1.0, abs(relative_speed) / self.REL_SPEED_SLOW_APPROACH)
      coasting_strength = distance_factor * speed_factor
      action['should_coast'] = True
      action['coasting_factor'] = max(self.MIN_COASTING_FACTOR, coasting_strength)

    elif scenario == "GENTLE_COASTING":
      action['should_coast'] = True
      action['coasting_factor'] = 0.5

    elif scenario == "GENTLE_DECEL":
      action['should_coast'] = False
      if relative_speed > 0:
        action['decel_adjust'] = -0.1

    elif scenario == "NORMAL_FOLLOW":
      action['should_coast'] = False

    elif scenario == "SAFETY_OVERRIDE":
      action['should_coast'] = False
      action['coasting_factor'] = 0.0
      action['accel_boost'] = 0.0
      action['decel_adjust'] = 0.0

    else:  # NO_LEAD
      action['should_coast'] = True
      action['coasting_factor'] = 0.8

    return action

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise, mode='acc',
                    personality=log.LongitudinalPersonality.standard,
                    dtsc_is_active=False, a_ego=0.0):

    self.personality = personality
    self._dtsc_is_active = dtsc_is_active
    self._is_e2e_mode = (mode == 'blended')

    if not self.enabled or len(cc.orientationNED) != 3:
      self.active = False
      return

    lead = rs.leadOne
    t_follow = get_T_FOLLOW(self.personality)

    scenario, scenario_info = self.classify_scenario(lead, v_ego, a_ego, t_follow)
    self.current_scenario = scenario

    action = self.determine_action(scenario, scenario_info, a_ego)
    self.current_action = action

    v_ego_kph = v_ego * 3.6
    should_activate = (
      action['should_coast'] and
      not user_ctrl_lon and
      not dtsc_is_active and
      action['coasting_factor'] > 0.1 and
      v_ego_kph <= self.COASTING_DISABLE_SPEED_KPH
    )

    self.active = should_activate
    self.just_disabled = self._active_prev and not self.active
    self._active_prev = self.active

  def apply_coasting(self, a_desired_trajectory, v_ego, lead, t_follow, a_ego=0.0):
    if not self.active or self.current_action['coasting_factor'] < 0.1:
      return a_desired_trajectory

    traj = np.copy(a_desired_trajectory)
    cf = self.current_action['coasting_factor']

    for i in range(len(traj)):
      if -1.0 < traj[i] < 0:
        traj[i] *= (1.0 - cf)
      elif 0 < traj[i] < 0.5:
        traj[i] *= (1.0 - cf * 0.5)

    return traj

  def apply_acceleration_boost(self, a_desired_trajectory):
    if self.current_action['accel_boost'] <= 1.0:
      return a_desired_trajectory

    traj = np.copy(a_desired_trajectory)
    bf = self.current_action['accel_boost']

    for i in range(len(traj)):
      if traj[i] > 0:
        traj[i] *= bf

    return traj

  def apply_deceleration_adjust(self, a_desired_trajectory):
    decel_adjust = self.current_action.get('decel_adjust', 0.0)
    if decel_adjust == 0.0:
      return a_desired_trajectory

    traj = np.copy(a_desired_trajectory)
    for i in range(len(traj)):
      if traj[i] < 0:
        traj[i] = min(traj[i], decel_adjust)

    return traj

  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=0.0,
                                  lead=None, t_follow=None, a_ego=0.0):

    if getattr(self, '_dtsc_is_active', False):
      return a_desired_trajectory

    if t_follow is None:
      t_follow = get_T_FOLLOW(self.personality)

    traj = a_desired_trajectory

    if self.current_scenario == "ACCELERATE_TO_CATCH_UP":
      traj = self.apply_acceleration_boost(traj)
    elif self.current_scenario == "GENTLE_DECEL":
      traj = self.apply_deceleration_adjust(traj)
    elif self.current_action['should_coast']:
      traj = self.apply_coasting(traj, v_ego, lead, t_follow, a_ego)

    return traj
