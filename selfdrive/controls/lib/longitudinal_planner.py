# [DP TWEAK] 起步保護邏輯：
      # 修改：將門檻提高到 30 km/h (約 8.33 m/s)
      # 這樣可以確保起步加速更連貫，不會在 15km/h 時突然軟掉
      is_starting_up = v_ego < 8.33 and self.a_desired_trajectory[0] > 0.1
      
      if not is_starting_up:
        self.a_desired_trajectory = self.acm.update_a_desired_trajectory(
            self.a_desired_trajectory,
            v_ego=v_ego,
            lead=sm['radarState'].leadOne
        )
