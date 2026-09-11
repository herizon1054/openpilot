from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_lane_centering",
    "type": "toggle_item",
    "title": lambda: tr("Lane Centering Assist (LCA)"),
    "description": lambda: tr("Experimentally bias the model's desired curvature toward the detected lane "
                              "center. Requires two confident lane lines and stays subject to the normal "
                              "curvature and jerk limits. Ported from StarPilot."),
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
  {
    "section": "Lateral",
    "key": "dp_lane_centering_e2e_authority",
    "type": "spin_button_item",
    "title": lambda: tr("Lane Centering E2E Override"),
    "description": lambda: tr("A ceiling on how much a confident end-to-end model path can override lane "
                              "centering. The actual amount also scales continuously with the model path's "
                              "confidence, vehicle speed, and how far it departs the lane center, so all "
                              "three need to be favorable to reach this ceiling. 100% = model can fully "
                              "override once those conditions are met; 0% = override is always disabled "
                              "regardless of the other conditions."),
    "flags": "PERSISTENT",
    "param_type": "INT",
    "default": "75",
    "min_val": 0,
    "max_val": 100,
    "step": 5,
    "suffix": lambda: tr("%"),
    "depends_on": "dp_lane_centering == 1",
  },
  {
    "section": "Lateral",
    "key": "dp_lane_centering_pause_on_signal",
    "type": "toggle_item",
    "title": lambda: tr("Pause Lane Centering on Turn Signal"),
    "description": lambda: tr("Fade the lane-centering correction out while a turn signal is active so it "
                              "does not fight a lane change or turn."),
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
    "depends_on": "dp_lane_centering == 1",
  },
]
