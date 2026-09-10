from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_lane_centering",
    "type": "toggle_item",
    "title": lambda: tr("Lane Centering (Beta)"),
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
    "description": lambda: tr("How strongly a confident end-to-end model path can override lane centering "
                              "when it deliberately departs the lane center. 100% gives the model full "
                              "authority; 0% disables the override."),
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
    "default": "1",
    "depends_on": "dp_lane_centering == 1",
  },
]
