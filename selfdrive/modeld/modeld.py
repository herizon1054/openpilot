#!/usr/bin/env python3
import os
os.environ['GMMU'] = '0' # for usbgpu fast loading, noop for qcom
from tinygrad.tensor import Tensor
import time
import pickle
import numpy as np
import cereal.messaging as messaging
from cereal import car, log
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value, get_curvature_from_plan, MAX_LATERAL_ACCEL_NO_ROLL
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, WARP_INPUTS, POLICY_INPUTS
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState
from openpilot.common.file_chunker import read_file_chunked, get_manifest_path
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.helpers import usbgpu_present, modeld_pkl_path, get_tg_input_devices
from dragonpilot.selfdrive.controls.lib.road_edge_detector import RoadEdgeDetector

LITE = os.getenv("LITE") is not None

PROCESS_NAME = "selfdrive.modeld.modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3

# =====================================================================
# 【與 cp 的刻意分歧】dp 彎道曲率增益 (curve curvature gain)
#
# 背景：cd210（cp 0.11.x 原廠 policy，checkpoint a27b3122…/100）彎道規劃的路徑偏寬，
# 同一段路方向盤角度比其他模型小、轉得不夠。以 rlog 逐層確認：modeld 換算（反而 +6~14%）、
# LCA（平均 -0.3%）、VehicleModel、latcontrol_torque（1.5 m/s² 以下達成 97~99%）都不是原因，
# 曲率不足來自模型 plan 本身。
#
# 做法：只在彎道放大曲率要求，依「模型要求的側向加速度」連續加權（不是門檻切換）。
# 所有分界點都依車型自動計算，以該車停止點 E 的百分比表示：
#
#   E（停止點）= min(該車 EPS 實測上限 × 95%,  cp 側向加速度上限 3.0 × 60%)
#       該車 EPS 實測上限 = CP.maxLateralAccel（opendbc torque_data 的 MAX_LAT_ACCEL_MEASURED）
#       cp 上限 = drive_helpers.MAX_LATERAL_ACCEL_NO_ROLL（直接 import，cp 改值時自動跟著變）
#   |要求| <  E × 17%           → 增益 1.0（直線/車道維持，與 cp 完全相同）
#   E × 17% ~ E × 55%         → 1.0 線性爬升到 DP_LAT_CURVE_GAIN
#   E × 55% ~ E × 83%         → DP_LAT_CURVE_GAIN（全額）
#   E × 83% ~ E               → 漸進收回 1.0（接近 EPS 上限，避免更快觸發 steerSaturated）
#   >= E                      → 增益 1.0（不放大、不削減）
#   例：Corolla TSS2（上限 1.87）→ E 1.78：0.30 / 0.98 / 1.47 / 1.78 m/s²
#       Prius（上限 1.50）        → E 1.43：0.24 / 0.78 / 1.18 / 1.43 m/s²
#       Ioniq 5（上限 2.71）      → E 1.80：0.31 / 0.99 / 1.49 / 1.80 m/s²（被 cp 60% 封頂）
#   因為收回區寬度是 E 的 17%，只要 DP_LAT_CURVE_GAIN <= 1.17，任何車型的輸出都對輸入
#   連續且單調（不會在上限附近「卡住」或反向）。
#
# 唯一保留的絕對值：直線雜訊下限 DP_LAT_CURVE_GAIN_STRAIGHT_FLOOR = 0.2 m/s²。
#   直線上車道維持的小修正量取決於道路與模型，不取決於車的 EPS 強弱，所以不能跟車型縮放；
#   若某車算出的起點（E × 17%）低於此值，代表增益會開始放大直線小修正，該車直接停用。
#   等效於 E < 1.18 m/s² 停用（多數 Honda/部分 Subaru 等 EPS 很弱的車）。
#
# 停用條件（增益 = 1.0，行為與 cp 完全相同）：
#   - 非 torque 控制車（angle 控制，如 TSS2.5 LTA、多數 Hyundai/Ford/Tesla；PID 控制，如 Honda）
#   - maxLateralAccel 非有限值或明顯是佔位值
#   - 該車的 EPS 上限不是實測值：只有車型（含 substitute.toml 對應後的車型）出現在 opendbc
#     torque_data/params.toml（車隊實測）才啟用；出現在 override.toml 的是人工估計/新車預設值
#     （例如 Honda Accord 11G、CR-V 6G、Pilot 4G、Acura MDX/RDX/TLX 等），停止點依據不可靠，停用。
#     之後資料庫改成實測值時會自動啟用，不需改程式。
#   - 起點低於直線雜訊下限（見上）
# 增益套用在平滑之前、dp_lat_offset_cm 之前；車速 <= MIN_LAT_CONTROL_SPEED 時沿用
#   prev_action（已含增益），不重複放大。
# 發布到 modelV2.action.desiredCurvature 的是放大後的值，selfdrived 的 steerSaturated
#   判斷、controlsd、LCA 都以放大後的值為準，彼此一致。
#
# 調整：只需動 DP_LAT_CURVE_GAIN。仍轉得不夠 → 1.10；切內線/轉太多 → 1.05；1.0 = 完全回到 cp。
# =====================================================================
DP_LAT_CURVE_GAIN = 1.10
DP_LAT_CURVE_GAIN_EPS_RATIO = 0.95          # 停止點 = 該車 EPS 實測上限 × 95%
DP_LAT_CURVE_GAIN_CP_LIMIT_RATIO = 0.60     # 停止點不超過 cp 側向加速度上限 × 60%
DP_LAT_CURVE_GAIN_RAMP_START = 0.17         # 以停止點 E 的比例：開始放大
DP_LAT_CURVE_GAIN_RAMP_FULL = 0.55          # 以停止點 E 的比例：放大到全額
DP_LAT_CURVE_GAIN_TAPER_START = 0.83        # 以停止點 E 的比例：開始收回
DP_LAT_CURVE_GAIN_STRAIGHT_FLOOR = 0.2      # m/s²，直線雜訊下限（道路特性，不隨車型縮放）

# 由 dp_curve_gain_setup() 依 CarParams 設定；None = 停用
# 內容：(放大起點, 全額點, 收回起點, 停止點)，單位 m/s²
_dp_curve_gain_bp: tuple[float, float, float, float] | None = None


def dp_torque_params_measured(fingerprint: str) -> bool:
  """車型的 torque 參數（含 maxLateralAccel）是否來自 params.toml 的車隊實測值。
  判斷邏輯與 opendbc interfaces.get_torque_params() 相同：先經 substitute.toml 對應，
  對應後的車型在 params.toml → 實測；在 override.toml 或找不到 → 非實測。讀檔失敗一律視為非實測（停用）。"""
  try:
    import tomllib
    from opendbc.car.interfaces import TORQUE_PARAMS_PATH, TORQUE_SUBSTITUTE_PATH
    with open(TORQUE_SUBSTITUTE_PATH, 'rb') as f:
      sub = tomllib.load(f)
    with open(TORQUE_PARAMS_PATH, 'rb') as f:
      measured = tomllib.load(f)
    resolved = sub.get(fingerprint, fingerprint)
    return resolved != 'legend' and resolved in measured
  except Exception:
    cloudlog.exception("dp curve gain: failed to read torque_data, disabling")
    return False


def dp_curve_gain_setup(CP) -> tuple[float, float, float, float] | None:
  """依車型 EPS 能力算出四個分界點；回傳 None 代表此車停用彎道增益。"""
  global _dp_curve_gain_bp
  _dp_curve_gain_bp = None
  if DP_LAT_CURVE_GAIN == 1.0:
    return None
  if CP.steerControlType != car.CarParams.SteerControlType.torque or CP.lateralTuning.which() != 'torque':
    return None
  max_lat = float(CP.maxLateralAccel)
  if not np.isfinite(max_lat) or max_lat <= 0.0 or max_lat > 10.0:
    return None
  if not dp_torque_params_measured(str(CP.carFingerprint)):
    return None
  end = min(max_lat * DP_LAT_CURVE_GAIN_EPS_RATIO, MAX_LATERAL_ACCEL_NO_ROLL * DP_LAT_CURVE_GAIN_CP_LIMIT_RATIO)
  bp = (end * DP_LAT_CURVE_GAIN_RAMP_START, end * DP_LAT_CURVE_GAIN_RAMP_FULL,
        end * DP_LAT_CURVE_GAIN_TAPER_START, end)
  if bp[0] < DP_LAT_CURVE_GAIN_STRAIGHT_FLOOR:
    return None
  _dp_curve_gain_bp = bp
  return bp


def dp_apply_curve_gain(desired_curvature: float, v_ego: float) -> float:
  bp = _dp_curve_gain_bp
  if bp is None:
    return desired_curvature
  ramp_start, ramp_full, taper_start, end = bp
  v2 = max(v_ego, MIN_LAT_CONTROL_SPEED) ** 2
  lat_accel = abs(desired_curvature) * v2
  if lat_accel <= ramp_start or lat_accel >= end:
    return desired_curvature
  w_in = float(np.interp(lat_accel, [ramp_start, ramp_full], [0.0, 1.0]))
  # 接近 EPS 上限時把增益漸進收回 1.0（不用硬夾制，避免輸出在上限附近「卡住不跟模型變化」）
  w_out = float(np.interp(lat_accel, [taper_start, end], [1.0, 0.0]))
  gain = 1.0 + (DP_LAT_CURVE_GAIN - 1.0) * w_in * w_out
  return float(desired_curvature * gain)


def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float, dp_lat_offset_cm: int) -> log.ModelDataV2.Action:
    plan = model_output['plan'][0]
    desired_accel, should_stop = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                                     plan[:,Plan.ACCELERATION][:,0],
                                                     ModelConstants.T_IDXS,
                                                     action_t=long_action_t)
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)

    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
    # dp(M6): 固定曲率偏移（dp_lat_offset_cm），上一幀曲率先扣掉偏移再參與平滑／沿用，避免重複累加
    curvature_offset = 2.0 * (dp_lat_offset_cm / 100.0) / 900.0 if dp_lat_offset_cm != 0 else 0.0
    prev_curvature_base = prev_action.desiredCurvature - curvature_offset
    if v_ego > MIN_LAT_CONTROL_SPEED:
      desired_curvature = dp_apply_curve_gain(desired_curvature, v_ego)  # dp: 彎道曲率增益（見檔頭說明）
      desired_curvature = smooth_value(desired_curvature, prev_curvature_base, LAT_SMOOTH_SECONDS)
    else:
      desired_curvature = prev_curvature_base

    # Apply lateral offset (driving style adjustment)
    # dp(M6 修正): 偏移量只加一次。prev_action.desiredCurvature 已含上一幀的偏移，原本在
    # 車速 <= MIN_LAT_CONTROL_SPEED 時直接沿用它再 += 偏移，每幀（20 Hz）累加一次：10 cm 時
    # 等 60 秒紅燈累積到約 0.27 1/m。LAT_SMOOTH_SECONDS > 0 時平滑濾波也會把偏移放大成 offset/alpha。
    # 改為：上面先用「扣掉偏移後的上一幀曲率」平滑／沿用，這裡再加一次偏移。
    # dp_lat_offset_cm 只在 modeld 啟動時讀一次，整趟固定，所以扣回的值與上一幀加上的值相同。
    desired_curvature += curvature_offset

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                  desiredAcceleration=float(desired_accel),
                                  shouldStop=bool(should_stop))

class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


class ModelState:
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, cam_w: int, cam_h: int, usbgpu: bool):
    input_devices = get_tg_input_devices(PROCESS_NAME, usbgpu)
    self.WARP_DEV, self.QUEUE_DEV = input_devices['WARP_DEV'], input_devices['QUEUE_DEV']
    jits = pickle.loads(read_file_chunked(modeld_pkl_path(usbgpu)))
    vision_metadata = jits['metadata']['vision']
    self.vision_input_shapes =  vision_metadata['input_shapes']
    self.vision_input_names = list(self.vision_input_shapes.keys())
    self.vision_output_slices = vision_metadata['output_slices']

    policy_metadata = jits['metadata']['policy']
    self.policy_input_shapes =  policy_metadata['input_shapes']
    self.policy_output_slices = policy_metadata['output_slices']

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)

    self.frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    self.input_queues, self.npy = make_input_queues(self.vision_input_shapes, self.policy_input_shapes, self.frame_skip, device=self.QUEUE_DEV)
    self.full_frames : dict[str, Tensor] = {}
    self._blob_cache : dict[int, Tensor] = {}
    self.parser = Parser()
    self.frame_buf_params = {k: get_nv12_info(cam_w, cam_h) for k in ('img', 'big_img')}
    self.run_policy = jits['run_policy']
    self.warp_enqueue = jits[(cam_w,cam_h)]

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    parsed_model_outputs = {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}
    return parsed_model_outputs

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
                inputs: dict[str, np.ndarray], prepare_only: bool) -> dict[str, np.ndarray] | None:
    for key in bufs.keys():
      ptr = np.frombuffer(bufs[key].data, dtype=np.uint8).ctypes.data
      yuv_size = self.frame_buf_params[key][3]
      # There is a ringbuffer of imgs, just cache tensors pointing to all of them
      cache_key = (key, ptr)
      if cache_key not in self._blob_cache:
        self._blob_cache[cache_key] = Tensor.from_blob(ptr, (yuv_size,), dtype='uint8', device=self.WARP_DEV)
      self.full_frames[key] = self._blob_cache[cache_key]

    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['tfm'][:,:] = transforms['img'][:,:]
    self.npy['big_tfm'][:,:] = transforms['big_img'][:,:]

    img, big_img = self.warp_enqueue(**{k: self.input_queues[k] for k in WARP_INPUTS}, frame=self.full_frames['img'], big_frame=self.full_frames['big_img'])

    if prepare_only:
      return None

    vision_output, policy_output = self.run_policy(
      **{k: self.input_queues[k] for k in POLICY_INPUTS}, img=img, big_img=big_img
    )

    vision_output = vision_output.numpy().flatten()
    policy_output = policy_output.numpy().flatten()
    vision_outputs_dict = self.parser.parse_vision_outputs(self.slice_outputs(vision_output, self.vision_output_slices))
    policy_outputs_dict = self.parser.parse_policy_outputs(self.slice_outputs(policy_output, self.policy_output_slices))
    combined_outputs_dict = {**vision_outputs_dict, **policy_outputs_dict}

    if SEND_RAW_PRED:
      combined_outputs_dict['raw_pred'] = np.concatenate([vision_output.copy(), policy_output.copy()])
    return combined_outputs_dict


def main(demo=False):
  cloudlog.warning("modeld init")

  _present = usbgpu_present()
  _compiled = os.path.isfile(get_manifest_path(modeld_pkl_path(usbgpu=True)))
  USBGPU = _present and _compiled
  params = Params()
  params.put_bool("UsbGpuPresent", _present)
  params.put_bool("UsbGpuCompiled", _compiled)

  if not USBGPU:
    # USB GPU currently saturates a core so can't do this yet,
    # also need to move the aux USB interrupts for good timings
    config_realtime_process(7, 54)

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  st = time.monotonic()
  cloudlog.warning("loading model")
  model = ModelState(vipc_client_main.width, vipc_client_main.height, USBGPU)
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # messaging
  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry", "modelExt"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay"])

  publish_state = PublishState()
  params = Params()

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_RUN_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()


  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # dp: 彎道曲率增益依車型 EPS 能力自動計算分界點（見檔頭說明）
  dp_gain_bp = dp_curve_gain_setup(CP)
  if dp_gain_bp is None:
    cloudlog.warning(f"dp curve gain: disabled for {CP.carFingerprint}")
  else:
    cloudlog.warning(f"dp curve gain: {DP_LAT_CURVE_GAIN:.2f}x, bp {dp_gain_bp[0]:.2f}/{dp_gain_bp[1]:.2f}/{dp_gain_bp[2]:.2f}/{dp_gain_bp[3]:.2f} m/s2 "
                     f"(maxLateralAccel {CP.maxLateralAccel:.2f}) for {CP.carFingerprint}")

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  dp_lat_lca_speed = int(params.get("dp_lat_lca_speed"))
  dp_lat_lca_auto_sec = float(params.get("dp_lat_lca_auto_sec"))
  DH = DesireHelper(dp_lat_lca_speed=dp_lat_lca_speed, dp_lat_lca_auto_sec=dp_lat_lca_auto_sec)

  dp_dev_is_rhd = params.get_bool("dp_dev_is_rhd")
  RED = RoadEdgeDetector(params.get_bool("dp_lat_road_edge_detection"))

  dp_lat_offset_cm = int(params.get("dp_lat_offset_cm") or 0)

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                         extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = dp_dev_is_rhd if LITE else sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    lat_delay = sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    if sm.updated["liveCalibration"] and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
      model_transform_main = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics, True).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}
    inputs:dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
    }

    mt1 = time.perf_counter()
    model_output = model.run(bufs, transforms, inputs, prepare_only)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')
      model_ext_send = messaging.new_message('modelExt')

      frame_delay = DT_MDL # compensate for time passed since the frame was captured: current_time - timestamp_eof is 50ms on average
      action_delay = DT_MDL / 2 # middle of the interval between model output (current state) and next frame (expected state)
      action = get_action_from_model(model_output, prev_action, lat_delay + frame_delay + action_delay, long_delay + frame_delay + action_delay, v_ego, dp_lat_offset_cm)
      prev_action = action
      fill_model_msg(drivingdata_send, modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      RED.update(modelv2_send.modelV2.roadEdgeStds, modelv2_send.modelV2.laneLineProbs)
      model_ext_send.modelExt.leftEdgeDetected = RED.left_edge_detected
      model_ext_send.modelExt.rightEdgeDetected = RED.right_edge_detected
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, RED.left_edge_detected, RED.right_edge_detected,
                list(modelv2_send.modelV2.laneLineProbs))  # dp(L3): LTD 路口判斷用
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
      pm.send('modelExt', model_ext_send)
    last_vipc_frame_id = meta_main.frame_id


if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
