"""
jetlink adapter for dragonpilot (openpilot-dp0111pre).

Ported from zoompilot's openpilot/sunnypilot/jetlink_adapter/__init__.py
(Copyright (c) 2026-, Zeph Leggett, MIT License; see jetlink_repo/LICENSE).

jetlink (jetlink_repo, vendored) runs comma's large driving model on a
computer attached to the comma's USB-C port (Jetson / Linux PC / Mac over USB,
iPhone over the cable's network) and never imports openpilot. It declares what
it needs as jetlink.openpilot.interface.Openpilot; Adapter below implements it
over this fork. The hooks in manager, modeld, hardwared, selfdrived and the UI
call the functions at the bottom, which answer as if the link were off when
jetlink is not present or speaks another API.

manager imports this module to build the process list and runs it as
jetlinkd, the resident USB gadget owner (~10 MB). So the top level is the
standard library only; everything else is imported where it is used.

dp0111pre divergences from zoompilot (search "dp:" below):
  - no sunnypilot model manager: the big-model slot and catalog are dp's own
    params (JetlinkBigModel, JetlinkCatalog), filled from the same sunnypilot
    catalog the phone/Mac apps list (refresh_catalog). With no pick on the comma
    (the default) the link FOLLOWS THE FAR END: the model the phone or Jetson
    has loaded drives (follow_far_end), so the phone's own model menu is the menu.
  - no deviceState.chestnutPresent / chestnut helpers: chestnut_present() walks
    sysfs for the same USB ids zoompilot lists.
  - model root: dp's Paths has no model_root(); /data/media/0/models on AGNOS.
  - modeld is dp's two-JIT modeld (warp_enqueue + run_policy), which has no
    standalone stateless warp: make_warp() builds zoompilot's stateless warp
    graph from dp's own make_frame_prepare.
  - in_control(): MADS does not exist; dp's ALKA takes its place, matched to
    MADS one for one: dp's lkasOn is ACC main on every brand (opendbc carstate,
    safety toyota.h), as MADS is enabled by main and disabled by main off.
"""
from __future__ import annotations

import functools
import os
import sys
import threading
from collections import namedtuple
from pathlib import Path

JETLINK_REPO = Path(__file__).resolve().parents[2] / "jetlink_repo"
if str(JETLINK_REPO) not in sys.path:
  sys.path.insert(0, str(JETLINK_REPO))

# the version of jetlink.openpilot's API this adapter is written to; any other
# is treated as jetlink being absent, with the reason as the offroad alert
API = 2

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index: jetlink.openpilot.MODES,
# written out so the panels build the setting without a jetlink checkout
MODES = ('off', 'usb', 'ios')

# the params jetlink reads and writes, all declared in common/params_keys.h.
# dp: no model manager; the big-model slot ({ref, displayName}, unset = follow the
# far end) and the catalog ({bundles}) are dp's own params
_Keys = namedtuple('_Keys', 'link offroad progress spec pointers big_model catalog charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='AcceleratorProgress', spec='JetlinkSpec',
             pointers='JetlinkModelPointers', big_model='JetlinkBigModel', catalog='JetlinkCatalog',
             charge_phone='JetlinkChargePhone')

# the selector version jetlink's registry filters catalogs by (jetlink.registry.catalog.REQUIRED_SELECTOR_VERSION)
CATALOG_SELECTOR = 19

# what the catalog param holds before the first refresh: jetlink's default big model
# alone, so a comma that never went online still has a model to name and provision
_DEFAULT_CATALOG = {'bundles': [{'ref': 'bf3e3631b3f91d92a1020a5e0dd4298b93ff4244', 'display_name': 'Cinque Terre V3 Model',
                                 'short_name': 'CTV3', 'index': 0, 'is_big': True,
                                 'minimum_selector_version': str(CATALOG_SELECTOR)}]}

# comma's chestnut (USB GPU), running and in its ROM. The comma's USB-C port
# hosts one and is never held as a jetlink device beside it
CHESTNUT_IDS = frozenset({(0xADD1, 0x0001), (0x3801, 0x0001), (0x174C, 0x2464), (0x174C, 0x2463)})

# where the build puts the warp for each camera (SConscript) and modeld loads
# it from: in the fork's tree, never in jetlink_repo
WARP_DIR = Path(__file__).resolve().parent / 'models'

OWNER_LOG = Path('/data/log/jetlink-owner.log')

_AGNOS = os.path.isfile('/AGNOS')

# what in_control() reads; modeld subscribes to all of them
IN_CONTROL = ('carState', 'carControl', 'controlsStateExt', 'carStateExt')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT,
  else /data/params on a device and ~/.comma<OPENPILOT_PREFIX>/params
  elsewhere, then /<OPENPILOT_PREFIX, or d>. Environment only, never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.path.join(os.environ.get('HOME', ''), '.comma' + prefix, 'params')
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def _model_root() -> Path:
  """dp: no Paths.model_root(); the big partition, where sunnypilot keeps its models.
  jetlink keeps its ONNX downloads in <root>/jetlink."""
  if _AGNOS:
    return Path('/data/media/0/models')
  return Path(os.environ.get('HOME', '')) / ('.comma' + os.environ.get('OPENPILOT_PREFIX', '')) / 'media' / '0' / 'models'


def warp_path(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  """The warp for one geometry: the build's target and what modeld opens."""
  return WARP_DIR / f'warp_{cam_w}x{cam_h}_{model_w}x{model_h}_tinygrad.pkl'


def chestnut_present() -> bool:
  """dp: a sysfs walk for any CHESTNUT_IDS device (dp's helpers.usbgpu_present
  only knows the running board's id)."""
  for d in Path("/sys/bus/usb/devices").glob("*"):
    try:
      if (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16)) in CHESTNUT_IDS:
        return True
    except Exception:
      pass
  return False


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  from jetlink.openpilot.interface import Keys, OwnerConfig

  from openpilot.common.basedir import BASEDIR
  # the provisioning run starts in the checkout with the checkout on its path,
  # where the tree links jetlink_repo/jetlink in as jetlink
  return OwnerConfig(params_dir=_params_dir(), keys=Keys(**KEYS._asdict()), chestnut_ids=CHESTNUT_IDS,
                     adapter=__name__, cwd=Path(BASEDIR), env={'PYTHONPATH': os.pathsep.join((BASEDIR, str(JETLINK_REPO)))}, log_file=OWNER_LOG)


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process."""
  from jetlink.openpilot.owner import main as run_owner
  run_owner(owner_config())


def adapter() -> Adapter:
  """The adapter, for jetlink's entry points that run as their own process:
  the provisioning run and the warp build."""
  return Adapter()


def _modeld_module():
  """dp's modeld module. manager runs it as `selfdrive.modeld.modeld`; reuse
  that one inside modeld rather than importing a second copy under the
  `openpilot.` alias (it would re-run modeld's module level)."""
  mod = sys.modules.get('selfdrive.modeld.modeld') or sys.modules.get('openpilot.selfdrive.modeld.modeld')
  if mod is None:
    import importlib
    mod = importlib.import_module('openpilot.selfdrive.modeld.modeld')
  return mod


class Adapter:
  """jetlink.openpilot.interface.Openpilot over dragonpilot."""

  def __init__(self):
    from jetlink.openpilot.interface import Keys

    from openpilot.common.basedir import BASEDIR
    from openpilot.common.swaglog import cloudlog
    self.keys = Keys(**KEYS._asdict())
    self.log = cloudlog
    self.basedir = Path(BASEDIR)
    # one Params per store (a test or a bench runs under its own prefix)
    self._stores: dict[Path, object] = {}

  # -- params ---------------------------------------------------------------

  def params_dir(self) -> Path:
    return _params_dir()

  def _params(self):
    where = _params_dir()
    store = self._stores.get(where)
    if store is None:
      from openpilot.common.params import Params
      store = self._stores[where] = Params()
    return store

  def get(self, key: str):
    # read from hardwared's and the UI's threads: an unknown key must not take them down
    try:
      value = self._params().get(key)
    except Exception:
      value = None
    if key == KEYS.catalog and not (isinstance(value, dict) and value.get('bundles')):
      return _DEFAULT_CATALOG
    return value

  def follow_far_end(self) -> bool:
    """dp: no model picked on the comma: the far end's loaded model drives
    (jetlink.openpilot.link.open_link, provision.run)."""
    return follows_far_end(self.get(KEYS.big_model))

  def put(self, key: str, value, *, block: bool = False) -> None:
    self._params().put(key, value, block=block)

  def remove(self, key: str) -> None:
    self._params().remove(key)

  # -- the device -------------------------------------------------------------

  def chestnut_present(self) -> bool:
    return chestnut_present()

  def camera(self) -> tuple[int, int, int, int]:
    # the camera modeld runs on this device (dp's modeld/SConscript builds both)
    from openpilot.system.hardware import HARDWARE
    from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    camera = _os_fisheye if HARDWARE.get_device_type() == "mici" else _ar_ox_fisheye
    return camera.width, camera.height, *MEDMODEL_INPUT_SIZE

  def warp_path(self, cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
    return warp_path(cam_w, cam_h, model_w, model_h)

  def model_root(self) -> Path:
    return _model_root()

  # dp: the sunnypilot chestnut catalog's selector, as the phone apps list it
  catalog_selector = CATALOG_SELECTOR

  # -- modeld -----------------------------------------------------------------

  def model_face(self):
    """comma's large model's face, from dp's modeld: Parser, constants, smoothing
    and action function. The constants slot is dp's ModelConstants (dp has no
    modeld_v2). The action function is zoompilot's (modeld.get_action_from_model_jetlink),
    which dp's modeld uses for every frame the large model drove."""
    from jetlink.openpilot.interface import ModelFace

    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    modeld = _modeld_module()
    return ModelFace(parser=Parser, frame_size=lambda w, h: get_nv12_info(w, h)[3], desire_len=ModelConstants.DESIRE_LEN,
                     constants=ModelConstants, lat_smooth_seconds=modeld.LAT_SMOOTH_SECONDS,
                     long_smooth_seconds=modeld.LONG_SMOOTH_SECONDS, get_action_from_model=modeld.get_action_from_model_jetlink)

  def event(self, name: str, **fields) -> None:
    self.log.event(name, **fields)

  # -- the build --------------------------------------------------------------

  def make_warp(self, cam_w: int, cam_h: int, model_w: int, model_h: int):
    """The stateless warp jetlink runs on the comma: both camera frames warped
    and stacked, (2, 6, H/2, W/2) uint8, under the names call_warp passes.

    dp: dp's compile_modeld.make_warp() is the queueing warp_enqueue of its
    two-JIT modeld, so the graph is built here from dp's make_frame_prepare,
    as zoompilot's compile_modeld.make_warp() does it. compile_modeld first:
    it patches tinygrad's firmware fetch as it loads."""
    from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_frame_prepare
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    from tinygrad.device import Device
    from tinygrad.tensor import Tensor
    nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    frame_prepare = make_frame_prepare(nv12, model_w, model_h)

    def warp(tfm, big_tfm, frame, big_frame):
      tfm = tfm.to(Device.DEFAULT)
      big_tfm = big_tfm.to(Device.DEFAULT)
      frame = frame.to(Device.DEFAULT)
      big_frame = big_frame.to(Device.DEFAULT)
      Tensor.realize(tfm, big_tfm, frame, big_frame)
      warped_frame = frame_prepare(frame, tfm).unsqueeze(0)
      warped_big_frame = frame_prepare(big_frame, big_tfm).unsqueeze(0)
      return Tensor.cat(warped_frame, warped_big_frame)

    return warp, nv12.size


# -- dp: the model menu ---------------------------------------------------------
# The phone and Mac apps pick their model from sunnypilot's chestnut catalog and
# report, in every hello, the sha256 of the model they have loaded and the ones
# they have built (kept by jetlinkd in /dev/shm/jetlink/server.json). The comma
# fetches the same catalog, resolves each model's sha256 once, and so can name
# the phone's models and offer them.

FOLLOW = 'follow'
SERVER_RECORD = Path('/dev/shm/jetlink/server.json')


def follows_far_end(slot) -> bool:
  """The slot is unset, or set to follow the far end."""
  return not (isinstance(slot, dict) and isinstance(slot.get('ref'), str) and slot.get('ref') and slot.get('ref') != FOLLOW)


def far_end_models() -> tuple[str | None, list[str]]:
  """(loaded sha256 or None, built sha256s) from the far end's last hello."""
  import json
  try:
    heard = json.loads(SERVER_RECORD.read_text())
  except (OSError, ValueError):
    return None, []
  if not isinstance(heard, dict):
    return None, []
  loaded = heard.get('loaded') if isinstance(heard.get('loaded'), str) and heard.get('loaded') else None
  built = [s for s in (heard.get('cached_models') or []) if isinstance(s, str)]
  return loaded, built


def model_menu(params=None) -> list[dict]:
  """Every catalog model, newest first: {ref, name, sha256 or None, size or None,
  loaded (on the far end now), built (on the far end), downloaded (on the comma)}."""
  from openpilot.common.params import Params
  params = params or Params()
  catalog = params.get(KEYS.catalog)
  bundles = catalog.get('bundles') if isinstance(catalog, dict) and catalog.get('bundles') else _DEFAULT_CATALOG['bundles']
  pointers = params.get(KEYS.pointers)
  pointers = pointers if isinstance(pointers, dict) else {}
  loaded, built = far_end_models()
  root = _model_root() / 'jetlink'
  rows = []
  for b in sorted((b for b in bundles if isinstance(b, dict) and b.get('ref')), key=lambda b: int(b.get('index', 0)), reverse=True):
    p = pointers.get(b['ref']) or {}
    sha, size = p.get('oid'), p.get('size')
    try:
      downloaded = bool(sha) and (root / f"{sha[:16]}.onnx").stat().st_size == int(size)
    except (OSError, TypeError, ValueError):
      downloaded = False
    rows.append({'ref': b['ref'], 'name': str(b.get('display_name') or b['ref'][:10]), 'sha256': sha, 'size': size,
                 'loaded': bool(sha) and sha == loaded, 'built': bool(sha) and sha in built, 'downloaded': downloaded})
  return rows


def model_name(sha256: str | None, params=None) -> str | None:
  """The catalog's name for a sha256 the far end reported, else its first 16 characters."""
  if not sha256:
    return None
  for row in model_menu(params):
    if row['sha256'] == sha256:
      return row['name']
  return sha256[:16]


def pick_model(ref: str | None, params=None) -> None:
  """Pick a catalog model on the comma, or None to follow the far end."""
  from openpilot.common.params import Params
  params = params or Params()
  if not ref or ref == FOLLOW:
    params.remove(KEYS.big_model)
    return
  name = next((r['name'] for r in model_menu(params) if r['ref'] == ref), ref[:10])
  params.put(KEYS.big_model, {'ref': ref, 'displayName': name})


def refresh_catalog(params=None, resolve: bool = True) -> int:
  """Fetch sunnypilot's chestnut catalogs (the pinned one, every newer one and
  zoompilot's extras, as the phone apps do), keep its big models, and resolve
  each one's sha256 and size from comma's LFS pointer once. Network, seconds to
  a minute: run it off the UI thread. Returns the number of models listed."""
  from jetlink.registry.catalog import fetch_catalogs, REQUIRED_SELECTOR_VERSION
  from jetlink.registry.lfs import fetch_pointer
  from openpilot.common.params import Params
  params = params or Params()
  merged = fetch_catalogs()
  bundles = [b for b in merged.get('bundles', []) if isinstance(b, dict) and b.get('is_big')
             and str(b.get('minimum_selector_version')) == str(REQUIRED_SELECTOR_VERSION)]
  if not bundles:
    raise RuntimeError("the catalog lists no big model")
  params.put(KEYS.catalog, {'bundles': bundles})
  if resolve:
    pointers = params.get(KEYS.pointers)
    pointers = dict(pointers) if isinstance(pointers, dict) else {}
    for b in bundles:
      if b['ref'] in pointers:
        continue
      try:
        p = fetch_pointer(b['ref'], timeout=10.0)
        pointers[b['ref']] = {'oid': p.oid, 'size': p.size}
      except Exception as e:
        _log_failure(f"could not resolve {b['ref'][:10]}: {e}", None)
    params.put(KEYS.pointers, pointers, block=True)
  return len(bundles)


# -- what the hooks call ------------------------------------------------------

class _Absent:
  """jetlink's answers when it cannot run here: not present (why is None), or
  a package this build cannot use (why says so, as the offroad alert, to
  someone who turned the link on)."""

  def __init__(self, why: str | None):
    self.why = why

  def enabled(self) -> bool:
    return False

  def status(self):
    return None

  def reason(self) -> str | None:
    if self.why is None:
      return None
    try:
      on = 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
    except (OSError, ValueError):
      on = False
    return self.why if on else None

  def prepare(self) -> bool:
    return False

  def attach(self, small, cam_w: int, cam_h: int):
    return None

  def request_shutdown(self, reason: str = '') -> bool:
    return False

  def shutdown_pending(self) -> bool:
    return False

  def should_extend_catalog(self) -> bool:
    return False

  def extend_catalog(self, catalog: dict) -> dict:
    return catalog

  def model_state(self, ref: str) -> str | None:
    return None


_bound = None
_binding = threading.Lock()


def _api():
  """jetlink for this process, bound to the adapter on first use. One per
  process: prepare() and attach() have to reach the same one."""
  global _bound
  if _bound is None:
    with _binding:
      if _bound is None:
        _bound = _bind()
  return _bound


def _bind():
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return _Absent(None)   # an empty jetlink_repo, which Python takes for a namespace package
    import jetlink.openpilot as jl
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return _Absent(None)   # not in this tree: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("jetlink package too old for this build", e)
    return _unusable(f"jetlink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"jetlink failed to load: {type(e).__name__}: {e}", e)
  api = getattr(jl, 'API', None)
  if api != API:
    return _unusable(f"jetlink package API {api}, this build expects {API}")
  try:
    return jl.bind(Adapter())
  except Exception as e:
    return _unusable(f"jetlink failed to start: {type(e).__name__}: {e}", e)


def _unusable(why: str, error: Exception | None = None) -> _Absent:
  _log_failure(why, error)
  return _Absent(why)


# Hook -> the failure last logged for it, cleared by a call that works
_failed_hooks: dict[str, str] = {}


def _log_failure(what: str, error: Exception | None) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.error("jetlink: %s", what, exc_info=error)
  except Exception:
    pass


def _guarded(default):
  """Whatever jetlink does wrong turns the link off and is logged, and never
  takes manager, hardwared, modeld or the UI down."""
  def wrap(hook):
    @functools.wraps(hook)
    def call(*args, **kwargs):
      try:
        result = hook(*args, **kwargs)
      except Exception as e:
        error = f"{type(e).__name__}: {e}"
        if _failed_hooks.get(hook.__name__) != error:
          _failed_hooks[hook.__name__] = error
          _log_failure(f"{hook.__name__}() failed", e)
        return default(*args, **kwargs) if callable(default) else default
      _failed_hooks.pop(hook.__name__, None)
      return result
    return call
  return wrap


@_guarded(False)
def should_run(started: bool, params, CP) -> bool:
  """manager's rule for jetlinkd: the link is on and no chestnut is fitted.
  jetlinkd runs onroad too: a gadget whose owner exits leaves the bus."""
  return _api().enabled()


@_guarded(None)
def status():
  """One snapshot for the UI (jetlink.openpilot.Status), or None without jetlink."""
  return _api().status()


@_guarded(None)
def reason() -> str | None:
  """Why the link the user turned on cannot run: hardwared's offroad alert."""
  return _api().reason()


@_guarded(False)
def prepare() -> bool:
  """modeld, before config_realtime_process: will the link join this modeld?"""
  return _api().prepare()


_alka_enabled: bool | None = None


def _alka() -> bool:
  """dp: is ALKA turned on? Read once per process, as card reads dp_lat_alka once
  into CP.alternativeExperience."""
  global _alka_enabled
  if _alka_enabled is None:
    try:
      from openpilot.common.params import Params
      _alka_enabled = Params().get_bool("dp_lat_alka")
    except Exception:
      _alka_enabled = False
  return _alka_enabled


# dp (divergence from zoompilot): ALKA on with ACC main on counts as in control, as MADS does,
# except while the gear is P, N or R. There ALKA is paused and nothing steers, and controlsd keeps
# ALKA from starting for ALKA_HOLD_SECONDS after a swap, so shifting to D during the large model's
# proving second cannot hand it the wheel. Without this the large model, ready after a hand-back,
# waited out a whole stop in park with ACC main on (2026-10-10 drive log: 20 s in N/R/P, nothing
# steering). False restores zoompilot's gate exactly: ACC main off is then the only way in
SWAP_WHILE_PARKED = True
PARKED_GEARS = frozenset({'park', 'neutral', 'reverse'})


def parked(car_state) -> bool:
  """The gear lets the large model swap in with ALKA on and ACC main on (SWAP_WHILE_PARKED)."""
  return SWAP_WHILE_PARKED and str(car_state.gearShifter) in PARKED_GEARS


@_guarded(True)
def in_control(sm) -> bool:
  """modeld, before every frame, onto the model: is openpilot or ALKA in control?
  jetlink's large model swaps in only while it is not.

  dp: zoompilot reads carControl.enabled or carControlSP.mads.enabled, MADS
  counting while paused (gear, door, seatbelt, brake hold, park brake, stock
  LKAS off: it steers again on its own) and released by ACC main off or its
  button. dp's ALKA, one for one: openpilot enabled, ALKA steering
  (carControl.latActive / controlsStateExt.alkaActive), or ALKA on with
  carStateExt.lkasOn, which dp tracks as ACC main on every brand, so ALKA paused
  (P/N/R, door, seatbelt, uncalibrated) still counts and ACC main off releases
  it. ALKA has no button of its own: main off is its only off switch. A service
  late or invalid counts as in control. dp: ALKA paused by P/N/R does not count
  (SWAP_WHILE_PARKED)."""
  if not (sm.all_alive(IN_CONTROL) and sm.all_valid(IN_CONTROL)):
    return True
  if sm['carControl'].enabled or sm['carControl'].latActive or sm['controlsStateExt'].alkaActive:
    return True
  return bool(_alka() and sm['carStateExt'].lkasOn and not parked(sm['carState']))


# dp: how long ALKA may not start steering after the large model swaps in: its
# proving second (jetlink model_state.PROVING_FRAMES at 20 Hz), plus a margin.
# zoompilot's MADS, turned on then, waits paused through bigModelLoading
ALKA_HOLD_SECONDS = 1.2


@_guarded(None)
def attach(small, cam_w: int, cam_h: int):
  """modeld, once the camera is up and `small` is built: the model to run,
  `small` driving until the link has joined; None unless prepare() said yes."""
  return _api().attach(small, cam_w, cam_h)


@_guarded(False)
def request_shutdown(reason: str = '') -> bool:
  """hardwared, once, when the comma is about to power off for good: ask for
  the far end to go down with it."""
  return _api().request_shutdown(reason)


@_guarded(False)
def shutdown_pending() -> bool:
  """hardwared, every loop after request_shutdown(), until it puts DoShutdown."""
  return _api().shutdown_pending()


@_guarded(None)
def model_state(ref: str) -> str | None:
  return _api().model_state(ref)
