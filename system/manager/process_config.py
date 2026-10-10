import os
import operator
import platform
import time

from cereal import car
from openpilot.common.params import Params
from openpilot.system.hardware import PC, TICI
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess
from openpilot.common.swaglog import cloudlog
from dragonpilot import jetlink_adapter

WEBCAM = os.getenv("USE_WEBCAM") is not None
LITE = os.getenv("LITE") is not None

TICI_DOS = "TICI_DOS" in os.environ

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox, block=True)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode")

def lat_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LateralManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def opview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("dp_dev_opview")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def beep(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("dp_dev_beep")

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started

def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def dashy(started: bool, params: Params, CP: car.CarParams) -> bool:
  return params.get_bool("dp_dev_dashy")

def comma_connect(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not params.get_bool("dp_dev_disable_connect")

def or_(*fns):
  return lambda *args: operator.or_(*(fn(*args) for fn in fns))

def and_(*fns):
  return lambda *args: operator.and_(*(fn(*args) for fn in fns))

class RestartingPythonProcess(PythonProcess):
  """dp - jetlink (ported from zoompilot): a PythonProcess that manager starts again
  after it dies; start() would leave a proc that has exited in place for good.
  For jetlinkd, which holds the USB gadget for as long as the link is on;
  jetlink's owner adopts what a dead one left and holds a crash loop back itself.

  One that dies within QUICK_DEATH of its start never got that far (an import
  error, a raise before the owner's loop, a second owner stepping aside for a
  live one), so the next start waits BACKOFF, doubling to BACKOFF_MAX, rather
  than forking manager twice a second for a whole drive. One that ran longer is
  started again on the next loop."""
  QUICK_DEATH = 10.0
  BACKOFF = 10.0
  BACKOFF_MAX = 300.0

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.started_at = 0.0
    self.backoff = 0.0
    self.next_start = 0.0

  def now(self) -> float:
    return time.monotonic()

  def start(self) -> None:
    now = self.now()
    if self.proc is not None and self.proc.exitcode is not None:
      if now - self.started_at < self.QUICK_DEATH:
        self.backoff = min(self.BACKOFF_MAX, 2 * self.backoff or self.BACKOFF)
        self.next_start = now + self.backoff
        cloudlog.warning(f"{self.name} died {now - self.started_at:.1f} s after it started, starting it again in {self.backoff:.0f} s")
      else:
        self.backoff = 0.0
      self.stop()  # reaps it, logs the exit code and clears proc
    if self.proc is None:
      if now < self.next_start:
        return
      self.started_at = now
    super().start()

procs = [
  DaemonProcess("manage_athenad", "system.athena.manage_athenad", "AthenadPid"),

  NativeProcess("loggerd", "system/loggerd", ["./loggerd"], logging),
  NativeProcess("encoderd", "system/loggerd", ["./encoderd"], only_onroad),
  NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], or_(notcar, opview)),
  PythonProcess("logmessaged", "system.logmessaged", always_run),

  NativeProcess("camerad", "system/camerad", ["./camerad"], driverview, enabled=not WEBCAM),
  PythonProcess("webcamerad", "tools.webcam.camerad", driverview, enabled=WEBCAM),
  PythonProcess("proclogd", "system.proclogd", only_onroad, enabled=platform.system() != "Darwin"),
  PythonProcess("journald", "system.journald", only_onroad, platform.system() != "Darwin"),
  PythonProcess("micd", "system.micd", iscar, enabled=not LITE),
  PythonProcess("timed", "system.timed", always_run, enabled=not PC),

  PythonProcess("modeld", "selfdrive.modeld.modeld", only_onroad),
  # dp - jetlink: always_run, jetlinkd holds the USB gadget open for as long as the link is enabled,
  # onroad included. A gadget whose owner exits leaves the bus: an unplug at every ignition edge
  RestartingPythonProcess(jetlink_adapter.OWNER, jetlink_adapter.__name__, and_(always_run, jetlink_adapter.should_run), enabled=not PC),
  PythonProcess("dmonitoringmodeld", "selfdrive.modeld.dmonitoringmodeld", driverview, enabled=(WEBCAM or not PC) and not LITE),

  PythonProcess("sensord", "system.sensord.sensord", only_onroad, enabled=not PC),
  PythonProcess("ui", "selfdrive.ui.ui", always_run, restart_if_crash=True),
  PythonProcess("soundd", "selfdrive.ui.soundd", driverview, enabled=not LITE),
  PythonProcess("beepd", "dragonpilot.selfdrive.ui.beepd", beep, enabled=LITE),
  PythonProcess("locationd", "selfdrive.locationd.locationd", only_onroad),
  NativeProcess("_pandad", "selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  PythonProcess("joystickd", "tools.joystick.joystickd", or_(joystick, notcar)),
  PythonProcess("selfdrived", "selfdrive.selfdrived.selfdrived", only_onroad),
  PythonProcess("card", "selfdrive.car.card", only_onroad),
  PythonProcess("deleter", "system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", driverview, enabled=(WEBCAM or not PC) and not LITE),
  PythonProcess("qcomgpsd", "system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  PythonProcess("pandad", "selfdrive.pandad.pandad" if not TICI_DOS else "selfdrive.pandad_tici.pandad", always_run),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", only_onroad),
  PythonProcess("lagd", "selfdrive.locationd.lagd", only_onroad),
  PythonProcess("ubloxd", "system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "selfdrive.controls.plannerd", not_long_maneuver),
  PythonProcess("maneuversd", "tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  PythonProcess("lateral_maneuversd", "tools.lateral_maneuvers.lateral_maneuversd", lat_maneuver),
  PythonProcess("radard", "selfdrive.controls.radard", only_onroad),
  PythonProcess("hardwared", "system.hardware.hardwared", always_run),
  PythonProcess("modem", "system.hardware.tici.modem", always_run, enabled=TICI and not LITE),
  PythonProcess("tombstoned", "system.tombstoned", always_run, enabled=not PC),
  PythonProcess("updated", "system.updated.updated", only_offroad, enabled=not PC),
  PythonProcess("uploader", "system.loggerd.uploader", and_(comma_connect, always_run)),
  PythonProcess("statsd", "system.statsd", always_run),
  PythonProcess("feedbackd", "selfdrive.ui.feedback.feedbackd", only_onroad),

  # debug procs
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "system.webrtc.webrtcd", or_(notcar, opview)),
  PythonProcess("webjoystick", "tools.bodyteleop.web", notcar),
  PythonProcess("joystick", "tools.joystick.joystick_control", and_(joystick, iscar)),

  # dashy
  PythonProcess("serverd", "dragonpilot.dashy.serverd", always_run),
  PythonProcess("dashyd", "dragonpilot.dashy.dashyd", and_(dashy, only_onroad)),
]

managed_processes = {p.name: p for p in procs}
