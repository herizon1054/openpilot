"""
Onroad events for jetlink, an accelerator that joins mid-drive.

Ported from zoompilot's sunnypilot/selfdrive/selfdrived/accelerator_events.py
(Copyright (c) 2026-, Zeph Leggett, MIT License).

It swaps in only while nothing is in control, so the driver is told when it is
ready and re-engages to use it. For a second after a swap nothing engages
while the large model builds its history and proves it keeps up (jetlink
hands back on a held frame in that second), and the "big model active" chime
at the end of it says the driver can. When it leaves, the small model drives
on and the driver is told so; nothing disengages.

Either way the switch costs modeld a frame or two, and a dropped camera frame is
an invalid pose from the model and a locationd soft disable on the next tick.
So a switch is `settling` for a second, during which selfdrived holds back
commIssue and the locationd checks.

dp divergences: every event is native (dp has no OnroadEventSP); the state
comes from modelExt (acceleratorState, bigModel) rather than modelDataV2SP and
modelV2.big; there is no native chestnut bigModelReady to remove.
"""
import cereal.messaging as messaging
from cereal import custom
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import Events, EventName
from dragonpilot import jetlink_adapter

AcceleratorState = custom.ModelExt.AcceleratorState

# how long each event is raised for, in selfdrived's ticks
OFFER_TICKS = round(3. / DT_CTRL)
HANDBACK_TICKS = round(5. / DT_CTRL)
SWITCHING_TICKS = round(1. / DT_CTRL)


class AcceleratorEvents:
  # An accelerator is optional: its daemon exiting costs the big model, never
  # engagement. Without this a dead link owner was processNotRunning, NO_ENTRY
  OPTIONAL_PROCESSES = frozenset({jetlink_adapter.OWNER})

  def __init__(self):
    self.offered = False
    self.big_model_running = False
    # ticks left of each event, and of the settling after a switch either way
    self.offer = self.handback = self.switching = self.settle = 0

  @property
  def settling(self) -> bool:
    """Within a second of the big model swapping in or handing back."""
    return self.settle > 0

  def update(self, sm: messaging.SubMaster, in_control: bool, events: Events) -> None:
    """`in_control`: openpilot is engaged or ALKA steers. The adapter's swap gate
    (jetlink_adapter.in_control) is shut then, and also while its inputs are
    late or invalid."""
    if not sm.seen['modelExt']:
      return
    status = sm['modelExt']
    big = status.bigModel

    # modelExt is in selfdrived's ignore lists, so its freshness is checked here
    fresh = sm.alive['modelExt'] and sm.valid['modelExt']
    if fresh and sm.seen['modelV2'] and sm.alive['modelV2']:
      if status.acceleratorState != AcceleratorState.ready or big:
        # ended by the swap as well as by the link going
        self.offered = False
        self.offer = 0
      elif in_control and not self.offered and self.handback == 0:
        # with nothing in control it swaps in at once. Once per readiness
        self.offered = True
        self.offer = OFFER_TICKS

    running_big = fresh and big and status.acceleratorState != AcceleratorState.none
    if running_big and not self.big_model_running:
      self.switching = SWITCHING_TICKS + 1   # the no-entry, then the chime
      self.settle = SWITCHING_TICKS + 1
    elif self.big_model_running and not running_big:
      self.switching = 0
      self.settle = SWITCHING_TICKS + 1
      if in_control:
        self.handback = HANDBACK_TICKS
    self.big_model_running = running_big
    if self.settle > 0:
      self.settle -= 1
    if not in_control:
      self.handback = 0

    if self.offer > 0:
      self.offer -= 1
      events.add(EventName.bigModelAvailable)
    if self.handback > 0:
      self.handback -= 1
      events.add(EventName.bigModelLinkLost)
    if self.switching > 0:
      self.switching -= 1
      if self.switching:
        events.add(EventName.bigModelLoading)
      else:
        events.add(EventName.bigModelReady)
