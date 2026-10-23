"""
retreat.py — the pure half of ROAM's retreat: back straight out of a
stop-zone e-stop, then let roam draw a new goal.

What it does:
    When rm_safety's stop zone latches while roam is on, the robot is stuck:
    everything is zeroed and the latch holds until a human resets it with
    the obstacle gone. Since 2026-10-23 rm_safety passes one thing under the
    latch: a fresh, pure, straight reverse, capped to its `retreat_max_vx`,
    and it releases the latch on its own after N ticks of that reverse with
    a clear scan (see rm_safety/README.md, "Stop-zone retreat"). This
    module decides WHEN policy_node sends that reverse and when it stops:

      start    roam on, the node is the selected source (the status names
               its own command topic), the status is fresh and reads
               STOPPED / stop_zone, the node's last request was not itself a
               reverse (it drove forward into this), the current goal has
               not had a retreat yet, and the cooldown since the last one
               has passed. Otherwise the latch is seen but refused
               (`hold_reason` says why) and the node warns.
      drive    publish exactly (-retreat_vx, 0, 0) every tick, no network.
      success  the status leaves the latched pair {STOPPED/stop_zone,
               SLOW/stop_zone_retreat} for a driving state (OK, or SLOW
               with a zone reason), or the robot has moved `retreat_m` from
               where the retreat started. Then roam draws the next goal.
      timeout  `retreat_timeout_s` with the latch still held: stop, hold the
               current goal, warn (a lateral obstacle, dry_run, or wheels
               that did not move).
      abort    KILLED, scan or odom stale, command timeout, internal error,
               a stale or never-received status, roam switched off, the
               source switched away, or a pose jump (the node reports it).
               Zero, no hand-off, the current goal kept.

    Straight back only, no trail: under the uniform clamp the forward leg's
    curvature is preserved, so 0.3 m straight back departs from the driven
    path by a centimetre or two, and rm_safety passes nothing but a straight
    reverse under the latch anyway. The rear is unmonitored; this is a blind
    reverse the operator accepts for a static scene with KILL in hand.

Subscribes:  nothing (pure module)
Publishes:   nothing (pure module)
Failure mode:
    Total: `step` never raises. A status it cannot read (never received, or
    older than the bound) aborts an active retreat and never starts one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# SafetyStatus.msg state constants, mirrored so this module stays ROS-free;
# a test pins them against the message definition.
STATE_OK = 0
STATE_SLOW = 1
STATE_STOPPED = 2
STATE_KILLED = 3

REASON_LATCHED = "stop_zone"
REASON_RETREAT = "stop_zone_retreat"
# Reasons on which an active retreat aborts outright: the layer below has
# lost perception, proprioception, the command, or itself.
_ABORT_REASONS = ("scan_stale", "odom_stale", "command_timeout", "internal_error")

STARTED = "started"
RELEASED = "released"
DISTANCE = "distance"
TIMEOUT = "timeout"
ABORTED = "aborted"


@dataclass(frozen=True)
class RetreatLimits:
    retreat_vx: float          # m/s, the reverse speed requested
    retreat_m: float           # metres back from the start pose that count as done
    retreat_timeout_s: float   # seconds with the latch still held before giving up
    retreat_cooldown_s: float  # seconds between retreats


@dataclass(frozen=True)
class SafetyView:
    """The last /safety/status the node saw. `state` None: never received."""
    state: Optional[int]
    reason: str
    age_s: float
    command_topic: str


@dataclass(frozen=True)
class Step:
    vx: float              # what to publish this tick (0.0 when done)
    wz: float              # always 0.0
    done: bool
    reason: Optional[str]  # STARTED, None (driving), RELEASED, DISTANCE, TIMEOUT, "aborted:<why>"


def is_latched(view: SafetyView) -> bool:
    return (view.state, view.reason) in (
        (STATE_STOPPED, REASON_LATCHED), (STATE_SLOW, REASON_RETREAT),
    )


class RetreatController:
    """One per policy node. Clockless: every time comes in through `step`."""

    def __init__(self, limits: RetreatLimits, status_max_age_s: float) -> None:
        self._limits = limits
        self._max_age = status_max_age_s
        self._active = False
        self._start_xy: Tuple[float, float] = (0.0, 0.0)
        self._start_at = 0.0
        self._last_end: Optional[float] = None
        self._used_for_goal = False
        self._retreats = 0
        self._aborts = 0
        self._timeouts = 0
        self._last_reason: Optional[str] = None
        self._hold_reason: Optional[str] = None

    @property
    def limits(self) -> RetreatLimits:
        return self._limits

    @property
    def active(self) -> bool:
        return self._active

    @property
    def hold_reason(self) -> Optional[str]:
        """Why a seen latch was not answered with a retreat (None when it was,
        or when no latch is seen)."""
        return self._hold_reason

    def goal_fixed(self) -> None:
        """Every goal fix: the new goal gets one retreat of its own."""
        self._used_for_goal = False

    def abort(self, why: str, now: float) -> Optional[Step]:
        """The node reports a condition the status cannot carry (a pose jump)."""
        if not self._active:
            return None
        return self._end(now, f"{ABORTED}:{why}")

    def _end(self, now: float, reason: str) -> Step:
        self._active = False
        self._last_end = now
        self._last_reason = reason
        if reason.startswith(ABORTED):
            self._aborts += 1
        elif reason == TIMEOUT:
            self._timeouts += 1
        return Step(0.0, 0.0, True, reason)

    def step(self, now: float, view: SafetyView, pose_xy: Tuple[float, float],
             roam_on: bool, my_topic: str, last_cmd_vx: float) -> Optional[Step]:
        """Once per control tick, with a trusted pose. None: nothing to do."""
        usable = view.state is not None and view.age_s <= self._max_age
        latched = usable and is_latched(view)
        if self._active:
            if not roam_on:
                return self._end(now, f"{ABORTED}:roam_off")
            if not usable:
                return self._end(now, f"{ABORTED}:status_stale")
            if view.state == STATE_KILLED:
                return self._end(now, f"{ABORTED}:killed")
            if view.reason in _ABORT_REASONS:
                return self._end(now, f"{ABORTED}:{view.reason}")
            if view.command_topic != my_topic:
                return self._end(now, f"{ABORTED}:source_changed")
            if not latched:
                return self._end(now, RELEASED)
            if math.dist(pose_xy, self._start_xy) >= self._limits.retreat_m:
                return self._end(now, DISTANCE)
            if now - self._start_at >= self._limits.retreat_timeout_s:
                return self._end(now, TIMEOUT)
            return Step(-self._limits.retreat_vx, 0.0, False, None)

        self._hold_reason = None
        if not (roam_on and latched and view.reason == REASON_LATCHED):
            return None
        if view.command_topic != my_topic:
            return None                      # another source is driving; not ours to answer
        if last_cmd_vx < 0.0:
            self._hold_reason = "last request was already a reverse"
            return None
        if self._used_for_goal:
            self._hold_reason = "this goal already had its retreat"
            return None
        if (self._last_end is not None
                and now - self._last_end < self._limits.retreat_cooldown_s):
            self._hold_reason = (
                f"cooldown: {self._limits.retreat_cooldown_s - (now - self._last_end):.0f} s left"
            )
            return None
        self._active = True
        self._start_xy = (float(pose_xy[0]), float(pose_xy[1]))
        self._start_at = now
        self._used_for_goal = True
        self._retreats += 1
        self._last_reason = STARTED
        return Step(-self._limits.retreat_vx, 0.0, False, STARTED)

    def status(self) -> dict:
        return {
            "retreating": self._active,
            "retreats": self._retreats,
            "retreat_aborts": self._aborts,
            "retreat_timeouts": self._timeouts,
            "retreat_last": self._last_reason,
        }
