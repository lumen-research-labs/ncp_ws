"""
retreat.py — the pure half of ROAM's manoeuvres: back straight out of a
stop-zone e-stop, and turn in place to face a goal the policy cannot.

The TURN (added the same day, after the first pinned run): the free-space
draw's fallback puts a goal beside or behind the robot when nothing ahead
is free, and M4 cannot steer to a goal that is not ahead (it pushes forward
into whatever stopped it, stalls, and the next draw lands behind again).
So after every roam hand-off whose goal lies more than `turn_tolerance_deg`
off the nose, `TurnController` turns the robot in place at `turn_wz` until
the goal is within tolerance, then the policy gets a goal ahead. It turns
only when the nearest lidar return (outside the robot's own body) is at
least `turn_clear_m` away: the chassis corners sweep 0.23 m from the lidar.
A retreat starting mid-turn cancels it; the turn ends on tolerance, on a
timeout, or when the scan stops reading clear (the node then lets the
policy try, and a pinned retreat follows if it cannot).

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
               either STOPPED / stop_zone (latched) or PINNED: a drivable
               state (OK or SLOW, nothing stale) with the robot STOPPED
               for `pinned_after_s`, whatever stopped it (boxed in with no
               free goal, braked to a crawl at the slow zone's boundary, a
               turn refused, a policy that commands nothing). "Stopped" is
               `StoppedClock`: the pose has neither left a circle of
               `stopped_move_m` nor turned `stopped_yaw_rad` since the
               clock re-anchored, and the clock re-anchors whenever the
               robot is not drivable, so time spent KILLED never counts
               (Daniel, 2026-11-04: "pinned is any situation where it is
               stopped for a bit"). Also: the node's last
               request was not itself a reverse (it drove forward into
               this), the current goal has not had a retreat yet, and the
               cooldown since the last one has passed. Otherwise the stop is
               seen but refused (`hold_reason` says why) and the node warns.
      drive    publish exactly (-retreat_vx, 0, 0) every tick, no network.
      success  a latched retreat ends when the status leaves the latched set
               {STOPPED/stop_zone, SLOW/stop_zone_retreat}; a pinned one
               ends by distance alone, `retreat_m` back from where it
               started (`retreat_max_m` while a latch holds). Then roam
               turns toward the most open floor and draws the next goal.
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
TURNED = "turned"
NOT_CLEAR = "not_clear"
RELEASED = "released"
DISTANCE = "distance"
TIMEOUT = "timeout"
ABORTED = "aborted"


@dataclass(frozen=True)
class RetreatLimits:
    retreat_vx: float             # m/s, the reverse speed requested
    retreat_m: float              # metres back that count as done once the latch is off
    retreat_max_m: float          # metres back after which a still-held latch gives up
    retreat_timeout_s: float      # seconds still stopped before giving up
    retreat_cooldown_s: float     # seconds between PINNED retreats (a latch always retreats)
    pinned_after_s: float         # stopped (no translation, no rotation) this long = pinned
    stopped_move_m: float         # the pose leaving a circle this wide restarts the stopped clock
    stopped_yaw_rad: float        # ... or the yaw leaving a band this wide


@dataclass(frozen=True)
class SafetyView:
    """The last /safety/status the node saw. `state` None: never received."""
    state: Optional[int]
    reason: str
    age_s: float
    command_topic: str
    min_range: float = math.inf   # the wide-arc minimum rm_safety reports




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


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class StoppedClock:
    """Seconds the robot has been stopped: the pose has neither left a circle
    of `move_m` nor turned `yaw_rad` since the clock re-anchored. It
    re-anchors whenever the pose leaves that band and whenever the robot is
    not drivable (killed, a stale input, a stop that is not the latch), so
    time spent KILLED never counts and the clock starts at zero on ARM. A
    crawl of a centimetre a second stays inside the band: that is stopped."""

    def __init__(self, move_m: float, yaw_rad: float) -> None:
        self._move_m = float(move_m)
        self._yaw_rad = float(yaw_rad)
        self._anchor: Optional[Tuple[float, float, float, float]] = None  # x, y, yaw, t

    def update(self, now: float, x: float, y: float, yaw: float, drivable: bool) -> float:
        """Once per tick with a trusted pose. Returns the seconds stopped."""
        if not drivable:
            self._anchor = None          # the next drivable tick starts the clock
            return 0.0
        if self._anchor is None:
            self._anchor = (float(x), float(y), float(yaw), now)
            return 0.0
        ax, ay, ayaw, at = self._anchor
        if (math.dist((x, y), (ax, ay)) > self._move_m
                or abs(_wrap(yaw - ayaw)) > self._yaw_rad):
            self._anchor = (float(x), float(y), float(yaw), now)
            return 0.0
        return max(0.0, now - at)


class RetreatController:
    """One per policy node. Clockless: every time comes in through `step`."""

    def __init__(self, limits: RetreatLimits, status_max_age_s: float) -> None:
        self._limits = limits
        self._max_age = status_max_age_s
        self._active = False
        self._start_xy: Tuple[float, float] = (0.0, 0.0)
        self._start_at = 0.0
        self._last_end: Optional[float] = None
        self._retreats = 0
        self._aborts = 0
        self._timeouts = 0
        self._last_reason: Optional[str] = None
        self._hold_reason: Optional[str] = None
        self._start_cause: Optional[str] = None

    @property
    def limits(self) -> RetreatLimits:
        return self._limits

    @property
    def start_cause(self) -> Optional[str]:
        """'latched' or 'pinned' for the current or last retreat."""
        return self._start_cause

    @property
    def active(self) -> bool:
        return self._active

    @property
    def hold_reason(self) -> Optional[str]:
        """Why a seen latch was not answered with a retreat (None when it was,
        or when no latch is seen)."""
        return self._hold_reason

    def drivable(self, view: SafetyView) -> bool:
        """A fresh status in a state the robot can drive from: OK or SLOW
        (the latch's own retreat state included), nothing stale, not killed.
        The stopped clock counts only while this holds."""
        usable = view.state is not None and view.age_s <= self._max_age
        return (usable and view.state not in (STATE_KILLED, STATE_STOPPED)
                and view.reason not in _ABORT_REASONS)

    def goal_fixed(self) -> None:
        """Every goal fix. Nothing to reset since the recovery turn: the
        cooldown alone bounds repeated retreats (a retreat always ends in a
        turn away from what stopped the robot)."""

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
             roam_on: bool, my_topic: str, last_cmd_vx: float,
             stopped_s: float = 0.0) -> Optional[Step]:
        """Once per control tick, with a trusted pose. `stopped_s`: seconds the
        robot has been stopped (`StoppedClock`). None: nothing to do."""
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
            if self._start_cause == "latched" and not latched:
                return self._end(now, RELEASED)
            back = math.dist(pose_xy, self._start_xy)
            # Under a held LATCH the reverse must go on: rm_safety releases
            # only during a passed reverse, and a latched robot can do
            # nothing else. Distance ends a retreat only once the latch is
            # off (the pinned case), or at retreat_max_m as the bound.
            if back >= self._limits.retreat_max_m or (
                    back >= self._limits.retreat_m and not latched):
                return self._end(now, DISTANCE)
            if now - self._start_at >= self._limits.retreat_timeout_s:
                return self._end(now, TIMEOUT)
            return Step(-self._limits.retreat_vx, 0.0, False, None)

        self._hold_reason = None
        fresh_latch = latched and view.reason == REASON_LATCHED
        stuck = self.drivable(view) and stopped_s >= self._limits.pinned_after_s
        if not (roam_on and (fresh_latch or stuck)):
            return None
        if view.command_topic != my_topic:
            return None                      # another source is driving; not ours to answer
        if last_cmd_vx < 0.0:
            self._hold_reason = "last request was already a reverse"
            return None
        # The cooldown bounds the PINNED loop only. A fresh latch needs a
        # drive to happen again, and a latched robot can do nothing but
        # reverse, so waiting would only freeze it.
        if (not fresh_latch and self._last_end is not None
                and now - self._last_end < self._limits.retreat_cooldown_s):
            self._hold_reason = (
                f"cooldown: {self._limits.retreat_cooldown_s - (now - self._last_end):.0f} s left"
            )
            return None
        self._active = True
        self._start_xy = (float(pose_xy[0]), float(pose_xy[1]))
        self._start_at = now
        self._retreats += 1
        self._last_reason = STARTED
        self._start_cause = "latched" if fresh_latch else "pinned"
        return Step(-self._limits.retreat_vx, 0.0, False, STARTED)

    def status(self) -> dict:
        return {
            "retreating": self._active,
            "retreats": self._retreats,
            "retreat_aborts": self._aborts,
            "retreat_timeouts": self._timeouts,
            "retreat_last": self._last_reason,
        }


@dataclass(frozen=True)
class TurnLimits:
    turn_wz: float             # rad/s, above the measured 0.107 rad/s dead band
    turn_tolerance_deg: float  # done when the goal is within this of the nose
    turn_timeout_s: float      # give up turning after this
    turn_clear_m: float        # turn only with no return closer than this


class TurnController:
    """Turn in place toward a goal the policy cannot reach from here."""

    def __init__(self, limits: TurnLimits) -> None:
        self._limits = limits
        self._active = False
        self._start_at = 0.0
        self._direction = 0.0
        self._turns = 0
        self._timeouts = 0
        self._refused = 0
        self._last_reason: Optional[str] = None

    @property
    def limits(self) -> TurnLimits:
        return self._limits

    @property
    def active(self) -> bool:
        return self._active

    def needed(self, bearing_rad: float) -> bool:
        return abs(math.degrees(bearing_rad)) > self._limits.turn_tolerance_deg

    def start(self, now: float, direction: float = 0.0) -> None:
        """`direction` +1 turns left, -1 right, the whole way round if that is
        the clear way; 0 steers by the sign of the error each tick."""
        self._active = True
        self._start_at = now
        self._direction = direction
        self._turns += 1
        self._last_reason = STARTED

    def cancel(self, reason: str) -> None:
        if self._active:
            self._active = False
            self._last_reason = f"{ABORTED}:{reason}"

    def step(self, now: float, bearing_rad: float, clear: bool) -> Step:
        """Once per tick while active: the goal's bearing in the robot frame
        and whether the scan reads clear around the body."""
        lim = self._limits
        if not clear:
            self._active = False
            self._refused += 1
            self._last_reason = NOT_CLEAR
            return Step(0.0, 0.0, True, NOT_CLEAR)
        if not self.needed(bearing_rad):
            self._active = False
            self._last_reason = TURNED
            return Step(0.0, 0.0, True, TURNED)
        if now - self._start_at >= lim.turn_timeout_s:
            self._active = False
            self._timeouts += 1
            self._last_reason = TIMEOUT
            return Step(0.0, 0.0, True, TIMEOUT)
        if self._direction:
            # A fixed sense: keep turning that way until the remaining angle
            # in that sense is within tolerance (the long way round passes
            # through the far side of the error's sign).
            wz = math.copysign(lim.turn_wz, self._direction)
        else:
            wz = math.copysign(lim.turn_wz, bearing_rad)
        return Step(0.0, wz, False, None)

    def status(self) -> dict:
        return {
            "turning": self._active,
            "turns": self._turns,
            "turn_timeouts": self._timeouts,
            "turn_refused": self._refused,
            "turn_last": self._last_reason,
        }
