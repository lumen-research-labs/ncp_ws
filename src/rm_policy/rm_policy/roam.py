"""
roam.py — the pure half of ROAM mode: the limits and the hand-off clock.

What it does:
    ROAM makes policy_node draw its own next goal. This module holds the two
    parts that need no ROS: `RoamLimits` (read from `config/roam.yaml`, the
    authority for every number) and `RoamSequencer`, which owns the clocks
    and decides, once per control tick, whether the current goal is finished.

    A goal is finished for one of two reasons:
      ARRIVED  the node's arrival latch is set and `dwell_s` has passed since
               the arrival (or since roam was switched on, if later);
      TIMEOUT  the goal has been current for more than `timeout_s` without
               arrival (counted from its fix, or from roam's switch-on if
               later), whatever the reason. The goal switches anyway. A timed
               out goal is abandoned, never retried.
    On a hand-off the sequencer draws the next ROBOT-FRAME offset: distance
    uniform in [range_min_m, range_max_m], bearing uniform in
    [-bearing_cap_deg, +bearing_cap_deg], from its own seeded generator.

    The node calls `goal_fixed(now)` on EVERY goal fix (first pose, parameter
    set, hand-off), `set_on(on, now)` on the switch and `step(now, reached)`
    once per tick after its arrival latch. Switching roam on restarts the
    clocks, so an old goal gets a full timeout and a latched robot a full
    dwell from the switch-on instant: no spurious hand-off on enable. A
    repeated "on" moves no clock. A hand-off restarts the goal clock itself,
    so a caller that forgets `goal_fixed` cannot make it storm.

Subscribes:  nothing (pure module)
Publishes:   nothing (pure module)
Failure mode:
    `load_roam_limits` / `limits_from_dict` raise FileNotFoundError or
    ValueError naming the file and the key, so a bad config refuses the node
    at construction. `RoamSequencer` is total: it never raises and returns
    None when there is nothing to do.
"""

from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

import yaml

ARRIVED = "arrived"
TIMEOUT = "timeout"

# The keys of the `roam:` block, in the order the dataclass declares them.
# `RoamLimits` fields and this tuple are pinned equal by a test.
KEYS = ("range_min_m", "range_max_m", "bearing_cap_deg", "dwell_s", "timeout_s", "seed")


@dataclass(frozen=True)
class RoamLimits:
    range_min_m: float
    range_max_m: float
    bearing_cap_deg: float
    dwell_s: float
    timeout_s: float
    seed: int


@dataclass(frozen=True)
class Handoff:
    reason: str        # ARRIVED or TIMEOUT
    offset_x: float    # robot frame, metres, +x ahead
    offset_y: float    # robot frame, metres, +y left
    n: int             # ordinal of this hand-off, 1-based
    elapsed_s: float   # how long the finished goal was current


def check_limits(limits: RoamLimits) -> Optional[str]:
    """The reason the limits are incoherent, or None."""
    if limits.range_min_m <= 0.0:
        return f"roam.range_min_m must be > 0, got {limits.range_min_m}"
    if limits.range_min_m > limits.range_max_m:
        return (f"roam.range_min_m ({limits.range_min_m}) must not exceed "
                f"roam.range_max_m ({limits.range_max_m})")
    if not 0.0 <= limits.bearing_cap_deg <= 180.0:
        return f"roam.bearing_cap_deg must be within [0, 180], got {limits.bearing_cap_deg}"
    if limits.dwell_s < 0.0:
        return f"roam.dwell_s must be >= 0, got {limits.dwell_s}"
    if limits.timeout_s <= 0.0:
        return f"roam.timeout_s must be > 0, got {limits.timeout_s}"
    return None


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def limits_from_dict(block: Mapping, source: str) -> RoamLimits:
    """Build and check the limits from a `roam:` block; `source` names it in errors."""
    if not isinstance(block, Mapping):
        raise ValueError(f"{source}: the roam block is not a mapping: {block!r}")
    missing = [k for k in KEYS if k not in block]
    if missing:
        raise ValueError(f"{source}: roam block is missing: {', '.join(missing)}")
    for key in KEYS:
        if not _is_number(block[key]):
            raise ValueError(f"{source}: roam.{key} is not a number: {block[key]!r}")
    if not isinstance(block["seed"], int):
        raise ValueError(f"{source}: roam.seed must be an integer, got {block['seed']!r}")
    limits = RoamLimits(
        range_min_m=float(block["range_min_m"]),
        range_max_m=float(block["range_max_m"]),
        bearing_cap_deg=float(block["bearing_cap_deg"]),
        dwell_s=float(block["dwell_s"]),
        timeout_s=float(block["timeout_s"]),
        seed=int(block["seed"]),
    )
    reason = check_limits(limits)
    if reason is not None:
        raise ValueError(f"{source}: {reason}")
    return limits


def load_roam_limits(path: str) -> RoamLimits:
    """Read `config/roam.yaml`. Missing file, block or key is an error."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"roam config {path!r} does not exist; pass config/roam.yaml")
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    block = data.get("roam") if isinstance(data, Mapping) else None
    if block is None:
        raise ValueError(f"{path}: has no 'roam:' block")
    return limits_from_dict(block, source=path)


class RoamSequencer:
    """The hand-off clock and the next-goal draw. One per policy node."""

    def __init__(self, limits: RoamLimits) -> None:
        self._limits = limits
        self._rng = random.Random(limits.seed)
        self._on = False
        self._n = 0
        self._timeouts = 0
        self._last_reason: Optional[str] = None
        self._fixed_at: Optional[float] = None
        self._reached_at: Optional[float] = None
        self._enabled_at: Optional[float] = None

    @property
    def limits(self) -> RoamLimits:
        return self._limits

    @property
    def on(self) -> bool:
        return self._on

    @property
    def n(self) -> int:
        return self._n

    @property
    def timeouts(self) -> int:
        return self._timeouts

    @property
    def last_reason(self) -> Optional[str]:
        return self._last_reason

    def set_on(self, on: bool, now: float) -> bool:
        """Switch roam. Returns whether the state changed. The off->on edge
        records `now` as the clock reference; a repeated "on" moves nothing."""
        if on == self._on:
            return False
        self._on = on
        if on:
            self._enabled_at = now
        return True

    def goal_fixed(self, now: float) -> None:
        """Every goal fix, from any source: the goal clock restarts and the
        arrival edge clears."""
        self._fixed_at = now
        self._reached_at = None

    def draw(self) -> Tuple[float, float]:
        """The next robot-frame offset: distance in the range, bearing in the cone."""
        distance = self._rng.uniform(self._limits.range_min_m, self._limits.range_max_m)
        bearing = math.radians(
            self._rng.uniform(-self._limits.bearing_cap_deg, self._limits.bearing_cap_deg)
        )
        return distance * math.cos(bearing), distance * math.sin(bearing)

    def step(self, now: float, goal_reached: bool) -> Optional[Handoff]:
        """Once per control tick, after the node's arrival latch. A Handoff
        means: fix the goal at this offset now."""
        if self._fixed_at is None:
            return None
        if goal_reached:
            if self._reached_at is None:
                # Recorded even while off, so the dwell is right whenever
                # roam is switched on.
                self._reached_at = now
        else:
            self._reached_at = None
        if not self._on or self._enabled_at is None:
            return None
        if goal_reached:
            reference = max(self._reached_at, self._enabled_at)
            if now - reference < self._limits.dwell_s:
                return None
            reason = ARRIVED
        else:
            reference = max(self._fixed_at, self._enabled_at)
            if now - reference <= self._limits.timeout_s:
                return None
            reason = TIMEOUT
        elapsed = now - max(self._fixed_at, self._enabled_at)
        self._n += 1
        if reason == TIMEOUT:
            self._timeouts += 1
        self._last_reason = reason
        self._fixed_at = now
        self._reached_at = None
        offset_x, offset_y = self.draw()
        return Handoff(reason, offset_x, offset_y, self._n, elapsed)

    def status(self, now: float) -> dict:
        """The debug-stream block: what the node is doing, for the dashboard."""
        return {
            "on": self._on,
            "n": self._n,
            "timeouts": self._timeouts,
            "last_reason": self._last_reason,
            "goal_age_s": (
                round(now - self._fixed_at, 1) if self._fixed_at is not None else None
            ),
        }
