"""
roam.py — the pure half of ROAM mode: the limits, the free-space draw and
the hand-off clock.

What it does:
    ROAM makes policy_node draw its own next goal. This module holds the parts
    that need no ROS: `RoamLimits` (read from `config/roam.yaml`, the
    authority for every number), `strip_is_free` (does the lidar see the way
    to a candidate goal clear?) and `RoamSequencer`, which owns the clocks
    and decides, once per control tick, whether the current goal is finished.

    A goal is finished for one of three reasons:
      ARRIVED  the node's arrival latch is set and `dwell_s` has passed since
               the arrival (or since roam was switched on, if later);
      TIMEOUT  the goal has been current for more than `timeout_s` without
               arrival, whatever the reason;
      STALLED  the distance to the goal has not shrunk by `progress_min_m`
               within `progress_window_s` (rm_waypoint's progress watchdog):
               a robot pushing on a wall, held by rm_safety, or ignoring the
               goal is not making progress, so the goal switches anyway.
    A finished goal is never retried.

    The next goal is a ROBOT-FRAME offset: distance uniform in
    [range_min_m, range_max_m], bearing uniform in [-bearing_cap_deg,
    +bearing_cap_deg], from the sequencer's own seeded generator, and it must
    land in free space the lidar can see: the strip from the robot to the
    goal plus `clearance_m`, `corridor_half_width_m` wide, must hold no
    return. No-return rays are UNKNOWN, not free, so a candidate needs at
    least `min_valid_fraction` of the rays over its strip to be valid. Up to
    `draw_attempts` random candidates are tried inside the cone, then the
    shortest leg at `draw_attempts` bearings all round, nearest heading
    first; if none is free the sequencer HOLDS the current goal (`holding`,
    counted in `held`) and retries once a second, so a boxed-in robot waits
    instead of being sent through a wall.

    The node calls `goal_fixed(now)` on EVERY goal fix, `set_on(on, now)` on
    the switch and `step(now, reached, goal_dist, scan)` once per tick after
    its arrival latch. Switching roam on restarts the clocks, so an old goal
    gets a full timeout and a latched robot a full dwell from the switch-on
    instant. A repeated "on" moves no clock. A hand-off restarts the goal
    clock itself, so a caller that forgets `goal_fixed` cannot make it storm.

Subscribes:  nothing (pure module)
Publishes:   nothing (pure module)
Failure mode:
    `load_roam_limits` / `limits_from_dict` raise FileNotFoundError or
    ValueError naming the file and the key, so a bad config refuses the node
    at construction. `RoamSequencer` is total: it never raises and returns
    None when there is nothing to do. A scan it cannot read (empty, bad
    increment) counts as unknown, which refuses every candidate and holds.
"""

from __future__ import annotations

import math
import os
import random
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, Tuple

import yaml

ARRIVED = "arrived"
TIMEOUT = "timeout"
STALLED = "stalled"
NO_FREE_GOAL = "no_free_goal"
RETREATED = "retreated"

# Seconds between draws while holding (no free goal in view). A draw costs
# draw_attempts passes over the scan; once a second is plenty for a robot
# that is not moving.
HOLD_RETRY_S = 1.0

# The keys of the `roam:` block, in the order the dataclass declares them.
# `RoamLimits` fields and this tuple are pinned equal by a test.
KEYS = (
    "range_min_m", "range_max_m", "bearing_cap_deg", "dwell_s", "timeout_s", "seed",
    "corridor_half_width_m", "clearance_m", "min_valid_fraction", "draw_attempts",
    "progress_window_s", "progress_min_m", "self_radius_m",
    "retreat_vx", "retreat_m", "retreat_max_m", "retreat_timeout_s", "retreat_cooldown_s",
    "pinned_after_s", "stopped_move_m", "stopped_yaw_deg", "body_floor_m",
    "turn_wz", "turn_tolerance_deg", "turn_timeout_s", "turn_clear_m",
    "recovery_turn_min_deg", "recovery_turn_max_deg",
)
_INT_KEYS = ("seed", "draw_attempts")


@dataclass(frozen=True)
class RoamLimits:
    range_min_m: float
    range_max_m: float
    bearing_cap_deg: float
    dwell_s: float
    timeout_s: float
    seed: int
    corridor_half_width_m: float
    clearance_m: float
    min_valid_fraction: float
    draw_attempts: int
    progress_window_s: float
    progress_min_m: float
    self_radius_m: float
    # The retreat (rm_policy/retreat.py): the reverse speed requested under
    # rm_safety's stop-zone latch, how far back counts as done, how long
    # before giving up, and the gap between retreats.
    retreat_vx: float
    retreat_m: float
    retreat_max_m: float
    retreat_timeout_s: float
    retreat_cooldown_s: float
    pinned_after_s: float
    stopped_move_m: float
    stopped_yaw_deg: float
    body_floor_m: float
    # The turn in place toward a goal off the nose (rm_policy/retreat.py).
    turn_wz: float
    turn_tolerance_deg: float
    turn_timeout_s: float
    turn_clear_m: float
    # After a retreat the robot turns between these two angles, toward the
    # side with the deepest free strip, before it draws a goal ahead.
    recovery_turn_min_deg: float
    recovery_turn_max_deg: float


@dataclass(frozen=True)
class Scan:
    """One raw LaserScan, as the node holds it. Beam i is at
    `angle_min + i * angle_increment`, forward 0, +left (REP-103). Rays below
    `range_min` (the TG30's 0.0 no-return sentinel) or non-finite are unknown."""
    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float


@dataclass(frozen=True)
class Handoff:
    reason: str        # ARRIVED, TIMEOUT or STALLED
    offset_x: float    # robot frame, metres, +x ahead
    offset_y: float    # robot frame, metres, +y left
    n: int             # ordinal of this hand-off, 1-based
    elapsed_s: float   # how long the finished goal was current
    attempts: int      # candidates tried before a free one (1 = the first)


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
    if limits.corridor_half_width_m <= 0.0:
        return f"roam.corridor_half_width_m must be > 0, got {limits.corridor_half_width_m}"
    if limits.clearance_m < 0.0:
        return f"roam.clearance_m must be >= 0, got {limits.clearance_m}"
    if not 0.0 < limits.min_valid_fraction <= 1.0:
        return f"roam.min_valid_fraction must be within (0, 1], got {limits.min_valid_fraction}"
    if limits.draw_attempts < 1:
        return f"roam.draw_attempts must be >= 1, got {limits.draw_attempts}"
    if limits.progress_window_s <= 0.0:
        return f"roam.progress_window_s must be > 0, got {limits.progress_window_s}"
    if limits.progress_min_m < 0.0:
        return f"roam.progress_min_m must be >= 0, got {limits.progress_min_m}"
    if not 0.0 <= limits.self_radius_m < limits.range_min_m:
        return (f"roam.self_radius_m must be >= 0 and below roam.range_min_m "
                f"({limits.range_min_m}), got {limits.self_radius_m}")
    if limits.retreat_vx <= 0.0:
        return f"roam.retreat_vx must be > 0, got {limits.retreat_vx}"
    if limits.retreat_m <= 0.0:
        return f"roam.retreat_m must be > 0, got {limits.retreat_m}"
    if limits.retreat_max_m < limits.retreat_m:
        return (f"roam.retreat_max_m ({limits.retreat_max_m}) must be at least "
                f"roam.retreat_m ({limits.retreat_m})")
    if limits.retreat_timeout_s <= 0.0:
        return f"roam.retreat_timeout_s must be > 0, got {limits.retreat_timeout_s}"
    if limits.retreat_cooldown_s < 0.0:
        return f"roam.retreat_cooldown_s must be >= 0, got {limits.retreat_cooldown_s}"
    if limits.pinned_after_s <= limits.dwell_s:
        return (f"roam.pinned_after_s ({limits.pinned_after_s}) must exceed roam.dwell_s "
                f"({limits.dwell_s}): the dwell at a reached goal is a stop, not pinned")
    if limits.stopped_move_m <= 0.0:
        return f"roam.stopped_move_m must be > 0, got {limits.stopped_move_m}"
    if not 0.0 < limits.stopped_yaw_deg < 90.0:
        return f"roam.stopped_yaw_deg must be within (0, 90), got {limits.stopped_yaw_deg}"
    if not 0.0 <= limits.body_floor_m < limits.self_radius_m:
        return (f"roam.body_floor_m must be >= 0 and below roam.self_radius_m "
                f"({limits.self_radius_m}), got {limits.body_floor_m}")
    if limits.turn_wz <= 0.0:
        return f"roam.turn_wz must be > 0, got {limits.turn_wz}"
    if not 0.0 < limits.turn_tolerance_deg < 180.0:
        return f"roam.turn_tolerance_deg must be within (0, 180), got {limits.turn_tolerance_deg}"
    if limits.turn_timeout_s <= 0.0:
        return f"roam.turn_timeout_s must be > 0, got {limits.turn_timeout_s}"
    if limits.turn_clear_m < 0.0:
        return f"roam.turn_clear_m must be >= 0, got {limits.turn_clear_m}"
    if not 0.0 <= limits.recovery_turn_min_deg <= limits.recovery_turn_max_deg <= 180.0:
        return (f"roam.recovery_turn_min_deg ({limits.recovery_turn_min_deg}) and "
                f"recovery_turn_max_deg ({limits.recovery_turn_max_deg}) must satisfy "
                f"0 <= min <= max <= 180")
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
    for key in _INT_KEYS:
        if not isinstance(block[key], int):
            raise ValueError(f"{source}: roam.{key} must be an integer, got {block[key]!r}")
    limits = RoamLimits(**{
        key: (int(block[key]) if key in _INT_KEYS else float(block[key])) for key in KEYS
    })
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


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# The chassis shows in the scan only behind this bearing (the standoffs at
# +-105 deg, the rear corners at +-135 to 150 deg; tests/fixtures/
# tg30_roam_hold_scan.yaml). Ahead of it a return under self_radius_m is a
# real object, not the robot: the one at 0.17 m that a turn would sweep.
BODY_SECTOR_FROM_DEG = 95.0


def is_return(r: float, phi: float, scan: Scan, self_radius_m: float,
              body_floor_m: float = 0.0) -> bool:
    """A valid return from the world: finite, above the sensor floor, not
    under the body floor (nothing in the world is nearer the lidar than the
    chassis; the standoffs read 0.06-0.11 m, 2026-11-04), and not the
    robot's own body (a short return in the rear body sectors)."""
    if not (math.isfinite(r) and r > 0.0 and r >= scan.range_min):
        return False
    if r < body_floor_m:
        return False
    if r < self_radius_m and abs(math.degrees(phi)) >= BODY_SECTOR_FROM_DEG:
        return False
    return True


def nearest_return(scan: Scan, self_radius_m: float, body_floor_m: float = 0.0) -> float:
    """The closest return from the world, any bearing; inf when there is
    none (or the scan cannot be read)."""
    if (not scan.ranges or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0.0):
        return math.inf
    nearest = math.inf
    for i, r in enumerate(scan.ranges):
        phi = _wrap(scan.angle_min + i * scan.angle_increment)
        if is_return(r, phi, scan, self_radius_m, body_floor_m) and r < nearest:
            nearest = r
    return nearest


# How far the depth search looks: the TG30's reach. A strip with no return
# inside it over this distance is as open as the lidar can say.
DEPTH_REACH_M = 30.0


def free_depth(scan: Scan, bearing: float, limits: RoamLimits,
               sector_half_deg: float = 7.5) -> Optional[float]:
    """How far the strip along `bearing` (corridor_half_width_m wide) is
    clear of returns, metres, up to DEPTH_REACH_M; None when too few rays in
    the +-sector_half_deg sector are valid to say (the blind rear, heavy
    dropout). Body returns ignored."""
    if (not scan.ranges or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0.0 or not math.isfinite(scan.angle_min)):
        return None
    half_width = limits.corridor_half_width_m
    half_sector = math.radians(sector_half_deg)
    in_sector = valid_in_sector = 0
    depth = DEPTH_REACH_M
    for i, r in enumerate(scan.ranges):
        phi = _wrap(scan.angle_min + i * scan.angle_increment)
        delta = _wrap(phi - bearing)
        sectored = abs(delta) <= half_sector
        if sectored:
            in_sector += 1
        if not is_return(r, phi, scan, limits.self_radius_m, limits.body_floor_m):
            continue
        if sectored:
            valid_in_sector += 1
        along = r * math.cos(delta)
        if along >= 0.0 and abs(r * math.sin(delta)) <= half_width and along < depth:
            depth = along
    if in_sector == 0 or valid_in_sector < limits.min_valid_fraction * in_sector:
        return None
    return depth


# The X3's chassis corners relative to the lidar (metres, +x forward, +y
# left): the 0.30 x 0.20 m box sits ~4.3 cm behind the lidar, so the front
# corners are 0.146 m out at +-43 deg and the rear ones 0.217 m at +-153 deg
# (the rear pair is what the scan shows at 0.19-0.23 m, 135-150 deg). A turn
# in place sweeps them through a 0.22 m circle the 0.15 m stop zone never
# sees; on 2026-11-04 a turn hit something that way.
BODY_CORNERS = ((0.107, 0.10), (0.107, -0.10), (-0.193, 0.10), (-0.193, -0.10))
SWEEP_MARGIN_M = 0.05


def sweep_clear(scan: Scan, direction: float, angle_rad: float, limits: RoamLimits) -> bool:
    """Would a turn in place of `angle_rad` in `direction` (+1 left, -1
    right) sweep a chassis corner into a lidar return? True when no valid
    return lies within a corner's radius plus margin on the arc that corner
    crosses. The blind rear cone has no returns and cannot be checked."""
    if (not scan.ranges or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0.0 or angle_rad <= 0.0):
        return True
    corners = [(math.hypot(x, y), math.atan2(y, x)) for x, y in BODY_CORNERS]
    reach = max(r for r, _ in corners) + SWEEP_MARGIN_M
    for i, r in enumerate(scan.ranges):
        phi = _wrap(scan.angle_min + i * scan.angle_increment)
        if not is_return(r, phi, scan, limits.self_radius_m, limits.body_floor_m) or r > reach:
            continue
        for radius, start in corners:
            if r > radius + SWEEP_MARGIN_M:
                continue
            # The arc this corner crosses, measured from its start in the
            # turn direction.
            offset = (direction * (phi - start)) % (2.0 * math.pi)
            if offset <= angle_rad:
                return False
    return True


def turn_direction(scan: Scan, bearing_rad: float, limits: RoamLimits) -> Optional[float]:
    """Which way to turn to face `bearing_rad`: the short way when its sweep
    is clear, else the long way when that one is, else None (do not turn)."""
    short = 1.0 if bearing_rad >= 0.0 else -1.0
    short_angle = abs(bearing_rad)
    if sweep_clear(scan, short, short_angle, limits):
        return short
    if sweep_clear(scan, -short, 2.0 * math.pi - short_angle, limits):
        return -short
    return None


def recovery_bearing(scan: Scan, limits: RoamLimits, step_deg: float = 15.0) -> float:
    """The bearing (radians, +left) to turn to after a retreat: between
    recovery_turn_min_deg and recovery_turn_max_deg either side, whichever
    strip is deepest (the most open floor), the smaller turn on a tie. With
    nothing known, turn right round (180 deg, the way the robot came)."""
    best_bearing, best_depth = math.pi, -1.0
    lo, hi = limits.recovery_turn_min_deg, limits.recovery_turn_max_deg
    candidates = []
    deg = lo
    while deg <= hi + 1e-9:
        candidates.append(deg)
        deg += step_deg
    if candidates[-1] < hi - 1e-9:
        candidates.append(hi)
    for deg in candidates:
        for sign in ((1.0,) if deg in (0.0, 180.0) else (1.0, -1.0)):
            bearing = math.radians(sign * deg)
            depth = free_depth(scan, bearing, limits)
            if depth is None:
                continue
            if depth > best_depth + 1e-9 or (abs(depth - best_depth) <= 1e-9
                                             and abs(bearing) < abs(best_bearing)):
                best_bearing, best_depth = bearing, depth
    return best_bearing


def strip_is_free(scan: Scan, bearing: float, distance: float, limits: RoamLimits) -> Optional[bool]:
    """Is the strip from the robot to `distance + clearance_m` along `bearing`,
    `corridor_half_width_m` wide, clear of lidar returns?

    True: no return inside the strip and enough valid rays over it.
    False: a return lies inside the strip.
    None: unknown, too few valid rays over the strip (no-returns), or the
    scan cannot be read.

    Returns closer than `self_radius_m` in the rear body sectors (beyond
    +-BODY_SECTOR_FROM_DEG) are the robot's own body (the lidar's standoffs
    at 0.07 m, the chassis corners at 0.19-0.23 m on the TG30) and are
    unknown, never obstacles: a 0.07 m return lies inside the strip of every
    bearing within 90 degrees of it, so counting it blocked the whole circle
    on 2026-10-23 and held the robot at its first goal. Ahead, a short
    return is a real object.
    """
    if (not scan.ranges or not math.isfinite(scan.angle_increment)
            or scan.angle_increment <= 0.0 or not math.isfinite(scan.angle_min)):
        return None
    reach = distance + limits.clearance_m
    half_width = limits.corridor_half_width_m
    half_angle = math.atan2(half_width, reach)
    in_window = 0
    valid_in_window = 0
    for i, r in enumerate(scan.ranges):
        phi = _wrap(scan.angle_min + i * scan.angle_increment)
        delta = _wrap(phi - bearing)
        windowed = abs(delta) <= half_angle
        if windowed:
            in_window += 1
        if not is_return(r, phi, scan, limits.self_radius_m, limits.body_floor_m):
            continue
        if windowed:
            valid_in_window += 1
        along = r * math.cos(delta)
        lateral = r * math.sin(delta)
        if 0.0 <= along <= reach and abs(lateral) <= half_width:
            return False
    if in_window == 0 or valid_in_window < limits.min_valid_fraction * in_window:
        return None
    return True


class RoamSequencer:
    """The hand-off clocks and the next-goal draw. One per policy node."""

    def __init__(self, limits: RoamLimits) -> None:
        self._limits = limits
        self._rng = random.Random(limits.seed)
        self._on = False
        self._n = 0
        self._timeouts = 0
        self._stalls = 0
        self._held = 0
        self._holding = False
        self._last_reason: Optional[str] = None
        self._fixed_at: Optional[float] = None
        self._reached_at: Optional[float] = None
        self._enabled_at: Optional[float] = None
        self._best_dist: Optional[float] = None
        self._best_at: Optional[float] = None
        self._retry_at: Optional[float] = None

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
    def stalls(self) -> int:
        return self._stalls

    @property
    def held(self) -> int:
        return self._held

    @property
    def holding(self) -> bool:
        """A hand-off is due but no free goal is in view; the current goal is kept."""
        return self._holding

    @property
    def last_reason(self) -> Optional[str]:
        return self._last_reason

    def set_on(self, on: bool, now: float) -> bool:
        """Switch roam. Returns whether the state changed. The off->on edge
        records `now` as the clock reference; a repeated "on" moves nothing."""
        if on == self._on:
            return False
        self._on = on
        self._holding = False
        if on:
            self._enabled_at = now
            self._best_dist = None
            self._best_at = None
        return True

    def goal_fixed(self, now: float) -> None:
        """Every goal fix, from any source: the goal clock restarts, the
        arrival edge and the progress record clear."""
        self._fixed_at = now
        self._reached_at = None
        self._best_dist = None
        self._best_at = None
        self._holding = False
        self._retry_at = None

    def _candidate(self, distance: float, bearing_rad: float) -> Tuple[float, float]:
        return distance * math.cos(bearing_rad), distance * math.sin(bearing_rad)

    def draw(self, scan: Optional[Scan] = None) -> Optional[Tuple[float, float, int]]:
        """The next robot-frame offset `(x, y, attempts)`, in free space when a
        scan is given; None when no candidate is free (hold)."""
        lim = self._limits
        cap = math.radians(lim.bearing_cap_deg)
        attempts = 0
        for _ in range(lim.draw_attempts):
            attempts += 1
            distance = self._rng.uniform(lim.range_min_m, lim.range_max_m)
            bearing = self._rng.uniform(-cap, cap)
            if scan is None or strip_is_free(scan, bearing, distance, lim):
                return (*self._candidate(distance, bearing), attempts)
        # Nothing free at random inside the cone: the shortest leg, which
        # needs the least clear floor, at evenly spaced bearings ALL ROUND,
        # nearest heading first. A robot that has just arrived usually faces
        # whatever it drove up to, and a goal beside or behind it (the
        # policy's weak side) beats holding forever. The stall watchdog
        # re-draws from the new heading if the policy cannot turn to it.
        n = max(lim.draw_attempts, 2)
        bearings = sorted(
            (-math.pi + 2.0 * math.pi * (k + 0.5) / n for k in range(n)), key=abs,
        )
        for bearing in bearings:
            attempts += 1
            if strip_is_free(scan, bearing, lim.range_min_m, lim):
                return (*self._candidate(lim.range_min_m, bearing), attempts)
        return None

    def step(self, now: float, goal_reached: bool, goal_dist: Optional[float] = None,
             scan: Optional[Scan] = None) -> Optional[Handoff]:
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
        lim = self._limits
        if goal_reached:
            reference = max(self._reached_at, self._enabled_at)
            if now - reference < lim.dwell_s:
                return None
            reason = ARRIVED
        else:
            start = max(self._fixed_at, self._enabled_at)
            if goal_dist is not None and math.isfinite(goal_dist):
                if self._best_dist is None or goal_dist < self._best_dist - lim.progress_min_m:
                    self._best_dist = goal_dist
                    self._best_at = now
            # The progress watchdog needs a distance; without one (a caller
            # that passes none) only the timeout applies.
            if now - start > lim.timeout_s:
                reason = TIMEOUT
            elif (self._best_at is not None
                    and now - max(self._best_at, start) > lim.progress_window_s):
                reason = STALLED
            else:
                return None
        return self._handoff(now, reason, scan, respect_retry=True)

    def retreated(self, now: float, scan: Optional[Scan] = None) -> Optional[Handoff]:
        """A retreat out of a stop-zone e-stop succeeded: the next goal is
        drawn at once, from where the robot now stands, reason RETREATED.
        None when no goal was ever fixed or nothing free is in view (hold)."""
        if self._fixed_at is None or not self._on:
            return None
        return self._handoff(now, RETREATED, scan, respect_retry=False)

    def _handoff(self, now: float, reason: str, scan: Optional[Scan],
                 respect_retry: bool) -> Optional[Handoff]:
        if (respect_retry and self._holding and self._retry_at is not None
                and now < self._retry_at):
            return None
        drawn = self.draw(scan)
        if drawn is None:
            if not self._holding:
                self._held += 1
            self._holding = True
            self._last_reason = NO_FREE_GOAL
            self._retry_at = now + HOLD_RETRY_S
            return None
        offset_x, offset_y, attempts = drawn
        reference = self._fixed_at if self._enabled_at is None else max(self._fixed_at, self._enabled_at)
        elapsed = now - reference
        self._n += 1
        if reason == TIMEOUT:
            self._timeouts += 1
        elif reason == STALLED:
            self._stalls += 1
        self._last_reason = reason
        self._fixed_at = now
        self._reached_at = None
        self._best_dist = None
        self._best_at = None
        self._holding = False
        self._retry_at = None
        return Handoff(reason, offset_x, offset_y, self._n, elapsed, attempts)

    def no_progress_s(self, now: float) -> float:
        """Seconds since the goal distance last shrank by progress_min_m (or
        since the goal was fixed or roam switched on, whichever is later).
        0.0 with no goal, roam off, or the latch set."""
        if self._fixed_at is None or not self._on or self._enabled_at is None:
            return 0.0
        if self._reached_at is not None:
            return 0.0
        start = max(self._fixed_at, self._enabled_at)
        reference = start if self._best_at is None else max(self._best_at, start)
        return max(0.0, now - reference)

    def status(self, now: float) -> dict:
        """The debug-stream block: what the node is doing, for the dashboard."""
        return {
            "on": self._on,
            "n": self._n,
            "timeouts": self._timeouts,
            "stalls": self._stalls,
            "held": self._held,
            "holding": self._holding,
            "last_reason": self._last_reason,
            "goal_age_s": (
                round(now - self._fixed_at, 1) if self._fixed_at is not None else None
            ),
        }
