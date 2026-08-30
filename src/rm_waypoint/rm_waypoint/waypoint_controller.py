"""
waypoint_controller.py — the go-to-waypoint control law and its give-up
conditions. Pure, ROS-free, unit-testable.

What it does:
    Given where the robot is, where the waypoint is, and how long the run has
    been going, returns the velocity command to publish and the state to
    report. That is the whole module: no ROS types, no clock, no I/O. The node
    supplies every input and performs every effect.

    The law is deliberately the simplest thing that works on THIS robot:

        far from heading   -> turn in place
        facing the target  -> drive, steering proportionally
        within tolerance   -> stop, ARRIVED

    with a proportional gain on each axis and a floor under the command
    magnitude. The floor is the part that is not textbook and is not optional:
    this robot has a MEASURED rotational dead-band — roughly the first
    0.107 rad/s of a commanded spin is lost to mecanum roller scrub before the
    chassis turns at all (2026-08-29 session 3) — and front-right needs a real
    push to break loose at all. A tapering proportional law therefore does not
    converge here, it ASYMPTOTES INTO THE DEAD-BAND: the command shrinks, the
    robot stops moving, the error stops shrinking, and the controller sits
    commanding 0.04 rad/s forever while reporting that it is nearly there.
    Below the floor this module commands exactly zero instead. Either move
    properly or do not pretend to be moving.

    No integral term, also deliberately. With `pose_source:=map` the pose is
    corrected by SLAM, which means it STEPS at every loop closure. An
    integrator would wind up on that step and then drive the windup back out
    as a real command. A P law forgets the step in one tick, which is the
    correct response to a pose that can legitimately jump — the same reasoning
    that makes policy_node reset its recurrent state there.

Subscribes to:  (nothing — pure module; the node supplies samples)
Publishes:      (nothing — pure module; the node publishes the result)

Failure mode:
    Fail-closed and total. Every path returns a command; there is no input for
    which this module raises, and no input for which it returns "keep doing
    what you were doing". A non-finite pose, an expired timeout, a stalled
    range: each returns ZERO velocity and a terminal state carrying the
    reason. The caller cannot accidentally hold the last command, because the
    last command is never what is returned.

    Note what this module does NOT do: it has no idea what an obstacle is. It
    does not need one. rm_safety owns the arcs and the stop zone, and when
    they fire the robot stops moving and the range stops falling — which the
    progress watchdog here sees as "no progress" and gives up on. Obstacle
    avoidance living in exactly one place, one layer down, is the arrangement
    this whole workspace is built around; a second opinion about obstacles up
    here could only ever disagree with the layer that actually has authority.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# --- state machine ---------------------------------------------------------
# Mirrors the constants in rm_interfaces/msg/WaypointStatus. Kept as plain ints
# so this module imports nothing from ROS; the node maps them across, and a
# test pins the two sets equal so they cannot drift apart.
IDLE = 0
TURNING = 1
DRIVING = 2
ARRIVED = 3
ABORTED = 4

_TERMINAL = (IDLE, ARRIVED, ABORTED)


@dataclass(frozen=True)
class Limits:
    """Every constant the law uses. Sourced from config/waypoint.yaml.

    Defaults match that file, but the node ALWAYS passes explicit values —
    these exist so a test can construct one in a line, not so the config can
    be omitted. A silently-defaulted limit is the bug class that cost this
    project several hardware runs (`closed_loop.guards` shadowing the module
    defaults, CLAUDE.md 2026-08-29 s2), so `limits_from_dict` treats a missing
    key as an error rather than filling it in from here.
    """

    max_vx: float = 0.15
    max_wz: float = 0.6
    min_vx: float = 0.05
    min_wz: float = 0.35
    kp_lin: float = 0.5
    kp_yaw: float = 1.2
    turn_in_place_rad: float = 0.6
    arrive_tol_m: float = 0.15
    max_range_m: float = 6.0
    timeout_s: float = 90.0
    progress_window_s: float = 6.0
    progress_min_m: float = 0.05


@dataclass(frozen=True)
class Command:
    """One tick's output: what to publish and what to report."""

    vx: float
    wz: float
    state: int
    reason: str
    range_m: float
    heading_error_rad: float

    @property
    def is_terminal(self) -> bool:
        """True when the run is over and the node must stop publishing.

        Terminal states always carry zero velocity, so a caller that ignored
        this and kept publishing would still be commanding a stop — the flag
        is about releasing /teleop/cmd_vel back to the gamepad, not about
        safety.
        """
        return self.state in _TERMINAL


def _wrap(angle: float) -> float:
    """Wrap to (-pi, pi].

    Not cosmetic. Without it a heading error of +3.2 rad reads as a large left
    turn when the short way round is a small right turn, so the robot takes
    the long way about — or, sitting near the wrap point, oscillates across it.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def stop(state: int, reason: str, range_m: float = 0.0,
         heading_error_rad: float = 0.0) -> Command:
    """A zero command in a terminal state. The only way this module stops."""
    return Command(
        vx=0.0, wz=0.0, state=state, reason=reason,
        range_m=range_m, heading_error_rad=heading_error_rad,
    )


def validate_target(
    robot_x: float, robot_y: float, target_x: float, target_y: float,
    limits: Limits,
) -> Optional[str]:
    """Return None if a waypoint may be accepted, else the refusal reason.

    Run ONCE, when the operator asks for it, so a bad waypoint is refused with
    an explanation at the moment of the click rather than aborting three
    seconds into a run. The range ceiling is the load-bearing check: a
    misplaced click and a frame confusion both produce a number that looks
    perfectly ordinary in a text box, and this is the only thing standing
    between that and the robot setting off across the building.
    """
    if not all(math.isfinite(v) for v in (target_x, target_y)):
        return "target is not finite"
    if not all(math.isfinite(v) for v in (robot_x, robot_y)):
        return "robot pose is not finite"
    rng = math.hypot(target_x - robot_x, target_y - robot_y)
    if rng > limits.max_range_m:
        return (
            f"waypoint is {rng:.2f} m away, over the {limits.max_range_m:.1f} m "
            f"ceiling (config/waypoint.yaml max_range_m)"
        )
    return None


def compute(
    *,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    target_x: float,
    target_y: float,
    elapsed_s: float,
    limits: Limits,
    progress: "ProgressWatchdog",
) -> Command:
    """One control tick.

    Order of precedence, and it matters — every give-up condition is checked
    BEFORE the control law, so a run that should have ended cannot emit one
    more command on its way out:

        1. non-finite pose      -> ABORTED (we do not know where we are)
        2. timeout              -> ABORTED
        3. arrived              -> ARRIVED
        4. no progress          -> ABORTED (usually rm_safety holding us)
        5. heading error large  -> TURNING
        6. otherwise            -> DRIVING
    """
    if not all(math.isfinite(v) for v in (robot_x, robot_y, robot_yaw)):
        return stop(ABORTED, "pose_nonfinite")
    if not all(math.isfinite(v) for v in (target_x, target_y)):
        return stop(ABORTED, "target_nonfinite")

    dx = target_x - robot_x
    dy = target_y - robot_y
    rng = math.hypot(dx, dy)
    # Bearing to the target expressed in the robot's OWN frame: how far it
    # would have to turn, not where the target is in the world.
    heading_err = _wrap(math.atan2(dy, dx) - robot_yaw)

    if elapsed_s > limits.timeout_s:
        return stop(ABORTED, f"timeout after {elapsed_s:.0f}s", rng, heading_err)

    # Arrival is judged on RANGE ALONE, with no heading condition. The point of
    # this controller is to put the robot somewhere, not to point it somewhere;
    # demanding a final heading would leave it spinning on the spot at the
    # destination for no operator benefit.
    if rng <= limits.arrive_tol_m:
        return stop(ARRIVED, f"within {limits.arrive_tol_m:.2f} m", rng, heading_err)

    stalled = progress.update(rng, elapsed_s)
    if stalled is not None:
        return stop(ABORTED, stalled, rng, heading_err)

    if abs(heading_err) > limits.turn_in_place_rad:
        wz = _shape(limits.kp_yaw * heading_err, limits.min_wz, limits.max_wz)
        if wz == 0.0:
            # Unreachable while kp_yaw * turn_in_place_rad >= min_wz, which
            # check_limits() enforces and a test pins. Aborting rather than
            # returning a silent zero-command TURNING state means a bad config
            # shows up as a reason on the dashboard instead of as a robot that
            # sits still looking busy.
            return stop(
                ABORTED, "yaw command below the dead-band floor", rng, heading_err
            )
        return Command(0.0, wz, TURNING, "", rng, heading_err)

    # --- driving ----------------------------------------------------------
    # Scale forward speed by how well we are pointed at the target, so a
    # steering correction slows the robot rather than curving it wide. cos() is
    # 1.0 dead-on and falls away toward the turn-in-place threshold.
    vx = _shape(
        limits.kp_lin * rng * math.cos(heading_err), limits.min_vx, limits.max_vx
    )
    # Yaw while driving gets NO dead-band floor. A small steering correction
    # the chassis ignores is harmless — the robot drives on, the error grows,
    # and it is corrected properly once it matters — whereas forcing every trim
    # up to min_wz would make the robot slalom the whole way there.
    wz = max(-limits.max_wz, min(limits.max_wz, limits.kp_yaw * heading_err))
    return Command(vx, wz, DRIVING, "", rng, heading_err)


def _shape(raw: float, floor: float, ceiling: float) -> float:
    """Clamp to `ceiling`, then collapse anything under `floor` to zero.

    The collapse is the dead-band handling described in the module docstring: a
    command too small to move this robot is not a small motion, it is NO
    motion, and commanding it anyway means claiming progress that is not
    happening. Sign is preserved through both operations.
    """
    magnitude = min(abs(raw), ceiling)
    if magnitude < floor:
        return 0.0
    return math.copysign(magnitude, raw)


class ProgressWatchdog:
    """Gives up when the range stops closing.

    THIS IS THE OBSTACLE CASE, and it is why the control law above needs no
    concept of an obstacle. When rm_safety's stop zone fires, the robot stops
    moving while this controller keeps commanding — the command is being zeroed
    a layer down, and nothing in the pose or the geometry says so. What does
    say so is that the range has stopped falling.

    It equally catches a wheel that will not break loose, a robot wedged on a
    cable, and a pose source that has frozen. All four want the same response:
    stop, say so, and let the operator decide. Never retry into an obstacle
    nobody has looked at.

    The reference range re-arms whenever REAL progress is made, so a slow
    approach is not mistaken for a stall: the test is "has it closed
    `progress_min_m` since the last time it did", not "is it moving right now".
    """

    def __init__(self, window_s: float, min_m: float) -> None:
        self._window_s = window_s
        self._min_m = min_m
        self._best_range: Optional[float] = None
        self._best_at: float = 0.0

    def update(self, range_m: float, elapsed_s: float) -> Optional[str]:
        """Record a range sample; return a reason string if progress stalled."""
        if self._best_range is None:
            self._best_range, self._best_at = range_m, elapsed_s
            return None
        if range_m <= self._best_range - self._min_m:
            self._best_range, self._best_at = range_m, elapsed_s
            return None
        if elapsed_s - self._best_at > self._window_s:
            return (
                f"no progress: {self._best_range - range_m:+.3f} m in "
                f"{elapsed_s - self._best_at:.1f}s "
                f"(obstacle, safety stop, or a stalled wheel)"
            )
        return None


def limits_from_dict(cfg: dict) -> Tuple[Limits, Optional[str]]:
    """Build `Limits` from the `waypoint:` block of config/waypoint.yaml.

    Returns `(limits, error)`. A MISSING key is an error, not a default: a
    config that silently omits or shadows a limit is exactly the bug class that
    cost this project several hardware runs, so it is better to refuse to
    start than to run on a number nobody chose.
    """
    fields = Limits.__dataclass_fields__
    missing = [name for name in fields if name not in cfg]
    if missing:
        return Limits(), f"config/waypoint.yaml is missing: {', '.join(sorted(missing))}"
    try:
        values = {name: float(cfg[name]) for name in fields}
    except (TypeError, ValueError) as exc:
        return Limits(), f"config/waypoint.yaml has a non-numeric limit: {exc}"

    limits = Limits(**values)
    return limits, check_limits(limits)


def check_limits(limits: Limits) -> Optional[str]:
    """Refuse a limit set that cannot produce sane motion.

    Each check prevents a specific failure, and every one of them is SILENT in
    the absence of this function: the robot would start, run, and simply
    behave wrongly, which is the hardest kind of fault to attribute on
    hardware.
    """
    if limits.min_vx > limits.max_vx:
        return "min_vx exceeds max_vx: the robot could never drive"
    if limits.min_wz > limits.max_wz:
        return "min_wz exceeds max_wz: the robot could never turn"
    if limits.arrive_tol_m >= limits.max_range_m:
        return "arrive_tol_m >= max_range_m: every waypoint would arrive instantly"
    # Turning in place must clear the dead-band floor at the very threshold
    # where turning begins, or the controller enters TURNING, commands zero,
    # never rotates, and eventually aborts on the progress watchdog with a
    # reason that points at the wrong thing entirely.
    if limits.kp_yaw * limits.turn_in_place_rad < limits.min_wz:
        return (
            f"kp_yaw * turn_in_place_rad "
            f"({limits.kp_yaw * limits.turn_in_place_rad:.3f}) is below min_wz "
            f"({limits.min_wz:.3f}): a turn at the threshold would command zero"
        )
    if limits.progress_min_m <= 0.0 or limits.progress_window_s <= 0.0:
        return "progress watchdog needs a positive window and distance"
    return None
