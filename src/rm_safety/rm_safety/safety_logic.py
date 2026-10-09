"""
safety_logic.py — Pure, deterministic safety FSM logic for the rm_safety node.

What it does:
    Implements all safety decisions as plain functions with no I/O, no ROS
    imports, no clock reads, and no logging side effects.  The rm_safety ROS
    node calls ``evaluate()`` once per cycle, passes its current sensor data
    and state, and acts on the returned ``Decision``.

Subscribes to:  (nothing — this module has no I/O; the node passes values in)
Publishes:      (nothing — this module has no I/O; the node acts on the return)

Failure mode:
    Fail-closed everywhere.  Any unexpected input (NaN, inf, zero/negative dt,
    empty scan arc, malformed limits) produces ZERO velocity output, never a
    pass-through of the last command.  An unhandled exception inside
    ``evaluate()`` is caught at the top level and returns a fully-zeroed
    STOPPED Decision with reason "internal_error" so the caller always gets a
    safe value.

Constants:
    All numeric limits are taken from the ``Limits`` dataclass, which the node
    builds from ``config/safety_limits.yaml`` via ``Limits.from_dict()``.
    No numeric literals appear in the logic bodies.

State enum values match SafetyStatus.msg constants exactly:
    OK=0, SLOW=1, STOPPED=2, KILLED=3

Reason vocabulary (returned in Decision.reason and published in SafetyStatus):
    "killed"          — hard kill switch active
    "scan_stale"      — /scan not received within scan_timeout_ms, or no valid
                        returns in the monitored arcs (perception loss)
    "odom_stale"      — /odom not received within odom_timeout_ms; proprioception
                        lost; cannot safely apply acceleration limiting
    "stop_zone"       — lidar stop-zone latched (obstacle < stop_zone_m)
    "command_timeout" — active command source has not published within
                        command_timeout_ms
    "slow_zone"       — obstacle in slow zone; velocity scaled proportionally
    "ok"              — nominal; all checks pass
    "internal_error"  — unexpected exception inside evaluate(); always zeroes
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# State enum — values MUST match rm_interfaces/msg/SafetyStatus.msg
# ---------------------------------------------------------------------------

class State(IntEnum):
    """FSM states.  Numeric values are contractually identical to those in
    ``SafetyStatus.msg``.  Do not renumber."""
    OK      = 0
    SLOW    = 1
    STOPPED = 2
    KILLED  = 3


# ---------------------------------------------------------------------------
# Limits dataclass
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class VelocityLimits:
    """Per-axis absolute velocity caps (m/s or rad/s)."""
    max_vx: float
    max_vy: float
    max_wz: float
    policy_vy_expected: float
    policy_vy_violation_action: str  # "zero_and_warn" is the only defined value


@dataclass(frozen=True)
class AccelerationLimits:
    """Per-axis acceleration caps (m/s² or rad/s²)."""
    max_ax: float
    max_ay: float
    max_alpha_z: float


@dataclass(frozen=True)
class EstopLimits:
    """Lidar e-stop zone configuration."""
    slow_zone_m: float
    stop_zone_m: float
    # The STOP zone's arcs, and the default for the slow zone.
    forward_arc_deg: float
    lateral_arc_deg: float
    consecutive_scans_to_trigger: int
    latching: bool
    # ------------------------------------------------------------------
    # SLOW-ZONE ARCS, separate from the stop zone's (added 2026-08-29).
    #
    # Before this, ONE arc pair served both zones, so narrowing the slow zone
    # to stop side walls from throttling forward motion also narrowed the
    # STOP zone — trading a comfort problem for a safety reduction. These
    # exist so the slow zone can be a narrow forward cone while the stop zone
    # keeps its full coverage.
    #
    # None means "same as the stop arcs", which is the pre-2026-08-29
    # behaviour and what a config without these keys gets.
    #
    # INVARIANT, enforced in from_dict: the slow arcs must not be WIDER than
    # the stop arcs. The logic relies on the slow arc being a subset, so that
    # min_range_slow >= min_range always holds; a wider slow arc would mean an
    # obstacle could be "slow" without the stop zone ever having looked at it.
    slow_forward_arc_deg: Optional[float] = None
    slow_lateral_arc_deg: Optional[float] = None
    # Chassis self-hit exclusion zones, degrees, [(lo, hi), ...]. A ray whose
    # angle falls in any zone is dropped before the forward/lateral arc
    # check in min_range_in_arcs(). See safety_limits.yaml estop.
    # self_occlusion_zones_deg for the measurement behind this.
    self_occlusion_zones_deg: Tuple[Tuple[float, float], ...] = ()
    # ------------------------------------------------------------------
    # ZONE MASTER SWITCHES (added 2026-08-30, operator-approved).
    #
    # Both default TRUE. A config that omits them gets the full e-stop,
    # which is the only acceptable default: a safety feature that has to be
    # remembered is one that will eventually be forgotten.
    #
    # WHAT DISABLING ACTUALLY REMOVES. These are the ONLY obstacle-based
    # protection on this robot, and there is NO PHYSICAL E-STOP. With
    # stop_zone_enabled false, nothing in this workspace will stop the robot
    # before a collision — the velocity clamps bound SPEED, not PROXIMITY,
    # so the robot will drive into a wall at max_vx rather than at 2 m/s.
    # That is a smaller impact, not the absence of one.
    #
    # WHAT SURVIVES, deliberately, because none of it is zone geometry:
    #   * every velocity clamp (max_vx / max_vy / max_wz)
    #   * every acceleration limit
    #   * the policy_vy violation check
    #   * /safety/kill and the killed-on-startup default
    #   * the command, scan and odom staleness watchdogs
    # So the kill switch remains the operator's stop, and a dead lidar or a
    # silent policy still stops the robot. Only "there is an object N metres
    # ahead" stops mattering.
    #
    # Startup-only, and NOT settable via `ros2 param set` — see
    # safety_node._on_set_parameters. Disabling the e-stop is a decision made
    # when the stack is brought up, not one a browser can make mid-run.
    slow_zone_enabled: bool = True
    stop_zone_enabled: bool = True

    @property
    def any_zone_enabled(self) -> bool:
        """True while at least one lidar zone is still acting on the scan.

        Used to decide whether a scan with no valid returns is a fault. With
        both zones off, nothing consumes the arc minimum, so "no returns in
        the monitored arcs" is not evidence about anything and must not stop
        the robot on its own. Message-age staleness is a separate check and
        is never gated on this.
        """
        return self.slow_zone_enabled or self.stop_zone_enabled

    @property
    def zones_status_suffix(self) -> str:
        """Reason-string marker naming exactly which zones are disabled.

        Empty while the e-stop is whole. Non-empty strings are appended to
        the nominal reason so that /safety/status — and therefore the
        dashboard, and therefore every recorded bag — carries the fact
        CONTINUOUSLY rather than only in a startup log line somebody has
        already scrolled past.
        """
        if self.slow_zone_enabled and self.stop_zone_enabled:
            return ""
        if not self.slow_zone_enabled and not self.stop_zone_enabled:
            return "_estop_disabled"
        if not self.slow_zone_enabled:
            return "_slow_zone_disabled"
        return "_stop_zone_disabled"

    @property
    def effective_slow_forward_arc_deg(self) -> float:
        """Slow-zone forward arc, falling back to the stop zone's."""
        return (
            self.forward_arc_deg
            if self.slow_forward_arc_deg is None
            else self.slow_forward_arc_deg
        )

    @property
    def effective_slow_lateral_arc_deg(self) -> float:
        """Slow-zone lateral arc, falling back to the stop zone's.

        0.0 is meaningful and is NOT the same as None: it disables lateral
        monitoring for the slow zone entirely, which is the point of narrowing
        to a forward cone. None means "inherit the stop zone's arc".
        """
        return (
            self.lateral_arc_deg
            if self.slow_lateral_arc_deg is None
            else self.slow_lateral_arc_deg
        )

    @property
    def slow_arcs_differ(self) -> bool:
        """True when the slow zone has its own, narrower geometry."""
        return (
            self.effective_slow_forward_arc_deg != self.forward_arc_deg
            or self.effective_slow_lateral_arc_deg != self.lateral_arc_deg
        )


@dataclass(frozen=True)
class TimeoutLimits:
    """Staleness timeouts (milliseconds)."""
    command_timeout_ms: float
    scan_timeout_ms: float
    odom_timeout_ms: float


@dataclass(frozen=True)
class Limits:
    """Aggregated safety limits.

    Build via ``Limits.from_dict(yaml_dict)`` rather than instantiating
    directly, so the field mapping is maintained in one place.

    Parameters
    ----------
    velocity:
        Absolute per-axis velocity caps.
    acceleration:
        Per-axis acceleration caps.
    estop:
        Lidar e-stop zone geometry and noise filter.
    timeouts:
        Staleness detection thresholds.
    """
    velocity: VelocityLimits
    acceleration: AccelerationLimits
    estop: EstopLimits
    timeouts: TimeoutLimits

    @staticmethod
    def from_dict(d: dict) -> "Limits":
        """Parse the top-level ``safety_limits.yaml`` dictionary.

        Parameters
        ----------
        d:
            Raw dict produced by ``yaml.safe_load`` on ``safety_limits.yaml``.

        Returns
        -------
        Limits:
            Fully populated, frozen dataclass.

        Raises
        ------
        KeyError:
            If any expected key is absent — the caller should treat this as a
            fatal configuration error.
        """
        v = d["velocity_limits"]
        a = d["acceleration_limits"]
        e = d["estop"]
        t = d["timeouts"]

        limits = Limits(
            velocity=VelocityLimits(
                max_vx=float(v["max_vx"]),
                max_vy=float(v["max_vy"]),
                max_wz=float(v["max_wz"]),
                policy_vy_expected=float(v["policy_vy_expected"]),
                policy_vy_violation_action=str(v["policy_vy_violation_action"]),
            ),
            acceleration=AccelerationLimits(
                max_ax=float(a["max_ax"]),
                max_ay=float(a["max_ay"]),
                max_alpha_z=float(a["max_alpha_z"]),
            ),
            estop=EstopLimits(
                slow_zone_m=float(e["slow_zone_m"]),
                stop_zone_m=float(e["stop_zone_m"]),
                forward_arc_deg=float(e["forward_arc_deg"]),
                lateral_arc_deg=float(e["lateral_arc_deg"]),
                slow_forward_arc_deg=(
                    float(e["slow_forward_arc_deg"])
                    if e.get("slow_forward_arc_deg") is not None else None
                ),
                slow_lateral_arc_deg=(
                    float(e["slow_lateral_arc_deg"])
                    if e.get("slow_lateral_arc_deg") is not None else None
                ),
                consecutive_scans_to_trigger=int(e["consecutive_scans_to_trigger"]),
                latching=bool(e["latching"]),
                # .get with a True default: a config predating these keys —
                # every config written before 2026-08-30 — gets the full
                # e-stop. Absence must never read as "disabled".
                slow_zone_enabled=bool(e.get("slow_zone_enabled", True)),
                stop_zone_enabled=bool(e.get("stop_zone_enabled", True)),
                self_occlusion_zones_deg=tuple(
                    (float(lo), float(hi))
                    for lo, hi in e.get("self_occlusion_zones_deg", [])
                ),
            ),
            timeouts=TimeoutLimits(
                command_timeout_ms=float(t["command_timeout_ms"]),
                scan_timeout_ms=float(t["scan_timeout_ms"]),
                odom_timeout_ms=float(t["odom_timeout_ms"]),
            ),
        )

        # INVARIANT: the slow arcs must be a SUBSET of the stop arcs.
        #
        # The zone logic relies on min_range_slow >= min_range, which holds
        # only while the slow arc is contained in the stop arc. A wider slow
        # arc would let an obstacle register as "slow" in a sector the stop
        # zone never examines — a slow zone with no stop zone behind it, which
        # is the opposite of the layering this design depends on.
        #
        # Fatal rather than clamped: a misconfigured safety arc must stop the
        # node from starting, not be silently corrected into something the
        # operator did not write.
        es = limits.estop
        if es.effective_slow_forward_arc_deg > es.forward_arc_deg:
            raise ValueError(
                f"slow_forward_arc_deg ({es.effective_slow_forward_arc_deg}) "
                f"must not exceed forward_arc_deg ({es.forward_arc_deg}): the "
                f"slow zone must sit inside the stop zone's coverage."
            )
        if es.effective_slow_lateral_arc_deg > es.lateral_arc_deg:
            raise ValueError(
                f"slow_lateral_arc_deg ({es.effective_slow_lateral_arc_deg}) "
                f"must not exceed lateral_arc_deg ({es.lateral_arc_deg}): the "
                f"slow zone must sit inside the stop zone's coverage."
            )
        return limits


# ---------------------------------------------------------------------------
# Inputs / Decision containers
# ---------------------------------------------------------------------------

@dataclass
class EvalInputs:
    """All inputs needed by ``evaluate()`` for one cycle.

    Ages are computed by the node (``now - last_stamp``), so this module stays
    clockless.

    Parameters
    ----------
    cmd_vx, cmd_vy, cmd_wz:
        Commanded velocity from the active source (policy or teleop).
    is_policy_source:
        True when the active command comes from ``/policy/cmd_vel``.  Governs
        whether ``policy_vy_check`` is applied.
    command_age_ms:
        Milliseconds since the last command was received.
    last_vx, last_vy, last_wz:
        The last velocity actually published by the safety node (used for
        acceleration limiting).
    dt:
        Elapsed time in seconds since the last publish cycle.
    scan_ranges:
        Full lidar range array (metres, may contain NaN/inf for invalid rays).
    scan_angle_min:
        Angle of the first element of ``scan_ranges`` (radians).
    scan_angle_increment:
        Angular step between consecutive elements (radians, must be > 0).
    scan_range_min:
        Sensor minimum valid range (metres, from LaserScan header).
    scan_range_max:
        Sensor maximum valid range (metres, from LaserScan header).
    scan_age_ms:
        Milliseconds since the last ``/scan`` message was received.
    is_killed:
        True when the hard kill switch is active.
    estop_latched:
        True when the node is already latched in stop-zone state.
    consecutive_stop_count:
        How many consecutive scans have been in the stop zone (for the noise
        filter).  The node holds this; evaluate returns the updated value.
    reset_latch:
        True when the node has received a ``/safety/reset`` call this cycle.
    """
    # Command
    cmd_vx: float
    cmd_vy: float
    cmd_wz: float
    is_policy_source: bool
    command_age_ms: float
    # Acceleration context
    last_vx: float
    last_vy: float
    last_wz: float
    dt: float
    # Lidar
    scan_ranges: List[float]
    scan_angle_min: float
    scan_angle_increment: float
    scan_range_min: float
    scan_range_max: float
    scan_age_ms: float
    # FSM state
    is_killed: bool
    estop_latched: bool
    consecutive_stop_count: int
    reset_latch: bool
    # Odometry staleness — set by the node when /odom has not arrived within
    # timeouts.odom_timeout_ms.  odom never received → age is inf → stale=True,
    # which is the correct fail-closed behaviour.  When True, evaluate() returns
    # a zeroed STOPPED decision with reason="odom_stale" at precedence 2.5
    # (after scan-stale, before stop-zone/command-stale).
    odom_stale: bool = False


@dataclass
class Decision:
    """Output of ``evaluate()`` for one cycle.

    The node publishes ``out_vx/out_vy/out_wz`` to ``/cmd_vel``, updates its
    internal ``estop_latched`` and ``consecutive_stop_count`` from the returned
    values, and builds a ``SafetyStatus`` from the remaining fields.

    Parameters
    ----------
    state:
        Current FSM state (see ``State`` enum).
    reason:
        String matching the SafetyStatus reason vocabulary.
    out_vx, out_vy, out_wz:
        Velocity to publish.  Zero on any non-OK/non-SLOW state.
    clamping_active:
        True if any axis was clamped or altered from the raw command.
    min_range:
        Minimum lidar range seen in monitored arcs this cycle (metres).
        ``math.inf`` when no valid scan is available.
    estop_latched:
        Updated latch state for the node to store for next cycle.
    consecutive_stop_count:
        Updated consecutive-stop counter for the node to store for next cycle.
    """
    state: State
    reason: str
    out_vx: float
    out_vy: float
    out_wz: float
    clamping_active: bool
    min_range: float
    estop_latched: bool
    consecutive_stop_count: int


# ---------------------------------------------------------------------------
# 1. Velocity clamping
# ---------------------------------------------------------------------------

def clamp_velocity(
    cmd_vx: float,
    cmd_vy: float,
    cmd_wz: float,
    limits: Limits,
) -> Tuple[float, float, float, bool]:
    """Scale the whole command by ONE factor so every axis fits its limit.

    The factor is the largest s <= 1 with |s*vx| <= max_vx, |s*vy| <= max_vy
    and |s*wz| <= max_wz, applied to all three axes together (operator
    decision 2026-10-09). Clamping each axis on its own changed the SHAPE of
    the command: the trained policy asks for 1.2 m/s with 0.36 rad/s, a
    3.3 m-radius arc, and a per-axis clamp turned that into 0.25 m/s with
    0.36 rad/s, a 0.7 m circle. The robot looped where the policy meant to
    arc. One factor keeps the curvature (wz/vx) and the strafe direction the
    source asked for, at the capped speed. It is never looser than the
    per-axis clamp: the factor is bounded by every axis's own limit, and a
    final per-axis clamp guards the floating-point rounding of the product.

    Fail-closed: a NaN or infinite value on any axis becomes 0.0 on THAT
    axis and raises the clamped flag; the finite axes are then scaled.

    Parameters
    ----------
    cmd_vx, cmd_vy, cmd_wz:
        Commanded velocity (m/s, m/s, rad/s).
    limits:
        Loaded safety limits.

    Returns
    -------
    (vx, vy, wz, clamped):
        Scaled velocities and a flag indicating whether the command was
        altered (any axis non-finite, or the scale factor below 1).
    """
    clamped = False
    finite = []
    for val in (cmd_vx, cmd_vy, cmd_wz):
        if math.isfinite(val):
            finite.append(val)
        else:
            finite.append(0.0)
            clamped = True
    vx, vy, wz = finite

    scale = 1.0
    for val, max_abs in (
        (vx, limits.velocity.max_vx),
        (vy, limits.velocity.max_vy),
        (wz, limits.velocity.max_wz),
    ):
        if abs(val) > max_abs:
            # abs(val) > max_abs >= 0, so the division is safe.
            scale = min(scale, max_abs / abs(val))
    if scale < 1.0:
        vx, vy, wz = vx * scale, vy * scale, wz * scale
        clamped = True

    # Rounding guard only: the scaled product can sit one ulp past the limit.
    vx = math.copysign(min(abs(vx), limits.velocity.max_vx), vx)
    vy = math.copysign(min(abs(vy), limits.velocity.max_vy), vy)
    wz = math.copysign(min(abs(wz), limits.velocity.max_wz), wz)
    return vx, vy, wz, clamped


# ---------------------------------------------------------------------------
# 2. Acceleration limiting
# ---------------------------------------------------------------------------

def apply_accel_limit(
    target_vx: float,
    target_vy: float,
    target_wz: float,
    last_vx: float,
    last_vy: float,
    last_wz: float,
    dt: float,
    limits: Limits,
) -> Tuple[float, float, float, bool]:
    """Limit the per-axis velocity change to max_a * dt.

    Fail-closed on bad dt: when dt <= 0, dt is NaN, or dt is infinite the
    function returns ZEROS (not the last command) with limited=True.  Returning
    the last command on a bad dt would silently perpetuate stale motion, which
    violates the fail-closed principle.  The caller must always check the
    returned limited flag and handle the zero case.

    Parameters
    ----------
    target_vx, target_vy, target_wz:
        Desired velocity after velocity clamping.
    last_vx, last_vy, last_wz:
        Velocity published in the previous cycle.
    dt:
        Elapsed seconds since the previous publish.
    limits:
        Loaded safety limits.

    Returns
    -------
    (vx, vy, wz, limited):
        Acceleration-limited velocities and a flag indicating whether any axis
        was limited.  Returns (0,0,0,True) if dt is bad.
    """
    if not math.isfinite(dt) or dt <= 0.0:
        return 0.0, 0.0, 0.0, True

    limited = False

    def _limit_axis(target: float, last: float, max_a: float) -> Tuple[float, bool]:
        max_delta = max_a * dt
        delta = target - last
        if abs(delta) > max_delta:
            return last + math.copysign(max_delta, delta), True
        return target, False

    vx, l0 = _limit_axis(target_vx, last_vx, limits.acceleration.max_ax)
    vy, l1 = _limit_axis(target_vy, last_vy, limits.acceleration.max_ay)
    wz, l2 = _limit_axis(target_wz, last_wz, limits.acceleration.max_alpha_z)
    limited = l0 or l1 or l2

    return vx, vy, wz, limited


# ---------------------------------------------------------------------------
# 3. Policy vy check
# ---------------------------------------------------------------------------

# Floating-point representation tolerance: values whose absolute distance from
# the expected vy (0.0) is smaller than this are indistinguishable from zero
# at IEEE-754 double precision and are treated as such.  This is NOT a tunable
# safety threshold — it is not in safety_limits.yaml because its value is
# determined by floating-point arithmetic, not by robot geometry or risk.
# Raising it would silently accept real lateral commands as noise; lowering it
# below ~1e-15 is meaningless (sub-epsilon from 0.0).
_POLICY_VY_EPSILON: float = 1e-6


def policy_vy_check(
    cmd_vy: float,
    limits: Limits,
) -> Tuple[float, bool]:
    """Detect and correct an unexpected lateral velocity on the POLICY source.

    This function applies ONLY to commands arriving on ``/policy/cmd_vel``.
    The policy action space is 2-dim ``[vx, wz]`` since interface 2.0.0; a
    non-zero ``linear.y`` there is a bug (e.g. a stale 3-dim model), not a
    command, and must be surfaced rather than silently clamped.

    For the TELEOP source, lateral velocity is legitimate (the operator may
    strafe).  Do NOT call this function on teleop commands — apply
    ``clamp_velocity`` instead, which handles vy normally.

    Parameters
    ----------
    cmd_vy:
        The ``linear.y`` component from ``/policy/cmd_vel``.
    limits:
        Loaded safety limits (``policy_vy_expected`` and
        ``policy_vy_violation_action`` are read here).

    Returns
    -------
    (corrected_vy, violated):
        ``corrected_vy`` is 0.0 when a violation is detected (or when
        ``cmd_vy`` is NaN/inf — fail-closed).
        ``violated`` is True when the input exceeded the epsilon threshold.
        The caller (the ROS node) is responsible for emitting the warning log.
    """
    if not math.isfinite(cmd_vy):
        return 0.0, True
    if abs(cmd_vy - limits.velocity.policy_vy_expected) > _POLICY_VY_EPSILON:
        return 0.0, True
    return cmd_vy, False


# ---------------------------------------------------------------------------
# 4. Lidar arc extraction
# ---------------------------------------------------------------------------

def min_range_in_arcs(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    limits: Limits,
    *,
    forward_arc_deg: Optional[float] = None,
    lateral_arc_deg: Optional[float] = None,
) -> float:
    """Compute the minimum valid range within the monitored forward and lateral arcs.

    The lidar is a 360° RPLidar A1 mounted on the robot.  This function
    selects only the arcs we monitor:

    * Forward arc: ``forward_arc_deg / 2`` centred on 0 rad (the robot's +x
      axis), i.e. ``[-half_fwd, +half_fwd]``.
    * Lateral arcs: ``lateral_arc_deg`` centred at ``+90°`` and ``-90°``
      (pure left and right).  Each arc spans ``lateral_arc_deg / 2`` to each
      side of its centre.

    The rear hemisphere is chassis-occluded and NOT monitored.

    Rays whose angle falls inside any of ``limits.estop.self_occlusion_zones_deg``
    are dropped before the arc check — these are known chassis self-hits
    (e.g. mounting standoffs), not real obstacles. This mirrors the lidar
    observation pipeline's self-occlusion mask (interface.yaml) but is a
    separate mask, since this function's arcs are independently configured
    (forward_arc_deg + lateral_arc_deg, not the observation's forward-180).
    Masking a ray removes it from consideration entirely — it does not count
    as "clear" evidence, so a monitored arc whose only returns are masked
    correctly falls back to no-signal-in-zone, not a phantom all-clear.

    Invalid returns (NaN, inf, <= 0, outside ``[range_min, range_max]``) are
    skipped.  If NO valid returns exist in the monitored arcs, ``+inf`` is
    returned.  The caller (``evaluate``) MUST treat a no-valid-returns result
    as an e-stop condition (fail-closed — zero valid returns means no
    perception, not "obstacle-free").

    Note on angle normalisation: angles are normalised to ``(-pi, pi]`` before
    comparison so that the function is robust to sensors whose ``angle_min`` is
    not exactly ``-pi``.

    Parameters
    ----------
    ranges:
        Full range array from the LaserScan message.
    angle_min:
        Angle of the first element (radians).
    angle_increment:
        Angular step between consecutive elements (radians, must be > 0).
    limits:
        Loaded safety limits (``estop`` group used here).

    Returns
    -------
    float:
        Minimum range (metres) in the monitored arcs, or ``math.inf`` if no
        valid returns are found.
    """
    if not ranges or angle_increment <= 0.0 or not math.isfinite(angle_increment):
        return math.inf

    fwd_deg = (
        limits.estop.forward_arc_deg if forward_arc_deg is None else forward_arc_deg
    )
    lat_deg = (
        limits.estop.lateral_arc_deg if lateral_arc_deg is None else lateral_arc_deg
    )
    half_fwd = math.radians(fwd_deg / 2.0)
    half_lat = math.radians(lat_deg / 2.0)
    # A lateral arc of zero DISABLES lateral monitoring rather than degenerating
    # into a hairline arc that only matches rays landing exactly on ±90°. That
    # distinction matters: the narrow slow cone sets this to 0 deliberately, and
    # "occasionally catches a ray that happens to hit ±90.0°" would be a
    # non-deterministic slow trigger rather than a disabled one.
    lateral_enabled = lat_deg > 0.0

    # Lateral arc centres at ±90°
    lat_centre_left  = math.pi / 2.0
    lat_centre_right = -math.pi / 2.0

    range_min = 0.0   # anything <= 0 is invalid regardless of sensor spec
    # range_max taken from LaserScan; passed in via scan_range_max in EvalInputs
    # but this function does not have access to it — it trusts the caller to
    # pass only physically meaningful ranges.  See NOTE below.
    #
    # NOTE: ``range_max`` filtering is done in ``evaluate()`` before calling
    # this function.  Here we only reject values <= 0 and non-finite values
    # to avoid duplicating the sensor spec across two call sites.

    min_r = math.inf

    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= range_min:
            continue

        angle_raw = angle_min + i * angle_increment
        # Normalise to (-pi, pi]
        angle = (angle_raw + math.pi) % (2 * math.pi) - math.pi
        angle_deg = math.degrees(angle)

        if any(lo <= angle_deg <= hi for lo, hi in limits.estop.self_occlusion_zones_deg):
            continue

        in_forward = abs(angle) <= half_fwd
        in_left  = lateral_enabled and abs(angle - lat_centre_left)  <= half_lat
        in_right = lateral_enabled and abs(angle - lat_centre_right) <= half_lat

        if in_forward or in_left or in_right:
            if r < min_r:
                min_r = r

    return min_r


def estop_zone(
    min_range: float,
    limits: Limits,
    min_range_slow: Optional[float] = None,
) -> str:
    """Map a minimum range value to an e-stop zone label.

    Boundary inclusivity (documented and tested):
    - ``min_range < stop_zone_m``  → ``"stop"``   (strictly less than)
    - ``min_range < slow_zone_m``  → ``"slow"``   (strictly less than)
    - Otherwise                    → ``"clear"``

    A range exactly equal to the stop boundary is therefore "slow", not
    "stop".  This is deliberately conservative — the transition to stop
    happens the moment a value drops *below* the boundary.  The slow-to-clear
    boundary works the same way.

    ``math.inf`` → ``"clear"`` (no valid returns; evaluate handles the
    "no perception" case separately — see the ``min_range_in_arcs`` docstring).

    Parameters
    ----------
    min_range:
        Minimum range returned by ``min_range_in_arcs`` (metres).
    limits:
        Loaded safety limits.

    Returns
    -------
    str:
        One of ``"clear"``, ``"slow"``, ``"stop"``.
    """
    # A DISABLED ZONE IS NEVER ENTERED. Gating here rather than at each call
    # site means every consumer of this function — the latch counter, the
    # velocity scaling, the reason string — is switched off by one decision,
    # with no second place to forget.
    if limits.estop.stop_zone_enabled and min_range < limits.estop.stop_zone_m:
        return "stop"
    if limits.estop.slow_zone_enabled:
        # The slow test uses the SLOW arcs' minimum when one was supplied. An
        # obstacle off to the side is then still fully covered by the stop test
        # above (which used the wide arcs) but no longer throttles forward
        # motion.
        slow_r = min_range if min_range_slow is None else min_range_slow
        if slow_r < limits.estop.slow_zone_m:
            return "slow"
    return "clear"


# ---------------------------------------------------------------------------
# 5. Consecutive-scan noise filter
# ---------------------------------------------------------------------------

def update_stop_counter(
    prev_count: int,
    zone_is_stop: bool,
    limits: Limits,
) -> int:
    """Maintain the consecutive-stop-scan counter.

    Increments when the current zone is "stop"; resets to 0 otherwise.  The
    latch logic in ``evaluate()`` checks ``new_count >=
    consecutive_scans_to_trigger``.

    A single bad scan never reaches the trigger threshold by itself (assuming
    ``consecutive_scans_to_trigger >= 2``), which prevents single noisy returns
    from latching a stop.

    Parameters
    ----------
    prev_count:
        Counter value from the previous cycle (held by the node).
    zone_is_stop:
        Whether the current scan's zone is "stop".
    limits:
        Loaded safety limits.

    Returns
    -------
    int:
        Updated counter.
    """
    if zone_is_stop:
        return prev_count + 1
    return 0


# ---------------------------------------------------------------------------
# 6. Slow-zone velocity scaling
# ---------------------------------------------------------------------------

def slow_scale(
    min_range: float,
    limits: Limits,
) -> float:
    """Proportional velocity scale factor for the slow zone.

    Ramp definition:
    - At ``slow_zone_m`` and beyond → 1.0 (full speed, no reduction).
    - Approaching ``stop_zone_m`` → ramps linearly toward 0.0.
    - At ``stop_zone_m`` → 0.0.
    - Below ``stop_zone_m`` → 0.0 (clamped; evaluate will have chosen STOPPED
      before this value is used, but defensive anyway).

    The scale is linear in the range ``[stop_zone_m, slow_zone_m]``.

    Parameters
    ----------
    min_range:
        Minimum range (metres) in the monitored arcs.
    limits:
        Loaded safety limits.

    Returns
    -------
    float:
        Scale factor in ``[0.0, 1.0]``.
    """
    span = limits.estop.slow_zone_m - limits.estop.stop_zone_m
    if span <= 0.0:
        # Degenerate config: treat as fully stopped.
        return 0.0
    if min_range >= limits.estop.slow_zone_m:
        return 1.0
    if min_range <= limits.estop.stop_zone_m:
        return 0.0
    return (min_range - limits.estop.stop_zone_m) / span


# ---------------------------------------------------------------------------
# 7. Top-level evaluate
# ---------------------------------------------------------------------------

def _zero_decision(
    reason: str,
    state: State,
    min_range: float,
    estop_latched: bool,
    consecutive_stop_count: int,
) -> Decision:
    """Return a fully-zeroed Decision with the given state and reason."""
    return Decision(
        state=state,
        reason=reason,
        out_vx=0.0,
        out_vy=0.0,
        out_wz=0.0,
        clamping_active=False,
        min_range=min_range,
        estop_latched=estop_latched,
        consecutive_stop_count=consecutive_stop_count,
    )


def evaluate(inputs: EvalInputs, limits: Limits) -> Decision:
    """Top-level safety evaluation for one cycle.

    Precedence (highest wins, lower levels not reached once a higher one fires):

    1. KILLED        — hard kill active; output zero regardless of all else.
    2. SCAN STALE    — scan_age_ms > scan_timeout_ms, or no valid returns in
                       arcs (loss of perception); output zero, state STOPPED,
                       reason "scan_stale".
    2.5. ODOM STALE  — inputs.odom_stale is True (node computed from odom age
                       vs odom_timeout_ms); proprioception lost; output zero,
                       state STOPPED, reason "odom_stale".  Placed here because
                       it is a perception/proprioception-loss condition that must
                       precede command and zone checks.  latch/counter passed
                       through unchanged (same as scan_stale branch).
    3. STOP ZONE     — either the estop is already latched (and reset_latch is
                       False), or the consecutive counter reaches the trigger
                       threshold this cycle.  Latch is set; output zero, state
                       STOPPED.
    4. COMMAND STALE — command_age_ms > command_timeout_ms; output zero,
                       state STOPPED.
    5. SLOW ZONE     — obstacle in slow zone, OR in the stop zone but not yet
                       latched (pre-latch window): scale velocity via
                       slow_scale(), state SLOW. The stop-zone case scales to
                       0.0 (min_r <= stop_zone_m), so it behaves as an
                       immediate soft-stop rather than full speed until the
                       latch trips (operator decision, 2026-08-14).
    6. NOMINAL       — velocity clamping + acceleration limiting; state OK.

    Fail-closed: any unhandled exception inside this function is caught and
    returns a zeroed STOPPED Decision with reason "internal_error".

    Parameters
    ----------
    inputs:
        All sensor readings, ages, and prior FSM state for this cycle.
    limits:
        Loaded safety limits.

    Returns
    -------
    Decision:
        Safe velocity to publish and updated FSM state for next cycle.
    """
    try:
        return _evaluate_inner(inputs, limits)
    except Exception:
        # Catch-all: any programming error → fail closed.
        return _zero_decision(
            reason="internal_error",
            state=State.STOPPED,
            min_range=math.inf,
            estop_latched=inputs.estop_latched,
            consecutive_stop_count=inputs.consecutive_stop_count,
        )


def _evaluate_inner(inputs: EvalInputs, limits: Limits) -> Decision:
    """Inner implementation of evaluate — exceptions propagate to evaluate()."""

    # ------------------------------------------------------------------
    # Precedence 1: KILLED
    # ------------------------------------------------------------------
    if inputs.is_killed:
        return _zero_decision(
            reason="killed",
            state=State.KILLED,
            min_range=math.inf,
            estop_latched=inputs.estop_latched,
            consecutive_stop_count=inputs.consecutive_stop_count,
        )

    # ------------------------------------------------------------------
    # Lidar processing — done early so subsequent levels can use min_range
    # and counter regardless of which branch wins.
    # ------------------------------------------------------------------

    # Filter ranges by sensor spec (range_min / range_max from LaserScan).
    # Produce a cleaned list for min_range_in_arcs (NaN/inf/out-of-spec → inf).
    def _sensor_valid(r: float) -> bool:
        return (
            math.isfinite(r)
            and r > inputs.scan_range_min
            and r <= inputs.scan_range_max
        )

    cleaned_ranges: List[float] = [
        r if _sensor_valid(r) else math.inf
        for r in inputs.scan_ranges
    ]

    min_r = min_range_in_arcs(
        cleaned_ranges,
        inputs.scan_angle_min,
        inputs.scan_angle_increment,
        limits,
    )

    # The SLOW zone gets its own, narrower arcs (2026-08-29). Computed as a
    # second pass over the same cleaned ranges rather than folded into the
    # first, because the two numbers answer different questions and are used
    # at different precedence levels:
    #
    #   min_r       — the WIDE stop arcs. Governs the stop zone and the latch,
    #                 and is what gets reported on /safety/status. Coverage
    #                 here is unchanged by the slow-arc narrowing.
    #   min_r_slow  — the NARROW slow arcs. Governs velocity scaling only.
    #
    # Since the slow arc is validated to be a subset of the stop arc,
    # min_r_slow >= min_r always. When the arcs are identical (the default,
    # and every config written before 2026-08-29) the second pass is skipped
    # and the two are the same value, so behaviour is bit-identical.
    if limits.estop.slow_arcs_differ:
        min_r_slow = min_range_in_arcs(
            cleaned_ranges,
            inputs.scan_angle_min,
            inputs.scan_angle_increment,
            limits,
            forward_arc_deg=limits.estop.effective_slow_forward_arc_deg,
            lateral_arc_deg=limits.estop.effective_slow_lateral_arc_deg,
        )
    else:
        min_r_slow = min_r

    # ------------------------------------------------------------------
    # Precedence 2: SCAN STALE or NO VALID PERCEPTION
    # ------------------------------------------------------------------
    scan_stale = inputs.scan_age_ms > limits.timeouts.scan_timeout_ms
    # No valid returns anywhere in the monitored arcs (inf) means loss of
    # perception — whether the list is empty, all-NaN, all-out-of-range, or
    # all-behind-the-robot.  Treat as an e-stop condition.  Fail-closed.
    #
    # GATED ON THE ZONES (2026-08-30). "No returns in the monitored arcs" is a
    # statement about the ARC GEOMETRY, and the arcs exist to feed the zones.
    # With both zones disabled nothing consumes min_r, so an all-no-return
    # scan is not evidence about anything — and on this hardware it is not
    # even rare: the TG30 reports ~31-39% of rays as no-returns in a normal
    # room, so an open space genuinely can leave the forward arc empty. Left
    # ungated it would stop the robot for a reason the operator has just
    # switched off, which reads as the disable not having worked.
    #
    # scan_stale itself is NOT gated and never will be. That is message AGE,
    # not zone geometry: a dead lidar means policy_node is building its
    # observation from a frozen scan, which is a fault regardless of what the
    # e-stop is doing.
    no_perception = (min_r == math.inf) and limits.estop.any_zone_enabled

    if scan_stale or no_perception:
        return _zero_decision(
            reason="scan_stale",
            state=State.STOPPED,
            min_range=min_r,
            estop_latched=inputs.estop_latched,
            consecutive_stop_count=inputs.consecutive_stop_count,
        )

    # ------------------------------------------------------------------
    # Precedence 2.5: ODOM STALE
    # Proprioception loss — /odom has not arrived within odom_timeout_ms.
    # This is a first-class stop condition with its own reason string so that
    # Phase 5 diagnostics can distinguish "odom dropped" from "command timed
    # out".  The node computes odom_stale from the actual odom arrival time and
    # passes it here; we do not see clocks directly (this module is clockless).
    # Latch and counter are passed through unchanged, matching the scan_stale
    # branch — we do not want to disturb the lidar noise filter state.
    # ------------------------------------------------------------------
    if inputs.odom_stale:
        return _zero_decision(
            reason="odom_stale",
            state=State.STOPPED,
            min_range=min_r,
            estop_latched=inputs.estop_latched,
            consecutive_stop_count=inputs.consecutive_stop_count,
        )

    # ------------------------------------------------------------------
    # Consecutive-scan noise filter and latch management
    # ------------------------------------------------------------------
    zone = estop_zone(min_r, limits, min_range_slow=min_r_slow)
    is_stop_zone = (zone == "stop")

    new_count = update_stop_counter(
        inputs.consecutive_stop_count, is_stop_zone, limits
    )

    # Apply reset before evaluating latch.
    if inputs.reset_latch:
        if is_stop_zone:
            # Operator called /safety/reset but the obstacle is still in the
            # stop zone.  Re-latch immediately so the reset cycle itself
            # outputs zero velocity — the robot must not move with a foot in
            # the stop zone.  Reset only re-arms when the path is genuinely
            # clear.
            new_latched = True
            new_count   = limits.estop.consecutive_scans_to_trigger
        else:
            new_latched = False
            new_count   = 0
    else:
        triggered_this_cycle = (
            is_stop_zone
            and new_count >= limits.estop.consecutive_scans_to_trigger
        )
        new_latched = inputs.estop_latched or triggered_this_cycle

    # A DISABLED STOP ZONE CLEARS ANY LATCH IT LEFT BEHIND.
    #
    # `is_stop_zone` is already false while disabled, so nothing new can
    # latch — but `inputs.estop_latched` carries forward, and the node also
    # restores a latch across restarts. Without this, disabling the stop zone
    # on a robot that had already latched would leave it stopped forever with
    # reason "stop_zone" and no obstacle logic still running to release it:
    # the operator sees the switch do nothing, which is the worst way for a
    # safety toggle to fail. The latch is a stop-zone artefact and dies with
    # the stop zone.
    if not limits.estop.stop_zone_enabled:
        new_latched = False
        new_count = 0

    # ------------------------------------------------------------------
    # Precedence 3: STOP ZONE (latched)
    # ------------------------------------------------------------------
    if new_latched:
        return Decision(
            state=State.STOPPED,
            reason="stop_zone",
            out_vx=0.0,
            out_vy=0.0,
            out_wz=0.0,
            clamping_active=False,
            min_range=min_r,
            estop_latched=True,
            consecutive_stop_count=new_count,
        )

    # ------------------------------------------------------------------
    # Precedence 4: COMMAND STALE
    # ------------------------------------------------------------------
    if inputs.command_age_ms > limits.timeouts.command_timeout_ms:
        return Decision(
            state=State.STOPPED,
            reason="command_timeout",
            out_vx=0.0,
            out_vy=0.0,
            out_wz=0.0,
            clamping_active=False,
            min_range=min_r,
            estop_latched=new_latched,
            consecutive_stop_count=new_count,
        )

    # ------------------------------------------------------------------
    # Velocity pipeline (levels 5 and 6)
    # ------------------------------------------------------------------
    clamping_active = False

    # Policy vy check (before clamping so the violation is always surfaced).
    raw_vy = inputs.cmd_vy
    if inputs.is_policy_source:
        raw_vy, vy_violated = policy_vy_check(inputs.cmd_vy, limits)
        if vy_violated:
            clamping_active = True
    else:
        vy_violated = False

    # Velocity clamping.
    vx, vy, wz, vel_clamped = clamp_velocity(
        inputs.cmd_vx, raw_vy, inputs.cmd_wz, limits
    )
    if vel_clamped:
        clamping_active = True

    # Acceleration limiting.
    vx, vy, wz, accel_limited = apply_accel_limit(
        vx, vy, wz,
        inputs.last_vx, inputs.last_vy, inputs.last_wz,
        inputs.dt,
        limits,
    )
    if accel_limited:
        # Bad dt → zeros; those zeros are already in vx/vy/wz.
        clamping_active = True

    # ------------------------------------------------------------------
    # Precedence 5: SLOW ZONE — scale the clamped+limited velocities.
    #
    # Also covers the pre-latch stop-zone window (operator decision,
    # 2026-08-14, see safety_validation.md §2b): a stop-zone reading that
    # hasn't yet reached consecutive_scans_to_trigger falls into "stop" here,
    # not "slow". Without this branch it fell through to NOMINAL and emitted
    # full velocity for (N-1) ticks — a closer obstacle braked *later* than a
    # farther one in the slow zone. slow_scale() already returns 0.0 for
    # min_r <= stop_zone_m, so a pre-latch stop-zone reading is scaled to zero
    # immediately, same effect as SLOW state ramping to zero at the boundary.
    # ------------------------------------------------------------------
    if zone in ("slow", "stop"):
        # WHICH range sets the scale depends on which zone we are in, and
        # getting this backwards would undo the 2026-08-14 pre-latch fix.
        #
        #   zone == "stop"  -> a stop-zone reading, seen by the WIDE arcs and
        #                      not yet latched. Scale from min_r, so
        #                      slow_scale() returns 0.0 and the pre-latch
        #                      window brakes immediately. Using min_r_slow
        #                      here would let a stop-zone obstacle that sits
        #                      outside the narrow cone scale to 1.0 and emit
        #                      FULL velocity for the whole pre-latch window.
        #   zone == "slow"  -> scale from min_r_slow, the narrow forward cone.
        #                      That is the entire point of the split: things
        #                      beside the robot no longer throttle it.
        scale = slow_scale(min_r if zone == "stop" else min_r_slow, limits)
        # TRANSLATION is scaled: moving the body through space is what closes
        # the distance to an obstacle, so proximity must throttle it.
        vx *= scale
        vy *= scale
        # YAW IS DELIBERATELY *NOT* SCALED (operator finding, floor test
        # 2026-08-28). Rotating in place does not close the distance to
        # anything: the robot's own measured extent in the scan plane is
        # ~0.26 m (the two chassis standoffs at ~0.25-0.26 m and ~0.11-0.13 m,
        # see safety_limits.yaml self_occlusion_zones_deg), so the circle swept
        # by a pure yaw sits entirely inside the 0.35 m stop zone. There is no
        # clearance to buy by slowing it down.
        #
        # There IS a large cost. Yaw is the only escape manoeuvre this robot
        # has, and scaling it suppressed that manoeuvre exactly when it was
        # needed most: with the scale applied, a commanded spin at min_range
        # 0.50 m took 11.8 s to turn 90 deg, and at 0.40 m took 35 s. On the
        # floor this read as "the robot reacts late to a spin command" — it was
        # not late, it was throttled to a near-standstill by its own proximity.
        # This is the same inversion as the 2026-08-14 "closer obstacle brakes
        # later" fix, in the yaw axis: there, being closer wrongly delayed
        # braking; here, being closer wrongly delayed turning away.
        #
        # Yaw remains bounded by everything else: the max_wz clamp (0.4 rad/s)
        # and max_alpha_z ramp are applied ABOVE this block and are untouched,
        # and the stop-zone LATCH (precedence 4) zeroes all three axes before
        # this branch is ever reached. This block only governs the slow zone
        # and the pre-latch window, where the robot is still permitted to move.
        return Decision(
            state=State.SLOW,
            reason="slow_zone" if zone == "slow" else "stop_zone_prelatch",
            out_vx=vx,
            out_vy=vy,
            out_wz=wz,
            clamping_active=clamping_active,
            min_range=min_r,
            estop_latched=new_latched,
            consecutive_stop_count=new_count,
        )

    # ------------------------------------------------------------------
    # Precedence 6: NOMINAL (OK)
    #
    # The reason carries the zone state, so a run with the e-stop switched off
    # is self-describing on /safety/status, on the dashboard, and in any
    # recorded bag — rather than being a property of a launch argument nobody
    # wrote down. `zones_status_suffix` is "" while the e-stop is whole, so
    # the nominal reason is still exactly "ok" in the normal case.
    # ------------------------------------------------------------------
    return Decision(
        state=State.OK,
        reason="ok" + limits.estop.zones_status_suffix,
        out_vx=vx,
        out_vy=vy,
        out_wz=wz,
        clamping_active=clamping_active,
        min_range=min_r,
        estop_latched=new_latched,
        consecutive_stop_count=new_count,
    )
