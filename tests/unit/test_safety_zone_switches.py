"""Unit tests for the lidar zone master switches (added 2026-08-30).

WHAT THESE PIN, and why each matters:

  * the DEFAULT is enabled, from every direction — a config that omits the
    keys, and a config that sets them true. A safety feature that has to be
    remembered is one that will eventually be forgotten, so "absent" must
    never read as "disabled";
  * disabling a zone removes ONLY that zone. Every clamp, every acceleration
    limit, the kill, and every staleness watchdog must survive, because the
    operator asked to pause proximity braking and nothing else;
  * a disabled stop zone CLEARS a latch it had already set, so the switch
    visibly takes effect on a robot that was already stopped;
  * the reason string names the degraded state, so /safety/status carries it
    continuously rather than it living only in a startup log line.

These run in the workstation tier: safety_logic is pure, so none of this
needs a ROS runtime or a robot.
"""

import math
import os
import sys

import pytest

# Make rm_safety importable without a colcon install, matching
# test_safety_logic.py. The workstation has no ROS runtime; safety_logic is
# pure, so a path insert is all it needs.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_RM_SAFETY_SRC = os.path.join(_REPO_ROOT, "src", "rm_safety")
if _RM_SAFETY_SRC not in sys.path:
    sys.path.insert(0, _RM_SAFETY_SRC)

from rm_safety.safety_logic import (  # noqa: E402
    EvalInputs,
    Limits,
    State,
    estop_zone,
    evaluate,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _limits_dict(**estop_overrides) -> dict:
    """A complete, valid safety_limits.yaml dict, tweakable in the estop block."""
    estop = {
        "slow_zone_m": 0.55,
        "stop_zone_m": 0.15,
        "forward_arc_deg": 90.0,
        "lateral_arc_deg": 60.0,
        "slow_forward_arc_deg": 30.0,
        "slow_lateral_arc_deg": 0.0,
        "consecutive_scans_to_trigger": 3,
        "latching": True,
        "self_occlusion_zones_deg": [],
    }
    estop.update(estop_overrides)
    return {
        "velocity_limits": {
            "max_vx": 0.25, "max_vy": 0.25, "max_wz": 0.8,
            "policy_vy_expected": 0.0,
            "policy_vy_violation_action": "zero_and_warn",
        },
        "acceleration_limits": {
            "max_ax": 0.75, "max_ay": 0.5, "max_alpha_z": 1.0,
        },
        "estop": estop,
        "timeouts": {
            "command_timeout_ms": 300.0,
            "scan_timeout_ms": 500.0,
            "odom_timeout_ms": 500.0,
        },
    }


def _scan_at(distance_m: float, n: int = 180):
    """A full scan, every ray at one distance, angle_min = -pi."""
    return list([distance_m] * n), -math.pi, (2.0 * math.pi) / n


def _inputs(distance_m: float, **kw) -> EvalInputs:
    """Nominal inputs — fresh everything, a modest forward command."""
    ranges, angle_min, angle_inc = _scan_at(distance_m)
    base = dict(
        cmd_vx=0.20, cmd_vy=0.0, cmd_wz=0.0,
        last_vx=0.20, last_vy=0.0, last_wz=0.0,
        dt=0.05,
        scan_ranges=ranges,
        scan_angle_min=angle_min,
        scan_angle_increment=angle_inc,
        # The LaserScan's own validity window. The TG30 reports 0.01/30.0;
        # rays outside it are dropped before the arc minimum is taken.
        scan_range_min=0.01,
        scan_range_max=30.0,
        scan_age_ms=10.0,
        odom_stale=False,
        command_age_ms=10.0,
        is_killed=False,
        is_policy_source=True,
        estop_latched=False,
        consecutive_stop_count=0,
        reset_latch=False,
    )
    base.update(kw)
    return EvalInputs(**base)


# ---------------------------------------------------------------------------
# Defaults — enabled unless explicitly switched off
# ---------------------------------------------------------------------------

def test_keys_absent_means_both_zones_enabled():
    """A config predating these keys gets the FULL e-stop, not a disabled one."""
    d = _limits_dict()
    assert "slow_zone_enabled" not in d["estop"]
    assert "stop_zone_enabled" not in d["estop"]

    limits = Limits.from_dict(d)

    assert limits.estop.slow_zone_enabled is True
    assert limits.estop.stop_zone_enabled is True
    assert limits.estop.any_zone_enabled is True
    assert limits.estop.zones_status_suffix == ""


def test_explicit_true_is_enabled():
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=True, stop_zone_enabled=True)
    )
    assert limits.estop.slow_zone_enabled is True
    assert limits.estop.stop_zone_enabled is True


def test_default_config_still_stops_and_slows():
    """Baseline: with the defaults, nothing about the e-stop has changed."""
    limits = Limits.from_dict(_limits_dict())

    assert estop_zone(0.10, limits, min_range_slow=0.10) == "stop"
    assert estop_zone(0.40, limits, min_range_slow=0.40) == "slow"
    assert estop_zone(5.00, limits, min_range_slow=5.00) == "clear"


# ---------------------------------------------------------------------------
# estop_zone honours the switches
# ---------------------------------------------------------------------------

def test_disabled_stop_zone_never_returns_stop():
    limits = Limits.from_dict(_limits_dict(stop_zone_enabled=False))
    # Well inside the stop radius, and it still is not "stop".
    assert estop_zone(0.01, limits, min_range_slow=0.01) == "slow"


def test_disabled_slow_zone_never_returns_slow():
    limits = Limits.from_dict(_limits_dict(slow_zone_enabled=False))
    assert estop_zone(0.40, limits, min_range_slow=0.40) == "clear"
    # The stop zone is untouched by the slow switch.
    assert estop_zone(0.10, limits, min_range_slow=0.10) == "stop"


def test_both_disabled_is_always_clear():
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    for r in (0.0, 0.01, 0.14, 0.30, 0.54, 5.0):
        assert estop_zone(r, limits, min_range_slow=r) == "clear", r


# ---------------------------------------------------------------------------
# evaluate() — the switches actually let the command through
# ---------------------------------------------------------------------------

def test_obstacle_in_stop_zone_passes_through_when_disabled():
    """THE POINT OF THE FEATURE: an obstacle 0.10 m ahead no longer stops it."""
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(_inputs(0.10), limits)

    assert d.state == State.OK
    assert d.out_vx == pytest.approx(0.20)
    assert d.estop_latched is False
    # min_range is still MEASURED and reported — the dashboard still shows
    # proximity, the layer just does not brake on it.
    assert d.min_range == pytest.approx(0.10, abs=1e-6)


def test_same_obstacle_stops_the_robot_with_the_defaults():
    """The control for the test above: nothing else in the inputs differs."""
    limits = Limits.from_dict(_limits_dict())
    d = evaluate(_inputs(0.10), limits)

    assert d.state != State.OK
    assert d.out_vx == 0.0


def test_slow_zone_disabled_stop_zone_still_latches():
    """Disabling the slow zone alone must not touch the e-stop."""
    limits = Limits.from_dict(_limits_dict(slow_zone_enabled=False))

    # An obstacle in the slow band now passes at full commanded velocity...
    d_slow = evaluate(_inputs(0.40), limits)
    assert d_slow.state == State.OK
    assert d_slow.out_vx == pytest.approx(0.20)

    # ...but the stop zone still fires, and still latches after N scans.
    inp = _inputs(0.10)
    latched = False
    for _ in range(limits.estop.consecutive_scans_to_trigger + 1):
        d = evaluate(inp, limits)
        inp = _inputs(
            0.10,
            estop_latched=d.estop_latched,
            consecutive_stop_count=d.consecutive_stop_count,
        )
        latched = latched or d.estop_latched
    assert latched is True
    assert d.state == State.STOPPED
    assert d.reason == "stop_zone"


# ---------------------------------------------------------------------------
# Everything that must SURVIVE a disabled e-stop
# ---------------------------------------------------------------------------

def test_velocity_clamps_survive_disabled_zones():
    """The operator kept the clamps. An over-range command is still clamped."""
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    # last_* is the SAFETY LAYER'S OWN last output, so it is already clamped
    # by construction — a prior of 2.0 m/s is not a state this node can have
    # been in. Sitting at the clamp means the acceleration limiter has nothing
    # to do and the clamp is the only binding constraint, which is what this
    # test is about.
    d = evaluate(
        _inputs(
            0.05,                      # deep inside the (disabled) stop zone
            cmd_vx=2.0, cmd_wz=3.14,   # the policy's full trained authority
            last_vx=limits.velocity.max_vx,
            last_wz=limits.velocity.max_wz,
        ),
        limits,
    )
    assert d.out_vx == pytest.approx(limits.velocity.max_vx)
    assert d.out_wz == pytest.approx(limits.velocity.max_wz)
    assert d.clamping_active is True


def test_acceleration_limit_survives_disabled_zones():
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(
        _inputs(5.0, cmd_vx=0.25, last_vx=0.0, dt=0.05),
        limits,
    )
    # One 50 ms tick at max_ax 0.75 m/s^2 buys 0.0375 m/s, not 0.25.
    assert d.out_vx == pytest.approx(0.0375, abs=1e-6)
    assert d.clamping_active is True


def test_kill_still_wins_over_disabled_zones():
    """The kill switch is the operator's stop and outranks everything."""
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(_inputs(5.0, is_killed=True), limits)
    assert d.state == State.KILLED
    assert d.reason == "killed"
    assert (d.out_vx, d.out_vy, d.out_wz) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "kw, expected_reason",
    [
        ({"scan_age_ms": 10_000.0}, "scan_stale"),
        ({"odom_stale": True}, "odom_stale"),
        ({"command_age_ms": 10_000.0}, "command_timeout"),
    ],
)
def test_staleness_watchdogs_survive_disabled_zones(kw, expected_reason):
    """A dead sensor or a silent command source still stops the robot.

    scan_stale in particular is deliberately NOT gated on the zones: it is
    message AGE, not zone geometry, and a frozen scan means policy_node is
    building its observation from stale data.
    """
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(_inputs(5.0, **kw), limits)
    assert d.state == State.STOPPED
    assert d.reason == expected_reason
    assert (d.out_vx, d.out_vy, d.out_wz) == (0.0, 0.0, 0.0)


def test_policy_vy_violation_still_zeroed_when_zones_disabled():
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(_inputs(5.0, cmd_vy=0.2, is_policy_source=True), limits)
    assert d.out_vy == 0.0
    assert d.clamping_active is True


# ---------------------------------------------------------------------------
# no_perception is gated; scan staleness is not
# ---------------------------------------------------------------------------

def test_no_perception_does_not_stop_when_both_zones_disabled():
    """An all-no-return scan is not a fault when nothing consumes min_range.

    The TG30 reports 31-39% of rays as no-returns in a normal room, so an
    open space genuinely can leave the forward arc empty. Left ungated this
    would stop the robot for a reason the operator just switched off.
    """
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(_inputs(5.0, scan_ranges=[]), limits)

    assert d.min_range == math.inf
    assert d.state == State.OK
    assert d.reason == "ok_estop_disabled"


def test_no_perception_still_stops_when_one_zone_remains():
    """With ANY zone live, an empty arc is still loss of perception."""
    limits = Limits.from_dict(_limits_dict(slow_zone_enabled=False))
    d = evaluate(_inputs(5.0, scan_ranges=[]), limits)
    assert d.state == State.STOPPED
    assert d.reason == "scan_stale"


def test_scan_staleness_is_never_gated_on_the_zones():
    """Message age is a fault regardless: the POLICY reads this scan too."""
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(_inputs(5.0, scan_age_ms=10_000.0), limits)
    assert d.state == State.STOPPED
    assert d.reason == "scan_stale"


# ---------------------------------------------------------------------------
# A disabled stop zone releases a latch it had already set
# ---------------------------------------------------------------------------

def test_disabling_the_stop_zone_clears_an_existing_latch():
    """Otherwise the switch appears to do nothing on an already-stopped robot.

    The latch survives in EvalInputs and across a node restart, so without
    this the robot stays STOPPED with reason 'stop_zone' and no obstacle
    logic still running to release it.
    """
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    d = evaluate(
        _inputs(
            0.05,
            estop_latched=True,
            consecutive_stop_count=limits.estop.consecutive_scans_to_trigger,
        ),
        limits,
    )
    assert d.estop_latched is False
    assert d.consecutive_stop_count == 0
    assert d.state == State.OK


def test_latch_is_preserved_when_the_stop_zone_is_still_enabled():
    """The control: only a DISABLED stop zone drops the latch."""
    limits = Limits.from_dict(_limits_dict(slow_zone_enabled=False))
    d = evaluate(_inputs(0.05, estop_latched=True), limits)
    assert d.estop_latched is True
    assert d.state == State.STOPPED


# ---------------------------------------------------------------------------
# The reason string names the degraded state
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "overrides, expected_suffix",
    [
        ({}, ""),
        ({"slow_zone_enabled": False}, "_slow_zone_disabled"),
        ({"stop_zone_enabled": False}, "_stop_zone_disabled"),
        (
            {"slow_zone_enabled": False, "stop_zone_enabled": False},
            "_estop_disabled",
        ),
    ],
)
def test_reason_suffix_names_exactly_what_is_off(overrides, expected_suffix):
    limits = Limits.from_dict(_limits_dict(**overrides))
    assert limits.estop.zones_status_suffix == expected_suffix

    d = evaluate(_inputs(5.0), limits)
    assert d.state == State.OK
    assert d.reason == "ok" + expected_suffix


def test_nominal_reason_is_still_exactly_ok_by_default():
    """Nothing about the normal case changes — bags and tooling see 'ok'."""
    limits = Limits.from_dict(_limits_dict())
    assert evaluate(_inputs(5.0), limits).reason == "ok"
