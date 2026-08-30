"""Tests for rm_waypoint.waypoint_controller — the go-to-waypoint control law.

This is a NEW autonomous motion path on a robot with no physical E-stop, so
the tests below are weighted toward the ways it could fail to STOP rather than
toward the ways it could fail to arrive. A controller that does not quite
reach the point is an inconvenience; one that keeps commanding into a wall,
or that converges on paper while standing still, is not.

Pure stdlib: workstation tier, no ROS.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_waypoint"))

from rm_waypoint import waypoint_controller as wc  # noqa: E402


def _limits(**overrides) -> wc.Limits:
    base = dict(
        max_vx=0.15, max_wz=0.6, min_vx=0.05, min_wz=0.35,
        kp_lin=0.5, kp_yaw=1.2, turn_in_place_rad=0.6, arrive_tol_m=0.15,
        max_range_m=6.0, timeout_s=90.0, progress_window_s=6.0,
        progress_min_m=0.05,
    )
    base.update(overrides)
    return wc.Limits(**base)


def _run(robot=(0.0, 0.0, 0.0), target=(2.0, 0.0), elapsed=0.0,
         limits=None, progress=None):
    limits = limits or _limits()
    progress = progress or wc.ProgressWatchdog(
        limits.progress_window_s, limits.progress_min_m
    )
    return wc.compute(
        robot_x=robot[0], robot_y=robot[1], robot_yaw=robot[2],
        target_x=target[0], target_y=target[1],
        elapsed_s=elapsed, limits=limits, progress=progress,
    )


# ---------------------------------------------------------------------------
# Stopping. The half that matters.
# ---------------------------------------------------------------------------


def test_arrival_stops_and_is_terminal() -> None:
    cmd = _run(target=(0.10, 0.0))
    assert cmd.state == wc.ARRIVED
    assert (cmd.vx, cmd.wz) == (0.0, 0.0)
    assert cmd.is_terminal


def test_timeout_stops_even_while_making_progress() -> None:
    """A run that is still closing must still end at the timeout.

    Otherwise a slow approach — or one being scaled down by rm_safety's slow
    zone — could drive indefinitely, which is precisely the unattended
    behaviour the timeout exists to bound.
    """
    cmd = _run(target=(3.0, 0.0), elapsed=91.0)
    assert cmd.state == wc.ABORTED
    assert "timeout" in cmd.reason
    assert (cmd.vx, cmd.wz) == (0.0, 0.0)


def test_nonfinite_pose_stops_rather_than_steering_on_it() -> None:
    for bad in (float("nan"), float("inf")):
        cmd = _run(robot=(bad, 0.0, 0.0))
        assert cmd.state == wc.ABORTED
        assert (cmd.vx, cmd.wz) == (0.0, 0.0)


def test_nonfinite_target_stops() -> None:
    cmd = _run(target=(float("nan"), 0.0))
    assert cmd.state == wc.ABORTED
    assert (cmd.vx, cmd.wz) == (0.0, 0.0)


def test_every_terminal_state_commands_zero() -> None:
    """No terminal path may emit motion, whatever the reason.

    Pinned as a property rather than case by case: a future give-up condition
    added without a test would still have to come through `stop()`.
    """
    for state in (wc.IDLE, wc.ARRIVED, wc.ABORTED):
        cmd = wc.stop(state, "whatever")
        assert (cmd.vx, cmd.wz) == (0.0, 0.0)
        assert cmd.is_terminal


def test_driving_and_turning_are_not_terminal() -> None:
    assert not wc.Command(0.1, 0.0, wc.DRIVING, "", 1.0, 0.0).is_terminal
    assert not wc.Command(0.0, 0.4, wc.TURNING, "", 1.0, 1.0).is_terminal


# ---------------------------------------------------------------------------
# The progress watchdog — the obstacle case
# ---------------------------------------------------------------------------


def test_stalled_range_aborts_after_the_window() -> None:
    """rm_safety zeroing the command must end the run, not be commanded into.

    The controller cannot see the stop zone. What it sees is a range that has
    stopped falling, and that has to be enough — otherwise it sits at full
    command against a latched e-stop until the timeout.
    """
    limits = _limits()
    progress = wc.ProgressWatchdog(limits.progress_window_s, limits.progress_min_m)
    # Held at exactly the same range for longer than the window.
    last = None
    for t in [0.0, 1.0, 3.0, 5.0, 6.5, 7.0]:
        last = _run(target=(1.0, 0.0), elapsed=t, limits=limits, progress=progress)
    assert last.state == wc.ABORTED
    assert "no progress" in last.reason


def test_slow_but_real_progress_does_not_trip_the_watchdog() -> None:
    """A crawl is not a stall.

    The reference re-arms on real progress, so an approach slowed by the slow
    zone — legitimately, and for a long time — must not be mistaken for being
    stuck.
    """
    progress = wc.ProgressWatchdog(6.0, 0.05)
    reason = None
    # Closing 0.06 m every 5 s: under the window each time, but always real.
    for i in range(12):
        reason = progress.update(3.0 - i * 0.06, i * 5.0)
        assert reason is None, f"tripped at sample {i}"


def test_watchdog_measures_from_the_best_range_not_the_last() -> None:
    """Drifting backwards must not re-arm the watchdog.

    If the reference tracked the LAST range rather than the BEST, a robot
    being pushed slowly backwards would look like it was making progress in
    the wrong direction forever and never abort.
    """
    progress = wc.ProgressWatchdog(6.0, 0.05)
    assert progress.update(2.0, 0.0) is None
    for t, r in [(1.0, 2.1), (3.0, 2.2), (5.0, 2.3)]:
        assert progress.update(r, t) is None
    assert progress.update(2.4, 7.0) is not None


# ---------------------------------------------------------------------------
# The dead-band floor. The reason this is not a textbook P controller.
# ---------------------------------------------------------------------------


def test_small_turn_commands_are_collapsed_to_zero_not_emitted() -> None:
    """Below min_wz the robot does not turn, so do not claim to be turning.

    This robot loses roughly the first 0.107 rad/s of a commanded spin to
    mecanum scrub (measured 2026-08-29 s3). A command under that heats the
    motors and moves nothing.
    """
    assert wc._shape(0.05, 0.35, 0.6) == 0.0
    assert wc._shape(-0.05, 0.35, 0.6) == 0.0


def test_shape_preserves_sign_and_clamps_to_the_ceiling() -> None:
    assert wc._shape(5.0, 0.05, 0.15) == pytest.approx(0.15)
    assert wc._shape(-5.0, 0.05, 0.15) == pytest.approx(-0.15)


def test_a_turn_at_the_threshold_clears_the_floor() -> None:
    """The config must not produce a TURNING state that commands zero.

    That combination is a robot which sits still while reporting that it is
    turning, and then aborts on the progress watchdog with a reason that
    points at obstacles. check_limits() rejects it; this pins that the shipped
    numbers satisfy it.
    """
    limits = _shipped_limits()
    assert limits.kp_yaw * limits.turn_in_place_rad >= limits.min_wz
    cmd = _run(robot=(0.0, 0.0, 0.0), target=(0.0, 1.0), limits=limits)
    assert cmd.state == wc.TURNING
    assert abs(cmd.wz) >= limits.min_wz


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def test_turns_toward_the_target_the_short_way_round() -> None:
    """A target just behind-left must turn LEFT, not 350 degrees right."""
    # Robot facing +x; target at bearing +3.0 rad (behind and to the left).
    target = (math.cos(3.0), math.sin(3.0))
    cmd = _run(target=target)
    assert cmd.state == wc.TURNING
    assert cmd.wz > 0.0

    # And the mirror image turns the other way.
    target = (math.cos(-3.0), math.sin(-3.0))
    cmd = _run(target=target)
    assert cmd.wz < 0.0


def test_heading_error_wraps_across_pi() -> None:
    """A yaw near +-pi must not read as a six-radian error.

    Unwrapped, a robot pointing south would compute enormous heading errors
    and spin the long way round every time.
    """
    # Robot facing -pi + 0.05; target dead ahead of it.
    yaw = -math.pi + 0.05
    target = (math.cos(yaw) * 2.0, math.sin(yaw) * 2.0)
    cmd = _run(robot=(0.0, 0.0, yaw), target=target)
    assert abs(cmd.heading_error_rad) < 1e-6
    assert cmd.state == wc.DRIVING


def test_drives_forward_when_already_facing_the_target() -> None:
    cmd = _run(target=(2.0, 0.0))
    assert cmd.state == wc.DRIVING
    assert cmd.vx > 0.0
    assert cmd.wz == pytest.approx(0.0)


def test_forward_speed_never_exceeds_max_vx() -> None:
    """Far away, the proportional term is large; the clamp is what holds.

    max_vx here is already below rm_safety's, so this controller should never
    be the thing rm_safety is clamping.
    """
    limits = _limits()
    cmd = _run(target=(5.9, 0.0), limits=limits)
    assert cmd.vx <= limits.max_vx + 1e-9


def test_never_commands_reverse() -> None:
    """This controller only ever drives forward; it turns to face instead.

    Reverse matters because rm_safety's e-stop arcs are FORWARD and lateral —
    there is no rear arc, so nothing downstream would stop the robot backing
    into something.
    """
    for bearing in [0.0, 0.3, 0.59, 1.5, 3.0, -2.0]:
        cmd = _run(target=(2 * math.cos(bearing), 2 * math.sin(bearing)))
        assert cmd.vx >= 0.0, f"reverse commanded at bearing {bearing}"


def test_steering_slows_forward_speed_rather_than_curving_wide() -> None:
    # Close enough that the proportional term is BELOW max_vx. With kp_lin 0.5
    # against a 0.15 m/s ceiling the law saturates beyond 0.30 m, so a target
    # at 2 m would give 0.15 in both cases and the comparison would be
    # vacuous — the assert below pins that the fixture actually tests the
    # taper rather than the clamp.
    straight = _run(target=(0.25, 0.0))
    angled = _run(target=(0.25 * math.cos(0.5), 0.25 * math.sin(0.5)))
    assert angled.state == wc.DRIVING
    assert straight.vx < _limits().max_vx, "test target must not saturate"
    assert angled.vx < straight.vx


# ---------------------------------------------------------------------------
# Acceptance checks
# ---------------------------------------------------------------------------


def test_a_distant_waypoint_is_refused() -> None:
    """The only guard between a mis-click and a drive across the building."""
    refusal = wc.validate_target(0.0, 0.0, 40.0, 0.0, _limits())
    assert refusal is not None
    assert "40.00 m" in refusal


def test_a_reachable_waypoint_is_accepted() -> None:
    assert wc.validate_target(0.0, 0.0, 1.5, 0.5, _limits()) is None


def test_nonfinite_waypoints_are_refused() -> None:
    assert wc.validate_target(0.0, 0.0, float("nan"), 0.0, _limits()) is not None
    assert wc.validate_target(0.0, 0.0, 1.0, float("inf"), _limits()) is not None
    assert wc.validate_target(float("nan"), 0.0, 1.0, 0.0, _limits()) is not None


# ---------------------------------------------------------------------------
# Config loading — a missing limit must not be silently defaulted
# ---------------------------------------------------------------------------


def _shipped_config() -> dict:
    import yaml
    with open(os.path.join(_REPO_ROOT, "config", "waypoint.yaml")) as handle:
        return yaml.safe_load(handle)["waypoint"]


def _shipped_limits() -> wc.Limits:
    limits, error = wc.limits_from_dict(_shipped_config())
    assert error is None, error
    return limits


def test_the_shipped_config_loads_and_passes_its_own_checks() -> None:
    """config/waypoint.yaml must be internally consistent, in the repo.

    This is the test that would have caught the `closed_loop.guards` class of
    bug one package over: a config key that quietly disagrees with the code.
    """
    limits, error = wc.limits_from_dict(_shipped_config())
    assert error is None, error
    assert wc.check_limits(limits) is None


def test_a_missing_key_is_an_error_not_a_default() -> None:
    cfg = _shipped_config()
    del cfg["max_vx"]
    _, error = wc.limits_from_dict(cfg)
    assert error is not None
    assert "max_vx" in error


def test_a_non_numeric_limit_is_refused() -> None:
    cfg = _shipped_config()
    cfg["max_vx"] = "fast"
    _, error = wc.limits_from_dict(cfg)
    assert error is not None


def test_the_shipped_limits_stay_under_the_safety_clamps() -> None:
    """rm_safety must always be the binding constraint, not this controller.

    If a waypoint run is being clamped by the safety layer, something here is
    wrong and should be visible rather than normal.
    """
    import yaml
    with open(os.path.join(_REPO_ROOT, "config", "safety_limits.yaml")) as handle:
        safety = yaml.safe_load(handle)
    limits = _shipped_limits()
    clamps = safety["velocity_limits"]
    assert limits.max_vx < float(clamps["max_vx"])
    assert limits.max_wz < float(clamps["max_wz"])


@pytest.mark.parametrize("overrides,fragment", [
    ({"min_vx": 0.5, "max_vx": 0.15}, "min_vx"),
    ({"min_wz": 1.0, "max_wz": 0.6}, "min_wz"),
    ({"arrive_tol_m": 7.0}, "arrive_tol_m"),
    ({"kp_yaw": 0.1}, "min_wz"),
    ({"progress_min_m": 0.0}, "progress"),
])
def test_incoherent_limits_are_rejected(overrides, fragment) -> None:
    error = wc.check_limits(_limits(**overrides))
    assert error is not None
    assert fragment in error


# ---------------------------------------------------------------------------
# The module's state ints must equal the message's constants
# ---------------------------------------------------------------------------


def test_state_constants_match_the_message_definition() -> None:
    """`waypoint_controller` keeps plain ints so it imports no ROS.

    That is only safe while the two definitions agree. If they drift, the node
    maps a DRIVING command onto an ARRIVED status and the dashboard reports a
    moving robot as stopped — a wrong readout on the one panel an operator
    watches while the robot is driving. Parsed from the .msg text rather than
    imported, so this runs in the workstation tier with no ROS.
    """
    path = os.path.join(
        _REPO_ROOT, "src", "rm_interfaces", "msg", "WaypointStatus.msg"
    )
    with open(path) as handle:
        lines = handle.read().splitlines()

    declared = {}
    for line in lines:
        line = line.split("#", 1)[0].strip()
        if line.startswith("uint8 ") and "=" in line:
            name, value = line[len("uint8 "):].split("=", 1)
            declared[name.strip()] = int(value)

    assert declared == {
        "IDLE": wc.IDLE,
        "TURNING": wc.TURNING,
        "DRIVING": wc.DRIVING,
        "ARRIVED": wc.ARRIVED,
        "ABORTED": wc.ABORTED,
    }
