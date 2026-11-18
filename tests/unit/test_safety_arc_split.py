"""Slow-zone arcs, separate from the stop zone's (2026-08-29).

WHY THE SPLIT EXISTS: one arc pair used to serve both zones, so a wall or a
chair BESIDE the robot scaled forward velocity down even with a clear path
ahead. Narrowing the shared arc would have fixed that by also narrowing the
STOP zone — trading a comfort problem for a real loss of protection.

So the property under test is not "the slow zone got narrower". It is that the
slow zone got narrower AND THE STOP ZONE DID NOT. Every test here is paired:
whatever stops throttling the robot must still be able to stop it.

Pure stdlib + pytest, no ROS. Workstation tier.
"""

from __future__ import annotations

import math
import os
import sys
from typing import List

import pytest
import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_safety"))

from rm_safety.safety_logic import (  # noqa: E402
    Limits,
    estop_zone,
    min_range_in_arcs,
)

_CONFIG_PATH = os.path.join(_REPO_ROOT, "config", "safety_limits.yaml")
_PROFILE_PATH = os.path.join(_REPO_ROOT, "config", "robots", "bernard-1.yaml")


def _raw() -> dict:
    with open(_CONFIG_PATH, "r") as fh:
        return yaml.safe_load(fh)


def _mount() -> dict:
    """bernard-1's lidar block: the zones are clearances beyond its min_range."""
    with open(_PROFILE_PATH, "r") as fh:
        return yaml.safe_load(fh)["lidar"]


@pytest.fixture
def raw() -> dict:
    return _raw()


@pytest.fixture
def limits(raw) -> Limits:
    return Limits.from_dict(raw, _mount())


# One ray per degree over the full circle, all clear at 5 m.
_AINC = math.radians(1.0)
_AMIN = -math.pi


def _scan_with(bearing_deg: float, range_m: float, n: int = 360) -> List[float]:
    """A clear 5 m scan with one return placed at `bearing_deg`."""
    ranges = [5.0] * n
    idx = int(round((math.radians(bearing_deg) - _AMIN) / _AINC))
    ranges[idx % n] = range_m
    return ranges


def _mins(limits: Limits, ranges: List[float]):
    """(wide stop-arc minimum, narrow slow-arc minimum) for a scan."""
    wide = min_range_in_arcs(ranges, _AMIN, _AINC, limits)
    narrow = min_range_in_arcs(
        ranges, _AMIN, _AINC, limits,
        forward_arc_deg=limits.estop.effective_slow_forward_arc_deg,
        lateral_arc_deg=limits.estop.effective_slow_lateral_arc_deg,
    )
    return wide, narrow


def _zone(limits: Limits, ranges: List[float]) -> str:
    wide, narrow = _mins(limits, ranges)
    return estop_zone(wide, limits, min_range_slow=narrow)


# ---------------------------------------------------------------------------
# The configured geometry is the one that was intended
# ---------------------------------------------------------------------------

def test_shipped_config_has_a_narrow_slow_cone_and_a_wide_stop_arc(limits) -> None:
    e = limits.estop
    assert e.effective_slow_forward_arc_deg == 30.0
    assert e.effective_slow_lateral_arc_deg == 0.0
    assert e.forward_arc_deg == 90.0
    assert e.lateral_arc_deg == 60.0
    assert e.slow_arcs_differ is True


# The robot's half-extent in the lidar's scan plane, metres. Derived from the
# MEASURED ~0.26 m chassis extent recorded in safety_limits.yaml's
# self_occlusion_zones_deg note (two standoffs at ~0.25-0.26 m and
# ~0.11-0.13 m). Not a spec-sheet number and not a guess.
_ROBOT_HALF_EXTENT_M = 0.13


def test_slow_cone_covers_the_robot_width_at_the_slow_radius(limits) -> None:
    """The sizing criterion, pinned so a later edit has to face it.

    A slow zone should react to what the robot is about to drive INTO, so at
    the slow radius the half-cone must subtend at least the robot's half-width.

    THIS COUPLES THE CONE ANGLE TO THE SLOW RADIUS. Shrinking slow_zone_m
    shrinks what a fixed angle covers, so reducing the radius without widening
    the cone eventually stops covering the robot's own width. That is not
    obvious from either value alone, which is why it is a test and not a
    comment.
    """
    e = limits.estop
    half = math.radians(e.effective_slow_forward_arc_deg / 2.0)
    lateral_cover_m = e.slow_zone_m * math.tan(half)
    assert lateral_cover_m >= _ROBOT_HALF_EXTENT_M, (
        f"slow cone covers only +-{lateral_cover_m:.3f} m at "
        f"{e.slow_zone_m} m — narrower than the robot's "
        f"{_ROBOT_HALF_EXTENT_M} m half-extent. Widen "
        f"slow_forward_arc_deg or raise slow_zone_m."
    )


def test_the_same_cone_would_be_too_narrow_for_the_stop_zone(limits) -> None:
    """Documents WHY the arcs are split rather than the shared arc narrowed.

    At the stop radius, +-15 deg covers less than the robot's half-width. This
    is the test that should fail loudly if someone later "simplifies" the two
    arcs back into one by copying the slow value over the stop value.
    """
    e = limits.estop
    half = math.radians(e.effective_slow_forward_arc_deg / 2.0)
    lateral_cover_m = e.stop_zone_m * math.tan(half)
    assert lateral_cover_m < _ROBOT_HALF_EXTENT_M, (
        "the slow cone is now wide enough for the stop radius too; if that is "
        "genuinely intended, re-derive the stop arc rather than inheriting it"
    )


# ---------------------------------------------------------------------------
# The point of the change: side obstacles stop throttling forward motion
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("bearing", [40.0, -40.0, 90.0, -90.0, 75.0])
def test_side_obstacle_in_the_slow_band_no_longer_slows_the_robot(
    limits, bearing
) -> None:
    ranges = _scan_with(bearing, 0.50)  # inside 0.65 slow, outside 0.35 stop
    assert _zone(limits, ranges) == "clear"


@pytest.mark.parametrize("bearing", [40.0, -40.0, 90.0, -90.0])
def test_but_the_same_side_obstacle_inside_the_stop_zone_STILL_STOPS(
    limits, bearing
) -> None:
    """The paired half. Narrowing the slow cone must not have narrowed this."""
    # Derived from config, not hardcoded: stop_zone_m has moved twice.
    ranges = _scan_with(bearing, limits.estop.stop_zone_m * 0.8)
    assert _zone(limits, ranges) == "stop"


def test_forward_obstacle_in_the_slow_band_still_slows(limits) -> None:
    ranges = _scan_with(0.0, 0.50)
    assert _zone(limits, ranges) == "slow"


@pytest.mark.parametrize("bearing", [0.0, 10.0, -10.0, 14.0, -14.0])
def test_anything_inside_the_cone_still_slows(limits, bearing) -> None:
    ranges = _scan_with(bearing, 0.50)
    assert _zone(limits, ranges) == "slow"


@pytest.mark.parametrize("bearing", [16.0, -16.0, 20.0, 30.0])
def test_just_outside_the_cone_does_not_slow(limits, bearing) -> None:
    ranges = _scan_with(bearing, 0.50)
    assert _zone(limits, ranges) == "clear"


def test_the_cone_edge_is_where_the_config_says_it_is(limits) -> None:
    """The boundary sits between 14 and 16 deg, as +-15 implies.

    Deliberately NOT asserting on exactly 15.000 deg. A ray reconstructed as
    angle_min + i*angle_increment lands on the boundary only up to floating
    point, so an exact-boundary assertion tests the rounding, not the
    geometry. The real sensor has 0.178 deg of angular quantization anyway, so
    a half-degree either way is not a distinction the hardware can make.
    """
    assert _zone(limits, _scan_with(14.0, 0.50)) == "slow"
    assert _zone(limits, _scan_with(16.0, 0.50)) == "clear"


# ---------------------------------------------------------------------------
# min_range reporting: the wide arc is still what gets published
# ---------------------------------------------------------------------------

def test_reported_min_range_still_reflects_the_wide_arc(limits) -> None:
    """/safety/status must keep reporting the safety-relevant minimum.

    If a side obstacle vanished from the reported min_range, the dashboard
    would show a clear robot next to a wall it can actually see.
    """
    ranges = _scan_with(45.0, 0.42)
    wide, narrow = _mins(limits, ranges)
    assert wide == pytest.approx(0.42)
    # The cone sees only the clear 5 m filler — the 0.42 m return at 45 deg is
    # outside it. (Not inf: the cone is full of valid, distant returns. "Not
    # seen" here means "does not set the minimum", not "no data".)
    assert narrow == pytest.approx(5.0)
    assert _zone(limits, ranges) == "clear"  # so it does not throttle


def test_slow_arc_minimum_is_never_below_the_wide_arc_minimum(limits) -> None:
    """The subset invariant the zone logic depends on, checked over a sweep."""
    for bearing in range(-180, 180, 7):
        wide, narrow = _mins(limits, _scan_with(float(bearing), 0.45))
        assert narrow >= wide, f"subset invariant broken at {bearing} deg"


# ---------------------------------------------------------------------------
# Backward compatibility
# ---------------------------------------------------------------------------

def test_a_config_without_the_new_keys_behaves_exactly_as_before(raw) -> None:
    """Every config written before 2026-08-29 must be bit-identical."""
    del raw["estop"]["slow_forward_arc_deg"]
    del raw["estop"]["slow_lateral_arc_deg"]
    old = Limits.from_dict(raw, _mount())
    assert old.estop.slow_arcs_differ is False
    assert old.estop.effective_slow_forward_arc_deg == old.estop.forward_arc_deg
    assert old.estop.effective_slow_lateral_arc_deg == old.estop.lateral_arc_deg
    # And the old wide-arc behaviour is back: a side obstacle slows again.
    assert _zone(old, _scan_with(45.0, 0.50)) == "slow"


def test_explicit_null_means_inherit(raw) -> None:
    raw["estop"]["slow_forward_arc_deg"] = None
    raw["estop"]["slow_lateral_arc_deg"] = None
    lim = Limits.from_dict(raw, _mount())
    assert lim.estop.slow_arcs_differ is False


def test_estop_zone_without_a_slow_minimum_is_the_old_single_arc_behaviour(
    limits,
) -> None:
    """The legacy call signature must not silently change meaning."""
    e = limits.estop
    assert estop_zone((e.stop_zone_m + e.slow_zone_m) / 2, limits) == "slow"
    assert estop_zone(e.stop_zone_m * 0.8, limits) == "stop"
    assert estop_zone(e.slow_zone_m * 2.0, limits) == "clear"


# ---------------------------------------------------------------------------
# Misconfiguration must be fatal, not quietly corrected
# ---------------------------------------------------------------------------

def test_a_slow_arc_wider_than_the_stop_arc_is_refused(raw) -> None:
    """A slow zone with no stop zone behind it inverts the layering."""
    raw["estop"]["slow_forward_arc_deg"] = 120.0  # > forward_arc_deg 90
    with pytest.raises(ValueError, match="slow_forward_arc_deg"):
        Limits.from_dict(raw, _mount())


def test_a_wider_slow_lateral_arc_is_refused(raw) -> None:
    raw["estop"]["slow_lateral_arc_deg"] = 90.0  # > lateral_arc_deg 60
    with pytest.raises(ValueError, match="slow_lateral_arc_deg"):
        Limits.from_dict(raw, _mount())


def test_equal_arcs_are_allowed(raw) -> None:
    raw["estop"]["slow_forward_arc_deg"] = raw["estop"]["forward_arc_deg"]
    raw["estop"]["slow_lateral_arc_deg"] = raw["estop"]["lateral_arc_deg"]
    lim = Limits.from_dict(raw, _mount())
    assert lim.estop.slow_arcs_differ is False


# ---------------------------------------------------------------------------
# Zero lateral arc means DISABLED, not a hairline
# ---------------------------------------------------------------------------

def test_zero_lateral_arc_disables_lateral_monitoring_entirely(limits) -> None:
    """A ray landing exactly on 90.000 deg must not sneak through.

    With `abs(angle - pi/2) <= 0` a ray at precisely +-90 deg would match, so
    the slow zone would fire depending on where the lidar's quantization
    happened to land — a non-deterministic trigger that would be miserable to
    diagnose.
    """
    ranges = _scan_with(90.0, 0.40)   # exactly on the lateral centre
    narrow = min_range_in_arcs(
        ranges, _AMIN, _AINC, limits,
        forward_arc_deg=limits.estop.effective_slow_forward_arc_deg,
        lateral_arc_deg=0.0,
    )
    # 5.0, not 0.40: the cone still reports its own clear returns; what must
    # NOT happen is the 0.40 m ray at +-90 deg setting the minimum.
    assert narrow == pytest.approx(5.0)

    # And with lateral monitoring ENABLED it is picked up, which proves the
    # test is actually exercising the disable rather than a missing ray.
    with_lateral = min_range_in_arcs(
        ranges, _AMIN, _AINC, limits,
        forward_arc_deg=limits.estop.effective_slow_forward_arc_deg,
        lateral_arc_deg=60.0,
    )
    assert with_lateral == pytest.approx(0.40)


# ---------------------------------------------------------------------------
# The slow radius is sized by the stopping sum, not chosen
# ---------------------------------------------------------------------------

def test_slow_band_covers_the_stopping_distance_with_margin(limits) -> None:
    """slow_zone_m - stop_zone_m must exceed how far the robot travels between
    first seeing an obstacle and being stopped.

    Two contributions, both real and both measured on this robot:
      braking       v^2 / (2 * max_ax)
      filter travel max_vx * consecutive_scans_to_trigger / scan_rate

    This is the sum that got slow_zone_m reverted from 0.50 back to 0.65 on
    2026-08-29 and then set to 0.55. It is a test so that raising max_vx, or
    lowering this radius, cannot quietly eat the margin — those three numbers
    are coupled and nothing else enforces it.
    """
    e, v, a = limits.estop, limits.velocity, limits.acceleration
    scan_hz = 10.0  # measured TG30 rate, interface.yaml control.measured_source_rates_hz
    braking_m = v.max_vx ** 2 / (2.0 * a.max_ax)
    filter_m = v.max_vx * e.consecutive_scans_to_trigger / scan_hz
    needed_m = braking_m + filter_m
    band_m = e.slow_zone_m - e.stop_zone_m
    assert band_m > needed_m, (
        f"slow band {band_m:.3f} m does not cover {needed_m:.3f} m of "
        f"stopping distance at max_vx {v.max_vx}"
    )
    assert band_m / needed_m >= 1.5, (
        f"stopping margin is only {band_m / needed_m:.2f}x. 1.29x (slow_zone "
        f"0.50) was judged too thin on 2026-08-29; do not go back there "
        f"without lowering max_vx too."
    )
