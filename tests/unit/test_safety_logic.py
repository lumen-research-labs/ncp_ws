"""
test_safety_logic.py — Unit tests for rm_safety/safety_logic.py.

No ROS runtime required.  Pure Python / pytest.

Fixture strategy:
    ``real_limits`` loads ``config/safety_limits.yaml`` via
    ``Limits.from_dict`` so tests always track the actual configured values.
    Inline comments quote the expected numeric constants so failures are
    immediately readable; those values are NOT literals in the logic.

Boundary inclusivity (documented in safety_logic.py, tested explicitly here):
    - stop_zone_m boundary: ``range < stop_zone_m`` → "stop"
      (exactly equal to stop_zone_m is "slow", not "stop")
    - slow_zone_m boundary: ``range < slow_zone_m`` → "slow"
      (exactly equal to slow_zone_m is "clear")
"""

from __future__ import annotations

import math
import os
import sys
from typing import List

import pytest
import yaml

# ---------------------------------------------------------------------------
# Make the rm_safety package importable without a colcon install.
# Adjust path so that ``from rm_safety.safety_logic import ...`` works when
# pytest is run from the repo root or from tests/unit/.
# ---------------------------------------------------------------------------
_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
_RM_SAFETY_SRC = os.path.join(_REPO_ROOT, "src", "rm_safety")
if _RM_SAFETY_SRC not in sys.path:
    sys.path.insert(0, _RM_SAFETY_SRC)

from rm_safety.safety_logic import (  # noqa: E402
    Limits,
    State,
    EvalInputs,
    Decision,
    apply_accel_limit,
    clamp_velocity,
    estop_zone,
    evaluate,
    min_range_in_arcs,
    policy_vy_check,
    slow_scale,
    update_stop_counter,
)

# ---------------------------------------------------------------------------
# Config path
# ---------------------------------------------------------------------------
_CONFIG_PATH = os.path.join(_REPO_ROOT, "config", "safety_limits.yaml")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def real_limits() -> Limits:
    """Load Limits from the actual config/safety_limits.yaml."""
    with open(_CONFIG_PATH, "r") as fh:
        raw = yaml.safe_load(fh)
    # The mount facts (zones, body floor) are the robot's: bernard-1's profile.
    with open(os.path.join(os.path.dirname(_CONFIG_PATH), "robots", "bernard-1.yaml"), "r") as fh:
        mount = yaml.safe_load(fh)["lidar"]
    return Limits.from_dict(raw, mount)


def _b1_mount() -> dict:
    """bernard-1's lidar block, for tests that rebuild limits from a tweaked file."""
    with open(os.path.join(_REPO_ROOT, "config", "robots", "bernard-1.yaml"), "r") as fh:
        return yaml.safe_load(fh)["lidar"]


def _make_inputs(
    limits: Limits,
    *,
    cmd_vx: float = 0.0,
    cmd_vy: float = 0.0,
    cmd_wz: float = 0.0,
    is_policy_source: bool = True,
    command_age_ms: float = 0.0,
    last_vx: float = 0.0,
    last_vy: float = 0.0,
    last_wz: float = 0.0,
    dt: float = 0.1,
    scan_ranges: List[float] | None = None,
    scan_angle_min: float = -math.pi,
    scan_angle_increment: float = math.radians(1.0),
    scan_range_min: float = 0.15,
    scan_range_max: float = 12.0,
    scan_age_ms: float = 0.0,
    is_killed: bool = False,
    estop_latched: bool = False,
    consecutive_stop_count: int = 0,
    reset_latch: bool = False,
    odom_stale: bool = False,
    retreat_clear_count: int = 0,
) -> EvalInputs:
    """Helper: build an EvalInputs with a clear 360-deg scan by default."""
    if scan_ranges is None:
        # 360 rays at 1°, all reporting 5.0 m (well clear of all zones).
        n = 360
        scan_ranges = [5.0] * n
        scan_angle_min = -math.pi
        scan_angle_increment = math.radians(1.0)
    return EvalInputs(
        cmd_vx=cmd_vx,
        cmd_vy=cmd_vy,
        cmd_wz=cmd_wz,
        is_policy_source=is_policy_source,
        command_age_ms=command_age_ms,
        last_vx=last_vx,
        last_vy=last_vy,
        last_wz=last_wz,
        dt=dt,
        scan_ranges=scan_ranges,
        scan_angle_min=scan_angle_min,
        scan_angle_increment=scan_angle_increment,
        scan_range_min=scan_range_min,
        scan_range_max=scan_range_max,
        scan_age_ms=scan_age_ms,
        is_killed=is_killed,
        estop_latched=estop_latched,
        consecutive_stop_count=consecutive_stop_count,
        reset_latch=reset_latch,
        odom_stale=odom_stale,
        retreat_clear_count=retreat_clear_count,
    )


# ===========================================================================
# Tests: Limits.from_dict
# ===========================================================================

class TestLimitsFromDict:
    def test_loads_velocity_limits(self, real_limits: Limits) -> None:
        # max_vx 0.15 -> 0.25 and max_wz 0.4 -> 0.8 on 2026-08-29, to make
        # the NCP debug test run at a watchable pace. Ahead of the Phase 2
        # hardware sign-off; slow_zone_m was widened to 0.65 to pay for the
        # longer stopping distance.
        assert real_limits.velocity.max_vx == pytest.approx(0.25)
        assert real_limits.velocity.max_vy == pytest.approx(0.25)
        assert real_limits.velocity.max_wz == pytest.approx(0.8)

    def test_loads_acceleration_limits(self, real_limits: Limits) -> None:
        assert real_limits.acceleration.max_ax == pytest.approx(0.75)
        assert real_limits.acceleration.max_ay == pytest.approx(0.5)
        assert real_limits.acceleration.max_alpha_z == pytest.approx(1.0)

    def test_loads_estop(self, real_limits: Limits) -> None:
        # 0.80 -> 0.50 -> 0.65 on 2026-08-29: down to sit under the debug
        # policy's new 0.9 m obstacle_dist, then back out to buy stopping
        # room when max_vx was raised to 0.25.
        assert real_limits.estop.slow_zone_m == pytest.approx(0.55)
        assert real_limits.estop.stop_zone_m == pytest.approx(0.15)
        assert real_limits.estop.consecutive_scans_to_trigger == 3

    def test_stop_zone_is_inside_slow_zone(self, real_limits: Limits) -> None:
        """The ordering invariant, independent of the tuned values.

        slow_scale() divides by (slow_zone_m - stop_zone_m). Equal thresholds
        divide by zero; inverted ones would scale velocity UP as an obstacle
        gets closer. Neither is caught by the value assertions above, and both
        are easy to introduce while retuning.
        """
        assert real_limits.estop.stop_zone_m > 0.0
        assert real_limits.estop.slow_zone_m > real_limits.estop.stop_zone_m

    def test_loads_timeouts(self, real_limits: Limits) -> None:
        assert real_limits.timeouts.command_timeout_ms == pytest.approx(300.0)
        assert real_limits.timeouts.scan_timeout_ms == pytest.approx(500.0)

    def test_missing_key_raises(self) -> None:
        with pytest.raises(KeyError):
            Limits.from_dict({})


# ===========================================================================
# Tests: clamp_velocity
# ===========================================================================

class TestClampVelocity:
    def test_in_range_passes_unchanged(self, real_limits: Limits) -> None:
        # max_vx=0.25, max_vy=0.25, max_wz=0.8
        vx, vy, wz, clamped = clamp_velocity(0.1, 0.1, 0.2, real_limits)
        assert vx == pytest.approx(0.1)
        assert vy == pytest.approx(0.1)
        assert wz == pytest.approx(0.2)
        assert clamped is False

    def test_zero_passes_unchanged(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(0.0, 0.0, 0.0, real_limits)
        assert vx == pytest.approx(0.0)
        assert vy == pytest.approx(0.0)
        assert wz == pytest.approx(0.0)
        assert clamped is False

    def test_over_range_clamped_positive(self, real_limits: Limits) -> None:
        # 0.5 > max_vx=0.25
        vx, vy, wz, clamped = clamp_velocity(0.5, 0.0, 0.0, real_limits)
        assert vx == pytest.approx(real_limits.velocity.max_vx)
        assert clamped is True

    def test_over_range_clamped_negative(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(-0.5, 0.0, 0.0, real_limits)
        assert vx == pytest.approx(-real_limits.velocity.max_vx)
        assert clamped is True

    def test_nan_vx_becomes_zero_and_clamped(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(math.nan, 0.0, 0.0, real_limits)
        assert vx == pytest.approx(0.0)
        assert clamped is True

    def test_inf_vy_becomes_zero_and_clamped(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(0.0, math.inf, 0.0, real_limits)
        assert vy == pytest.approx(0.0)
        assert clamped is True

    def test_neg_inf_wz_becomes_zero_and_clamped(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(0.0, 0.0, -math.inf, real_limits)
        assert wz == pytest.approx(0.0)
        assert clamped is True

    def test_all_axes_scaled_by_the_single_tightest_factor(self, real_limits: Limits) -> None:
        """One factor for the whole command: the axis furthest over its limit
        sets it, and the others shrink with it (uniform scaling, 2026-10-09)."""
        lim = real_limits.velocity
        vx, vy, wz, clamped = clamp_velocity(1.0, 1.0, 2.0, real_limits)
        s = min(lim.max_vx / 1.0, lim.max_vy / 1.0, lim.max_wz / 2.0)
        assert vx == pytest.approx(1.0 * s)
        assert vy == pytest.approx(1.0 * s)
        assert wz == pytest.approx(2.0 * s)
        assert clamped is True
        assert abs(vx) <= lim.max_vx and abs(vy) <= lim.max_vy and abs(wz) <= lim.max_wz

    def test_uniform_scaling_preserves_curvature(self, real_limits: Limits) -> None:
        """The case that produced loops on hardware: 1.2 m/s with 0.36 rad/s
        is a 3.3 m arc. Scaling vx alone made it a 0.7 m circle."""
        vx, vy, wz, clamped = clamp_velocity(1.2, 0.0, 0.36, real_limits)
        assert clamped is True
        assert vx == pytest.approx(real_limits.velocity.max_vx)
        assert wz / vx == pytest.approx(0.36 / 1.2)
        assert wz == pytest.approx(0.36 * real_limits.velocity.max_vx / 1.2)

    @pytest.mark.parametrize(
        "cmd", [(1.0, 0.0, 0.0), (0.0, 0.0, 5.0), (0.3, 0.3, 0.3), (-2.0, 0.5, -3.14), (0.1, 0.0, 0.9)]
    )
    def test_uniform_scaling_is_never_looser_than_per_axis(self, real_limits: Limits, cmd) -> None:
        lim = real_limits.velocity
        vx, vy, wz, _ = clamp_velocity(*cmd, real_limits)
        assert abs(vx) <= lim.max_vx + 1e-12
        assert abs(vy) <= lim.max_vy + 1e-12
        assert abs(wz) <= lim.max_wz + 1e-12
        # Signs survive scaling.
        for out, inp in ((vx, cmd[0]), (vy, cmd[1]), (wz, cmd[2])):
            assert (out >= 0) == (inp >= 0)

    def test_nonfinite_axis_is_zeroed_and_the_rest_still_scaled(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(1.0, math.nan, 0.5, real_limits)
        assert vy == pytest.approx(0.0)
        assert clamped is True
        assert vx == pytest.approx(real_limits.velocity.max_vx)
        assert wz == pytest.approx(0.5 * real_limits.velocity.max_vx / 1.0)

    def test_exactly_at_limit_not_clamped(self, real_limits: Limits) -> None:
        # Exactly at max_vx should pass through without clamping.
        vx, vy, wz, clamped = clamp_velocity(
            real_limits.velocity.max_vx, 0.0, 0.0, real_limits
        )
        assert vx == pytest.approx(real_limits.velocity.max_vx)
        assert clamped is False


# ===========================================================================
# Tests: apply_accel_limit
# ===========================================================================

class TestApplyAccelLimit:
    def test_small_step_passes_unchanged(self, real_limits: Limits) -> None:
        # delta = 0.04 m/s over dt=0.1 s → accel = 0.4 m/s^2 < max_ax=0.75 →
        # allowed, no limiting.
        vx, vy, wz, limited = apply_accel_limit(
            0.04, 0.0, 0.0,   # target
            0.0,  0.0, 0.0,   # last
            0.1,              # dt
            real_limits,
        )
        assert vx == pytest.approx(0.04)
        assert limited is False

    def test_large_step_limited_to_max_a_times_dt(self, real_limits: Limits) -> None:
        # max_ax=0.75, dt=0.1 → max_delta=0.075
        # target=0.3, last=0 → delta=0.3, should be clamped to 0.075
        vx, vy, wz, limited = apply_accel_limit(
            0.3, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.1,
            real_limits,
        )
        assert vx == pytest.approx(0.075)
        assert limited is True

    def test_negative_step_limited(self, real_limits: Limits) -> None:
        # target=-0.3, last=0, max_ax=0.75, dt=0.1 → allowed=-0.075
        vx, vy, wz, limited = apply_accel_limit(
            -0.3, 0.0, 0.0,
            0.0,  0.0, 0.0,
            0.1,
            real_limits,
        )
        assert vx == pytest.approx(-0.075)
        assert limited is True

    def test_zero_dt_returns_zeros(self, real_limits: Limits) -> None:
        vx, vy, wz, limited = apply_accel_limit(
            0.2, 0.1, 0.1,
            0.0, 0.0, 0.0,
            0.0,   # bad dt
            real_limits,
        )
        assert vx == pytest.approx(0.0)
        assert vy == pytest.approx(0.0)
        assert wz == pytest.approx(0.0)
        assert limited is True

    def test_negative_dt_returns_zeros(self, real_limits: Limits) -> None:
        vx, vy, wz, limited = apply_accel_limit(
            0.2, 0.1, 0.1,
            0.0, 0.0, 0.0,
            -0.05,
            real_limits,
        )
        assert vx == pytest.approx(0.0)
        assert limited is True

    def test_nan_dt_returns_zeros(self, real_limits: Limits) -> None:
        vx, vy, wz, limited = apply_accel_limit(
            0.2, 0.0, 0.0,
            0.0, 0.0, 0.0,
            math.nan,
            real_limits,
        )
        assert vx == pytest.approx(0.0)
        assert limited is True

    def test_inf_dt_returns_zeros(self, real_limits: Limits) -> None:
        # inf dt is not finite → fail-closed
        vx, vy, wz, limited = apply_accel_limit(
            0.2, 0.0, 0.0,
            0.0, 0.0, 0.0,
            math.inf,
            real_limits,
        )
        assert vx == pytest.approx(0.0)
        assert limited is True

    def test_exact_max_accel_step_not_limited(self, real_limits: Limits) -> None:
        # max_ax=0.75, dt=0.1 → max_delta=0.075; target exactly at max
        vx, vy, wz, limited = apply_accel_limit(
            0.075, 0.0, 0.0,
            0.0,  0.0, 0.0,
            0.1,
            real_limits,
        )
        assert vx == pytest.approx(0.075)
        assert limited is False


# ===========================================================================
# Tests: policy_vy_check
# ===========================================================================

class TestPolicyVyCheck:
    def test_zero_vy_ok(self, real_limits: Limits) -> None:
        corrected, violated = policy_vy_check(0.0, real_limits)
        assert corrected == pytest.approx(0.0)
        assert violated is False

    def test_tiny_vy_within_epsilon_ok(self, real_limits: Limits) -> None:
        # 1e-9 is well below the epsilon of 1e-6
        corrected, violated = policy_vy_check(1e-9, real_limits)
        assert corrected == pytest.approx(1e-9)
        assert violated is False

    def test_vy_above_epsilon_is_violation(self, real_limits: Limits) -> None:
        corrected, violated = policy_vy_check(0.2, real_limits)
        assert corrected == pytest.approx(0.0)
        assert violated is True

    def test_negative_vy_violation(self, real_limits: Limits) -> None:
        corrected, violated = policy_vy_check(-0.1, real_limits)
        assert corrected == pytest.approx(0.0)
        assert violated is True

    def test_nan_vy_is_violation(self, real_limits: Limits) -> None:
        corrected, violated = policy_vy_check(math.nan, real_limits)
        assert corrected == pytest.approx(0.0)
        assert violated is True

    def test_inf_vy_is_violation(self, real_limits: Limits) -> None:
        corrected, violated = policy_vy_check(math.inf, real_limits)
        assert corrected == pytest.approx(0.0)
        assert violated is True


# ===========================================================================
# Tests: estop_zone boundary inclusivity
# ===========================================================================

class TestEstopZone:
    """
    Boundary rules (tested explicitly):
      range < stop_zone_m  → "stop"   (strictly less than)
      range == stop_zone_m → "slow"   (at the boundary, NOT stop)
      range < slow_zone_m  → "slow"
      range == slow_zone_m → "clear"  (at the boundary, NOT slow)
      range > slow_zone_m  → "clear"
    """

    def test_clear_far(self, real_limits: Limits) -> None:
        # 5.0 m >> slow_zone_m=0.80 m
        assert estop_zone(5.0, real_limits) == "clear"

    def test_clear_at_slow_boundary(self, real_limits: Limits) -> None:
        # Exactly slow_zone_m=0.80 → clear (boundary is exclusive)
        assert estop_zone(real_limits.estop.slow_zone_m, real_limits) == "clear"

    def test_slow_just_inside_boundary(self, real_limits: Limits) -> None:
        # Just below slow_zone_m
        val = real_limits.estop.slow_zone_m - 1e-9
        assert estop_zone(val, real_limits) == "slow"

    def test_slow_midzone(self, real_limits: Limits) -> None:
        # Midpoint of the slow zone (0.35+0.80)/2 ≈ 0.575
        mid = (real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m) / 2.0
        assert estop_zone(mid, real_limits) == "slow"

    def test_slow_at_stop_boundary(self, real_limits: Limits) -> None:
        # Exactly stop_zone_m=0.35 → slow (NOT stop; boundary is exclusive)
        assert estop_zone(real_limits.estop.stop_zone_m, real_limits) == "slow"

    def test_stop_just_inside_boundary(self, real_limits: Limits) -> None:
        val = real_limits.estop.stop_zone_m - 1e-9
        assert estop_zone(val, real_limits) == "stop"

    def test_stop_zero(self, real_limits: Limits) -> None:
        assert estop_zone(0.0, real_limits) == "stop"

    def test_inf_is_clear(self, real_limits: Limits) -> None:
        # +inf means no valid returns; evaluate handles the e-stop separately.
        assert estop_zone(math.inf, real_limits) == "clear"


# ===========================================================================
# Tests: update_stop_counter (noise filter)
# ===========================================================================

class TestUpdateStopCounter:
    def test_increments_when_in_stop_zone(self, real_limits: Limits) -> None:
        assert update_stop_counter(0, True, real_limits) == 1
        assert update_stop_counter(2, True, real_limits) == 3

    def test_resets_when_clear(self, real_limits: Limits) -> None:
        assert update_stop_counter(5, False, real_limits) == 0

    def test_single_stop_does_not_hit_trigger(self, real_limits: Limits) -> None:
        # consecutive_scans_to_trigger=3; one scan gives count=1 < 3
        count = update_stop_counter(0, True, real_limits)
        assert count < real_limits.estop.consecutive_scans_to_trigger

    def test_three_consecutive_hits_trigger(self, real_limits: Limits) -> None:
        count = 0
        for _ in range(real_limits.estop.consecutive_scans_to_trigger):
            count = update_stop_counter(count, True, real_limits)
        assert count >= real_limits.estop.consecutive_scans_to_trigger


# ===========================================================================
# Tests: slow_scale
# ===========================================================================

class TestSlowScale:
    def test_at_or_beyond_slow_zone_is_one(self, real_limits: Limits) -> None:
        assert slow_scale(real_limits.estop.slow_zone_m, real_limits) == pytest.approx(1.0)
        assert slow_scale(5.0, real_limits) == pytest.approx(1.0)

    def test_at_or_below_stop_zone_is_zero(self, real_limits: Limits) -> None:
        assert slow_scale(real_limits.estop.stop_zone_m, real_limits) == pytest.approx(0.0)
        assert slow_scale(0.0, real_limits) == pytest.approx(0.0)

    def test_midzone_is_half(self, real_limits: Limits) -> None:
        mid = (real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m) / 2.0
        scale = slow_scale(mid, real_limits)
        assert scale == pytest.approx(0.5, abs=1e-9)

    def test_scale_in_zero_to_one(self, real_limits: Limits) -> None:
        for r in [0.36, 0.50, 0.60, 0.75, 0.79]:
            s = slow_scale(r, real_limits)
            assert 0.0 <= s <= 1.0


# ===========================================================================
# Tests: min_range_in_arcs
# ===========================================================================

def _flat_scan(
    value: float, n: int = 360
) -> tuple[List[float], float, float]:
    """Return a flat scan where every ray reports ``value``."""
    ranges = [value] * n
    angle_min = -math.pi
    angle_increment = math.radians(360.0 / n)
    return ranges, angle_min, angle_increment


class TestMinRangeInArcs:
    def test_all_clear_returns_that_value(self, real_limits: Limits) -> None:
        ranges, amin, ainc = _flat_scan(5.0)
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == pytest.approx(5.0)

    def test_obstacle_directly_ahead_detected(self, real_limits: Limits) -> None:
        # Place a 0.2 m return at angle=0 (index 180 in a ±180° scan).
        ranges, amin, ainc = _flat_scan(5.0)
        idx = round((0.0 - amin) / ainc)  # should be 180
        ranges[idx] = 0.2
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == pytest.approx(0.2)

    def test_obstacle_directly_behind_not_detected(self, real_limits: Limits) -> None:
        # Place a very close return at angle=pi (directly behind).
        n = 360
        ranges, amin, ainc = _flat_scan(5.0, n)
        # angle=pi normalises to pi; idx for angle closest to pi.
        # At idx 0 angle=-pi (same as pi after wrapping); depends on layout.
        # Use a slightly different approach: find the ray whose angle is ~pi.
        target_angle = math.pi - ainc / 2.0  # just under pi, clearly "rear"
        idx = round((target_angle - amin) / ainc)
        idx = max(0, min(n - 1, idx))
        ranges[idx] = 0.1
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        # The rear obstacle should NOT reduce min_range below the clear 5.0.
        assert result == pytest.approx(5.0)

    def test_nan_returns_ignored(self, real_limits: Limits) -> None:
        ranges, amin, ainc = _flat_scan(math.nan)
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == math.inf

    def test_inf_returns_ignored(self, real_limits: Limits) -> None:
        ranges, amin, ainc = _flat_scan(math.inf)
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == math.inf

    def test_zero_returns_ignored(self, real_limits: Limits) -> None:
        ranges, amin, ainc = _flat_scan(0.0)
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == math.inf

    def test_empty_ranges_returns_inf(self, real_limits: Limits) -> None:
        result = min_range_in_arcs([], 0.0, math.radians(1.0), real_limits)
        assert result == math.inf

    def test_lateral_obstacle_detected(self, real_limits: Limits) -> None:
        # Obstacle at angle = +90° (left side)
        n = 360
        ranges, amin, ainc = _flat_scan(5.0, n)
        idx = round((math.pi / 2.0 - amin) / ainc)
        idx = max(0, min(n - 1, idx))
        ranges[idx] = 0.3
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == pytest.approx(0.3)

    def test_self_occlusion_zone_masks_chassis_standoff(
        self, real_limits: Limits
    ) -> None:
        """A close return INSIDE a self_occlusion_zones_deg zone (real
        config: LEFT standoff ~95-120 deg, RIGHT ~-132 to -105 deg) must be
        masked out entirely, not reported as an obstacle -- this is the fix
        for the 2026-08-14 chassis standoff finding (see safety_limits.yaml
        estop.self_occlusion_zones_deg and CLAUDE.md).
        """
        n = 3600  # 0.1 deg resolution, enough to land inside a narrow zone
        ranges, amin, ainc = _flat_scan(5.0, n)
        for deg in (100.0, 110.0, -110.0, -120.0):
            idx = round((math.radians(deg) - amin) / ainc)
            idx = max(0, min(n - 1, idx))
            ranges[idx] = 0.05  # deep inside the 0.35 m stop zone
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        # All four close returns are masked; the flat 5.0 m background wins.
        assert result == pytest.approx(5.0)

    def test_just_outside_self_occlusion_zone_still_detected(
        self, real_limits: Limits
    ) -> None:
        """A close return just past a masked zone's edge (still inside the
        forward/lateral arcs) must still be treated as a real obstacle --
        the mask must not swallow more than its configured span.
        """
        n = 3600
        ranges, amin, ainc = _flat_scan(5.0, n)
        # 90 deg is inside lateral_arc_deg (60-120) but outside the LEFT
        # occlusion zone (95-120).
        idx = round((math.radians(90.0) - amin) / ainc)
        idx = max(0, min(n - 1, idx))
        ranges[idx] = 0.2
        result = min_range_in_arcs(ranges, amin, ainc, real_limits)
        assert result == pytest.approx(0.2)


# ===========================================================================
# Tests: evaluate — high-level FSM behaviour
# ===========================================================================

class TestEvaluate:
    # -----------------------------------------------------------------------
    # KILLED overrides everything
    # -----------------------------------------------------------------------

    def test_killed_overrides_clear_scan_and_valid_command(
        self, real_limits: Limits
    ) -> None:
        inp = _make_inputs(real_limits, is_killed=True, cmd_vx=0.1)
        dec = evaluate(inp, real_limits)
        assert dec.state == State.KILLED
        assert dec.out_vx == pytest.approx(0.0)
        assert dec.out_vy == pytest.approx(0.0)
        assert dec.out_wz == pytest.approx(0.0)

    def test_killed_reason_string(self, real_limits: Limits) -> None:
        inp = _make_inputs(real_limits, is_killed=True)
        dec = evaluate(inp, real_limits)
        assert dec.reason == "killed"

    # -----------------------------------------------------------------------
    # Scan staleness → STOPPED
    # -----------------------------------------------------------------------

    def test_stale_scan_causes_stopped(self, real_limits: Limits) -> None:
        # scan_timeout_ms=500; pass 600.
        inp = _make_inputs(
            real_limits,
            scan_age_ms=real_limits.timeouts.scan_timeout_ms + 100,
            cmd_vx=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.reason == "scan_stale"
        assert dec.out_vx == pytest.approx(0.0)

    def test_scan_age_exactly_at_timeout_is_still_ok(
        self, real_limits: Limits
    ) -> None:
        # Exactly at the timeout boundary is NOT stale (>  not >=).
        inp = _make_inputs(
            real_limits,
            scan_age_ms=real_limits.timeouts.scan_timeout_ms,
            cmd_vx=0.1,
        )
        dec = evaluate(inp, real_limits)
        # Should be OK or SLOW (not STOPPED due to staleness).
        assert dec.state in (State.OK, State.SLOW)

    # -----------------------------------------------------------------------
    # No valid returns in arcs → treated as e-stop (perception loss)
    # -----------------------------------------------------------------------

    def test_all_nan_scan_causes_stopped(self, real_limits: Limits) -> None:
        n = 360
        nan_scan = [math.nan] * n
        inp = _make_inputs(
            real_limits,
            scan_ranges=nan_scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
            cmd_vx=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        # reason is scan_stale because no_perception treats like scan_stale
        assert dec.out_vx == pytest.approx(0.0)

    # -----------------------------------------------------------------------
    # Consecutive-scan noise filter
    # -----------------------------------------------------------------------

    def test_single_stop_scan_does_not_latch(self, real_limits: Limits) -> None:
        """One scan in stop zone must NOT trigger latch."""
        n = 360
        close_scan = [5.0] * n
        # 0.20 m: above scan_range_min (0.15), below stop_zone_m (0.35).
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        close_scan[idx] = 0.20
        inp = _make_inputs(
            real_limits,
            scan_ranges=close_scan,
            scan_range_min=0.01,  # TG30's real floor
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
            consecutive_stop_count=0,
        )
        dec = evaluate(inp, real_limits)
        # Counter increments to 1; trigger is 3 → latch must not have fired.
        assert dec.estop_latched is False
        # Must not be STOPPED specifically because of stop_zone (it may be in
        # a transient STOPPED state for another reason, but the latch check
        # above is the authoritative assertion that the noise filter held).
        assert not (dec.state == State.STOPPED and dec.reason == "stop_zone")

    def test_consecutive_trigger_latches_stop(self, real_limits: Limits) -> None:
        """N consecutive stop-zone scans must latch; no single earlier cycle may."""
        n = 360
        close_scan = [5.0] * n
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        # Inside the stop zone, derived from config. scan_range_min is set to
        # the TG30's real 0.01 floor below, not the fixture default of 0.15:
        # stop_zone_m is now 0.15, so a 0.15 sensor floor would leave NO range
        # that is both valid and inside the stop zone.
        close_scan[idx] = real_limits.estop.stop_zone_m * 0.8

        trigger = real_limits.estop.consecutive_scans_to_trigger  # 3

        dec = None
        count = 0
        latched = False
        for cycle in range(trigger):
            inp = _make_inputs(
                real_limits,
                scan_ranges=close_scan,
                scan_range_min=0.01,  # TG30's real floor
                scan_angle_min=-math.pi,
                scan_angle_increment=math.radians(1.0),
                consecutive_stop_count=count,
                estop_latched=latched,
            )
            dec = evaluate(inp, real_limits)

            # The whole purpose of the noise filter is that the latch must NOT
            # fire on any cycle BEFORE we reach the trigger count.
            if cycle < trigger - 1:
                assert dec.estop_latched is False, (
                    f"Latch fired early on cycle {cycle} "
                    f"(trigger requires {trigger} consecutive scans)"
                )

            count = dec.consecutive_stop_count
            latched = dec.estop_latched

        assert dec is not None
        assert dec.state == State.STOPPED
        assert dec.estop_latched is True

    # -----------------------------------------------------------------------
    # Latching: stays stopped even when scan clears, until reset
    # -----------------------------------------------------------------------

    def test_latched_stop_persists_on_clear_scan(self, real_limits: Limits) -> None:
        # estop_latched=True, clear scan (5.0 m everywhere).
        inp = _make_inputs(
            real_limits,
            estop_latched=True,
            reset_latch=False,
            cmd_vx=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.reason == "stop_zone"
        assert dec.out_vx == pytest.approx(0.0)

    def test_reset_clears_latch(self, real_limits: Limits) -> None:
        inp = _make_inputs(
            real_limits,
            estop_latched=True,
            reset_latch=True,
            cmd_vx=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.estop_latched is False
        # With latch cleared and clear scan, should be OK.
        assert dec.state == State.OK

    # -----------------------------------------------------------------------
    # Command staleness → STOPPED
    # -----------------------------------------------------------------------

    def test_stale_command_causes_stopped(self, real_limits: Limits) -> None:
        # command_timeout_ms=300; pass 400.
        inp = _make_inputs(
            real_limits,
            command_age_ms=real_limits.timeouts.command_timeout_ms + 100,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.reason == "command_timeout"
        assert dec.out_vx == pytest.approx(0.0)

    def test_command_age_at_timeout_is_ok(self, real_limits: Limits) -> None:
        inp = _make_inputs(
            real_limits,
            command_age_ms=real_limits.timeouts.command_timeout_ms,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state in (State.OK, State.SLOW)

    # -----------------------------------------------------------------------
    # Slow zone → SLOW state with scaled velocity
    # -----------------------------------------------------------------------

    def test_obstacle_in_slow_zone_gives_slow_state(
        self, real_limits: Limits
    ) -> None:
        n = 360
        mid_range = (
            real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m
        ) / 2.0  # 0.575 m
        slow_scan = [5.0] * n
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        slow_scan[idx] = mid_range
        inp = _make_inputs(
            real_limits,
            scan_ranges=slow_scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
            cmd_vx=real_limits.velocity.max_vx,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.SLOW
        # Velocity should be reduced but not zero.
        assert dec.out_vx > 0.0
        assert dec.out_vx < real_limits.velocity.max_vx

    def test_slow_zone_velocity_scaled_by_expected_factor(
        self, real_limits: Limits
    ) -> None:
        # Place obstacle at midpoint → scale should be 0.5.
        # Command exactly at max_vx so clamp_velocity() is a no-op and the
        # slow-zone scale factor is isolated cleanly (was hardcoded 0.2,
        # which exceeded max_vx after it was lowered to 0.15 2026-08-14 and
        # got clamped before scaling, breaking this test's assumption).
        n = 360
        mid_range = (
            real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m
        ) / 2.0
        slow_scan = [5.0] * n
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        slow_scan[idx] = mid_range
        cmd_vx = real_limits.velocity.max_vx
        inp = _make_inputs(
            real_limits,
            scan_ranges=slow_scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
            cmd_vx=cmd_vx,
            last_vx=cmd_vx,   # no accel limit so we see the scale clearly
            dt=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.SLOW
        expected_scale = 0.5
        assert dec.out_vx == pytest.approx(cmd_vx * expected_scale, abs=1e-3)

    # -----------------------------------------------------------------------
    # Nominal (OK) path: clamping and accel limiting
    # -----------------------------------------------------------------------

    def test_nominal_command_passes_through(self, real_limits: Limits) -> None:
        # Set last_vx/wz equal to the command so that acceleration limiting
        # does not apply — we are testing that a steady-state valid command
        # passes through unmodified, not the ramp-up behaviour.
        inp = _make_inputs(
            real_limits,
            cmd_vx=0.1, cmd_vy=0.0, cmd_wz=0.1,
            last_vx=0.1, last_vy=0.0, last_wz=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.OK
        assert dec.out_vx == pytest.approx(0.1)
        assert dec.out_wz == pytest.approx(0.1)

    def test_over_limit_command_clamped(self, real_limits: Limits) -> None:
        inp = _make_inputs(real_limits, cmd_vx=1.0, is_policy_source=False)
        dec = evaluate(inp, real_limits)
        assert dec.clamping_active is True
        assert dec.out_vx <= real_limits.velocity.max_vx + 1e-9

    def test_nan_command_produces_zero(self, real_limits: Limits) -> None:
        inp = _make_inputs(real_limits, cmd_vx=math.nan)
        dec = evaluate(inp, real_limits)
        assert dec.out_vx == pytest.approx(0.0)
        assert dec.clamping_active is True

    # -----------------------------------------------------------------------
    # Policy vy violation detected and corrected inside evaluate
    # -----------------------------------------------------------------------

    def test_policy_vy_violation_zeroed_and_flagged(
        self, real_limits: Limits
    ) -> None:
        inp = _make_inputs(
            real_limits,
            cmd_vy=0.2,
            is_policy_source=True,
        )
        dec = evaluate(inp, real_limits)
        assert dec.out_vy == pytest.approx(0.0)
        assert dec.clamping_active is True

    def test_teleop_vy_not_zeroed_only_clamped(self, real_limits: Limits) -> None:
        # Teleop is allowed to strafe; vy within limit should pass through.
        # Set last_vy equal to the command so the acceleration limiter does not
        # reduce it — we are testing the policy_vy_check bypass, not the ramp.
        inp = _make_inputs(
            real_limits,
            cmd_vy=0.1,   # within max_vy=0.25
            last_vy=0.1,
            is_policy_source=False,
        )
        dec = evaluate(inp, real_limits)
        assert dec.out_vy == pytest.approx(0.1)

    # -----------------------------------------------------------------------
    # internal_error fail-safe — we force an exception via a broken input type
    # -----------------------------------------------------------------------

    def test_internal_exception_returns_stopped_zero(
        self, real_limits: Limits
    ) -> None:
        # Pass an EvalInputs with scan_ranges that is not a list at all,
        # which will cause an error inside _evaluate_inner.
        inp = _make_inputs(real_limits)
        # Mutate after creation to inject a deliberately broken scan_ranges.
        inp.scan_ranges = None  # type: ignore[assignment]
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.reason == "internal_error"
        assert dec.out_vx == pytest.approx(0.0)
        assert dec.out_vy == pytest.approx(0.0)
        assert dec.out_wz == pytest.approx(0.0)

    # -----------------------------------------------------------------------
    # FIX 1: empty scan (zero ranges) with a fresh timestamp → STOPPED
    # This was the blocker: bool([]) == False caused the guard to be skipped
    # and the robot was allowed to move with no perception.
    # -----------------------------------------------------------------------

    def test_empty_scan_fresh_timestamp_causes_stopped(
        self, real_limits: Limits
    ) -> None:
        """Empty scan_ranges with a fresh timestamp must still trigger STOPPED.

        Before the fix, ``no_perception = (min_r == math.inf) and bool([])``
        evaluated to False (bool([]) is False), so an empty lidar message
        silently bypassed the perception-loss e-stop.  After the fix the guard
        is simply ``(min_r == math.inf)``, which catches all no-valid-returns
        cases uniformly.
        """
        inp = _make_inputs(
            real_limits,
            scan_ranges=[],          # empty — no rays at all
            scan_age_ms=0.0,         # fresh timestamp, NOT stale
            cmd_vx=0.1,              # would cause motion if the guard were bypassed
            command_age_ms=0.0,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.out_vx == pytest.approx(0.0)
        assert dec.out_vy == pytest.approx(0.0)
        assert dec.out_wz == pytest.approx(0.0)

    # -----------------------------------------------------------------------
    # FIX 2: reset with obstacle still in stop zone must not permit motion
    # -----------------------------------------------------------------------

    def test_reset_with_obstacle_still_present_stays_stopped(
        self, real_limits: Limits
    ) -> None:
        """reset_latch=True while an obstacle is still in the stop zone must
        not cause motion.  The latch must re-assert in the same cycle so the
        reset cycle itself outputs zero velocity.
        """
        n = 360
        close_scan = [5.0] * n
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        # Inside the stop zone, derived. See the note in
        # test_consecutive_trigger_latches_stop about scan_range_min.
        close_scan[idx] = real_limits.estop.stop_zone_m * 0.8

        inp = _make_inputs(
            real_limits,
            scan_ranges=close_scan,
            scan_range_min=0.01,  # TG30's real floor
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
            estop_latched=True,      # already latched from a prior cycle
            reset_latch=True,        # operator pressed reset — but obstacle still there
            cmd_vx=0.1,
            command_age_ms=0.0,
        )
        dec = evaluate(inp, real_limits)
        # Obstacle still in stop zone → must re-latch and output zero.
        assert dec.state == State.STOPPED
        assert dec.out_vx == pytest.approx(0.0)

    def test_reset_with_clear_scan_rearms_ok(self, real_limits: Limits) -> None:
        """reset_latch=True with a clear scan must genuinely re-arm the FSM.

        The obstacle is gone; the reset should succeed and the next cycle
        should be OK (not stuck in STOPPED because of a spurious re-latch).
        """
        inp = _make_inputs(
            real_limits,
            estop_latched=True,
            reset_latch=True,
            cmd_vx=0.1,
            last_vx=0.1,   # steady-state so accel limiter does not interfere
            command_age_ms=0.0,
            # default scan: 360 rays at 5.0 m — well clear of all zones
        )
        dec = evaluate(inp, real_limits)
        assert dec.estop_latched is False
        assert dec.state == State.OK

    # -----------------------------------------------------------------------
    # Killed + valid command + clear scan → still KILLED (precedence check)
    # -----------------------------------------------------------------------

    def test_killed_takes_precedence_over_stop_zone(
        self, real_limits: Limits
    ) -> None:
        n = 360
        close_scan = [5.0] * n
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        close_scan[idx] = 0.20  # above range_min (0.15), below stop_zone_m (0.35)
        inp = _make_inputs(
            real_limits,
            is_killed=True,
            scan_ranges=close_scan,
            scan_range_min=0.01,  # TG30's real floor
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.KILLED
        assert dec.reason == "killed"


# ===========================================================================
# Tests: odom_stale — FIX A
# ===========================================================================

class TestOdomStale:
    """
    odom_stale=True is a first-class stop condition with reason "odom_stale".
    Precedence 2.5: after scan_stale (2), before stop_zone (3) and
    command_timeout (4).
    """

    def test_odom_stale_produces_stopped_with_correct_reason(
        self, real_limits: Limits
    ) -> None:
        """odom_stale=True + fresh command + clear scan → STOPPED, reason odom_stale."""
        inp = _make_inputs(
            real_limits,
            cmd_vx=0.1,
            command_age_ms=0.0,   # fresh command — not a command-timeout case
            odom_stale=True,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.reason == "odom_stale"

    def test_odom_stale_output_is_all_zeros(self, real_limits: Limits) -> None:
        """All three output velocity components must be zero when odom is stale."""
        inp = _make_inputs(
            real_limits,
            cmd_vx=0.2,
            cmd_vy=0.0,
            cmd_wz=0.3,
            odom_stale=True,
        )
        dec = evaluate(inp, real_limits)
        assert dec.out_vx == pytest.approx(0.0)
        assert dec.out_vy == pytest.approx(0.0)
        assert dec.out_wz == pytest.approx(0.0)

    def test_odom_stale_passthrough_latch_and_counter(
        self, real_limits: Limits
    ) -> None:
        """latch and consecutive_stop_count must be passed through unchanged
        (same as the scan_stale branch — odom dropout must not disturb the
        lidar noise filter)."""
        inp = _make_inputs(
            real_limits,
            odom_stale=True,
            estop_latched=True,
            consecutive_stop_count=2,
        )
        dec = evaluate(inp, real_limits)
        assert dec.estop_latched is True
        assert dec.consecutive_stop_count == 2

    def test_killed_overrides_odom_stale(self, real_limits: Limits) -> None:
        """KILLED (precedence 1) must win over odom_stale (precedence 2.5)."""
        inp = _make_inputs(
            real_limits,
            is_killed=True,
            odom_stale=True,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.KILLED
        assert dec.reason == "killed"

    def test_scan_stale_overrides_odom_stale(self, real_limits: Limits) -> None:
        """scan_stale (precedence 2) must win over odom_stale (precedence 2.5).

        When both scan_stale and odom_stale are True, the reason reported must
        be "scan_stale" because scan staleness is checked first and short-
        circuits before the odom check is reached.
        """
        inp = _make_inputs(
            real_limits,
            scan_age_ms=real_limits.timeouts.scan_timeout_ms + 100.0,  # stale
            odom_stale=True,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.STOPPED
        assert dec.reason == "scan_stale"

    def test_odom_fresh_does_not_stop(self, real_limits: Limits) -> None:
        """odom_stale=False (the default) with a fresh command → not stopped
        due to odom.  State should be OK with a clear scan."""
        inp = _make_inputs(
            real_limits,
            cmd_vx=0.1,
            last_vx=0.1,   # steady-state: no accel step
            odom_stale=False,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.OK
        assert dec.reason == "ok"


# ---------------------------------------------------------------------------
# Slow-zone axis asymmetry: translation is scaled, YAW IS NOT.
#
# Regression pin for the floor-test finding of 2026-08-28. Before the fix,
# evaluate() applied slow_scale() to all three axes, so a commanded spin was
# throttled in proportion to how close the obstacle was: at min_range 0.50 m a
# 90 deg turn took 11.8 s, and at 0.40 m it took 35 s. On the floor that read
# as "the robot reacts late to a spin command" -- it was not late, it was
# throttled to a near-standstill by its own proximity, exactly when turning
# away was the only thing left to do.
#
# Rotating in place cannot close the distance to anything: the robot's own
# measured extent in the scan plane is ~0.26 m (chassis standoffs, see
# safety_limits.yaml self_occlusion_zones_deg), so a pure yaw sweeps a circle
# entirely inside the 0.35 m stop zone. Yaw stays bounded by the max_wz clamp,
# the max_alpha_z ramp, and the stop-zone latch -- none of which this changes.
#
# NOTE this whole behaviour was previously UNTESTED: removing `wz *= scale`
# left all 145 tests green, which is why it survived to a floor test.
# ---------------------------------------------------------------------------


def _slow_zone_scan(limits: Limits, min_range_m: float) -> List[float]:
    """A 360-ray scan whose nearest forward return is exactly min_range_m."""
    scan = [5.0] * 360
    scan[0] = min_range_m  # index 0 == -pi... set a true forward ray instead
    scan[180] = min_range_m  # angle_min=-pi + 180*1deg = 0 rad, straight ahead
    scan[0] = 5.0
    return scan


class TestSlowZoneDoesNotScaleYaw:
    def test_yaw_survives_slow_zone_at_full_commanded_rate(
        self, real_limits: Limits
    ) -> None:
        """A spin command in the slow zone comes out at the max_wz clamp, NOT
        scaled down by proximity."""
        # Halfway between the stop and slow thresholds: squarely inside the
        # scaling band whatever those are tuned to. Derived, not hardcoded,
        # so retuning the zones does not silently move this test off the band.
        midpoint = (
            real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m
        ) / 2.0
        inputs = _make_inputs(
            real_limits,
            cmd_wz=1.2,  # what the debug reactive rule commands
            last_wz=real_limits.velocity.max_wz,  # already spinning: no ramp
            scan_ranges=_slow_zone_scan(real_limits, midpoint),
        )
        d = evaluate(inputs, real_limits)
        assert d.state == State.SLOW
        assert d.reason == "slow_zone"
        assert d.out_wz == pytest.approx(real_limits.velocity.max_wz)

    def test_translation_is_still_scaled_in_the_slow_zone(
        self, real_limits: Limits
    ) -> None:
        """The fix must NOT have disabled slow-zone braking. Forward motion is
        what closes distance, and it stays throttled."""
        stop_m = real_limits.estop.stop_zone_m
        slow_m = real_limits.estop.slow_zone_m
        midpoint = (stop_m + slow_m) / 2.0
        inputs = _make_inputs(
            real_limits,
            cmd_vx=1.0,
            last_vx=real_limits.velocity.max_vx,  # already moving: no ramp
            scan_ranges=_slow_zone_scan(real_limits, midpoint),
        )
        d = evaluate(inputs, real_limits)
        assert d.state == State.SLOW
        expected_scale = (midpoint - stop_m) / (slow_m - stop_m)
        assert d.out_vx == pytest.approx(
            real_limits.velocity.max_vx * expected_scale, rel=1e-6
        )
        assert d.out_vx < real_limits.velocity.max_vx

    def test_yaw_is_still_zeroed_by_the_stop_zone_latch(
        self, real_limits: Limits
    ) -> None:
        """The escape-manoeuvre exemption must not leak past the latch: once
        STOPPED, every axis is zero including yaw."""
        inputs = _make_inputs(
            real_limits,
            cmd_wz=1.2,
            last_wz=real_limits.velocity.max_wz,
            scan_ranges=_slow_zone_scan(
                real_limits, real_limits.estop.stop_zone_m * 0.8
            ),  # inside stop zone, derived from config
            estop_latched=True,
        )
        d = evaluate(inputs, real_limits)
        assert d.state == State.STOPPED
        assert d.out_wz == 0.0
        assert d.out_vx == 0.0

    def test_yaw_still_bounded_by_max_wz_clamp(self, real_limits: Limits) -> None:
        """Not scaling is not the same as not clamping."""
        inputs = _make_inputs(
            real_limits,
            cmd_wz=99.0,
            last_wz=real_limits.velocity.max_wz,
            scan_ranges=_slow_zone_scan(
                real_limits,
                (real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m) / 2,
            ),
        )
        d = evaluate(inputs, real_limits)
        assert abs(d.out_wz) <= real_limits.velocity.max_wz + 1e-9


# ===========================================================================
# Stop-zone RETREAT (2026-10-23): the one exception to "latched means zero".
# A fresh, pure, straight reverse passes capped and ramped under the latch;
# the latch releases only after N such ticks with a clear wide-arc scan.
# ===========================================================================

import dataclasses as _dc


def _close_scan(limits: Limits, n: int = 360) -> List[float]:
    """A clear 360-deg scan with one return inside the stop zone dead ahead."""
    scan = [5.0] * n
    idx = round((0.0 - (-math.pi)) / math.radians(1.0))
    scan[idx] = limits.estop.stop_zone_m * 0.8
    return scan


def _latched_inputs(limits: Limits, **overrides) -> EvalInputs:
    """Latched, close scan, TG30 floor, pure reverse unless overridden."""
    kwargs = dict(
        cmd_vx=-0.2, cmd_vy=0.0, cmd_wz=0.0,
        scan_ranges=_close_scan(limits), scan_range_min=0.01,
        estop_latched=True,
        consecutive_stop_count=limits.estop.consecutive_scans_to_trigger,
        dt=0.05,
    )
    kwargs.update(overrides)
    return _make_inputs(limits, **kwargs)


def _with_estop(limits: Limits, **fields) -> Limits:
    return _dc.replace(limits, estop=_dc.replace(limits.estop, **fields))


class TestStopZoneRetreat:

    def test_shipped_config_enables_the_retreat_within_the_clamp(self, real_limits):
        es = real_limits.estop
        assert es.retreat_enabled
        assert 0.0 < es.retreat_max_vx <= real_limits.velocity.max_vx
        assert es.retreat_release_range_m > es.stop_zone_m
        assert es.retreat_release_ticks >= 1

    def test_latched_pure_reverse_is_passed_capped_and_ramped(self, real_limits):
        inp = _latched_inputs(real_limits)
        dec = evaluate(inp, real_limits)
        assert dec.state == State.SLOW and dec.reason == "stop_zone_retreat"
        # First tick from rest: the ramp bounds it before the cap does.
        ramp = real_limits.acceleration.max_ax * inp.dt
        assert dec.out_vx == pytest.approx(-min(real_limits.estop.retreat_max_vx, ramp))
        assert dec.out_vy == 0.0 and dec.out_wz == 0.0
        assert dec.estop_latched is True
        assert dec.clamping_active is True
        assert dec.min_range == pytest.approx(real_limits.estop.stop_zone_m * 0.8)
        assert dec.retreat_clear_count == 0      # the scan is not clear

    def test_retreat_never_exceeds_retreat_max_vx_in_steady_state(self, real_limits):
        last = 0.0
        for _ in range(40):
            dec = evaluate(_latched_inputs(real_limits, last_vx=last), real_limits)
            assert dec.reason == "stop_zone_retreat"
            assert abs(dec.out_vx) <= real_limits.estop.retreat_max_vx + 1e-12
            last = dec.out_vx
        assert last == pytest.approx(-real_limits.estop.retreat_max_vx)

    @pytest.mark.parametrize("field, value", [
        ("cmd_wz", 1e-3), ("cmd_wz", 0.35), ("cmd_wz", -0.35),
        ("cmd_vy", 1e-3), ("cmd_vy", -0.05),
        ("cmd_vx", 0.0), ("cmd_vx", 0.1), ("cmd_vx", float("nan")),
    ])
    def test_anything_but_pure_straight_reverse_stays_zero(self, real_limits, field, value):
        dec = evaluate(_latched_inputs(real_limits, **{field: value}), real_limits)
        assert dec.state == State.STOPPED and dec.reason == "stop_zone"
        assert (dec.out_vx, dec.out_vy, dec.out_wz) == (0.0, 0.0, 0.0)
        assert dec.estop_latched is True and dec.retreat_clear_count == 0

    def test_a_stale_reverse_stays_zero(self, real_limits):
        stale = real_limits.timeouts.command_timeout_ms + 1.0
        dec = evaluate(_latched_inputs(real_limits, command_age_ms=stale), real_limits)
        assert dec.state == State.STOPPED and dec.reason == "stop_zone"
        assert dec.out_vx == 0.0

    def test_a_bad_dt_gives_zero(self, real_limits):
        dec = evaluate(_latched_inputs(real_limits, dt=0.0), real_limits)
        assert dec.reason == "stop_zone_retreat" and dec.out_vx == 0.0

    def test_the_teleop_source_gets_it_too(self, real_limits):
        dec = evaluate(_latched_inputs(real_limits, is_policy_source=False), real_limits)
        assert dec.reason == "stop_zone_retreat" and dec.out_vx < 0.0

    def test_absent_or_zero_key_keeps_todays_behaviour(self, real_limits):
        with open(_CONFIG_PATH, "r") as fh:
            raw = yaml.safe_load(fh)
        del raw["estop"]["retreat_max_vx"]
        absent = Limits.from_dict(raw, _b1_mount())
        assert absent.estop.retreat_max_vx == 0.0 and not absent.estop.retreat_enabled
        for limits in (absent, _with_estop(real_limits, retreat_max_vx=0.0)):
            dec = evaluate(_latched_inputs(limits), limits)
            assert dec.state == State.STOPPED and dec.reason == "stop_zone"
            assert dec.out_vx == 0.0

    @pytest.mark.parametrize("fields, fragment", [
        (dict(retreat_max_vx=-0.1), "retreat_max_vx"),
        (dict(retreat_max_vx=float("inf")), "retreat_max_vx"),
        (dict(retreat_max_vx=9.0), "retreat_max_vx"),
        (dict(retreat_release_range_m=0.10), "retreat_release_range_m"),
        (dict(retreat_release_ticks=0), "retreat_release_ticks"),
    ])
    def test_incoherent_retreat_keys_are_refused(self, fields, fragment):
        with open(_CONFIG_PATH, "r") as fh:
            raw = yaml.safe_load(fh)
        raw["estop"].update(fields)
        with pytest.raises(ValueError, match=fragment):
            Limits.from_dict(raw, _b1_mount())

    @pytest.mark.parametrize("override", [
        dict(is_killed=True), dict(scan_age_ms=1e6), dict(odom_stale=True),
    ])
    def test_kill_and_staleness_win_over_the_retreat(self, real_limits, override):
        dec = evaluate(_latched_inputs(real_limits, **override), real_limits)
        assert (dec.out_vx, dec.out_vy, dec.out_wz) == (0.0, 0.0, 0.0)
        assert dec.reason in ("killed", "scan_stale", "odom_stale")
        assert dec.estop_latched is True

    def test_a_prelatch_reverse_passes_capped_while_a_forward_is_zeroed(self, real_limits):
        """The pre-latch window is the same obstacle ahead: a pure straight
        reverse passes capped (not latched, not scaled); a forward is braked
        to zero as before."""
        inp = _latched_inputs(real_limits, estop_latched=False, consecutive_stop_count=0)
        dec = evaluate(inp, real_limits)
        assert dec.state == State.SLOW and dec.reason == "stop_zone_prelatch"
        assert -real_limits.estop.retreat_max_vx <= dec.out_vx < 0.0
        assert dec.estop_latched is False
        fwd = evaluate(_latched_inputs(real_limits, estop_latched=False,
                                       consecutive_stop_count=0, cmd_vx=0.2), real_limits)
        assert fwd.reason == "stop_zone_prelatch" and fwd.out_vx == 0.0

    def test_unreachable_with_the_stop_zone_disabled(self, real_limits):
        limits = _with_estop(real_limits, stop_zone_enabled=False)
        dec = evaluate(_latched_inputs(limits), limits)
        assert dec.reason != "stop_zone_retreat"
        assert dec.estop_latched is False

    # ---- the release ------------------------------------------------------

    def _release_run(self, limits, *, reverse_each_tick=True, clear=True, ticks=None):
        """Drive the latch with a reverse command and a scan that is clear to
        the release range (or not); return the decisions."""
        scan = [5.0] * 360 if clear else _close_scan(limits)
        if clear:
            scan[180] = limits.estop.retreat_release_range_m  # exactly at the range: clear
        decisions = []
        latched, count, clear_count, last = True, limits.estop.consecutive_scans_to_trigger, 0, 0.0
        for _ in range(ticks or limits.estop.retreat_release_ticks + 2):
            inp = _latched_inputs(
                limits, scan_ranges=scan, estop_latched=latched,
                consecutive_stop_count=count, retreat_clear_count=clear_count,
                last_vx=last, cmd_vx=(-0.2 if reverse_each_tick else 0.0),
            )
            dec = evaluate(inp, limits)
            decisions.append(dec)
            latched, count, clear_count, last = (
                dec.estop_latched, dec.consecutive_stop_count, dec.retreat_clear_count, dec.out_vx,
            )
        return decisions

    def test_release_needs_n_clear_ticks_with_an_active_reverse(self, real_limits):
        n = real_limits.estop.retreat_release_ticks
        decs = self._release_run(real_limits)
        for k, dec in enumerate(decs[:n - 1]):
            assert dec.estop_latched is True and dec.reason == "stop_zone_retreat", k
            assert dec.retreat_clear_count == k + 1
        released = decs[n - 1]
        assert released.estop_latched is False
        assert released.reason != "stop_zone_retreat" and released.state in (State.OK, State.SLOW)
        assert released.consecutive_stop_count == 0 and released.retreat_clear_count == 0
        # ... and the reverse command itself continues under the normal clamp.
        assert released.out_vx < 0.0

    def test_a_clear_scan_without_reverse_never_releases(self, real_limits):
        decs = self._release_run(real_limits, reverse_each_tick=False, ticks=50)
        assert all(d.estop_latched and d.reason == "stop_zone" and d.out_vx == 0.0 for d in decs)

    def test_a_reverse_into_an_unclear_scan_never_releases(self, real_limits):
        decs = self._release_run(real_limits, clear=False, ticks=50)
        assert all(d.estop_latched and d.reason == "stop_zone_retreat" for d in decs)
        assert all(d.retreat_clear_count == 0 for d in decs)

    def test_the_release_counter_resets_when_the_reverse_pauses(self, real_limits):
        n = real_limits.estop.retreat_release_ticks
        scan = [5.0] * 360
        latched, count, clear_count = True, 3, 0
        for k in range(n - 1):
            dec = evaluate(_latched_inputs(real_limits, scan_ranges=scan, estop_latched=latched,
                                           consecutive_stop_count=count, retreat_clear_count=clear_count),
                           real_limits)
            latched, count, clear_count = dec.estop_latched, dec.consecutive_stop_count, dec.retreat_clear_count
        assert clear_count == n - 1 and latched
        pause = evaluate(_latched_inputs(real_limits, scan_ranges=scan, estop_latched=True,
                                         consecutive_stop_count=count, retreat_clear_count=clear_count,
                                         cmd_vx=0.0), real_limits)
        assert pause.estop_latched is True and pause.retreat_clear_count == 0

    def test_an_obstacle_returning_after_release_relatches(self, real_limits):
        latched, count = False, 0
        for _ in range(real_limits.estop.consecutive_scans_to_trigger):
            dec = evaluate(_latched_inputs(real_limits, estop_latched=latched,
                                           consecutive_stop_count=count, cmd_vx=0.1), real_limits)
            latched, count = dec.estop_latched, dec.consecutive_stop_count
        assert latched is True and dec.reason == "stop_zone"

    def test_stopped_state_always_means_zero_output(self, real_limits):
        cases = [
            _latched_inputs(real_limits, cmd_vx=0.1),
            _latched_inputs(real_limits, is_killed=True),
            _latched_inputs(real_limits, scan_age_ms=1e6),
            _latched_inputs(real_limits, odom_stale=True),
            _latched_inputs(real_limits, command_age_ms=1e6),
            _make_inputs(real_limits, cmd_vx=-0.2, command_age_ms=1e6),
        ]
        for inp in cases:
            dec = evaluate(inp, real_limits)
            if dec.state == State.STOPPED:
                assert (dec.out_vx, dec.out_vy, dec.out_wz) == (0.0, 0.0, 0.0)


class TestSlowBandRetreat:
    """A pure straight reverse in the slow band is capped, not scaled."""

    def _slow_inputs(self, limits, **overrides):
        scan = [5.0] * 360
        scan[180] = limits.estop.stop_zone_m + 0.05      # ahead, inside the slow cone
        kwargs = dict(cmd_vx=-0.2, cmd_vy=0.0, cmd_wz=0.0, scan_ranges=scan,
                      scan_range_min=0.01, dt=0.05, last_vx=-0.12)
        kwargs.update(overrides)
        return _make_inputs(limits, **kwargs)

    def test_pure_reverse_passes_at_the_retreat_speed(self, real_limits):
        dec = evaluate(self._slow_inputs(real_limits), real_limits)
        assert dec.state == State.SLOW and dec.reason == "slow_zone"
        assert dec.out_vx == pytest.approx(-real_limits.estop.retreat_max_vx)
        assert dec.out_vy == 0.0 and dec.out_wz == 0.0
        assert dec.estop_latched is False

    def test_forward_is_still_scaled_to_a_crawl(self, real_limits):
        dec = evaluate(self._slow_inputs(real_limits, cmd_vx=0.84, last_vx=0.0), real_limits)
        assert dec.reason == "slow_zone"
        assert 0.0 <= dec.out_vx < 0.05

    @pytest.mark.parametrize("field, value", [("cmd_wz", 0.2), ("cmd_vy", 0.05)])
    def test_a_turning_or_strafing_reverse_is_scaled_like_anything_else(self, real_limits, field, value):
        dec = evaluate(self._slow_inputs(real_limits, **{field: value}), real_limits)
        assert abs(dec.out_vx) < 0.05

    def test_off_key_scales_reverse_as_before(self, real_limits):
        limits = _with_estop(real_limits, retreat_max_vx=0.0)
        dec = evaluate(self._slow_inputs(limits), limits)
        assert abs(dec.out_vx) < 0.05
