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
    return Limits.from_dict(raw)


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
    )


# ===========================================================================
# Tests: Limits.from_dict
# ===========================================================================

class TestLimitsFromDict:
    def test_loads_velocity_limits(self, real_limits: Limits) -> None:
        assert real_limits.velocity.max_vx == pytest.approx(0.25)
        assert real_limits.velocity.max_vy == pytest.approx(0.25)
        assert real_limits.velocity.max_wz == pytest.approx(0.4)

    def test_loads_acceleration_limits(self, real_limits: Limits) -> None:
        assert real_limits.acceleration.max_ax == pytest.approx(0.5)
        assert real_limits.acceleration.max_ay == pytest.approx(0.5)
        assert real_limits.acceleration.max_alpha_z == pytest.approx(1.0)

    def test_loads_estop(self, real_limits: Limits) -> None:
        assert real_limits.estop.slow_zone_m == pytest.approx(0.80)
        assert real_limits.estop.stop_zone_m == pytest.approx(0.35)
        assert real_limits.estop.consecutive_scans_to_trigger == 3

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
        # max_vx=0.25, max_vy=0.25, max_wz=0.4
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

    def test_all_axes_clamped_independently(self, real_limits: Limits) -> None:
        vx, vy, wz, clamped = clamp_velocity(1.0, 1.0, 2.0, real_limits)
        assert vx == pytest.approx(real_limits.velocity.max_vx)
        assert vy == pytest.approx(real_limits.velocity.max_vy)
        assert wz == pytest.approx(real_limits.velocity.max_wz)
        assert clamped is True

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
        # delta = 0.05 m/s over dt=0.1 s → accel = 0.5 m/s² = max_ax → allowed
        # Use slightly less than max to confirm no limiting.
        vx, vy, wz, limited = apply_accel_limit(
            0.04, 0.0, 0.0,   # target
            0.0,  0.0, 0.0,   # last
            0.1,              # dt
            real_limits,
        )
        assert vx == pytest.approx(0.04)
        assert limited is False

    def test_large_step_limited_to_max_a_times_dt(self, real_limits: Limits) -> None:
        # max_ax=0.5, dt=0.1 → max_delta=0.05
        # target=0.3, last=0 → delta=0.3, should be clamped to 0.05
        vx, vy, wz, limited = apply_accel_limit(
            0.3, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.1,
            real_limits,
        )
        assert vx == pytest.approx(0.05)
        assert limited is True

    def test_negative_step_limited(self, real_limits: Limits) -> None:
        # target=-0.3, last=0, max_ax=0.5, dt=0.1 → allowed=-0.05
        vx, vy, wz, limited = apply_accel_limit(
            -0.3, 0.0, 0.0,
            0.0,  0.0, 0.0,
            0.1,
            real_limits,
        )
        assert vx == pytest.approx(-0.05)
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
        # max_ax=0.5, dt=0.1 → max_delta=0.05; target exactly at max
        vx, vy, wz, limited = apply_accel_limit(
            0.05, 0.0, 0.0,
            0.0,  0.0, 0.0,
            0.1,
            real_limits,
        )
        assert vx == pytest.approx(0.05)
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
        # 0.20 m: above scan_range_min (0.15) so not filtered, below
        # stop_zone_m (0.35) so in the stop zone.
        close_scan[idx] = 0.20

        trigger = real_limits.estop.consecutive_scans_to_trigger  # 3

        dec = None
        count = 0
        latched = False
        for cycle in range(trigger):
            inp = _make_inputs(
                real_limits,
                scan_ranges=close_scan,
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
        n = 360
        mid_range = (
            real_limits.estop.stop_zone_m + real_limits.estop.slow_zone_m
        ) / 2.0
        slow_scan = [5.0] * n
        idx = round((0.0 - (-math.pi)) / math.radians(1.0))
        slow_scan[idx] = mid_range
        inp = _make_inputs(
            real_limits,
            scan_ranges=slow_scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=math.radians(1.0),
            cmd_vx=0.2,
            last_vx=0.2,   # no accel limit so we see the scale clearly
            dt=0.1,
        )
        dec = evaluate(inp, real_limits)
        assert dec.state == State.SLOW
        expected_scale = 0.5
        assert dec.out_vx == pytest.approx(0.2 * expected_scale, abs=1e-3)

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
        # 0.20 m — above scan_range_min (0.15), below stop_zone_m (0.35).
        close_scan[idx] = 0.20

        inp = _make_inputs(
            real_limits,
            scan_ranges=close_scan,
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
