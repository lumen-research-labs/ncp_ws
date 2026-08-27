"""
test_scan_resampler.py — Unit tests for rm_policy.scan_resampler.

Pure pytest, no ROS runtime. Uses the real TG30 LaserScan fixtures
(`tests/fixtures/tg30_scan_sample.yaml`, `tests/fixtures/tg30_open_scan.yaml`)
to regression-guard the 39%-zeros no-return handling against genuine hardware
data, plus synthetic scans for deterministic geometry/ordering checks.
"""

from __future__ import annotations

import math
import os
from typing import Dict, Iterator, List

import pytest
import yaml

import sys

_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
sys.path.insert(
    0, os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy")
)

from scan_resampler import resample_forward_arc, ScanResampleResult  # noqa: E402

FIXTURES_DIR = os.path.join(_REPO_ROOT, "tests", "fixtures")
SAMPLE_FIXTURE = os.path.join(FIXTURES_DIR, "tg30_scan_sample.yaml")
OPEN_FIXTURE = os.path.join(FIXTURES_DIR, "tg30_open_scan.yaml")

# interface.yaml conventions used throughout (not magic — mirrors the frozen
# contract's lidar block; tests exercise both 32 and 36 to prove the module
# isn't hardcoded to either).
RANGE_MIN_M = 0.01
RANGE_MAX_M = 30.0
ARC_DEG = 180.0


def _load_scans(path: str) -> List[Dict]:
    with open(path, "r") as f:
        docs = list(yaml.safe_load_all(f))
    # Multi-document YAML files may have a trailing empty document (a bare
    # "---" with nothing after it parses to None) -- drop it rather than
    # letting it masquerade as a scan.
    return [d for d in docs if d is not None]


def _all_fixture_scans() -> Iterator[Dict]:
    for path in (SAMPLE_FIXTURE, OPEN_FIXTURE):
        for scan in _load_scans(path):
            yield scan


# ---------------------------------------------------------------------------
# Real-fixture regression tests
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("n_bins", [32, 36])
def test_output_length_matches_n_bins_on_real_scans(n_bins: int) -> None:
    scan = next(_all_fixture_scans())
    result = resample_forward_arc(
        scan["ranges"],
        scan["angle_min"],
        scan["angle_increment"],
        n_bins=n_bins,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result.fault is None, result.fault
    assert result.bins is not None
    assert len(result.bins) == n_bins


@pytest.mark.parametrize("n_bins", [32, 36])
def test_no_bin_ever_zero_or_nonfinite_across_all_fixture_scans(
    n_bins: int,
) -> None:
    """Core regression: the TG30's ~39% zero no-returns must never survive
    into the output as zero-distance or non-finite bins."""
    checked = 0
    for scan in _all_fixture_scans():
        result = resample_forward_arc(
            scan["ranges"],
            scan["angle_min"],
            scan["angle_increment"],
            n_bins=n_bins,
            arc_deg=ARC_DEG,
            range_min=RANGE_MIN_M,
            range_max=RANGE_MAX_M,
        )
        assert result.fault is None, f"unexpected fault: {result.fault}"
        assert result.bins is not None
        for b in result.bins:
            assert math.isfinite(b), f"non-finite bin {b}"
            assert b != 0.0, "a bin read exactly 0.0 -- no-return leaked through"
            assert b > 0.0
        checked += 1
    assert checked > 0


def test_fixtures_contain_real_zero_no_returns() -> None:
    """Sanity check that the fixtures actually exercise the hazard: if this
    ever fails, the fixtures changed and the above test is no longer a real
    regression guard."""
    scan = next(_all_fixture_scans())
    zero_count = sum(1 for r in scan["ranges"] if r == 0.0)
    assert zero_count > 0


# ---------------------------------------------------------------------------
# All-zero scan -> fail closed, never a zero-distance obstacle
# ---------------------------------------------------------------------------


def test_all_zero_scan_never_reads_as_zero_distance_obstacle() -> None:
    n_points = 1000
    angle_min = -math.pi
    angle_increment = (2 * math.pi) / (n_points - 1)
    ranges = [0.0] * n_points

    result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=36,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )

    # Either fails closed (fault) or reports every bin at range_max --
    # either way, never a near-zero "obstacle".
    if result.fault is not None:
        assert result.bins is None
    else:
        assert all(b == pytest.approx(RANGE_MAX_M) for b in result.bins)


# ---------------------------------------------------------------------------
# Synthetic geometry / ordering / min-pooling tests
# ---------------------------------------------------------------------------


def _synthetic_scan(
    n_points: int = 2020,
    angle_min: float = -math.pi,
    fill_range: float = 10.0,
) -> Dict:
    angle_increment = (2 * math.pi) / (n_points - 1)
    ranges = [fill_range] * n_points
    return {
        "angle_min": angle_min,
        "angle_increment": angle_increment,
        "ranges": ranges,
    }


def test_single_close_obstacle_lands_in_correct_bin_neighbours_unaffected() -> None:
    n_bins = 36
    scan = _synthetic_scan(fill_range=10.0)
    angle_min = scan["angle_min"]
    angle_increment = scan["angle_increment"]
    ranges = scan["ranges"]

    # Place a close obstacle exactly at forward (angle 0).
    target_angle = 0.0
    idx = round((target_angle - angle_min) / angle_increment)
    ranges[idx] = 0.5

    result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=n_bins,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result.fault is None, result.fault
    bins = result.bins

    # Derive the expected bin from first principles, NOT from the module's own
    # formula, so a consistent off-by-one in the module cannot satisfy this test.
    # The obstacle sits at the exact angle of beam `idx`; the arc spans
    # [-ARC_DEG/2, +ARC_DEG/2] split into n_bins equal sectors.
    import math as _m
    actual_angle = angle_min + idx * angle_increment
    half_arc = _m.radians(ARC_DEG) / 2.0
    bin_width = _m.radians(ARC_DEG) / n_bins
    expected_bin = int((actual_angle + half_arc) / bin_width)
    expected_bin = min(expected_bin, n_bins - 1)

    assert bins[expected_bin] == pytest.approx(0.5), (
        f"obstacle at {actual_angle:.6f} rad expected in bin {expected_bin}, "
        f"got bins={bins[max(0, expected_bin - 1):expected_bin + 2]}"
    )

    # EVERY other bin must be untouched — not just the far ones. This is what
    # makes an off-by-one detectable: a neighbour holding 0.5 now fails.
    for i, b in enumerate(bins):
        if i == expected_bin:
            continue
        assert b == pytest.approx(10.0), f"bin {i} contaminated: {b}"


def test_ray_exactly_at_range_min_is_kept_as_real_data() -> None:
    """Boundary case, deliberately divergent from rm_safety.safety_logic.

    `scan_resampler._is_no_return` uses `r < range_min`, so a ray reading
    EXACTLY range_min is real data and survives to the policy. safety_logic
    uses `r > scan_range_min` and discards the same ray. The divergence is
    documented in `_is_no_return`; this test pins our side of it so the
    behaviour cannot drift silently.
    """
    n_bins = 36
    scan = _synthetic_scan(fill_range=10.0)
    angle_min = scan["angle_min"]
    angle_increment = scan["angle_increment"]
    ranges = scan["ranges"]

    idx = round((0.0 - angle_min) / angle_increment)
    ranges[idx] = RANGE_MIN_M  # exactly the boundary

    result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=n_bins,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result.fault is None, result.fault
    # Kept as a real (very close) obstacle — NOT mapped to range_max.
    assert min(result.bins) == pytest.approx(RANGE_MIN_M)


@pytest.mark.parametrize("value", [-0.0, 5e-324, 1e-300])
def test_negative_zero_and_subnormals_are_no_returns(value: float) -> None:
    """-0.0 and subnormal floats must map to range_max, not a zero-distance hit."""
    n_bins = 36
    scan = _synthetic_scan(fill_range=10.0)
    angle_min = scan["angle_min"]
    angle_increment = scan["angle_increment"]
    ranges = scan["ranges"]

    idx = round((0.0 - angle_min) / angle_increment)
    ranges[idx] = value

    result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=n_bins,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result.fault is None, result.fault
    assert all(b == pytest.approx(10.0) for b in result.bins), (
        f"{value!r} leaked through as an obstacle: min bin {min(result.bins)}"
    )


def test_ordering_right_to_left() -> None:
    """Right (starboard, negative angle) -> low index; left (port, positive
    angle) -> high index."""
    n_bins = 36
    scan = _synthetic_scan(fill_range=10.0)
    angle_min = scan["angle_min"]
    angle_increment = scan["angle_increment"]

    # Right-only obstacle: place a close return at -80 degrees.
    right_ranges = list(scan["ranges"])
    right_angle = math.radians(-80.0)
    right_idx = round((right_angle - angle_min) / angle_increment)
    right_ranges[right_idx] = 0.3

    result_right = resample_forward_arc(
        right_ranges,
        angle_min,
        angle_increment,
        n_bins=n_bins,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result_right.fault is None
    right_min_idx = min(
        range(n_bins), key=lambda i: result_right.bins[i]
    )
    assert right_min_idx < n_bins // 2

    # Left-only obstacle: +80 degrees.
    left_ranges = list(scan["ranges"])
    left_angle = math.radians(80.0)
    left_idx = round((left_angle - angle_min) / angle_increment)
    left_ranges[left_idx] = 0.3

    result_left = resample_forward_arc(
        left_ranges,
        angle_min,
        angle_increment,
        n_bins=n_bins,
        arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result_left.fault is None
    left_min_idx = min(range(n_bins), key=lambda i: result_left.bins[i])
    assert left_min_idx > n_bins // 2


def test_min_pooling_not_mean() -> None:
    """A single close ray in a bin with many far rays must keep the bin's
    minimum, not average it away."""
    n_bins = 4
    n_points = 400
    angle_min = -math.pi / 2  # exactly covers a 180 deg arc
    angle_increment = math.pi / (n_points - 1)
    ranges = [10.0] * n_points
    # Put one very close ray in the middle of the array (falls in an inner bin).
    ranges[n_points // 2] = 0.05

    result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=n_bins,
        arc_deg=180.0,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result.fault is None
    assert min(result.bins) == pytest.approx(0.05)
    # Other bins should still be ~10.0 (mean would have pulled the min-bin way down).
    below_far = [b for b in result.bins if b < 5.0]
    assert len(below_far) == 1


# ---------------------------------------------------------------------------
# No-return mapping tests
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad_value",
    [0.0, math.inf, -math.inf, math.nan, -1.0, RANGE_MIN_M / 2.0],
)
def test_no_return_values_map_to_range_max_not_near_zero(bad_value: float) -> None:
    n_points = 400
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    ranges = [bad_value] * n_points

    result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=4,
        arc_deg=180.0,
        range_min=RANGE_MIN_M,
        range_max=RANGE_MAX_M,
    )
    assert result.fault is None, result.fault
    for b in result.bins:
        assert b == pytest.approx(RANGE_MAX_M)


# ---------------------------------------------------------------------------
# Fault-path tests (requirement 7)
# ---------------------------------------------------------------------------


def test_fault_on_none_ranges() -> None:
    result = resample_forward_arc(
        None, -math.pi, 0.01, n_bins=36, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "empty_ranges"
    assert result.bins is None


def test_fault_on_empty_ranges() -> None:
    result = resample_forward_arc(
        [], -math.pi, 0.01, n_bins=36, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "empty_ranges"


@pytest.mark.parametrize("bad_incr", [0.0, -0.01, math.inf, math.nan])
def test_fault_on_invalid_angle_increment(bad_incr: float) -> None:
    result = resample_forward_arc(
        [1.0] * 100, -math.pi, bad_incr, n_bins=36, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "invalid_angle_increment"


def test_fault_on_nonfinite_angle_min() -> None:
    result = resample_forward_arc(
        [1.0] * 100, math.nan, 0.01, n_bins=36, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "invalid_angle_increment"


@pytest.mark.parametrize("bad_n", [0, -1, -36])
def test_fault_on_invalid_n_bins(bad_n: int) -> None:
    result = resample_forward_arc(
        [1.0] * 2020, -math.pi, 0.0031, n_bins=bad_n, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "invalid_n_bins"


@pytest.mark.parametrize("bad_arc", [0.0, -10.0, 361.0, math.nan])
def test_fault_on_invalid_arc_deg(bad_arc: float) -> None:
    result = resample_forward_arc(
        [1.0] * 2020, -math.pi, 0.0031, n_bins=36, arc_deg=bad_arc,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "invalid_arc_deg"


@pytest.mark.parametrize(
    "range_min,range_max",
    [(1.0, 1.0), (5.0, 1.0), (math.nan, 30.0), (0.01, math.inf)],
)
def test_fault_on_invalid_range_bounds(range_min: float, range_max: float) -> None:
    result = resample_forward_arc(
        [1.0] * 2020, -math.pi, 0.0031, n_bins=36, arc_deg=180.0,
        range_min=range_min, range_max=range_max,
    )
    assert result.fault == "invalid_range_bounds"


def test_fault_on_arc_wider_than_scan_coverage() -> None:
    # Scan only covers ~10 degrees total, but we request the full 180.
    n_points = 10
    angle_increment = math.radians(1.0)
    result = resample_forward_arc(
        [1.0] * n_points, 0.0, angle_increment, n_bins=4, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "arc_exceeds_scan_coverage"


def test_fault_on_empty_bin_when_resolution_too_coarse() -> None:
    # 4 points spanning 180 deg (60 deg apart) but ask for 36 bins (5 deg
    # wide each) -> most bins get zero member rays.
    n_points = 4
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    result = resample_forward_arc(
        [1.0] * n_points, angle_min, angle_increment, n_bins=36, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault is not None
    assert result.fault.startswith("empty_bin:")
    assert result.bins is None


def test_internal_error_is_caught_and_fails_closed() -> None:
    # Passing a non-numeric ranges element should be caught by the top-level
    # try/except and reported as internal_error, never raise out.
    ranges = [1.0] * 50
    ranges[10] = "not-a-number"  # type: ignore[list-item]
    n_points = 50
    angle_increment = math.pi / (n_points - 1)
    result = resample_forward_arc(
        ranges, -math.pi / 2, angle_increment, n_bins=4, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    assert result.fault == "internal_error"
    assert result.bins is None
