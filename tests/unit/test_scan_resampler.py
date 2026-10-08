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

from scan_resampler import (  # noqa: E402
    resample_forward_arc,
    ScanResampleResult,
    SAMPLING_MIN_POOL,
    SAMPLING_NEAREST_RAY,
)

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
        sampling=SAMPLING_MIN_POOL,
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
        sampling=SAMPLING_MIN_POOL,
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
        sampling=SAMPLING_MIN_POOL,
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
        sampling=SAMPLING_MIN_POOL,
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
        sampling=SAMPLING_MIN_POOL,
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


# --- sampling mode: training fidelity vs conservatism -----------------------
# Governing principle (operator, 2026-08-27): match what the policy saw in
# training as closely as possible. Training used --beams 32 --fov 180, i.e.
# ONE ray per beam direction, so SAMPLING_NEAREST_RAY is the faithful mode and
# is the module default. SAMPLING_MIN_POOL stays available for any consumer
# that wants conservatism instead of fidelity.


def test_default_sampling_is_nearest_ray_for_training_fidelity() -> None:
    """The DEFAULT must be the training-faithful mode, not the conservative one.

    Pinned deliberately: if someone flips the default to min_pool, every
    observation silently shifts toward 'obstacles closer and wider than
    training', with no error anywhere.
    """
    n_bins, n_points = 4, 400
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    ranges = [10.0] * n_points
    ranges[n_points // 2] = 0.05  # a close ray that is NOT at a bin centre

    default = resample_forward_arc(
        ranges, angle_min, angle_increment, n_bins=n_bins, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
    )
    explicit = resample_forward_arc(
        ranges, angle_min, angle_increment, n_bins=n_bins, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_NEAREST_RAY,
    )
    assert default.bins == explicit.bins


def test_nearest_ray_ignores_a_closer_ray_that_is_not_at_the_bin_centre() -> None:
    """The defining difference between the modes.

    A 32-beam lidar pointed at the bin centre would NOT see an obstacle that
    sits off-centre within the same 5.625 deg sector. min_pool would.
    """
    n_bins, n_points = 4, 400
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    ranges = [10.0] * n_points
    ranges[n_points // 2] = 0.05

    near = resample_forward_arc(
        ranges, angle_min, angle_increment, n_bins=n_bins, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_NEAREST_RAY,
    )
    pool = resample_forward_arc(
        ranges, angle_min, angle_increment, n_bins=n_bins, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_MIN_POOL,
    )
    assert near.fault is None and pool.fault is None
    assert min(pool.bins) == pytest.approx(0.05), "min_pool must see it"
    assert min(near.bins) == pytest.approx(10.0), (
        "nearest_ray must NOT see an off-centre ray — that is the whole point"
    )


def test_modes_agree_when_every_ray_in_a_bin_is_identical() -> None:
    """With a uniform scan the two modes cannot disagree — a sanity floor."""
    n_bins, n_points = 8, 400
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    ranges = [3.25] * n_points
    kw = dict(n_bins=n_bins, arc_deg=180.0,
              range_min=RANGE_MIN_M, range_max=RANGE_MAX_M)
    a = resample_forward_arc(ranges, angle_min, angle_increment,
                             sampling=SAMPLING_NEAREST_RAY, **kw)
    b = resample_forward_arc(ranges, angle_min, angle_increment,
                             sampling=SAMPLING_MIN_POOL, **kw)
    assert a.bins == b.bins


def test_invalid_sampling_mode_faults_closed() -> None:
    scan = _synthetic_scan(fill_range=10.0)
    result = resample_forward_arc(
        scan["ranges"], scan["angle_min"], scan["angle_increment"],
        n_bins=32, arc_deg=ARC_DEG,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling="average",  # not a mode we support
    )
    assert result.bins is None
    assert result.fault == "invalid_sampling_mode"


def test_modes_differ_materially_on_real_tg30_data() -> None:
    """On a real cluttered scan the choice of mode is not academic.

    If this ever stops holding, either the fixture changed or the modes have
    silently converged (e.g. a forwarding bug making `sampling` a no-op --
    which is exactly the defect found on 2026-08-27).
    """
    scans = _load_scans(SAMPLE_FIXTURE)
    scan = scans[0]
    kw = dict(n_bins=32, arc_deg=ARC_DEG,
              range_min=RANGE_MIN_M, range_max=RANGE_MAX_M)
    near = resample_forward_arc(scan["ranges"], scan["angle_min"],
                                scan["angle_increment"],
                                sampling=SAMPLING_NEAREST_RAY, **kw)
    pool = resample_forward_arc(scan["ranges"], scan["angle_min"],
                                scan["angle_increment"],
                                sampling=SAMPLING_MIN_POOL, **kw)
    assert near.fault is None and pool.fault is None
    differing = sum(1 for x, y in zip(near.bins, pool.bins) if x != y)
    assert differing > 0, "modes must not be a no-op on real data"
    # min_pool can only ever be <= nearest_ray, never further away.
    for x, y in zip(near.bins, pool.bins):
        assert y <= x + 1e-9


def test_nearest_ray_skips_a_dropout_centre_ray() -> None:
    """A dropout at the bin centre must NOT make the bin read as clear.

    The TG30 drops ~39% of rays; simulation dropped none. Accepting a dropout
    as the centre sample would report range_max ("clear") for a sector where
    real returns exist - a failure mode training never contained. Measured on
    tg30_open_scan.yaml, 16% of bins have a dropout centre ray.
    """
    n_bins, n_points = 4, 400
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    ranges = [2.0] * n_points

    # Kill the ray nearest each bin centre, leaving its neighbours real.
    half, bw = math.pi / 2, math.pi / n_bins
    best = [(9e9, -1)] * n_bins
    for i in range(n_points):
        a = angle_min + i * angle_increment
        if a < -half or a > half:
            continue
        idx = min(int((a + half) / bw), n_bins - 1)
        off = abs(a - (-half + (idx + 0.5) * bw))
        if off < best[idx][0]:
            best[idx] = (off, i)
    for _, i in best:
        ranges[i] = 0.0  # dropout exactly at the centre

    result = resample_forward_arc(
        ranges, angle_min, angle_increment, n_bins=n_bins, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_NEAREST_RAY,
    )
    assert result.fault is None
    assert all(b == pytest.approx(2.0) for b in result.bins), (
        f"dropout centre rays leaked through as 'clear': {result.bins}"
    )


def test_bin_with_only_dropouts_falls_back_to_range_max() -> None:
    """Nothing detected anywhere in the sector really is range_max."""
    n_bins, n_points = 4, 400
    angle_min = -math.pi / 2
    angle_increment = math.pi / (n_points - 1)
    ranges = [0.0] * n_points  # every ray a dropout

    result = resample_forward_arc(
        ranges, angle_min, angle_increment, n_bins=n_bins, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_NEAREST_RAY,
    )
    assert result.fault is None
    assert all(b == pytest.approx(RANGE_MAX_M) for b in result.bins)
    assert all(math.isfinite(b) for b in result.bins), "must never be inf"


# ---------------------------------------------------------------------------
# Interface 4.0.0: nearest-ray search window and self-occlusion mask zones
# ---------------------------------------------------------------------------

# The 4.0.0 lidar block (config/interface.yaml): 720 bins over 270 deg, ~2
# TG30 rays per bin, and the chassis standoff sectors the training sensor
# could not see.
BEAMS_720 = 720
ARC_270 = 270.0
ZONES_DEG = [[92.0, 123.0], [-135.0, -97.0]]


def _edge_to_edge_scan(n_bins: int, per_bin: int, arc_deg: float, value: float):
    """A forward-arc-only scan whose first and last rays sit exactly on the
    arc edges (the convention the module's coverage check expects), with
    `per_bin` rays landing in every bin for these small sizes."""
    n_points = n_bins * per_bin
    arc = math.radians(arc_deg)
    return [value] * n_points, -arc / 2.0, arc / (n_points - 1)


def _nearest(ranges, angle_min, inc, n_bins, arc_deg, **kw):
    return resample_forward_arc(
        ranges, angle_min, inc, n_bins=n_bins, arc_deg=arc_deg,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_NEAREST_RAY, **kw,
    )


def test_window_fallback_borrows_the_angularly_nearest_real_return() -> None:
    """An all-dropout bin takes the neighbour ray nearest its CENTRE, which is
    not the same as the nearest obstacle: nearest by angle, never a min."""
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 2.0)
    ranges[4] = ranges[5] = 0.0      # bin 2: both rays dropouts
    ranges[3] = 1.0                   # bin 1's upper ray, 0.86 bins from centre
    ranges[6] = 3.0                   # bin 3's lower ray, 0.77 bins from centre
    strict = _nearest(ranges, amin, inc, 6, 180.0, search_window_bins=0)
    window = _nearest(ranges, amin, inc, 6, 180.0, search_window_bins=1)
    assert strict.fault is None and window.fault is None
    assert strict.bins[2] == pytest.approx(RANGE_MAX_M)
    assert window.bins[2] == pytest.approx(3.0), window.bins


def test_window_reach_is_window_plus_half_a_bin() -> None:
    """Reach is (window + 0.5) bins from the centre: a ray 1.86 bins away is
    out of reach for window 1 and in reach for window 2."""
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 2.0)
    for i in range(2, 8):             # bins 1, 2, 3 all dropouts
        ranges[i] = 0.0
    one = _nearest(ranges, amin, inc, 6, 180.0, search_window_bins=1)
    two = _nearest(ranges, amin, inc, 6, 180.0, search_window_bins=2)
    assert one.bins[2] == pytest.approx(RANGE_MAX_M)
    assert two.bins[2] == pytest.approx(2.0)


def test_window_never_overrides_a_bin_that_has_its_own_real_return() -> None:
    """Rays inside the bin always win; the window is a fallback only."""
    for scan in _all_fixture_scans():
        strict = _nearest(scan["ranges"], scan["angle_min"], scan["angle_increment"],
                          BEAMS_720, ARC_270, search_window_bins=0)
        window = _nearest(scan["ranges"], scan["angle_min"], scan["angle_increment"],
                          BEAMS_720, ARC_270, search_window_bins=2)
        assert strict.fault is None and window.fault is None
        for s, w in zip(strict.bins, window.bins):
            if s != RANGE_MAX_M:
                assert w == s


def test_window_is_ignored_in_min_pool_mode() -> None:
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 2.0)
    ranges[4] = ranges[5] = 0.0
    result = resample_forward_arc(
        ranges, amin, inc, n_bins=6, arc_deg=180.0,
        range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
        sampling=SAMPLING_MIN_POOL, search_window_bins=2,
    )
    assert result.fault is None
    assert result.bins[2] == pytest.approx(RANGE_MAX_M)


def test_window_may_borrow_a_real_ray_from_just_outside_the_arc() -> None:
    """An edge bin can use a ray beyond the arc edge; the TG30 covers 360 deg
    so those rays exist. Here bins 0-1 (angles -90..-86 deg) are dropouts and
    the only nearby real return sits at -90.5 deg."""
    n_points = 720                     # 0.5 deg rays over the full circle
    inc = 2.0 * math.pi / n_points
    ranges = [2.0] * n_points
    for i in range(180, 188):          # -90.0 .. -86.5 deg
        ranges[i] = 0.0
    ranges[179] = 1.5                  # -90.5 deg, outside the 180 deg arc
    strict = _nearest(ranges, -math.pi, inc, 90, 180.0, search_window_bins=0)
    window = _nearest(ranges, -math.pi, inc, 90, 180.0, search_window_bins=1)
    assert strict.bins[0] == pytest.approx(RANGE_MAX_M)
    assert window.bins[0] == pytest.approx(1.5)


def test_window_cuts_empty_bins_on_real_tg30_scans_at_720_bins() -> None:
    """The measurement that motivates the option: strict per-bin sampling
    leaves 5-8% of 720 bins with no real return on the recorded fixtures;
    +/-2 bins leaves ~1.3%."""
    for scan in _all_fixture_scans():
        strict = _nearest(scan["ranges"], scan["angle_min"], scan["angle_increment"],
                          BEAMS_720, ARC_270, search_window_bins=0)
        window = _nearest(scan["ranges"], scan["angle_min"], scan["angle_increment"],
                          BEAMS_720, ARC_270, search_window_bins=2)
        empty_strict = sum(1 for b in strict.bins if b == RANGE_MAX_M) / BEAMS_720
        empty_window = sum(1 for b in window.bins if b == RANGE_MAX_M) / BEAMS_720
        assert empty_strict >= 0.04, empty_strict
        assert empty_window <= 0.025, empty_window
        assert empty_window < empty_strict


def test_mask_zone_blanks_its_rays_in_both_modes_without_an_empty_bin_fault() -> None:
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 0.5)
    bin0 = [-90.0, -90.0 + 180.0 / 6]  # exactly bin 0's angular extent
    for mode in (SAMPLING_NEAREST_RAY, SAMPLING_MIN_POOL):
        result = resample_forward_arc(
            ranges, amin, inc, n_bins=6, arc_deg=180.0,
            range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
            sampling=mode, mask_zones_deg=[bin0],
        )
        assert result.fault is None, result.fault
        assert result.bins[0] == pytest.approx(RANGE_MAX_M)
        assert all(b == pytest.approx(0.5) for b in result.bins[1:])


def test_masked_rays_never_feed_the_window_fallback() -> None:
    """A masked sector next to a dropout bin must not leak its (chassis)
    range into that bin through the window."""
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 2.0)
    ranges[0] = ranges[1] = 0.1        # bin 0: chassis-like returns, masked
    ranges[2] = ranges[3] = 0.0        # bin 1: dropouts
    bin0 = [-90.0, -90.0 + 180.0 / 6]
    result = _nearest(ranges, amin, inc, 6, 180.0,
                      search_window_bins=1, mask_zones_deg=[bin0])
    assert result.fault is None
    assert result.bins[0] == pytest.approx(RANGE_MAX_M)
    assert result.bins[1] == pytest.approx(2.0)   # borrowed from bin 2, not bin 0


def test_mask_zones_blank_the_standoff_sectors_on_real_scans() -> None:
    """With the 4.0.0 zones, every 720-bin whose centre lies inside a zone by
    more than the window's reach reads range_max on the recorded fixtures."""
    bin_deg = ARC_270 / BEAMS_720
    reach_deg = (2 + 0.5) * bin_deg
    for scan in _all_fixture_scans():
        result = _nearest(scan["ranges"], scan["angle_min"], scan["angle_increment"],
                          BEAMS_720, ARC_270, search_window_bins=2,
                          mask_zones_deg=ZONES_DEG)
        assert result.fault is None, result.fault
        for i, b in enumerate(result.bins):
            centre = -ARC_270 / 2 + (i + 0.5) * bin_deg
            deep_inside = any(lo + reach_deg < centre < hi - reach_deg for lo, hi in ZONES_DEG)
            if deep_inside:
                assert b == RANGE_MAX_M, (i, centre, b)


@pytest.mark.parametrize("window", [-1, 1.5, "2", None])
def test_invalid_search_window_faults_closed(window) -> None:
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 2.0)
    result = _nearest(ranges, amin, inc, 6, 180.0, search_window_bins=window)
    assert result.bins is None
    assert result.fault == "invalid_search_window"


@pytest.mark.parametrize(
    "zones",
    [[[10.0]], [[10.0, 5.0]], [[float("nan"), 1.0]], [[0.0, float("inf")]], [["a", "b"]], [5.0]],
)
def test_invalid_mask_zone_faults_closed(zones) -> None:
    ranges, amin, inc = _edge_to_edge_scan(6, 2, 180.0, 2.0)
    result = _nearest(ranges, amin, inc, 6, 180.0, mask_zones_deg=zones)
    assert result.bins is None
    assert result.fault == "invalid_mask_zone"
