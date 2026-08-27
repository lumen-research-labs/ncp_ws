"""
test_observation_builder.py — Unit tests for rm_policy.observation_builder.

Pure pytest, no ROS runtime, stdlib only (see module note below for the one
exception). Covers block boundaries, hand-computed known values, a real TG30
scan fixture, the documented out-of-range goal normalization examples, the
previous_action(0,0) non-zero-normalization pin, every fault path, and a
numpy conformance check against the vendored (numpy) reference formulas.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict, List

import pytest
import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.observation_builder import (  # noqa: E402
    ACTION_BIAS_V,
    ACTION_BIAS_W,
    ACTION_SCALE_V,
    ACTION_SCALE_W,
    LASER_CLIP_M,
    LIDAR_BEAMS,
    OBS_DIM,
    ObservationResult,
    build_observation,
)

FIXTURES_DIR = os.path.join(_REPO_ROOT, "tests", "fixtures")
SAMPLE_FIXTURE = os.path.join(FIXTURES_DIR, "tg30_scan_sample.yaml")

# The exact geometry the trained policy assumes: 32 rays across 180 deg,
# REP-103, matching X3_AMIN/X3_AINC in test_ncp_policy_vendored.py. With this
# geometry each ray lands exactly on a bin centre, so resample_forward_arc's
# SAMPLING_NEAREST_RAY output is the identity permutation of the input scan
# (verified directly below) -- which is what lets the hand-computed tests
# reason about the input scan values directly.
X3_AMIN = -math.pi / 2
X3_AINC = math.pi / (LIDAR_BEAMS - 1)

DEFAULT_POSE = (0.0, 0.0)
DEFAULT_YAW = 0.0
DEFAULT_GOAL = (2.5, 0.0)
DEFAULT_ACTION = (0.0, 0.0)


def _load_scans(path: str) -> List[Dict]:
    with open(path, "r") as f:
        docs = list(yaml.safe_load_all(f))
    return [d for d in docs if d is not None]


def _build(
    ranges,
    angle_min=X3_AMIN,
    angle_increment=X3_AINC,
    pose_xy=DEFAULT_POSE,
    yaw=DEFAULT_YAW,
    goal_xy=DEFAULT_GOAL,
    previous_action=DEFAULT_ACTION,
) -> ObservationResult:
    return build_observation(
        ranges, angle_min, angle_increment, pose_xy, yaw, goal_xy, previous_action
    )


# ---------------------------------------------------------------------------
# Block boundaries
# ---------------------------------------------------------------------------


def test_total_length_is_36():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan)
    assert result.fault is None
    assert len(result.obs) == 36 == OBS_DIM


def test_lidar_occupies_0_to_32():
    scan = [1.0] * LIDAR_BEAMS  # -> normalized -0.6 in every lidar slot
    result = _build(scan, goal_xy=(0.0, 0.0), previous_action=(0.0, 0.0))
    assert result.fault is None
    expected_lidar = (1.0 - LASER_CLIP_M / 2) / LASER_CLIP_M * 2
    assert all(abs(v - expected_lidar) < 1e-9 for v in result.obs[0:32])


def test_goal_occupies_32_to_34():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, pose_xy=(0.0, 0.0), yaw=0.0, goal_xy=(0.0, 0.0))
    assert result.fault is None
    assert result.obs[32:34] == pytest.approx([-1.0, -1.0])


def test_previous_action_occupies_34_to_36():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, previous_action=(1.5, 0.0))
    assert result.fault is None
    # v=1.5 -> (1.5-0.5)/1.5 = 0.666...
    assert result.obs[34] == pytest.approx(2.0 / 3.0)
    assert result.obs[35] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Hand-computed known values
# ---------------------------------------------------------------------------


def test_resample_identity_at_x3_geometry():
    """Sanity precondition every other hand-computed test relies on: at the
    exact trained geometry, resample_forward_arc is the identity permutation
    of the input scan (each ray lands exactly on its own bin centre)."""
    from rm_policy.scan_resampler import SAMPLING_NEAREST_RAY, resample_forward_arc

    scan = [0.3 + 0.1 * i for i in range(LIDAR_BEAMS)]
    res = resample_forward_arc(
        scan,
        X3_AMIN,
        X3_AINC,
        n_bins=LIDAR_BEAMS,
        arc_deg=180.0,
        range_min=0.01,
        range_max=30.0,
        sampling=SAMPLING_NEAREST_RAY,
    )
    assert res.fault is None
    assert res.bins == pytest.approx(scan, abs=1e-9)


def test_mid_clip_lidar_normalizes_to_zero():
    scan = [LASER_CLIP_M / 2.0] * LIDAR_BEAMS  # 2.5 m -> 0.0
    result = _build(scan)
    assert result.fault is None
    assert all(v == pytest.approx(0.0, abs=1e-9) for v in result.obs[0:32])


def test_beyond_clip_lidar_normalizes_to_plus_one():
    scan = [10.0] * LIDAR_BEAMS  # beyond LASER_CLIP_M (5.0), but < range_max (30.0)
    result = _build(scan)
    assert result.fault is None
    assert all(v == pytest.approx(1.0) for v in result.obs[0:32])


def test_touching_lidar_normalizes_to_minus_one():
    scan = [0.02] * LIDAR_BEAMS  # just above range_min, near zero
    result = _build(scan)
    assert result.fault is None
    expected = (0.02 - LASER_CLIP_M / 2) / LASER_CLIP_M * 2
    assert all(v == pytest.approx(expected) for v in result.obs[0:32])
    assert expected < -0.99


# ---------------------------------------------------------------------------
# Real TG30 fixture (no-return heavy)
# ---------------------------------------------------------------------------


def test_real_tg30_scan_produces_finite_in_range_observation():
    scans = _load_scans(SAMPLE_FIXTURE)
    scan = scans[0]
    result = build_observation(
        scan["ranges"],
        scan["angle_min"],
        scan["angle_increment"],
        pose_xy=(0.0, 0.0),
        yaw=0.0,
        goal_xy=(2.5, 0.0),
        previous_action=(0.0, 0.0),
    )
    assert result.fault is None, result.fault
    assert len(result.obs) == OBS_DIM
    assert all(math.isfinite(v) for v in result.obs)
    # Every lidar slot must be inside the normalized [-1, 1] range even though
    # ~27-39% of raw rays in this fixture are TG30 zero-no-returns.
    assert all(-1.0 - 1e-9 <= v <= 1.0 + 1e-9 for v in result.obs[0:32])


# ---------------------------------------------------------------------------
# Goal normalization, including documented out-of-range examples
# ---------------------------------------------------------------------------


def test_goal_directly_behind_gives_minus_two():
    """config/interface.yaml normalization_warning: 'a goal directly behind
    gives -2.00'. Pinned here as a regression -- this is EXPECTED, not a bug
    to fix. Goal 5 m directly behind (robot facing +x, yaw 0) -> local
    x = -5.0 -> -5.0/5.0 - 1.0 = -2.0."""
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, pose_xy=(0.0, 0.0), yaw=0.0, goal_xy=(-5.0, 0.0))
    assert result.fault is None
    assert result.obs[32] == pytest.approx(-2.0)
    assert result.obs[33] == pytest.approx(-1.0)


def test_goal_45_deg_off_gives_minus_1p71():
    """config/interface.yaml normalization_warning: 'a goal 45 deg off the
    bow gives -1.71'. Reproduced here as a regression pin for the raw
    normalization arithmetic, not as a claim about a specific physical
    bearing: local x = -5*cos(45deg) = -3.5355 -> -3.5355/5 - 1 = -1.7071."""
    local_x = -5.0 * math.cos(math.radians(45))
    local_y = 5.0 * math.sin(math.radians(45))
    scan = [2.5] * LIDAR_BEAMS
    # pose at origin, yaw 0 -> local_goal is just goal_xy unchanged.
    result = _build(scan, pose_xy=(0.0, 0.0), yaw=0.0, goal_xy=(local_x, local_y))
    assert result.fault is None
    assert result.obs[32] == pytest.approx(-1.71, abs=5e-3)


def test_local_goal_rotates_into_robot_frame():
    """Robot at origin facing +y (yaw 90 deg); goal 10 m up the y axis is
    dead ahead -> local (10, 0)."""
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, pose_xy=(0.0, 0.0), yaw=math.pi / 2, goal_xy=(0.0, 10.0))
    assert result.fault is None
    assert result.obs[32] == pytest.approx(10.0 / 5.0 - 1.0)
    assert result.obs[33] == pytest.approx(0.0 / 5.0 - 1.0)


# ---------------------------------------------------------------------------
# previous_action(0, 0) normalizes to a NON-zero vector
# ---------------------------------------------------------------------------


def test_zero_previous_action_normalizes_to_nonzero():
    """config/interface.yaml init_note: a zero ACTION normalizes to a
    non-zero vector (v=0 -> -0.333, w=0 -> 0.0). Must NOT be 'fixed' to
    zeros."""
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, previous_action=(0.0, 0.0))
    assert result.fault is None
    assert result.obs[34] == pytest.approx(-1.0 / 3.0)
    assert result.obs[35] == pytest.approx(0.0)
    assert result.obs[34] != 0.0


def test_action_bias_scale_constants_match_bounds():
    assert ACTION_BIAS_V == pytest.approx(0.5)
    assert ACTION_SCALE_V == pytest.approx(1.5)
    assert ACTION_BIAS_W == pytest.approx(0.0)
    assert ACTION_SCALE_W == pytest.approx(3.14)


# ---------------------------------------------------------------------------
# Fault paths
# ---------------------------------------------------------------------------


def test_fault_on_empty_ranges_propagates_lidar_reason():
    result = _build(None)
    assert result.obs is None
    assert result.fault == "lidar:empty_ranges"


def test_fault_on_lidar_internal_error_propagates():
    # angle_increment <= 0 is a scan_resampler precondition failure.
    result = _build([1.0] * LIDAR_BEAMS, angle_increment=0.0)
    assert result.obs is None
    assert result.fault == "lidar:invalid_angle_increment"


def test_fault_on_missing_goal():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, goal_xy=None)
    assert result.obs is None
    assert result.fault == "goal_missing"


def test_fault_on_missing_pose_with_goal_defined():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, pose_xy=None, goal_xy=(2.5, 0.0))
    assert result.obs is None
    assert result.fault == "pose_missing"


def test_fault_on_missing_yaw_with_goal_defined():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, yaw=None, goal_xy=(2.5, 0.0))
    assert result.obs is None
    assert result.fault == "pose_missing"


def test_fault_never_substitutes_default_goal():
    """Belt-and-braces on the same requirement as the two tests above: a
    missing pose must never silently produce SOME 36-length vector."""
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, pose_xy=None)
    assert result.obs is None


def test_fault_on_non_finite_pose():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, pose_xy=(float("nan"), 0.0))
    assert result.obs is None
    assert result.fault == "non_finite_pose_or_goal"


def test_fault_on_non_finite_goal():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, goal_xy=(float("inf"), 0.0))
    assert result.obs is None
    assert result.fault == "non_finite_pose_or_goal"


def test_fault_on_missing_previous_action():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, previous_action=None)
    assert result.obs is None
    assert result.fault == "invalid_previous_action"


def test_fault_on_wrong_length_previous_action():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, previous_action=(0.0, 0.0, 0.0))
    assert result.obs is None
    assert result.fault == "invalid_previous_action"


def test_fault_on_non_finite_previous_action():
    scan = [2.5] * LIDAR_BEAMS
    result = _build(scan, previous_action=(float("nan"), 0.0))
    assert result.obs is None
    assert result.fault == "non_finite_previous_action"


def test_fault_on_unexpected_exception_is_internal_error():
    scan = [2.5] * LIDAR_BEAMS
    # goal_xy of a non-iterable type blows up _all_finite()/_local_goal()
    # inside the inner builder; the outer try/except must still fail closed.
    result = _build(scan, goal_xy=5)  # type: ignore[arg-type]
    assert result.obs is None
    assert result.fault == "internal_error"


# ---------------------------------------------------------------------------
# Numpy conformance test against the vendored reference implementation
# ---------------------------------------------------------------------------


def test_matches_vendored_numpy_implementation():
    np = pytest.importorskip("numpy", reason="numpy not installed (workstation)")
    pytest.importorskip("torch", reason="torch not installed (workstation)")
    pytest.importorskip("ncps", reason="ncps not installed (workstation)")

    sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))
    from rm_policy.vendor import ncp_policy as policy  # noqa: E402

    import random as _random

    rng = _random.Random(42)

    for _ in range(20):
        scan = [rng.uniform(0.2, 20.0) for _ in range(LIDAR_BEAMS)]
        pose_xy = (rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0))
        yaw = rng.uniform(-math.pi, math.pi)
        goal_xy = (rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0))
        action = (rng.uniform(-1.0, 2.0), rng.uniform(-3.14, 3.14))

        ours = build_observation(scan, X3_AMIN, X3_AINC, pose_xy, yaw, goal_xy, action)
        assert ours.fault is None, ours.fault

        vendor_local = policy.local_goal(pose_xy, yaw, goal_xy)
        vendor_obs = policy.build_obs(np.array(scan, dtype=np.float32), vendor_local, action)

        assert np.allclose(ours.obs, vendor_obs, atol=1e-6), (ours.obs, vendor_obs)
