"""Tests for rm_policy.observation_builder.

DEPENDENCY TIER: observation_builder now delegates all normalization to the
vendored, byte-exact upstream module, which imports numpy/torch/ncps at module
scope. So this whole file runs on the ROBOT tier, not the workstation tier.

That is a deliberate trade made on operator instruction (2026-08-27): the
policy code is to be an exact copy of upstream, so we call upstream's formulas
rather than reimplementing them in stdlib to keep the workstation green. The
robot runs the full suite, so nothing is actually unverified -- it is verified
somewhere else. Do NOT reintroduce a pure-Python reimplementation to move these
tests back to the workstation; that is the drift risk this design removed.
"""

from __future__ import annotations

import io
import math
import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not installed (workstation tier)")
pytest.importorskip("torch", reason="torch not installed (workstation tier)")
pytest.importorskip("ncps", reason="ncps not installed (workstation tier)")

yaml = pytest.importorskip("yaml")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.observation_builder import (  # noqa: E402
    build_observation,
    ObservationResult,
)
from rm_policy.vendor import ncp_policy  # noqa: E402

FIXTURES = os.path.join(_REPO_ROOT, "tests", "fixtures")
SAMPLE = os.path.join(FIXTURES, "tg30_scan_sample.yaml")

POSE = (0.0, 0.0)
YAW = 0.0
GOAL = (2.5, 0.0)
PREV = (0.0, 0.0)


def _load_scan(path):
    docs = [d for d in yaml.safe_load_all(io.open(path, encoding="utf-8")) if d]
    return docs[0]


def _flat_scan(n=2000, value=3.0):
    """A synthetic full-circle scan with the TG30's geometry."""
    return {
        "ranges": [value] * n,
        "angle_min": -math.pi,
        "angle_increment": 2.0 * math.pi / (n - 1),
    }


def _build(scan, **kw):
    args = dict(pose_xy=POSE, yaw=YAW, goal_xy=GOAL, previous_action=PREV)
    args.update(kw)
    return build_observation(
        scan["ranges"], scan["angle_min"], scan["angle_increment"],
        args["pose_xy"], args["yaw"], args["goal_xy"], args["previous_action"],
    )


# --- shape and block boundaries --------------------------------------------

def test_returns_exactly_36_finite_values() -> None:
    r = _build(_flat_scan())
    assert r.fault is None, r.fault
    assert len(r.obs) == ncp_policy.OBS_DIM == 36
    assert all(math.isfinite(float(v)) for v in r.obs)


def test_block_boundaries_are_32_2_2() -> None:
    """Lidar 0:32, goal 32:34, previous action 34:36 -- order is load-bearing."""
    r = _build(_flat_scan(value=2.5))
    obs = r.obs
    # 2.5 m is exactly the clip midpoint -> normalizes to 0.0
    assert all(float(v) == pytest.approx(0.0, abs=1e-6) for v in obs[0:32])
    # goal 2.5 m ahead -> (2.5/5 - 1, 0/5 - 1) = (-0.5, -1.0)
    assert float(obs[32]) == pytest.approx(-0.5, abs=1e-6)
    assert float(obs[33]) == pytest.approx(-1.0, abs=1e-6)
    # previous action (0,0) -> ((0-0.5)/1.5, 0/3.14) = (-0.3333, 0.0)
    assert float(obs[34]) == pytest.approx(-1.0 / 3.0, abs=1e-6)
    assert float(obs[35]) == pytest.approx(0.0, abs=1e-6)


def test_previous_action_zero_is_not_a_zero_vector() -> None:
    """A zero ACTION normalizes to a non-zero pair. Do not 'fix' this."""
    r = _build(_flat_scan())
    assert float(r.obs[34]) != 0.0
    assert float(r.obs[34]) == pytest.approx(-1.0 / 3.0, abs=1e-6)


# --- delegation: we must match upstream's own pipeline exactly --------------

def test_matches_upstream_call_sequence_exactly() -> None:
    """The observation must equal prepare_scan -> local_goal -> build_obs.

    This is the guarantee that we add no arithmetic of our own. If someone
    reintroduces a local formula, this diverges.
    """
    from rm_policy import scan_resampler

    scan = _load_scan(SAMPLE)
    res = _build(scan)
    assert res.fault is None, res.fault

    binned = scan_resampler.resample_forward_arc(
        scan["ranges"], scan["angle_min"], scan["angle_increment"],
        n_bins=32, arc_deg=180.0, range_min=0.01, range_max=30.0,
        sampling=scan_resampler.SAMPLING_NEAREST_RAY,
    )
    expected = ncp_policy.build_obs(
        ncp_policy.prepare_scan(binned.bins),
        ncp_policy.local_goal(POSE, YAW, GOAL),
        PREV,
    )
    assert np.allclose(np.asarray(res.obs), np.asarray(expected), atol=0.0)


# --- the TG30 no-return protection now lives upstream of the policy ---------

def test_real_tg30_scan_never_reads_as_a_wall() -> None:
    """~28-39% of TG30 rays are 0.0. None may normalize to -1.0 ('touching')."""
    scan = _load_scan(SAMPLE)
    raw_zeros = sum(1 for x in scan["ranges"] if x == 0.0)
    assert raw_zeros > 0, "fixture must actually contain no-returns"

    r = _build(scan)
    assert r.fault is None, r.fault
    lidar = [float(v) for v in r.obs[0:32]]
    assert all(math.isfinite(v) for v in lidar)
    assert all(v > -1.0 + 1e-9 for v in lidar), (
        f"a no-return normalized to 'obstacle at zero distance': min={min(lidar)}"
    )


def test_all_dropout_scan_reads_as_clear_not_as_a_wall() -> None:
    scan = _flat_scan(value=0.0)  # every ray a TG30 no-return
    r = _build(scan)
    assert r.fault is None, r.fault
    assert all(float(v) == pytest.approx(1.0, abs=1e-6) for v in r.obs[0:32])


# --- goal normalization: documented out-of-range values are EXPECTED --------

@pytest.mark.parametrize(
    "goal,yaw,expect_x,expect_y",
    [
        ((5.0, 0.0), 0.0, 0.0, -1.0),        # 5 m ahead
        ((10.0, 0.0), 0.0, 1.0, -1.0),       # 10 m ahead, top of the clean range
        ((-5.0, 0.0), 0.0, -2.0, -1.0),      # directly behind -> -2.00
    ],
)
def test_goal_normalization_including_out_of_range_values(
    goal, yaw, expect_x, expect_y
) -> None:
    """These are the checkpoint's formula, not a bug. Pinned so nobody 'fixes' it."""
    r = _build(_flat_scan(), goal_xy=goal, yaw=yaw)
    assert r.fault is None, r.fault
    assert float(r.obs[32]) == pytest.approx(expect_x, abs=1e-5)
    assert float(r.obs[33]) == pytest.approx(expect_y, abs=1e-5)


# --- fail-closed paths ------------------------------------------------------

@pytest.mark.parametrize(
    "kw,expected",
    [
        (dict(pose_xy=None), "pose_missing"),
        (dict(yaw=None), "pose_missing"),
        (dict(goal_xy=None), "goal_missing"),
        (dict(pose_xy=(float("nan"), 0.0)), "non_finite_pose_or_goal"),
        (dict(goal_xy=(float("inf"), 0.0)), "non_finite_pose_or_goal"),
        (dict(previous_action=None), "invalid_previous_action"),
        (dict(previous_action=(1.0,)), "invalid_previous_action"),
        (dict(previous_action=(float("nan"), 0.0)), "non_finite_previous_action"),
    ],
)
def test_fault_paths_return_no_observation(kw, expected) -> None:
    r = _build(_flat_scan(), **kw)
    assert r.obs is None, "a fault must never carry a partial observation"
    assert r.fault == expected


def test_lidar_fault_is_propagated_with_a_prefix() -> None:
    r = build_observation(None, -math.pi, 0.003, POSE, YAW, GOAL, PREV)
    assert r.obs is None
    assert r.fault is not None and r.fault.startswith("lidar:")


def test_garbage_input_faults_rather_than_raising() -> None:
    r = build_observation("not a scan", -math.pi, 0.003, POSE, YAW, GOAL, PREV)
    assert r.obs is None
    assert r.fault is not None
