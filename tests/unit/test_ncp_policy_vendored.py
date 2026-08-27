"""Tests for the VENDORED NCP policy module (rm_policy.vendor.ncp_policy).

Copied from LumenLabs/NCP-on-Rosmaster @ 0ad548c (tests/test_rosmaster_policy.py)
and repointed at our vendored copy, per the operator decision of 2026-08-27 that
all fixes live in ncp_ws and the upstream repo is not modified.

DEPENDENCY SKIP: this module needs numpy + torch + ncps, which the vendored
policy imports at module scope. Those are NOT installed on the dev workstation
(deliberately — the rest of the test suite is pure stdlib so it runs anywhere,
and torch on aarch64 is a robot-side concern). Without the guard below, a
missing numpy takes down collection for the ENTIRE suite, not just this file.
These tests therefore run only where the deps exist; treat them as a
robot-side / CI-side tier, not part of the workstation unit tier.
"""

import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not installed (workstation)")
pytest.importorskip("torch", reason="torch not installed (workstation)")
pytest.importorskip("ncps", reason="ncps not installed (workstation)")

# Same path shim as tests/unit/test_scan_resampler.py — the packages are not
# pip-installed on the workstation, they are colcon packages under src/.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.vendor import ncp_policy as policy  # noqa: E402


# The X3 sensor's exact geometry: 32 beams across the front 180°, REP-103.
X3_AMIN = -np.pi / 2
X3_AINC = np.pi / (policy.BEAMS - 1)


def test_check_scan_accepts_x3_layout():
    assert policy.check_scan(policy.BEAMS, X3_AMIN, X3_AINC) == []


def test_check_scan_reports_beam_count_mismatch():
    problems = policy.check_scan(720, X3_AMIN, np.pi / 719)
    assert any("expected 32 beams, got 720" in p for p in problems)


def test_check_scan_reports_fov_mismatch():
    # 32 beams but spread over 360°
    problems = policy.check_scan(policy.BEAMS, -np.pi, 2 * np.pi / policy.BEAMS)
    assert any("span" in p for p in problems)


def test_check_scan_reports_off_center_scan():
    # correct count and span, but starting at 0 (centered 90° left)
    problems = policy.check_scan(policy.BEAMS, 0.0, X3_AINC)
    assert any("not centered" in p for p in problems)


def test_check_scan_wraps_center_angle():
    # 0..2π-style convention pointing straight ahead (center wraps to 0)
    problems = policy.check_scan(policy.BEAMS, 3 * np.pi / 2, X3_AINC)
    assert problems == []


def test_prepare_scan_maps_nonfinite_to_clip():
    out = policy.prepare_scan([np.inf, np.nan, 2.0, -np.inf])
    assert out.dtype == np.float32
    assert np.allclose(out, [policy.LASER_CLIP, policy.LASER_CLIP, 2.0, policy.LASER_CLIP])


def test_build_obs_matches_barn_normalization():
    scan = np.full(policy.BEAMS, 2.5, dtype=np.float32)  # mid-clip -> 0
    obs = policy.build_obs(scan, [5.0, 0.0], [0.5, 0.0])
    assert obs.shape == (policy.OBS_DIM,)
    assert np.allclose(obs[: policy.BEAMS], 0.0)
    assert np.allclose(obs[policy.BEAMS : policy.BEAMS + 2], [0.0, -1.0])  # g/5 - 1
    assert np.allclose(obs[-2:], (np.array([0.5, 0.0]) - policy._ACTION_BIAS) / policy._ACTION_SCALE)


def test_build_obs_clips_laser():
    scan = np.array([10.0] * policy.BEAMS)  # beyond clip -> +1 after normalize
    obs = policy.build_obs(scan, [0, 0], [0, 0])
    assert np.allclose(obs[: policy.BEAMS], 1.0)


def test_local_goal_rotates_into_robot_frame():
    # robot at origin facing +y (yaw 90°); goal 10 m up the y axis is dead ahead
    g = policy.local_goal([0, 0], np.pi / 2, [0, 10])
    assert np.allclose(g, [10, 0], atol=1e-6)


def test_reactive_clear_cruises():
    action, sees = policy.reactive_action(np.full(policy.BEAMS, 4.0))
    assert not sees
    assert np.allclose(action, [policy.CRUISE_V, 0.0])


@pytest.mark.parametrize(
    "blocked_side,expected_w_sign",
    [(slice(0, 16), +1), (slice(16, 32), -1)],  # right blocked -> spin left, and vice versa
)
def test_reactive_obstacle_spins_toward_open_side(blocked_side, expected_w_sign):
    scan = np.full(policy.BEAMS, 4.0)
    scan[blocked_side] = 0.5
    action, sees = policy.reactive_action(scan)
    assert sees
    assert action[0] == 0.0
    assert np.sign(action[1]) == expected_w_sign
    assert abs(action[1]) == policy.SPIN_W


def test_reward_progress_success_collision():
    assert policy.reward(5.0, 4.0, min_range=3.0) == pytest.approx(1.0)
    assert policy.reward(1.0, 0.3, min_range=3.0) == pytest.approx(0.7 + policy.SUCCESS_REWARD)
    assert policy.reward(5.0, 5.0, min_range=0.1) == pytest.approx(policy.COLLISION_REWARD)


def test_actor_shapes_bounds_and_state():
    actor = policy.NcpActor()
    obs = np.zeros(policy.OBS_DIM, dtype=np.float32)
    action, state = actor.act(obs)
    assert action.shape == (2,)
    assert np.all(action >= policy.ACTION_LOW) and np.all(action <= policy.ACTION_HIGH)
    assert state is not None
    # carrying state changes the result vs a fresh reset — the memory is live
    a2, state = actor.act(obs, state)
    a_fresh, _ = actor.act(obs, None)
    assert np.allclose(a_fresh, action)
    assert not np.allclose(a2, action)


def test_actor_seed_reproducible():
    a1, _ = policy.NcpActor(seed=7).act(np.ones(policy.OBS_DIM, dtype=np.float32))
    a2, _ = policy.NcpActor(seed=7).act(np.ones(policy.OBS_DIM, dtype=np.float32))
    assert np.allclose(a1, a2)


# --- LOCAL DIVERGENCE TEST (not from upstream) ------------------------------
# Pins divergence 1 in rm_policy/vendor/ncp_policy.py's header: the TG30
# encodes a no-return as 0.0, never inf, so prepare_scan must mask it.

def test_prepare_scan_maps_tg30_zero_no_return_to_clip():
    """A 0.0 must become LASER_CLIP, not survive as a zero-distance obstacle."""
    out = policy.prepare_scan([0.0, 2.0, 0.0, 4.0])
    assert np.allclose(out, [policy.LASER_CLIP, 2.0, policy.LASER_CLIP, 4.0])


def test_prepare_scan_maps_sub_range_min_and_negative_to_clip():
    out = policy.prepare_scan([-1.0, 0.005, 2.0])
    assert np.allclose(out, [policy.LASER_CLIP, policy.LASER_CLIP, 2.0])


def test_zero_no_return_does_not_reach_obs_as_zero_distance():
    """End-to-end: 0.0 rays must normalize to +1.0 (clear), never -1.0 (touching)."""
    scan = policy.prepare_scan([0.0] * policy.BEAMS)
    obs = policy.build_obs(scan, np.array([5.0, 0.0]), np.zeros(2))
    assert np.allclose(obs[: policy.BEAMS], 1.0), (
        "a full no-return scan must read as clear, not as a wall at zero range"
    )
