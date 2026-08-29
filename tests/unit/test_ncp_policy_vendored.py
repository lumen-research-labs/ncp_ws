"""Tests for the VENDORED NCP policy module (rm_policy.vendor.ncp_policy).

Copied from LumenLabs/NCP-on-Rosmaster @ 69785c8 (tests/test_rosmaster_policy.py)
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

import math
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
    # LOCAL FIX (not upstream): reactive_action returns a float32 array, and
    # np.float32(1.2) is 1.2000000476837158, so exact equality against the
    # float64 module constant fails. Upstream's assertion is simply wrong on
    # any numpy; it never ran here because numpy is absent on the workstation.
    # Behaviour is unchanged — this is a test-precision fix only.
    assert abs(action[1]) == pytest.approx(policy.SPIN_W)


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


# --- UPSTREAM BEHAVIOUR, PINNED (not a divergence) --------------------------
# vendor/ncp_policy.py is a BYTE-EXACT copy of upstream and carries no local
# edits (see vendor/VENDOR.md). An earlier revision patched prepare_scan to
# mask the TG30's 0.0 no-return; that patch was removed and the protection
# moved into rm_policy.scan_resampler, which is code we own.
#
# These tests therefore assert what upstream ACTUALLY does, including the
# limitation. They exist so the limitation is explicit and cannot be
# rediscovered the hard way -- not to demand it be fixed here.


def test_upstream_prepare_scan_does_not_mask_zero_no_returns() -> None:
    """UPSTREAM BEHAVIOUR: prepare_scan handles only non-finite values.

    The TG30 emits 0.0 for a no-return and never inf, so a raw TG30 scan fed
    straight to this function keeps its 0.0s and build_obs then normalizes
    them to -1.0 -- "obstacle at zero distance". That is why nothing in this
    project may call prepare_scan on a raw scan: rm_policy.scan_resampler runs
    first and maps every no-return to range_max. See vendor/VENDOR.md.
    """
    out = policy.prepare_scan([0.0, 2.0, np.inf, np.nan])
    assert out[0] == 0.0, "upstream leaves 0.0 untouched -- by design, not a bug"
    assert out[1] == pytest.approx(2.0)
    assert out[2] == pytest.approx(policy.LASER_CLIP)  # inf IS handled
    assert out[3] == pytest.approx(policy.LASER_CLIP)  # nan IS handled


def test_raw_zero_would_normalize_to_touching_which_is_why_we_resample() -> None:
    """Demonstrates the hazard the resampler exists to prevent."""
    obs = policy.build_obs(
        policy.prepare_scan([0.0] * policy.BEAMS),
        np.array([5.0, 0.0]),
        np.zeros(2),
    )
    assert np.allclose(obs[: policy.BEAMS], -1.0), (
        "a raw 0.0 normalizes to -1.0 = 'touching a wall'; scan_resampler "
        "must always run first"
    )


def test_resampled_scan_is_safe_through_the_same_upstream_path() -> None:
    """The pipeline we actually use: resampler first, then upstream verbatim."""
    import os
    import sys as _sys
    _sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))
    from rm_policy import scan_resampler

    n = 2000
    raw = [0.0] * n  # every ray a TG30 no-return
    binned = scan_resampler.resample_forward_arc(
        raw, -math.pi, 2.0 * math.pi / (n - 1),
        n_bins=policy.BEAMS, arc_deg=policy.FOV_DEG,
        range_min=0.01, range_max=30.0,
    )
    assert binned.fault is None
    obs = policy.build_obs(
        policy.prepare_scan(binned.bins), np.array([5.0, 0.0]), np.zeros(2)
    )
    assert np.allclose(obs[: policy.BEAMS], 1.0), (
        "with the resampler in front, an all-dropout scan reads as CLEAR"
    )


# --- TRAINED CHECKPOINT (upstream 69785c8 / weights a9af7af) -----------------
# The actor architecture changed when the trained weights landed: a multi-layer
# head plus VecNormalize buffers (see vendor/VENDOR.md). These pin the contract
# the exported .pt is written against, so a future re-vendor that silently
# reverts the shape fails here rather than at node startup on the robot.
#
# Ported from upstream tests/test_rosmaster_policy.py @ 69785c8.


def test_actor_state_dict_matches_export_contract() -> None:
    """The exact key set weights/export_from_sb3.py writes."""
    keys = set(policy.NcpActor().state_dict())
    assert {
        "head.0.weight", "head.2.weight", "head.4.weight", "obs_mean", "obs_var"
    } <= keys
    assert all(k.startswith(("cfc.", "head.", "obs_")) for k in keys)
    assert policy.NcpActor().state_dict()["head.4.weight"].shape == (2, 64)


def test_actor_applies_obs_normalization() -> None:
    """obs_mean/obs_var are LIVE in act(), not decorative buffers."""
    actor = policy.NcpActor()
    obs = np.full(policy.OBS_DIM, 0.5, dtype=np.float32)
    a_identity, _ = actor.act(obs)
    actor.obs_mean.fill_(0.5)  # now obs normalizes to exactly zero
    a_zero_in, _ = actor.act(obs)
    a_ref, _ = policy.NcpActor().act(np.zeros(policy.OBS_DIM, dtype=np.float32))
    assert not np.allclose(a_identity, a_zero_in)
    assert np.allclose(a_zero_in, a_ref)


def test_default_buffers_pass_observations_through_unchanged() -> None:
    """No weights file => mean 0 / var 1 => the obs reaches the CfC untouched.

    Pins the property that makes random-weight plumbing runs meaningful: with
    no checkpoint loaded, normalization is the identity (up to the 1e-8 epsilon)
    rather than some arbitrary transform.
    """
    actor = policy.NcpActor()
    assert np.allclose(actor.obs_mean.numpy(), 0.0)
    assert np.allclose(actor.obs_var.numpy(), 1.0)


# --- The real checkpoint, if a copy is reachable ----------------------------
# Skips where the upstream checkout is absent (e.g. a bare CI box). On the
# robot and the dev workstation the sibling checkout is the convention used by
# test_vendor_is_pristine.py and ncp.launch.py's upstream_path default.

_WEIGHTS = os.path.abspath(
    os.path.join(_REPO_ROOT, "..", "NCP-on-Rosmaster", "weights", "actor.pt")
)


@pytest.mark.skipif(
    not os.path.isfile(_WEIGHTS), reason="no upstream weights checkout present"
)
def test_trained_checkpoint_loads_strictly_into_the_vendored_actor() -> None:
    """The whole point of the re-vendor: actor.pt must load with NO slack.

    strict=True is deliberate. A checkpoint that only loads with
    strict=False is a checkpoint whose architecture does not match, and the
    missing tensors would be silently left at their random init.
    """
    import torch

    actor = policy.NcpActor()
    state = torch.load(_WEIGHTS, map_location="cpu")
    actor.load_state_dict(state)  # strict=True by default — must not raise

    # VecNormalize stats actually arrived; the defaults would be exactly 0/1.
    assert not np.allclose(actor.obs_mean.numpy(), 0.0), (
        "obs_mean is still the default — the checkpoint's VecNormalize "
        "statistics did not load"
    )
    assert np.all(actor.obs_var.numpy() > 0.0)


@pytest.mark.skipif(
    not os.path.isfile(_WEIGHTS), reason="no upstream weights checkout present"
)
def test_trained_actor_emits_finite_in_bounds_actions_on_a_clear_scan() -> None:
    """Smoke test the loaded policy over a short recurrent rollout.

    Not a judgement about whether the policy is GOOD — with rm_safety clamping
    max_vx to a fraction of the trained 2.0 m/s range, on-robot behaviour is
    not evidence about the policy anyway (interface.yaml
    authority_gap_warning). This only asserts the numbers are usable: finite,
    in bounds, and not frozen at a single constant across a changing input.
    """
    import torch

    actor = policy.NcpActor()
    actor.load_state_dict(torch.load(_WEIGHTS, map_location="cpu"))

    state = None
    actions = []
    clear = np.full(policy.BEAMS, policy.LASER_CLIP, dtype=np.float32)
    for step in range(10):
        # Goal walks toward the robot, so the observation genuinely changes.
        goal = np.array([2.5 - 0.1 * step, 0.0], dtype=np.float32)
        last = actions[-1] if actions else np.zeros(2, dtype=np.float32)
        obs = policy.build_obs(clear, goal, last)
        action, state = actor.act(obs, state)
        assert np.all(np.isfinite(action)), f"non-finite action at step {step}"
        assert np.all(action >= policy.ACTION_LOW)
        assert np.all(action <= policy.ACTION_HIGH)
        actions.append(action)

    spread = np.ptp(np.array(actions), axis=0)
    assert spread.max() > 1e-6, (
        "action is identical across 10 ticks of a CHANGING observation — the "
        "network is not responding to its input"
    )
