"""
observation_builder.py — Assembles the 36-dim observation vector for the
trained NCP policy, per config/interface.yaml interface_version 3.0.0.

What it does:
    Combines a resampled forward-arc lidar scan (via rm_policy.scan_resampler),
    a goal expressed in the robot's own frame, and the previously commanded
    action into the exact 36-element vector the trained CfC/AutoNCP checkpoint
    expects:

        obs[0:32]  lidar    — 32 beams, forward 180 deg, right -> left,
                              normalized to [-1, 1] against the TRAINING clip
        obs[32:34] goal     — (x, y) in the robot frame, normalized
        obs[34:36] action   — previous commanded (v, w), normalized

    Order and every formula/constant here are DICTATED BY THE TRAINED
    CHECKPOINT and are reproduced from
    `src/rm_policy/rm_policy/vendor/ncp_policy.py` (`build_obs`, `local_goal`,
    `prepare_scan`, `ACTION_LOW`/`ACTION_HIGH`). They are reimplemented in
    pure Python here, NOT imported, because the vendored module imports
    numpy/torch/ncps at module scope and none of those are installed on this
    workstation (deliberately — see that module's docstring). Reimplementing
    a formula instead of importing it is exactly the kind of silent-drift risk
    CLAUDE.md warns about, so equivalence to the vendored numpy implementation
    is PROVEN, not assumed: see the numpy conformance test in
    tests/unit/test_observation_builder.py, guarded by
    `pytest.importorskip("numpy")` so it runs on the robot and skips here.

    GOVERNING PRINCIPLE (operator, 2026-08-27): match what the trained policy
    saw in training as closely as possible. Every formula below is copied
    verbatim from the vendored module, including the LITERAL 3.14 angular
    action bound (NOT math.pi — see `_ACTION_HIGH_W` below and
    config/interface.yaml `pi_is_not_pi_note`).

Subscribes to:
    (nothing — pure module, no ROS/topic access. Callers pass plain floats
    and sequences: raw LaserScan ranges + geometry, the robot's current pose
    (x, y, yaw) if known, a goal in the world frame if one is defined, and
    the previously commanded action.)

Publishes:
    (nothing — pure module. The caller (policy_node) acts on the returned
    ObservationResult.)

Failure mode:
    Fail-closed, matching `rm_policy.scan_resampler` and
    `rm_safety.safety_logic`'s discipline. `build_observation()` never raises
    and never returns a partial vector: any of the following produces an
    `ObservationResult` with `obs=None` and a non-None `fault`, and the
    caller MUST check `fault` before reading `obs`:
      * the lidar resample step faults — the reason is propagated, prefixed
        "lidar:" (e.g. "lidar:empty_bin:4")
      * a goal is defined (`goal_xy` is not None) but `pose_xy` or `yaw` is
        None, OR a goal is simply undefined (`goal_xy` is None) — per
        config/interface.yaml `goal_local.failure_mode`, a missing pose or
        goal must NEVER be papered over with a plausible-looking default;
        that would silently command motion in an arbitrary direction
      * `previous_action` is None or not exactly 2 elements
      * any input (pose, yaw, goal, previous_action) contains a non-finite
        value
      * the assembled vector is not exactly OBS_DIM (36) elements, or
        contains a non-finite value after assembly
      * an unexpected exception anywhere in assembly ("internal_error")
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from rm_policy.scan_resampler import SAMPLING_NEAREST_RAY, resample_forward_arc

# ---------------------------------------------------------------------------
# Constants — sourced from config/interface.yaml (observation.layout) and
# src/rm_policy/rm_policy/vendor/ncp_policy.py. Retyped, not imported, because
# the vendor module drags in numpy/torch/ncps at module scope, which are not
# installed on this workstation. The numpy conformance test in
# tests/unit/test_observation_builder.py is what keeps these two copies from
# silently drifting apart — if you change a value here, that test must still
# pass against the vendored module's own constant.
# ---------------------------------------------------------------------------

# -- lidar block: config/interface.yaml observation.layout[lidar_ranges] --
LIDAR_BEAMS = 32  # vendor.ncp_policy.BEAMS
LIDAR_ARC_DEG = 180.0
LIDAR_RANGE_MIN_M = 0.01  # sensor floor; a ray at/below this is a no-return
LIDAR_RANGE_MAX_M = 30.0  # sensor ceiling for resampling — NOT the training
# clip below. Both numbers are correct and deliberately different; see
# config/interface.yaml `laser_clip_note`. Do not reconcile them.
LIDAR_SAMPLING = SAMPLING_NEAREST_RAY  # training-faithful mode; see scan_resampler
LASER_CLIP_M = 5.0  # vendor.ncp_policy.LASER_CLIP — the TRAINING clip

# -- goal_local block: config/interface.yaml observation.layout[goal_local] --
GOAL_DIM = 2
GOAL_NORM_SCALE_M = 5.0  # "xy / 5.0 - 1.0" — vendor.ncp_policy.build_obs
GOAL_NORM_OFFSET = 1.0

# -- previous_action block: config/interface.yaml observation.layout[previous_action] --
# vendor.ncp_policy.ACTION_LOW / ACTION_HIGH. The angular bound is LITERALLY
# 3.14, not math.pi — that is what the network was trained against
# (config/interface.yaml action.pi_is_not_pi_note). Using math.pi here would
# shift the observation ~0.05% with no error raised anywhere.
ACTION_LOW_V = -1.0
ACTION_HIGH_V = 2.0
ACTION_LOW_W = -3.14
ACTION_HIGH_W = 3.14
ACTION_BIAS_V = (ACTION_HIGH_V + ACTION_LOW_V) / 2.0
ACTION_SCALE_V = (ACTION_HIGH_V - ACTION_LOW_V) / 2.0
ACTION_BIAS_W = (ACTION_HIGH_W + ACTION_LOW_W) / 2.0
ACTION_SCALE_W = (ACTION_HIGH_W - ACTION_LOW_W) / 2.0
ACTION_DIM = 2

# -- total layout: config/interface.yaml observation.total_dim --
OBS_DIM = LIDAR_BEAMS + GOAL_DIM + ACTION_DIM  # 36


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ObservationResult:
    """Output of `build_observation`.

    Parameters
    ----------
    obs:
        Exactly `OBS_DIM` (36) floats, in layout order, on success. `None`
        whenever `fault` is set — the caller must check `fault` first and
        must never substitute a default vector for `None`.
    fault:
        `None` on success. Otherwise a reason string identifying which
        precondition failed; see this module's docstring "Failure mode".
    """

    obs: Optional[List[float]]
    fault: Optional[str]


# ---------------------------------------------------------------------------
# Pure-Python reimplementations of vendor.ncp_policy formulas
# ---------------------------------------------------------------------------


def _normalize_lidar(bins_m: Sequence[float]) -> List[float]:
    """`vendor.ncp_policy.build_obs`'s lidar normalization, verbatim:
    clip to [0, LASER_CLIP_M], then (x - LASER_CLIP_M/2) / LASER_CLIP_M * 2.
    """
    out = []
    half = LASER_CLIP_M / 2.0
    for r in bins_m:
        clipped = min(max(r, 0.0), LASER_CLIP_M)
        out.append((clipped - half) / LASER_CLIP_M * 2.0)
    return out


def _local_goal(
    pose_xy: Tuple[float, float], yaw: float, goal_xy: Tuple[float, float]
) -> Tuple[float, float]:
    """`vendor.ncp_policy.local_goal`, verbatim: world-frame goal -> robot
    frame, by subtracting position and rotating by -yaw."""
    dx = goal_xy[0] - pose_xy[0]
    dy = goal_xy[1] - pose_xy[1]
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy)


def _normalize_goal(local_xy: Tuple[float, float]) -> List[float]:
    """`vendor.ncp_policy.build_obs`'s goal normalization, verbatim:
    xy / GOAL_NORM_SCALE_M - GOAL_NORM_OFFSET. Deliberately unclipped — a 45
    deg off-bow goal normalizes to -1.71, a goal behind to -2.00. These are
    EXPECTED out-of-distribution values per config/interface.yaml
    `normalization_warning`, not bugs to clip away."""
    return [
        local_xy[0] / GOAL_NORM_SCALE_M - GOAL_NORM_OFFSET,
        local_xy[1] / GOAL_NORM_SCALE_M - GOAL_NORM_OFFSET,
    ]


def _normalize_previous_action(action: Sequence[float]) -> List[float]:
    """`vendor.ncp_policy.build_obs`'s previous_action normalization,
    verbatim: (a - bias) / scale per component. A zero action normalizes to
    (-0.333..., 0.0), NOT to zeros — config/interface.yaml `init_note` is
    explicit this must not be "fixed" to a zero vector."""
    v, w = action[0], action[1]
    return [
        (v - ACTION_BIAS_V) / ACTION_SCALE_V,
        (w - ACTION_BIAS_W) / ACTION_SCALE_W,
    ]


def _all_finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in values)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def build_observation(
    ranges: Optional[Sequence[float]],
    angle_min: float,
    angle_increment: float,
    pose_xy: Optional[Tuple[float, float]],
    yaw: Optional[float],
    goal_xy: Optional[Tuple[float, float]],
    previous_action: Optional[Sequence[float]],
) -> ObservationResult:
    """Assemble the 36-dim observation for one control tick.

    Parameters
    ----------
    ranges, angle_min, angle_increment:
        Raw LaserScan fields, passed straight through to
        `rm_policy.scan_resampler.resample_forward_arc`.
    pose_xy:
        Current (x, y) robot position in the odom frame, or `None` if `/odom`
        has not yet arrived. Required whenever `goal_xy` is not `None`.
    yaw:
        Current robot yaw (radians) in the odom frame, or `None`. Required
        whenever `goal_xy` is not `None`.
    goal_xy:
        The fixed goal point in the odom frame (see config/interface.yaml
        goal_local.preprocessing — this is a STATIC point, re-expressed in
        the robot frame every tick), or `None` if no goal is defined yet.
    previous_action:
        The (v, w) action actually commanded on the previous tick — 2
        elements, physical units (m/s, rad/s). Pass `(0.0, 0.0)` on the very
        first tick; per config/interface.yaml `init_note` this deliberately
        normalizes to a non-zero vector.

    Returns
    -------
    ObservationResult:
        `obs` populated (36 floats) on success; `fault` set and `obs` `None`
        on any precondition failure or unexpected exception. Never raises.
    """
    try:
        return _build_observation_inner(
            ranges, angle_min, angle_increment, pose_xy, yaw, goal_xy, previous_action
        )
    except Exception:
        return ObservationResult(obs=None, fault="internal_error")


def _build_observation_inner(
    ranges: Optional[Sequence[float]],
    angle_min: float,
    angle_increment: float,
    pose_xy: Optional[Tuple[float, float]],
    yaw: Optional[float],
    goal_xy: Optional[Tuple[float, float]],
    previous_action: Optional[Sequence[float]],
) -> ObservationResult:
    # --- lidar block --------------------------------------------------
    lidar_result = resample_forward_arc(
        ranges,
        angle_min,
        angle_increment,
        n_bins=LIDAR_BEAMS,
        arc_deg=LIDAR_ARC_DEG,
        range_min=LIDAR_RANGE_MIN_M,
        range_max=LIDAR_RANGE_MAX_M,
        sampling=LIDAR_SAMPLING,
    )
    if lidar_result.fault is not None or lidar_result.bins is None:
        return ObservationResult(obs=None, fault=f"lidar:{lidar_result.fault}")

    lidar_norm = _normalize_lidar(lidar_result.bins)

    # --- goal_local block -----------------------------------------------
    # A missing pose/goal must FAIL, never substitute a default goal vector
    # — config/interface.yaml goal_local.failure_mode is explicit that a
    # plausible-looking goal is worse than none.
    if goal_xy is None:
        return ObservationResult(obs=None, fault="goal_missing")
    if pose_xy is None or yaw is None:
        return ObservationResult(obs=None, fault="pose_missing")
    if not _all_finite(pose_xy) or not math.isfinite(yaw) or not _all_finite(goal_xy):
        return ObservationResult(obs=None, fault="non_finite_pose_or_goal")

    local_xy = _local_goal(pose_xy, yaw, goal_xy)
    goal_norm = _normalize_goal(local_xy)

    # --- previous_action block -------------------------------------------
    if previous_action is None or len(previous_action) != ACTION_DIM:
        return ObservationResult(obs=None, fault="invalid_previous_action")
    if not _all_finite(previous_action):
        return ObservationResult(obs=None, fault="non_finite_previous_action")

    action_norm = _normalize_previous_action(previous_action)

    # --- assemble & fail closed on any surprise --------------------------
    obs = lidar_norm + goal_norm + action_norm

    if len(obs) != OBS_DIM:
        return ObservationResult(obs=None, fault="wrong_obs_length")
    if not _all_finite(obs):
        return ObservationResult(obs=None, fault="non_finite_obs")

    return ObservationResult(obs=obs, fault=None)
