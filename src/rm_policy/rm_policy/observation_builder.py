"""
observation_builder.py — assemble the trained NCP's observation vector.

What it does:
    Turns one raw LaserScan plus the robot's pose and last action into the
    exact vector the trained checkpoint expects (`ncp_policy.OBS_DIM`, 724 in
    interface 4.0.0), per `config/interface.yaml`:

        obs[0:720]    lidar, 720 beams across the forward 270 deg, right -> left
        obs[720:722]  goal (x, y) in the ROBOT frame
        obs[722:724]  previous action

    It is deliberately THIN. All normalization arithmetic is performed by the
    VENDORED, BYTE-EXACT upstream module `vendor.ncp_policy`, called in
    upstream's own order (prepare_scan -> local_goal -> build_obs), exactly as
    `NCP-on-Rosmaster/rosmaster/node.py` does. This module contributes only
    two things upstream has no equivalent for: reducing a ~2000-point 360 deg
    TG30 scan to the 720 forward beams the network was trained on (with the
    search window and self-occlusion zones from interface.yaml), and
    fail-closed validation.

    NO FORMULA IS REIMPLEMENTED HERE, with one exception. An earlier revision
    reimplemented the normalization in pure Python so the tests could run
    without numpy on the dev workstation. That duplicated every constant and
    formula, which is a silent-drift risk against the checkpoint and was
    removed on operator instruction (2026-08-27: keep the policy code an
    exact copy of upstream). The cost is that this module now requires
    numpy, so its tests run on the robot tier rather than the workstation
    tier. That is the correct trade: the robot runs the full suite, and
    fidelity to the checkpoint outranks test-runner convenience.

    The exception (2026-11-04): the previous-action block is normalized by
    the ACTION BOUNDS THE CHECKPOINT WAS TRAINED WITH, and the vendored
    `build_obs` hardcodes the M8-v2/M4 bounds. A profile whose contract
    declares other bounds (5.0.0, M9: the robot's clamps) passes an
    `ActionBounds` (`rm_policy.action_contract`, read from its interface
    file), and the last two entries are recomputed with the same formula
    `build_obs` uses, in float32. With no bounds passed the vendored output
    is returned untouched; with bounds equal to the vendored constants the
    result is bit-identical (tested).

Subscribes to:  (nothing — pure module; the node passes values in)
Publishes:      (nothing — pure module; the node acts on the return)

Failure mode:
    Fail-closed, matching `rm_safety.safety_logic`'s discipline. Every failure
    returns `ObservationResult(obs=None, fault=<reason>)` — never a partial or
    plausible-looking vector. The caller cannot reach `obs` without branching
    on `fault` first. In particular a missing pose or goal is a FAULT and
    never a substituted default: `config/interface.yaml` is explicit that a
    plausible-looking goal silently commands motion in an arbitrary direction,
    which is worse than not driving.

Constants:
    None of the contract numbers live here. Lidar geometry is passed in by the
    caller (sourced from config/interface.yaml); every normalization constant
    lives in the vendored module. This module holds no magic numbers, and no
    copies of anything the checkpoint depends on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from rm_policy import scan_resampler
from rm_policy.action_contract import ActionBounds
from rm_policy.vendor import ncp_policy

# Lidar reduction parameters. Defaults mirror config/interface.yaml's
# observation.layout[lidar_ranges]; the node passes them explicitly.
DEFAULT_N_BEAMS = ncp_policy.BEAMS          # 720, from the checkpoint itself
DEFAULT_ARC_DEG = ncp_policy.FOV_DEG        # 270.0, from the checkpoint itself
DEFAULT_RANGE_MIN_M = 0.01                  # TG30 spec floor
DEFAULT_RANGE_MAX_M = 30.0                  # TG30 spec reach; NOT the training
                                            # clip (5.0) — see interface.yaml
                                            # laser_clip_note. Do not reconcile.


@dataclass(frozen=True)
class ObservationResult:
    """Either a complete observation or a fault. Never both, never neither."""

    obs: Optional[Sequence[float]]
    fault: Optional[str]


def build_observation(
    ranges: Optional[Sequence[float]],
    angle_min: float,
    angle_increment: float,
    pose_xy: Optional[Tuple[float, float]],
    yaw: Optional[float],
    goal_xy: Optional[Tuple[float, float]],
    previous_action: Optional[Sequence[float]],
    *,
    n_beams: int = DEFAULT_N_BEAMS,
    arc_deg: float = DEFAULT_ARC_DEG,
    range_min: float = DEFAULT_RANGE_MIN_M,
    range_max: float = DEFAULT_RANGE_MAX_M,
    search_window_bins: int = 0,
    mask_zones_deg: Sequence[Sequence[float]] = (),
    action_bounds: Optional[ActionBounds] = None,
) -> ObservationResult:
    """Build the observation vector, or return a fault.

    `search_window_bins` and `mask_zones_deg` are passed straight to
    `scan_resampler.resample_forward_arc`; the node sources them from
    config/interface.yaml (`nearest_ray_search_window_bins`,
    `self_occlusion_zones_deg`). Both default to off.

    `pose_xy`/`yaw` are the robot's current pose in the odom frame; `goal_xy`
    is the static goal in that same frame. `previous_action` is the (v, w)
    the policy REQUESTED last tick (not what the robot achieved) — see
    interface.yaml `clamped_action_note` for why. `action_bounds` are the
    contract's trained action bounds; `None` keeps the vendored constants.
    """
    try:
        return _build_observation_inner(
            ranges, angle_min, angle_increment, pose_xy, yaw, goal_xy,
            previous_action, n_beams=n_beams, arc_deg=arc_deg,
            range_min=range_min, range_max=range_max,
            search_window_bins=search_window_bins, mask_zones_deg=mask_zones_deg,
            action_bounds=action_bounds,
        )
    except Exception:
        return ObservationResult(obs=None, fault="internal_error")


def _build_observation_inner(
    ranges, angle_min, angle_increment, pose_xy, yaw, goal_xy, previous_action,
    *, n_beams, arc_deg, range_min, range_max, search_window_bins, mask_zones_deg,
    action_bounds,
) -> ObservationResult:
    # --- 1. Reduce the raw scan to the trained beam layout. OUR code, and the
    # only place the TG30's 0.0-no-return sentinel is handled: the resampler
    # maps 0.0 / sub-range_min / negative / inf / NaN to range_max BEFORE the
    # policy sees anything. This is why the vendored module needs no patch.
    scan_result = scan_resampler.resample_forward_arc(
        ranges, angle_min, angle_increment,
        n_bins=n_beams, arc_deg=arc_deg,
        range_min=range_min, range_max=range_max,
        sampling=scan_resampler.SAMPLING_NEAREST_RAY,
        search_window_bins=search_window_bins,
        mask_zones_deg=mask_zones_deg,
    )
    if scan_result.fault is not None:
        return ObservationResult(obs=None, fault=f"lidar:{scan_result.fault}")

    # --- 2. Pose and goal must both exist. Never substitute a default. ---
    if pose_xy is None or yaw is None:
        return ObservationResult(obs=None, fault="pose_missing")
    if goal_xy is None:
        return ObservationResult(obs=None, fault="goal_missing")
    if not all(math.isfinite(v) for v in (pose_xy[0], pose_xy[1], yaw,
                                          goal_xy[0], goal_xy[1])):
        return ObservationResult(obs=None, fault="non_finite_pose_or_goal")

    if previous_action is None or len(previous_action) != 2:
        return ObservationResult(obs=None, fault="invalid_previous_action")
    if not all(math.isfinite(float(a)) for a in previous_action):
        return ObservationResult(obs=None, fault="non_finite_previous_action")

    # --- 3. Upstream's own pipeline, in upstream's own order. This mirrors
    # NCP-on-Rosmaster/rosmaster/node.py exactly. prepare_scan is a no-op on
    # our already-cleaned vector, and is called anyway so the call sequence
    # stays identical to upstream's — if upstream ever changes what it does,
    # we inherit the change rather than diverging from it.
    scan_m = ncp_policy.prepare_scan(scan_result.bins)
    local_goal_xy = ncp_policy.local_goal(pose_xy, yaw, goal_xy)
    obs = ncp_policy.build_obs(scan_m, local_goal_xy, previous_action)
    if action_bounds is not None:
        # The one formula applied outside the vendored module: the previous
        # action normalized by the CONTRACT's bounds (module docstring).
        obs[-2:] = action_bounds.normalize(previous_action)

    # --- 4. Fail-closed validation of the assembled vector. ---
    if len(obs) != n_beams + 4:  # lidar + goal (2) + previous action (2)
        return ObservationResult(obs=None, fault="wrong_obs_length")
    if not all(math.isfinite(float(v)) for v in obs):
        return ObservationResult(obs=None, fault="non_finite_obs")

    return ObservationResult(obs=obs, fault=None)
