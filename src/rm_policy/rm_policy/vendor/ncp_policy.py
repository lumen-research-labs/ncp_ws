"""NCP policy plumbing for the Rosmaster X3 — ROS-free, unit-testable.

Mirrors the barn_gym BarnEnv observation/action contract exactly
(optomotive arch-comparison worktree, --beams 720 --fov 270, the native
BARN Hokuyo UST-10LX layout):

    obs(724) = [laser(720, normalized), local goal(x, y), last action(v, w)]
    action   = [v, w]  (linear m/s, angular rad/s, physical units)

The actor is the trained policy's exact hookup — a CfC on an AutoNCP
wiring (256 neurons, 128 motor) with mixed memory, then the SB3 MLP head
[64, 64] to (v, w) — with the checkpoint's VecNormalize obs statistics
carried as buffers. `weights/actor.pt` holds the M8-v2 run, snapshot
s1436792 (63% held-out success, official BARN 0.219), exported bit-exact
from the SB3 checkpoint by `weights/export_from_sb3.py`. `weights/actor_m4.pt`
is the earlier M4 run (32 beams over 180 deg, obs 36); a deployment that
loads it must build the actor with obs_dim=36 and feed it that layout.
Without a weights file the actor runs with random weights and pass-through
normalization.
"""

import numpy as np
import torch
import torch.nn as nn
from ncps.torch import CfC
from ncps.wirings import AutoNCP

BEAMS = 720
FOV_DEG = 270.0
LASER_CLIP = 5.0  # meters
OBS_DIM = BEAMS + 2 + 2

ACTION_LOW = np.array([-1.0, -3.14], dtype=np.float32)  # v m/s, w rad/s
ACTION_HIGH = np.array([2.0, 3.14], dtype=np.float32)
_ACTION_BIAS = (ACTION_HIGH + ACTION_LOW) / 2
_ACTION_SCALE = (ACTION_HIGH - ACTION_LOW) / 2

UNITS = 256
MOTOR_NEURONS = 128

# Debug reactive rule (mode:=debug): unmistakable "I can see" behavior.
OBSTACLE_DIST = 1.5  # meters: anything closer in the FOV triggers a spin
SPIN_W = 1.2  # rad/s
CRUISE_V = 0.4  # m/s when nothing is near

COLLISION_DIST = 0.25  # meters, for the logged reward
SUCCESS_DIST = 0.4
SUCCESS_REWARD = 20.0
COLLISION_REWARD = -1.0


SCAN_TOL_DEG = 10.0


def check_scan(n_ranges, angle_min, angle_increment, tol_deg=SCAN_TOL_DEG):
    """Compare a LaserScan's geometry against the expected sensor layout
    (BEAMS rays across the frontal FOV_DEG, REP-103: 0 = straight ahead).

    Returns a list of human-readable mismatch strings — empty means the
    scan is what the NCP trained on. No silent adaptation happens
    elsewhere: the node warns on any mismatch, and refuses to drive on a
    beam-count mismatch (the obs vector would be garbage).
    """
    problems = []
    if n_ranges != BEAMS:
        problems.append(f"expected {BEAMS} beams, got {n_ranges}")
    span = np.rad2deg(angle_increment * (n_ranges - 1))
    if abs(span - FOV_DEG) > tol_deg:
        problems.append(f"expected ~{FOV_DEG:.0f}° span, got {span:.1f}°")
    center = np.rad2deg(angle_min) + span / 2
    center = (center + 180.0) % 360.0 - 180.0
    if abs(center) > tol_deg:
        problems.append(f"scan not centered straight ahead (center at {center:.1f}°)")
    return problems


def prepare_scan(ranges):
    """Ranges as float32 meters; non-finite (inf/nan = no return) -> LASER_CLIP."""
    r = np.array(ranges, dtype=np.float32)
    r[~np.isfinite(r)] = LASER_CLIP
    return r


def local_goal(pose_xy, yaw, goal_xy):
    """World-frame goal -> robot-frame (x, y), as BarnEnv._obs does."""
    d = np.asarray(goal_xy, dtype=np.float64) - np.asarray(pose_xy, dtype=np.float64)
    c, s = np.cos(-yaw), np.sin(-yaw)
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]], dtype=np.float32)


def build_obs(scan_m, local_goal_xy, last_action):
    """Normalize exactly as BarnEnv._obs: same formulas, same order."""
    s = np.clip(scan_m, 0.0, LASER_CLIP)
    s = (s - LASER_CLIP / 2) / LASER_CLIP * 2  # -> [-1, 1]
    g = np.asarray(local_goal_xy, dtype=np.float32) / 5.0 - 1.0
    a = (np.asarray(last_action, dtype=np.float32) - _ACTION_BIAS) / _ACTION_SCALE
    return np.concatenate([s, g, a]).astype(np.float32)


def reactive_action(scan_m, obstacle_dist=OBSTACLE_DIST):
    """Debug rule: see something close -> stop and spin toward the open side;
    otherwise creep forward. Returns ((v, w), sees_obstacle)."""
    scan_m = np.asarray(scan_m)
    if scan_m.min() < obstacle_dist:
        half = len(scan_m) // 2
        # second half of the scan = positive angles = robot's left;
        # positive w = CCW = turn left
        w = SPIN_W if scan_m[half:].mean() > scan_m[:half].mean() else -SPIN_W
        return np.array([0.0, w], dtype=np.float32), True
    return np.array([CRUISE_V, 0.0], dtype=np.float32), False


def reward(prev_goal_dist, goal_dist, min_range):
    """BarnEnv per-step reward, computed onboard for logging only."""
    r = prev_goal_dist - goal_dist
    if goal_dist < SUCCESS_DIST:
        r += SUCCESS_REWARD
    if min_range < COLLISION_DIST:
        r += COLLISION_REWARD
    return float(r)


class NcpActor(nn.Module):
    """CfC + AutoNCP actor with the trained policy's exact hookup
    (mixed_memory=True keeps the state (h, c)-shaped like the SB3 stack).

    Head is the trained RecurrentPPO stack after the recurrent core:
    motor neurons -> MLP [64, 64] (tanh, SB3 default net_arch) -> (v, w).
    The obs_mean/obs_var buffers hold the checkpoint's VecNormalize
    statistics, applied in act() exactly as at training time; the default
    buffers (mean 0, var 1) leave observations untouched for random-weight
    runs. Both ride along in the state_dict, so one .pt carries everything.
    """

    OBS_CLIP = 10.0  # VecNormalize clip_obs at training time

    def __init__(self, obs_dim=OBS_DIM, units=UNITS, motor=MOTOR_NEURONS, seed=0):
        super().__init__()
        torch.manual_seed(seed)
        self.cfc = CfC(
            obs_dim, AutoNCP(units, motor), batch_first=True, mixed_memory=True
        )
        self.head = nn.Sequential(
            nn.Linear(motor, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, 2)
        )
        self.register_buffer("obs_mean", torch.zeros(obs_dim, dtype=torch.float64))
        self.register_buffer("obs_var", torch.ones(obs_dim, dtype=torch.float64))

    @torch.no_grad()
    def act(self, obs, state=None):
        """One 5 Hz tick: obs(OBS_DIM) -> ((v, w) clipped to action bounds, state).

        Pass the returned state back in on the next tick; None resets it.
        """
        o = (torch.as_tensor(obs, dtype=torch.float64) - self.obs_mean) / torch.sqrt(
            self.obs_var + 1e-8
        )
        o = torch.clamp(o, -self.OBS_CLIP, self.OBS_CLIP)
        x = o.to(torch.float32).view(1, 1, -1)
        out, state = self.cfc(x, state)
        action = self.head(out[0, 0]).numpy()
        return np.clip(action, ACTION_LOW, ACTION_HIGH), state
