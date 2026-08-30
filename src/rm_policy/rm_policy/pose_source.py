"""
pose_source.py — pose validation shared by the odom and map (SLAM) pose paths.
ROS-free and unit-testable.

What it does:
    Holds the two decisions that must be made about a pose sample before the
    policy is allowed to act on it, independent of where the sample came from:

      1. Is it usable at all (finite, and not too old)?
      2. Did it JUMP — move further between ticks than the robot physically
         could have?

    The second is the reason this module exists. With `/odom` alone, pose
    evolves continuously: it drifts, but it never teleports. A SLAM pose does
    teleport — every loop closure and every relocalization is a step change,
    by design, because correcting the estimate is the whole point.

    That step is a hazard specific to THIS pipeline. `goal_local` is 2 of the
    36 numbers the network reads, and the network is a CfC with recurrent
    state. A pose correction moves the goal in the robot's frame instantly,
    so without intervention the policy would keep steering on a hidden state
    accumulated toward where the goal used to be. The correct response is to
    reset the recurrent state — the same thing already done for a stale-sensor
    gap, and for the same reason: the input history is no longer continuous.

    A jump is judged against IMPLIED SPEED rather than raw displacement, so
    the test scales with the tick interval instead of assuming one. A fixed
    displacement threshold silently becomes stricter as the loop runs faster,
    and at a 5 Hz control rate against a 30 Hz pose source that difference is
    a factor of six.

Subscribes to:  (nothing — pure module; the node supplies samples)
Publishes:      (nothing — pure module; the node acts on the verdict)

Failure mode:
    Total and fail-closed. Every function returns a verdict for every input,
    including non-finite values and absent history; nothing raises. An
    unusable pose is reported as unusable rather than repaired, because a
    substituted pose is a plausible-looking lie about where the robot is —
    the same reasoning as interface.yaml's goal_local.failure_mode, one level
    down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# A jump is motion the robot could not physically have performed. The bound is
# the safety layer's own max_vx with a generous multiplier: the point is to
# catch teleports of tens of centimetres, not to police the last few percent
# of a legitimately fast tick. Too tight and honest acceleration trips it;
# too loose and a small relocalization slips through with the recurrent state
# left stale.
DEFAULT_JUMP_SPEED_MPS = 1.0

# Yaw equivalent. max_wz is 0.8 rad/s, so this is a similar multiplier.
DEFAULT_JUMP_YAW_RATE = 3.0

# Floor on the displacement that can count as a jump, metres. Without it, a
# very short dt makes the speed test arbitrarily sensitive and ordinary pose
# noise reads as a teleport.
DEFAULT_JUMP_FLOOR_M = 0.05
DEFAULT_JUMP_FLOOR_RAD = 0.10


@dataclass(frozen=True)
class Pose2D:
    """A planar pose. `stamp` is monotonic seconds, as the node measures it."""

    x: float
    y: float
    yaw: float
    stamp: float

    @property
    def is_finite(self) -> bool:
        return all(
            math.isfinite(v) for v in (self.x, self.y, self.yaw, self.stamp)
        )


def pose_is_usable(pose: Optional[Pose2D], now: float, max_age_s: float) -> Optional[str]:
    """Return None if the pose may be used, else a reason string.

    Deliberately mirrors the fault-string convention used by
    `scan_resampler` and `policy_runner`: a None result means "proceed", and
    anything else is a reason the caller reports and acts on. The caller must
    never fall back to a default pose on a non-None result.
    """
    if pose is None:
        return "pose_missing"
    if not pose.is_finite:
        return "pose_nonfinite"
    age = now - pose.stamp
    if age < 0.0:
        # A pose stamped in the future means the clock moved under us. On
        # bernard-1 that is not hypothetical: it has no NTP and its clock has
        # been pushed by hand mid-session more than once. Treat it as
        # unusable rather than as an enormous negative age that trivially
        # passes the freshness test.
        return "pose_future_stamp"
    if age > max_age_s:
        return f"pose_stale:{age * 1000.0:.0f}ms"
    return None


def detect_pose_jump(
    previous: Optional[Pose2D],
    current: Pose2D,
    *,
    max_speed_mps: float = DEFAULT_JUMP_SPEED_MPS,
    max_yaw_rate: float = DEFAULT_JUMP_YAW_RATE,
    floor_m: float = DEFAULT_JUMP_FLOOR_M,
    floor_rad: float = DEFAULT_JUMP_FLOOR_RAD,
) -> Optional[str]:
    """Return a reason string if `current` is discontinuous with `previous`.

    None means the pose evolved plausibly. A non-None result is the caller's
    signal to reset the policy's recurrent state — NOT to reject the pose. The
    corrected pose is the better estimate; it is the accumulated hidden state
    that has been invalidated by it.

    No previous pose is NOT a jump. The first sample of a run has nothing to
    be discontinuous with, and reporting one would reset a recurrent state
    that is already empty.
    """
    if previous is None:
        return None
    if not (previous.is_finite and current.is_finite):
        return "jump_nonfinite"

    dt = current.stamp - previous.stamp
    if dt <= 0.0:
        # Same or backwards stamp: no interval to judge a speed over. Not
        # called a jump — that would reset the network on every duplicate
        # sample — but not endorsed either.
        return None

    dx = current.x - previous.x
    dy = current.y - previous.y
    translation = math.hypot(dx, dy)
    allowed_translation = max(floor_m, max_speed_mps * dt)
    if translation > allowed_translation:
        return (
            f"pose_jump_translation:{translation:.3f}m"
            f">{allowed_translation:.3f}m"
        )

    dyaw = abs(_wrap_to_pi(current.yaw - previous.yaw))
    allowed_yaw = max(floor_rad, max_yaw_rate * dt)
    if dyaw > allowed_yaw:
        return f"pose_jump_yaw:{dyaw:.3f}rad>{allowed_yaw:.3f}rad"

    return None


def _wrap_to_pi(angle: float) -> float:
    """Wrap to (-pi, pi].

    Load-bearing for the yaw jump test: without it, a heading crossing the
    +-pi discontinuity reads as a ~6.28 rad instantaneous rotation and would
    reset the recurrent state every time the robot happened to point south.
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
