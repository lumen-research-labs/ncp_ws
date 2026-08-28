"""
mecanum.py — pure mecanum kinematics for the ROSMASTER X3.

What it does:
    Converts a body twist (vx, vy, wz) to the four wheel ground speeds and
    back, using the standard REP-103 convention (x forward, y left, z up /
    CCW-positive). No ROS, no I/O — imported by rm_base.base_node and by the
    unit tests.

Wheel order everywhere in this package is (front_left, back_left,
front_right, back_right) == motor channels (M1, M2, M3, M4) as measured on
bernard-1 (2026-08-28 per-motor identification). This is NOT the Yahboom
sample order (M1,M2,M3,M4 = FL,FR,BL,BR); do not "fix" it back.

Failure mode:
    Pure arithmetic. `inverse` never raises. `forward` divides by
    lever_arm_m; callers pass the geometry constant, which is > 0 by
    construction (half the wheelbase plus half the track).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WheelSpeeds:
    """Ground speed of each wheel contact patch, m/s, forward-positive."""

    front_left: float
    back_left: float
    front_right: float
    back_right: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.front_left, self.back_left, self.front_right, self.back_right)


@dataclass(frozen=True)
class BodyTwist:
    vx: float  # m/s, +forward
    vy: float  # m/s, +left
    wz: float  # rad/s, +CCW (turn left)


def inverse(twist: BodyTwist, lever_arm_m: float) -> WheelSpeeds:
    """Body twist -> wheel ground speeds.

    lever_arm_m is (wheelbase_x / 2 + track_y / 2): the sum of the half
    distances from the base centre to a wheel along x and y. For an X3 the
    two half-distances are combined because a mecanum wheel's yaw moment arm
    is that sum.
    """
    rot = twist.wz * lever_arm_m
    return WheelSpeeds(
        front_left=twist.vx - twist.vy - rot,
        back_left=twist.vx + twist.vy - rot,
        front_right=twist.vx + twist.vy + rot,
        back_right=twist.vx - twist.vy + rot,
    )


def forward(wheels: WheelSpeeds, lever_arm_m: float) -> BodyTwist:
    """Wheel ground speeds -> body twist (odometry direction)."""
    fl, bl, fr, br = wheels.as_tuple()
    return BodyTwist(
        vx=(fl + bl + fr + br) / 4.0,
        vy=(-fl + bl + fr - br) / 4.0,
        wz=(-fl - bl + fr + br) / (4.0 * lever_arm_m),
    )
