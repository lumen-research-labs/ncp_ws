"""
frames.py — the one 2D frame conversion the dashboard performs. ROS-free.

What it does:
    Turns a WORLD point the operator clicked on the map into the ROBOT-FRAME
    OFFSET that policy_node's `goal_x`/`goal_y` parameters expect.

    policy_node stores that pair as an offset and re-derives the world goal
    against its own current pose:

        goal_world = robot_xy + R(robot_yaw) @ (goal_x, goal_y)

    so this module is exactly the inverse, R(-yaw), and the two MUST remain
    inverses. A sign error here does not raise: it puts the goal somewhere
    plausible and wrong, mirrored or rotated about the robot, which looks like
    the policy misbehaving rather than like a bad target. The round-trip is
    pinned by a test for that reason.

    It lives in its own module, rather than inline in dashboard_node, so it can
    be tested on the workstation — dashboard_node imports rclpy, tf2 and the
    custom messages, none of which exist here.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    None of its own; it is total over finite inputs. Validation of the inputs
    (finite, right frame, pose actually available) belongs to the caller, which
    is the layer that can refuse with a reason the operator will see.
"""
from __future__ import annotations

import math
from typing import Tuple


def world_to_body_offset(
    world_x: float,
    world_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> Tuple[float, float]:
    """World point -> offset in the robot's frame. Inverse of R(yaw).

    `robot_yaw` must be the yaw of the SAME pose sample the caller intends
    policy_node to rotate the result back out of. Mixing samples — a pose from
    one message with a click rendered against another — displaces the goal by
    however far the robot travelled in between.
    """
    dx = world_x - robot_x
    dy = world_y - robot_y
    cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
    return (cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy)
