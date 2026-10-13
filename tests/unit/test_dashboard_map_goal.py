"""Click-the-map-to-set-the-policy-goal, and the trail clearing on re-arm.

Two features, one file, because they share the same premise: the map view is
now the primary way an operator targets and watches a run, so what it commits
and what it shows have to stay honest.

The important test here is the ROUND TRIP. dashboard_node converts a world
click into the robot-frame offset policy_node's `goal_x`/`goal_y` hold, and
policy_node converts it straight back. A sign error in either direction raises
nothing — it places the goal mirrored or rotated about the robot, which reads
as the policy steering strangely rather than as a bad target.
"""

import math
import os
import re
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("rm_dashboard",):
    _p = os.path.join(_REPO_ROOT, "src", _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from rm_dashboard.frames import world_to_body_offset  # noqa: E402

_INDEX = os.path.join(
    _REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html"
)
_DASHBOARD_NODE = os.path.join(
    _REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "dashboard_node.py"
)
_POLICY_NODE = os.path.join(
    _REPO_ROOT, "src", "rm_policy", "rm_policy", "policy_node.py"
)


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _policy_node_forward(offset_x, offset_y, robot_x, robot_y, robot_yaw):
    """policy_node's body -> world step, written out independently here.

    Deliberately NOT imported from policy_node: that module needs rclpy, which
    the workstation tier does not have. Written from its documented contract
    (`goal_world = robot_xy + R(robot_yaw) @ offset`) so that if either side
    changes, the round trip below fails.
    """
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return (robot_x + c * offset_x - s * offset_y,
            robot_y + s * offset_x + c * offset_y)


# ---------------------------------------------------------------------------
# the round trip
# ---------------------------------------------------------------------------

_POSES = [
    (0.0, 0.0, 0.0),
    (2.5, -0.7, 0.0),
    (1.0, 1.0, math.pi / 2),
    (-3.2, 4.8, -2.7),
    (0.4, -0.4, math.pi),          # exactly at the wrap
    (7.1, 2.2, 3.0 * math.pi),     # a yaw well outside (-pi, pi]
]
_POINTS = [(0.0, 0.0), (1.5, 0.0), (0.0, 1.5), (-2.0, -3.0), (12.0, 0.25)]


@pytest.mark.parametrize("robot", _POSES)
@pytest.mark.parametrize("world", _POINTS)
def test_world_click_round_trips_through_policy_nodes_offset(robot, world):
    """The goal policy_node adopts is the point the operator clicked."""
    rx, ry, ryaw = robot
    wx, wy = world
    ox, oy = world_to_body_offset(wx, wy, rx, ry, ryaw)
    back_x, back_y = _policy_node_forward(ox, oy, rx, ry, ryaw)
    assert back_x == pytest.approx(wx, abs=1e-9)
    assert back_y == pytest.approx(wy, abs=1e-9)


def test_a_point_dead_ahead_becomes_a_pure_forward_offset():
    """Sanity anchor: +x in the body frame is straight ahead, not sideways.

    Guards the specific failure a round-trip test alone cannot see — both
    directions wrong in the same way still round-trips.
    """
    # robot at the origin facing +y (yaw 90 deg); a point 2 m up the y axis is
    # 2 m DEAD AHEAD of it.
    ox, oy = world_to_body_offset(0.0, 2.0, 0.0, 0.0, math.pi / 2)
    assert ox == pytest.approx(2.0, abs=1e-9)
    assert oy == pytest.approx(0.0, abs=1e-9)


def test_a_point_to_the_left_is_positive_y_in_the_body_frame():
    """ROS convention: +y is left. A mirrored conversion fails here."""
    ox, oy = world_to_body_offset(0.0, 1.0, 0.0, 0.0, 0.0)
    assert ox == pytest.approx(0.0, abs=1e-9)
    assert oy == pytest.approx(1.0, abs=1e-9)


def test_policy_node_still_applies_the_rotation_this_inverts():
    """Pin the contract the round trip is written against.

    If policy_node stops rotating the offset by the robot yaw, or stops adding
    it to the robot position, this file's premise is void and the goal would
    land somewhere else entirely.
    """
    src = _read(_POLICY_NODE)
    assert "_rotate_2d(" in src
    assert "self._goal_world_x = self._robot_x + dx" in src
    assert "self._goal_world_y = self._robot_y + dy" in src
    assert "return (c * x - s * y, s * x + c * y)" in src


# ---------------------------------------------------------------------------
# the command surface
# ---------------------------------------------------------------------------

def test_goal_map_is_an_allowed_control():
    src = _read(_DASHBOARD_NODE)
    controls = re.search(r"_CONTROLS = \((.*?)\)", src, re.S).group(1)
    assert '"goal_map"' in controls


def test_goal_map_refuses_a_frame_mismatch_rather_than_converting():
    """map and odom differ by the SLAM correction, which grows as you drive."""
    src = _read(_DASHBOARD_NODE)
    handler = src.split("def _set_goal_from_world")[1].split("def _reset_odom")[0]
    assert "expected_frame" in handler
    # The refusal must come BEFORE anything is written to a policy node, or a
    # mismatched frame would set the goal and then report an error about it.
    # (_retarget is the one path that writes; it takes the per-policy
    # refusals alongside the offsets and never sends a refused one.)
    refusal = handler.index('if frame != expected_frame:')
    assert refusal < handler.index("self._retarget(")
    branch = handler[refusal:handler.index("self._retarget(")]
    assert '"ok": False' in branch
    assert "refused" in branch


def test_goal_map_refuses_when_no_pose_has_been_published():
    """Without a pose there is nothing to express the offset relative to."""
    src = _read(_DASHBOARD_NODE)
    handler = src.split("def _set_goal_from_world")[1].split("def _reset_odom")[0]
    assert 'debug.get("pose")' in handler
    assert "isfinite" in handler


# ---------------------------------------------------------------------------
# the page
# ---------------------------------------------------------------------------

def test_the_map_click_still_does_not_drive():
    """A click arms numbers. Motion needs a separate, explicit press."""
    page = _read(_INDEX)
    click = page.split('$("mapcv").onclick')[1].split("};")[0]
    assert "post(" not in click


def test_committing_a_policy_goal_is_a_separate_button_from_driving():
    """Two commit buttons, not a mode switch on the map.

    A mode the operator has forgotten they are in sends a click to the wrong
    consumer, and nothing about the click says which one it went to.
    """
    page = _read(_INDEX)
    assert 'id="setgoalmap"' in page
    assert 'id="drivebtn"' in page
    handler = page.split('$("setgoalmap").onclick')[1].split("};")[0]
    assert 'post("goal_map"' in handler
    assert "frame" in handler
    assert "confirm(" in handler


def test_the_policy_goal_is_sent_as_a_world_point_not_a_local_offset():
    """The browser must not do the conversion with its own stale pose."""
    page = _read(_INDEX)
    handler = page.split('$("setgoalmap").onclick')[1].split("};")[0]
    for forbidden in ("Math.cos", "Math.sin", "pose"):
        assert forbidden not in handler


def test_the_trail_clears_on_the_rearm_edge_only():
    """A run's trail starts when the robot is armed.

    Cleared on a KILLED -> armed TRANSITION, not on every armed frame, or the
    trail would be wiped continuously and never show anything.
    """
    page = _read(_INDEX)
    assert "prevSafetyState" in page
    guard = page.split("function renderSafety")[1].split("prevSafetyState = name")[0]
    assert 'prevSafetyState === "KILLED"' in guard
    assert 'name !== "KILLED"' in guard
    assert "poseTrail.length = 0" in guard


def test_the_trail_clear_follows_rm_safety_not_the_arm_button():
    """Arming from the CLI or another browser must clear it too."""
    page = _read(_INDEX)
    arm = page.split('$("resetbtn").onclick')[1].split("};")[0]
    assert "poseTrail" not in arm
    assert 'val(snap,"/safety/status")' in page.split("function renderSafety")[1][:400]
