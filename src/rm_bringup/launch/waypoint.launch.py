"""
waypoint.launch.py — the go-to-waypoint controller, on its own.

What it does:
    Starts rm_waypoint/waypoint_node, which drives the robot to a point an
    operator picks — on the dashboard map, or over `/waypoint/goto` directly.
    It exists for repositioning between policy runs ("drive back to the start
    so I can go again") and is not part of the NCP pipeline; nothing in that
    pipeline reads it.

    ncp.launch.py includes this behind `with_waypoint` (default FALSE), so
    this file is for adding the controller to a stack that is already up.

Why the default is off:
    This is the only autonomous motion path in the workspace that is not the
    policy, and on a robot with no physical E-stop a new one should be
    something you ask for rather than something you get. Nothing is lost by
    the default: the dashboard greys its DRIVE THERE button and says why when
    the service is absent, rather than offering a control that does nothing.

Where it sits, which is the whole safety argument:

    /waypoint/goto -> waypoint_node -> /teleop/cmd_vel -> rm_safety -> /cmd_vel

    It publishes `/teleop/cmd_vel`, an INPUT to rm_safety exactly like the
    gamepad, so it drives behind the same clamps, the same slow and stop arcs,
    the same acceleration limiter and the same kill as every other source. It
    has no path to `/cmd_vel`. The topic is a module constant in the node, not
    a parameter, so no launch argument can point it anywhere else.

    THREE INDEPENDENT THINGS must be true before it can move the robot, and
    starting this launch file is only the first:

        1. this node is running and an operator sends it a waypoint
        2. rm_safety's command_source is 'teleop'
        3. rm_safety has been armed (/safety/reset)

NOTE ON /teleop/cmd_vel SHARING:
    teleop_joy.launch.py's remapped `joy_ctrl` publishes the same topic. That
    is not a conflict in the way two /cmd_vel publishers would be — rm_safety
    takes the latest message either way — but while a waypoint run is ACTIVE,
    moving the gamepad stick fights it. The node publishes nothing at all when
    idle (not even zeros) precisely so the gamepad is unencumbered the rest of
    the time. Cancel the waypoint before driving manually.

Launch arguments:
    config_path   config/waypoint.yaml. THE authority for every limit and
                  gain; the node refuses to start without it, and refuses a
                  file with a missing key rather than defaulting it.
    pose_source   "odom" (default) or "map". Must match the frame waypoints
                  are sent in, and "map" needs slam_toolbox running. A
                  waypoint in the wrong frame is REFUSED, not converted.
    dry_run       "false" by default. "true" computes and reports everything
                  on /waypoint/status but publishes no Twist — the way to
                  watch the controller's intent without motion.
    rate_hz       0.0 means "take it from the config" (10 Hz). Must stay
                  comfortably above rm_safety's 300 ms command timeout.

Failure mode:
    Fail-closed and silent when idle. Any fault — a missing or stale pose, a
    timeout, a stalled range, a kill, an exception — stops the robot and
    latches idle with a reason on /waypoint/status. It never retries by
    itself. See the node's docstring.

Usage:
    ros2 launch rm_bringup waypoint.launch.py
    ros2 launch rm_bringup waypoint.launch.py pose_source:=map
    ros2 service call /waypoint/goto rm_interfaces/srv/SetWaypoint \\
        "{frame_id: 'odom', x: 1.0, y: 0.0}"
    ros2 service call /waypoint/cancel std_srvs/srv/Trigger
"""

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolved from this file rather than from an install share/ directory, so the
# same path works from a colcon install, a --symlink-install tree, and a
# checkout run directly on the robot. launch/ -> rm_bringup/ -> src/ -> repo.
_LAUNCH_DIR = pathlib.Path(__file__).resolve().parent
_REPO_ROOT = _LAUNCH_DIR.parents[2]
_DEFAULT_CONFIG = str(_REPO_ROOT / "config" / "waypoint.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=_DEFAULT_CONFIG),
        DeclareLaunchArgument("pose_source", default_value="odom"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("rate_hz", default_value="0.0"),
        Node(
            package="rm_waypoint",
            executable="waypoint_node",
            name="waypoint_node",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "pose_source": LaunchConfiguration("pose_source"),
                "dry_run": LaunchConfiguration("dry_run"),
                "rate_hz": LaunchConfiguration("rate_hz"),
            }],
        ),
    ])
