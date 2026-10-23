"""
dashboard.launch.py — the web dashboard, on its own.

What it does:
    Starts rm_dashboard/dashboard_node, which serves the live stack view at
    http://<robot>:8080/. Nothing else. ncp.launch.py includes this behind
    `with_dashboard` (default true), so this file is for the case where the
    stack is already up — started by bench_up.sh, or by an earlier
    ncp.launch.py you do not want to restart and lose the CfC's recurrent
    state over.

Why the dashboard runs ON THE ROBOT:
    It needs only TCP to reach a browser, so it works whenever SSH does.
    Running it workstation-side would put it behind DDS discovery across the
    lab network, which is the single most repeatedly-broken thing in this
    project (the 239.255.0.1 multicast drop, the maxInitialPeersRange default,
    the RELIABLE/BEST_EFFORT mismatch). A diagnostic must not share a failure
    mode with the thing it is diagnosing.

Launch arguments:
    port             8080 by default.
    host             0.0.0.0 by default — the workstation must reach it, so
                     binding loopback would defeat the purpose. This is a lab
                     network with no WAN uplink; on any other network,
                     understand that anyone who can route to the robot can
                     load this page and, with controls enabled, kill or arm it.
    enable_controls  "true" by default. "false" serves a read-only page: the
                     command endpoints refuse, and the buttons render disabled.
    stream_hz        10.0 — snapshot broadcast rate, above the 5 Hz control
                     loop so no policy tick is missed.
    scan_stride      6 — decimation of /scan FOR DISPLAY ONLY. The TG30's
                     ~2000 points at 10 Hz would be ~200 KB/s of JSON for a
                     plot no denser than the pixels available. Every statistic
                     (min range, dropout count) is computed on the full scan
                     before decimation, so nothing derived is affected.
    map_max_cells    400 — longest side of the SLAM occupancy grid served to
                     the browser. The grid is decimated by taking the MOST
                     OCCUPIED cell of each block, never an average: a wall is
                     one or two cells thick at 0.05 m resolution, and
                     averaging deletes walls at any real decimation factor.
                     The map is served on its own endpoint and fetched only
                     when it changes, so it never rides the 10 Hz stream.
    tf_poll_hz       5.0 — how often map -> odom (the SLAM correction) and
                     map -> base_footprint (the corrected pose) are read.
    topics_path      config/topics.yaml, the command-source table: one
                     source button per row, and the policy rows' debug
                     streams and nodes. Defaults to the in-repo file.

Failure mode:
    The node publishes NO topics, so it can never become a second /cmd_vel
    publisher. Its only outbound effects are /safety/kill, /safety/reset,
    parameter sets on safety_node/policy_node, and the two /waypoint services
    — none of which can start motion on their own. A source change
    force-kills; arming stays a separate, explicit act; and a waypoint reaches
    the wheels only via rm_waypoint -> /teleop/cmd_vel -> rm_safety, which
    requires the robot to be ARMED and the source to be TELEOP already. If the
    port is already bound the node logs the error and continues headless
    rather than dying.

Usage:
    ros2 launch rm_bringup dashboard.launch.py
    ros2 launch rm_bringup dashboard.launch.py enable_controls:=false
"""

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_TOPICS_PATH = str(
    pathlib.Path(__file__).resolve().parents[3] / "config" / "topics.yaml"
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("topics_path", default_value=_DEFAULT_TOPICS_PATH),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("enable_controls", default_value="true"),
        DeclareLaunchArgument("stream_hz", default_value="10.0"),
        DeclareLaunchArgument("scan_stride", default_value="6"),
        DeclareLaunchArgument("map_max_cells", default_value="400"),
        DeclareLaunchArgument("tf_poll_hz", default_value="5.0"),
        Node(
            package="rm_dashboard",
            executable="dashboard_node",
            name="dashboard_node",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "stream_hz": LaunchConfiguration("stream_hz"),
                "scan_stride": LaunchConfiguration("scan_stride"),
                "enable_controls": LaunchConfiguration("enable_controls"),
                "map_max_cells": LaunchConfiguration("map_max_cells"),
                "tf_poll_hz": LaunchConfiguration("tf_poll_hz"),
                "topics_path": LaunchConfiguration("topics_path"),
            }],
        ),
    ])
