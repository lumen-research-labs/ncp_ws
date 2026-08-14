"""
safety.launch.py — Launch the rm_safety node (sole publisher of /cmd_vel).

What it does:
    Starts a single `rm_safety/safety_node` with parameters sourced from
    `config/safety_limits.yaml`.  The node itself loads all numeric limits from
    that YAML at runtime (no limits are duplicated here); this launch file only
    sets the three node-level parameters that are NOT part of the limits file:
    `command_source`, `safety_loop_rate_hz`, and `safety_limits_path`.

Launch arguments:
    command_source        (default "teleop")  — active command source. MUST stay
                                                 "teleop" until Phase 4 sign-off;
                                                 "policy" lets /policy/cmd_vel
                                                 drive.  There is no trained
                                                 policy in Phase 2/3.
    safety_loop_rate_hz   (default "20.0")     — control-loop rate. Kept at 20 Hz
                                                 because it must be
                                                 >= publishing.zero_publish_rate_hz
                                                 (20 Hz in safety_limits.yaml) for
                                                 the continuous-zero-when-killed
                                                 guarantee.  This is the SAFETY
                                                 republish rate, independent of
                                                 the 5 Hz Phase 3 POLICY control
                                                 rate (CLAUDE.md 2026-08-12 s3).
    safety_limits_path    (default: in-repo    — absolute path to
                           config/safety_limits.yaml, resolved relative to this
                           launch file's source location under `--symlink-install`)

Why killed-on-startup is not a launch concern:
    The node comes up KILLED regardless of this launch file
    (safety_limits.yaml: startup.start_killed=true).  Launching can never make
    the robot move; motion needs an explicit /safety/reset.  Do NOT add a
    parameter here to override that.

Failure mode:
    If safety_limits.yaml cannot be found/parsed, the node raises on init and
    exits — it never starts in an unknown-limits state.  Prefer overriding
    safety_limits_path explicitly on the robot (durable fix: point it at the
    in-repo config of the cloned repo) rather than relying on the default
    source-relative resolution.

Usage:
    ros2 launch rm_bringup safety.launch.py
    ros2 launch rm_bringup safety.launch.py command_source:=teleop \
        safety_limits_path:=/abs/path/to/config/safety_limits.yaml
"""

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Default safety_limits.yaml path, resolved relative to THIS file's source.
# Under `colcon build --symlink-install` (the project standard) the installed
# launch file is a symlink back to this source tree, so __file__ points here:
#   <repo>/src/rm_bringup/launch/safety.launch.py
# parents: [0]=launch [1]=rm_bringup [2]=src [3]=<repo root>
# -> <repo root>/config/safety_limits.yaml
# This mirrors the node's own default resolution and is overridable via the
# `safety_limits_path` launch argument for a non-symlink install on the robot.
# ---------------------------------------------------------------------------
_DEFAULT_LIMITS_PATH = str(
    pathlib.Path(__file__).resolve().parents[3] / "config" / "safety_limits.yaml"
)


def generate_launch_description() -> LaunchDescription:
    command_source = LaunchConfiguration("command_source")
    safety_loop_rate_hz = LaunchConfiguration("safety_loop_rate_hz")
    safety_limits_path = LaunchConfiguration("safety_limits_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "command_source",
            default_value="teleop",
            description="Active command source: 'teleop' (default) or 'policy'. "
                        "Keep 'teleop' until Phase 4 sign-off.",
        ),
        DeclareLaunchArgument(
            "safety_loop_rate_hz",
            default_value="20.0",
            description="Safety control-loop / republish rate (Hz). Must be "
                        ">= publishing.zero_publish_rate_hz in safety_limits.yaml.",
        ),
        DeclareLaunchArgument(
            "safety_limits_path",
            default_value=_DEFAULT_LIMITS_PATH,
            description="Absolute path to config/safety_limits.yaml.",
        ),
        Node(
            package="rm_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            parameters=[{
                "command_source": command_source,
                "safety_loop_rate_hz": safety_loop_rate_hz,
                "safety_limits_path": safety_limits_path,
            }],
            # rm_safety is the SOLE publisher of /cmd_vel. Do not add
            # remappings that would route another node onto /cmd_vel, and never
            # launch cpt_b2_engine (previous team's stack) alongside this.
        ),
    ])
