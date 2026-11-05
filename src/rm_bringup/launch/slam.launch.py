"""
slam.launch.py — slam_toolbox (async online mapping), for a corrected pose.

What it does:
    Runs `slam_toolbox`'s async online node against the raw /scan, publishing
    the `map -> odom` transform and a /map occupancy grid. Nothing else in the
    stack changes shape: the EKF still publishes `odom -> base_footprint`, so
    this ADDS a correcting frame above the existing chain rather than
    replacing any of it.

        map --(slam_toolbox)--> odom --(EKF)--> base_footprint --> base_link

Why it exists:
    policy_node derives the robot's pose, and therefore `goal_local` — 2 of
    the 36 numbers the trained network reads — from /odom. That is dead
    reckoning with a tape-fitted scale and wheel slip baked in; it over-reads
    linear distance by ~5% (measured 2026-08-29) and the operator confirmed
    the goal visibly drifting on driven runs. Fixing the goal in the map frame
    instead is what stops it migrating.

    Set `pose_source:=map` on policy_node (ncp.launch.py does this for you
    when with_slam is true) to actually consume it. Running this launch file
    alone changes nothing about how the robot drives — it only publishes a
    frame nobody is reading yet, which is a deliberately safe intermediate
    state to bring up and watch.

What it does NOT fix:
    The odometry itself. base_node still integrates the same encoders and the
    EKF still fuses them. /odom remains untrustworthy for anything reading it
    directly, including rm_safety's staleness watchdog (which only needs
    presence, not accuracy) and any future heading-hold controller.

Launch arguments:
    params_file  path to config/slam_toolbox.yaml.
    scan_topic   /scan_sanitized by default — full resolution, with the
                 TG30's 0.0 no-return sentinel rewritten to +inf by the
                 scan_sanitizer node this file also starts. NOT the raw /scan:
                 ~31-39% of rays are that sentinel and slam_toolbox does not
                 know it means "nothing there". NOT /ncp/scan either — that is
                 720 beams over the forward 270 deg, reduced for the policy;
                 scan matching wants every ray and the full 360 deg.

Failure mode:
    Additive and non-blocking. slam_toolbox publishes a transform; it does not
    publish /cmd_vel and cannot move the robot. If it dies or never converges,
    `map -> odom` goes stale and policy_node's pose lookup fails — which is a
    FAIL-CLOSED path there (zero velocity, reason `pose_*`), not a fallback to
    a stale pose. With `pose_source:=odom` (the default) this node is inert.

Usage:
    ros2 launch rm_bringup slam.launch.py
"""

import pathlib

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parent
_REPO_ROOT = _LAUNCH_DIR.parents[2]
_DEFAULT_PARAMS = str(_REPO_ROOT / "config" / "slam_toolbox.yaml")
_DEFAULT_LIMITS_PATH = _REPO_ROOT / "config" / "safety_limits.yaml"


def _body_radius_m(limits_path: pathlib.Path = _DEFAULT_LIMITS_PATH) -> float:
    """estop.body_radius_m from config/safety_limits.yaml: returns nearer than
    this are the robot's own chassis (the corners, 0.17-0.23 m) and the
    sanitizer turns them into +inf so slam_toolbox never matches on them.
    A missing key is a startup error, never a silent 0.0."""
    with open(limits_path, "r", encoding="utf-8") as fh:
        limits = yaml.safe_load(fh)
    try:
        return float(limits["estop"]["body_radius_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f"slam.launch.py: {limits_path}: estop.body_radius_m missing or not a "
            f"number ({exc!r}); the scan sanitizer needs it."
        ) from exc


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=_DEFAULT_PARAMS,
            description="Path to config/slam_toolbox.yaml.",
        ),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan_sanitized",
            description="Sanitized full-resolution scan (the sanitizer node "
                        "below produces it). NOT the raw /scan — see the "
                        "sanitizer's docstring; ~31-39%% of TG30 rays are a "
                        "0.0 no-return sentinel that slam_toolbox does not "
                        "know about. NEVER /ncp/scan either: that is the "
                        "reduced 720-beam forward-270 policy input.",
        ),

        # Full-resolution /scan with the 0.0 no-return sentinel rewritten to
        # +inf. slam_toolbox reads the raw scan with nothing of ours in
        # between, and whether its min_laser_range discards or clamps a short
        # reading is an implementation detail of a dependency we would
        # otherwise be betting the pose estimate on.
        Node(
            package="rm_policy",
            executable="scan_sanitizer_node",
            name="scan_sanitizer",
            output="screen",
            parameters=[{
                "scan_topic": "/scan",
                "output_topic": "/scan_sanitized",
                # The chassis corners (0.17-0.23 m) are above slam_toolbox's
                # 0.15 m min_laser_range and would reach its scan matcher as
                # points fixed in the robot frame. Measured 2026-11-04.
                "body_radius_m": _body_radius_m(),
            }],
        ),
        # respawn=True is what makes the dashboard's RESET MAP control work:
        # slam_toolbox 2.6.9 has no reset service, so a fresh map is a restart
        # of this process. The dashboard terminates it and the launch system
        # brings it back here with the same parameters and an empty pose
        # graph. Consequence worth knowing: a slam_toolbox CRASH also comes
        # back silently with a fresh map. The dashboard's SLAM panel shows it
        # (correction history and loop-closure count start over) and the
        # launch log records the exit. stack_down.sh is unaffected: it stops
        # the launch parent before the children, and a shutting-down launch
        # does not respawn.
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            respawn=True,
            respawn_delay=0.5,
            parameters=[
                LaunchConfiguration("params_file"),
                # use_sim_time false is explicit, not inherited: bernard-1 has
                # no NTP and a hand-pushed clock, so anything ambiguous about
                # the time source here turns into TF extrapolation errors that
                # look like a SLAM failure.
                {"use_sim_time": False,
                 "scan_topic": LaunchConfiguration("scan_topic")},
            ],
        ),
    ])
