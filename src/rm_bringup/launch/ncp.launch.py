"""
ncp.launch.py — the NCP policy pipeline, behind rm_safety.

What it does:
    Brings up the scan bridge plus ONE policy consumer, and (optionally) the
    safety layer, so the trained NCP can be exercised on the real robot
    without any node ever publishing to /cmd_vel except rm_safety.

        /scan (TG30, ~1982 pts, 360 deg)
              |
              +-- rm_policy/scan_bridge_node --> /ncp/scan (32 beams, 180 deg)
                        |
                        +-- consumer --> /policy/cmd_vel
                                              |
                                              +-- rm_safety --> /cmd_vel

    Only the UPSTREAM consumer reads /ncp/scan. rm_policy's policy_node
    resamples internally and reads the raw /scan directly -- feeding it the
    bridge output would double-resample and fault every tick.

Launch arguments:
    consumer      "upstream" (default) or "policy_node".

                  "upstream" runs NCP-on-Rosmaster/rosmaster/node.py
                  UNMODIFIED via its own interpreter. This is the only way to
                  get its `mode:=debug` reactive rule ("see something -> stop
                  and spin"), which is the fastest way to prove on real
                  hardware that lidar -> observation -> command is live at
                  all. rm_policy's own policy_node has no such mode.

                  "policy_node" runs rm_policy/policy_node, the
                  contract-checked (interface_version 3.0.0), fail-closed
                  production path with staleness watchdogs and a goal fixed
                  from the first /odom.

    mode          "debug" (default) or "ncp". Only meaningful for
                  consumer:=upstream. debug = reactive rule drives and the
                  NCP runs in shadow; ncp = the network drives.

    upstream_path Path to the NCP-on-Rosmaster checkout on the robot.

    weights       Path to a trained actor state_dict. "" (default) means
                  RANDOM WEIGHTS. There is no trained checkpoint in this repo
                  or the upstream one as of 2026-08-27, so `mode:=ncp` with
                  the default exercises the PLUMBING, not a trained policy.

    with_safety   "true" (default) also launches safety.launch.py. Set
                  "false" only when rm_safety is already running.

Why the safety layer is not optional in practice:
    ncp_policy.ACTION_HIGH is v = 2.0 m/s / w = 3.14 rad/s, and neither
    consumer scales it down -- the trained head emits final physical units.
    safety_limits.yaml clamps max_vx to 0.15 m/s. That ~13x gap is the
    intended design (the safety layer is the binding constraint); do NOT
    raise the clamp to "let the policy through". Running either consumer with
    cmd_topic pointed at /cmd_vel would send 2 m/s straight to the base
    driver. The upstream node's default cmd_topic IS /cmd_vel, which is
    exactly why this launch file overrides it explicitly.

Failure mode:
    rm_safety comes up KILLED (safety_limits.yaml startup.start_killed), so
    launching this can never move the robot; motion needs an explicit
    /safety/reset AND command_source:=policy. Both consumers publish zero on
    any fault, and the scan bridge publishes nothing at all on a resampler
    fault, letting each consumer's deadman stop the robot.

Usage (on the robot, after colcon build + source install/setup.bash):
    ros2 launch rm_bringup ncp.launch.py
    ros2 launch rm_bringup ncp.launch.py mode:=ncp
    ros2 launch rm_bringup ncp.launch.py consumer:=policy_node
"""

import pathlib

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parent
_REPO_ROOT = _LAUNCH_DIR.parents[2]
_DEFAULT_INTERFACE_PATH = str(_REPO_ROOT / "config" / "interface.yaml")
_DEFAULT_LIMITS_PATH = str(_REPO_ROOT / "config" / "safety_limits.yaml")

# Sibling checkout convention, matching tests/unit/test_vendor_is_pristine.py.
_DEFAULT_UPSTREAM_PATH = str(_REPO_ROOT.parent / "NCP-on-Rosmaster")

# The bridge's output. Both consumers read this, never the raw /scan.
_NCP_SCAN_TOPIC = "/ncp/scan"

# rm_safety's policy INPUT. Never /cmd_vel — rm_safety is its sole publisher.
_POLICY_CMD_TOPIC = "/policy/cmd_vel"


def _upstream_process(context, *args, **kwargs):
    """Build the upstream node.py process, or nothing if it isn't selected.

    This is an OpaqueFunction rather than a plain ExecuteProcess because the
    `weights` argument must be OMITTED when empty, not passed as an empty
    string. rcl cannot parse a parameter override with no value: launching
    with `-p weights:=` aborts the process at rclpy.init() with

        Couldn't parse parameter override rule: '-p weights:='

    (found on hardware, 2026-08-28). An empty `weights` is the DEFAULT and
    means "random weights", so this is the common path, not a corner case.
    Resolving the substitutions here lets the list be built conditionally.
    """
    if context.perform_substitution(LaunchConfiguration("consumer")) != "upstream":
        return []

    upstream = context.perform_substitution(LaunchConfiguration("upstream_path"))
    mode = context.perform_substitution(LaunchConfiguration("mode"))
    weights = context.perform_substitution(LaunchConfiguration("weights"))

    cmd = [
        "python3", str(pathlib.Path(upstream) / "rosmaster" / "node.py"),
        "--ros-args",
        "-p", f"mode:={mode}",
        "-p", f"scan_topic:={_NCP_SCAN_TOPIC}",
        "-p", "odom_topic:=/odom",
        # OVERRIDE. Upstream defaults cmd_topic to /cmd_vel, which would
        # bypass rm_safety entirely at up to 2 m/s.
        "-p", f"cmd_topic:={_POLICY_CMD_TOPIC}",
    ]
    if weights:
        cmd += ["-p", f"weights:={weights}"]
    return [ExecuteProcess(cmd=cmd, name="ncp_rosmaster", output="screen")]


def generate_launch_description() -> LaunchDescription:
    consumer = LaunchConfiguration("consumer")
    mode = LaunchConfiguration("mode")
    upstream_path = LaunchConfiguration("upstream_path")
    weights = LaunchConfiguration("weights")
    with_safety = LaunchConfiguration("with_safety")
    command_source = LaunchConfiguration("command_source")

    # `upstream` is selected inside _upstream_process (an OpaqueFunction), so
    # only the policy_node branch needs a launch-time condition here.
    is_policy_node = IfCondition(
        PythonExpression(["'", consumer, "' == 'policy_node'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "consumer", default_value="upstream",
            description="'upstream' (NCP-on-Rosmaster node.py, has debug mode) "
                        "or 'policy_node' (rm_policy, contract-checked).",
        ),
        DeclareLaunchArgument(
            "mode", default_value="debug",
            description="'debug' (reactive rule drives, NCP in shadow) or "
                        "'ncp'. Only used by consumer:=upstream.",
        ),
        DeclareLaunchArgument(
            "upstream_path", default_value=_DEFAULT_UPSTREAM_PATH,
            description="Path to the NCP-on-Rosmaster checkout.",
        ),
        DeclareLaunchArgument(
            "weights", default_value="",
            description="Trained actor state_dict. '' = RANDOM WEIGHTS.",
        ),
        DeclareLaunchArgument(
            "with_safety", default_value="true",
            description="Also launch safety.launch.py.",
        ),
        # Passed straight through to safety.launch.py. Defaults to "teleop",
        # matching that file: with "teleop", rm_safety IGNORES /policy/cmd_vel
        # entirely, so the whole NCP chain runs and publishes but CANNOT reach
        # the base driver. That is the correct default for bringing the stack
        # up and watching it. Set "policy" only when you intend the policy to
        # drive, with an operator and a kill switch present.
        DeclareLaunchArgument(
            "command_source", default_value="teleop",
            description="rm_safety's active source: 'teleop' (default, policy "
                        "cannot drive) or 'policy'.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_LAUNCH_DIR / "safety.launch.py")),
            condition=IfCondition(with_safety),
            launch_arguments={"command_source": command_source}.items(),
        ),

        Node(
            package="rm_policy",
            executable="scan_bridge_node",
            name="ncp_scan_bridge",
            output="screen",
            parameters=[{
                "scan_topic": "/scan",
                "output_topic": _NCP_SCAN_TOPIC,
            }],
        ),

        OpaqueFunction(function=_upstream_process),

        Node(
            condition=is_policy_node,
            package="rm_policy",
            executable="policy_node",
            name="policy_node",
            output="screen",
            parameters=[{
                # RAW /scan, deliberately NOT the bridge output: policy_node
                # resamples internally, and re-resampling an already-reduced
                # 174.375 deg scan into a 180 deg arc faults every tick.
                "scan_topic": "/scan",
                "odom_topic": "/odom",
                "cmd_vel_topic": _POLICY_CMD_TOPIC,
                "weights": weights,
                "interface_config_path": _DEFAULT_INTERFACE_PATH,
                "safety_limits_path": _DEFAULT_LIMITS_PATH,
            }],
        ),
    ])
