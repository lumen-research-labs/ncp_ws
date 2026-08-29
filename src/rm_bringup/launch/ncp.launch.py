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

    with_lidar    "true" (default) starts the YDLIDAR TG30 on its by-id
                  CP2102 port. Set "false" if the lidar is already up.
    with_base     "true" (default) starts rm_base (base.launch.py), the
                  mecanum driver that OWNS the base-board serial port.
    with_odom     "true" (default) starts odom.launch.py — the vendor odom
                  integrator + madgwick + EKF, fed by rm_base's /vel_raw.
                  Without /odom, rm_safety's staleness watchdog fires at
                  precedence 3 (above the zone checks) and zeros EVERY
                  command with reason "odom_stale", whatever you send.

    control_mode  rm_base mode: "closed_loop" (default here — the path
                  validated on the floor 2026-08-29) or "open_loop".
    dry_run       "false" (default here). NOTE this differs from
                  base.launch.py's own default of "true": bringing up the
                  full NCP stack implies intending to drive. rm_safety still
                  starts KILLED, and command_source still defaults to
                  "teleop", so two explicit acts are still required.
    calibration_path  path to config/base_calibration.yaml.

Bringing the WHOLE stack up:
    This file now starts everything ROS-side, so it is the one command. It
    does NOT do the host-level cleanup that must happen first — killing the
    vendor Mcnamu_driver_X3 (single-writer serial port), killing the Yahboom
    rosmaster_main.py autostart (recurs every boot), and creating
    /var/lib/rm_safety. Run scripts/prep_robot.sh once first.

    Do NOT also run scripts/bench_up.sh: it starts its own safety, base, odom
    and lidar, and two safety_nodes means two publishers on /cmd_vel — the
    violation this whole design exists to prevent. bench_up.sh is for base
    calibration work on its own; this file is for the full pipeline.

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
    bash scripts/prep_robot.sh                      # once, first

    # full stack, nothing can move (command_source defaults to teleop)
    ros2 launch rm_bringup ncp.launch.py

    # full stack, policy allowed to drive — operator + kill switch present
    ros2 launch rm_bringup ncp.launch.py command_source:=policy

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

_DEFAULT_CALIBRATION_PATH = str(_REPO_ROOT / "config" / "base_calibration.yaml")

# by-id, never /dev/rplidar: that symlink races between the lidar (CP2102) and
# the base board (CH340) across reboots and has pointed at the wrong device on
# multiple boots. See the robot-ssh skill.
_LIDAR_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)

# Matches Yahboom's "4ROS" config. frame_id:=laser_link is mandatory -- the
# driver default "laser" is not in the TF tree.
_LIDAR_PARAMS = {
    "port": _LIDAR_PORT,
    "baudrate": 512000,
    "lidar_type": 0,
    "device_type": 0,
    "sample_rate": 20,
    "isSingleChannel": False,
    "support_motor_dtr": True,
    "reversion": True,
    "inverted": True,
    "range_max": 64.0,
    "range_min": 0.01,
    "frequency": 10.0,
    "frame_id": "laser_link",
}

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
    with_lidar = LaunchConfiguration("with_lidar")
    with_base = LaunchConfiguration("with_base")
    with_odom = LaunchConfiguration("with_odom")
    control_mode = LaunchConfiguration("control_mode")
    dry_run = LaunchConfiguration("dry_run")
    calibration_path = LaunchConfiguration("calibration_path")

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
        DeclareLaunchArgument(
            "with_lidar", default_value="true",
            description="Start the YDLIDAR TG30 on its by-id CP2102 port.",
        ),
        DeclareLaunchArgument(
            "with_base", default_value="true",
            description="Start rm_base, the mecanum driver. It OWNS the "
                        "base-board serial port -- the vendor "
                        "Mcnamu_driver_X3 must already be dead.",
        ),
        DeclareLaunchArgument(
            "with_odom", default_value="true",
            description="Start the odom chain. Without /odom rm_safety zeros "
                        "every command with reason 'odom_stale'.",
        ),
        DeclareLaunchArgument(
            "control_mode", default_value="closed_loop",
            description="rm_base mode. closed_loop is the floor-validated "
                        "path; open_loop drives PWM straight from the table.",
        ),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true sends only zero PWM. Defaults FALSE here "
                        "(unlike base.launch.py) because bringing up the full "
                        "NCP stack implies intending to drive; rm_safety "
                        "still starts KILLED.",
        ),
        DeclareLaunchArgument(
            "calibration_path", default_value=_DEFAULT_CALIBRATION_PATH,
            description="Path to config/base_calibration.yaml.",
        ),

        # ---- hardware layer, bottom up -------------------------------------
        # Lidar first: the scan bridge and both consumers are useless without
        # /scan, and rm_safety treats missing perception as a stop condition.
        Node(
            condition=IfCondition(with_lidar),
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            parameters=[_LIDAR_PARAMS],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_LAUNCH_DIR / "base.launch.py")),
            condition=IfCondition(with_base),
            launch_arguments={
                "control_mode": control_mode,
                "dry_run": dry_run,
                "calibration_path": calibration_path,
            }.items(),
        ),

        # Consumes rm_base's /vel_raw + /imu/data_raw, produces /odom.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_LAUNCH_DIR / "odom.launch.py")),
            condition=IfCondition(with_odom),
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
