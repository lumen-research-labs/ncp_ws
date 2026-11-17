"""
ncp.launch.py — the NCP policy pipeline, behind rm_safety.

What it does:
    Brings up the scan bridge plus one policy node PER PROFILE (default:
    both trained policies), and (optionally) the safety layer, so the
    trained NCPs can be exercised on the real robot without any node ever
    publishing to /cmd_vel except rm_safety. Each policy node publishes its
    own request on its own topic; rm_safety forwards only the one named by
    command_source, so switching policies is a parameter set, never a
    restart.

        /scan (TG30, ~1982 pts, 360 deg)
              |
              +-- rm_policy/scan_bridge_node --> /ncp/scan (first profile's beams)
              |
              +-- policy_node_m8v2 --> /policy/m8v2/cmd_vel --+
              |                                               +-- rm_safety --> /cmd_vel
              +-- policy_node_m4   --> /policy/m4/cmd_vel   --+   (command_source picks ONE)

    Node names, command topics and debug streams come from the rows of
    config/topics.yaml `command_sources`, the same table rm_safety validates
    command_source against and the dashboard draws its buttons from.

    rm_dashboard subscribes to every arrow above and serves it as one live
    page on http://<robot>:8080/. It is a leaf: nothing depends on it and it
    publishes nothing.

    Only the UPSTREAM consumer reads /ncp/scan. rm_policy's policy_node
    resamples internally and reads the raw /scan directly -- feeding it the
    bridge output would double-resample and fault every tick.

Launch arguments:
    consumer      "policy_node" (DEFAULT) or "upstream".

                  "policy_node" runs rm_policy/policy_node, the
                  contract-checked (interface_version 4.0.0), fail-closed
                  production path with staleness watchdogs and a goal fixed
                  from the first /odom. This is the default because a trained
                  checkpoint now exists (2026-08-29) and this is the path that
                  verifies the contract the checkpoint was exported against.

                  "upstream" runs NCP-on-Rosmaster/rosmaster/node.py
                  UNMODIFIED via its own interpreter. This is the only way to
                  get its `mode:=debug` reactive rule ("see something -> stop
                  and spin"), which is the fastest way to prove on real
                  hardware that lidar -> observation -> command is live at
                  all. rm_policy's own policy_node has no such mode. It is
                  also the A/B reference: both run the SAME vendored network
                  code, so a disagreement between them is a wrapper bug, not
                  a policy difference.

    mode          "debug" (default) or "ncp". Only meaningful for
                  consumer:=upstream. debug = reactive rule drives and the
                  NCP runs in shadow; ncp = the network drives.

    upstream_path Path to the NCP-on-Rosmaster checkout on the robot.

    obstacle_dist Debug rule only: spin away from anything closer than this,
                  in metres. Default 0.9 here, overriding upstream's 1.5.
                  The three thresholds must stay ordered so the POLICY reacts
                  before the SAFETY layer has to:
                      0.90 m  policy spins away       (this)
                      0.50 m  safety scales velocity  (slow_zone_m)
                      0.35 m  safety zeroes + latches (stop_zone_m)
                  Upstream's 1.5 is measured across the whole 270 deg arc
                  including the sides, so in a normal room something is nearly
                  always inside it and the robot spins continuously instead of
                  ever cruising.

    profiles      Comma-separated list of trained policies to run, default
                  "m8v2,m4,m9,dynabarn" (all). Each name is a row of the _PROFILES
                  table below (contract file + checkpoint) AND a row of
                  config/topics.yaml command_sources (node name, command
                  topic, debug stream):
                      "m8v2"  config/interface.yaml (4.0.0), 720 beams over
                              270 deg, <upstream>/weights/actor.pt;
                              policy_node_m8v2 -> /policy/m8v2/cmd_vel
                      "m4"    config/interface_m4.yaml (3.0.0), 32 beams
                              over 180 deg, <upstream>/weights/actor_m4.pt;
                              policy_node_m4 -> /policy/m4/cmd_vel
                      "m9"    config/interface_m9.yaml (5.0.0), 720 beams
                              over 270 deg, trained at the robot's clamps
                              (+-0.25 m/s, +-0.8 rad/s), <upstream>/weights/
                              actor_m9.pt; policy_node_m9 -> /policy/m9/cmd_vel
                      "dynabarn" config/interface_dynabarn.yaml (5.1.0), the
                              m9 contract with the lumen gauntlet's DynaBARN-X3
                              checkpoint, <upstream>/weights/actor_dynabarn.pt;
                              policy_node_dynabarn -> /policy/dynabarn/cmd_vel
                  Every listed policy runs at once. rm_safety forwards only
                  the row named by command_source, so the live policy is
                  chosen with `ros2 param set /safety_node command_source
                  m4` or the dashboard's source buttons, and a change still
                  force-kills. The scan bridge serves the FIRST listed
                  profile's contract. policy_node builds the network to its
                  contract's width and loads its checkpoint strictly, so a
                  mismatched pair refuses to start; the dashboard header
                  shows the live policy's version and beam count.
                  consumer:=upstream needs the list to START with m8v2 (the
                  vendored upstream module is fixed at 720 beams).

    interface_config_path
                  Override the contract file of ONE profile: pass
                  profiles:=<one> with it. Default "profile" = from the
                  table. Rarely needed.

    weights       Path to a trained actor state_dict, for ONE profile (pass
                  profiles:=<one> with it). Default "profile" = the
                  profile's checkpoint in the upstream checkout. Pass
                  weights:='' for random weights, which exercises the
                  PLUMBING and nothing else.

                  The checkpoint lives in the UPSTREAM repo. It must be
                  present on the robot: bernard-1 has no internet, so it gets
                  there by scripts/deploy_to_robot.sh, not by git pull. Its
                  sha256 is logged at policy_node startup and published on
                  /ncp_debug, so a recorded run is traceable to exact bytes.

    with_safety   "true" (default) also launches safety.launch.py. Set
                  "false" only when rm_safety is already running.

    with_lidar    "true" (default) starts the lidar driver on its by-id
                  CP2102 port. Set "false" if the lidar is already up.
    lidar         Which lidar this robot carries: "tg30" (default; bernard-1
                  and bernard-2, YDLIDAR TG30 via ydlidar_ros2_driver at
                  512000 baud) or "a1" (bernard-3, a SLAMTEC RPLIDAR A1 via
                  sllidar_ros2 at 115200 baud, scan_mode Standard). Both
                  adapters are a CP2102 on the same by-id path; only the
                  protocol differs, and the wrong driver opens the port and
                  then reads nothing (2026-11-18). Anything else is refused.
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

    with_slam     "true" (DEFAULT) starts slam_toolbox, which publishes
                  map -> odom and corrects the odometry drift, and switches
                  pose_source to "map" with it. Default TRUE since
                  2026-08-31: /odom's yaw drifts ~0.6 deg/s with the robot
                  STANDING STILL (measured, 120 s stationary — see below),
                  which rotates goal_local bodily around the robot. SLAM
                  cancels it; without SLAM nothing does.
    pose_source   policy_node's pose: "odom" (dead reckoning, drifts) or
                  "map" (SLAM-corrected via TF). Empty (default) follows
                  with_slam, so the default stack drives on "map". Set
                  explicitly to "odom" WITH with_slam:=true to run SLAM while
                  still driving on odometry — the A/B, and the safe way to
                  watch a map build before trusting it.

                  WHY map IS THE DEFAULT, measured 2026-08-31 over three runs
                  (scripts/diagnostics/pose_ab.py):
                    * stationary, 120 s: map-frame pose held to 0.028 m in x
                      and 0.032 m in y — SLAM is stable at rest.
                    * over the same 120 s the correction ratcheted +71.4 deg
                      of YAW, i.e. odom yaw drifts ~0.6 deg/s at rest. A 90 s
                      run on pose_source:=odom accumulates ~54 deg of heading
                      error, and goal_local is computed by rotating the goal
                      into the body frame.
                    * driven: odom path over-read map path by 4.5-7.7 %,
                      consistent with the long-known linear over-read.
                  THE COST is real: ~6 correction steps per 2 min at rest,
                  each one resetting the CfC recurrent state. That is the
                  cheaper failure — a reset is visible and bounded, a rotating
                  goal is neither.

    with_dashboard    "true" (default) also starts rm_dashboard, the live web
                  view of every stage at http://<robot>:8080/. It subscribes
                  only — it publishes no topic and cannot become a second
                  /cmd_vel publisher. dashboard_port and dashboard_controls
                  are passed through; dashboard_controls:=false serves a
                  read-only page. The page also shows SLAM health — the
                  map -> odom correction, its jumps, and the map itself.

    with_waypoint "false" (DEFAULT) — set true to start rm_waypoint, the
                  go-to-waypoint controller used to reposition the robot
                  between runs (click a point on the dashboard's map). Off by
                  default because it is the only autonomous motion path here
                  that is not the policy. It publishes /teleop/cmd_vel, an
                  INPUT to rm_safety, so it drives behind the same clamps,
                  arcs and kill as everything else, and it reaches the wheels
                  only when command_source is 'teleop' AND the robot has been
                  armed. Its pose_source follows this file's, so a waypoint is
                  always in the frame the policy is driving in.

    (roam)        NOT a launch argument. Every policy node reads
                  config/roam.yaml at startup, and roam is switched at
                  runtime (the dashboard's ROAM button, or `ros2 param set
                  /policy_node_m4 roam true`): an operator act after
                  bring-up, like arming. On, each policy node draws its own
                  next goal on arrival or timeout and keeps going; it arms
                  nothing. See docs/roam.md.

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
    safety_limits.yaml clamps max_vx to 0.25 m/s and max_wz to 0.8 rad/s.
    That ~8x / ~4x gap is the intended design (the safety layer is the binding
    constraint); do NOT raise the clamp to "let the policy through". The
    consequence to hold onto when reading the policy's behaviour: it will run
    almost entirely saturated, in a regime it never saw in training, so what
    the robot does under these clamps is evidence about the PIPELINE, not
    about whether the policy is any good. Running either consumer with
    cmd_topic pointed at /cmd_vel would send 2 m/s straight to the base
    driver. The upstream node's default cmd_topic IS /cmd_vel, which is
    exactly why this launch file overrides it explicitly.

Failure mode:
    rm_safety comes up KILLED (safety_limits.yaml startup.start_killed), so
    launching this can never move the robot; motion needs an explicit
    /safety/reset AND command_source set to a policy row (m8v2, m4, or
    policy for the upstream consumer). Every policy node publishes zero on
    any fault, and the scan bridge publishes nothing at all on a resampler
    fault, letting each consumer's deadman stop the robot.

Usage (on the robot, after colcon build + source install/setup.bash):
    bash scripts/prep_robot.sh                      # once, first

    # full stack, nothing can move (command_source defaults to teleop)
    ros2 launch rm_bringup ncp.launch.py

    # full stack, the M8-v2 policy allowed to drive — operator + kill switch
    ros2 launch rm_bringup ncp.launch.py command_source:=m8v2

    # the M4 policy alone (its node is the only one started)
    ros2 launch rm_bringup ncp.launch.py profiles:=m4 command_source:=m4

    # the reactive debug rule, for proving the sensor path end to end
    ros2 launch rm_bringup ncp.launch.py consumer:=upstream mode:=debug

    # plumbing only, no checkpoint (one profile: weights:= overrides one)
    ros2 launch rm_bringup ncp.launch.py profiles:=m8v2 weights:=''
"""

import pathlib
import subprocess

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

from rm_safety.command_sources import load_command_sources

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parent
_REPO_ROOT = _LAUNCH_DIR.parents[2]
_DEFAULT_LIMITS_PATH = str(_REPO_ROOT / "config" / "safety_limits.yaml")
_DEFAULT_TOPICS_PATH = str(_REPO_ROOT / "config" / "topics.yaml")
_DEFAULT_ROAM_PATH = str(_REPO_ROOT / "config" / "roam.yaml")

# The command-source table: every policy node's name, command topic and debug
# stream come from here, and rm_safety and the dashboard read the same file
# (both are handed _DEFAULT_TOPICS_PATH below). Loaded once per launch; a
# malformed table fails the launch here, before any node starts, with the
# loader's reason.
_SOURCES = load_command_sources(_DEFAULT_TOPICS_PATH)

# The row the UPSTREAM consumer (and the bench tools) publish on. No policy
# node of ours stands behind it, which is why it is not a profile.
_UPSTREAM_SOURCE = "policy"

# Policy profiles: one trained checkpoint and the contract it was exported
# against. The contract file is relative to config/, the checkpoint to
# <upstream_path>/weights/. Adding a policy means adding a row here AND a
# row with a node in config/topics.yaml command_sources, a contract file, a
# checkpoint in the upstream repo, and a line in scripts/deploy_upstream.sh's
# CRITICAL list.
_PROFILES = {
    "m8v2": ("interface.yaml", "actor.pt"),
    "m4": ("interface_m4.yaml", "actor_m4.pt"),
    "m9": ("interface_m9.yaml", "actor_m9.pt"),
    "dynabarn": ("interface_dynabarn.yaml", "actor_dynabarn.pt"),
}
# The profile the upstream consumer needs first in the list (the vendored
# module is fixed at its beam count), and the first of the default list.
_DEFAULT_PROFILE = "m8v2"
_DEFAULT_PROFILES = "m8v2,m4,m9,dynabarn"
# Sentinel default for `weights` / `interface_config_path`: resolve from the
# profile. Needed because '' already means "random weights".
_FROM_PROFILE = "profile"

# Sibling checkout convention, matching tests/unit/test_vendor_is_pristine.py.
_DEFAULT_UPSTREAM_PATH = str(_REPO_ROOT.parent / "NCP-on-Rosmaster")

# The trained checkpoints ship in the upstream repo, not this one — they are
# the training stack's artefacts and duplicating multi-MB binaries into two
# repos is how they drift apart. Defaulting to the profile's checkpoint
# (rather than to "", random weights) is deliberate: with the default the
# node FAILS TO START if the file is missing, which is far better than
# silently exercising an untrained network in a run the operator believes is
# testing a policy. Pass weights:='' for a plumbing run.

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
    # REQUIRED BY slam_toolbox, measured 2026-08-29. Without it the TG30's ray
    # count VARIES scan to scan — 9 distinct values (1967..1975) across 60
    # consecutive scans. slam_toolbox's karto backend latches the first scan's
    # count into its LaserRangeFinder and then REJECTS every scan that differs:
    #     LaserRangeScan contains 1970 range readings, expected 1951
    # printed once per scan, forever, with no map ever built and no map->odom
    # transform ever published. It does not crash and it does not warn that it
    # has stopped working, so it looks like SLAM merely failing to converge.
    #
    # With the flag: a constant 2020 rays, verified over 60 scans. NOTE this
    # is a genuinely live parameter on this driver, UNLIKE invalid_range_is_inf
    # which is inert here (see interface.yaml no_return_encoding) — the two
    # look equally plausible in the driver's parameter list, so measure rather
    # than assume when reaching for another one.
    #
    # Harmless to the policy path: scan_resampler handles any ray count, and
    # the padding rays arrive as the same 0.0 no-return sentinel it already
    # maps to range_max.
    "fixed_resolution": True,
}

# The SLAMTEC RPLIDAR A1 that a stock Yahboom X3 ships with (bernard-3,
# identified 2026-11-18 by its GET_INFO reply: model 0x18, firmware 1.29).
# sllidar_ros2's own default scan_mode is "Sensitivity", an A2/A3 mode the A1
# rejects (2026-08-12); "Standard" is the A1's. frame_id:=laser_link for the
# same TF reason as the TG30. No-returns arrive as +inf, which the sanitizer,
# the resampler and rm_safety already treat as "nothing there". The mount's
# orientation (inverted) is unverified on bernard-3: check that an object
# ahead of the robot appears dead ahead on the dashboard before trusting it.
_A1_LIDAR_PARAMS = {
    "channel_type": "serial",
    "serial_port": _LIDAR_PORT,
    "serial_baudrate": 115200,
    "frame_id": "laser_link",
    "inverted": False,
    "angle_compensate": True,
    "scan_mode": "Standard",
    # 5.5 Hz, the A1's design rate: Standard mode samples at 2 kHz, so 10 Hz
    # (the driver default; the motor settled at 7 Hz anyway) fills only ~200
    # of the 360 angle-compensated slots per revolution and the rest arrive
    # as no-returns (290 of 360 on the first scan, 2026-11-18). At 5.5 Hz
    # every slot is a real sample. rm_safety's 500 ms scan timeout and the
    # 5 Hz policy tick both sit inside the 182 ms period.
    "scan_frequency": 5.5,
}
_LIDARS = ("tg30", "a1")


def _refuse_unknown_lidar(context):
    """Fail fast on a lidar name this file has no driver block for."""
    lidar = LaunchConfiguration("lidar").perform(context)
    if lidar not in _LIDARS:
        raise RuntimeError(
            f"ncp.launch.py: lidar:={lidar!r} is not one of {_LIDARS}; the robot's "
            f"lidar must be named explicitly (bernard-1/-2 tg30, bernard-3 a1)."
        )
    return []


# The bridge's output. The upstream consumer reads this, never the raw /scan.
_NCP_SCAN_TOPIC = "/ncp/scan"


def _refuse_if_stack_already_running(context, *args, **kwargs):
    """Abort the launch if this stack is ALREADY up.

    A second `ros2 launch` on top of a running stack does not replace it, it
    DUPLICATES it — and `pkill -f ncp.launch.py` kills only the launch parent,
    leaving every node behind to be duplicated by the next attempt. That has
    happened three times on 2026-08-29 alone, and the second occurrence had
    two EKFs and two slam_toolbox instances writing the SAME TF edges with
    different estimates. The visible symptom was the navigation goal jumping
    around on the map while the robot sat still, which reads as a SLAM fault
    and cost a long diagnostic detour.

    Checking for safety_node specifically: two of those is two publishers on
    /cmd_vel, the one violation this whole workspace is built to prevent, so
    it is both the most dangerous duplicate and a reliable proxy for "a stack
    is already running".

    Deliberately a hard abort rather than a warning. A warning scrolls past in
    a launch that prints hundreds of lines, and the failure it prevents is
    silent and expensive to diagnose.
    """
    try:
        found = subprocess.run(
            ["pgrep", "-f", "[/]safety_node"],
            capture_output=True, text=True, timeout=5,
        ).stdout.split()
    except Exception:
        # If the check itself cannot run, do NOT block the launch — a broken
        # guard must not become the thing that stops the robot coming up.
        return []

    if not found:
        return []

    raise RuntimeError(
        f"REFUSING TO LAUNCH: safety_node is already running (PID(s) "
        f"{' '.join(found)}).\n"
        f"  Launching again would DUPLICATE the whole stack — two "
        f"safety_nodes means two publishers on /cmd_vel, and two EKFs and two "
        f"slam_toolboxes writing the same TF edges.\n"
        f"  `pkill -f ncp.launch.py` does NOT fix this: it kills only the "
        f"launch parent and orphans every node.\n"
        f"  Stop the running stack first:\n"
        f"      bash scripts/stack_down.sh && bash scripts/prep_robot.sh"
    )

def _resolved_pose_source(pose_source, with_slam):
    """The frame the stack drives in: an explicit `pose_source`, else follow SLAM.

    ONE place decides, and both policy_node and waypoint_node read it. A
    default sitting separately on each node could silently disagree with
    whether slam_toolbox is actually running — and the two failure directions
    are quite different. Asking for a map pose nothing publishes is a
    fail-closed stop, loud and safe. Reading odom while SLAM runs is a SILENT
    downgrade back onto the drifting pose SLAM was added to correct. Worse,
    two nodes resolving it differently would put the policy's goal and an
    operator's waypoint in different frames, separated by a correction that
    grows with every metre driven.
    """
    return PythonExpression([
        "'", pose_source, "' or ('map' if '", with_slam,
        "'.lower() == 'true' else 'odom')",
    ])


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

    first, _source, _interface, weights = _resolve_profiles(context)[0]
    if first != _DEFAULT_PROFILE:
        raise RuntimeError(
            f"consumer:=upstream needs profiles:= to start with {_DEFAULT_PROFILE}: "
            f"the scan bridge serves the first profile's beams and the vendored "
            f"upstream module is fixed at {_PROFILES[_DEFAULT_PROFILE][0]}'s count "
            f"(its node.py refuses any other scan length). Got {first!r} first."
        )
    upstream = context.perform_substitution(LaunchConfiguration("upstream_path"))
    mode = context.perform_substitution(LaunchConfiguration("mode"))
    obstacle_dist = context.perform_substitution(
        LaunchConfiguration("obstacle_dist")
    )

    cmd = [
        "python3", str(pathlib.Path(upstream) / "rosmaster" / "node.py"),
        "--ros-args",
        "-p", f"mode:={mode}",
        "-p", f"scan_topic:={_NCP_SCAN_TOPIC}",
        "-p", "odom_topic:=/odom",
        "-p", f"obstacle_dist:={obstacle_dist}",
        # OVERRIDE. Upstream defaults cmd_topic to /cmd_vel, which would
        # bypass rm_safety entirely at up to 2 m/s. It publishes on the
        # table's "policy" row, so command_source:=policy selects it.
        "-p", f"cmd_topic:={_SOURCES[_UPSTREAM_SOURCE].topic}",
    ]
    if weights:
        cmd += ["-p", f"weights:={weights}"]
    return [ExecuteProcess(cmd=cmd, name="ncp_rosmaster", output="screen")]


def _resolve_profiles(context):
    """[(profile, its command_sources row, contract path, weights path)] in
    the order `profiles:=` lists them, sentinels resolved. The first entry's
    contract is what the scan bridge serves.

    ONE place resolves the list, so the bridge, the policy nodes and the
    upstream consumer can never disagree about which checkpoints, contracts
    and topics are live.
    """
    raw = context.perform_substitution(LaunchConfiguration("profiles"))
    names = [p.strip() for p in raw.split(",") if p.strip()]
    if not names:
        raise RuntimeError(
            f"profiles:= must list at least one of {sorted(_PROFILES)}."
        )
    unknown = [p for p in names if p not in _PROFILES]
    if unknown:
        raise RuntimeError(
            f"profiles:={raw!r}: {unknown} not in {sorted(_PROFILES)}."
        )
    if len(set(names)) != len(names):
        raise RuntimeError(f"profiles:={raw!r} lists a profile twice.")

    interface_override = context.perform_substitution(
        LaunchConfiguration("interface_config_path")
    )
    weights_override = context.perform_substitution(LaunchConfiguration("weights"))
    overridden = [
        arg for arg, value in (
            ("interface_config_path", interface_override),
            ("weights", weights_override),
        ) if value != _FROM_PROFILE
    ]
    if overridden and len(names) > 1:
        raise RuntimeError(
            f"{' and '.join(overridden)} override ONE profile; pass "
            f"profiles:=<one of {names}> with it."
        )
    upstream = context.perform_substitution(LaunchConfiguration("upstream_path"))

    resolved = []
    for name in names:
        source = _SOURCES.get(name)
        if source is None or source.node is None:
            raise RuntimeError(
                f"profile {name!r} has no command_sources row with a node in "
                f"config/topics.yaml. Every profile needs one: that row is "
                f"its node name, command topic and debug stream."
            )
        interface_file, weights_file = _PROFILES[name]
        interface = (
            interface_override if interface_override != _FROM_PROFILE
            else str(_REPO_ROOT / "config" / interface_file)
        )
        weights = (
            weights_override if weights_override != _FROM_PROFILE
            else str(pathlib.Path(upstream) / "weights" / weights_file)
        )
        resolved.append((name, source, interface, weights))
    return resolved


def _policy_consumer(context):
    """The scan bridge, on the FIRST profile's contract, and (for
    consumer:=policy_node) one policy_node per profile, each named and
    topic'd from its config/topics.yaml row. Built here rather than as
    static Node actions because everything depends on `profiles`."""
    profiles = _resolve_profiles(context)
    _first, _source, first_interface, _weights = profiles[0]
    # A command_source naming a policy row whose node this launch does not
    # start would come up forwarding nothing: zero forever with reason
    # command_timeout. Fail-closed, but a confusing bring-up; refuse it here.
    selected = context.perform_substitution(LaunchConfiguration("command_source"))
    started = {name for name, _s, _i, _w in profiles}
    row = _SOURCES.get(selected)
    if row is not None and row.node is not None and selected not in started:
        raise RuntimeError(
            f"command_source:={selected} names {row.node}, but profiles:= "
            f"({', '.join(started)}) does not start it. Add it to profiles:= "
            f"or select a source that will be publishing."
        )
    nodes = [
        Node(
            package="rm_policy",
            executable="scan_bridge_node",
            name="ncp_scan_bridge",
            output="screen",
            parameters=[{
                "scan_topic": "/scan",
                "output_topic": _NCP_SCAN_TOPIC,
                "interface_config_path": first_interface,
            }],
        ),
    ]
    if context.perform_substitution(LaunchConfiguration("consumer")) != "policy_node":
        return nodes
    for _name, source, interface, weights in profiles:
        params = {
            # RAW /scan, deliberately NOT the bridge output: policy_node
            # resamples internally, and re-resampling an already-reduced
            # scan (bin centres, slightly narrower than the arc) into the
            # full arc faults every tick.
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            # Its own row of the table: rm_safety subscribes to exactly
            # this topic under exactly this name.
            "cmd_vel_topic": source.topic,
            "debug_topic": source.debug,
            "pose_source": _resolved_pose_source(
                LaunchConfiguration("pose_source"), LaunchConfiguration("with_slam")
            ),
            "interface_config_path": interface,
            "safety_limits_path": _DEFAULT_LIMITS_PATH,
            # Read at startup whether or not roam is on, so a bad file fails
            # the launch. `roam` itself is a runtime parameter, not a launch
            # argument (see the docstring).
            "roam_config_path": _DEFAULT_ROAM_PATH,
            # picks this profile's block under roam.yaml `profiles:`, if any
            "profile": _name,
        }
        # An empty `weights` must be OMITTED, not passed: rclpy aborts on
        # `-p weights:=` (see _upstream_process). Empty is the default
        # inside policy_node and means random weights.
        if weights:
            params["weights"] = weights
        nodes.append(Node(
            package="rm_policy",
            executable="policy_node",
            name=source.node,
            output="screen",
            parameters=[params],
        ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    with_safety = LaunchConfiguration("with_safety")
    command_source = LaunchConfiguration("command_source")
    with_lidar = LaunchConfiguration("with_lidar")
    lidar = LaunchConfiguration("lidar")
    with_base = LaunchConfiguration("with_base")
    with_odom = LaunchConfiguration("with_odom")
    control_mode = LaunchConfiguration("control_mode")
    dry_run = LaunchConfiguration("dry_run")
    calibration_path = LaunchConfiguration("calibration_path")
    with_slam = LaunchConfiguration("with_slam")
    pose_source = LaunchConfiguration("pose_source")
    with_dashboard = LaunchConfiguration("with_dashboard")
    dashboard_port = LaunchConfiguration("dashboard_port")
    dashboard_controls = LaunchConfiguration("dashboard_controls")
    with_waypoint = LaunchConfiguration("with_waypoint")

    return LaunchDescription([
        # FIRST action: refuse to stack a second copy on a running one.
        OpaqueFunction(function=_refuse_if_stack_already_running),
        OpaqueFunction(function=_refuse_unknown_lidar),

        DeclareLaunchArgument(
            "consumer", default_value="policy_node",
            description="'policy_node' (rm_policy, contract-checked, DEFAULT) "
                        "or 'upstream' (NCP-on-Rosmaster node.py, has the "
                        "reactive debug mode).",
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
            "profiles", default_value=_DEFAULT_PROFILES,
            description="Comma-separated trained policies to run, each on "
                        "its own command topic: " + ", ".join(
                f"'{k}' ({v[0]}, {v[1]})" for k, v in _PROFILES.items()
            ) + ". rm_safety forwards the one named by command_source; the "
                "scan bridge serves the first listed.",
        ),
        DeclareLaunchArgument(
            "interface_config_path", default_value=_FROM_PROFILE,
            description="Contract file override for ONE profile (pass "
                        "profiles:=<one>); default from the table.",
        ),
        DeclareLaunchArgument(
            "weights", default_value=_FROM_PROFILE,
            description="Trained actor state_dict for ONE profile (pass "
                        "profiles:=<one>). Default: the profile's checkpoint "
                        "in the upstream checkout. Pass weights:='' for "
                        "RANDOM WEIGHTS (plumbing runs).",
        ),
        DeclareLaunchArgument(
            "obstacle_dist", default_value="0.9",
            description="Debug rule: spin away from anything closer than this "
                        "(metres). Upstream's own default is 1.5, which in a "
                        "normal room keeps something inside the 270 deg arc "
                        "almost always, so the robot spins continuously and "
                        "never cruises. 0.9 sits above safety's 0.50 m slow "
                        "zone, so the policy still turns away before the "
                        "safety layer has to intervene.",
        ),
        DeclareLaunchArgument(
            "with_safety", default_value="true",
            description="Also launch safety.launch.py.",
        ),
        # Passed straight through to safety.launch.py. Defaults to "teleop",
        # matching that file: with "teleop", rm_safety IGNORES every policy
        # topic, so the whole NCP chain runs and publishes but CANNOT reach
        # the base driver. That is the correct default for bringing the stack
        # up and watching it. Name a policy row only when you intend that
        # policy to drive, with an operator and a kill switch present.
        DeclareLaunchArgument(
            "command_source", default_value="teleop",
            description="rm_safety's active source, a config/topics.yaml "
                        "command_sources row: " + ", ".join(
                f"'{n}'" for n in _SOURCES
            ) + ". 'teleop' (default) means no policy can drive.",
        ),
        # LIDAR ZONE MASTER SWITCHES — passed through to safety.launch.py.
        #
        # BOTH DEFAULT "false" IN THIS FILE, by operator decision 2026-08-31,
        # so a full-stack policy run has NO PROXIMITY PROTECTION unless it is
        # asked for. The reason is that both zones fired on the policy rather
        # than on a hazard: a measured run latched stop_zone for 363 of 450
        # samples and drove for 61, so what was being evaluated was the safety
        # layer, not the network.
        #
        # THIS INVERTS a1b7652's "default ON from every direction" property,
        # and does so DELIBERATELY AND ONLY HERE. config/safety_limits.yaml
        # still says true, so bench_up.sh, safety.launch.py on its own,
        # base-only work and velocity_accuracy.py all keep the full e-stop —
        # this file is the single place that opts out, because this file is
        # the one that means "an operator is running the policy right now".
        # AND composition still holds: with the config at true, these two
        # arguments are what decides, and setting either to "true" here
        # restores that zone.
        #
        # WHAT REMAINS when both are false: every velocity clamp, the
        # acceleration limiter, the policy vy check, all three staleness
        # watchdogs, killed-on-startup, and /safety/kill — which is now the
        # ONLY thing that stops the robot before an obstacle. The clamps bound
        # SPEED, not PROXIMITY: nothing prevents driving into a wall at
        # max_vx. min_range is still measured and published, and the reason
        # string reads ok_estop_disabled continuously, so /safety/status, the
        # dashboard and every bag carry the fact.
        #
        # NEVER run unattended in this configuration. There is no physical
        # E-stop on this robot.
        DeclareLaunchArgument(
            "slow_zone_enabled", default_value="false",
            description="Proximity velocity SCALING. DEFAULT FALSE here: its "
                        "braking prevented evaluating what the policy itself "
                        "does. Clamps are unaffected. 'true' restores it.",
        ),
        DeclareLaunchArgument(
            "stop_zone_enabled", default_value="false",
            description="The lidar E-STOP and latch. DEFAULT FALSE here, by "
                        "operator decision 2026-08-31: NO obstacle protection "
                        "remains and /safety/kill becomes the only stop. "
                        "Operator-supervised use ONLY, never unattended. "
                        "'true' restores it.",
        ),
        DeclareLaunchArgument(
            "with_lidar", default_value="true",
            description="Start the lidar driver on its by-id CP2102 port.",
        ),
        DeclareLaunchArgument(
            "lidar", default_value="tg30",
            description="Which lidar this robot carries: tg30 (YDLIDAR TG30, "
                        "bernard-1/-2) or a1 (SLAMTEC RPLIDAR A1, bernard-3).",
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
        DeclareLaunchArgument(
            "with_slam", default_value="true",
            description="Start slam_toolbox, which publishes map -> odom and "
                        "corrects the odometry drift. Default TRUE since "
                        "2026-08-31: /odom's yaw drifts ~0.6 deg/s with the "
                        "robot standing still, so pose_source:=odom rotates "
                        "goal_local around the robot as a run proceeds. This "
                        "ALSO switches pose_source to 'map' unless overridden.",
        ),
        DeclareLaunchArgument(
            "pose_source", default_value="",
            description="policy_node's pose: 'odom' (dead reckoning) or 'map' "
                        "(SLAM-corrected, via TF). Empty (the default) means "
                        "'follow with_slam' — 'map' when SLAM is up, 'odom' "
                        "when it is not, so the default stack drives on 'map'. "
                        "Set it explicitly to 'odom' to run SLAM while still "
                        "driving on odom, which is the A/B and the safe way to "
                        "watch a map build before trusting it.",
        ),
        DeclareLaunchArgument(
            "with_dashboard", default_value="true",
            description="Start rm_dashboard — the live web view of the whole "
                        "stack at http://<robot>:8080/. Subscribe-only on "
                        "topics; it can never publish /cmd_vel.",
        ),
        DeclareLaunchArgument(
            "dashboard_port", default_value="8080",
            description="Port the dashboard serves on.",
        ),
        DeclareLaunchArgument(
            "dashboard_controls", default_value="true",
            description="false serves a read-only dashboard (no kill, arm, "
                        "source, goal or waypoint from the browser).",
        ),
        DeclareLaunchArgument(
            "with_waypoint", default_value="false",
            description="Start rm_waypoint, the go-to-waypoint controller for "
                        "repositioning between runs. DEFAULT OFF: it is the "
                        "only autonomous motion path here that is not the "
                        "policy, and on a robot with no physical E-stop that "
                        "should be asked for. It publishes /teleop/cmd_vel, so "
                        "it still drives behind rm_safety's clamps and kill, "
                        "and it reaches the wheels only when command_source is "
                        "'teleop' AND the robot has been armed. Its pose_source "
                        "follows this file's, so waypoints are in the same "
                        "frame the policy is using.",
        ),

        # ---- hardware layer, bottom up -------------------------------------
        # Lidar first: the scan bridge and both consumers are useless without
        # /scan, and rm_safety treats missing perception as a stop condition.
        Node(
            condition=IfCondition(PythonExpression(
                ["'", with_lidar, "' == 'true' and '", lidar, "' == 'tg30'"])),
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            parameters=[_LIDAR_PARAMS],
        ),
        Node(
            condition=IfCondition(PythonExpression(
                ["'", with_lidar, "' == 'true' and '", lidar, "' == 'a1'"])),
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[_A1_LIDAR_PARAMS],
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
            launch_arguments={
                "command_source": command_source,
                "topics_path": _DEFAULT_TOPICS_PATH,
                "slow_zone_enabled": LaunchConfiguration("slow_zone_enabled"),
                "stop_zone_enabled": LaunchConfiguration("stop_zone_enabled"),
            }.items(),
        ),

        # The scan bridge and one policy_node per profile, from the table.
        OpaqueFunction(function=_policy_consumer),
        OpaqueFunction(function=_upstream_process),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_LAUNCH_DIR / "slam.launch.py")),
            condition=IfCondition(with_slam),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_LAUNCH_DIR / "waypoint.launch.py")),
            condition=IfCondition(with_waypoint),
            launch_arguments={
                # The SAME resolution policy_node uses below, from the same
                # helper. A waypoint is therefore always expressed in the frame
                # the policy is driving in. Two nodes disagreeing about what
                # frame "1.5, 0.0" names would be a silent error, and a mobile
                # one — the offset between odom and map is the SLAM correction,
                # which grows as you drive.
                "pose_source": _resolved_pose_source(pose_source, with_slam),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(_LAUNCH_DIR / "dashboard.launch.py")),
            condition=IfCondition(with_dashboard),
            launch_arguments={
                "port": dashboard_port,
                "enable_controls": dashboard_controls,
                "topics_path": _DEFAULT_TOPICS_PATH,
            }.items(),
        ),

    ])
