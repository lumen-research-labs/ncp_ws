"""
teleop_joy.launch.py — the Yahboom gamepad, remapped onto /teleop/cmd_vel.

What it does:
    Runs `yahboomcar_ctrl/yahboom_joy_X3` (ROS node name `joy_ctrl`) with
    `/cmd_vel` remapped to `/teleop/cmd_vel`, plus the raw `joy` driver it
    reads from.

WHY THIS EXISTS — a /cmd_vel ownership violation in the vendor bringup:
    `yahboomcar_bringup_X3_launch.py` hardcodes `yahboom_joy_node` into its
    LaunchDescription with no argument to disable it, and that node publishes
    `/cmd_vel` DIRECTLY. So the standard bringup permanently occupies the one
    topic CLAUDE.md reserves for rm_safety as sole publisher -- the same
    violation the build plan forbids for `cpt_b2_engine`, except it arrives
    with every routine bringup rather than from a node we choose to start.

    Measured on bernard-1 (2026-08-27): idle, `joy_ctrl` ADVERTISES /cmd_vel
    but publishes no messages, so the conflict is latent rather than active.
    That is not a defence -- the moment the operator touches the gamepad while
    rm_safety is running, two publishers race on the base driver's input, and
    the safety layer's clamps do not apply to the other one.

    Remapping to /teleop/cmd_vel does not merely avoid the clash; it is where
    the architecture already wanted the gamepad. `/teleop/cmd_vel` is a
    declared INPUT to rm_safety, so the remap puts manual driving BEHIND the
    same clamps and e-stop arcs as the policy, and makes the gamepad a genuine
    manual override. That matters more than usual here: there is no physical
    E-stop on this robot.

USAGE — the vendor's own joy node must be stopped first:
    The vendor launch gives no way to omit it, so the sequence is:

        ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py   # as usual
        pgrep -af '[y]ahboom_joy_X3'                                     # find it
        kill <PID>                                                       # that PID only
        ros2 launch rm_bringup teleop_joy.launch.py

    Never `pkill -f yahboom_joy_X3` from an SSH one-liner that also contains
    the string -- the pattern matches your own shell (robot-ssh skill, §6).

    Verify afterwards; this is the check that matters:
        ros2 topic info /cmd_vel          -> Publisher count: 0 (before rm_safety)
        ros2 topic info /teleop/cmd_vel   -> Publisher count: 1

Failure mode:
    Purely additive -- this launch file starts nodes and remaps a topic; it
    cannot itself move the robot. If the vendor's `joy_ctrl` was NOT stopped
    first, BOTH will be running: one on /cmd_vel and one on /teleop/cmd_vel,
    which is worse than either alone. Always run the verify step above.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# rm_safety's teleop INPUT. Never /cmd_vel -- rm_safety is its sole publisher.
_TELEOP_CMD_TOPIC = "/teleop/cmd_vel"


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # The raw joystick driver. Publishes /joy; owns no velocity topic, so
        # it needs no remap. Harmless if the vendor bringup's copy is also
        # running (two publishers on /joy is not a control-path conflict), but
        # stopping that one too keeps `ros2 node list` honest.
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        # The Yahboom gamepad->velocity mapper, node name `joy_ctrl`.
        # THE REMAP IS THE ENTIRE POINT OF THIS FILE.
        Node(
            package="yahboomcar_ctrl",
            executable="yahboom_joy_X3",
            name="joy_ctrl",
            output="screen",
            remappings=[("/cmd_vel", _TELEOP_CMD_TOPIC)],
        ),
    ])
