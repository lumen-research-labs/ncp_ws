"""
base.launch.py — the mecanum base driver (rm_base_node).

What it does:
    Starts rm_base_node, which OWNS the base-board serial link and REPLACES
    the vendor `Mcnamu_driver_X3`. It drives the wheels with raw PWM (never
    the firmware speed PID) and republishes /imu/data_raw, /imu/mag, /voltage
    and a sign-corrected /vel_raw.

    Only one process may hold the base-board serial port. Before launching
    this, STOP the vendor driver:

        pgrep -af '[M]cnamu_driver_X3'      # find it
        kill <PID>                          # bringup does not respawn it

    Keep the rest of yahboomcar_bringup (EKF, imu_filter_madgwick,
    robot_state_publisher, the odom integrator) running — rm_base_node feeds
    them the same topics the vendor driver did.

Launch arguments:
    control_mode     "open_loop" (default) or "closed_loop". open_loop is the
                     validated path: PWM straight from base_calibration.yaml.
                     closed_loop adds a per-wheel PI trim on the
                     sign-corrected encoders — it is what makes the four
                     wheels hold the SAME speed under uneven load, but it must
                     be proven wheels-up (watch /base/debug sign_fault and
                     stalled) before it is trusted on the floor.
    dry_run          "true" (default) — compute and publish everything, send
                     only zero PWM. Motion requires BOTH dry_run:=false AND
                     an armed rm_safety. Two explicit acts; there is no
                     physical E-stop.
    calibration_path  path to config/base_calibration.yaml (default: in-repo).
    serial_port      by-id CH340 path (default: bernard-1's).

Failure mode:
    rm_base_node fails closed — no /cmd_vel within its watchdog timeout, any
    exception, or dry_run true all result in zero PWM. On SIGINT it floods
    stop before releasing the serial port.
"""

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parent
_REPO_ROOT = _LAUNCH_DIR.parents[2]
_DEFAULT_CALIBRATION_PATH = str(_REPO_ROOT / "config" / "base_calibration.yaml")
_DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def generate_launch_description() -> LaunchDescription:
    control_mode = LaunchConfiguration("control_mode")
    dry_run = LaunchConfiguration("dry_run")
    calibration_path = LaunchConfiguration("calibration_path")
    serial_port = LaunchConfiguration("serial_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_mode", default_value="open_loop",
            description="open_loop (default, validated) or closed_loop "
                        "(per-wheel PI on the sign-corrected encoders).",
        ),
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="true (default) sends only zero PWM. Motion needs "
                        "dry_run:=false AND an armed rm_safety.",
        ),
        DeclareLaunchArgument(
            "calibration_path", default_value=_DEFAULT_CALIBRATION_PATH,
            description="Path to config/base_calibration.yaml.",
        ),
        DeclareLaunchArgument(
            "serial_port", default_value=_DEFAULT_SERIAL_PORT,
            description="by-id CH340 base-board serial path.",
        ),
        Node(
            package="rm_base",
            executable="rm_base_node",
            name="rm_base_node",
            output="screen",
            parameters=[{
                "control_mode": control_mode,
                "dry_run": dry_run,
                "calibration_path": calibration_path,
                "serial_port": serial_port,
            }],
        ),
    ])
