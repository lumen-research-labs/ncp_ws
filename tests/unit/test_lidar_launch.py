"""ncp.launch.py starts the lidar driver that matches the robot's lidar.

Two units carry a YDLIDAR TG30 (ydlidar_ros2_driver, 512000 baud); bernard-3
carries the SLAMTEC RPLIDAR A1 a stock Yahboom X3 ships with (sllidar_ros2,
115200 baud). Both adapters are a CP2102 on the same by-id path, so the wrong
driver opens the port and then reads nothing (2026-11-18). Source pins, as
the other launch tests: the launch system is not importable here.
"""

from __future__ import annotations

import os
import re

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "ncp.launch.py")


def _src() -> str:
    with open(_LAUNCH, "r", encoding="utf-8") as fh:
        return fh.read()


def _block(src: str, name: str) -> str:
    m = re.search(name + r" = \{.*?\n\}", src, re.S)
    assert m, f"{name} not found"
    return m.group(0)


def test_the_lidar_argument_defaults_to_the_tg30_and_names_both() -> None:
    src = _src()
    assert '_LIDARS = ("tg30", "a1")' in src
    assert '"lidar", default_value="tg30"' in src
    assert "OpaqueFunction(function=_refuse_unknown_lidar)" in src


def test_both_drivers_share_the_by_id_port_and_the_tf_frame() -> None:
    src = _src()
    tg30, a1 = _block(src, "_LIDAR_PARAMS"), _block(src, "_A1_LIDAR_PARAMS")
    assert '"port": _LIDAR_PORT' in tg30 and '"serial_port": _LIDAR_PORT' in a1
    assert '"frame_id": "laser_link"' in tg30 and '"frame_id": "laser_link"' in a1
    assert '"baudrate": 512000' in tg30
    assert '"serial_baudrate": 115200' in a1
    assert '"scan_mode": "Standard"' in a1, "the driver default Sensitivity is an A2/A3 mode the A1 rejects"
    assert '"scan_frequency": 5.5' in a1, "2 kHz over 360 slots needs the 5.5 Hz design rate"


def test_each_driver_node_is_gated_on_with_lidar_and_the_lidar_name() -> None:
    src = _src()
    assert "' == 'true' and '\", lidar, \"' == 'tg30'" in src
    assert "' == 'true' and '\", lidar, \"' == 'a1'" in src
    assert 'executable="ydlidar_ros2_driver_node"' in src
    assert 'executable="sllidar_node"' in src and 'package="sllidar_ros2"' in src


def test_stack_down_stops_both_lidar_drivers() -> None:
    """A driver the teardown does not name survives it and keeps the serial
    port, so the next launch's driver cannot open it (2026-11-18, sllidar)."""
    src = open(os.path.join(_REPO_ROOT, "scripts", "stack_down.sh"), encoding="utf-8").read()
    order = src[src.index("ORDER=("):src.index(")", src.index("ORDER=("))]
    assert "ydlidar_ros2_driver_node" in order and "sllidar_node" in order


def test_a_yaw_offset_puts_the_rotate_relay_between_the_driver_and_scan() -> None:
    src = _src()
    assert '"lidar_yaw_deg", default_value="0"' in src
    assert "'/scan_driver' if float('\", lidar_yaw_deg, \"') != 0.0 else '/scan'" in src
    assert src.count('remappings=[("scan", driver_scan_topic)]') == 2, "both drivers get the remap"
    assert 'executable="scan_rotate_node"' in src
    assert '"input_topic": "/scan_driver"' in src and '"output_topic": "/scan"' in src
    assert "' == 'true' and \", rotated]" in src, "the relay runs only with the lidar and a non-zero yaw"
    setup = open(os.path.join(_REPO_ROOT, "src", "rm_policy", "setup.py"), encoding="utf-8").read()
    assert "scan_rotate_node = rm_policy.scan_rotate_node:main" in setup
    down = open(os.path.join(_REPO_ROOT, "scripts", "stack_down.sh"), encoding="utf-8").read()
    assert "scan_rotate_node" in down[down.index("OUR_NODES=("):down.index(")", down.index("OUR_NODES=("))]
