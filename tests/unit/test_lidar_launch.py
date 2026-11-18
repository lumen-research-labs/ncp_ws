"""ncp.launch.py configures itself from the robot profile.

The lidar driver (two units carry a YDLIDAR TG30, ydlidar_ros2_driver at
512000 baud; bernard-3 the SLAMTEC RPLIDAR A1 a stock Yahboom X3 ships with,
sllidar_ros2 at 115200; both adapters are a CP2102 on the same by-id path, so
the wrong driver opens the port and reads nothing, 2026-11-18), the lidar's
yaw, the base calibration and the ROS domain all come from
config/robots/<name>.yaml; none is a launch argument any more. Source pins,
as the other launch tests: the launch system is not importable here.
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


def test_the_driver_table_matches_the_models_a_profile_may_name() -> None:
    src = _src()
    assert '"lidar", default_value' not in src and "lidar_yaw_deg" not in src, "these are the profile's now"
    assert 'if set(_DRIVERS) != set(LIDAR_MODELS):' in src
    assert '"tg30": ("ydlidar_ros2_driver", "ydlidar_ros2_driver_node", _LIDAR_PARAMS)' in src
    assert '"a1": ("sllidar_ros2", "sllidar_node", _A1_LIDAR_PARAMS)' in src
    assert "OpaqueFunction(function=_lidar_driver)" in src
    assert "_refuse_unknown_lidar" not in src, "the profile loader refuses an unknown model"


def test_both_drivers_share_the_by_id_port_and_the_tf_frame() -> None:
    src = _src()
    tg30, a1 = _block(src, "_LIDAR_PARAMS"), _block(src, "_A1_LIDAR_PARAMS")
    assert '"port": _LIDAR_PORT' in tg30 and '"serial_port": _LIDAR_PORT' in a1
    assert '"frame_id": "laser_link"' in tg30 and '"frame_id": "laser_link"' in a1
    assert '"baudrate": 512000' in tg30
    assert '"serial_baudrate": 115200' in a1
    assert '"scan_mode": "Standard"' in a1, "the driver default Sensitivity is an A2/A3 mode the A1 rejects"
    assert '"scan_frequency": 5.5' in a1, "2 kHz over 360 slots needs the 5.5 Hz design rate"


def test_the_driver_and_the_relay_are_chosen_from_the_profile() -> None:
    src = _src()
    body = src[src.index("def _lidar_driver(context):"):src.index("def _base_driver(context):")]
    assert 'LaunchConfiguration("with_lidar").perform(context).lower() != "true"' in body
    assert "lidar = _robot_profile(context).lidar" in body
    assert "package, executable, params = _DRIVERS[lidar.model]" in body
    assert "rotated = lidar.yaw_deg != 0.0" in body
    assert 'remappings=[("scan", "/scan_driver" if rotated else "/scan")]' in body
    assert 'executable="scan_rotate_node"' in body
    assert '"input_topic": "/scan_driver"' in body and '"output_topic": "/scan"' in body
    assert '"yaw_deg": float(lidar.yaw_deg)' in body


def test_stack_down_stops_both_lidar_drivers() -> None:
    """A driver the teardown does not name survives it and keeps the serial
    port, so the next launch's driver cannot open it (2026-11-18, sllidar)."""
    src = open(os.path.join(_REPO_ROOT, "scripts", "stack_down.sh"), encoding="utf-8").read()
    order = src[src.index("ORDER=("):src.index(")", src.index("ORDER=("))]
    assert "ydlidar_ros2_driver_node" in order and "sllidar_node" in order


def test_the_relay_is_installed_and_torn_down() -> None:
    setup = open(os.path.join(_REPO_ROOT, "src", "rm_policy", "setup.py"), encoding="utf-8").read()
    assert "scan_rotate_node = rm_policy.scan_rotate_node:main" in setup
    down = open(os.path.join(_REPO_ROOT, "scripts", "stack_down.sh"), encoding="utf-8").read()
    assert "scan_rotate_node" in down[down.index("OUR_NODES=("):down.index(")", down.index("OUR_NODES=("))]


def test_the_base_runs_on_the_profiles_calibration_unless_overridden() -> None:
    src = _src()
    assert '"calibration_path", default_value=""' in src
    assert "_DEFAULT_CALIBRATION_PATH" not in src
    body = src[src.index("def _base_driver(context):"):src.index("# The bridge's output.")]
    assert "path = _robot_profile(context).calibration_path" in body
    assert '"calibration_path": path,' in body
    assert "OpaqueFunction(function=_base_driver)" in src


def test_every_child_runs_on_the_profiles_domain_and_a_disagreeing_shell_is_refused() -> None:
    src = _src()
    body = src[src.index("def _refuse_without_robot_profile(context):"):src.index("# The row the UPSTREAM consumer")]
    assert 'shell = os.environ.get("ROS_DOMAIN_ID", "")' in body
    assert "if shell and shell != str(profile.ros_domain_id):" in body
    assert 'return [SetEnvironmentVariable("ROS_DOMAIN_ID", str(profile.ros_domain_id))]' in body
    actions = src[src.index("return LaunchDescription(["):]
    assert actions.index("_refuse_without_robot_profile") < actions.index("_lidar_driver"), "the domain is set before any node"
    for name in ("bench_up.sh", "bench_down.sh", "drive_test.sh", "run_stack.sh", "stack_down.sh"):
        sh = open(os.path.join(_REPO_ROOT, "scripts", name), encoding="utf-8").read()
        assert "export ROS_DOMAIN_ID=28" not in sh, name
        assert 'source "$(dirname "$0")/robot_env.sh"' in sh, name


def test_the_dashboard_names_its_robot() -> None:
    src = _src()
    assert '"robot_profile_path": robot_profile,' in src[src.index('"dashboard.launch.py"'):]
    dash = open(os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "dashboard.launch.py"), encoding="utf-8").read()
    assert 'DeclareLaunchArgument("robot_profile_path", default_value=_default_robot_profile())' in dash
    assert '"robot_profile_path": LaunchConfiguration("robot_profile_path")' in dash
    node = open(os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "dashboard_node.py"), encoding="utf-8").read()
    assert 'self.declare_parameter("robot_profile_path", "")' in node
    assert "self._robot = load_robot_profile(profile_path)" in node
    assert "if domain and domain != str(self._robot.ros_domain_id):" in node
    snap = node[node.index("def _snapshot("):node.index("def _dispatch(")]
    assert '"robot": {' in snap and '"name": self._robot.name,' in snap and '"domain": self._robot.ros_domain_id,' in snap
    html = open(os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html"), encoding="utf-8").read()
    assert "<title>NCP stack</title>" in html and "NCP Stack — bernard" not in html, "no unit's name in the static title"
    brand = html[html.index('<div class="brand">'):html.index("</div>", html.index('<div class="brand">'))]
    assert "bernard" not in brand.lower(), "the header names the robot from the snapshot, not from the source"
    assert '<span id="robot">' in html and 'id="robotmeta"' in html
    hdr = html[html.index("function renderHeader(snap)"):]
    hdr = hdr[:hdr.index("\n}\n")]
    assert "snap.robot" in hdr and "document.title = `${robot.name}" in hdr


def test_the_launch_resolves_the_robot_profile_and_hands_it_to_every_scan_consumer() -> None:
    src = _src()
    assert '"robot_profile", default_value=_default_robot_profile()' in src
    assert "OpaqueFunction(function=_refuse_without_robot_profile)" in src
    assert src.count('"robot_profile_path": context.perform_substitution(LaunchConfiguration("robot_profile"))') == 2, "the bridge and every policy node"
    assert '"robot_profile_path": robot_profile,' in src, "safety.launch gets it"
    assert 'launch_arguments={"robot_profile": robot_profile}.items()' in src, "slam.launch gets it"
    safety = open(os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "safety.launch.py"), encoding="utf-8").read()
    assert '"robot_profile_path": robot_profile_path,' in safety
    slam = open(os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "slam.launch.py"), encoding="utf-8").read()
    assert "load_robot_profile(path).lidar.body_radius_m" in slam and "safety_limits" not in slam
