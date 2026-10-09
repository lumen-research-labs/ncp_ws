"""RESET MAP: the dashboard's in-place SLAM restart.

slam_toolbox 2.6.9 has no reset service, so a fresh map is a restart of its
process, which slam.launch.py marks respawnable. The ROS-free half (finding
that one process, deciding whether a reset is allowed) is tested directly;
the rest is pinned as source text the way the other dashboard tests do,
because importing dashboard_node needs rclpy and importing the launch file
needs launch_ros.

Pure stdlib: workstation tier.
"""

from __future__ import annotations

import os
import re
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_dashboard"))

from rm_dashboard.slam_restart import (  # noqa: E402
    SLAM_EXECUTABLE, find_slam_pids, reset_refusal,
)

_INDEX = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html")
_NODE = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "dashboard_node.py")
_SLAM_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "slam.launch.py")


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _fake_proc(tmp_path, entries):
    """A /proc look-alike: {name: argv list} -> directories with cmdline files."""
    for name, argv in entries.items():
        d = tmp_path / name
        d.mkdir()
        (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")
    return str(tmp_path)


# --- finding the process -----------------------------------------------------

def test_finds_exactly_the_slam_executable_by_argv0_basename(tmp_path) -> None:
    root = _fake_proc(tmp_path, {
        "100": ["/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node", "--ros-args"],
        "101": ["python3", "/home/jetson/ncp_ws/install/rm_dashboard/lib/rm_dashboard/dashboard_node"],
        "102": ["pkill", "-f", "async_slam_toolbox_node"],          # mentions it as an ARGUMENT
        "103": ["bash", "-c", "ros2 run slam_toolbox async_slam_toolbox_node"],
        "104": ["/opt/ros/humble/lib/slam_toolbox/sync_slam_toolbox_node"],  # a different node
        "self": ["irrelevant"],
        "cpuinfo": ["irrelevant"],
    })
    assert find_slam_pids(root) == [100]


def test_two_processes_are_both_reported_ascending(tmp_path) -> None:
    root = _fake_proc(tmp_path, {
        "900": ["async_slam_toolbox_node"],
        "250": ["/x/y/async_slam_toolbox_node"],
    })
    assert find_slam_pids(root) == [250, 900]


def test_unreadable_or_vanished_entries_are_skipped(tmp_path) -> None:
    root = _fake_proc(tmp_path, {"300": ["async_slam_toolbox_node"]})
    (tmp_path / "301").mkdir()                # no cmdline: the process exited mid-scan
    assert find_slam_pids(root) == [300]
    assert find_slam_pids(str(tmp_path / "does-not-exist")) == []


def test_default_executable_is_the_async_node() -> None:
    assert SLAM_EXECUTABLE == "async_slam_toolbox_node"


# --- refusals --------------------------------------------------------------------

def test_allowed_only_in_map_mode_with_one_process() -> None:
    assert reset_refusal("map", [4242]) is None


@pytest.mark.parametrize("pose_source", ["odom", None, ""])
def test_refused_when_the_policy_is_not_on_the_map_pose(pose_source) -> None:
    reason = reset_refusal(pose_source, [4242])
    assert reason is not None and "pose_source" in reason


def test_refused_when_slam_is_not_running() -> None:
    reason = reset_refusal("map", [])
    assert reason is not None and "no async_slam_toolbox_node" in reason


def test_refused_on_a_duplicate_stack_and_names_the_fix() -> None:
    reason = reset_refusal("map", [10, 11])
    assert reason is not None
    assert "duplicate stack" in reason and "stack_down.sh" in reason


# --- the wiring, pinned as text ------------------------------------------------

def test_launch_file_makes_slam_toolbox_respawn() -> None:
    src = _read(_SLAM_LAUNCH)
    block = src[src.index('executable="async_slam_toolbox_node"'):]
    block = block[:block.index("parameters=")]
    assert "respawn=True" in block, "RESET MAP relies on the launch respawning slam_toolbox"


def test_dashboard_registers_the_command_and_kills_first() -> None:
    src = _read(_NODE)
    controls = src[src.index("_CONTROLS = ("):]
    controls = controls[:controls.index(")")]
    assert '"reset_map"' in controls
    body = src[src.index("def _reset_map("):src.index("def _send_jog(")]
    # The kill comes before the signal, and the signal is SIGTERM to one pid.
    assert body.index('"/safety/kill"') < body.index("signal.SIGTERM")
    assert "reset_refusal(" in body
    assert "self._slam.reset()" in body


def test_page_has_the_button_and_asks_first() -> None:
    html = _read(_INDEX)
    assert 'id="resetmap"' in html
    handler = html[html.index('$("resetmap").onclick'):]
    handler = handler[:handler.index("};")]
    assert "confirm(" in handler and 'post("reset_map"' in handler
    assert re.search(r'\$\("resetmap"\)\.disabled\s*=', html)


def test_pose_reset_refusal_points_at_reset_map_not_a_stack_restart() -> None:
    src = _read(_NODE)
    body = src[src.index("def _reset_odom("):src.index("def _reset_map(")]
    assert "RESET MAP" in body
    assert "Restart the stack" not in body
