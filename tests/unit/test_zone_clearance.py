"""The safety zones are clearances beyond the robot's lidar.min_range_m.

bernard-3 (RPLIDAR A1, blind inside 0.15 m) drove into things with both
zones on (2026-11-18): the shared 0.15 m stop zone, a LIDAR RANGE that was
bernard-1's, sat on the blind radius and could never fire, and the slow zone
failed open as an object crossed into it. Now safety_limits.yaml carries
clearances and each robot's profile carries min_range_m, the larger of the
blind radius and the lidar-to-bumper distance. Workstation tier.
"""

from __future__ import annotations

import copy
import os
import sys

import pytest

yaml = pytest.importorskip("yaml")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_safety"))
_CONFIG = os.path.join(_REPO_ROOT, "config")

from rm_safety.robot_profile import load_robot_profile  # noqa: E402
from rm_safety.safety_logic import Limits, estop_zone  # noqa: E402


def _raw() -> dict:
    with open(os.path.join(_CONFIG, "safety_limits.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _profile(name: str):
    return load_robot_profile(os.path.join(_CONFIG, "robots", f"{name}.yaml"))


def test_the_file_carries_clearances_not_ranges() -> None:
    est = _raw()["estop"]
    assert "stop_zone_m" not in est and "slow_zone_m" not in est
    assert est["stop_clearance_m"] == pytest.approx(0.08)
    assert est["slow_clearance_m"] == pytest.approx(0.48)


def test_bernard_1_keeps_the_zones_it_always_had() -> None:
    lim = Limits.from_dict(_raw(), _profile("bernard-1").mount)
    assert lim.estop.min_range_m == pytest.approx(0.07)
    assert lim.estop.stop_zone_m == pytest.approx(0.15)
    assert lim.estop.slow_zone_m == pytest.approx(0.55)


def test_bernard_3s_zones_sit_outside_the_a1s_blind_radius() -> None:
    lim = Limits.from_dict(_raw(), _profile("bernard-3").mount)
    assert lim.estop.min_range_m == pytest.approx(0.15)
    assert lim.estop.stop_zone_m == pytest.approx(0.23)
    assert lim.estop.slow_zone_m == pytest.approx(0.63)
    # the bug: at the blind radius the old stop zone could not fire; now it is "stop"
    assert estop_zone(0.16, lim) == "stop"
    assert estop_zone(0.30, lim) == "slow" and estop_zone(0.70, lim) == "clear"


def test_the_profiles_min_range_is_the_sensor_or_the_bumper() -> None:
    """A1 blind radius 0.15 m > bumper 0.075 m; TG30 sees past the bumper (0.07 m)."""
    assert _profile("bernard-3").lidar.min_range_m == 0.15
    assert _profile("bernard-1").lidar.min_range_m == 0.07


@pytest.mark.parametrize("key", ["stop_zone_m", "slow_zone_m"])
def test_a_zone_written_as_a_lidar_range_is_refused(key) -> None:
    raw = _raw(); raw["estop"][key] = 0.15
    with pytest.raises(ValueError, match="clearances beyond the robot"):
        Limits.from_dict(raw, _profile("bernard-1").mount)


def test_a_mount_without_min_range_is_refused() -> None:
    mount = copy.deepcopy(_profile("bernard-1").mount); mount.pop("min_range_m")
    with pytest.raises(ValueError, match="min_range_m"):
        Limits.from_dict(_raw(), mount)


@pytest.mark.parametrize("fields, needle", [
    ({"stop_clearance_m": 0.0}, "stop_clearance_m"),
    ({"stop_clearance_m": float("nan")}, "stop_clearance_m"),
    ({"slow_clearance_m": 0.08}, "slow_clearance_m"),
    ({"slow_clearance_m": 0.05}, "slow_clearance_m"),
])
def test_bad_clearances_are_refused(fields, needle) -> None:
    raw = _raw(); raw["estop"].update(fields)
    with pytest.raises(ValueError, match=needle):
        Limits.from_dict(raw, _profile("bernard-1").mount)


def test_without_a_mount_the_clearances_are_the_ranges() -> None:
    lim = Limits.from_dict(_raw())
    assert lim.estop.min_range_m == 0.0
    assert lim.estop.stop_zone_m == pytest.approx(0.08) and lim.estop.slow_zone_m == pytest.approx(0.48)


def test_the_start_line_and_the_runway_tool_follow() -> None:
    node = open(os.path.join(_REPO_ROOT, "src", "rm_safety", "rm_safety", "safety_node.py"), encoding="utf-8").read()
    assert "es.min_range_m" in node and "es.stop_clearance_m" in node
    tool = open(os.path.join(_REPO_ROOT, "scripts", "diagnostics", "velocity_accuracy.py"), encoding="utf-8").read()
    assert 'limits["estop"]["slow_zone_m"]' not in tool
    assert "Limits.from_dict(limits, mount).estop.slow_zone_m" in tool
