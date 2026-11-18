"""The body floor: a lidar return nearer than the chassis is the robot, and no
scan consumer sees it. The floor and the self-occlusion zones are the
ROBOT's facts (config/robots/<name>.yaml lidar:), read by the contract and
the limits readers; this file proves them on bernard-1's measured scan.
Workstation tier (pure modules only).

Measured 2026-11-04 on bernard-1 (tests/fixtures/tg30_scan_2026-11-04_standoffs.yaml):
the right chassis standoffs return 0.06-0.11 m continuously from -138 to -94
deg. The self-occlusion zones end at -97 deg, so the -97..-94 band leaked into
the policy observation as a 0.10 m obstacle (M9 held full left yaw) and into
rm_safety's arcs (min_range_in_arcs read 0.085 m). The floor removes both.
"""

from __future__ import annotations

import ast
import copy
import dataclasses
import io
import math
import os
import sys

import pytest

yaml = pytest.importorskip("yaml")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_safety"))
_CONFIG = os.path.join(_REPO_ROOT, "config")
_FIXTURE = os.path.join(_REPO_ROOT, "tests", "fixtures", "tg30_scan_2026-11-04_standoffs.yaml")
_SLAM_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "slam.launch.py")

from rm_policy import scan_resampler as sr  # noqa: E402
from rm_policy.lidar_contract import lidar_contract_from  # noqa: E402
from rm_safety.robot_profile import load_robot_profile  # noqa: E402
from rm_safety.safety_logic import Limits, min_range_in_arcs  # noqa: E402

BODY_FLOOR_M = 0.12
BODY_RADIUS_M = 0.25
B1 = load_robot_profile(os.path.join(_CONFIG, "robots", "bernard-1.yaml"))
MOUNT = B1.mount


def _cfg(name: str) -> dict:
    with open(os.path.join(_CONFIG, name), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _fixture():
    docs = [d for d in yaml.safe_load_all(io.open(_FIXTURE, encoding="utf-8")) if d]
    d = docs[0]
    return d["ranges"], d["angle_min"], d["angle_increment"]


def _mount(floor: float) -> dict:
    m = copy.deepcopy(MOUNT)
    m["body_floor_m"] = floor
    return m


def _bins(mount: dict):
    ranges, amin, ainc = _fixture()
    c = lidar_contract_from(_cfg("interface.yaml"), mount)
    res = sr.resample_forward_arc(
        ranges, amin, ainc, n_bins=c.n_beams, arc_deg=c.arc_deg,
        range_min=c.resample_floor_m, range_max=c.range_max_m,
        sampling=sr.SAMPLING_NEAREST_RAY,
        search_window_bins=c.search_window_bins, mask_zones_deg=c.mask_zones_deg,
    )
    assert res.fault is None
    return res.bins


def test_the_fixture_is_the_measured_scan() -> None:
    ranges, amin, ainc = _fixture()
    assert len(ranges) == 2020
    short = [(math.degrees(amin + i * ainc), r) for i, r in enumerate(ranges) if 0.0 < r < BODY_FLOOR_M]
    bearings = sorted(b for b, _ in short)
    assert len(short) > 100, "the standoffs are missing from the fixture"
    assert bearings[0] < -137.0 and bearings[-1] > -95.0, bearings[::20]
    assert max(r for _, r in short) < BODY_FLOOR_M


def test_without_the_floor_the_standoffs_reach_the_observation() -> None:
    """The bug: the zones alone let the -97..-94 deg band through."""
    c = lidar_contract_from(_cfg("interface.yaml"), MOUNT)
    bins = _bins(_mount(0.0))
    leaked = [(-c.arc_deg / 2 + (i + 0.5) * c.arc_deg / c.n_beams, b) for i, b in enumerate(bins) if b < BODY_FLOOR_M]
    assert leaked, "the fixture no longer reproduces the leak"
    assert all(-98.0 < deg < -93.0 for deg, _ in leaked), leaked


def test_with_the_floor_nothing_under_the_nearest_real_object_remains() -> None:
    c = lidar_contract_from(_cfg("interface.yaml"), MOUNT)
    assert c.body_floor_m == BODY_FLOOR_M and c.resample_floor_m == BODY_FLOOR_M
    assert min(_bins(MOUNT)) > 0.5   # the nearest real object in that scan was 0.58 m


def test_the_safety_arcs_drop_the_standoffs_the_same_way() -> None:
    ranges, amin, ainc = _fixture()
    raw = _cfg("safety_limits.yaml")
    real = Limits.from_dict(raw, MOUNT)
    assert real.estop.body_floor_m == BODY_FLOOR_M
    assert real.estop.self_occlusion_zones_deg == B1.lidar.self_occlusion_zones_deg
    off = Limits.from_dict(raw, _mount(0.0))
    assert min_range_in_arcs(ranges, amin, ainc, off) < BODY_FLOOR_M      # the bug
    assert min_range_in_arcs(ranges, amin, ainc, real) > 0.5              # the fix


@pytest.mark.parametrize("value, needle", [
    (-0.01, "body_floor_m"), (float("nan"), "body_floor_m"), (0.15, "below stop_zone_m"), (0.5, "below stop_zone_m"),
])
def test_bad_floors_in_a_profile_are_refused_by_the_limits_reader(value, needle) -> None:
    with pytest.raises(ValueError) as exc:
        Limits.from_dict(_cfg("safety_limits.yaml"), _mount(value))
    assert needle in str(exc.value)


@pytest.mark.parametrize("value", [-0.01, float("nan"), float("inf"), "near"])
def test_bad_floors_in_a_profile_are_refused_by_the_contract_reader(value) -> None:
    with pytest.raises(ValueError):
        lidar_contract_from(_cfg("interface.yaml"), _mount(value))


def test_the_facts_have_one_home() -> None:
    """Neither the contracts, nor the limits, nor roam carry the mount facts;
    both robot profiles do, and a file that carries them is refused."""
    for name in ("interface.yaml", "interface_m4.yaml", "interface_m9.yaml", "interface_dynabarn.yaml"):
        block = next(b for b in _cfg(name)["observation"]["layout"] if b["name"] == "lidar_ranges")
        assert "self_occlusion_zones_deg" not in block and "body_floor_m" not in block, name
    est = _cfg("safety_limits.yaml")["estop"]
    assert not {"self_occlusion_zones_deg", "body_floor_m", "body_radius_m"} & set(est)
    assert "body_floor_m" not in _cfg("roam.yaml")["roam"]
    for robot in ("bernard-1", "bernard-3"):
        p = load_robot_profile(os.path.join(_CONFIG, "robots", f"{robot}.yaml"))
        assert p.lidar.body_floor_m == BODY_FLOOR_M and p.lidar.body_radius_m == BODY_RADIUS_M
    assert _cfg("roam.yaml")["roam"]["self_radius_m"] == BODY_RADIUS_M
    raw = _cfg("safety_limits.yaml"); raw["estop"]["body_floor_m"] = 0.12
    with pytest.raises(ValueError, match="mount facts live in the robot profile"):
        Limits.from_dict(raw, MOUNT)


def _sanitize():
    src = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "scan_sanitizer_node.py")
    tree = ast.parse(open(src, encoding="utf-8").read())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "sanitize_ranges")
    ns = {"math": math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), src, "exec"), ns)
    return ns["sanitize_ranges"]


def test_the_sanitizer_hides_the_corners_from_third_parties() -> None:
    sanitize = _sanitize()
    out = sanitize([0.0, 0.07, 0.17, 0.249, 0.25, 0.26, 2.0], 0.01, BODY_RADIUS_M)
    assert out[:4] == [math.inf] * 4
    assert out[4:] == [0.25, 0.26, 2.0]
    assert sanitize([0.07, 0.17], 0.01) == [0.07, 0.17]   # radius 0.0 = off, as before


def test_the_slam_launch_takes_the_radius_from_the_robot_profile() -> None:
    src = open(_SLAM_LAUNCH, encoding="utf-8").read()
    assert "load_robot_profile(path).lidar.body_radius_m" in src
    assert "safety_limits" not in src


def test_roam_turns_are_not_refused_by_the_chassis() -> None:
    """Roam reads the raw scan with its own body rule. On the fixture every
    turn was refused ("a return sits inside the corner sweep either way")
    because the standoffs at 0.09 m sat inside the front-right corner's
    reach; with the profile's floor the nearest return is the real object."""
    from rm_policy import roam
    ranges, amin, ainc = _fixture()
    scan = roam.Scan(ranges=ranges, angle_min=amin, angle_increment=ainc, range_min=0.01)
    base = roam.load_roam_limits(os.path.join(_CONFIG, "roam.yaml"))
    assert base.body_floor_m == 0.0, "the floor is the profile's, not roam.yaml's"
    lim = dataclasses.replace(base, body_floor_m=B1.lidar.body_floor_m)
    assert roam.check_limits(lim) is None
    assert roam.nearest_return(scan, lim.self_radius_m) < BODY_FLOOR_M          # the bug
    assert roam.nearest_return(scan, lim.self_radius_m, lim.body_floor_m) > lim.turn_clear_m  # the fix: 0.39 m, a real object
    assert roam.turn_direction(scan, math.radians(60.0), lim) is not None
    assert roam.turn_direction(scan, math.radians(-60.0), lim) is not None
