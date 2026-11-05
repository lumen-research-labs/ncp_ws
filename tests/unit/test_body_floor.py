"""The body floor: a lidar return nearer than the chassis is the robot, and no
scan consumer sees it. Workstation tier (pure modules only).

Measured 2026-11-04 on bernard-1 (tests/fixtures/tg30_scan_2026-11-04_standoffs.yaml):
the right chassis standoffs return 0.06-0.11 m continuously from -138 to -94
deg. The self-occlusion zones end at -97 deg, so the -97..-94 band leaked into
the policy observation as a 0.10 m obstacle (M9 held full left yaw) and into
rm_safety's arcs (min_range_in_arcs read 0.085 m). The floor removes both.
"""

from __future__ import annotations

import ast
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
from rm_safety.safety_logic import Limits, min_range_in_arcs  # noqa: E402

BODY_FLOOR_M = 0.12
BODY_RADIUS_M = 0.25
PROFILE_FILES = ("interface.yaml", "interface_m4.yaml", "interface_m9.yaml")


def _cfg(name: str) -> dict:
    with open(os.path.join(_CONFIG, name), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _fixture():
    docs = [d for d in yaml.safe_load_all(io.open(_FIXTURE, encoding="utf-8")) if d]
    d = docs[0]
    return d["ranges"], d["angle_min"], d["angle_increment"]


def _bins(range_min: float):
    ranges, amin, ainc = _fixture()
    c = lidar_contract_from(_cfg("interface.yaml"))
    res = sr.resample_forward_arc(
        ranges, amin, ainc, n_bins=c.n_beams, arc_deg=c.arc_deg,
        range_min=range_min, range_max=c.range_max_m,
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
    """The bug: the sensor floor alone lets the -97..-94 deg band through."""
    c = lidar_contract_from(_cfg("interface.yaml"))
    bins = _bins(c.range_min_m)
    leaked = [(-c.arc_deg / 2 + (i + 0.5) * c.arc_deg / c.n_beams, b) for i, b in enumerate(bins) if b < BODY_FLOOR_M]
    assert leaked, "the fixture no longer reproduces the leak"
    assert all(-98.0 < deg < -93.0 for deg, _ in leaked), leaked


def test_with_the_floor_nothing_under_the_nearest_real_object_remains() -> None:
    c = lidar_contract_from(_cfg("interface.yaml"))
    assert c.body_floor_m == BODY_FLOOR_M and c.resample_floor_m == BODY_FLOOR_M
    bins = _bins(c.resample_floor_m)
    assert min(bins) > 0.5, min(bins)   # the nearest real object in that scan was 0.58 m


def test_the_safety_arcs_drop_the_standoffs_the_same_way() -> None:
    ranges, amin, ainc = _fixture()
    real = Limits.from_dict(_cfg("safety_limits.yaml"))
    assert real.estop.body_floor_m == BODY_FLOOR_M
    raw = yaml.safe_load(open(os.path.join(_CONFIG, "safety_limits.yaml"), encoding="utf-8"))
    raw["estop"]["body_floor_m"] = 0.0
    off = Limits.from_dict(raw)
    assert min_range_in_arcs(ranges, amin, ainc, off) < BODY_FLOOR_M      # the bug
    assert min_range_in_arcs(ranges, amin, ainc, real) > 0.5              # the fix


@pytest.mark.parametrize("value, needle", [
    (-0.01, "body_floor_m"), (float("nan"), "body_floor_m"), (0.15, "below stop_zone_m"), (0.5, "below stop_zone_m"),
])
def test_bad_safety_floors_are_refused_by_name(value, needle) -> None:
    raw = yaml.safe_load(open(os.path.join(_CONFIG, "safety_limits.yaml"), encoding="utf-8"))
    raw["estop"]["body_floor_m"] = value
    with pytest.raises(ValueError) as exc:
        Limits.from_dict(raw)
    assert needle in str(exc.value)


@pytest.mark.parametrize("value", [-0.01, float("nan"), float("inf"), "near"])
def test_bad_contract_floors_are_refused(value) -> None:
    cfg = _cfg("interface.yaml")
    block = next(b for b in cfg["observation"]["layout"] if b["name"] == "lidar_ranges")
    block["body_floor_m"] = value
    with pytest.raises(ValueError):
        lidar_contract_from(cfg)


def test_an_absent_floor_is_off_and_every_shipped_profile_sets_it() -> None:
    cfg = _cfg("interface.yaml")
    block = next(b for b in cfg["observation"]["layout"] if b["name"] == "lidar_ranges")
    del block["body_floor_m"]
    c = lidar_contract_from(cfg)
    assert c.body_floor_m == 0.0 and c.resample_floor_m == c.range_min_m
    for name in PROFILE_FILES:
        assert lidar_contract_from(_cfg(name)).body_floor_m == BODY_FLOOR_M, name


def test_the_two_paths_and_roam_agree_on_the_body() -> None:
    """Independent files, the same measurement: the floor in every contract
    equals the safety file's; the sanitizer's radius equals roam's."""
    est = _cfg("safety_limits.yaml")["estop"]
    assert est["body_floor_m"] == BODY_FLOOR_M
    assert est["body_radius_m"] == BODY_RADIUS_M == _cfg("roam.yaml")["roam"]["self_radius_m"]


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


def test_the_slam_launch_passes_the_radius_from_the_limits_file() -> None:
    src = open(_SLAM_LAUNCH, encoding="utf-8").read()
    assert "body_radius_m" in src and "safety_limits" in src
