"""rm_safety.robot_profile: one file per robot, resolved by hostname.

The shipped profiles must load and agree with each other; a malformed one
must be refused with the file and key named; a hostname no profile lists
must be refused with the known names listed. Workstation tier.
"""

from __future__ import annotations

import os
import pathlib
import sys

import pytest

yaml = pytest.importorskip("yaml")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_safety"))
_ROBOTS = os.path.join(_REPO_ROOT, "config", "robots")

from rm_safety.robot_profile import (  # noqa: E402
    DEFAULT_ROBOTS_DIR, LIDAR_MODELS, check_profiles, list_profiles,
    load_robot_profile, parse_robot_profile, resolve_profile_path,
)


def _good(**over) -> dict:
    d = {
        "robot": {"name": "testbot", "hostnames": ["testbot", "jetson-x"], "ros_domain_id": 40},
        "lidar": {"model": "tg30", "yaw_deg": 0.0, "self_occlusion_zones_deg": [[92.0, 123.0]],
                  "body_floor_m": 0.12, "body_radius_m": 0.25},
        "base": {"calibration": "config/base_calibration.yaml"},
    }
    for k, v in over.items():
        block, key = k.split("__")
        if v is None:
            d[block].pop(key, None)
        else:
            d[block][key] = v
    return d


def test_the_shipped_profiles_load_and_agree() -> None:
    assert os.path.realpath(DEFAULT_ROBOTS_DIR) == os.path.realpath(_ROBOTS)
    profiles = {p.name: p for p in check_profiles(_ROBOTS)}
    assert set(profiles) == {"bernard-1", "bernard-3"}
    b1, b3 = profiles["bernard-1"], profiles["bernard-3"]
    assert b1.ros_domain_id == 28 and b3.ros_domain_id == 29
    assert b1.lidar.model == "tg30" and b1.lidar.yaw_deg == 0.0
    assert b3.lidar.model == "a1" and b3.lidar.yaw_deg == 180.0
    assert "yahboom" in b3.hostnames and "bernard-3" in b3.hostnames
    # bernard-1's mount numbers are the ones the contracts and the limits carried until 2026-11-18.
    assert b1.lidar.self_occlusion_zones_deg == ((92.0, 123.0), (-135.0, -97.0))
    assert b1.lidar.body_floor_m == 0.12 and b1.lidar.body_radius_m == 0.25
    assert b3.lidar.self_occlusion_zones_deg == ()
    assert os.path.isfile(b1.calibration_path) and os.path.isfile(b3.calibration_path)
    assert b1.calibration_path.endswith("base_calibration.yaml")
    assert b3.calibration_path.endswith("base_calibration_bernard3.yaml")


def test_the_mount_view_is_what_the_readers_take() -> None:
    b1 = {p.name: p for p in list_profiles(_ROBOTS)}["bernard-1"]
    assert b1.mount == {"self_occlusion_zones_deg": [[92.0, 123.0], [-135.0, -97.0]],
                        "body_floor_m": 0.12, "body_radius_m": 0.25}


def test_resolution_by_hostname_or_name() -> None:
    assert resolve_profile_path(_ROBOTS, "yahboom").endswith("bernard-3.yaml")
    assert resolve_profile_path(_ROBOTS, "bernard-3").endswith("bernard-3.yaml")
    assert resolve_profile_path(_ROBOTS, "bernard-1").endswith("bernard-1.yaml")
    with pytest.raises(ValueError) as exc:
        resolve_profile_path(_ROBOTS, "some-other-box")
    msg = str(exc.value)
    assert "some-other-box" in msg and "bernard-1" in msg and "bernard-3" in msg and "yahboom" in msg


@pytest.mark.parametrize("over, needle", [
    ({"robot__name": None}, "robot.name"),
    ({"robot__name": "Bad Name"}, "robot.name"),
    ({"robot__hostnames": []}, "robot.hostnames"),
    ({"robot__hostnames": "yahboom"}, "robot.hostnames"),
    ({"robot__ros_domain_id": "28"}, "ros_domain_id"),
    ({"robot__ros_domain_id": 300}, "ros_domain_id"),
    ({"robot__ros_domain_id": True}, "ros_domain_id"),
    ({"lidar__model": "rplidar"}, "lidar.model"),
    ({"lidar__yaw_deg": float("nan")}, "yaw_deg"),
    ({"lidar__yaw_deg": "back"}, "yaw_deg"),
    ({"lidar__self_occlusion_zones_deg": None}, "self_occlusion_zones_deg"),
    ({"lidar__self_occlusion_zones_deg": [[100.0]]}, "self_occlusion_zones_deg"),
    ({"lidar__self_occlusion_zones_deg": [[130.0, 100.0]]}, "self_occlusion_zones_deg"),
    ({"lidar__self_occlusion_zones_deg": [[-200.0, -100.0]]}, "self_occlusion_zones_deg"),
    ({"lidar__body_floor_m": 0.3}, "body_floor_m"),
    ({"lidar__body_floor_m": -0.1}, "body_floor_m"),
    ({"lidar__body_radius_m": None}, "body_radius_m"),
    ({"base__calibration": "config/no_such_file.yaml"}, "base.calibration"),
    ({"base__calibration": None}, "base.calibration"),
])
def test_malformed_profiles_are_refused_with_the_file_and_key_named(over, needle) -> None:
    with pytest.raises(ValueError) as exc:
        parse_robot_profile(_good(**over), "robots/testbot.yaml", pathlib.Path(_REPO_ROOT))
    assert "robots/testbot.yaml" in str(exc.value)
    assert needle in str(exc.value)


def test_a_missing_block_is_named() -> None:
    for block in ("robot", "lidar", "base"):
        raw = _good(); raw.pop(block)
        with pytest.raises(ValueError, match=f"`{block}:`"):
            parse_robot_profile(raw, "x.yaml", pathlib.Path(_REPO_ROOT))


def _write(dirpath: pathlib.Path, raw: dict) -> None:
    (dirpath / f"{raw['robot']['name']}.yaml").write_text(yaml.safe_dump(raw))


def test_two_profiles_may_not_share_a_hostname_or_a_domain(tmp_path) -> None:
    a = _good(); b = _good(robot__name="otherbot", robot__hostnames=["jetson-x"], robot__ros_domain_id=41)
    _write(tmp_path, a); _write(tmp_path, b)
    with pytest.raises(ValueError, match="both claim hostname 'jetson-x'"):
        check_profiles(str(tmp_path), pathlib.Path(_REPO_ROOT))
    b["robot"]["hostnames"] = ["otherbot"]; b["robot"]["ros_domain_id"] = 40
    _write(tmp_path, b)
    with pytest.raises(ValueError, match="both use ROS domain 40"):
        check_profiles(str(tmp_path), pathlib.Path(_REPO_ROOT))
    b["robot"]["ros_domain_id"] = 41
    _write(tmp_path, b)
    assert [p.name for p in check_profiles(str(tmp_path), pathlib.Path(_REPO_ROOT))] == ["otherbot", "testbot"]


def test_the_file_name_must_be_the_robots_name(tmp_path) -> None:
    (tmp_path / "wrong.yaml").write_text(yaml.safe_dump(_good()))
    with pytest.raises(ValueError, match="file name 'wrong' must equal robot.name 'testbot'"):
        list_profiles(str(tmp_path), pathlib.Path(_REPO_ROOT))


def test_an_empty_or_missing_robots_dir_is_refused(tmp_path) -> None:
    with pytest.raises(ValueError, match="no robot profiles"):
        list_profiles(str(tmp_path), pathlib.Path(_REPO_ROOT))
    with pytest.raises(ValueError, match="does not exist"):
        list_profiles(str(tmp_path / "nowhere"), pathlib.Path(_REPO_ROOT))


def test_the_scripts_source_robot_env_and_the_launch_gets_the_profile() -> None:
    run = open(os.path.join(_REPO_ROOT, "scripts", "run_stack.sh"), encoding="utf-8").read()
    down = open(os.path.join(_REPO_ROOT, "scripts", "stack_down.sh"), encoding="utf-8").read()
    env = open(os.path.join(_REPO_ROOT, "scripts", "robot_env.sh"), encoding="utf-8").read()
    assert 'source "$(dirname "$0")/robot_env.sh"' in run and 'source "$(dirname "$0")/robot_env.sh"' in down
    assert "export ROS_DOMAIN_ID=28" not in run, "the domain comes from the profile now"
    assert 'robot_profile:=${RM_ROBOT_PROFILE}' in run
    assert "resolve_profile_path" in env and "export RM_ROBOT_PROFILE ROS_DOMAIN_ID RM_ROBOT_NAME" in env
    assert LIDAR_MODELS == ("tg30", "a1")
