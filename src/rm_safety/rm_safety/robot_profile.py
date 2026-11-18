"""
robot_profile.py — one file per robot, `config/robots/<name>.yaml`, loaded and
validated. Pure python, no ROS.

What it does:
    A robot is a profile: the facts about one physical unit that the stack
    must not hardcode, because the second unit (bernard-3, a stock Yahboom
    X3 brought up 2026-11-17) differed from the first in every one of them:

      robot:  name, the hostnames `hostname` returns on that unit, and its
              ROS domain (two units on one network with one domain share
              /cmd_vel and every dashboard shows both);
      lidar:  the driver to start (model), where the sensor's zero points
              (yaw_deg), and the mount measurements that were spread over
              interface*.yaml, safety_limits.yaml and roam.yaml before:
              the self-occlusion zones, the body floor and the body radius;
      base:   which calibration file rm_base loads (the encoder sign lives
              in that file, next to the wheel model it belongs to).

    `resolve_profile_path` picks the profile by the robot's own hostname (or
    by name, for an explicit override), so `run_stack.sh` needs no arguments
    on any unit. `check_profiles` holds the whole directory to one rule set:
    every file loads, and no two profiles share a name, a hostname or a
    domain. The launch, the nodes, the scripts and the dashboard all read the
    same file through this module (policy_node reads the YAML itself, as it
    already does for safety_limits.yaml; the keys are the same).

    This module lives in rm_safety for the same reason command_sources.py
    does: the safety node is an authority that must never import the
    packages that import it.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    Every function raises ValueError (or the open/parse error) naming the
    file and the key. Callers treat that as fatal at startup: a stack that
    does not know which robot it is on must not start, because the
    alternative is bernard-1's measurements driving bernard-3, which is what
    happened for two days.
"""

from __future__ import annotations

import dataclasses
import math
import pathlib
import re
import socket
from typing import Any, List, Optional, Sequence, Tuple

import yaml

_THIS_DIR = pathlib.Path(__file__).parent.resolve()
REPO_ROOT = _THIS_DIR.parents[2]
DEFAULT_ROBOTS_DIR = str(REPO_ROOT / "config" / "robots")

LIDAR_MODELS = ("tg30", "a1")
_NAME_RE = re.compile(r"^[a-z][a-z0-9-]*$")
_HOSTNAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9.-]*$")
# ROS 2 domain IDs: 0..101 is the range every platform supports; 232 is the
# protocol ceiling. Refuse anything outside the safe range.
_DOMAIN_MAX = 101


@dataclasses.dataclass(frozen=True)
class LidarMount:
    model: str                                   # one of LIDAR_MODELS
    yaw_deg: float                               # where the sensor's zero points, degrees left of +x
    self_occlusion_zones_deg: Tuple[Tuple[float, float], ...]
    body_floor_m: float                          # under this: the chassis, not the world
    body_radius_m: float                         # the corners; the sanitizer's floor for third parties


@dataclasses.dataclass(frozen=True)
class RobotProfile:
    name: str
    hostnames: Tuple[str, ...]
    ros_domain_id: int
    lidar: LidarMount
    calibration_path: str                        # absolute
    path: str                                    # the profile file itself

    @property
    def mount(self) -> dict:
        """The lidar block as the contract and limits readers take it."""
        return {
            "self_occlusion_zones_deg": [list(z) for z in self.lidar.self_occlusion_zones_deg],
            "body_floor_m": self.lidar.body_floor_m,
            "body_radius_m": self.lidar.body_radius_m,
        }


def _number(block: dict, key: str, path: str, *, finite: bool = True) -> float:
    try:
        v = float(block[key])
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"{path}: {key} must be a number, got {block.get(key)!r}") from None
    if finite and not math.isfinite(v):
        raise ValueError(f"{path}: {key} must be finite, got {v!r}")
    return v


def parse_zones(raw: Any, path: str) -> Tuple[Tuple[float, float], ...]:
    """`[[lo, hi], ...]` in degrees, each finite, lo <= hi, within [-180, 180]."""
    if raw is None:
        raise ValueError(f"{path}: lidar.self_occlusion_zones_deg is required ([] for none)")
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"{path}: lidar.self_occlusion_zones_deg must be a list of [lo, hi] pairs")
    zones = []
    for z in raw:
        try:
            if len(z) != 2:
                raise ValueError
            lo, hi = float(z[0]), float(z[1])
        except (TypeError, ValueError):
            raise ValueError(f"{path}: lidar.self_occlusion_zones_deg entry {z!r} is not a [lo, hi] pair") from None
        if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi or lo < -180.0 or hi > 180.0:
            raise ValueError(
                f"{path}: lidar.self_occlusion_zones_deg entry [{lo}, {hi}] must be finite "
                f"degrees within [-180, 180] with lo <= hi"
            )
        zones.append((lo, hi))
    return tuple(zones)


def parse_robot_profile(raw: Any, path: str, repo_root: pathlib.Path = REPO_ROOT) -> RobotProfile:
    """Validate a parsed profile document. Raises ValueError, never defaults."""
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: the profile must be a mapping with robot:, lidar: and base: blocks")
    for block in ("robot", "lidar", "base"):
        if not isinstance(raw.get(block), dict):
            raise ValueError(f"{path}: missing or malformed `{block}:` block")
    robot, lidar, base = raw["robot"], raw["lidar"], raw["base"]

    name = robot.get("name")
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise ValueError(f"{path}: robot.name must match {_NAME_RE.pattern}, got {name!r}")
    hostnames = robot.get("hostnames")
    if (not isinstance(hostnames, (list, tuple)) or not hostnames
            or not all(isinstance(h, str) and _HOSTNAME_RE.match(h) for h in hostnames)):
        raise ValueError(f"{path}: robot.hostnames must be a non-empty list of hostnames, got {hostnames!r}")
    domain = robot.get("ros_domain_id")
    if not isinstance(domain, int) or isinstance(domain, bool) or not 0 <= domain <= _DOMAIN_MAX:
        raise ValueError(f"{path}: robot.ros_domain_id must be an integer in [0, {_DOMAIN_MAX}], got {domain!r}")

    model = lidar.get("model")
    if model not in LIDAR_MODELS:
        raise ValueError(f"{path}: lidar.model must be one of {LIDAR_MODELS}, got {model!r}")
    yaw = _number(lidar, "yaw_deg", path)
    zones = parse_zones(lidar.get("self_occlusion_zones_deg"), path)
    floor = _number(lidar, "body_floor_m", path)
    radius = _number(lidar, "body_radius_m", path)
    if not 0.0 <= floor < radius:
        raise ValueError(
            f"{path}: lidar.body_floor_m ({floor}) must be >= 0 and below lidar.body_radius_m ({radius}); "
            f"the floor is the standoffs, the radius the chassis corners"
        )

    cal = base.get("calibration")
    if not isinstance(cal, str) or not cal:
        raise ValueError(f"{path}: base.calibration must be a path to the robot's base calibration file")
    cal_path = pathlib.Path(cal)
    if not cal_path.is_absolute():
        cal_path = repo_root / cal_path
    if not cal_path.is_file():
        raise ValueError(f"{path}: base.calibration {cal!r} does not exist (looked at {cal_path})")

    return RobotProfile(
        name=name, hostnames=tuple(hostnames), ros_domain_id=domain,
        lidar=LidarMount(model=model, yaw_deg=yaw, self_occlusion_zones_deg=zones,
                         body_floor_m=floor, body_radius_m=radius),
        calibration_path=str(cal_path), path=str(path),
    )


def load_robot_profile(path: str, repo_root: pathlib.Path = REPO_ROOT) -> RobotProfile:
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    return parse_robot_profile(raw, str(path), repo_root)


def list_profiles(robots_dir: str = DEFAULT_ROBOTS_DIR, repo_root: pathlib.Path = REPO_ROOT) -> List[RobotProfile]:
    d = pathlib.Path(robots_dir)
    if not d.is_dir():
        raise ValueError(f"{robots_dir}: the robots directory does not exist")
    files = sorted(p for p in d.iterdir() if p.suffix in (".yaml", ".yml"))
    if not files:
        raise ValueError(f"{robots_dir}: no robot profiles (expected <name>.yaml files)")
    profiles = [load_robot_profile(str(p), repo_root) for p in files]
    for p, f in zip(profiles, files):
        if f.stem != p.name:
            raise ValueError(f"{f}: file name {f.stem!r} must equal robot.name {p.name!r}")
    return profiles


def check_profiles(robots_dir: str = DEFAULT_ROBOTS_DIR, repo_root: pathlib.Path = REPO_ROOT) -> List[RobotProfile]:
    """Every profile loads, and no two share a name, a hostname or a domain."""
    profiles = list_profiles(robots_dir, repo_root)
    seen_host: dict = {}
    seen_domain: dict = {}
    for p in profiles:
        for h in p.hostnames:
            if h in seen_host:
                raise ValueError(f"{p.path} and {seen_host[h]} both claim hostname {h!r}")
            seen_host[h] = p.path
        if p.ros_domain_id in seen_domain:
            raise ValueError(
                f"{p.path} and {seen_domain[p.ros_domain_id]} both use ROS domain "
                f"{p.ros_domain_id}; two robots on one network would share /cmd_vel"
            )
        seen_domain[p.ros_domain_id] = p.path
    return profiles


def resolve_profile_path(robots_dir: str = DEFAULT_ROBOTS_DIR, hostname: Optional[str] = None,
                         repo_root: pathlib.Path = REPO_ROOT) -> str:
    """The profile whose `hostnames` (or name) matches `hostname` (default: this
    machine's). Exactly one must match."""
    host = hostname if hostname is not None else socket.gethostname()
    profiles = check_profiles(robots_dir, repo_root)
    hits = [p for p in profiles if host == p.name or host in p.hostnames]
    if len(hits) == 1:
        return hits[0].path
    known = ", ".join(f"{p.name} ({', '.join(p.hostnames)})" for p in profiles)
    if not hits:
        raise ValueError(
            f"no robot profile in {robots_dir} lists hostname {host!r}; known: {known}. "
            f"Add this host to a profile's robot.hostnames, or set ROBOT=<name>."
        )
    raise ValueError(f"hostname {host!r} matches more than one profile: {[p.path for p in hits]}")
