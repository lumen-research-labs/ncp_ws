"""
lidar_contract.py — the lidar block of config/interface.yaml as a value.

What it does:
    Reads the `lidar_ranges` entry of `observation.layout` out of an already
    parsed interface.yaml and returns the numbers every scan consumer needs
    (bin count, arc, range bounds, search window, self-occlusion zones) as one
    frozen object. Both `policy_node` and `scan_bridge_node` call this, so the
    contract is read in exactly one place and the two nodes cannot drift
    apart on it.

    One contract ("profile") per trained checkpoint: 4.0.0 (720 beams over
    270 deg, config/interface.yaml), 5.0.0 (the same sensor, interface_m9.yaml)
    and 3.0.0 (32 over 180, config/interface_m4.yaml). The observation is
    always `n_beams + 2 + 2` wide (lidar, goal, previous action), so `obs_dim`
    is derived here and the policy runner builds the network from it; a
    checkpoint of the other width then fails its strict load at startup
    instead of running wrong.

    The MOUNT facts come from the robot profile (config/robots/<name>.yaml,
    `lidar:` block, read by rm_safety/robot_profile.py), not from the
    contract: `self_occlusion_zones_deg` (chassis self-hits the training
    sensor never saw) and `body_floor_m` (nothing in the world is nearer the
    lidar than the chassis; a return under the floor is the robot and takes
    the no-return path, range_max, "clear"). A contract is the checkpoint's
    fact and is the same on every robot; the mount is the unit's fact and
    differed between bernard-1 and bernard-3 (2026-11-18). A contract file
    that still carries either key is refused, so the fact has one home.
    `resample_floor_m` is what the nodes pass as the resampler's range_min.
    With no mount given (`mount=None`, tests and tools) there are no zones
    and the floor is off.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    Raises `ValueError` with the missing or malformed key named. A node that
    cannot read its contract must not start; the callers turn this into a
    startup error rather than substituting defaults.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

LIDAR_BLOCK_NAME = "lidar_ranges"


# The non-lidar blocks every profile shares: goal (x, y) and previous action.
NON_LIDAR_DIMS = 4


@dataclass(frozen=True)
class LidarContract:
    interface_version: str
    n_beams: int
    arc_deg: float
    range_min_m: float
    range_max_m: float
    search_window_bins: int
    mask_zones_deg: Tuple[Tuple[float, float], ...]
    body_floor_m: float = 0.0

    @property
    def obs_dim(self) -> int:
        return self.n_beams + NON_LIDAR_DIMS

    @property
    def resample_floor_m(self) -> float:
        """The resampler's range_min: the sensor floor or the body floor,
        whichever is higher."""
        return max(self.range_min_m, self.body_floor_m)


MOUNT_KEYS = ("self_occlusion_zones_deg", "body_floor_m")


def lidar_contract_from(interface_cfg: Mapping, mount: Optional[Mapping] = None) -> LidarContract:
    """Extract the lidar block from a parsed interface.yaml mapping; the
    mount facts (zones, body floor) from the robot profile's `lidar` block."""
    try:
        version = str(interface_cfg["interface_version"])
        layout = interface_cfg["observation"]["layout"]
    except (KeyError, TypeError) as exc:
        raise ValueError("interface.yaml: missing interface_version or observation.layout") from exc
    block = next((b for b in layout if isinstance(b, Mapping)
                  and b.get("name") == LIDAR_BLOCK_NAME), None)
    if block is None:
        raise ValueError(f"interface.yaml: no observation.layout entry named {LIDAR_BLOCK_NAME!r}")
    try:
        shape = block["shape"]
        if len(shape) != 1:
            raise ValueError(f"shape must have one dimension, got {shape!r}")
        stray = [k for k in MOUNT_KEYS if k in block]
        if stray:
            raise ValueError(
                f"the lidar block carries {stray}: mount facts live in the robot "
                f"profile (config/robots/<name>.yaml lidar:), not in the contract"
            )
        zones: Tuple[Tuple[float, float], ...] = ()
        body_floor = 0.0
        if mount is not None:
            if not isinstance(mount, Mapping):
                raise ValueError(f"robot profile lidar block is not a mapping: {mount!r}")
            missing = [k for k in MOUNT_KEYS if k not in mount]
            if missing:
                raise ValueError(f"robot profile lidar block is missing {missing}")
            zones = tuple((float(z[0]), float(z[1])) for z in mount["self_occlusion_zones_deg"])
            body_floor = float(mount["body_floor_m"])
        if not (math.isfinite(body_floor) and body_floor >= 0.0):
            raise ValueError(
                f"body_floor_m must be finite and >= 0 (metres from the lidar; "
                f"the chassis standoffs read 0.06-0.11 m), got {body_floor!r}"
            )
        return LidarContract(
            interface_version=version,
            n_beams=int(shape[0]),
            arc_deg=float(block["arc_deg"]),
            range_min_m=float(block["range_min_m"]),
            range_max_m=float(block["range_max_m"]),
            search_window_bins=int(block.get("nearest_ray_search_window_bins", 0)),
            mask_zones_deg=zones,
            body_floor_m=body_floor,
        )
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ValueError(f"interface.yaml: malformed {LIDAR_BLOCK_NAME} block: {exc}") from exc
