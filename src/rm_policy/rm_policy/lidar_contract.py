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

    `body_floor_m` is the BODY FLOOR: nothing in the world can be nearer the
    lidar than the chassis is, so a return under it is the robot (the
    standoffs read 0.06-0.11 m, 2026-11-04) and takes the no-return path in
    the resampler (range_max, "clear"). `resample_floor_m` is what the nodes
    pass as the resampler's range_min. Absent = 0.0 = off; every shipped
    profile sets it (tests/unit/test_body_floor.py).

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
from typing import Mapping, Tuple

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


def lidar_contract_from(interface_cfg: Mapping) -> LidarContract:
    """Extract the lidar block from a parsed interface.yaml mapping."""
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
        zones = tuple(
            (float(z[0]), float(z[1])) for z in block.get("self_occlusion_zones_deg", [])
        )
        body_floor = float(block.get("body_floor_m", 0.0))
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
