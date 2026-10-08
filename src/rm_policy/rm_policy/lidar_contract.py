"""
lidar_contract.py — the lidar block of config/interface.yaml as a value.

What it does:
    Reads the `lidar_ranges` entry of `observation.layout` out of an already
    parsed interface.yaml and returns the numbers every scan consumer needs
    (bin count, arc, range bounds, search window, self-occlusion zones) as one
    frozen object. Both `policy_node` and `scan_bridge_node` call this, so the
    contract is read in exactly one place and the two nodes cannot drift
    apart on it.

    The bin count and arc are also baked into the vendored checkpoint module
    (`ncp_policy.BEAMS` / `FOV_DEG`); the caller is responsible for checking
    the two agree, because this module is deliberately free of that import.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    Raises `ValueError` with the missing or malformed key named. A node that
    cannot read its contract must not start; the callers turn this into a
    startup error rather than substituting defaults.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Tuple

LIDAR_BLOCK_NAME = "lidar_ranges"


@dataclass(frozen=True)
class LidarContract:
    n_beams: int
    arc_deg: float
    range_min_m: float
    range_max_m: float
    search_window_bins: int
    mask_zones_deg: Tuple[Tuple[float, float], ...]


def lidar_contract_from(interface_cfg: Mapping) -> LidarContract:
    """Extract the lidar block from a parsed interface.yaml mapping."""
    try:
        layout = interface_cfg["observation"]["layout"]
    except (KeyError, TypeError) as exc:
        raise ValueError("interface.yaml: missing observation.layout") from exc
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
        return LidarContract(
            n_beams=int(shape[0]),
            arc_deg=float(block["arc_deg"]),
            range_min_m=float(block["range_min_m"]),
            range_max_m=float(block["range_max_m"]),
            search_window_bins=int(block.get("nearest_ray_search_window_bins", 0)),
            mask_zones_deg=zones,
        )
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ValueError(f"interface.yaml: malformed {LIDAR_BLOCK_NAME} block: {exc}") from exc
