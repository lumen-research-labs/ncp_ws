"""
action_contract.py — the action block of a profile's interface.yaml as a value.

What it does:
    Reads `action.layout` (the `cmd_vx` and `cmd_wz` ranges) and
    `action.pre_scaled` out of an already parsed interface file and returns
    the bounds the checkpoint was trained with, as one frozen object. Two
    things in the tick depend on them, and both must match training:

      * the previous-action block of the observation is
        `(a - bias) / scale` with `bias = (high + low) / 2` and
        `scale = (high - low) / 2` (`observation_builder`);
      * the action the network emits is clipped to the bounds before it is
        published and fed back (`policy_runner`). The training env clips in
        `step()` and feeds the CLIPPED action back (barn_gym/env.py), so an
        unclipped value here would put the next observation off-distribution.

    The vendored `ncp_policy` hardcodes the M8-v2 and M4 bounds ([-1, 2] m/s,
    [-3.14, 3.14] rad/s) in `ACTION_LOW` / `ACTION_HIGH` and applies them in
    `build_obs` and `NcpActor.act`. The 3.0.0 and 4.0.0 contracts declare
    exactly those numbers (3.14, not pi; pinned by a test), so for them the
    runner's clip and the observation's last block come out unchanged, bit
    for bit. 5.0.0 (M9, `config/interface_m9.yaml`) declares the robot's
    clamps, +-0.25 m/s and +-0.8 rad/s, which is what that network saw.

    This module does not import the vendored module (torch), so it and its
    tests run on the workstation tier. `policy_runner` holds the check that
    a contract's bounds sit inside the vendored clip: wider bounds would be
    clipped by `NcpActor.act` first and the fed-back action would differ
    from training with no error.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    Raises `ValueError` naming the file, the key and what was read. A node
    whose action bounds cannot be read must not start; the caller turns this
    into a startup error rather than substituting the vendored constants.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

import numpy as np

VX_NAME = "cmd_vx"
WZ_NAME = "cmd_wz"


@dataclass(frozen=True)
class ActionBounds:
    """`low` and `high` are `(v, w)` in m/s and rad/s."""

    low: Tuple[float, float]
    high: Tuple[float, float]

    # float32 throughout, as the vendored module does it, so a profile that
    # declares the vendored numbers gets an observation bit-identical to
    # `ncp_policy.build_obs`.
    @property
    def low32(self) -> np.ndarray:
        return np.asarray(self.low, dtype=np.float32)

    @property
    def high32(self) -> np.ndarray:
        return np.asarray(self.high, dtype=np.float32)

    @property
    def bias(self) -> np.ndarray:
        return (self.high32 + self.low32) / 2

    @property
    def scale(self) -> np.ndarray:
        return (self.high32 - self.low32) / 2

    def normalize(self, action: Sequence[float]) -> np.ndarray:
        """The previous-action block: `(a - bias) / scale`, float32."""
        return (np.asarray(action, dtype=np.float32) - self.bias) / self.scale

    def clip(self, action: Sequence[float]) -> np.ndarray:
        return np.clip(np.asarray(action, dtype=np.float32), self.low32, self.high32)

    def within(self, low: Sequence[float], high: Sequence[float]) -> bool:
        """True when this range sits inside `[low, high]` on both axes."""
        return bool(
            np.all(self.low32 >= np.asarray(low, dtype=np.float32))
            and np.all(self.high32 <= np.asarray(high, dtype=np.float32))
        )


def action_bounds_from(interface_cfg: Mapping, path: str = "interface.yaml") -> ActionBounds:
    """Extract the action bounds from a parsed interface.yaml mapping."""
    try:
        action = interface_cfg["action"]
        layout = action["layout"]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{path}: missing action.layout") from exc
    if not isinstance(action, Mapping) or action.get("pre_scaled") is not True:
        raise ValueError(
            f"{path}: action.pre_scaled must be true (the actor's linear head "
            f"emits m/s and rad/s and nothing re-scales them), got "
            f"{action.get('pre_scaled') if isinstance(action, Mapping) else action!r}"
        )
    ranges = {}
    for name in (VX_NAME, WZ_NAME):
        block = next((b for b in layout if isinstance(b, Mapping) and b.get("name") == name), None)
        if block is None:
            raise ValueError(f"{path}: no action.layout entry named {name!r}")
        rng = block.get("range")
        try:
            if len(rng) != 2:
                raise ValueError("two values required")
            lo, hi = float(rng[0]), float(rng[1])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{path}: action.layout[{name}].range must be [low, high], got {rng!r}"
            ) from exc
        if not (math.isfinite(lo) and math.isfinite(hi)) or not lo < hi:
            raise ValueError(
                f"{path}: action.layout[{name}].range must be finite with "
                f"low < high, got [{lo}, {hi}]"
            )
        ranges[name] = (lo, hi)
    return ActionBounds(
        low=(ranges[VX_NAME][0], ranges[WZ_NAME][0]),
        high=(ranges[VX_NAME][1], ranges[WZ_NAME][1]),
    )
