"""
policy_runner.py — Loads the trained NCP actor and runs one inference tick.

What it does:
    Thin, ROS-free wrapper around `rm_policy.vendor.ncp_policy.NcpActor` (the
    CfC/AutoNCP network vendored from the trained checkpoint's source repo).
    Consumes a plain `obs_dim`-element observation vector (already built and
    normalized by `observation_builder`) and returns a plain `(v, w)` action
    tuple in physical units (m/s, rad/s) — the actor's own linear head + clip
    already produces pre-scaled units, so nothing here re-scales the output.
    Owns the actor's recurrent state across ticks (a CfC is stateful; state
    must be threaded call-to-call or the network sees a discontinuous
    history) and validates every input and output for shape/NaN/inf before
    trusting it.

    Kept deliberately separate from `policy_node.py` per BUILD_PLAN.md
    Phase 4 / project structural discipline: observation building, model
    inference, and the ROS2 node wrapper must never be merged into one file.
    This module knows nothing about ROS, topics, or timers, and is pure
    enough to unit test by calling `.act()` with synthetic vectors.

    DIVERGENCE FROM BUILD_PLAN.md Phase 4 (flagged, not silently absorbed):
    Phase 4 describes `policy_runner.py` as wrapping an ONNX Runtime /
    TensorRT session. No ONNX export of the trained checkpoint exists yet —
    the checkpoint that landed 2026-08-27 is a torch `state_dict` for the
    vendored `NcpActor` (see `vendor/ncp_policy.py` provenance header), so
    this module wraps that directly via `torch`/`ncps`, not an ONNX/TensorRT
    session. Swapping to an exported runtime later is a rewrite of this
    file's internals only; the `PolicyRunner.act()` contract (vector in,
    `InferenceResult` out) should not need to change at call sites.

Subscribes to:  (nothing — pure module, no ROS I/O; caller passes a vector)
Publishes:      (nothing — pure module; caller acts on the returned action)

Failure mode:
    - Bad or missing weights file at construction time -> raises
      `RuntimeError`. This happens during node `__init__`, before any
      subscription or timer exists, so the node fails to come up rather than
      silently driving on garbage/mismatched weights.
    - Malformed observation at inference time (wrong length, NaN, inf) ->
      does NOT run the network. Returns `InferenceResult(action=None,
      fault=<reason>)`. Never returns a partial or best-effort action.
    - NaN/inf/wrong-shape anywhere in the raw network output -> same:
      `action=None`, `fault` set. This can happen even with a well-formed
      input if the recurrent state has diverged; callers MUST check `fault`
      before touching `action` — exactly the `ScanResampleResult` /
      observation-builder-result convention used elsewhere in this package.
    - `reset()` clears the recurrent state. The caller (policy_node) is
      responsible for calling it after a long observation gap (stale scan or
      odom), so a resumed session does not carry a hidden state built from a
      discontinuous history. This module never resets itself implicitly,
      since it has no notion of staleness/time.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple

import numpy as np
import torch

from rm_policy.vendor import ncp_policy

# Bytes per read when hashing the checkpoint. The file is ~2 MB, so this is
# about keeping a constant memory profile, not about speed.
_HASH_CHUNK_BYTES = 1 << 20


def _sha256_file(path: str) -> str:
    """Content hash of a weights file, for run provenance.

    The point is that "which checkpoint was this?" is answerable AFTER the
    fact, from a bag or a dashboard snapshot, without trusting that the file
    at a given path is still the file that ran. A path alone does not identify
    a checkpoint — `weights/actor.pt` is a name that gets overwritten.
    """
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(_HASH_CHUNK_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()

# ---------------------------------------------------------------------------
# Result container — same shape convention as scan_resampler.ScanResampleResult:
# a payload field that is None whenever `fault` is set, and vice versa.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class InferenceResult:
    """Output of `PolicyRunner.act`.

    action:
        `(v, w)` in physical units (m/s, rad/s), already clipped to the
        trained action bounds by `NcpActor.act`. `None` whenever `fault` is
        set — the caller must check `fault` first and must never substitute
        a default action.
    fault:
        `None` on success. Otherwise a reason string identifying which
        precondition failed or why the raw network output was rejected.
    """

    action: Optional[Tuple[float, float]]
    fault: Optional[str]


class PolicyRunner:
    """Owns one `NcpActor` instance and its recurrent state across ticks."""

    def __init__(
        self, weights_path: str = "", seed: int = 0,
        obs_dim: int = ncp_policy.OBS_DIM,
    ) -> None:
        """Construct the actor and optionally load a trained state_dict.

        Parameters
        ----------
        obs_dim:
            Width of the observation the network is built for, from the
            contract (`LidarContract.obs_dim`): 36 for the m4 profile, 724
            for m8v2. The default is the vendored module's own constant.
            A checkpoint exported for the other width fails the strict load
            below, which is the check that the contract and the weights are
            the same policy.
        weights_path:
            Path to a torch state_dict for `NcpActor`. Empty string (the
            default) means "random weights" — this is a legitimate mode for
            Phase 2/3 pipeline exercises where no trained checkpoint is
            being evaluated, not an error condition.
        seed:
            Passed through to `NcpActor` for reproducible random-weight
            initialization when `weights_path` is empty.

        Raises
        ------
        RuntimeError:
            If `weights_path` is non-empty and the file cannot be loaded, or
            its contents do not match `NcpActor`'s parameter shapes. Raised
            at construction time (node startup), deliberately before any
            ROS interface exists, so a bad weights file prevents the node
            from starting at all rather than degrading at runtime.
        """
        self._obs_dim = int(obs_dim)
        self._actor = ncp_policy.NcpActor(obs_dim=self._obs_dim, seed=seed)
        self._weights_path = weights_path
        self._weights_sha256: str = ""
        if weights_path:
            try:
                # map_location="cpu" unconditionally: the actor is tiny and the
                # 5 Hz tick is sub-millisecond on the Jetson's CPU, so there is
                # nothing to gain from CUDA and plenty to lose — a checkpoint
                # saved from a GPU box otherwise tries to allocate a device
                # that may not exist. Upstream made the same fix (69785c8).
                state_dict = torch.load(weights_path, map_location="cpu")
                # strict=True (the default) is load-bearing. A checkpoint whose
                # architecture does not match must fail HERE, at node __init__,
                # not load partially and leave the missing tensors at their
                # random init — which is exactly what the pre-2026-08-29
                # vendored actor would have done under strict=False.
                self._actor.load_state_dict(state_dict)
                self._weights_sha256 = _sha256_file(weights_path)
            except Exception as exc:
                raise RuntimeError(
                    f"policy_runner: failed to load weights from "
                    f"{weights_path!r}: {exc}"
                ) from exc
        self._actor.eval()
        self._state: Any = None  # CfC recurrent (h, c) state; None = reset

    @property
    def weights_path(self) -> str:
        """Path the actor's weights were loaded from; "" means random init."""
        return self._weights_path

    @property
    def obs_dim(self) -> int:
        return self._obs_dim

    @property
    def weights_sha256(self) -> str:
        """SHA-256 of the loaded checkpoint; "" when running random weights.

        Published on the debug stream so every run is traceable to the exact
        bytes that produced it. `weights/actor.pt` is a path that gets
        overwritten; this is the identity that does not.
        """
        return self._weights_sha256

    def reset(self) -> None:
        """Clear recurrent state. Call after any observation discontinuity
        (stale scan/odom, node (re)start) so the next tick does not carry a
        hidden state built from a gap in the input history."""
        self._state = None

    def act(self, obs: Sequence[float]) -> InferenceResult:
        """Run one inference tick.

        `obs` must be exactly `obs_dim` finite floats (36 or 724 by profile),
        in the exact order the contract / `observation_builder` produce.
        This function does not reorder, pad, or truncate a mismatched vector
        — that would silently feed the network garbage that still looks like
        a plausible observation.
        """
        try:
            arr = np.asarray(obs, dtype=np.float32)
        except Exception:
            return InferenceResult(action=None, fault="obs_conversion_failed")

        if arr.shape != (self._obs_dim,):
            return InferenceResult(
                action=None, fault=f"obs_shape_mismatch:{arr.shape}"
            )
        if not np.all(np.isfinite(arr)):
            return InferenceResult(action=None, fault="obs_nonfinite")

        try:
            raw_action, new_state = self._actor.act(arr, self._state)
        except Exception as exc:
            return InferenceResult(
                action=None, fault=f"inference_exception:{exc!r}"
            )

        raw_action = np.asarray(raw_action, dtype=np.float32)
        if raw_action.shape != (2,) or not np.all(np.isfinite(raw_action)):
            # Do NOT commit new_state — a corrupted output means we cannot
            # trust the state that produced it either. Next tick retries
            # from the last known-good state rather than compounding.
            return InferenceResult(action=None, fault="action_invalid")

        self._state = new_state
        return InferenceResult(
            action=(float(raw_action[0]), float(raw_action[1])), fault=None
        )
