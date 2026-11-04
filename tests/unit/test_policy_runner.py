"""Tests for rm_policy.policy_runner's action bounds (robot tier: torch).

The runner clips every action to the contract's bounds after the vendored
actor's own clip, so a checkpoint trained at narrower bounds (5.0.0, M9)
emits, publishes and feeds back a value inside them, as its training env did.
"""

from __future__ import annotations

import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not installed (workstation tier)")
pytest.importorskip("torch", reason="torch not installed (workstation tier)")
pytest.importorskip("ncps", reason="ncps not installed (workstation tier)")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.action_contract import ActionBounds  # noqa: E402
from rm_policy.policy_runner import PolicyRunner  # noqa: E402
from rm_policy.vendor import ncp_policy  # noqa: E402

M9 = ActionBounds(low=(-0.25, -0.8), high=(0.25, 0.8))
VENDORED = ActionBounds(
    low=tuple(float(v) for v in ncp_policy.ACTION_LOW),
    high=tuple(float(v) for v in ncp_policy.ACTION_HIGH),
)


def _obs_sequence(n=30, seed=0):
    rng = np.random.default_rng(seed)
    return [rng.uniform(-1.0, 1.0, size=ncp_policy.OBS_DIM).astype(np.float32) for _ in range(n)]


def test_default_bounds_are_the_vendored_ones() -> None:
    assert PolicyRunner().action_bounds == VENDORED


def test_actions_are_clipped_to_the_contract_bounds() -> None:
    runner = PolicyRunner(seed=0, action_bounds=M9)
    seen_outside_vendored_default = False
    for obs in _obs_sequence(seed=1):
        res = runner.act(obs)
        assert res.fault is None
        v, w = res.action
        assert -0.25 <= v <= 0.25 and -0.8 <= w <= 0.8
        seen_outside_vendored_default |= abs(v) == 0.25 or abs(w) == 0.8
    # Random weights saturate somewhere in 30 ticks; if they never did the
    # clip was not exercised and this test proves nothing.
    assert seen_outside_vendored_default


def test_with_the_vendored_bounds_the_action_is_the_actors_own() -> None:
    a, b = PolicyRunner(seed=3), PolicyRunner(seed=3, action_bounds=VENDORED)
    for obs in _obs_sequence(seed=2):
        ra, rb = a.act(obs), b.act(obs)
        assert ra.fault is None and rb.fault is None
        assert ra.action == rb.action


@pytest.mark.parametrize("bounds", [
    ActionBounds((-1.0, -4.0), (2.0, 4.0)),
    ActionBounds((-1.5, -3.14), (2.0, 3.14)),
    ActionBounds((-1.0, -3.14), (2.5, 3.14)),
])
def test_bounds_wider_than_the_vendored_clip_are_refused(bounds) -> None:
    with pytest.raises(RuntimeError) as exc:
        PolicyRunner(action_bounds=bounds)
    assert "exceed the vendored actor's clip" in str(exc.value)
    assert str(bounds.low) in str(exc.value)
