"""Tests for rm_policy.action_contract: the action bounds a profile's
interface file declares, read without the vendored module (workstation tier).

The bounds matter twice per tick: the previous-action block of the
observation is normalized by them and the emitted action is clipped to them.
3.0.0 and 4.0.0 declare the numbers the vendored ncp_policy hardcodes; 5.0.0
(M9) declares the robot's clamps.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not installed")
yaml = pytest.importorskip("yaml")

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))
_CONFIG = os.path.join(_REPO_ROOT, "config")

from rm_policy.action_contract import (  # noqa: E402
    ActionBounds,
    action_bounds_from,
)

M9 = ActionBounds(low=(-0.25, -0.8), high=(0.25, 0.8))
VENDORED_NUMBERS = ActionBounds(low=(-1.0, -3.14), high=(2.0, 3.14))  # 3.14, not pi


def _cfg(filename: str) -> dict:
    with open(os.path.join(_CONFIG, filename), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _find_key(node, key):
    """First value of `key` anywhere in a parsed YAML tree."""
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for v in node.values():
            found = _find_key(v, key)
            if found is not None:
                return found
    return None


def test_m9_declares_the_robots_clamps() -> None:
    b = action_bounds_from(_cfg("interface_m9.yaml"), "interface_m9.yaml")
    assert b == M9


def test_m9_bounds_sit_inside_the_safety_clamps() -> None:
    """The trained range must not exceed what rm_safety passes, or M9 runs
    saturated like the others. The clamps may be wider; never narrower."""
    limits = _cfg("safety_limits.yaml")
    max_vx, max_wz = float(_find_key(limits, "max_vx")), float(_find_key(limits, "max_wz"))
    assert M9.within((-max_vx, -max_wz), (max_vx, max_wz)), (
        f"M9's trained bounds {M9} exceed rm_safety's clamps vx {max_vx} wz {max_wz}; "
        f"the policy would run saturated, which the 5.0.0 contract says it does not"
    )


@pytest.mark.parametrize("filename", ["interface.yaml", "interface_m4.yaml"])
def test_existing_profiles_declare_the_vendored_numbers(filename) -> None:
    """3.0.0 and 4.0.0 run on the vendored constants, so the files must say
    exactly those (3.14, literally): the automated pi_is_not_pi check."""
    assert action_bounds_from(_cfg(filename), filename) == VENDORED_NUMBERS


def test_the_vendored_constants_are_those_numbers() -> None:
    pytest.importorskip("torch", reason="torch not installed (workstation tier)")
    pytest.importorskip("ncps", reason="ncps not installed (workstation tier)")
    from rm_policy.vendor import ncp_policy
    assert tuple(float(v) for v in ncp_policy.ACTION_LOW) == VENDORED_NUMBERS.low
    assert tuple(float(v) for v in ncp_policy.ACTION_HIGH) == VENDORED_NUMBERS.high


def test_normalize_is_the_training_formula() -> None:
    assert M9.normalize((0.0, 0.0)) == pytest.approx((0.0, 0.0))
    assert M9.normalize((0.25, -0.8)) == pytest.approx((1.0, -1.0))
    assert M9.normalize((0.1, 0.4)) == pytest.approx((0.4, 0.5), abs=1e-6)
    # The 4.0.0 numbers: a zero action is NOT a zero vector (interface.yaml init_note).
    assert VENDORED_NUMBERS.normalize((0.0, 0.0)) == pytest.approx((-1.0 / 3.0, 0.0), abs=1e-6)
    assert M9.normalize((0.1, 0.4)).dtype == np.float32


def test_clip_and_within() -> None:
    assert M9.clip((0.3, -1.0)) == pytest.approx((0.25, -0.8))
    assert M9.clip((-0.1, 0.2)) == pytest.approx((-0.1, 0.2))
    assert M9.within(VENDORED_NUMBERS.low, VENDORED_NUMBERS.high)
    assert not ActionBounds((-1.0, -4.0), (2.0, 4.0)).within(VENDORED_NUMBERS.low, VENDORED_NUMBERS.high)
    assert not ActionBounds((-1.5, -3.14), (2.0, 3.14)).within(VENDORED_NUMBERS.low, VENDORED_NUMBERS.high)


def _action(vx, wz, pre_scaled=True):
    return {"action": {"pre_scaled": pre_scaled, "layout": [
        {"name": "cmd_vx", "range": vx}, {"name": "cmd_wz", "range": wz}]}}


@pytest.mark.parametrize("cfg, needle", [
    ({}, "action.layout"),
    ({"action": "none"}, "action.layout"),
    ({"action": {"layout": []}}, "pre_scaled"),
    (_action([-0.25, 0.25], [-0.8, 0.8], pre_scaled=False), "pre_scaled"),
    ({"action": {"pre_scaled": True, "layout": [{"name": "cmd_wz", "range": [-0.8, 0.8]}]}}, "'cmd_vx'"),
    ({"action": {"pre_scaled": True, "layout": [{"name": "cmd_vx", "range": [-0.25, 0.25]}]}}, "'cmd_wz'"),
    (_action([0.25, -0.25], [-0.8, 0.8]), "low < high"),
    (_action([-0.25, 0.25], [0.8, 0.8]), "low < high"),
    (_action([-0.25, math.inf], [-0.8, 0.8]), "finite"),
    (_action([-0.25, "fast"], [-0.8, 0.8]), "[low, high]"),
    (_action([0.25], [-0.8, 0.8]), "[low, high]"),
    (_action(None, [-0.8, 0.8]), "[low, high]"),
])
def test_malformed_blocks_are_refused_with_the_file_and_key_named(cfg, needle) -> None:
    with pytest.raises(ValueError) as exc:
        action_bounds_from(cfg, "interface_x.yaml")
    assert "interface_x.yaml" in str(exc.value)
    assert needle in str(exc.value)


def test_m9_observation_block_matches_the_4_0_0_one() -> None:
    """5.0.0 changes the action bounds and the previous-action text only:
    the lidar and goal blocks, the width and the control rate are 4.0.0's."""
    a, b = _cfg("interface.yaml"), _cfg("interface_m9.yaml")
    assert b["interface_version"] == "5.0.0"
    assert b["observation"]["total_dim"] == a["observation"]["total_dim"] == 724
    assert b["control"]["rate_hz"] == a["control"]["rate_hz"]
    by_name = lambda cfg: {blk["name"]: blk for blk in cfg["observation"]["layout"]}  # noqa: E731
    la, lb = by_name(a), by_name(b)
    assert set(la) == set(lb) == {"lidar_ranges", "goal_local", "previous_action"}
    assert lb["lidar_ranges"] == la["lidar_ranges"]
    assert lb["goal_local"] == la["goal_local"]
    assert lb["previous_action"]["shape"] == la["previous_action"]["shape"] == [2]
    assert b["checkpoint"]["file"].endswith("actor_m9.pt")
