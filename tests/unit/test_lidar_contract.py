"""Guard: the three places that state the lidar contract agree.

config/interface.yaml (the authority), policy_node's version pin, and the
vendored checkpoint module's BEAMS / FOV_DEG must describe the same vector,
because a disagreement produces no runtime error -- just wrong numbers.

Pure stdlib + yaml, so it runs on the workstation tier. The vendored-constants
check needs torch and skips where that is absent.
"""

from __future__ import annotations

import os
import re
import sys

import pytest
import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.lidar_contract import LidarContract, lidar_contract_from  # noqa: E402

_INTERFACE = os.path.join(_REPO_ROOT, "config", "interface.yaml")
_POLICY_NODE = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "policy_node.py")


def _cfg() -> dict:
    with open(_INTERFACE, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_interface_yaml_lidar_block_is_the_4_0_0_contract() -> None:
    c = lidar_contract_from(_cfg())
    assert c == LidarContract(
        n_beams=720, arc_deg=270.0, range_min_m=0.01, range_max_m=30.0,
        search_window_bins=2, mask_zones_deg=((92.0, 123.0), (-135.0, -97.0)),
    )
    assert _cfg()["observation"]["total_dim"] == 720 + 2 + 2


def test_policy_node_version_pin_matches_the_yaml() -> None:
    with open(_POLICY_NODE, "r", encoding="utf-8") as fh:
        src = fh.read()
    pin = re.search(r'_EXPECTED_INTERFACE_VERSION: str = "([^"]+)"', src).group(1)
    assert pin == _cfg()["interface_version"] == "4.0.0"


def test_yaml_agrees_with_the_vendored_checkpoint_constants() -> None:
    pytest.importorskip("numpy", reason="numpy not installed (workstation tier)")
    pytest.importorskip("torch", reason="torch not installed (workstation tier)")
    pytest.importorskip("ncps", reason="ncps not installed (workstation tier)")
    from rm_policy.vendor import ncp_policy
    c = lidar_contract_from(_cfg())
    assert (c.n_beams, c.arc_deg) == (ncp_policy.BEAMS, ncp_policy.FOV_DEG)
    assert _cfg()["observation"]["total_dim"] == ncp_policy.OBS_DIM


def test_checkpoint_provenance_is_recorded() -> None:
    ck = _cfg()["checkpoint"]
    assert re.fullmatch(r"[0-9a-f]{64}", ck["sha256"])
    assert ck["snapshot"] == "s1436792"


@pytest.mark.parametrize(
    "cfg",
    [
        {},
        {"observation": {}},
        {"observation": {"layout": []}},
        {"observation": {"layout": [{"name": "lidar_ranges"}]}},
        {"observation": {"layout": [{"name": "lidar_ranges", "shape": [720, 1],
                                     "arc_deg": 270, "range_min_m": 0.01, "range_max_m": 30}]}},
        {"observation": {"layout": [{"name": "lidar_ranges", "shape": [720],
                                     "arc_deg": "wide", "range_min_m": 0.01, "range_max_m": 30}]}},
    ],
)
def test_malformed_contract_raises_rather_than_defaulting(cfg) -> None:
    with pytest.raises(ValueError):
        lidar_contract_from(cfg)
