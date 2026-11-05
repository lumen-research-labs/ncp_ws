"""Guard: the places that state a lidar contract agree, for every profile.

Each profile's contract file (the authority), policy_node's supported
versions, the launch's profile table and -- for the default profile -- the
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

_CONFIG = os.path.join(_REPO_ROOT, "config")
_POLICY_NODE = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "policy_node.py")
_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "ncp.launch.py")

# The policy profiles (ncp.launch.py _PROFILES): contract file -> what it
# must say. Width is lidar + goal (2) + previous action (2).
PROFILES = {
    "m8v2": ("interface.yaml", LidarContract(
        interface_version="4.0.0", n_beams=720, arc_deg=270.0, range_min_m=0.01,
        range_max_m=30.0, search_window_bins=2,
        mask_zones_deg=((92.0, 123.0), (-135.0, -97.0)), body_floor_m=0.12)),
    "m4": ("interface_m4.yaml", LidarContract(
        interface_version="3.0.0", n_beams=32, arc_deg=180.0, range_min_m=0.01,
        range_max_m=30.0, search_window_bins=0, mask_zones_deg=(), body_floor_m=0.12)),
    # M9: the 4.0.0 sensor with the action bounds at the robot's clamps
    # (tests/unit/test_action_contract.py pins the action block).
    "m9": ("interface_m9.yaml", LidarContract(
        interface_version="5.0.0", n_beams=720, arc_deg=270.0, range_min_m=0.01,
        range_max_m=30.0, search_window_bins=2,
        mask_zones_deg=((92.0, 123.0), (-135.0, -97.0)), body_floor_m=0.12)),
}


def _cfg(filename: str = "interface.yaml") -> dict:
    with open(os.path.join(_CONFIG, filename), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@pytest.mark.parametrize("profile", sorted(PROFILES))
def test_each_profile_file_is_its_contract(profile) -> None:
    filename, expected = PROFILES[profile]
    cfg = _cfg(filename)
    c = lidar_contract_from(cfg)
    assert c == expected
    assert cfg["observation"]["total_dim"] == c.obs_dim == expected.n_beams + 4


def test_policy_node_supports_exactly_the_profile_versions() -> None:
    with open(_POLICY_NODE, "r", encoding="utf-8") as fh:
        src = fh.read()
    supported = re.search(r"_SUPPORTED_INTERFACE_VERSIONS = \(([^)]*)\)", src).group(1)
    supported = set(re.findall(r'"([^"]+)"', supported))
    assert supported == {c.interface_version for _, c in PROFILES.values()}


def test_launch_profile_table_names_existing_files() -> None:
    with open(_LAUNCH, "r", encoding="utf-8") as fh:
        src = fh.read()
    table = src[src.index("_PROFILES = {"):src.index("}", src.index("_PROFILES = {"))]
    rows = dict(re.findall(r'"(\w+)": \("([^"]+)", "[^"]+"\)', table))
    assert rows == {k: v[0] for k, v in PROFILES.items()}
    for filename in rows.values():
        assert os.path.isfile(os.path.join(_CONFIG, filename)), filename


def test_default_profile_agrees_with_the_vendored_checkpoint_constants() -> None:
    """Only the default profile has to match the vendored module: the upstream
    consumer runs on it, and the launch refuses consumer:=upstream elsewhere."""
    pytest.importorskip("numpy", reason="numpy not installed (workstation tier)")
    pytest.importorskip("torch", reason="torch not installed (workstation tier)")
    pytest.importorskip("ncps", reason="ncps not installed (workstation tier)")
    from rm_policy.vendor import ncp_policy
    c = lidar_contract_from(_cfg("interface.yaml"))
    assert (c.n_beams, c.arc_deg) == (ncp_policy.BEAMS, ncp_policy.FOV_DEG)
    assert c.obs_dim == ncp_policy.OBS_DIM


@pytest.mark.parametrize("profile", sorted(PROFILES))
def test_checkpoint_provenance_is_recorded(profile) -> None:
    ck = _cfg(PROFILES[profile][0])["checkpoint"]
    assert re.fullmatch(r"[0-9a-f]{64}", ck["sha256"])
    assert ck["file"].endswith(".pt")


@pytest.mark.parametrize(
    "cfg",
    [
        {},
        {"observation": {}},
        {"observation": {"layout": [{"name": "lidar_ranges", "shape": [720],
                                     "arc_deg": 270, "range_min_m": 0.01, "range_max_m": 30}]}},  # no version
        {"interface_version": "4.0.0", "observation": {"layout": []}},
        {"interface_version": "4.0.0", "observation": {"layout": [{"name": "lidar_ranges"}]}},
        {"interface_version": "4.0.0", "observation": {"layout": [{"name": "lidar_ranges", "shape": [720, 1],
                                     "arc_deg": 270, "range_min_m": 0.01, "range_max_m": 30}]}},
        {"interface_version": "4.0.0", "observation": {"layout": [{"name": "lidar_ranges", "shape": [720],
                                     "arc_deg": "wide", "range_min_m": 0.01, "range_max_m": 30}]}},
    ],
)
def test_malformed_contract_raises_rather_than_defaulting(cfg) -> None:
    with pytest.raises(ValueError):
        lidar_contract_from(cfg)
