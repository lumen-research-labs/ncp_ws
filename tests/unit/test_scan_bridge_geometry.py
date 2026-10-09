"""Guard: the scan bridge's SYNTHETIC LaserScan geometry must satisfy
upstream's own `policy.check_scan()`.

Why this test exists: `NCP-on-Rosmaster/rosmaster/node.py` refuses to build an
observation from any scan whose length is not `policy.BEAMS`, and warns on any
scan whose span/centre are off. `rm_policy.scan_bridge_node` exists purely to
manufacture a scan that satisfies it, so that upstream runs UNMODIFIED on a
TG30. If the bridge's geometry ever drifts out of upstream's tolerance the
robot silently stops driving (deadman) rather than erroring, which is exactly
the kind of failure that costs a hardware session -- so it is pinned here.

The geometry is recomputed rather than imported, because importing
scan_bridge_node pulls in rclpy, which is not present on the workstation tier.
Keep the two in sync; that is what the shared-constants assertions below are
for.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

# Same tier convention as test_observation_builder.py: the vendored module
# imports numpy/torch/ncps at module scope, so these assertions run on the
# ROBOT tier. scan_resampler itself is pure stdlib.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy import scan_resampler  # noqa: E402  (pure stdlib)

# The default profile's beam layout (m8v2). Duplicated as literals ONLY so
# the geometry assertions below run on the numpy-less workstation tier; the
# guarded test `test_literals_match_the_vendored_checkpoint_constants` proves
# they still equal ncp_policy's values wherever the vendored module is
# importable. The m4 profile's layout is exercised alongside where the
# geometry is profile-independent.
BEAMS = 720
FOV_DEG = 270.0
M4_BEAMS = 32
M4_FOV_DEG = 180.0


def _ncp_policy():
    """Import the vendored module or skip: it needs numpy/torch/ncps, which
    are absent on the workstation tier (same convention as
    test_observation_builder.py)."""
    pytest.importorskip("numpy", reason="numpy not installed (workstation tier)")
    pytest.importorskip("torch", reason="torch not installed (workstation tier)")
    pytest.importorskip("ncps", reason="ncps not installed (workstation tier)")
    from rm_policy.vendor import ncp_policy
    return ncp_policy


def test_literals_match_the_vendored_checkpoint_constants() -> None:
    ncp_policy = _ncp_policy()
    assert (BEAMS, FOV_DEG) == (ncp_policy.BEAMS, ncp_policy.FOV_DEG)


def _bridge_geometry(n_beams: int, arc_deg: float):
    """Mirror of scan_bridge_node's angle computation: BIN CENTRES."""
    bin_width = math.radians(arc_deg) / n_beams
    angle_min = -math.radians(arc_deg) / 2.0 + bin_width / 2.0
    return angle_min, bin_width


def test_bridge_geometry_passes_upstream_check_scan() -> None:
    ncp_policy = _ncp_policy()
    angle_min, angle_increment = _bridge_geometry(BEAMS, FOV_DEG)
    assert ncp_policy.check_scan(BEAMS, angle_min, angle_increment) == []


@pytest.mark.parametrize("beams,fov", [(BEAMS, FOV_DEG), (M4_BEAMS, M4_FOV_DEG)])
def test_bridge_geometry_is_centred_straight_ahead(beams, fov) -> None:
    angle_min, angle_increment = _bridge_geometry(beams, fov)
    centre = angle_min + angle_increment * (beams - 1) / 2.0
    assert abs(centre) < 1e-12, "forward beam must sit at 0 rad (REP-103)"


def test_upstream_refuses_the_m4_profile_bridge_output() -> None:
    """Why ncp.launch.py refuses consumer:=upstream with profile:=m4: the
    vendored module is fixed at 720 beams and check_scan says so."""
    ncp_policy = _ncp_policy()
    angle_min, angle_increment = _bridge_geometry(M4_BEAMS, M4_FOV_DEG)
    problems = ncp_policy.check_scan(M4_BEAMS, angle_min, angle_increment)
    assert any("expected 720 beams, got 32" in p for p in problems)


def test_bridge_output_is_not_safe_to_re_resample() -> None:
    """The bridge output spans 269.625 deg, so asking scan_resampler for a
    full 270 deg arc from it FAULTS. This is why ncp.launch.py points
    rm_policy's policy_node at the raw /scan and not at /ncp/scan."""
    angle_min, angle_increment = _bridge_geometry(BEAMS, FOV_DEG)
    result = scan_resampler.resample_forward_arc(
        [1.0] * BEAMS, angle_min, angle_increment,
        n_bins=BEAMS, arc_deg=FOV_DEG, range_min=0.01, range_max=30.0,
    )
    assert result.fault == "arc_exceeds_scan_coverage"
    assert result.bins is None


@pytest.mark.parametrize("beams,fov", [(BEAMS, FOV_DEG), (M4_BEAMS, M4_FOV_DEG)])
def test_raw_tg30_scan_reduces_to_exactly_the_trained_beam_count(beams, fov) -> None:
    """End-to-end shape check on a realistic TG30 geometry (360 deg, ~1982
    points, angle_min == -pi), including the 0.0-no-return sentinel."""
    n_points = 1982
    angle_increment = 2.0 * math.pi / n_points
    ranges = [0.0 if i % 3 == 0 else 2.5 for i in range(n_points)]  # ~33% dropout
    result = scan_resampler.resample_forward_arc(
        ranges, -math.pi, angle_increment,
        n_bins=beams, arc_deg=fov,
        range_min=0.01, range_max=30.0,
    )
    assert result.fault is None
    assert len(result.bins) == beams
    # No 0.0 survives: a no-return must never read as a zero-distance obstacle.
    # At ~2 rays per 0.375 deg bin, a bin whose only ray is a dropout reads
    # range_max under strict sampling; nothing in between is possible.
    assert all(r in (2.5, 30.0) for r in result.bins)
    # With the 4.0.0 search window every such bin finds a real neighbour.
    windowed = scan_resampler.resample_forward_arc(
        ranges, -math.pi, angle_increment,
        n_bins=beams, arc_deg=fov,
        range_min=0.01, range_max=30.0, search_window_bins=2,
    )
    assert windowed.fault is None
    assert all(r == 2.5 for r in windowed.bins)
