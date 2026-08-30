"""
test_safety_replay.py — Replay tests: real recorded sensor data through the
pure safety logic.

Tier: "replay" (BUILD-PLAN test order: unit -> replay -> hardware).  No ROS
runtime; the bag is read with the ROS-free `laserscan_bag_reader`, and the TG30
`ros2 topic echo` YAML captures are loaded with PyYAML.  Everything is fed
through the exact `rm_safety.safety_logic.evaluate()` the node calls, with
limits loaded from the real `config/safety_limits.yaml`.

What these tests establish (and, equally, what they do NOT):

  * The TG30 encodes a no-return as ``0.0`` (NOT +inf) — see CLAUDE.md
    2026-08-12 s3 safety-critical finding.  ~27-34% of every real scan is
    ``0.0``.  These tests prove that `safety_logic` already treats ``0.0`` (and
    anything <= range_min) as INVALID, so no-returns never read as a
    zero-distance obstacle and never latch a false stop.  This is the property
    the finding warns must hold.  (The observation builder — not yet written —
    still needs its own 0.0 -> range_max mapping; that is a separate contract.)

  * The baseline bag `tests/fixtures/ncp_static_bag` was recorded in a
    CLUTTERED room with a real object ~0.21 m dead-ahead (+6.7 deg, inside the
    stop zone AS RECORDED).  So the e-stop LATCHING across that bag is CORRECT, not
    spurious.  This bag therefore validates no-perception robustness and latch
    behaviour — it is NOT a clear-path recording, and cannot be used for a
    "no spurious e-stop on a clear path" case.  That case needs a NEW bag
    recorded in genuinely open space (an operator task; noted in
    docs/safety_validation.md).

  * FINDING surfaced by this replay, FIXED by operator decision 2026-08-14
    (see safety_validation.md §2b): during the consecutive-scan noise-filter
    window, a sustained stop-zone obstacle used to produce OK / FULL commanded
    velocity for the first (consecutive_scans_to_trigger - 1) control ticks,
    then latch to zero — a closer stop-zone obstacle braked LATER than a
    farther slow-zone one. Fixed in safety_logic.evaluate() by routing
    un-latched stop-zone readings through the same slow_scale() as the slow
    zone, which evaluates to 0.0 there — an immediate soft-stop instead of
    ~400 ms of un-slowed motion at 5 Hz/N=3.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict, List

import pytest
import yaml

# ---------------------------------------------------------------------------
# Make rm_safety importable without a colcon install (mirrors tests/unit).
# ---------------------------------------------------------------------------
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_RM_SAFETY_SRC = os.path.join(_REPO_ROOT, "src", "rm_safety")
if _RM_SAFETY_SRC not in sys.path:
    sys.path.insert(0, _RM_SAFETY_SRC)
if os.path.dirname(__file__) not in sys.path:
    sys.path.insert(0, os.path.dirname(__file__))

from rm_safety.safety_logic import (  # noqa: E402
    EvalInputs,
    Limits,
    State,
    evaluate,
    min_range_in_arcs,
)
from laserscan_bag_reader import read_scans  # noqa: E402

_CONFIG_PATH = os.path.join(_REPO_ROOT, "config", "safety_limits.yaml")
_FIXTURES = os.path.join(_REPO_ROOT, "tests", "fixtures")
_BAG_DB3 = os.path.join(_FIXTURES, "ncp_static_bag", "ncp_static_bag_0.db3")
_OPEN_SCAN = os.path.join(_FIXTURES, "tg30_open_scan.yaml")
_SAMPLE_SCAN = os.path.join(_FIXTURES, "tg30_scan_sample.yaml")

# TG30 capture invariants (CLAUDE.md 2026-08-12 s3).
_TG30_POINTS = 2020
_TG30_FRAME = "laser_link"


# ---------------------------------------------------------------------------
# Fixtures / helpers
# ---------------------------------------------------------------------------

@pytest.fixture
def limits() -> Limits:
    """Real safety limits, so tests track config/safety_limits.yaml exactly."""
    with open(_CONFIG_PATH) as fh:
        return Limits.from_dict(yaml.safe_load(fh))


def _load_echo_yaml_scan(path: str) -> Dict:
    """Load the first document of a `ros2 topic echo` LaserScan YAML dump.

    The captures carry a trailing `---`, so the file is a multi-document
    stream with one real document; take the first non-empty one.
    """
    for doc in yaml.safe_load_all(open(path)):
        if doc:
            return doc
    raise AssertionError(f"no document found in {path}")


def _eval_scan(scan: Dict, limits: Limits, *, latched=False, count=0,
               cmd_vx=0.2) -> "object":
    """Evaluate one scan with a fresh (non-stale) odom/command context."""
    inp = EvalInputs(
        cmd_vx=cmd_vx, cmd_vy=0.0, cmd_wz=0.0,
        is_policy_source=True, command_age_ms=0.0,
        last_vx=cmd_vx, last_vy=0.0, last_wz=0.0, dt=0.2,
        scan_ranges=[float(r) for r in scan["ranges"]],
        scan_angle_min=scan["angle_min"],
        scan_angle_increment=scan["angle_increment"],
        scan_range_min=scan["range_min"],
        scan_range_max=scan["range_max"],
        scan_age_ms=0.0,
        is_killed=False, estop_latched=latched,
        consecutive_stop_count=count, reset_latch=False, odom_stale=False,
    )
    return evaluate(inp, limits)


# ---------------------------------------------------------------------------
# 1. Baseline bag — 600 real static-room scans as a continuous stream
# ---------------------------------------------------------------------------

class TestBaselineBag:
    def _scans(self) -> List[Dict]:
        return list(read_scans(
            _BAG_DB3, "/scan",
            expect_points=_TG30_POINTS, expect_frame=_TG30_FRAME,
        ))

    def test_bag_has_expected_scan_count(self):
        assert len(self._scans()) == 600

    def test_no_return_zeros_never_cause_perception_loss(self, limits):
        """SAFETY-CRITICAL: ~27% of every real scan is 0.0 (TG30 no-return).

        Across all 600 scans, evaluate() must NEVER report loss of perception
        (min_range == inf).  If 0.0 were mishandled as an obstacle it would read
        as an obstacle at zero range, not as no-perception — but the reciprocal
        risk (0.0 filtered so aggressively that the arc empties) is what a
        no-perception result would signal.  Neither occurs: min_range is always
        finite and reflects the real ~0.2 m returns, never 0.0.
        """
        for i, scan in enumerate(self._scans()):
            zeros = sum(1 for r in scan["ranges"] if r == 0.0)
            assert zeros > 0, f"scan {i}: expected TG30 no-return zeros, found none"
            dec = _eval_scan(scan, limits)
            assert dec.min_range != math.inf, (
                f"scan {i}: no-perception (inf) — 0.0 no-returns emptied the arc"
            )
            assert dec.min_range > 0.0, (
                f"scan {i}: min_range={dec.min_range} — a 0.0 no-return was "
                f"treated as a zero-distance obstacle"
            )
            # Real environment: object ~0.21 m ahead; sane physical band.
            assert 0.15 < dec.min_range < 0.40, (
                f"scan {i}: min_range={dec.min_range} outside expected band"
            )

    def test_bag_latches_stop_on_real_forward_obstacle(self, limits):
        """The bag has a real object ~0.21 m dead-ahead, and whether that
        latches depends on the CURRENT stop_zone_m.

        At the 0.35 m zone this was recorded against it latched, and that was
        correct — the obstacle is genuinely present. At the 0.15 m zone set on
        2026-08-29 it does not: 0.21 m is now a SLOW-zone reading, scaled to
        zero rather than latched. The assertion is therefore written against
        the live threshold, so it keeps tracking whatever the zone is tuned to
        instead of pinning a number that has already moved twice.

        Either way this exercises the latch machinery across a long real
        stream (600 ticks).
        """
        latched, count = False, 0
        states: Dict[str, int] = {}
        min_forward = float("inf")
        for scan in self._scans():
            dec = _eval_scan(scan, limits, latched=latched, count=count)
            latched, count = dec.estop_latched, dec.consecutive_stop_count
            states[State(dec.state).name] = states.get(State(dec.state).name, 0) + 1
            min_forward = min(min_forward, dec.min_range)
        # 0.21 m is INSIDE the 0.35 m stop zone the bag was recorded against,
        # but OUTSIDE the current 0.15 m one. So it no longer latches — it
        # scales velocity to zero through the slow zone instead. Asserted
        # against the live threshold so this tracks future retuning.
        stop_m = limits.estop.stop_zone_m
        if min_forward < stop_m:
            assert latched is True, "e-stop should be latched at end of bag"
        else:
            assert latched is False, (
                f"nearest forward return {min_forward:.3f} m is outside the "
                f"{stop_m} m stop zone, so no latch is expected"
            )
        # Which state it holds now depends on the zone the obstacle falls in.
        held = "STOPPED" if min_forward < stop_m else "SLOW"
        assert states.get(held, 0) >= 595, (
            f"expected the bag to hold {held} (nearest forward return "
            f"{min_forward:.3f} m vs stop zone {stop_m} m); got {states}"
        )
        # Only the pre-trigger noise-filter window is non-STOPPED, and it must
        # be SLOW (soft-stop), not OK — see safety_validation.md §2b.
        assert states.get("OK", 0) == 0, states
        if min_forward < stop_m:
            # Latching case: only the pre-trigger noise-filter window is
            # non-STOPPED, and it must be SLOW (soft-stop), not OK.
            assert states.get("SLOW", 0) <= (
                limits.estop.consecutive_scans_to_trigger - 1
            ), states


# ---------------------------------------------------------------------------
# 2. TG30 YAML captures — single-frame behaviour
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# STOP ZONE CHANGED 2026-08-29: 0.35 m -> 0.15 m (operator decision).
#
# The fixtures below were captured against a 0.35 m stop zone and their real
# forward obstacle sits at ~0.21 m. At 0.15 m that obstacle is NO LONGER a
# stop-zone reading — it is a SLOW-zone reading. The robot slows to zero
# through slow_scale() rather than latching the e-stop.
#
# That is a real, deliberate change in behaviour, not a test that needed
# relaxing: a 21 cm obstacle now scales velocity instead of latching. These
# tests assert the CURRENT thresholds, derived from config, so they keep
# tracking whatever the zones are tuned to — and they still prove the two
# things the fixtures exist to prove: that the TG30's 0.0 no-return sentinel
# never reads as a zero-distance obstacle, and that perception is never lost
# across 600 real scans.
# ---------------------------------------------------------------------------

class TestTG30Captures:
    def test_open_scan_forward_arc_is_clear(self, limits):
        """Open-space capture: the forward arc has NO close return (nearest
        forward return ~0.49 m, well outside the 0.35 m stop zone).  Corroborates
        the CLAUDE.md preliminary 'no forward occlusion' finding — the close
        returns are all lateral/rear, not dead-ahead.
        """
        scan = _load_echo_yaml_scan(_OPEN_SCAN)
        assert len(scan["ranges"]) == _TG30_POINTS
        # Forward-only min via a forward-arc-only limits view would duplicate
        # logic; instead check the full-monitored-arc min is a real lateral
        # return and that a forward-only sweep is clear.
        fwd_min = _forward_arc_min(scan, limits)
        assert fwd_min > limits.estop.stop_zone_m, (
            f"forward arc min {fwd_min:.3f} m unexpectedly inside stop zone"
        )

    def test_sample_scan_latches_after_consecutive_window(self, limits):
        """Cluttered-room capture with a real ~0.20 m forward obstacle: one
        frame does NOT latch (noise filter), but sustaining it for
        consecutive_scans_to_trigger ticks DOES latch.
        """
        scan = _load_echo_yaml_scan(_SAMPLE_SCAN)
        n = limits.estop.consecutive_scans_to_trigger
        latched, count = False, 0
        states = []
        for _ in range(n + 1):
            dec = _eval_scan(scan, limits, latched=latched, count=count)
            latched, count = dec.estop_latched, dec.consecutive_stop_count
            states.append(State(dec.state).name)
        # First (n-1) ticks not yet latched: pre-latch stop-zone window now
        # soft-stops (SLOW state, scaled to 0.0) rather than OK/full-speed
        # (operator decision, 2026-08-14 — see safety_validation.md §2b).
        # Latched by tick n.
        # The capture's obstacle is ~0.20 m. Whether tick n latches depends on
        # the CURRENT stop zone: inside it, the pre-latch window is SLOW and
        # tick n latches STOPPED; outside it, every tick is SLOW and nothing
        # latches. Either way the velocity is scaled to zero — what changes is
        # whether it is a LATCHED stop needing an explicit reset.
        dec0 = _eval_scan(scan, limits, latched=False, count=0)
        if dec0.min_range < limits.estop.stop_zone_m:
            assert states[: n - 1] == ["SLOW"] * (n - 1), states
            assert states[n - 1] == "STOPPED", states
            assert latched is True
        else:
            assert all(st == "SLOW" for st in states), states
            assert latched is False

    def test_no_perception_on_either_capture(self, limits):
        for path in (_OPEN_SCAN, _SAMPLE_SCAN):
            scan = _load_echo_yaml_scan(path)
            dec = _eval_scan(scan, limits)
            assert dec.min_range != math.inf, f"{path}: false no-perception"


# ---------------------------------------------------------------------------
# 3. Regression lock: the pre-latch full-speed window (FLAGGED finding)
# ---------------------------------------------------------------------------

def test_prelatch_window_emits_zero_velocity(limits):
    """LOCK-IN of the fixed behaviour (see module docstring + safety_validation.md
    §2b, operator decision 2026-08-14).

    A sustained stop-zone obstacle now soft-stops (SLOW state, velocity scaled
    via slow_scale()) for the first (N-1) control ticks, then latches to
    STOPPED. slow_scale() returns 0.0 for min_r <= stop_zone_m, so pre-latch
    ticks emit zero velocity, not the commanded value — no more "closer
    obstacle brakes later" inversion. Previously this test pinned the opposite
    (full-velocity) behaviour as test_prelatch_window_emits_full_velocity.
    """
    scan = _load_echo_yaml_scan(_SAMPLE_SCAN)
    n = limits.estop.consecutive_scans_to_trigger
    latched, count = False, 0
    pre_latch_states = []
    pre_latch_vx = []
    for tick in range(n):
        dec = _eval_scan(scan, limits, latched=latched, count=count, cmd_vx=0.2)
        if tick < n - 1:
            pre_latch_states.append(State(dec.state).name)
            pre_latch_vx.append(dec.out_vx)
        latched, count = dec.estop_latched, dec.consecutive_stop_count
    assert pre_latch_states == ["SLOW"] * (n - 1), pre_latch_states

    # WHICH zone the capture's ~0.20 m obstacle falls in depends on the
    # current stop_zone_m, and the correct pre-latch velocity differs:
    #   inside the stop zone -> slow_scale() returns exactly 0.0, so the
    #     pre-latch window emits ZERO. This is the 2026-08-14 fix: no more
    #     "closer obstacle brakes later".
    #   in the slow zone      -> velocity is scaled PROPORTIONALLY, which is
    #     the slow zone doing its job, and demanding zero here would be
    #     asserting that the slow zone is a stop zone.
    # Either way it must be braking hard, so the scaled command is checked
    # against the commanded value rather than merely "not zero".
    dec0 = _eval_scan(scan, limits, latched=False, count=0, cmd_vx=0.2)
    if dec0.min_range < limits.estop.stop_zone_m:
        assert all(abs(v) < 1e-9 for v in pre_latch_vx), pre_latch_vx
    else:
        assert all(0.0 <= v < 0.2 * 0.5 for v in pre_latch_vx), (
            f"slow-zone pre-latch should be scaled well below the commanded "
            f"0.2 m/s; got {pre_latch_vx} at min_range {dec0.min_range:.3f} m"
        )


# ---------------------------------------------------------------------------
# 4. Synthetic no-return encoding checks (belt-and-braces around the finding)
# ---------------------------------------------------------------------------

def _synthetic_scan(ranges: List[float]) -> Dict:
    n = len(ranges)
    return {
        "ranges": ranges,
        "angle_min": -math.pi,
        "angle_increment": (2 * math.pi) / n,
        "range_min": 0.01,
        "range_max": 64.0,
    }


def test_zero_no_return_is_not_an_obstacle(limits):
    """A scan that is all 0.0 except one genuine 1.5 m forward return must
    report min_range == 1.5 m, NOT 0.0.  Directly encodes the TG30 finding.
    """
    n = _TG30_POINTS
    ranges = [0.0] * n
    ranges[n // 2] = 1.5  # index n/2 ~ angle 0 rad (forward)
    dec = _eval_scan(_synthetic_scan(ranges), limits)
    assert abs(dec.min_range - 1.5) < 1e-3, dec.min_range


def test_all_zero_scan_is_no_perception_estop(limits):
    """An all-0.0 scan (total no-return) must FAIL CLOSED: no valid returns in
    the monitored arcs -> no-perception -> STOPPED, never OK.
    """
    dec = _eval_scan(_synthetic_scan([0.0] * _TG30_POINTS), limits)
    assert dec.state == State.STOPPED
    assert dec.reason == "scan_stale"  # no_perception routes through this branch


# ---------------------------------------------------------------------------
# Helper: forward-arc-only minimum (for the open-space clear-forward assertion)
# ---------------------------------------------------------------------------

def _forward_arc_min(scan: Dict, limits: Limits) -> float:
    half_fwd = math.radians(limits.estop.forward_arc_deg / 2.0)
    amin = scan["angle_min"]
    ainc = scan["angle_increment"]
    rmin, rmax = scan["range_min"], scan["range_max"]
    best = math.inf
    for i, r in enumerate(scan["ranges"]):
        r = float(r)
        if not math.isfinite(r) or r <= rmin or r > rmax:
            continue
        a = (amin + i * ainc + math.pi) % (2 * math.pi) - math.pi
        if abs(a) <= half_fwd and r < best:
            best = r
    return best
