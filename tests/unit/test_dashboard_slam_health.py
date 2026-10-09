"""Tests for rm_dashboard.slam_health — the map -> odom correction summary.

The value of this panel is that it distinguishes three states that look
identical from the outside: SLAM working, SLAM running but contributing
nothing, and SLAM silently dead. On 2026-08-29 the third of those cost a
session — slam_toolbox rejected every scan without erroring, and the symptom
surfaced several layers away as a fail-closed `pose_*` stop. So the tests
below are mostly about the summary telling those three apart honestly,
including saying "I do not know" where it does not.

Pure stdlib: workstation tier, no ROS and no browser.
"""

from __future__ import annotations

import math
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_dashboard"))

from rm_dashboard.slam_health import (  # noqa: E402
    CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN, OCCUPIED_THRESHOLD,
    SlamHealth, pack_grid, summarize_map,
)


def test_no_samples_reports_unavailable_not_zero() -> None:
    """Absence must not render as a correction of 0.0 m.

    Zero is a claim that SLAM is running and correcting nothing, which is a
    real and different (bad) state. Never having heard from it is not that.
    """
    snap = SlamHealth().snapshot(now=100.0, max_age_s=3.0)
    assert snap["available"] is False
    assert snap["correction_m"] is None
    assert snap["stale"] is True
    assert snap["jump_count"] == 0


def test_first_sample_is_not_counted_as_a_jump() -> None:
    """SLAM starting up is not a loop closure.

    Otherwise every session opens with a spurious event at the head of the
    history, and the count stops meaning what it says.
    """
    health = SlamHealth()
    assert health.update(1.0, 2.0, 0.5, now=10.0) is None
    assert health.snapshot(10.0, 3.0)["jump_count"] == 0


def test_continuous_creep_is_not_a_jump() -> None:
    """Scan matching trimming millimetres per update is normal, not an event."""
    health = SlamHealth()
    health.update(0.0, 0.0, 0.0, now=0.0)
    for i in range(1, 40):
        assert health.update(i * 0.002, 0.0, 0.0, now=float(i)) is None
    assert health.snapshot(40.0, 3.0)["jump_count"] == 0


def test_a_step_is_reported_as_a_jump() -> None:
    health = SlamHealth()
    health.update(0.0, 0.0, 0.0, now=0.0)
    jump = health.update(0.4, 0.0, 0.0, now=1.0)
    assert jump is not None
    assert jump["step_m"] == 0.4
    snap = health.snapshot(1.0, 3.0)
    assert snap["jump_count"] == 1
    assert snap["jumps"][-1]["step_m"] == 0.4


def test_a_yaw_only_correction_counts_as_a_jump() -> None:
    """A relocalization can rotate the estimate without translating it."""
    health = SlamHealth()
    health.update(0.0, 0.0, 0.0, now=0.0)
    assert health.update(0.0, 0.0, 0.5, now=1.0) is not None


def test_yaw_wrap_is_not_a_six_radian_jump() -> None:
    """A correction straddling +-pi must not read as an enormous rotation.

    Unwrapped, the jump count becomes a function of which way the robot is
    pointing rather than of what SLAM did.
    """
    health = SlamHealth()
    health.update(0.0, 0.0, math.pi - 0.01, now=0.0)
    assert health.update(0.0, 0.0, -math.pi + 0.01, now=1.0) is None
    assert health.snapshot(1.0, 3.0)["jump_count"] == 0


def test_nonfinite_samples_are_discarded_not_stored() -> None:
    """A NaN must not poison the magnitude, the maximum or the next delta."""
    health = SlamHealth()
    health.update(0.3, 0.4, 0.0, now=0.0)          # correction 0.5
    health.update(float("nan"), 0.0, 0.0, now=1.0)
    health.update(float("inf"), 0.0, 0.0, now=2.0)
    snap = health.snapshot(2.0, 3.0)
    assert snap["correction_m"] == 0.5
    assert snap["max_correction_m"] == 0.5
    assert snap["jump_count"] == 0


def test_staleness_is_reported_against_the_supplied_age() -> None:
    health = SlamHealth()
    health.update(0.1, 0.0, 0.0, now=10.0)
    assert health.snapshot(11.0, max_age_s=3.0)["stale"] is False
    assert health.snapshot(20.0, max_age_s=3.0)["stale"] is True


def test_max_correction_is_a_high_water_mark() -> None:
    """The peak must survive the correction shrinking again after a closure."""
    health = SlamHealth()
    health.update(0.0, 0.0, 0.0, now=0.0)
    health.update(0.9, 0.0, 0.0, now=1.0)
    health.update(0.05, 0.0, 0.0, now=2.0)
    snap = health.snapshot(2.0, 3.0)
    assert snap["correction_m"] == 0.05
    assert snap["max_correction_m"] == 0.9


def test_trend_needs_two_samples() -> None:
    """One sample is not a trend, and 0.0 would be a claim that it is flat."""
    health = SlamHealth()
    health.update(0.2, 0.0, 0.0, now=0.0)
    assert health.snapshot(0.0, 3.0)["trend_m"] is None
    health.update(0.3, 0.0, 0.0, now=1.0)
    assert health.snapshot(1.0, 3.0)["trend_m"] == 0.1


def test_a_running_slam_that_corrects_nothing_is_visible() -> None:
    """The 'running but not working' state must be distinguishable.

    Available and fresh, but a flat zero correction across a long window — the
    signature of scan matching contributing nothing, which on this robot is
    wrong, because /odom is known to over-read by ~5%.
    """
    health = SlamHealth()
    for i in range(50):
        health.update(0.0, 0.0, 0.0, now=float(i))
    snap = health.snapshot(49.0, 3.0)
    assert snap["available"] is True
    assert snap["stale"] is False
    assert snap["correction_m"] == 0.0
    assert snap["trend_m"] == 0.0


def test_samples_are_capped_so_a_long_session_does_not_grow_the_payload() -> None:
    health = SlamHealth()
    for i in range(5000):
        health.update(i * 0.0001, 0.0, 0.0, now=float(i))
    snap = health.snapshot(5000.0, 3.0)
    assert len(snap["samples"]) <= 60
    assert len(snap["jumps"]) <= 12


# ---------------------------------------------------------------------------
# Map summary
# ---------------------------------------------------------------------------


def test_map_summary_reports_coverage() -> None:
    s = summarize_map(width=100, height=200, resolution=0.05,
                      known_cells=4000, occupied_cells=250)
    assert s["cells"] == 20000
    assert s["known_fraction"] == 0.2
    assert s["known_area_m2"] == 10.0
    assert s["occupied_cells"] == 250


def test_map_summary_survives_an_empty_grid() -> None:
    """A dashboard must not raise on a degenerate map; it must render it."""
    s = summarize_map(0, 0, 0.0, 0, 0)
    assert s["cells"] == 0
    assert s["known_fraction"] == 0.0
    assert s["known_area_m2"] == 0.0


# ---------------------------------------------------------------------------
# Grid packing. The decimation rule is a correctness property, not cosmetics.
# ---------------------------------------------------------------------------


def _unpack(payload):
    import base64
    return list(base64.b64decode(payload["data"]))


def test_a_small_grid_is_passed_through_undecimated() -> None:
    grid = [-1, 0, 100, 0]
    p = pack_grid(grid, width=2, height=2, max_cells=400)
    assert p["step"] == 1
    assert (p["width"], p["height"]) == (2, 2)
    assert _unpack(p) == [CELL_UNKNOWN, CELL_FREE, CELL_OCCUPIED, CELL_FREE]


def test_decimation_keeps_walls_rather_than_averaging_them_away() -> None:
    """THE load-bearing property of this function.

    A wall is one or two cells thick at 0.05 m resolution. Averaging or
    point-sampling deletes it at any real decimation factor, and a map picture
    that has lost its walls looks like a room the robot can drive through —
    which is exactly the wrong error for a picture someone is about to drive
    by. One occupied cell in a block must survive as an occupied block.
    """
    # 8x8 of free space with a single occupied cell, decimated 4x.
    grid = [0] * 64
    grid[5 * 8 + 3] = 100
    p = pack_grid(grid, width=8, height=8, max_cells=2)
    assert p["step"] == 4
    assert (p["width"], p["height"]) == (2, 2)
    cells = _unpack(p)
    assert CELL_OCCUPIED in cells, "the wall was decimated away"
    assert cells.count(CELL_OCCUPIED) == 1
    # row 5 -> output row 1, col 3 -> output col 0
    assert cells[1 * 2 + 0] == CELL_OCCUPIED


def test_an_all_unknown_block_stays_unknown() -> None:
    """Unknown must not be promoted to free.

    Free means "scanned and empty". Rendering unexplored space as free would
    invite an operator to click a waypoint into a region SLAM has never seen.
    """
    p = pack_grid([-1] * 16, width=4, height=4, max_cells=2)
    assert set(_unpack(p)) == {CELL_UNKNOWN}


def test_a_block_mixing_unknown_and_free_reads_free() -> None:
    grid = [-1, -1, -1, 0]
    p = pack_grid(grid, width=2, height=2, max_cells=1)
    assert _unpack(p) == [CELL_FREE]


def test_low_probability_cells_are_free_not_occupied() -> None:
    """Below the occupied threshold is free, so noise does not paint walls."""
    p = pack_grid([OCCUPIED_THRESHOLD - 1], width=1, height=1, max_cells=400)
    assert _unpack(p) == [CELL_FREE]
    p = pack_grid([OCCUPIED_THRESHOLD], width=1, height=1, max_cells=400)
    assert _unpack(p) == [CELL_OCCUPIED]


def test_the_output_never_exceeds_the_cell_budget() -> None:
    """The budget is what keeps a big map off a small robot's network."""
    p = pack_grid([0] * (1000 * 800), width=1000, height=800, max_cells=200)
    assert max(p["width"], p["height"]) <= 200
    assert len(_unpack(p)) == p["width"] * p["height"]


def test_a_ragged_final_block_does_not_lose_or_invent_cells() -> None:
    """A grid whose size is not a multiple of the step must still round-trip.

    The last row and column are partial; getting this wrong shifts every
    marker on the map relative to the picture, which would put a clicked
    waypoint somewhere other than where it was clicked.
    """
    p = pack_grid([0] * (7 * 5), width=7, height=5, max_cells=3)
    assert p["step"] == 3
    assert (p["width"], p["height"]) == (3, 2)
    assert len(_unpack(p)) == 6


def test_a_degenerate_grid_returns_an_empty_payload_rather_than_raising() -> None:
    p = pack_grid([], width=0, height=0, max_cells=400)
    assert p["width"] == 0 and p["data"] == ""


def test_reset_forgets_the_old_map_entirely() -> None:
    """RESET MAP restarts slam_toolbox; the first transform of the new map must
    not read as a loop closure against the last one of the old."""
    h = SlamHealth()
    h.update(0.0, 0.0, 0.0, now=1.0)
    h.update(2.0, 0.0, 0.0, now=2.0)        # a jump
    assert h.snapshot(now=2.0, max_age_s=3.0)["jump_count"] == 1
    h.reset()
    snap = h.snapshot(now=2.5, max_age_s=3.0)
    assert snap["available"] is False
    assert snap["jump_count"] == 0
    assert snap["correction_m"] is None
    h.update(0.0, 0.0, 0.0, now=3.0)        # the new map's first transform
    snap = h.snapshot(now=3.0, max_age_s=3.0)
    assert snap["jump_count"] == 0
    assert snap["available"] is True
