"""Tests for rm_dashboard.state_store — rate, staleness and snapshot logic.

Pure stdlib: runs in the workstation tier, no ROS and no browser. This is the
half of the dashboard that can be wrong in a way that MISLEADS rather than
merely looks bad — a wrong rate or a topic wrongly reported fresh sends an
operator chasing the wrong thing, which on this project has cost whole
sessions. So it is the half that gets tested.
"""

from __future__ import annotations

import os
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_dashboard"))

from rm_dashboard.state_store import StateStore  # noqa: E402


def _feed(store, topic, n, period, start=1000.0):
    """Feed `n` messages at a fixed period on a controlled clock."""
    for i in range(n):
        store.update(topic, {"i": i}, now=start + i * period)
    return start + (n - 1) * period


def test_unknown_topic_is_absent_not_invented() -> None:
    store = StateStore(expected_hz={"/scan": 10.0})
    snap = store.snapshot(now=1000.0)
    assert "/scan" in snap, "a declared topic must appear even before data"
    assert "/nope" not in snap


def test_declared_topic_with_no_data_is_stale_and_zero_count() -> None:
    store = StateStore(expected_hz={"/scan": 10.0})
    slot = store.snapshot(now=1000.0)["/scan"]
    assert slot["stale"] is True
    assert slot["count"] == 0
    assert slot["value"] is None
    assert slot["hz"] is None
    assert slot["age_ms"] is None


def test_rate_is_none_until_two_samples() -> None:
    """One message does not establish a rate; reporting 0.0 would be a lie."""
    store = StateStore()
    store.update("/x", 1, now=1000.0)
    assert store.snapshot(now=1000.0)["/x"]["hz"] is None
    store.update("/x", 2, now=1000.1)
    assert store.snapshot(now=1000.1)["/x"]["hz"] == pytest.approx(10.0)


def test_measured_rate_matches_the_feed() -> None:
    store = StateStore(expected_hz={"/cmd_vel": 20.0})
    last = _feed(store, "/cmd_vel", 21, 0.05)
    slot = store.snapshot(now=last)["/cmd_vel"]
    assert slot["hz"] == pytest.approx(20.0, rel=1e-6)
    assert slot["count"] == 21
    assert slot["stale"] is False


def test_jitter_is_zero_for_a_perfect_feed_and_positive_for_a_ragged_one() -> None:
    perfect = StateStore()
    _feed(perfect, "/a", 10, 0.1)
    assert perfect.snapshot(now=1000.9)["/a"]["jitter_ms"] == pytest.approx(0.0, abs=1e-6)

    ragged = StateStore()
    t = 1000.0
    for gap in (0.10, 0.02, 0.30, 0.05, 0.22, 0.03):
        t += gap
        ragged.update("/a", 1, now=t)
    assert ragged.snapshot(now=t)["/a"]["jitter_ms"] > 5.0


def test_topic_goes_stale_after_silence() -> None:
    """The failure this exists to show: publishing, then stopping."""
    store = StateStore(expected_hz={"/scan": 10.0})
    last = _feed(store, "/scan", 10, 0.1)
    assert store.snapshot(now=last)["/scan"]["stale"] is False
    # 4 expected periods = 0.4 s; well past it.
    late = store.snapshot(now=last + 1.0)["/scan"]
    assert late["stale"] is True
    assert late["count"] == 10, "count distinguishes 'stopped' from 'never started'"
    assert late["age_ms"] == pytest.approx(1000.0)


def test_a_fast_topic_is_not_stale_from_one_dropped_message() -> None:
    """The staleness floor exists so a single drop is not an alarm."""
    store = StateStore(expected_hz={"/cmd_vel": 20.0})
    last = _feed(store, "/cmd_vel", 20, 0.05)
    # Two whole periods late: annoying, not stale.
    assert store.snapshot(now=last + 0.1)["/cmd_vel"]["stale"] is False


def test_undeclared_topic_uses_the_floor_not_a_guessed_rate() -> None:
    store = StateStore()
    store.update("/x", 1, now=1000.0)
    store.update("/x", 2, now=1000.1)
    assert store.snapshot(now=1000.2)["/x"]["stale"] is False
    assert store.snapshot(now=1002.0)["/x"]["stale"] is True


def test_snapshot_is_json_serializable() -> None:
    """The snapshot goes straight out over SSE — a non-serializable value
    would kill the stream for every connected client at once."""
    import json

    store = StateStore(expected_hz={"/scan": 10.0})
    store.update("/scan", {"ranges": [1.0, 2.0], "min_range_m": 1.0}, now=1000.0)
    store.update("/scan", {"ranges": [1.0, 2.0], "min_range_m": 1.0}, now=1000.1)
    json.dumps(store.snapshot(now=1000.1))


def test_latest_value_wins() -> None:
    store = StateStore()
    store.update("/x", {"v": 1}, now=1000.0)
    store.update("/x", {"v": 2}, now=1000.1)
    assert store.snapshot(now=1000.1)["/x"]["value"] == {"v": 2}


def test_concurrent_writers_do_not_corrupt_a_snapshot() -> None:
    """The ROS thread writes while the HTTP thread reads. A snapshot must
    always be a complete, coherent set — never a half-updated one."""
    import threading

    store = StateStore()
    stop = threading.Event()

    def writer(name):
        i = 0
        while not stop.is_set():
            store.update(name, {"i": i})
            i += 1

    threads = [threading.Thread(target=writer, args=(f"/t{n}",)) for n in range(4)]
    for t in threads:
        t.start()
    try:
        for _ in range(200):
            snap = store.snapshot()
            for topic, slot in snap.items():
                assert set(slot) == {
                    "value", "count", "expected_hz", "hz",
                    "jitter_ms", "age_ms", "stale",
                }
                assert slot["count"] >= 1
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
