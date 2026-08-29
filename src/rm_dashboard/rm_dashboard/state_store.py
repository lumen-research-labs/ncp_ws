"""
state_store.py — the dashboard's shared state: latest value per topic, plus
arrival-rate and staleness bookkeeping. ROS-free and thread-safe.

What it does:
    Holds one slot per monitored topic. The ROS thread writes into it from
    subscription callbacks; the HTTP thread reads a consistent snapshot out of
    it. Every write records a monotonic arrival time, from which the store
    derives a measured rate and an age — which is most of what a stack
    dashboard is actually for. On this project nearly every failure so far has
    announced itself as "topic X is publishing but at the wrong rate", "topic X
    stopped", or "topic X was never there" (the QoS mismatch of 2026-08-27, the
    doubled EKF of 2026-08-29, every staleness watchdog in rm_safety), and none
    of those are visible in the VALUES alone.

    Rates are measured over a sliding window of arrival times rather than as an
    EWMA, because the number that matters operationally is "is /policy/cmd_vel
    at 5 Hz right now" and a window gives that directly, with a jitter figure
    alongside it. A rate is reported as None until the window holds at least
    two samples — a single message does not establish a rate, and printing one
    from a single sample would be an invention.

    Kept deliberately separate from the ROS node and the HTTP server so the
    snapshot logic can be unit-tested with no ROS runtime and no browser (the
    workstation tier). This module imports nothing outside the standard
    library.

Subscribes to:  (nothing — pure module; the node feeds it)
Publishes:      (nothing — pure module; the server reads snapshots from it)

Failure mode:
    Non-blocking and total: every accessor returns a well-formed snapshot even
    when nothing has ever arrived (values None, `age_ms` None, `stale` True).
    A dashboard that raises or blocks because a topic is missing is useless in
    exactly the situation you opened it for. Staleness is therefore reported,
    never raised, and an absent topic is represented explicitly rather than
    omitted — a missing key in the payload would render as a blank panel,
    which reads as "fine" instead of "not there".
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Optional

# Arrival timestamps kept per topic. At 30 Hz (the fastest topic here, /odom)
# this is ~1.3 s of history — long enough for a stable rate estimate, short
# enough that the reported rate follows a real change quickly rather than
# averaging across it.
_RATE_WINDOW = 40

# A topic is stale when nothing has arrived for this multiple of its expected
# period. Deliberately generous: this drives a colour on a web page, not a
# safety decision. rm_safety's own watchdogs are the authority on staleness
# that matters, they use the constants in safety_limits.yaml, and nothing here
# may ever be mistaken for them.
_STALE_PERIOD_MULTIPLE = 4.0

# Floor on the staleness threshold, seconds. Without it, a topic with a high
# expected rate would be marked stale on a single dropped message.
_MIN_STALE_SEC = 0.5


@dataclass
class TopicSlot:
    """Latest payload for one topic, plus its arrival history."""

    # Expected publish rate, Hz, from the measured figures in
    # config/interface.yaml. Used only to decide when to call a topic stale
    # and to show "5.0 / 5.0" style expected-vs-actual in the UI.
    expected_hz: Optional[float] = None
    value: Optional[Any] = None
    arrivals: Deque[float] = field(default_factory=lambda: deque(maxlen=_RATE_WINDOW))
    # Total messages since the node started. Distinguishes "never published"
    # from "published and then stopped", which look identical if you only
    # look at the age.
    count: int = 0


class StateStore:
    """Thread-safe latest-value store with rate and staleness derivation."""

    def __init__(self, expected_hz: Optional[Dict[str, float]] = None) -> None:
        self._lock = threading.Lock()
        self._slots: Dict[str, TopicSlot] = {}
        for name, hz in (expected_hz or {}).items():
            self._slots[name] = TopicSlot(expected_hz=hz)

    def update(self, topic: str, value: Any, now: Optional[float] = None) -> None:
        """Record a new value for `topic`. Called from the ROS thread."""
        stamp = time.monotonic() if now is None else now
        with self._lock:
            slot = self._slots.get(topic)
            if slot is None:
                slot = TopicSlot()
                self._slots[topic] = slot
            slot.value = value
            slot.arrivals.append(stamp)
            slot.count += 1

    def snapshot(self, now: Optional[float] = None) -> Dict[str, Any]:
        """A consistent, JSON-serializable view of everything.

        Taken under the lock in one pass so the HTTP thread can never observe
        a half-updated set of topics — a snapshot showing a fresh action
        against a stale scan that in fact arrived together would send someone
        hunting a timing bug that does not exist.
        """
        stamp = time.monotonic() if now is None else now
        out: Dict[str, Any] = {}
        with self._lock:
            for topic, slot in self._slots.items():
                out[topic] = {
                    "value": slot.value,
                    "count": slot.count,
                    "expected_hz": slot.expected_hz,
                    "hz": _rate_hz(slot.arrivals),
                    "jitter_ms": _jitter_ms(slot.arrivals),
                    "age_ms": _age_ms(slot.arrivals, stamp),
                    "stale": _is_stale(slot, stamp),
                }
        return out


# ---------------------------------------------------------------------------
# Derivations. Free functions so they are testable without constructing a
# store, and so the locking above stays a thin shell around pure logic.
# ---------------------------------------------------------------------------


def _rate_hz(arrivals: Deque[float]) -> Optional[float]:
    """Measured rate over the arrival window, or None if undetermined.

    None (not 0.0) with fewer than two samples: zero is a claim that the topic
    is not publishing, and one message is not evidence for that.
    """
    if len(arrivals) < 2:
        return None
    span = arrivals[-1] - arrivals[0]
    if span <= 0.0:
        return None
    return (len(arrivals) - 1) / span


def _jitter_ms(arrivals: Deque[float]) -> Optional[float]:
    """Standard deviation of the inter-arrival interval, milliseconds.

    This is the figure that caught the doubled EKF and the QoS mismatch on
    this project — a topic can hold a plausible mean rate while its intervals
    are visibly wrong. Population (not sample) standard deviation: the window
    is the whole of what is being described, and there is no wider population
    being estimated.
    """
    if len(arrivals) < 3:
        return None
    gaps = [
        (arrivals[i] - arrivals[i - 1]) * 1000.0 for i in range(1, len(arrivals))
    ]
    mean = sum(gaps) / len(gaps)
    variance = sum((g - mean) ** 2 for g in gaps) / len(gaps)
    return variance ** 0.5


def _age_ms(arrivals: Deque[float], now: float) -> Optional[float]:
    """Milliseconds since the last message; None if none ever arrived."""
    if not arrivals:
        return None
    return max(0.0, (now - arrivals[-1]) * 1000.0)


def _is_stale(slot: TopicSlot, now: float) -> bool:
    """True when a topic has gone quiet — INCLUDING never having published.

    Never-published is deliberately stale rather than a third state. On a
    dashboard the two demand the same response (go find out why there is no
    data), and `count == 0` is carried alongside for anyone who needs to tell
    them apart.
    """
    if not slot.arrivals:
        return True
    threshold = _MIN_STALE_SEC
    if slot.expected_hz:
        threshold = max(threshold, _STALE_PERIOD_MULTIPLE / slot.expected_hz)
    return (now - slot.arrivals[-1]) > threshold
