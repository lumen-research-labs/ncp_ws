"""
sources.py — which policy the dashboard follows, and whether a command
source is live. Pure python, no ROS; the node feeds it store snapshots.

What it does:
    The page has one header, one observation panel and one command chain,
    and with several policy nodes running at once (ncp.launch.py
    `profiles:=`) they have to follow ONE of them. `active_policy` picks it:
    the selected command source when that is a policy row, else the policy
    most recently selected, else the first policy row that is publishing,
    else the first policy row in the table. That order keeps the page on
    the policy the operator last cared about through a switch to teleop,
    and gives it something sensible to show before anyone has selected
    anything.

    `is_live` is the liveness the source buttons grey on: the row's command
    topic has been seen and is not stale. Never-published and stale read
    the same here, because a button for a source nothing is publishing
    would select silence (and kill, since every switch does).

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    Total functions. An empty table yields None; a missing slot is not
    live. Nothing here raises on the shapes the StateStore produces.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Sequence


def is_live(slot: Optional[Dict[str, Any]]) -> bool:
    """True once a StateStore slot has received something and is not stale."""
    if not slot:
        return False
    return int(slot.get("count", 0)) > 0 and not bool(slot.get("stale", True))


def active_policy(
    command_source: Optional[str],
    last_policy: Optional[str],
    policies: Sequence[str],
    live: Iterable[str],
) -> Optional[str]:
    """The policy row the page follows. See the module docstring for the order."""
    if command_source in policies:
        return command_source
    if last_policy in policies:
        return last_policy
    live_set = set(live)
    for name in policies:
        if name in live_set:
            return name
    return policies[0] if policies else None
