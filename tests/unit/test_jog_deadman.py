"""The jog deadman: a manual drive command must expire on its own.

The caller is a browser on a network. It can close its tab, sleep the laptop,
lose WiFi or just stop sending mid-keypress, and NONE of those deliver a
"stop" message. So a jog cannot mean "drive until told otherwise" — it means
"drive for at most ttl_s more", and the robot stops when the re-sends stop.

These tests pin that property against the values the node and the page
actually use, so a later change that lengthens the TTL or slows the browser's
resend has to face the arithmetic.

Pure stdlib. Workstation tier — no ROS, no browser.
"""

from __future__ import annotations

import os
import re

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_NODE = os.path.join(
    _REPO_ROOT, "src", "rm_waypoint", "rm_waypoint", "waypoint_node.py"
)
_PAGE = os.path.join(
    _REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html"
)
_SRV = os.path.join(_REPO_ROOT, "src", "rm_interfaces", "srv", "Jog.srv")


def _node() -> str:
    with open(_NODE) as fh:
        return fh.read()


def _page() -> str:
    with open(_PAGE) as fh:
        return fh.read()


def _const(text: str, name: str) -> float:
    m = re.search(rf"^{name}\s*=\s*([0-9.]+)", text, re.M)
    assert m, f"{name} not found"
    return float(m.group(1))


def _js_const(text: str, name: str) -> float:
    m = re.search(rf"{name}\s*=\s*([0-9.]+)", text)
    assert m, f"{name} not found in the page"
    return float(m.group(1))


def test_the_node_caps_the_ttl_the_browser_asks_for() -> None:
    """The browser supplies a TTL, but the NODE decides the maximum.

    Otherwise the deadman is only as short as whatever a page — or anything
    else that can reach the service — chose to ask for.
    """
    node = _node()
    cap = _const(node, "_JOG_MAX_TTL_S")
    assert 0.0 < cap <= 1.0, f"jog TTL cap {cap}s is too long for a deadman"
    assert "ttl = min(ttl, _JOG_MAX_TTL_S)" in node, "the cap is not applied"


def test_a_nonpositive_ttl_is_refused() -> None:
    """A jog with no expiry is the one thing this must never accept."""
    node = _node()
    assert "if ttl <= 0.0:" in node
    assert "a jog must expire" in node


def test_the_browser_resends_faster_than_the_ttl_expires() -> None:
    """If the resend interval exceeded the TTL the robot would stutter —
    driving, expiring, driving — instead of moving smoothly."""
    page = _page()
    ttl_s = _js_const(page, "JOG_TTL_S")
    send_ms = _js_const(page, "JOG_SEND_MS")
    assert send_ms / 1000.0 < ttl_s / 2.0, (
        f"resend every {send_ms} ms against a {ttl_s}s TTL leaves no margin"
    )


def test_the_browsers_ttl_is_within_the_nodes_cap() -> None:
    """A page asking for more than the node allows would silently get less;
    better that the two agree."""
    assert _js_const(_page(), "JOG_TTL_S") <= _const(_node(), "_JOG_MAX_TTL_S")


def test_expiry_goes_SILENT_rather_than_publishing_zeros() -> None:
    """The gamepad shares /teleop/cmd_vel. A stream of zeros from here would
    fight it; silence lets rm_safety's command_timeout do the stopping."""
    node = _node()
    assert "EXPIRED: go silent rather than publishing zeros" in node


def test_releasing_a_key_does_not_depend_on_a_stop_message() -> None:
    """jogStop() must not POST a zero command — the expiry is the mechanism,
    and relying on a delivered 'stop' would reintroduce the failure the TTL
    exists to remove."""
    page = _page()
    body = page[page.index("function jogStop("):]
    body = body[: body.index("\n}")]
    assert "fetch(" not in body, "jogStop must not send anything"


def test_losing_window_focus_stops_the_jog() -> None:
    """A held key whose keyup lands in another window would otherwise leave
    the page believing it is still held, re-sending forever."""
    page = _page()
    assert 'window.addEventListener("blur", jogStop)' in page
    assert "visibilitychange" in page


def test_jog_velocities_are_clamped_to_the_configured_limits() -> None:
    """A browser must not be able to ask for more than the operator set."""
    node = _node()
    assert "max(-self._limits.max_vx, min(self._limits.max_vx, vx))" in node
    assert "max(-self._limits.max_wz, min(self._limits.max_wz, wz))" in node


def test_reverse_is_allowed_for_jog_unlike_waypoints() -> None:
    """Backing out of a corner is exactly what repositioning needs, and there
    is a human watching — which is not true of an autonomous waypoint."""
    assert "REVERSE IS ALLOWED" in open(_SRV).read()


def test_a_jog_cancels_an_active_waypoint() -> None:
    """Manual input wins: an operator reaching for the keys mid-drive is a
    correction, not a second opinion to be merged."""
    node = _node()
    assert "jog received — cancelling waypoint" in node


def test_the_dashboard_still_has_no_publisher() -> None:
    """The jog goes out as a SERVICE call. The moment the dashboard owns a
    Twist publisher it stops being structurally incapable of becoming a
    second /cmd_vel source."""
    path = os.path.join(
        _REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "dashboard_node.py"
    )
    with open(path) as fh:
        src = fh.read()
    assert "create_publisher" not in src, "rm_dashboard must never publish"
