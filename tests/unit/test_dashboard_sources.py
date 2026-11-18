"""The dashboard's view of the command-source table.

With several policy nodes running, the page follows ONE (the active policy)
and offers one button per table row. The pure resolver is tested directly;
the node and the page are pinned as text, the way the other dashboard tests
do it, because importing dashboard_node needs rclpy.

Pure stdlib + yaml: workstation tier.
"""

from __future__ import annotations

import os
import re
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("rm_dashboard", "rm_safety"):
    _p = os.path.join(_REPO_ROOT, "src", _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from rm_dashboard.sources import active_policy, is_live  # noqa: E402
from rm_safety.command_sources import load_command_sources  # noqa: E402

_NODE = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "dashboard_node.py")
_INDEX = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html")
_PACKAGE = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "package.xml")
_TOPICS = os.path.join(_REPO_ROOT, "config", "topics.yaml")

POLICIES = ["policy", "m8v2", "m4", "m9", "dynabarn"]   # table order


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


# --- the resolver ----------------------------------------------------------------

def test_selected_policy_wins() -> None:
    assert active_policy("m4", "m8v2", POLICIES, {"m8v2"}) == "m4"


def test_teleop_selected_follows_the_last_policy_even_if_silent() -> None:
    """Switching to teleop between runs must not flip the page to another policy."""
    assert active_policy("teleop", "m4", POLICIES, {"m8v2"}) == "m4"


def test_nothing_selected_yet_follows_the_first_live_policy() -> None:
    assert active_policy(None, None, POLICIES, {"m4"}) == "m4"
    assert active_policy("teleop", None, POLICIES, {"m4", "m8v2"}) == "m8v2"


def test_nothing_live_falls_back_to_the_first_table_row() -> None:
    assert active_policy("teleop", None, POLICIES, set()) == "policy"
    assert active_policy("teleop", "bogus", POLICIES, set()) == "policy"


def test_no_policies_yields_none() -> None:
    assert active_policy("m4", "m4", [], {"m4"}) is None


@pytest.mark.parametrize("slot, want", [
    (None, False),
    ({}, False),
    ({"count": 0, "stale": True}, False),
    ({"count": 3, "stale": True}, False),
    ({"count": 3, "stale": False}, True),
    ({"count": 1}, False),              # no staleness field: not proven fresh
])
def test_is_live(slot, want) -> None:
    assert is_live(slot) is want


# --- the node, pinned as text ------------------------------------------------------

def test_node_reads_the_table_and_subscribes_per_row() -> None:
    src = _read(_NODE)
    assert "from rm_safety.command_sources import" in src
    assert 'self.declare_parameter("topics_path", DEFAULT_TOPICS_PATH)' in src
    assert "for source in self._sources.values():" in src
    assert "self._twist_cb(source.topic)" in src
    assert "self._debug_cb(source.debug)" in src
    # No topic literal survives outside the table.
    for literal in ('"/policy/cmd_vel"', '"/teleop/cmd_vel"', '"/ncp_debug"',
                    '"/policy_node/set_parameters_atomically"'):
        assert literal not in src, literal
    assert '<depend>rm_safety</depend>' in _read(_PACKAGE)


def test_source_command_accepts_exactly_the_table() -> None:
    src = _read(_NODE)
    body = src[src.index('if command == "source":'):src.index('if command == "goal":')]
    assert "value not in self._sources" in body
    assert "('policy', 'teleop')" not in body


def test_goals_go_to_every_running_policy() -> None:
    src = _read(_NODE)
    assert "def _retarget(" in src and "def _retarget_targets(" in src
    for cmd in ('if command == "goal":', "def _set_goal_from_world(", "def _reset_map("):
        body = src[src.index(cmd):]
        body = body[:body.index("\n    def ")] if "\n    def " in body else body
        assert "self._retarget(" in body, cmd
    # One param client per policy row with a node, keyed by row name.
    assert "self._policy_params = {" in src
    assert 'f"/{source.node}/set_parameters_atomically"' in src


def test_map_click_refuses_a_policy_whose_debug_stream_is_stale() -> None:
    src = _read(_NODE)
    body = src[src.index("def _set_goal_from_world("):src.index("def _reset_odom(")]
    stale = body.index("is_live(topics.get(self._sources[name].debug))")
    assert stale < body.index("self._retarget(")
    assert "stale" in body[stale:stale + 600]


def test_manual_paths_refuse_on_any_policy_kind_source() -> None:
    src = _read(_NODE)
    for fn in ("def _send_jog(", "def _send_waypoint("):
        body = src[src.index(fn):]
        body = body[:body.index("\n    def ")]
        assert "self._source_is_policy(self._command_source)" in body, fn
        assert '== "policy"' not in body, fn


def test_snapshot_carries_the_table_and_the_active_policy() -> None:
    src = _read(_NODE)
    body = src[src.index("def _snapshot("):src.index("def _dispatch(")]
    for key in ('"sources":', '"active_policy":', '"live": ', '"params_ready": ', '"robot": {'):
        assert key in body, key
    assert "self._active_policy(topics)" in body and "self._live_sources(topics)" in body
    helpers = src[src.index("def _live_sources("):src.index("def _policy_debug(")]
    assert "is_live(" in helpers and "active_policy(" in helpers


# --- the page, pinned as text ------------------------------------------------------

def test_page_builds_source_buttons_from_the_snapshot() -> None:
    html = _read(_INDEX)
    assert 'id="srcbtns"' in html
    assert "function renderSources(snap)" in html
    assert 'data-src="teleop"' not in html and 'data-src="policy"' not in html
    # A policy button greys until its node publishes; teleop never does for that.
    handler = html[html.index("function renderSources(snap)"):]
    handler = handler[:handler.index("\n}\n")]
    assert 'r.kind === "policy" && !r.live' in handler
    assert "confirm(" in html[html.index("function askSource("):]


def test_page_follows_the_active_policy_not_a_fixed_topic() -> None:
    html = _read(_INDEX)
    assert '"/ncp_debug"' not in html and '"/policy/cmd_vel"' not in html
    assert "snap.active_policy" in html
    for fn in ("function renderHeader(", "function renderObs(", "function renderScan(",
               "function renderMap(", "function renderCmd(", "function chainFor("):
        body = html[html.index(fn):]
        body = body[:body.index("\n}\n")]
        assert "ap.debug" in body or "ap.topic" in body, fn


def test_observation_labels_follow_the_vector_width() -> None:
    """The 4.0.0 vector is 724 wide; a hard-coded 0-31 label was a lie."""
    html = _read(_INDEX)
    assert "0–31 lidar" not in html
    assert re.search(r'\$\("obslidar"\)\.textContent\s*=', html)


def test_table_rows_the_page_will_render() -> None:
    rows = load_command_sources(_TOPICS)
    assert [n for n, s in rows.items() if s.is_policy] == POLICIES
