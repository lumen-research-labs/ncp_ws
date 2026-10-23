"""Roam mode: the pure sequencer, the shipped limits, and the wiring pins.

The sequencer decides when a goal is finished (arrived after a dwell, or
timed out) and draws the next random offset. Its clocks are pure functions of
the timestamps the node hands it, so every decision is tested here without a
clock. The second half pins, by reading the source text, that the node, the
launch and the dashboard wire it the way docs/roam.md says: one place writes
the goal, the step sits after every fault return, the parameter is settable,
and nothing here can arm the robot.

Pure stdlib + yaml. Workstation tier.
"""

from __future__ import annotations

import dataclasses
import math
import os
import re
import sys

import pytest
import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.roam import (  # noqa: E402
    ARRIVED,
    HOLD_RETRY_S,
    KEYS,
    NO_FREE_GOAL,
    STALLED,
    TIMEOUT,
    RoamLimits,
    RoamSequencer,
    Scan,
    check_limits,
    limits_from_dict,
    load_roam_limits,
    strip_is_free,
)

_ROAM_YAML = os.path.join(_REPO_ROOT, "config", "roam.yaml")
_INTERFACE_YAML = os.path.join(_REPO_ROOT, "config", "interface.yaml")
_POLICY_NODE = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "policy_node.py")
_VENDOR = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "vendor", "ncp_policy.py")
_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "ncp.launch.py")
_DASHBOARD_NODE = os.path.join(
    _REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "dashboard_node.py"
)
_INDEX = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html")
_DOCS = os.path.join(_REPO_ROOT, "docs")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _limits(**overrides) -> RoamLimits:
    base = dict(range_min_m=1.0, range_max_m=2.5, bearing_cap_deg=45.0,
                dwell_s=1.0, timeout_s=30.0, seed=0,
                corridor_half_width_m=0.20, clearance_m=0.5, min_valid_fraction=0.5,
                draw_attempts=32, progress_window_s=10.0, progress_min_m=0.05)
    base.update(overrides)
    return RoamLimits(**base)


def _seq(on_at=0.0, **overrides) -> RoamSequencer:
    seq = RoamSequencer(_limits(**overrides))
    if on_at is not None:
        seq.set_on(True, on_at)
    return seq


# A 360-ray, 1-degree scan starting at -pi, like the TG30's layout: `f` maps
# a bearing in degrees to a range; 0.0 is the no-return sentinel.
_N, _INC, _AMIN = 360, math.radians(1.0), -math.pi


def _scan(f) -> Scan:
    return Scan([f(math.degrees(_AMIN + i * _INC)) for i in range(_N)], _AMIN, _INC, 0.01)


_OPEN = _scan(lambda a: 10.0)
_WALL_AHEAD = _scan(lambda a: 1.5 if abs(a) <= 30 else 10.0)   # a wall 1.5 m ahead
_BOXED = _scan(lambda a: 0.8)                                   # walls all round
_UNKNOWN = _scan(lambda a: 0.0)                                 # nothing but no-returns


def _polar(offset):
    return math.hypot(offset[0], offset[1]), math.degrees(math.atan2(offset[1], offset[0]))


# ---------------------------------------------------------------------------
# the draw
# ---------------------------------------------------------------------------

def test_draws_stay_inside_the_configured_range_and_cone():
    seq = _seq()
    for _ in range(2000):
        x, y, attempts = seq.draw()
        assert attempts == 1          # no scan: the first candidate is taken
        d = math.hypot(x, y)
        assert 1.0 <= d <= 2.5
        assert abs(math.degrees(math.atan2(y, x))) <= 45.0
        assert x >= 0.0  # ahead, for any cap at or under 90 deg


def test_same_seed_draws_the_same_sequence_and_different_seeds_differ():
    a = _seq(seed=7).draw()
    b = _seq(seed=7).draw()
    c = _seq(seed=8).draw()
    assert a == b
    assert a != c
    s1, s2 = _seq(seed=3), _seq(seed=3)
    assert [s1.draw() for _ in range(20)] == [s2.draw() for _ in range(20)]


# ---------------------------------------------------------------------------
# free space
# ---------------------------------------------------------------------------

def test_strip_is_free_reads_the_scan_as_a_strip_to_the_goal_plus_clearance():
    lim = _limits()
    assert strip_is_free(_OPEN, 0.0, 2.0, lim) is True
    assert strip_is_free(_WALL_AHEAD, 0.0, 2.0, lim) is False
    # 1.0 m goal + 0.5 m clearance reaches the wall at 1.5 m exactly: blocked.
    assert strip_is_free(_WALL_AHEAD, 0.0, 1.0, lim) is False
    assert strip_is_free(_WALL_AHEAD, 0.0, 0.9, lim) is True
    # Beside the wall the strip is clear.
    assert strip_is_free(_WALL_AHEAD, math.radians(90.0), 2.0, lim) is True
    # A return just outside the strip's width does not block it.
    side = _scan(lambda a: 0.5 if abs(a - 30.0) < 0.5 else 10.0)   # 0.5 m at 30 deg: lateral 0.25 m
    assert strip_is_free(side, 0.0, 2.0, lim) is True
    narrow = _scan(lambda a: 1.0 if abs(a - 10.0) < 0.5 else 10.0)  # 1.0 m at 10 deg: lateral 0.17 m
    assert strip_is_free(narrow, 0.0, 2.0, lim) is False


def test_no_returns_are_unknown_not_free():
    lim = _limits()
    assert strip_is_free(_UNKNOWN, 0.0, 2.0, lim) is None
    # Half the rays over the strip valid and far: enough to call it free.
    half = _scan(lambda a: 10.0 if int(a) % 2 == 0 else 0.0)
    assert strip_is_free(half, 0.0, 2.0, lim) is True
    # Fewer than the fraction: unknown.
    sparse = _scan(lambda a: 10.0 if int(a) % 4 == 0 else 0.0)
    assert strip_is_free(sparse, 0.0, 2.0, lim) is None
    # An unreadable scan is unknown, never free.
    assert strip_is_free(Scan([], _AMIN, _INC, 0.01), 0.0, 2.0, lim) is None
    assert strip_is_free(Scan([10.0] * 10, _AMIN, 0.0, 0.01), 0.0, 2.0, lim) is None


def test_draw_avoids_the_wall_and_holds_when_boxed_in():
    seq = _seq()
    x, y, attempts = seq.draw(_WALL_AHEAD)
    d, b = _polar((x, y))
    assert attempts >= 1
    assert strip_is_free(_WALL_AHEAD, math.radians(b), d, seq.limits) is True
    assert seq.draw(_BOXED) is None
    assert seq.draw(_UNKNOWN) is None


def test_draw_falls_back_to_the_shortest_leg_across_the_cone():
    # A wall at 1.6 m everywhere in the cone: no random leg over 1.1 m is
    # free, but the shortest leg (1.0 m + 0.5 m clearance) is.
    near_wall = _scan(lambda a: 1.6)
    seq = _seq(draw_attempts=4, seed=1)
    x, y, attempts = seq.draw(near_wall)
    d, b = _polar((x, y))
    assert d == pytest.approx(1.0)
    assert abs(b) <= 45.0
    assert attempts > 4


# ---------------------------------------------------------------------------
# the hand-off clock
# ---------------------------------------------------------------------------

def test_off_never_hands_off():
    seq = _seq(on_at=None)
    seq.goal_fixed(0.0)
    assert seq.step(500.0, goal_reached=True) is None
    assert seq.step(1000.0, goal_reached=False) is None
    assert seq.n == 0 and seq.timeouts == 0 and seq.last_reason is None


def test_arrival_hands_off_after_the_dwell_not_before():
    seq = _seq()
    seq.goal_fixed(0.0)
    assert seq.step(10.0, goal_reached=True) is None      # the arrival edge
    assert seq.step(10.9, goal_reached=True) is None      # inside the dwell
    h = seq.step(11.0, goal_reached=True)
    assert h is not None and h.reason == ARRIVED and h.n == 1
    assert h.elapsed_s == pytest.approx(11.0)
    assert seq.n == 1 and seq.timeouts == 0 and seq.last_reason == ARRIVED


def test_timeout_hands_off_and_is_counted():
    seq = _seq()
    seq.goal_fixed(0.0)
    assert seq.step(29.9, goal_reached=False) is None
    h = seq.step(30.1, goal_reached=False)
    assert h is not None and h.reason == TIMEOUT
    assert h.elapsed_s == pytest.approx(30.1)
    assert seq.timeouts == 1 and seq.last_reason == TIMEOUT


def test_enabling_while_latched_dwells_from_the_switch_on():
    seq = _seq(on_at=None)
    seq.goal_fixed(0.0)
    seq.step(5.0, goal_reached=True)             # latched long before roam
    seq.set_on(True, 100.0)
    assert seq.step(100.5, goal_reached=True) is None
    h = seq.step(101.0, goal_reached=True)
    assert h is not None and h.reason == ARRIVED


def test_enabling_with_an_old_goal_gives_it_a_full_timeout():
    seq = _seq(on_at=None)
    seq.goal_fixed(0.0)
    seq.set_on(True, 100.0)                       # goal is already 100 s old
    assert seq.step(100.1, goal_reached=False) is None
    assert seq.step(129.9, goal_reached=False) is None
    h = seq.step(130.1, goal_reached=False)
    assert h is not None and h.reason == TIMEOUT


def test_re_enabling_does_not_move_the_clock():
    seq = _seq()
    seq.goal_fixed(0.0)
    assert seq.set_on(True, 25.0) is False        # already on: no change
    h = seq.step(30.1, goal_reached=False)
    assert h is not None and h.reason == TIMEOUT


def test_a_hand_off_restarts_the_goal_clock_even_without_goal_fixed():
    seq = _seq()
    seq.goal_fixed(0.0)
    assert seq.step(31.0, goal_reached=False) is not None
    assert seq.step(31.2, goal_reached=False) is None   # no storm
    assert seq.step(61.0, goal_reached=False) is None
    assert seq.step(61.3, goal_reached=False) is not None


def test_no_progress_hands_off_as_stalled_and_progress_resets_the_clock():
    seq = _seq()
    seq.goal_fixed(0.0)
    # The progress clock starts at the first distance sample, which the node
    # supplies on the tick after the fix.
    assert seq.step(0.2, goal_reached=False, goal_dist=2.0) is None
    assert seq.step(9.9, goal_reached=False, goal_dist=2.0) is None
    h = seq.step(10.3, goal_reached=False, goal_dist=2.0)
    assert h is not None and h.reason == STALLED
    assert seq.stalls == 1 and seq.timeouts == 0 and seq.last_reason == STALLED
    # Closing on the goal keeps it alive; the timeout stays the ceiling.
    seq = _seq()
    seq.goal_fixed(0.0)
    for t, d in ((2.0, 2.0), (8.0, 1.9), (16.0, 1.8), (24.0, 1.7)):
        assert seq.step(t, goal_reached=False, goal_dist=d) is None
    h = seq.step(30.5, goal_reached=False, goal_dist=1.65)
    assert h is not None and h.reason == TIMEOUT


def test_progress_smaller_than_the_minimum_does_not_count():
    seq = _seq()
    seq.goal_fixed(0.0)
    assert seq.step(0.2, goal_reached=False, goal_dist=2.00) is None
    assert seq.step(8.0, goal_reached=False, goal_dist=1.97) is None   # 3 cm: noise
    h = seq.step(10.5, goal_reached=False, goal_dist=1.96)
    assert h is not None and h.reason == STALLED


def test_a_due_hand_off_with_no_free_goal_holds_and_retries_once_a_second():
    seq = _seq()
    seq.goal_fixed(0.0)
    assert seq.step(31.0, goal_reached=False, goal_dist=2.0, scan=_BOXED) is None
    assert seq.holding and seq.held == 1 and seq.last_reason == NO_FREE_GOAL
    assert seq.n == 0 and seq.timeouts == 0
    assert seq.status(31.0)["holding"] is True
    # Within the retry interval nothing is drawn again (held stays 1 either
    # way; the point is no storm of draws).
    assert seq.step(31.0 + HOLD_RETRY_S / 2, goal_reached=False, goal_dist=2.0, scan=_BOXED) is None
    assert seq.held == 1
    # Open floor appears: the next retry hands off.
    h = seq.step(31.0 + HOLD_RETRY_S + 0.1, goal_reached=False, goal_dist=2.0, scan=_OPEN)
    assert h is not None and h.reason == TIMEOUT and not seq.holding
    assert seq.n == 1 and seq.timeouts == 1 and seq.held == 1


def test_hand_off_offsets_are_free_in_the_scan_they_were_drawn_from():
    seq = _seq()
    seq.goal_fixed(0.0)
    h = seq.step(31.0, goal_reached=False, goal_dist=2.0, scan=_WALL_AHEAD)
    assert h is not None
    d, b = _polar((h.offset_x, h.offset_y))
    assert strip_is_free(_WALL_AHEAD, math.radians(b), d, seq.limits) is True


def test_goal_fixed_clears_the_arrival_and_restarts_the_timeout():
    seq = _seq()
    seq.goal_fixed(0.0)
    seq.step(10.0, goal_reached=True)
    seq.goal_fixed(10.5)                           # an operator re-target
    assert seq.step(11.6, goal_reached=False) is None   # no arrived hand-off
    assert seq.step(40.0, goal_reached=False) is None   # 29.5 s: not yet
    assert seq.step(40.6, goal_reached=False) is not None


def test_no_goal_ever_fixed_means_no_hand_off():
    seq = _seq()
    assert seq.step(1000.0, goal_reached=True) is None
    assert seq.step(2000.0, goal_reached=False) is None


def test_set_on_reports_whether_it_changed():
    seq = _seq(on_at=None)
    assert seq.set_on(False, 0.0) is False
    assert seq.set_on(True, 0.0) is True
    assert seq.set_on(True, 1.0) is False
    assert seq.set_on(False, 2.0) is True


def test_status_carries_the_counters_and_goal_age():
    seq = _seq()
    assert seq.status(5.0) == {
        "on": True, "n": 0, "timeouts": 0, "stalls": 0, "held": 0, "holding": False,
        "last_reason": None, "goal_age_s": None,
    }
    seq.goal_fixed(5.0)
    assert seq.status(7.25)["goal_age_s"] == pytest.approx(2.2, abs=0.06)


# ---------------------------------------------------------------------------
# the limits
# ---------------------------------------------------------------------------

def test_the_shipped_config_loads_and_passes_its_own_checks():
    limits = load_roam_limits(_ROAM_YAML)
    assert check_limits(limits) is None
    assert limits == _limits()


def test_keys_dataclass_and_shipped_file_agree():
    fields = tuple(f.name for f in dataclasses.fields(RoamLimits))
    assert fields == KEYS
    with open(_ROAM_YAML, encoding="utf-8") as handle:
        block = yaml.safe_load(handle)["roam"]
    assert set(block) == set(KEYS)


def test_the_shipped_range_min_exceeds_the_arrival_radius():
    success_dist = float(re.search(r"SUCCESS_DIST\s*=\s*([0-9.]+)", _read(_VENDOR)).group(1))
    assert load_roam_limits(_ROAM_YAML).range_min_m > success_dist


def test_the_shipped_range_max_is_inside_the_static_default_goal():
    with open(_INTERFACE_YAML, encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    goal_default = next(
        block["goal_default_m"] for block in cfg["observation"]["layout"]
        if block["name"] == "goal_local"
    )
    assert load_roam_limits(_ROAM_YAML).range_max_m <= float(goal_default[0])


def test_a_missing_key_names_the_key_and_the_file(tmp_path):
    with open(_ROAM_YAML, encoding="utf-8") as handle:
        block = yaml.safe_load(handle)["roam"]
    del block["dwell_s"]
    with pytest.raises(ValueError, match="dwell_s") as excinfo:
        limits_from_dict(block, source="config/roam.yaml")
    assert "config/roam.yaml" in str(excinfo.value)


@pytest.mark.parametrize("key, value, fragment", [
    ("range_min_m", "1.0", "not a number"),
    ("range_min_m", True, "not a number"),
    ("seed", 0.5, "integer"),
    ("draw_attempts", 2.5, "integer"),
])
def test_non_numeric_and_non_integer_seed_are_refused(key, value, fragment):
    block = dataclasses.asdict(_limits())
    block[key] = value
    with pytest.raises(ValueError, match=fragment):
        limits_from_dict(block, source="x.yaml")


@pytest.mark.parametrize("overrides, fragment", [
    (dict(range_min_m=3.0, range_max_m=2.0), "range_min_m"),
    (dict(range_min_m=0.0), "range_min_m"),
    (dict(bearing_cap_deg=200.0), "bearing_cap_deg"),
    (dict(dwell_s=-1.0), "dwell_s"),
    (dict(timeout_s=0.0), "timeout_s"),
    (dict(corridor_half_width_m=0.0), "corridor_half_width_m"),
    (dict(clearance_m=-0.1), "clearance_m"),
    (dict(min_valid_fraction=0.0), "min_valid_fraction"),
    (dict(min_valid_fraction=1.5), "min_valid_fraction"),
    (dict(draw_attempts=0), "draw_attempts"),
    (dict(progress_window_s=0.0), "progress_window_s"),
    (dict(progress_min_m=-0.01), "progress_min_m"),
])
def test_incoherent_limits_are_rejected(overrides, fragment):
    reason = check_limits(_limits(**overrides))
    assert reason is not None and fragment in reason
    with pytest.raises(ValueError, match=fragment):
        limits_from_dict(dataclasses.asdict(_limits(**overrides)), source="x.yaml")


def test_a_missing_file_names_the_path(tmp_path):
    missing = str(tmp_path / "nope.yaml")
    with pytest.raises(FileNotFoundError, match=re.escape(missing)):
        load_roam_limits(missing)


def test_a_file_without_a_roam_block_is_refused(tmp_path):
    path = tmp_path / "roam.yaml"
    path.write_text("waypoint: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="roam:"):
        load_roam_limits(str(path))


# ---------------------------------------------------------------------------
# policy_node wiring (source pins; the workstation tier has no rclpy)
# ---------------------------------------------------------------------------

def test_policy_node_declares_both_parameters_and_loads_the_limits_before_the_runner():
    src = _read(_POLICY_NODE)
    assert 'self.declare_parameter("roam", False)' in src
    assert 'self.declare_parameter("roam_config_path", ' in src
    assert src.index("load_roam_limits(") < src.index("self._runner = PolicyRunner(")
    assert "must exceed goal_reached_dist" in src


def test_the_goal_is_written_in_exactly_one_place_and_every_site_goes_through_it():
    src = _read(_POLICY_NODE)
    assert src.count("self._goal_world_x = self._robot_x + dx") == 1
    assert src.count("self._goal_world_x = ") == 1
    body = src[src.index("def _fix_goal("):src.index("def _scan_cb(")]
    for needle in ("self._goal_reached = False", "self._runner.reset()",
                   "self._roam.goal_fixed(", "self._prev_action_v = 0.0"):
        assert needle in body, needle
    assert src.count("self._fix_goal(") >= 4
    assert 'why="parameter set"' in src
    assert 'why=f"first /odom pose"' in src or 'why="first /odom pose"' in src
    assert 'why=f"roam #' in src


def test_the_roam_step_sits_after_every_fault_return_and_before_the_latch_zero():
    src = _read(_POLICY_NODE)
    step = src.index("self._roam.step(")
    for earlier in ('reason="scan_stale"', 'reason="odom_stale"', 'reason="goal_undefined"'):
        assert src.index(earlier) < step
    assert step < src.index('self._publish_zero(reason="goal_reached")')
    # The goal distance (progress watchdog) and the raw scan (free-space
    # draw) go in; the node's own lidar range_min marks the no-returns.
    call = src[step:src.index(")", src.index("Scan(", step)) + 1]
    assert "goal_dist" in call and "Scan(self._scan_ranges" in call
    assert "self._lidar.range_min_m" in call
    assert "elif self._roam.holding:" in src


def test_on_set_parameters_accepts_roam_and_still_rejects_the_rest():
    src = _read(_POLICY_NODE)
    body = src[src.index("def _on_set_parameters("):src.index("def _fix_goal(")]
    assert 'param.name == "roam"' in body
    assert "is not settable at runtime" in body
    assert "self._set_roam(roam)" in body
    assert body.index("self._fix_goal(") < body.index("self._set_roam(roam)")


def test_roam_status_is_published_and_the_node_watches_no_safety_state():
    src = _read(_POLICY_NODE)
    assert '"roam": self._roam.status(' in src
    assert "SafetyStatus" not in src
    assert "/safety/status" not in src
    assert src.count("create_publisher(") == 2


# ---------------------------------------------------------------------------
# launch wiring
# ---------------------------------------------------------------------------

def test_the_launch_passes_the_config_path_to_every_policy_node_and_has_no_roam_argument():
    src = _read(_LAUNCH)
    assert '_DEFAULT_ROAM_PATH = str(_REPO_ROOT / "config" / "roam.yaml")' in src
    loop = src[src.index("for _name, source, interface, weights in profiles:"):
               src.index("nodes.append(Node(")]
    assert '"roam_config_path": _DEFAULT_ROAM_PATH' in loop
    declared = re.findall(r'DeclareLaunchArgument\(\s*"(\w+)"', src)
    assert "roam" not in declared
    assert os.path.isfile(os.path.join(_REPO_ROOT, "config", "roam.yaml"))


# ---------------------------------------------------------------------------
# dashboard wiring
# ---------------------------------------------------------------------------

def test_roam_is_an_allowed_control():
    src = _read(_DASHBOARD_NODE)
    controls = re.search(r"_CONTROLS = \((.*?)\)", src, re.S).group(1)
    assert '"roam"' in controls


def test_roam_command_refuses_a_non_bool_before_any_set_and_shares_the_aggregation():
    src = _read(_DASHBOARD_NODE)
    body = src[src.index('if command == "roam":'):src.index('if command == "goal_map":')]
    assert "isinstance(on, bool)" in body
    assert body.index('"ok": False') < body.index("self._set_on_policies(")
    assert "self._retarget_targets()" in body
    assert '_bool_param("roam", on)' in body
    for forbidden in ("_reset_client", "_safety_params", "/safety/reset", "command_source"):
        assert forbidden not in body, forbidden
    assert "def _bool_param(" in src and "PARAMETER_BOOL" in src
    retarget = src[src.index("def _retarget("):src.index("def _snapshot(")]
    assert "self._set_on_policies(" in retarget


def test_page_has_the_roam_button_and_pill_fed_by_the_debug_stream():
    page = _read(_INDEX)
    assert 'id="roambtn"' in page and 'id="roamstat"' in page
    body = page[page.index("function renderRoam(snap)"):]
    body = body[:body.index("\n}\n") + 3]
    for needle in ("ap.debug", "r.on", "r.n", "r.timeouts", "r.last_reason"):
        assert needle in body, needle
    render = page[page.index("function render(snap)"):]
    assert "renderRoam(snap);" in render[:render.index("\n}\n")]
    assert re.search(r'\$\("roambtn"\)\.disabled\s*=\s*!en \|\| !svc\.policy_params', page)
    # The page reads the stream; the server adds no copy to the snapshot.
    node = _read(_DASHBOARD_NODE)
    start = node.index("def _snapshot(")
    assert "roam" not in node[start:node.index("\n    def ", start + 10)]


def test_roam_asks_before_enabling_only_and_the_stop_buttons_still_never_ask():
    page = _read(_INDEX)
    handler = page[page.index('$("roambtn").onclick'):]
    handler = handler[:handler.index("\n};") + 3]
    assert "if(on && !confirm(" in handler
    assert handler.count("confirm(") == 1
    assert 'post("roam",{on})' in handler
    assert re.search(r'\$\("killbtn"\)\.onclick\s*=\s*\(\)\s*=>\s*post\("kill"\)', page)
    assert '$("wpcancel").onclick = () => post("waypoint_cancel");' in page


# ---------------------------------------------------------------------------
# docs
# ---------------------------------------------------------------------------

def _relative_links(path: str):
    text = _read(path)
    for target in re.findall(r"\]\(([^)\s]+)\)", text):
        if "://" in target or target.startswith("#") or target.startswith("mailto:"):
            continue
        yield target.split("#")[0]


def _doc_files():
    yield os.path.join(_REPO_ROOT, "README.md")
    for name in sorted(os.listdir(_DOCS)):
        if name.endswith(".md"):
            yield os.path.join(_DOCS, name)
    src = os.path.join(_REPO_ROOT, "src")
    for pkg in sorted(os.listdir(src)):
        readme = os.path.join(src, pkg, "README.md")
        if os.path.isfile(readme):
            yield readme


def test_roam_doc_exists_is_indexed_and_under_the_cap():
    doc = os.path.join(_DOCS, "roam.md")
    assert os.path.isfile(doc)
    assert "](roam.md)" in _read(os.path.join(_DOCS, "README.md"))
    text = _read(doc)
    assert len(text.split()) <= 600
    assert "—" not in text


def test_every_relative_doc_link_resolves():
    broken = []
    for doc in _doc_files():
        for target in _relative_links(doc):
            if not os.path.exists(os.path.normpath(os.path.join(os.path.dirname(doc), target))):
                broken.append(f"{os.path.relpath(doc, _REPO_ROOT)} -> {target}")
    assert not broken, broken


def test_roam_is_reachable_from_the_operator_docs():
    docs = [os.path.join(_DOCS, "operating_guide.md")] + [
        os.path.join(_REPO_ROOT, "src", pkg, "README.md")
        for pkg in ("rm_policy", "rm_bringup", "rm_dashboard")
    ]
    for doc in docs:
        targets = [
            os.path.normpath(os.path.join(os.path.dirname(doc), t))
            for t in _relative_links(doc)
        ]
        assert os.path.join(_DOCS, "roam.md") in targets, doc
