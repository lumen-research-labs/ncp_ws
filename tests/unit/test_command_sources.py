"""The command-source table: config/topics.yaml, its loader, and rm_safety's
use of it.

rm_safety forwards exactly one of the table's rows, named by `command_source`.
The loader is what stands between a typo in the YAML and a safety node that
accepts a source name and forwards nothing (the silent failure of
2026-08-29), or that takes its own /cmd_vel as an input. The safety-node
half runs against the real module through the same ROS stubs
test_safety_node_zone_switches installs.

Pure stdlib + yaml: workstation tier.
"""

from __future__ import annotations

import copy
import os
import sys

import pytest
import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("rm_safety",):
    _p = os.path.join(_REPO_ROOT, "src", _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from rm_safety.command_sources import (  # noqa: E402
    BASE_CMD_VEL_TOPIC, DEFAULT_TOPICS_PATH, KIND_POLICY, KIND_TELEOP,
    TELEOP_SOURCE, CommandSource, load_command_sources, parse_command_sources,
)
from test_safety_node_zone_switches import _Log, sn  # noqa: E402

_TOPICS = os.path.join(_REPO_ROOT, "config", "topics.yaml")
_SAFETY_NODE = os.path.join(_REPO_ROOT, "src", "rm_safety", "rm_safety", "safety_node.py")


def _raw() -> dict:
    with open(_TOPICS, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# --- the real table ------------------------------------------------------------

def test_default_path_is_the_repo_table() -> None:
    assert os.path.realpath(DEFAULT_TOPICS_PATH) == os.path.realpath(_TOPICS)


def test_real_table_rows() -> None:
    """The five rows the stack is built around, pinned field by field."""
    t = load_command_sources(_TOPICS)
    assert list(t) == ["teleop", "policy", "m8v2", "m4", "m9"]
    assert t["teleop"] == CommandSource(
        "teleop", "/teleop/cmd_vel", KIND_TELEOP, None, None, t["teleop"].description)
    assert t["policy"].topic == "/policy/cmd_vel"
    assert t["policy"].kind == KIND_POLICY
    assert t["policy"].node is None and t["policy"].debug == "/ncp_debug"
    assert t["m8v2"] == CommandSource(
        "m8v2", "/policy/m8v2/cmd_vel", KIND_POLICY, "policy_node_m8v2",
        "/ncp_debug/m8v2", t["m8v2"].description)
    assert t["m4"] == CommandSource(
        "m4", "/policy/m4/cmd_vel", KIND_POLICY, "policy_node_m4",
        "/ncp_debug/m4", t["m4"].description)
    assert t["m9"] == CommandSource(
        "m9", "/policy/m9/cmd_vel", KIND_POLICY, "policy_node_m9",
        "/ncp_debug/m9", t["m9"].description)
    for row in t.values():
        assert row.description, f"{row.name} needs a description (the button tooltip)"
    assert [r.is_policy for r in t.values()] == [False, True, True, True, True]


def test_no_row_names_the_output_topic_and_topics_are_unique() -> None:
    t = load_command_sources(_TOPICS)
    topics = [r.topic for r in t.values()]
    assert BASE_CMD_VEL_TOPIC not in topics
    assert len(set(topics)) == len(topics)
    assert _raw()["base_driver"]["cmd_vel"] == BASE_CMD_VEL_TOPIC


# --- refusals ------------------------------------------------------------------

def _without(name):
    raw = _raw()
    del raw["command_sources"][name]
    return raw


def _with(name, **fields):
    raw = _raw()
    row = raw["command_sources"].setdefault(name, {})
    for k, v in fields.items():
        if v is None:
            row.pop(k, None)
        else:
            row[k] = v
    return raw


@pytest.mark.parametrize("raw, needle", [
    (None, "mapping"),
    ({"command_sources": []}, "mapping"),
    ({"command_sources": {}}, "empty"),
    (_without("teleop"), "'teleop'"),
    (_with("teleop", kind="policy"), "'teleop'"),
    (_with("m4", topic="/cmd_vel"), "PUBLISHES"),
    (_with("m4", topic="/policy/m8v2/cmd_vel"), "appears on two"),
    (_with("m4", debug="/ncp_debug/m8v2"), "appears on two"),
    (_with("m4", node="policy_node_m8v2"), "appears on two"),
    (_with("m4", kind="autopilot"), "kind"),
    (_with("m4", debug=None), "no debug"),
    (_with("teleop", node="x", debug="/d"), "not a policy row"),
    (_with("m4", topic="policy/m4/cmd_vel"), "absolute topic"),
    (_with("m4", topic="/policy m4"), "absolute topic"),
    (_with("m4", debug="ncp_debug"), "absolute topic"),
    (_with("m4", node="/policy_node_m4"), "bare node name"),
    (_with("M4", topic="/x", kind="policy", debug="/y"), "name"),
    (_with("m4", topic="/ncp_debug/m8v2"), "both a command and a debug"),
    (_with("m4", debug="/cmd_vel"), "second publisher"),
    ({**_raw(), "base_driver": {"cmd_vel": "/base/cmd_vel"}}, "base_driver.cmd_vel"),
    ({**_raw(), "command_sources": {**_raw()["command_sources"], "m4": "string"}}, "mapping"),
])
def test_malformed_tables_are_refused(raw, needle) -> None:
    with pytest.raises(ValueError) as exc:
        parse_command_sources(copy.deepcopy(raw))
    assert needle in str(exc.value)


def test_a_row_named_twice_is_refused_not_last_wins(tmp_path) -> None:
    """PyYAML keeps the last duplicate key silently; the loader must not."""
    text = open(_TOPICS, "r", encoding="utf-8").read()
    dup = text.replace("  m4:\n", "  m4:\n    topic: /policy/other/cmd_vel\n    kind: policy\n  m4:\n", 1)
    assert dup != text
    path = tmp_path / "topics.yaml"
    path.write_text(dup, encoding="utf-8")
    with pytest.raises(ValueError) as exc:
        load_command_sources(str(path))
    assert "'m4' appears twice" in str(exc.value)


def test_missing_file_raises() -> None:
    with pytest.raises(OSError):
        load_command_sources(os.path.join(_REPO_ROOT, "config", "does_not_exist.yaml"))


# --- rm_safety's use of it ---------------------------------------------------------

class _P:
    def __init__(self, name, value):
        self.name = name
        self.value = value


def _node(source: str = TELEOP_SOURCE):
    """A SafetyNode with no ROS lifecycle: the table, the buffers, the switch."""
    n = object.__new__(sn.SafetyNode)
    n._sources = load_command_sources(_TOPICS)
    n._commands = {name: sn._CommandBuffer() for name in n._sources}
    n._command_source = source
    n._is_killed = False
    n._kill_writes = []
    n._write_kill_state = lambda killed: n._kill_writes.append(killed)
    n._log = _Log()
    n.get_logger = lambda: n._log
    return n


def test_safety_node_module_has_the_table_plumbing() -> None:
    for attr in ("_load_sources", "_command_cb", "_select_command"):
        assert hasattr(sn.SafetyNode, attr), attr
    assert sn._DEFAULT_COMMAND_SOURCE == TELEOP_SOURCE
    assert sn._CommandBuffer().stamp is None, "never published = infinitely stale"


@pytest.mark.parametrize("target", ["m4", "m8v2", "policy"])
def test_switching_to_a_table_row_is_accepted_and_force_kills(target) -> None:
    n = _node("teleop")
    result = n._on_set_parameters([_P("command_source", target)])
    assert result.successful is True
    assert n._command_source == target
    assert n._is_killed is True
    assert n._kill_writes == [True]
    assert any("KILLED" in m for m in n._log.warns)


def test_switching_to_an_unknown_name_is_rejected_and_names_the_rows() -> None:
    n = _node("teleop")
    result = n._on_set_parameters([_P("command_source", "m99")])
    assert result.successful is False
    for name in ("teleop", "policy", "m8v2", "m4", "m9"):
        assert f"'{name}'" in result.reason
    assert "topics.yaml" in result.reason
    assert n._command_source == "teleop"
    assert n._is_killed is False and n._kill_writes == []


def test_a_rejected_parameter_in_the_same_set_leaves_the_source_untouched() -> None:
    """rcl applies a multi-parameter set as a whole, so the node must too:
    a valid command_source followed by a rejected parameter must change
    nothing, or the forwarded source and the readable one would disagree."""
    n = _node("teleop")
    result = n._on_set_parameters([
        _P("command_source", "m4"), _P("stop_zone_enabled", False),
    ])
    assert result.successful is False
    assert n._command_source == "teleop"
    assert n._is_killed is False and n._kill_writes == []

    result = n._on_set_parameters([
        _P("command_source", "m4"), _P("command_source", "bogus"),
    ])
    assert result.successful is False
    assert n._command_source == "teleop" and n._kill_writes == []

    # And a set that is valid throughout applies the LAST value, once.
    result = n._on_set_parameters([
        _P("command_source", "m8v2"), _P("command_source", "m4"),
    ])
    assert result.successful is True
    assert n._command_source == "m4" and n._kill_writes == [True]


def test_reselecting_the_current_row_does_not_kill() -> None:
    n = _node("m8v2")
    result = n._on_set_parameters([_P("command_source", "m8v2")])
    assert result.successful is True
    assert n._is_killed is False and n._kill_writes == []


class _Twist:
    def __init__(self, vx, vy, wz):
        class _V:
            pass
        self.linear = _V()
        self.linear.x, self.linear.y = vx, vy
        self.angular = _V()
        self.angular.z = wz


def test_each_row_has_its_own_buffer_and_only_the_selected_one_is_read() -> None:
    n = _node("teleop")
    n._command_cb("m4")(_Twist(0.4, 0.0, 0.1))
    n._command_cb("m8v2")(_Twist(1.2, 0.0, 0.36))
    n._command_cb("teleop")(_Twist(0.1, 0.05, -0.2))

    source, cmd = n._select_command()
    assert (source.name, source.is_policy) == ("teleop", False)
    assert (cmd.vx, cmd.vy, cmd.wz) == (0.1, 0.05, -0.2)
    assert cmd.stamp is not None

    n._command_source = "m8v2"
    source, cmd = n._select_command()
    assert (source.name, source.is_policy, source.topic) == (
        "m8v2", True, "/policy/m8v2/cmd_vel")
    assert (cmd.vx, cmd.wz) == (1.2, 0.36)

    n._command_source = "m4"
    source, cmd = n._select_command()
    assert (source.name, cmd.vx, cmd.wz) == ("m4", 0.4, 0.1)

    # A row that never published stays infinitely stale.
    n._command_source = "policy"
    _source, cmd = n._select_command()
    assert cmd.stamp is None and (cmd.vx, cmd.vy, cmd.wz) == (0.0, 0.0, 0.0)


def test_control_loop_reads_through_select_command_only() -> None:
    with open(_SAFETY_NODE, "r", encoding="utf-8") as fh:
        src = fh.read()
    loop = src[src.index("def _control_loop_inner("):src.index("# Entry point")]
    assert "self._select_command()" in loop
    assert "source.is_policy" in loop
    for stale in ("_policy_vx", "_teleop_vx", "_policy_stamp", "_teleop_stamp",
                  '== "policy"', '"/policy/cmd_vel"'):
        assert stale not in src, stale
    # Subscriptions come from the table, one per row.
    assert "for name, source in self._sources.items():" in src
    assert "source.topic, self._command_cb(name)" in src
