"""One policy node per profile, from the command-source table.

ncp.launch.py's `profiles:=` runs every listed trained policy at once, each
publishing its own request on its own config/topics.yaml row; rm_safety
forwards the row named by command_source. These pins tie the launch, the two
launch files it hands the table to, and the recording script to that table,
as text, because importing a launch file needs launch_ros.

Pure stdlib + yaml: workstation tier.
"""

from __future__ import annotations

import os
import re
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_safety"))

from rm_safety.command_sources import load_command_sources  # noqa: E402

_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "ncp.launch.py")
_SAFETY_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "safety.launch.py")
_DASH_LAUNCH = os.path.join(_REPO_ROOT, "src", "rm_bringup", "launch", "dashboard.launch.py")
_RECORD = os.path.join(_REPO_ROOT, "scripts", "record_run.sh")
_TOPICS = os.path.join(_REPO_ROOT, "config", "topics.yaml")


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _profiles(src: str) -> dict:
    table = src[src.index("_PROFILES = {"):]
    table = table[:table.index("}")]
    return dict(re.findall(r'"(\w+)": \("([^"]+)", "[^"]+"\)', table))


def test_default_runs_both_policies() -> None:
    src = _read(_LAUNCH)
    assert '_DEFAULT_PROFILES = "m8v2,m4,m9"' in src
    assert '"profiles", default_value=_DEFAULT_PROFILES' in src
    assert 'LaunchConfiguration("profile")' not in src, "profile:= was replaced by profiles:="


def test_profiles_and_table_rows_with_nodes_are_the_same_set() -> None:
    """A profile is a checkpoint + contract; its table row is its node name,
    command topic and debug stream. Neither may exist without the other."""
    profiles = set(_profiles(_read(_LAUNCH)))
    with_node = {n for n, s in load_command_sources(_TOPICS).items() if s.node}
    assert profiles == with_node == {"m8v2", "m4", "m9"}


def test_policy_nodes_are_named_and_topiced_from_the_table() -> None:
    src = _read(_LAUNCH)
    assert "from rm_safety.command_sources import load_command_sources" in src
    assert "_SOURCES = load_command_sources(_DEFAULT_TOPICS_PATH)" in src
    body = src[src.index("def _policy_consumer("):src.index("def generate_launch_description(")]
    assert "for _name, source, interface, weights in profiles:" in body
    assert '"cmd_vel_topic": source.topic' in body
    assert '"debug_topic": source.debug' in body
    assert "name=source.node" in body
    # The bridge serves the first listed profile's contract.
    assert "profiles[0]" in body and '"interface_config_path": first_interface' in body
    # No topic literal survives outside the table.
    assert '"/policy/cmd_vel"' not in src and '"/ncp_debug"' not in src
    assert "_POLICY_CMD_TOPIC" not in src


def test_resolver_refuses_bad_lists_and_multi_profile_overrides() -> None:
    body = _read(_LAUNCH)
    body = body[body.index("def _resolve_profiles("):body.index("def _policy_consumer(")]
    for needle in ("at least one", "not in", "twice", "override ONE profile",
                   "no command_sources row with a node"):
        assert needle in body, needle


def test_launch_refuses_a_command_source_whose_node_it_does_not_start() -> None:
    src = _read(_LAUNCH)
    body = src[src.index("def _policy_consumer("):src.index("def generate_launch_description(")]
    assert 'LaunchConfiguration("command_source")' in body
    assert "selected not in started" in body
    assert body.index("selected not in started") < body.index("Node(")


def test_upstream_consumer_needs_m8v2_first_and_publishes_on_the_policy_row() -> None:
    src = _read(_LAUNCH)
    body = src[src.index("def _upstream_process("):src.index("def _resolve_profiles(")]
    assert "_resolve_profiles(context)[0]" in body
    assert "if first != _DEFAULT_PROFILE:" in body
    assert 'cmd_topic:={_SOURCES[_UPSTREAM_SOURCE].topic}' in body
    assert '_UPSTREAM_SOURCE = "policy"' in src
    assert load_command_sources(_TOPICS)["policy"].node is None


def test_the_table_path_reaches_safety_and_dashboard() -> None:
    src = _read(_LAUNCH)
    assert src.count('"topics_path": _DEFAULT_TOPICS_PATH') == 2
    for path in (_SAFETY_LAUNCH, _DASH_LAUNCH):
        launch = _read(path)
        assert 'DeclareLaunchArgument(' in launch and '"topics_path"' in launch, path
        assert '"topics_path": ' in launch[launch.index("parameters=["):], path


def test_record_run_records_every_source_and_debug_topic() -> None:
    src = _read(_RECORD)
    block = src[src.index("TOPICS=("):]
    block = block[:block.index(")")]
    recorded = set(block.split()[1:])
    for row in load_command_sources(_TOPICS).values():
        assert row.topic in recorded, row.topic
        if row.debug:
            assert row.debug in recorded, row.debug
    assert "/cmd_vel" in recorded and "/safety/status" in recorded
