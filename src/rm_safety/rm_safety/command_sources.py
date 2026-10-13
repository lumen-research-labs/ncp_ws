"""
command_sources.py — the table of command sources rm_safety may forward,
loaded and validated from config/topics.yaml. Pure python, no ROS.

What it does:
    Reads the `command_sources` mapping of config/topics.yaml into a dict of
    `CommandSource` rows (name -> topic, kind, node, debug, description) and
    refuses any table that could route around the safety layer or that the
    rest of the stack could misread:

      * a row whose topic or debug stream is /cmd_vel (the table's own
        base_driver.cmd_vel): an input that IS the output, or a second
        publisher on it;
      * two rows on one topic, one debug stream, or one node name, and a
        YAML document that names one row twice (PyYAML would otherwise keep
        the last silently);
      * no `teleop` row, or a `teleop` row that is not of kind teleop (it is
        the startup default and the direction every switch falls back to);
      * a kind other than "teleop" / "policy";
      * a `node` on a row that is not a policy row, or that has no `debug`
        (the dashboard re-targets by node and follows by debug stream);
      * malformed names, topics or rows.

    This module lives in rm_safety because the safety node is the table's
    authority: what it accepts as `command_source` is exactly these names.
    rm_bringup's launch and rm_dashboard import it so the three can never
    disagree about a name or a topic. The dependency runs one way only; the
    safety node imports nothing of theirs.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    `load_command_sources` raises ValueError (or the open/parse error) on any
    problem. Callers treat that as fatal at startup: a node that cannot name
    its sources must not start, because the alternative is accepting a
    `command_source` value and forwarding nothing, the silent failure this
    project already hit once (CLAUDE.md, 2026-08-29).
"""

from __future__ import annotations

import dataclasses
import pathlib
import re
from typing import Any, Dict, Optional

import yaml

# Default table path, valid in a standard colcon workspace layout where the
# package source tree is adjacent to config/ (the same resolution
# safety_node uses for safety_limits.yaml). Launch files pass it explicitly.
_THIS_DIR = pathlib.Path(__file__).parent.resolve()
DEFAULT_TOPICS_PATH = str(_THIS_DIR.parents[2] / "config" / "topics.yaml")

KIND_TELEOP = "teleop"
KIND_POLICY = "policy"
KINDS = (KIND_TELEOP, KIND_POLICY)

# The row that must exist: rm_safety's startup default and the operator path.
TELEOP_SOURCE = "teleop"

# The one topic no row may name. rm_safety is its sole publisher.
BASE_CMD_VEL_TOPIC = "/cmd_vel"

_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


class _StrictLoader(yaml.SafeLoader):
    """SafeLoader that refuses a mapping with a repeated key.

    PyYAML keeps the LAST of two `m4:` rows and says nothing. For a table
    whose rows decide what the safety layer forwards, that silence is the
    failure mode this loader exists to remove.
    """

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key_node, _value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if key in seen:
                raise ValueError(
                    f"topics.yaml: key {key!r} appears twice in one mapping "
                    f"(line {key_node.start_mark.line + 1})"
                )
            seen.add(key)
        return super().construct_mapping(node, deep=deep)
_TOPIC_RE = re.compile(r"^/[A-Za-z0-9_/]+$")
_NODE_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


@dataclasses.dataclass(frozen=True)
class CommandSource:
    """One row of the table. `node` and `debug` are None where not given."""

    name: str
    topic: str
    kind: str
    node: Optional[str] = None
    debug: Optional[str] = None
    description: str = ""

    @property
    def is_policy(self) -> bool:
        return self.kind == KIND_POLICY


def parse_command_sources(raw: Any) -> Dict[str, CommandSource]:
    """Validate a parsed topics.yaml document. Raises ValueError, never defaults."""
    if not isinstance(raw, dict) or not isinstance(raw.get("command_sources"), dict):
        raise ValueError("topics.yaml: `command_sources` must be a mapping of name -> row")
    table = raw["command_sources"]
    if not table:
        raise ValueError("topics.yaml: `command_sources` is empty")
    # The output topic, from the same document, so the two cannot drift.
    # It must still BE /cmd_vel: rm_safety publishes that name, and a table
    # claiming otherwise would let a row take the real one.
    base = (raw.get("base_driver") or {}).get("cmd_vel", BASE_CMD_VEL_TOPIC)
    if base != BASE_CMD_VEL_TOPIC:
        raise ValueError(
            f"topics.yaml: base_driver.cmd_vel is {base!r}; rm_safety publishes "
            f"{BASE_CMD_VEL_TOPIC} and nothing else"
        )

    out: Dict[str, CommandSource] = {}
    for name, row in table.items():
        if not isinstance(name, str) or not _NAME_RE.match(name):
            raise ValueError(
                f"topics.yaml: command source name {name!r} must match "
                f"{_NAME_RE.pattern} (it is a parameter value and a node-name suffix)"
            )
        if not isinstance(row, dict):
            raise ValueError(f"topics.yaml: command source {name!r} must be a mapping")
        topic = row.get("topic")
        kind = row.get("kind")
        node = row.get("node")
        debug = row.get("debug")
        description = row.get("description", "")
        if not isinstance(topic, str) or not _TOPIC_RE.match(topic):
            raise ValueError(f"topics.yaml: {name}.topic must be an absolute topic, got {topic!r}")
        if topic == BASE_CMD_VEL_TOPIC:
            raise ValueError(
                f"topics.yaml: {name}.topic is {BASE_CMD_VEL_TOPIC}, which rm_safety "
                f"PUBLISHES. An input on the output topic would feed the safety "
                f"layer its own commands."
            )
        if kind not in KINDS:
            raise ValueError(f"topics.yaml: {name}.kind must be one of {KINDS}, got {kind!r}")
        if node is not None and (not isinstance(node, str) or not _NODE_RE.match(node)):
            raise ValueError(f"topics.yaml: {name}.node must be a bare node name, got {node!r}")
        if debug is not None and (not isinstance(debug, str) or not _TOPIC_RE.match(debug)):
            raise ValueError(f"topics.yaml: {name}.debug must be an absolute topic, got {debug!r}")
        if debug == BASE_CMD_VEL_TOPIC:
            raise ValueError(
                f"topics.yaml: {name}.debug is {BASE_CMD_VEL_TOPIC}; a debug stream "
                f"there would be a second publisher on the output topic"
            )
        if node is not None and kind != KIND_POLICY:
            raise ValueError(f"topics.yaml: {name} has a node but is not a policy row")
        if node is not None and debug is None:
            raise ValueError(
                f"topics.yaml: {name} has a node but no debug stream; the dashboard "
                f"follows a policy by its debug topic"
            )
        if not isinstance(description, str):
            raise ValueError(f"topics.yaml: {name}.description must be a string")
        out[name] = CommandSource(
            name=name, topic=topic, kind=kind, node=node, debug=debug,
            description=description,
        )

    teleop = out.get(TELEOP_SOURCE)
    if teleop is None or teleop.kind != KIND_TELEOP:
        raise ValueError(
            f"topics.yaml: a {TELEOP_SOURCE!r} row of kind {KIND_TELEOP!r} is required; "
            f"it is rm_safety's startup default"
        )

    _refuse_duplicates("topic", [s.topic for s in out.values()])
    _refuse_duplicates("debug", [s.debug for s in out.values() if s.debug])
    _refuse_duplicates("node", [s.node for s in out.values() if s.node])
    shared = {s.topic for s in out.values()} & {s.debug for s in out.values() if s.debug}
    if shared:
        raise ValueError(f"topics.yaml: {sorted(shared)} used as both a command and a debug topic")
    return out


def load_command_sources(path: str = DEFAULT_TOPICS_PATH) -> Dict[str, CommandSource]:
    """Load and validate the table from `path`. Raises on any problem."""
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.load(fh, Loader=_StrictLoader)
    return parse_command_sources(raw)


def _refuse_duplicates(field: str, values) -> None:
    seen = set()
    for v in values:
        if v in seen:
            raise ValueError(f"topics.yaml: {field} {v!r} appears on two command sources")
        seen.add(v)
