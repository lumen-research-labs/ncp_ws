"""Tests for rm_policy.scan_sanitizer_node.sanitize_ranges.

The TG30 encodes a no-return as 0.0 and ~31-39% of rays in a normal room are
no-returns. Consumers we own already know that; slam_toolbox does not, and
whether its min_laser_range discards or clamps a short reading is an
implementation detail of a dependency we would otherwise be betting the pose
estimate on. This function removes the question by rewriting the sentinel to
+inf before any third-party consumer sees it.

Pure stdlib. Workstation tier.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

# Import the pure function WITHOUT importing the ROS node module, which needs
# rclpy. Same pattern the other pure-module tests here use.
import importlib.util  # noqa: E402

_SRC = os.path.join(
    _REPO_ROOT, "src", "rm_policy", "rm_policy", "scan_sanitizer_node.py"
)


def _load_sanitize():
    """Extract sanitize_ranges without executing the rclpy imports."""
    import ast

    tree = ast.parse(open(_SRC).read())
    fn = next(
        n for n in tree.body
        if isinstance(n, ast.FunctionDef) and n.name == "sanitize_ranges"
    )
    ns = {"math": math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), _SRC, "exec"), ns)
    return ns["sanitize_ranges"]


sanitize_ranges = _load_sanitize()

RMIN = 0.01  # what the TG30 driver advertises


def test_the_tg30_zero_sentinel_becomes_inf() -> None:
    """The whole point: 0.0 means "nothing there", not "touching the sensor"."""
    assert sanitize_ranges([0.0], RMIN) == [math.inf]


def test_real_returns_pass_through_untouched() -> None:
    assert sanitize_ranges([0.5, 1.0, 12.75], RMIN) == [0.5, 1.0, 12.75]


def test_nan_becomes_inf() -> None:
    assert sanitize_ranges([float("nan")], RMIN) == [math.inf]


def test_existing_inf_is_preserved() -> None:
    assert sanitize_ranges([math.inf], RMIN) == [math.inf]


def test_negative_ranges_become_inf() -> None:
    assert sanitize_ranges([-1.0, -0.001], RMIN) == [math.inf, math.inf]


def test_below_range_min_becomes_inf() -> None:
    assert sanitize_ranges([0.005], 0.01) == [math.inf]


def test_exactly_range_min_is_kept() -> None:
    """A reading AT the sensor's floor is a real reading, not an error."""
    assert sanitize_ranges([0.01], 0.01) == [0.01]


def test_long_readings_are_NOT_clamped() -> None:
    """range_max is deliberately not applied.

    A reading beyond range_max is a LONG reading, not an invalid one. Clamping
    it here would invent an obstacle at the ceiling distance — the mirror
    image of the 0.0 bug this function exists to fix.
    """
    assert sanitize_ranges([64.0, 900.0], RMIN) == [64.0, 900.0]


def test_ray_count_is_always_preserved() -> None:
    """slam_toolbox's karto latches the first scan's ray count and rejects
    anything different, so a sanitizer that changed the length would break
    the very consumer it exists to serve."""
    raw = [0.0, 1.0, float("nan"), 2.0, -1.0, math.inf, 0.5]
    assert len(sanitize_ranges(raw, RMIN)) == len(raw)


def test_a_realistic_scan_keeps_every_real_return() -> None:
    """~31% zeros, as measured on bernard-1: no real return may be lost."""
    raw = [0.0 if i % 3 == 0 else 1.0 + i * 0.001 for i in range(2020)]
    out = sanitize_ranges(raw, RMIN)
    assert len(out) == 2020
    assert sum(1 for v in out if math.isinf(v)) == sum(1 for v in raw if v == 0.0)
    for i, v in enumerate(raw):
        if v != 0.0:
            assert out[i] == pytest.approx(v)


def test_an_all_dropout_scan_becomes_all_inf_not_all_zero() -> None:
    """Fail toward "I see nothing", never toward "I am surrounded"."""
    out = sanitize_ranges([0.0] * 100, RMIN)
    assert all(math.isinf(v) for v in out)


def test_none_entries_are_tolerated() -> None:
    assert sanitize_ranges([None, 1.0], RMIN) == [math.inf, 1.0]


def test_empty_scan_stays_empty() -> None:
    assert sanitize_ranges([], RMIN) == []
