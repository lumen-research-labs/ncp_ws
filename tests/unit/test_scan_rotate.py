"""rm_policy.scan_rotate_node.rotated_angle_min: a LaserScan turned about the
sensor's axis by shifting angle_min, so a lidar mounted zero-backward reads as
if its zero pointed along the robot's +x (bernard-3's RPLIDAR A1, 2026-11-18).
Pure stdlib, extracted without the rclpy imports. Workstation tier.
"""

from __future__ import annotations

import ast
import math
import os

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_SRC = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "scan_rotate_node.py")


def _rotated_angle_min():
    tree = ast.parse(open(_SRC, encoding="utf-8").read())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "rotated_angle_min")
    ns = {"math": math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), _SRC, "exec"), ns)
    return ns["rotated_angle_min"]


def _ray_angles(angle_min: float, inc: float, n: int):
    return [((angle_min + i * inc) + math.pi) % (2 * math.pi) - math.pi for i in range(n)]


def test_a_half_turn_sends_the_scans_front_to_the_robots_rear() -> None:
    rot = _rotated_angle_min()
    inc = math.radians(1.0)
    before = _ray_angles(-math.pi, inc, 360)
    after = _ray_angles(rot(-math.pi, math.pi), inc, 360)
    # The ray that was dead ahead (0 deg) now reads as dead behind, and the
    # ray at +90 (left) now reads at -90 (right): a rotation, not a mirror.
    i0 = min(range(360), key=lambda i: abs(before[i]))
    assert abs(abs(after[i0]) - math.pi) < 1e-9
    i90 = min(range(360), key=lambda i: abs(before[i] - math.pi / 2))
    assert after[i90] == pytest.approx(-math.pi / 2, abs=1e-9)


def test_angle_min_stays_bounded_and_the_sequence_is_preserved() -> None:
    rot = _rotated_angle_min()
    for yaw_deg in (180.0, 90.0, -90.0, 45.0, 359.0, -720.0 + 30.0):
        a = rot(-math.pi, math.radians(yaw_deg))
        assert -math.pi <= a < math.pi
        # Every ray moves by the same amount: rotation keeps neighbours adjacent.
        inc = math.radians(0.5)
        before = _ray_angles(-math.pi, inc, 720)
        after = _ray_angles(a, inc, 720)
        for i in (0, 100, 359, 719):
            d = ((after[i] - before[i]) - math.radians(yaw_deg) + math.pi) % (2 * math.pi) - math.pi
            assert abs(d) < 1e-9


def test_a_zero_offset_is_the_identity() -> None:
    rot = _rotated_angle_min()
    assert rot(-math.pi, 0.0) == pytest.approx(-math.pi)
    assert rot(0.3, 0.0) == pytest.approx(0.3)
