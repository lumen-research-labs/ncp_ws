"""
test_rm_base_kinematics.py — unit tests for rm_base.mecanum and rm_base.pwm_map.

Pure pytest, no ROS runtime, no Rosmaster_Lib. Guards the two things that
would silently drive the robot wrong: the mecanum sign convention (forward /
left / CCW) and the open-loop PWM dead-band model.
"""
from __future__ import annotations

import math
import os
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_RM_BASE_SRC = os.path.join(_REPO_ROOT, "src", "rm_base")
if _RM_BASE_SRC not in sys.path:
    sys.path.insert(0, _RM_BASE_SRC)

from rm_base import mecanum, pwm_map  # noqa: E402

LEVER = 0.165


# ---- mecanum -----------------------------------------------------------

def test_pure_forward_all_wheels_equal_and_positive():
    w = mecanum.inverse(mecanum.BodyTwist(0.2, 0.0, 0.0), LEVER)
    assert w.as_tuple() == pytest.approx((0.2, 0.2, 0.2, 0.2))


def test_pure_left_strafe_signs():
    # +vy is left: left wheels roll backward, right wheels forward (X rollers)
    w = mecanum.inverse(mecanum.BodyTwist(0.0, 0.1, 0.0), LEVER)
    assert w.front_left < 0 and w.back_left > 0
    assert w.front_right > 0 and w.back_right < 0
    # magnitudes equal
    assert abs(w.front_left) == pytest.approx(0.1)


def test_pure_ccw_yaw_left_side_back_right_side_forward():
    w = mecanum.inverse(mecanum.BodyTwist(0.0, 0.0, 1.0), LEVER)
    assert w.front_left == pytest.approx(-LEVER)
    assert w.back_left == pytest.approx(-LEVER)
    assert w.front_right == pytest.approx(LEVER)
    assert w.back_right == pytest.approx(LEVER)


@pytest.mark.parametrize("vx,vy,wz", [
    (0.15, 0.0, 0.0),
    (0.0, 0.12, 0.0),
    (0.0, 0.0, 0.4),
    (0.1, -0.05, 0.3),
    (-0.08, 0.04, -0.2),
])
def test_forward_inverts_inverse(vx, vy, wz):
    t = mecanum.BodyTwist(vx, vy, wz)
    back = mecanum.forward(mecanum.inverse(t, LEVER), LEVER)
    assert (back.vx, back.vy, back.wz) == pytest.approx((vx, vy, wz))


# ---- pwm_map ---------------------------------------------------------

def test_deadband_returns_zero():
    m = pwm_map.WheelPwmModel(dead_pwm=2.0, gain_pwm_per_mps=128.0, v_min_mps=0.02)
    assert m.to_pwm(0.0) == 0
    assert m.to_pwm(0.01) == 0
    assert m.to_pwm(-0.019) == 0


def test_above_deadband_is_signed_and_offset():
    m = pwm_map.WheelPwmModel(dead_pwm=2.0, gain_pwm_per_mps=128.0)
    assert m.to_pwm(0.1) == round(2.0 + 128.0 * 0.1)      # 15
    assert m.to_pwm(-0.1) == -round(2.0 + 128.0 * 0.1)    # -15


def test_clamped_to_max_pwm():
    m = pwm_map.WheelPwmModel(dead_pwm=40.0, gain_pwm_per_mps=190.0, max_pwm=95)
    assert m.to_pwm(5.0) == 95
    assert m.to_pwm(-5.0) == -95


def test_weak_front_right_needs_more_pwm_than_healthy():
    healthy = pwm_map.WheelPwmModel(dead_pwm=2.0, gain_pwm_per_mps=128.0)
    weak = pwm_map.WheelPwmModel(dead_pwm=40.0, gain_pwm_per_mps=190.0, max_pwm=95)
    assert weak.to_pwm(0.1) > healthy.to_pwm(0.1) + 20


def test_to_speed_is_zero_in_deadband():
    m = pwm_map.WheelPwmModel(dead_pwm=10.0, gain_pwm_per_mps=100.0)
    assert m.to_speed(5) == 0.0
    assert m.to_speed(60) == pytest.approx((60 - 10) / 100.0)


def test_slew_limit_caps_step():
    assert pwm_map.slew_limit(0, 50, 8) == 8
    assert pwm_map.slew_limit(50, 0, 8) == 42
    assert pwm_map.slew_limit(10, 12, 8) == 12
    assert pwm_map.slew_limit(10, 12, 0) == 12  # disabled


def test_slew_limit_reaches_target_over_ticks():
    pwm = 0
    for _ in range(20):
        pwm = pwm_map.slew_limit(pwm, 55, 8)
    assert pwm == 55
