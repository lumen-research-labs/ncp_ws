"""
Unit tests for rm_base.wheel_control — the closed-loop wheel speed controller.

Pure python, no ROS runtime. These cover the three guards that exist because
the STM32 firmware's own loop railed the PWM on this robot: anti-windup, stall
detection, and the encoder-polarity tripwire. A regression in any of them
reproduces the exact runaway rm_base was built to escape.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "src" / "rm_base"))

from rm_base.pwm_map import WheelPwmModel  # noqa: E402
from rm_base.wheel_control import (  # noqa: E402
    EncoderTracker,
    Guards,
    PiGains,
    WheelController,
)

DT = 0.05  # 20 Hz control tick


def make(model=None, gains=None, guards=None) -> WheelController:
    return WheelController(
        model=model or WheelPwmModel(dead_pwm=2.0, gain_pwm_per_mps=128.0,
                                     v_min_mps=0.02, max_pwm=60),
        gains=gains or PiGains(kp=60.0, ki=80.0, i_max_pwm=25.0),
        guards=guards or Guards(),
    )


# ---- baseline behaviour ---------------------------------------------------

def test_zero_target_commands_zero_and_resets():
    ctrl = make()
    ctrl.update(0.2, 0.05, DT, feedback_valid=True)
    assert ctrl._integral != 0.0
    out = ctrl.update(0.0, 0.0, DT, feedback_valid=True)
    assert out.pwm == 0
    assert out.integral_pwm == 0.0
    # Windup must not survive a stop into the next move.
    assert ctrl._integral == 0.0


def test_invalid_feedback_degrades_to_feedforward():
    ctrl = make()
    out = ctrl.update(0.15, 0.0, DT, feedback_valid=False)
    assert out.pwm == ctrl.model.to_pwm(0.15)
    assert out.integral_pwm == 0.0
    assert not out.stalled and not out.sign_fault


def test_zero_error_holds_feedforward():
    ctrl = make()
    out = ctrl.update(0.15, 0.15, DT, feedback_valid=True)
    assert out.pwm == ctrl.model.to_pwm(0.15)


def test_slow_wheel_gets_more_pwm_than_feedforward():
    """The whole point: a wheel dragging behind is pushed harder."""
    ctrl = make()
    ff = ctrl.model.to_pwm(0.15)
    last = None
    for _ in range(10):
        last = ctrl.update(0.15, 0.10, DT, feedback_valid=True)
    assert last.pwm > ff
    assert last.integral_pwm > 0.0


def test_fast_wheel_gets_less_pwm_than_feedforward():
    ctrl = make()
    ff = ctrl.model.to_pwm(0.15)
    last = None
    for _ in range(10):
        last = ctrl.update(0.15, 0.20, DT, feedback_valid=True)
    assert last.pwm < ff


def test_converges_when_the_wheel_reaches_the_target():
    """Error -> 0 leaves the trim standing rather than unwinding it."""
    ctrl = make()
    for _ in range(10):
        ctrl.update(0.15, 0.10, DT, feedback_valid=True)
    trim = ctrl._integral
    for _ in range(10):
        out = ctrl.update(0.15, 0.15, DT, feedback_valid=True)
    assert out.integral_pwm == pytest.approx(trim)


# ---- anti-windup ----------------------------------------------------------

def test_integral_is_clamped_to_i_max():
    ctrl = make(gains=PiGains(kp=60.0, ki=80.0, i_max_pwm=25.0))
    for _ in range(500):
        out = ctrl.update(0.15, 0.14, DT, feedback_valid=True)
    assert abs(out.integral_pwm) <= 25.0 + 1e-9


def test_output_never_exceeds_max_pwm():
    ctrl = make()
    for _ in range(500):
        out = ctrl.update(0.3, -0.3, DT, feedback_valid=True)
        assert abs(out.pwm) <= ctrl.model.max_pwm


def test_saturated_output_stops_integrating_further():
    """The firmware's failure was integrating while already at the rail."""
    ctrl = make(guards=Guards(stall_measured_mps=0.0))  # disable stall guard
    for _ in range(20):
        ctrl.update(0.4, 0.05, DT, feedback_valid=True)
    saturated_integral = ctrl._integral
    for _ in range(200):
        out = ctrl.update(0.4, 0.05, DT, feedback_valid=True)
    assert out.saturated
    assert ctrl._integral == pytest.approx(saturated_integral)


def test_saturation_still_unwinds_when_the_error_reverses():
    ctrl = make(guards=Guards(stall_measured_mps=0.0))
    for _ in range(50):
        ctrl.update(0.4, 0.05, DT, feedback_valid=True)
    high = ctrl._integral
    for _ in range(20):
        ctrl.update(0.10, 0.30, DT, feedback_valid=True)
    assert ctrl._integral < high


# ---- stall guard ----------------------------------------------------------

def test_stall_freezes_the_integrator():
    """bernard-1's front-right: commanded, but its encoder reads nothing."""
    ctrl = make()
    guards = ctrl.guards
    for _ in range(guards.stall_ticks):
        out = ctrl.update(0.15, 0.0, DT, feedback_valid=True)
    assert out.stalled
    frozen = ctrl._integral
    for _ in range(100):
        out = ctrl.update(0.15, 0.0, DT, feedback_valid=True)
    assert out.stalled
    assert ctrl._integral == pytest.approx(frozen)
    assert abs(out.pwm) <= ctrl.model.max_pwm


def test_stall_clears_once_the_wheel_turns():
    ctrl = make()
    for _ in range(10):
        ctrl.update(0.15, 0.0, DT, feedback_valid=True)
    out = ctrl.update(0.15, 0.12, DT, feedback_valid=True)
    assert not out.stalled


def test_stall_guard_ignores_wheels_commanded_slowly():
    ctrl = make(guards=Guards(stall_target_mps=0.20))
    for _ in range(50):
        out = ctrl.update(0.15, 0.0, DT, feedback_valid=True)
    assert not out.stalled


# ---- sign guard -----------------------------------------------------------

def test_reverse_rotation_latches_a_sign_fault_and_zeroes_output():
    ctrl = make()
    for _ in range(ctrl.guards.sign_fault_ticks):
        out = ctrl.update(0.15, -0.15, DT, feedback_valid=True)
    assert out.sign_fault
    assert out.pwm == 0
    assert ctrl.sign_fault_latched


def test_sign_fault_stays_latched_after_the_feedback_looks_sane_again():
    ctrl = make()
    for _ in range(ctrl.guards.sign_fault_ticks):
        ctrl.update(0.15, -0.15, DT, feedback_valid=True)
    out = ctrl.update(0.15, 0.15, DT, feedback_valid=True)
    assert out.sign_fault
    assert out.pwm == 0
    ctrl.reset()
    assert not ctrl.sign_fault_latched
    assert ctrl.update(0.15, 0.15, DT, feedback_valid=True).pwm > 0


def test_small_reverse_reading_is_noise_not_a_sign_fault():
    """One count of encoder quantisation must not zero a wheel."""
    ctrl = make()
    for _ in range(50):
        out = ctrl.update(0.15, -0.004, DT, feedback_valid=True)
    assert not out.sign_fault


def test_a_brief_reverse_blip_does_not_latch():
    ctrl = make()
    for _ in range(ctrl.guards.sign_fault_ticks - 1):
        ctrl.update(0.15, -0.15, DT, feedback_valid=True)
    ctrl.update(0.15, 0.15, DT, feedback_valid=True)
    assert not ctrl.sign_fault_latched


# ---- encoder tracker ------------------------------------------------------

def tracker() -> EncoderTracker:
    return EncoderTracker(counts_per_rev=1320.0, wheel_circumference_m=0.204)


def test_first_sample_is_never_fresh():
    _speeds, fresh, _dt = tracker().update([0, 0, 0, 0], DT)
    assert not fresh


def test_forward_rotation_is_positive_despite_decreasing_counts():
    """The firmware sign bug, corrected: forward makes the count go DOWN."""
    trk = tracker()
    trk.update([0, 0, 0, 0], DT)
    speeds, fresh, window = trk.update([-100, -100, -100, -100], DT)
    assert fresh
    assert window == pytest.approx(DT)
    expected = 100 * (0.204 / 1320.0) / DT
    assert speeds[0] == pytest.approx(expected)
    assert all(s > 0 for s in speeds)


def test_unchanged_counts_are_not_a_zero_measurement():
    """Polling faster than the board reports must not fabricate a zero."""
    trk = tracker()
    trk.update([0, 0, 0, 0], DT)
    trk.update([-100, -100, -100, -100], DT)
    speeds, fresh, _ = trk.update([-100, -100, -100, -100], DT)
    assert not fresh
    assert speeds[0] > 0  # holds the last real measurement


def test_dt_accumulates_across_repeated_samples():
    trk = tracker()
    trk.update([0, 0, 0, 0], DT)
    trk.update([0, 0, 0, 0], DT)  # no change: window grows
    _speeds, fresh, window = trk.update([-100, -100, -100, -100], DT)
    assert fresh
    assert window == pytest.approx(2 * DT)


def test_implausible_jump_is_rejected():
    trk = tracker()
    trk.update([0, 0, 0, 0], DT)
    trk.update([-100, -100, -100, -100], DT)
    good = trk._last
    speeds, fresh, _ = trk.update([-9_000_000, 0, 0, 0], DT)
    assert not fresh
    assert speeds == good


def test_commanded_reversal_does_not_latch_while_the_wheel_coasts_down():
    """The bug that latched all four wheels on the first full-stack run.

    The debug policy alternates cruise (all wheels forward) with spin (left
    and right opposed), so a wheel's target flips sign constantly. Right after
    the flip the wheel is still coasting the OLD way while the new target
    points the other — genuinely "moving against the command", but from
    momentum, not wrong polarity. The guard must ride that out.
    """
    ctrl = make()
    for _ in range(20):                      # settled, driving forward
        ctrl.update(0.15, 0.15, DT, feedback_valid=True)

    # Command reverses; the wheel decays through zero over ~0.5 s.
    speed = 0.15
    for _ in range(10):
        speed -= 0.03
        out = ctrl.update(-0.10, speed, DT, feedback_valid=True)
        assert not out.sign_fault, f"latched while coasting at {speed:+.3f}"
    assert not ctrl.sign_fault_latched


def test_repeated_reversals_never_latch():
    """Cruise/spin alternation must survive indefinitely."""
    ctrl = make()
    for cycle in range(8):
        target = 0.15 if cycle % 2 == 0 else -0.15
        speed = -target          # starts fully reversed vs the new target
        for _ in range(8):
            speed += target * 0.25
            ctrl.update(target, speed, DT, feedback_valid=True)
    assert not ctrl.sign_fault_latched


def test_genuine_polarity_inversion_still_latches_promptly():
    """The guard must keep doing its job: inverted feedback is sustained.

    Unlike a reversal, a real polarity fault reports the wrong direction on
    every tick forever, so it trips well inside a second at 20 Hz.
    """
    ctrl = make()
    for i in range(ctrl.guards.sign_fault_ticks):
        out = ctrl.update(0.15, -0.15, DT, feedback_valid=True)
    assert out.sign_fault
    assert out.pwm == 0
    # 20 Hz: must not take more than a second to notice.
    assert ctrl.guards.sign_fault_ticks * DT <= 1.0
