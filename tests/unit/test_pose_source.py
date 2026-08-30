"""Tests for rm_policy.pose_source — pose usability and jump detection.

The jump detector guards a real hazard: `goal_local` is 2 of the 36 numbers
the CfC reads, and a SLAM loop closure moves the goal in the robot frame
instantly. Without a reset the network keeps steering on hidden state built
toward where the goal used to be. So the tests below care about two things in
particular — that a genuine teleport IS caught, and that ordinary driving is
NOT caught, because a detector that fires constantly resets the recurrent
state constantly and is worse than none.

Pure stdlib. Workstation tier.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.pose_source import (  # noqa: E402
    Pose2D,
    detect_pose_jump,
    pose_is_usable,
)


def _p(x=0.0, y=0.0, yaw=0.0, stamp=100.0) -> Pose2D:
    return Pose2D(x=x, y=y, yaw=yaw, stamp=stamp)


# ---------------------------------------------------------------------------
# Usability
# ---------------------------------------------------------------------------

def test_fresh_finite_pose_is_usable() -> None:
    assert pose_is_usable(_p(stamp=100.0), now=100.1, max_age_s=0.5) is None


def test_missing_pose_is_reported_not_defaulted() -> None:
    assert pose_is_usable(None, now=100.0, max_age_s=0.5) == "pose_missing"


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_pose_is_rejected(bad) -> None:
    assert pose_is_usable(_p(x=bad), now=100.0, max_age_s=0.5) == "pose_nonfinite"
    assert pose_is_usable(_p(yaw=bad), now=100.0, max_age_s=0.5) == "pose_nonfinite"


def test_stale_pose_is_rejected_with_its_age() -> None:
    reason = pose_is_usable(_p(stamp=100.0), now=101.0, max_age_s=0.5)
    assert reason is not None and reason.startswith("pose_stale:")
    assert "1000ms" in reason


def test_pose_stamped_in_the_future_is_rejected() -> None:
    """bernard-1 has no NTP and its clock has been pushed by hand mid-session.

    A future stamp yields a negative age, which would sail through a naive
    `age > max_age` freshness test — the stalest possible pose passing as the
    freshest.
    """
    assert pose_is_usable(_p(stamp=200.0), now=100.0, max_age_s=0.5) == "pose_future_stamp"


def test_age_exactly_at_the_limit_is_still_usable() -> None:
    assert pose_is_usable(_p(stamp=100.0), now=100.5, max_age_s=0.5) is None


# ---------------------------------------------------------------------------
# Jump detection — the false-negative half (a teleport MUST be caught)
# ---------------------------------------------------------------------------

def test_a_loop_closure_sized_correction_is_caught() -> None:
    """The case this exists for: SLAM snaps the pose half a metre."""
    prev = _p(x=1.0, y=1.0, stamp=100.0)
    curr = _p(x=1.5, y=1.0, stamp=100.2)
    reason = detect_pose_jump(prev, curr)
    assert reason is not None and reason.startswith("pose_jump_translation")


def test_a_heading_correction_is_caught() -> None:
    prev = _p(yaw=0.0, stamp=100.0)
    curr = _p(yaw=1.2, stamp=100.2)
    reason = detect_pose_jump(prev, curr)
    assert reason is not None and reason.startswith("pose_jump_yaw")


def test_a_large_relocalization_in_both_axes_is_caught() -> None:
    prev = _p(x=0.0, y=0.0, yaw=0.0, stamp=100.0)
    curr = _p(x=2.0, y=-1.5, yaw=2.0, stamp=100.2)
    assert detect_pose_jump(prev, curr) is not None


# ---------------------------------------------------------------------------
# Jump detection — the false-positive half (normal driving must NOT trip it)
# ---------------------------------------------------------------------------

def test_driving_at_the_safety_clamp_is_not_a_jump() -> None:
    """max_vx is 0.25 m/s. A tick of honest driving must never reset the net."""
    prev = _p(x=0.0, stamp=100.0)
    curr = _p(x=0.25 * 0.2, stamp=100.2)  # one 5 Hz tick at full clamp
    assert detect_pose_jump(prev, curr) is None


def test_turning_at_the_yaw_clamp_is_not_a_jump() -> None:
    """max_wz is 0.8 rad/s."""
    prev = _p(yaw=0.0, stamp=100.0)
    curr = _p(yaw=0.8 * 0.2, stamp=100.2)
    assert detect_pose_jump(prev, curr) is None


def test_the_first_pose_of_a_run_is_not_a_jump() -> None:
    """Nothing to be discontinuous with, and the state is already empty."""
    assert detect_pose_jump(None, _p(x=99.0, y=99.0)) is None


def test_small_pose_noise_is_not_a_jump() -> None:
    prev = _p(x=1.0, y=1.0, stamp=100.0)
    curr = _p(x=1.004, y=1.003, yaw=0.01, stamp=100.05)
    assert detect_pose_jump(prev, curr) is None


def test_yaw_wrapping_across_pi_is_not_a_jump() -> None:
    """A robot pointing just short of +-pi must not read as a 6.28 rad spin.

    Without wrapping this fires every time the heading crosses the
    discontinuity, i.e. repeatedly and for no reason, resetting the recurrent
    state mid-run.
    """
    prev = _p(yaw=math.pi - 0.05, stamp=100.0)
    curr = _p(yaw=-math.pi + 0.05, stamp=100.2)
    assert detect_pose_jump(prev, curr) is None


# ---------------------------------------------------------------------------
# The threshold scales with dt rather than assuming a tick rate
# ---------------------------------------------------------------------------

def test_the_same_displacement_is_ok_over_a_long_gap_and_a_jump_over_a_short_one() -> None:
    """A fixed displacement threshold would get this backwards.

    0.4 m is fine over 2 s of driving and impossible in 50 ms; only an
    implied-speed test can tell those apart.
    """
    slow = detect_pose_jump(_p(stamp=100.0), _p(x=0.4, stamp=102.0))
    fast = detect_pose_jump(_p(stamp=100.0), _p(x=0.4, stamp=100.05))
    assert slow is None
    assert fast is not None


def test_a_tiny_dt_does_not_make_noise_read_as_a_teleport() -> None:
    """The displacement floor exists for this: at dt -> 0 the speed test
    becomes arbitrarily sensitive."""
    assert detect_pose_jump(_p(stamp=100.0), _p(x=0.01, stamp=100.001)) is None


def test_zero_or_backwards_dt_is_not_reported_as_a_jump() -> None:
    """A repeated TF sample must not reset the network every tick."""
    assert detect_pose_jump(_p(stamp=100.0), _p(x=0.5, stamp=100.0)) is None
    assert detect_pose_jump(_p(stamp=100.0), _p(x=0.5, stamp=99.0)) is None


def test_nonfinite_in_a_jump_comparison_is_reported() -> None:
    assert detect_pose_jump(_p(), _p(x=math.nan)) == "jump_nonfinite"


def test_thresholds_are_configurable() -> None:
    prev, curr = _p(stamp=100.0), _p(x=0.1, stamp=100.2)
    assert detect_pose_jump(prev, curr) is None
    assert detect_pose_jump(
        prev, curr, max_speed_mps=0.01, floor_m=0.001
    ) is not None
