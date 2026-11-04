"""Roam's retreat out of a stop-zone e-stop: the pure controller and its wiring.

rm_safety passes exactly one thing under its stop-zone latch, a fresh pure
straight reverse, and releases the latch after N clear ticks of it. This
module decides when policy_node sends that reverse and when it stops. Every
branch of that decision is a case here, clockless; the wiring pins read the
node's source. Pure stdlib + yaml. Workstation tier.
"""

from __future__ import annotations

import math
import os
import re
import sys

import pytest
import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_policy"))

from rm_policy.retreat import (  # noqa: E402
    ABORTED,
    DISTANCE,
    NOT_CLEAR,
    TURNED,
    TurnController,
    TurnLimits,
    RELEASED,
    STARTED,
    STATE_KILLED,
    STATE_OK,
    STATE_SLOW,
    STATE_STOPPED,
    TIMEOUT,
    RetreatController,
    RetreatLimits,
    SafetyView,
    is_latched,
)
from rm_policy.roam import RoamSequencer, load_roam_limits  # noqa: E402

_MY = "/policy/m4/cmd_vel"
_OTHER = "/policy/m8v2/cmd_vel"
_ROAM_YAML = os.path.join(_REPO_ROOT, "config", "roam.yaml")
_SAFETY_YAML = os.path.join(_REPO_ROOT, "config", "safety_limits.yaml")
_MSG = os.path.join(_REPO_ROOT, "src", "rm_interfaces", "msg", "SafetyStatus.msg")
_POLICY_NODE = os.path.join(_REPO_ROOT, "src", "rm_policy", "rm_policy", "policy_node.py")
_SAFETY_NODE = os.path.join(_REPO_ROOT, "src", "rm_safety", "rm_safety", "safety_node.py")


def _read(path):
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _limits(**overrides) -> RetreatLimits:
    base = dict(retreat_vx=0.12, retreat_m=0.3, retreat_max_m=0.8, retreat_timeout_s=8.0,
                retreat_cooldown_s=20.0, retreat_pinned_range_m=0.30, retreat_pinned_after_s=10.0)
    base.update(overrides)
    return RetreatLimits(**base)


def _ctl(**overrides) -> RetreatController:
    return RetreatController(_limits(**overrides), status_max_age_s=0.5)


def _view(state=STATE_STOPPED, reason="stop_zone", age=0.0, topic=_MY,
          min_range=float("inf")) -> SafetyView:
    return SafetyView(state, reason, age, topic, min_range)


_PINNED = _view(STATE_SLOW, "slow_zone", min_range=0.20)


_LATCHED = _view()
_RETREATING = _view(STATE_SLOW, "stop_zone_retreat")
_DRIVING = _view(STATE_OK, "ok")


def _start(ctl, now=0.0):
    step = ctl.step(now, _LATCHED, (0.0, 0.0), True, _MY, 0.1)
    assert step is not None and step.reason == STARTED and not step.done
    assert (step.vx, step.wz) == (-0.12, 0.0)
    return step


# ---------------------------------------------------------------------------
# starting
# ---------------------------------------------------------------------------

def test_starts_only_on_a_fresh_latch_while_roaming_and_selected():
    assert _ctl().step(0.0, _LATCHED, (0, 0), roam_on=False, my_topic=_MY, last_cmd_vx=0.1) is None
    assert _ctl().step(0.0, _LATCHED, (0, 0), True, _MY, 0.1) is not None
    assert _ctl().step(0.0, _view(topic=_OTHER), (0, 0), True, _MY, 0.1) is None
    assert _ctl().step(0.0, _view(age=0.6), (0, 0), True, _MY, 0.1) is None     # stale status
    assert _ctl().step(0.0, _view(state=None), (0, 0), True, _MY, 0.1) is None  # never received
    assert _ctl().step(0.0, _DRIVING, (0, 0), True, _MY, 0.1) is None
    assert _ctl().step(0.0, _view(STATE_KILLED, "killed"), (0, 0), True, _MY, 0.1) is None
    assert _ctl().step(0.0, _RETREATING, (0, 0), True, _MY, 0.1) is None        # not a fresh latch


def test_pinned_in_the_slow_band_starts_only_after_the_stall_window():
    ctl = _ctl()
    assert ctl.step(0.0, _PINNED, (0, 0), True, _MY, 0.1, stalled_s=9.9) is None
    step = ctl.step(0.0, _PINNED, (0, 0), True, _MY, 0.1, stalled_s=10.0)
    assert step is not None and step.reason == STARTED and ctl.start_cause == "pinned"
    # Slow with the obstacle farther than the pinned range is driving, not stuck.
    far = _view(STATE_SLOW, "slow_zone", min_range=0.45)
    assert _ctl().step(0.0, far, (0, 0), True, _MY, 0.1, stalled_s=30.0) is None
    # A latch starts at once, whatever the stall clock says.
    assert _ctl().step(0.0, _LATCHED, (0, 0), True, _MY, 0.1, stalled_s=0.0) is not None
    assert _ctl().start_cause is None


def test_a_pinned_retreat_ends_when_the_obstacle_is_back_past_the_pinned_range():
    ctl = _ctl()
    ctl.step(0.0, _PINNED, (0, 0), True, _MY, 0.1, stalled_s=12.0)
    still = _view(STATE_SLOW, "slow_zone", min_range=0.26)
    assert ctl.step(0.5, still, (-0.05, 0.0), True, _MY, -0.12).done is False
    done = ctl.step(1.0, _view(STATE_SLOW, "slow_zone", min_range=0.34), (-0.1, 0.0), True, _MY, -0.12)
    assert done.done and done.reason == RELEASED


def test_a_latch_after_the_nodes_own_reverse_is_refused_with_a_reason():
    ctl = _ctl()
    assert ctl.step(0.0, _LATCHED, (0, 0), True, _MY, -0.05) is None
    assert ctl.hold_reason and "reverse" in ctl.hold_reason
    assert ctl.step(0.0, _LATCHED, (0, 0), True, _MY, 0.0) is not None   # zero counts as forward


def test_a_cooldown_bounds_pinned_retreats_but_a_fresh_latch_always_retreats():
    ctl = _ctl()
    _start(ctl, 0.0)
    assert ctl.step(1.0, _DRIVING, (0.0, 0.0), True, _MY, -0.12).reason == RELEASED
    # Pinned again inside the cooldown: refused with the seconds left.
    assert ctl.step(5.0, _PINNED, (0, 0), True, _MY, 0.1, stalled_s=30.0) is None
    assert "cooldown" in ctl.hold_reason
    # A fresh latch inside the cooldown: a latched robot can do nothing else.
    assert ctl.step(6.0, _LATCHED, (0, 0), True, _MY, 0.1).reason == STARTED
    assert ctl.status()["retreats"] == 2
    assert ctl.step(7.0, _DRIVING, (0.0, 0.0), True, _MY, -0.12).reason == RELEASED
    # Pinned after the cooldown: allowed.
    assert ctl.step(28.0, _PINNED, (0, 0), True, _MY, 0.1, stalled_s=30.0).reason == STARTED


# ---------------------------------------------------------------------------
# driving and ending
# ---------------------------------------------------------------------------

def test_drives_exactly_straight_reverse_until_the_latch_releases():
    ctl = _ctl()
    _start(ctl, 0.0)
    for t in (0.2, 0.4, 0.6):
        step = ctl.step(t, _RETREATING, (-0.01 * t, 0.0), True, _MY, -0.12)
        assert step == step.__class__(-0.12, 0.0, False, None)
    done = ctl.step(0.8, _DRIVING, (-0.1, 0.0), True, _MY, -0.12)
    assert done.done and done.reason == RELEASED and done.vx == 0.0
    assert not ctl.active and ctl.status()["retreat_last"] == RELEASED


def test_a_slow_zone_reason_after_the_latch_counts_as_released():
    ctl = _ctl()
    _start(ctl, 0.0)
    done = ctl.step(0.5, _view(STATE_SLOW, "slow_zone"), (0.0, 0.0), True, _MY, -0.12)
    assert done.done and done.reason == RELEASED


def test_under_a_held_latch_the_reverse_goes_on_to_the_bound():
    ctl = _ctl()
    _start(ctl, 0.0)
    assert ctl.step(1.0, _RETREATING, (-0.29, 0.0), True, _MY, -0.12).done is False
    assert ctl.step(1.2, _RETREATING, (-0.30, 0.0), True, _MY, -0.12).done is False   # latched: keep going
    assert ctl.step(2.0, _RETREATING, (-0.60, 0.0), True, _MY, -0.12).done is False
    done = ctl.step(3.0, _RETREATING, (-0.80, 0.0), True, _MY, -0.12)
    assert done.done and done.reason == DISTANCE


def test_distance_ends_a_pinned_retreat_at_retreat_m():
    ctl = _ctl()
    ctl.step(0.0, _PINNED, (0, 0), True, _MY, 0.1, stalled_s=12.0)
    still = _view(STATE_SLOW, "slow_zone", min_range=0.26)
    assert ctl.step(1.0, still, (-0.29, 0.0), True, _MY, -0.12).done is False
    done = ctl.step(1.2, still, (-0.30, 0.0), True, _MY, -0.12)
    assert done.done and done.reason == DISTANCE


def test_timeout_with_the_latch_held_gives_up_and_is_counted():
    ctl = _ctl()
    _start(ctl, 0.0)
    assert ctl.step(7.9, _LATCHED, (0.0, 0.0), True, _MY, -0.12).done is False
    done = ctl.step(8.0, _LATCHED, (0.0, 0.0), True, _MY, -0.12)
    assert done.done and done.reason == TIMEOUT
    assert ctl.status()["retreat_timeouts"] == 1


@pytest.mark.parametrize("view, roam_on, fragment", [
    (_view(STATE_KILLED, "killed"), True, "killed"),
    (_view(STATE_STOPPED, "scan_stale"), True, "scan_stale"),
    (_view(STATE_STOPPED, "odom_stale"), True, "odom_stale"),
    (_view(STATE_STOPPED, "command_timeout"), True, "command_timeout"),
    (_view(STATE_STOPPED, "internal_error"), True, "internal_error"),
    (_view(age=0.6), True, "status_stale"),
    (_view(state=None), True, "status_stale"),
    (_view(topic=_OTHER), True, "source_changed"),
    (_LATCHED, False, "roam_off"),
])
def test_aborts_publish_zero_and_hand_off_nothing(view, roam_on, fragment):
    ctl = _ctl()
    _start(ctl, 0.0)
    done = ctl.step(0.5, view, (0.0, 0.0), roam_on, _MY, -0.12)
    assert done.done and done.reason == f"{ABORTED}:{fragment}"
    assert (done.vx, done.wz) == (0.0, 0.0)
    assert not ctl.active and ctl.status()["retreat_aborts"] == 1


def test_a_pose_jump_reported_by_the_node_aborts_only_an_active_retreat():
    ctl = _ctl()
    assert ctl.abort("pose_jump", 0.0) is None
    _start(ctl, 0.0)
    done = ctl.abort("pose_jump", 0.3)
    assert done.done and done.reason == f"{ABORTED}:pose_jump"


def test_is_latched_names_exactly_the_two_latched_pairs():
    assert is_latched(_LATCHED) and is_latched(_RETREATING)
    assert not is_latched(_DRIVING)
    assert not is_latched(_view(STATE_SLOW, "stop_zone_prelatch"))
    assert not is_latched(_view(STATE_STOPPED, "command_timeout"))


def test_status_carries_the_counters():
    assert _ctl().status() == {
        "retreating": False, "retreats": 0, "retreat_aborts": 0,
        "retreat_timeouts": 0, "retreat_last": None,
    }


# ---------------------------------------------------------------------------
# the sequencer's retreated hand-off
# ---------------------------------------------------------------------------

def test_the_sequencer_draws_at_once_after_a_retreat():
    seq = RoamSequencer(load_roam_limits(_ROAM_YAML))
    assert seq.retreated(5.0) is None            # no goal ever fixed
    seq.goal_fixed(0.0)
    assert seq.retreated(5.0) is None            # roam off
    seq.set_on(True, 1.0)
    h = seq.retreated(5.0)
    assert h is not None and h.reason == "retreated" and h.n == 1
    assert seq.last_reason == "retreated"


# ---------------------------------------------------------------------------
# the shipped numbers and the message
# ---------------------------------------------------------------------------

def test_the_requested_reverse_is_within_both_safety_bounds():
    sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_safety"))
    from rm_safety.safety_logic import Limits  # noqa: E402
    roam = load_roam_limits(_ROAM_YAML)
    with open(_SAFETY_YAML, encoding="utf-8") as handle:
        safety = Limits.from_dict(yaml.safe_load(handle))
    assert 0.0 < roam.retreat_vx <= safety.estop.retreat_max_vx
    assert roam.retreat_vx <= safety.velocity.max_vx
    # Far enough back to clear the release range from the stop boundary.
    assert roam.retreat_m >= safety.estop.retreat_release_range_m - safety.estop.stop_zone_m
    assert roam.retreat_max_m >= roam.retreat_m


def test_state_constants_mirror_the_message_definition():
    msg = _read(_MSG)
    values = dict(re.findall(r"uint8\s+(\w+)\s*=\s*(\d+)", msg))
    assert (int(values["OK"]), int(values["SLOW"]), int(values["STOPPED"]), int(values["KILLED"])) == (
        STATE_OK, STATE_SLOW, STATE_STOPPED, STATE_KILLED)
    assert "string command_topic" in msg
    assert "status.command_topic = source.topic" in _read(_SAFETY_NODE)


# ---------------------------------------------------------------------------
# policy_node wiring (source pins; the workstation tier has no rclpy)
# ---------------------------------------------------------------------------

def test_the_safety_status_feeds_exactly_the_retreat_and_nothing_else():
    src = _read(_POLICY_NODE)
    assert src.count("SafetyStatus") == 4          # docstring, import, subscription, callback
    assert src.count('"/safety/status"') == 1
    assert src.count("self._safety_view(") == 1    # read in exactly one place
    assert src.index("self._safety_view(") > src.index("def _control_loop_inner(")
    for forbidden in ("/safety/reset", "/safety/kill", "safety_params", "safety_node/"):
        assert forbidden not in src, forbidden     # it never arms, kills or reads the source
    assert src.count("create_publisher(") == 2


def test_the_retreat_step_sits_after_the_pose_block_and_before_the_goal_block():
    src = _read(_POLICY_NODE)
    step = src.index("self._retreat.step(")
    assert src.index("self._prev_pose = pose") < step < src.index('reason="goal_undefined"')
    assert step < src.index("build_observation(")
    body = src[src.index("def _publish_reverse("):src.index("def _scan_cb(")]
    assert "self._cmd_pub.publish(" in body and "_prev_action" not in body
    assert 'self._retreat.abort("pose_jump", now)' in src
    assert "self._roam.retreated(now, scan)" in src
    assert "self._retreat.goal_fixed()" in src[src.index("def _fix_goal("):src.index("def _safety_cb(")]
    assert '{**self._roam.status(time.monotonic()), **self._retreat.status(),' in src
    assert '**self._turn.status()}' in src


# ---------------------------------------------------------------------------
# the turn in place
# ---------------------------------------------------------------------------

def _turn(**overrides) -> TurnController:
    base = dict(turn_wz=0.5, turn_tolerance_deg=15.0, turn_timeout_s=12.0, turn_clear_m=0.30)
    base.update(overrides)
    return TurnController(TurnLimits(**base))


def test_turn_is_needed_only_beyond_the_tolerance():
    t = _turn()
    assert not t.needed(math.radians(14.0)) and not t.needed(math.radians(-14.0))
    assert t.needed(math.radians(16.0)) and t.needed(math.radians(-170.0))


def test_turn_yaws_toward_the_goal_until_within_tolerance():
    t = _turn()
    t.start(0.0)
    assert t.active and t.status()["turns"] == 1
    left = t.step(0.2, math.radians(120.0), clear=True)
    assert (left.vx, left.wz, left.done) == (0.0, 0.5, False)
    right = t.step(0.4, math.radians(-40.0), clear=True)   # the bearing is re-read each tick
    assert (right.vx, right.wz, right.done) == (0.0, -0.5, False)
    done = t.step(0.6, math.radians(10.0), clear=True)
    assert done.done and done.reason == TURNED and not t.active


def test_turn_refuses_when_anything_is_close_and_gives_up_on_timeout():
    t = _turn()
    t.start(0.0)
    refused = t.step(0.2, math.radians(90.0), clear=False)
    assert refused.done and refused.reason == NOT_CLEAR and refused.wz == 0.0
    assert t.status()["turn_refused"] == 1
    t.start(10.0)
    assert t.step(21.9, math.radians(90.0), clear=True).done is False
    timed = t.step(22.0, math.radians(90.0), clear=True)
    assert timed.done and timed.reason == "timeout" and t.status()["turn_timeouts"] == 1


def test_a_fixed_sense_turns_the_long_way_round_and_still_ends_on_tolerance():
    t = _turn()
    t.start(0.0, direction=-1.0)                     # right, although the goal is left
    step = t.step(0.2, math.radians(90.0), clear=True)
    assert (step.wz, step.done) == (-0.5, False)
    step = t.step(1.0, math.radians(170.0), clear=True)   # passing behind
    assert (step.wz, step.done) == (-0.5, False)
    step = t.step(2.0, math.radians(-120.0), clear=True)  # error flips sign on the far side
    assert (step.wz, step.done) == (-0.5, False)
    done = t.step(3.0, math.radians(-10.0), clear=True)
    assert done.done and done.reason == TURNED


def test_a_retreat_cancels_a_turn():
    t = _turn()
    t.start(0.0)
    t.cancel("retreat")
    assert not t.active and t.status()["turn_last"] == "aborted:retreat"


def test_the_node_turns_after_roam_hand_offs_only_with_clear_floor():
    src = _read(_POLICY_NODE)
    assert src.count("self._after_roam_handoff(now, scan)") == 2  # the stall/arrival hand-off and the post-recovery one
    assert "turn_direction(scan, bearing, self._roam.limits)" in src
    assert "recovery_bearing(scan, self._roam.limits)" in src
    assert src.index("self._turn_target_yaw = self._robot_yaw + bearing") < src.index("self._roam.retreated(now, scan)")
    block = src[src.index("if self._turn.active and self._goal_world_x is not None:"):]
    block = block[:block.index("# Goal must be defined")]
    assert "nearest_return(scan, self._roam.limits.self_radius_m)" in block
    assert "self._publish_turn(turn.wz)" in block and "self._roam.goal_fixed(now)" in block
    assert src.index("self._retreat.step(") < src.index("if self._turn.active and") < src.index('reason="goal_undefined"')
    turn_pub = src[src.index("def _publish_turn("):src.index("def _publish_reverse(")]
    assert "cmd.linear.x = 0.0" in turn_pub and "_prev_action" not in turn_pub
    assert 'self._turn.cancel("retreat")' in src and 'self._turn.cancel("new goal")' in src
