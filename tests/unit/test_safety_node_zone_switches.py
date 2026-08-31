"""Workstation-tier tests for safety_node's zone-switch plumbing.

WHY THIS FILE IS UNUSUAL, and worth the stubbing it costs:

    The workstation tier has never imported a ROS node module — there is no
    rclpy here — so an undefined name inside one is invisible until the node
    is launched on the robot. That has already cost a session once
    (`dashboard_node` died on a missing `SetPose` import, 2026-08-29), and the
    zone switches touch safety_node in four places, on the one node whose
    failure mode is a robot that will not stop.

    So this module stubs the ROS imports, imports safety_node FOR REAL, and
    exercises the switch plumbing on an instance built with `object.__new__`
    (skipping `__init__`, which needs a live ROS context). That is enough to
    catch the class of bug the tier otherwise cannot see: a typo'd attribute,
    a missing import, a method that does not exist.

    It deliberately does NOT try to fake the node lifecycle. Subscriptions,
    timers and services are the robot tier's business.
"""

import os
import sys
import types

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("rm_safety",):
    _p = os.path.join(_REPO_ROOT, "src", _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)


# ---------------------------------------------------------------------------
# ROS stubs — the minimum safety_node touches at IMPORT time
# ---------------------------------------------------------------------------

def _install_ros_stubs() -> None:
    """Register fake ROS modules in sys.modules if the real ones are absent.

    Only stubs what is missing, so this is a no-op on the robot, where the
    real packages are importable and the same tests then run against them.
    """
    def _mod(name, **attrs):
        if name in sys.modules:
            return sys.modules[name]
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    try:
        import rclpy  # noqa: F401
        return  # real ROS present (the robot tier) — use it
    except ImportError:
        pass

    class _Node:
        """Stand-in for rclpy.node.Node — only what safety_node subclasses."""
        def __init__(self, *a, **kw):
            pass

    rclpy = _mod("rclpy", init=lambda *a, **k: None,
                 shutdown=lambda *a, **k: None, ok=lambda: True,
                 spin=lambda *a, **k: None)
    node_mod = _mod("rclpy.node", Node=_Node)
    rclpy.node = node_mod

    class _QoSProfile:
        def __init__(self, *a, **kw):
            pass

    class _Enum:
        BEST_EFFORT = RELIABLE = VOLATILE = TRANSIENT_LOCAL = 0

    qos = _mod("rclpy.qos", QoSProfile=_QoSProfile,
               ReliabilityPolicy=_Enum, DurabilityPolicy=_Enum,
               HistoryPolicy=_Enum)
    rclpy.qos = qos

    class _Msg:
        def __init__(self, *a, **kw):
            pass

    _mod("geometry_msgs"); _mod("geometry_msgs.msg", Twist=_Msg)
    _mod("nav_msgs"); _mod("nav_msgs.msg", Odometry=_Msg)
    _mod("sensor_msgs"); _mod("sensor_msgs.msg", LaserScan=_Msg)
    _mod("std_msgs"); _mod("std_msgs.msg", Header=_Msg, Float32=_Msg, String=_Msg)

    class _SetParametersResult:
        def __init__(self, successful=True, reason=""):
            self.successful = successful
            self.reason = reason

    _mod("rcl_interfaces")
    _mod("rcl_interfaces.msg", SetParametersResult=_SetParametersResult)

    class _Trigger:
        class Request: pass
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""

    _mod("std_srvs"); _mod("std_srvs.srv", Trigger=_Trigger)

    class _SafetyStatus:
        OK, SLOW, STOPPED, KILLED = 0, 1, 2, 3
        def __init__(self, *a, **kw):
            pass

    _mod("rm_interfaces"); _mod("rm_interfaces.msg", SafetyStatus=_SafetyStatus)


_install_ros_stubs()

from rm_safety import safety_node as sn          # noqa: E402
from rm_safety.safety_logic import Limits        # noqa: E402

from test_safety_zone_switches import _limits_dict  # noqa: E402


# ---------------------------------------------------------------------------
# Collaborators
# ---------------------------------------------------------------------------

class _Log:
    """Captures log calls so the tests can assert on what an operator sees."""
    def __init__(self):
        self.infos, self.warns, self.errors = [], [], []

    def info(self, m):
        self.infos.append(str(m))

    def warn(self, m):
        self.warns.append(str(m))

    def warning(self, m):
        self.warns.append(str(m))

    def error(self, m):
        self.errors.append(str(m))


class _ParamValue:
    def __init__(self, v):
        self.bool_value = v
        self.string_value = v if isinstance(v, str) else ""
        self.double_value = v if isinstance(v, float) else 0.0


class _Param:
    def __init__(self, v):
        self._v = v

    def get_parameter_value(self):
        return _ParamValue(self._v)


def _node(limits: Limits, *, slow_param=True, stop_param=True):
    """A SafetyNode with no ROS lifecycle — enough for the switch plumbing."""
    n = object.__new__(sn.SafetyNode)
    n._limits = limits
    n._zones_last_warned_s = 0.0
    n._log = _Log()
    n.get_logger = lambda: n._log
    n._params = {
        "slow_zone_enabled": slow_param,
        "stop_zone_enabled": stop_param,
    }
    n.get_parameter = lambda name: _Param(n._params[name])
    return n


# ---------------------------------------------------------------------------
# The module imports at all — the bug class this tier could not see
# ---------------------------------------------------------------------------

def test_safety_node_module_imports():
    """A missing import or typo'd name in safety_node fails HERE, not on the robot."""
    assert hasattr(sn, "SafetyNode")
    for attr in (
        "_apply_zone_switches",
        "_announce_zone_state",
        "_warn_zones_disabled_periodically",
        "_on_set_parameters",
    ):
        assert hasattr(sn.SafetyNode, attr), attr
    assert sn._DEFAULT_ZONE_ENABLED is True
    assert sn._ZONES_DISABLED_WARN_PERIOD_S > 0


# ---------------------------------------------------------------------------
# AND composition
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "yaml_slow, yaml_stop, p_slow, p_stop, want_slow, want_stop",
    [
        (True,  True,  True,  True,  True,  True),   # nobody disables
        (True,  True,  False, True,  False, True),   # launch disables slow
        (True,  True,  True,  False, True,  False),  # launch disables stop
        (True,  True,  False, False, False, False),  # launch disables both
        (False, True,  True,  True,  False, True),   # yaml disables slow
        (True,  False, True,  True,  True,  False),  # yaml disables stop
        (False, False, True,  True,  False, False),  # yaml disables both
    ],
)
def test_and_composition(yaml_slow, yaml_stop, p_slow, p_stop, want_slow, want_stop):
    """EITHER source can disable; NEITHER can re-enable what the other switched off.

    That direction means whichever file you read, the protection it claims is
    at most what is actually running — a config cannot overstate the e-stop.
    """
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=yaml_slow, stop_zone_enabled=yaml_stop)
    )
    n = _node(limits, slow_param=p_slow, stop_param=p_stop)
    out = n._apply_zone_switches(limits)

    assert out.estop.slow_zone_enabled is want_slow
    assert out.estop.stop_zone_enabled is want_stop


def test_apply_zone_switches_leaves_every_other_limit_untouched():
    """The override touches the two flags and nothing else in the limits."""
    limits = Limits.from_dict(_limits_dict())
    n = _node(limits, slow_param=False, stop_param=False)
    out = n._apply_zone_switches(limits)

    assert out.velocity == limits.velocity
    assert out.acceleration == limits.acceleration
    assert out.timeouts == limits.timeouts
    assert out.estop.slow_zone_m == limits.estop.slow_zone_m
    assert out.estop.stop_zone_m == limits.estop.stop_zone_m
    assert out.estop.forward_arc_deg == limits.estop.forward_arc_deg
    assert out.estop.consecutive_scans_to_trigger == \
        limits.estop.consecutive_scans_to_trigger


# ---------------------------------------------------------------------------
# The disabled state is LOUD
# ---------------------------------------------------------------------------

def test_enabled_announcement_is_info_not_warn():
    limits = Limits.from_dict(_limits_dict())
    n = _node(limits)
    n._announce_zone_state()

    assert n._log.warns == []
    assert any("ACTIVE" in m for m in n._log.infos)


def test_disabled_announcement_warns_and_names_the_hazard():
    limits = Limits.from_dict(
        _limits_dict(slow_zone_enabled=False, stop_zone_enabled=False)
    )
    n = _node(limits)
    n._announce_zone_state()

    assert n._log.warns, "a disabled e-stop must not be announced at info level"
    blob = "\n".join(n._log.warns)
    # It must say what is gone, that there is no physical E-stop, and what
    # still works — an operator reading only this line should not be surprised.
    assert "NO PHYSICAL E-STOP" in blob
    assert "STOP zone" in blob
    assert "/safety/kill" in blob
    assert "clamps" in blob


def test_periodic_warning_is_silent_while_the_estop_is_whole():
    n = _node(Limits.from_dict(_limits_dict()))
    for t in range(0, 120, 5):
        n._warn_zones_disabled_periodically(float(t))
    assert n._log.warns == []


def test_periodic_warning_repeats_on_the_configured_period():
    """Startup logs scroll away; an hour-old stack is when it gets forgotten."""
    limits = Limits.from_dict(_limits_dict(stop_zone_enabled=False))
    n = _node(limits)

    period = sn._ZONES_DISABLED_WARN_PERIOD_S
    n._warn_zones_disabled_periodically(period + 0.1)
    assert len(n._log.warns) == 1

    # Too soon — must not repeat.
    n._warn_zones_disabled_periodically(period + 0.2)
    assert len(n._log.warns) == 1

    # A full period later — must repeat.
    n._warn_zones_disabled_periodically(2 * period + 0.3)
    assert len(n._log.warns) == 2


# ---------------------------------------------------------------------------
# Not settable at runtime
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["slow_zone_enabled", "stop_zone_enabled"])
def test_zone_switches_are_rejected_at_runtime(name):
    """No `ros2 param set`, and therefore no unauthenticated dashboard, may
    switch the e-stop off mid-run. Disabling it is a bring-up decision."""
    limits = Limits.from_dict(_limits_dict())
    n = _node(limits)
    n._command_source = "teleop"

    class _P:
        def __init__(self, name, value):
            self.name = name
            self.value = value

    result = n._on_set_parameters([_P(name, False)])

    assert result.successful is False
    assert "not settable at runtime" in result.reason.lower()
    assert "/safety/kill" in result.reason
    # And the live limits are unchanged.
    assert n._limits.estop.slow_zone_enabled is True
    assert n._limits.estop.stop_zone_enabled is True
