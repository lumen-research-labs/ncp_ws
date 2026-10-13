"""
safety_node.py — ROS2 node wrapper for the rm_safety pipeline.

What it does:
    Sits between every command source and the base driver.  The sources are
    the rows of config/topics.yaml `command_sources` (teleop, policy, m8v2,
    m4 as of 2026-10-13); this node subscribes to all of them, keeps one
    buffer per row, and forwards ONLY the row named by `command_source`.  On
    every timer tick it builds an EvalInputs from the latest sensor data and
    that one buffer, calls safety_logic.evaluate(), publishes the resulting
    velocity to /cmd_vel (the ONLY publisher of that topic in the entire
    system), and publishes a SafetyStatus message.

    All decision logic (clamping, e-stop, kill-state transitions) lives in
    safety_logic.py.  This node is responsible only for I/O, timing, state
    persistence, and calling evaluate().

Subscribes:
    <every command_sources row's topic in config/topics.yaml>
                      [geometry_msgs/Twist]  @ ~5 Hz (policy rows) / variable
                                              (teleop). One buffer per row;
                                              only the selected row is read.
    /scan             [sensor_msgs/LaserScan] @ ~10 Hz (Phase 3 TBD)
    /odom             [nav_msgs/Odometry]     @ ~10 Hz (Phase 3 TBD)

Publishes:
    /cmd_vel          [geometry_msgs/Twist]  @ safety_loop_rate_hz  (SOLE PUBLISHER)
    /safety/status    [rm_interfaces/SafetyStatus]  @ publishing.status_rate_hz

Services:
    /safety/kill      [std_srvs/Trigger]  — activate hard kill; persists to disk
    /safety/reset     [std_srvs/Trigger]  — clear kill and request latch reset

Parameters:
    command_source        (str,   "teleop")        — active command source: a
                                                     row name from
                                                     config/topics.yaml
                                                     `command_sources`. Any
                                                     other value is REFUSED, at
                                                     startup and at runtime.
                                                     Default is "teleop" so that
                                                     bringing the stack up can
                                                     never connect the policy to
                                                     the base driver.
                                                     SETTABLE AT RUNTIME (since
                                                     2026-08-29), and every
                                                     change force-kills, so a
                                                     switch can only stop the
                                                     robot, never start it. All
                                                     other parameters here are
                                                     read once at startup and a
                                                     runtime set is REJECTED.
    safety_loop_rate_hz   (float, 20.0)            — control loop rate.
                                                     PROVISIONAL: 20 Hz is above
                                                     the 5 Hz prior from the
                                                     previous team and above the
                                                     measured ~10 Hz sensor rates.
                                                     The definitive rate is a
                                                     Phase 3 decision, blocked on
                                                     the unmeasured /scan rate.
                                                     Must be >= publishing.zero_publish_rate_hz
                                                     (20 Hz from safety_limits.yaml)
                                                     to guarantee continuous-zero
                                                     publishing when killed.
    safety_limits_path    (str,   <in-repo path>)  — absolute path to
                                                     config/safety_limits.yaml.
                                                     Pass via launch; the default
                                                     resolves relative to this
                                                     file's location and is only
                                                     valid in a development layout.
    topics_path           (str,   <in-repo path>)  — absolute path to
                                                     config/topics.yaml, the
                                                     command-source table. Same
                                                     resolution rule. Read once;
                                                     a malformed table is fatal
                                                     (rm_safety/command_sources.py).

Failure mode:
    - Node comes up KILLED (start_killed: true in safety_limits.yaml).  No
      motion is possible as a side effect of launching.
    - If safety_limits.yaml cannot be loaded, the node raises on __init__ and
      refuses to start rather than proceeding in an unknown-limits state.
    - If the persisted kill state file is unreadable, assume KILLED (fail-closed).
    - Any exception inside the control loop callback: publish zero Twist for that
      tick and log at ERROR.  Never let an exception skip the publish and allow a
      prior command to persist downstream.
    - Stale odom (age > odom_timeout_ms): the node computes odom_stale=True and
      passes it into EvalInputs.  safety_logic.evaluate() returns state=STOPPED
      with reason="odom_stale" at precedence 2.5 (after scan-stale, before
      stop-zone and command-stale).  The reason string is distinct from
      "command_timeout" so Phase 5 diagnostics can correctly identify odom
      dropout vs. command dropout.
    - On shutdown: publishes a final zero Twist best-effort so the robot is left
      stopped regardless of what was last commanded.
"""

# ---------------------------------------------------------------------------
# Subscribes:  <each config/topics.yaml command_sources row> [geometry_msgs/Twist]
# Subscribes:  /scan            [sensor_msgs/LaserScan] @ ~10 Hz (Phase 3 TBD)
# Subscribes:  /odom            [nav_msgs/Odometry]     @ ~10 Hz (Phase 3 TBD)
# Publishes:   /cmd_vel         [geometry_msgs/Twist]  @ safety_loop_rate_hz
# Publishes:   /safety/status   [rm_interfaces/SafetyStatus]  @ status_rate_hz
# Services:    /safety/kill     [std_srvs/Trigger]
# Services:    /safety/reset    [std_srvs/Trigger]
# Parameters:  command_source (str, "teleop") — active command source, a table row
# Parameters:  safety_loop_rate_hz (float, 20.0) — control loop rate (provisional)
# Parameters:  safety_limits_path (str, <repo-relative>) — path to safety_limits.yaml
# Parameters:  topics_path (str, <repo-relative>) — path to topics.yaml (the table)
# Failure mode: fail-closed; killed-on-startup; any exception → publish zero
# ---------------------------------------------------------------------------

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import time
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
import rclpy.node
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from rm_interfaces.msg import SafetyStatus
from rm_safety.command_sources import (
    CommandSource,
    DEFAULT_TOPICS_PATH,
    load_command_sources,
)
from rm_safety.safety_logic import (
    EvalInputs,
    Limits,
    State,
    evaluate,
)

# ---------------------------------------------------------------------------
# Default path resolution — only works in a standard colcon workspace layout
# where the package source tree is adjacent to config/.  Prefer passing
# safety_limits_path explicitly via a launch file parameter.
# ---------------------------------------------------------------------------
_THIS_DIR = pathlib.Path(__file__).parent.resolve()
_DEFAULT_LIMITS_PATH = str(_THIS_DIR.parents[2] / "config" / "safety_limits.yaml")

# ---------------------------------------------------------------------------
# Default loop rate (Hz).
# PROVISIONAL — this value is above the 5 Hz prior from the previous team's
# stack and above the measured ~10 Hz sensor rates, but the definitive control
# rate is a Phase 3 decision that is still BLOCKED on the unmeasured /scan
# rate.  Do not raise this before Phase 3 is complete.
# Must remain >= publishing.zero_publish_rate_hz (20 Hz in safety_limits.yaml)
# to guarantee that the continuous-zero-when-killed invariant is met by the
# single shared timer loop.
# ---------------------------------------------------------------------------
_DEFAULT_LOOP_RATE_HZ: float = 20.0

# Default active command source.  "teleop" (the row config/topics.yaml
# requires) because the default must never allow a policy path to be active
# without an explicit operator opt-in.
_DEFAULT_COMMAND_SOURCE: str = "teleop"


@dataclasses.dataclass
class _CommandBuffer:
    """The latest Twist from one command source, and when it arrived.

    One per table row. `stamp` is time.monotonic() at arrival, or None if the
    row has never published, which the staleness check reads as infinitely
    old, so a freshly selected source that is silent fails closed.
    """
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    stamp: Optional[float] = None

# ---------------------------------------------------------------------------
# LIDAR ZONE MASTER SWITCHES — both default ON, and the composition is AND.
#
# The effective setting is (YAML value) AND (parameter value), so BOTH must
# say true for a zone to run and EITHER can switch it off. That direction is
# chosen on purpose: turning the e-stop off should require writing "false"
# somewhere explicit, while nothing anyone forgets to write can turn it off
# by accident.
#
# How often the disabled state is re-announced while running. Loud enough
# that it cannot be missed in a log, slow enough not to bury everything else.
# ---------------------------------------------------------------------------
_DEFAULT_ZONE_ENABLED: bool = True
_ZONES_DISABLED_WARN_PERIOD_S: float = 10.0


class SafetyNode(rclpy.node.Node):
    """ROS2 node: sole publisher of /cmd_vel; wraps safety_logic.evaluate()."""

    def __init__(self) -> None:
        super().__init__("safety_node")

        # ------------------------------------------------------------------
        # Declare and read parameters
        # ------------------------------------------------------------------
        self.declare_parameter("command_source", _DEFAULT_COMMAND_SOURCE)
        self.declare_parameter("safety_loop_rate_hz", _DEFAULT_LOOP_RATE_HZ)
        self.declare_parameter("safety_limits_path", _DEFAULT_LIMITS_PATH)
        self.declare_parameter("topics_path", DEFAULT_TOPICS_PATH)
        self.declare_parameter("slow_zone_enabled", _DEFAULT_ZONE_ENABLED)
        self.declare_parameter("stop_zone_enabled", _DEFAULT_ZONE_ENABLED)

        self._command_source: str = (
            self.get_parameter("command_source").get_parameter_value().string_value
        )
        self._loop_rate_hz: float = (
            self.get_parameter("safety_loop_rate_hz")
            .get_parameter_value()
            .double_value
        )
        limits_path: str = (
            self.get_parameter("safety_limits_path")
            .get_parameter_value()
            .string_value
        )
        topics_path: str = (
            self.get_parameter("topics_path").get_parameter_value().string_value
        )

        # ------------------------------------------------------------------
        # The command-source table — fatal if unreadable or malformed. A
        # node that cannot name its inputs must not start: the alternative
        # is accepting a `command_source` and forwarding nothing.
        # ------------------------------------------------------------------
        self._sources: Dict[str, CommandSource] = self._load_sources(topics_path)
        if self._command_source not in self._sources:
            raise ValueError(
                f"command_source must be one of {sorted(self._sources)} "
                f"(config/topics.yaml command_sources), got: "
                f"{self._command_source!r}"
            )

        # ------------------------------------------------------------------
        # Load safety limits — fatal if unreadable; the node must NOT start
        # in an unknown-limits state.
        # ------------------------------------------------------------------
        self._limits: Limits = self._load_limits(limits_path)

        # ------------------------------------------------------------------
        # Apply the zone master switches, and say so loudly.
        #
        # AND-composed with the YAML: both sources must say true for a zone to
        # run. Neither can re-enable what the other switched off, so reading
        # either one alone can never overstate how much protection is live.
        # ------------------------------------------------------------------
        self._limits = self._apply_zone_switches(self._limits)
        self._zones_last_warned_s: float = 0.0
        self._announce_zone_state()

        # ------------------------------------------------------------------
        # Validate loop rate vs. zero_publish_rate_hz guarantee
        # ------------------------------------------------------------------
        zero_rate = self._limits_raw.get("publishing", {}).get("zero_publish_rate_hz", 0.0)
        if self._loop_rate_hz < zero_rate:
            raise ValueError(
                f"safety_loop_rate_hz ({self._loop_rate_hz} Hz) is below "
                f"publishing.zero_publish_rate_hz ({zero_rate} Hz) from "
                f"safety_limits.yaml.  The continuous-zero-when-killed guarantee "
                f"would be violated.  Raise safety_loop_rate_hz or lower "
                f"zero_publish_rate_hz."
            )

        status_rate_hz: float = (
            self._limits_raw.get("publishing", {}).get("status_rate_hz", 10.0)
        )

        # ------------------------------------------------------------------
        # Kill-state initialisation — fail-closed
        # ------------------------------------------------------------------
        self._is_killed: bool = True  # safe default before persistence check
        self._kill_state_path: str = self._limits_raw["startup"]["kill_state_path"]
        self._persist_kill_state: bool = bool(
            self._limits_raw["startup"]["persist_kill_state"]
        )
        start_killed: bool = bool(self._limits_raw["startup"]["start_killed"])

        if start_killed:
            # start_killed is mandatory per CLAUDE.md; always begin killed.
            self._is_killed = True
        elif self._persist_kill_state:
            # start_killed is False: honor persisted state if readable.
            self._is_killed = self._read_kill_state()
        else:
            self._is_killed = False

        if self._is_killed:
            self.get_logger().warn(
                "safety_node: started KILLED (start_killed=true or persisted "
                "state).  Call /safety/reset to enable motion."
            )

        # ------------------------------------------------------------------
        # FSM carry-over state
        # ------------------------------------------------------------------
        self._estop_latched: bool = False
        self._consecutive_stop_count: int = 0
        self._reset_latch: bool = False  # consumed once per tick

        # Last published velocity (for acceleration-limit context)
        self._last_vx: float = 0.0
        self._last_vy: float = 0.0
        self._last_wz: float = 0.0

        # Wall-clock time of the last control loop tick (seconds, from time.monotonic)
        self._last_tick_time: Optional[float] = None

        # ------------------------------------------------------------------
        # Sensor / command state — all initialised to "stale"
        # Each _*_stamp is a float from time.monotonic() at arrival time, or
        # None if the topic has never been received.
        # ------------------------------------------------------------------

        # One command buffer per table row, keyed by row name. All rows are
        # subscribed and kept current; only the selected row is ever read
        # (see _select_command). A row that has never published keeps a None
        # stamp and is infinitely stale.
        self._commands: Dict[str, _CommandBuffer] = {
            name: _CommandBuffer() for name in self._sources
        }

        # Lidar scan
        self._scan_ranges: List[float] = []
        self._scan_angle_min: float = 0.0
        self._scan_angle_increment: float = 0.0
        self._scan_range_min: float = 0.0
        self._scan_range_max: float = 1.0  # safe non-zero default
        self._scan_stamp: Optional[float] = None

        # Odometry — used for staleness watchdog and accel-limit context.
        # The velocity components are passed as last_vx/vy/wz in EvalInputs
        # for the acceleration limiter only when odom is fresh.  When stale,
        # the accel-limiter context is frozen at the last known published
        # velocity (self._last_vx/vy/wz) rather than the odom reading, so a
        # stale odom cannot cause the limiter to ramp toward an old reference.
        # Stale odom is also flagged as a stop condition (see _control_loop).
        self._odom_stamp: Optional[float] = None
        self._odom_stale: bool = True  # computed each tick

        # ------------------------------------------------------------------
        # QoS profile for sensor subscriptions — best-effort, volatile
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status_pub = self.create_publisher(SafetyStatus, "/safety/status", 10)

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        for name, source in self._sources.items():
            self.create_subscription(
                Twist, source.topic, self._command_cb(name), sensor_qos
            )
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, sensor_qos
        )
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, sensor_qos
        )

        # ------------------------------------------------------------------
        # Services
        # ------------------------------------------------------------------
        self.create_service(Trigger, "/safety/kill", self._kill_srv)
        self.create_service(Trigger, "/safety/reset", self._reset_srv)

        # ------------------------------------------------------------------
        # command_source is settable at RUNTIME, and every change force-kills.
        #
        # Before 2026-08-29 this parameter was read once here in __init__ with
        # no callback, so `ros2 param set /safety_node command_source policy`
        # printed "Set parameter successful" and changed NOTHING. That failed
        # safe but silently, which is the worst combination to debug: the
        # operator believes the policy is connected and the robot simply never
        # moves, with no error anywhere.
        #
        # The fix could have gone either way — make it read-only so the
        # attempt is REJECTED, or make it real. It is made real because
        # rm_dashboard exposes source switching, and because switching source
        # is exactly the operation you want during a policy run without
        # tearing the whole stack down and losing the CfC's recurrent state.
        #
        # What makes that safe is the force-kill below: a source change can
        # only ever STOP the robot. Handing control to the policy therefore
        # still takes two explicit acts (set the source, then /safety/reset),
        # which is the same two-act guarantee start_killed gives at boot.
        # ------------------------------------------------------------------
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ------------------------------------------------------------------
        # Control loop timer — single timer drives both /cmd_vel and
        # /safety/status publishing.
        #
        # Continuous-zero guarantee: because the single loop always publishes
        # every tick (including when killed — evaluate() returns zero velocity
        # when is_killed=True), and this loop runs at safety_loop_rate_hz
        # which is validated to be >= publishing.zero_publish_rate_hz, the
        # continuous-zero invariant is always met without a separate timer.
        # ------------------------------------------------------------------
        loop_period = 1.0 / self._loop_rate_hz
        self._control_timer = self.create_timer(loop_period, self._control_loop)

        # Status is published from the same loop.  We track a separate counter
        # to throttle it to status_rate_hz without an extra timer, since the
        # loop itself runs at a potentially higher rate.
        self._status_period_ticks: int = max(
            1, round(self._loop_rate_hz / status_rate_hz)
        )
        self._tick_counter: int = 0

        self.get_logger().info(
            f"safety_node ready: source={self._command_source!r} of "
            f"{sorted(self._sources)} (" + ", ".join(
                f"{n}={s.topic}" for n, s in self._sources.items()
            ) + f"), loop={self._loop_rate_hz} Hz, killed={self._is_killed}"
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _load_limits(self, path: str) -> Limits:
        """Load and parse safety_limits.yaml; raises on any failure (fatal)."""
        try:
            with open(path, "r") as fh:
                raw = yaml.safe_load(fh)
        except Exception as exc:
            raise RuntimeError(
                f"safety_node: cannot load safety_limits.yaml from {path!r}: {exc}"
            ) from exc
        # Keep the raw dict for non-Limits fields (startup, publishing).
        self._limits_raw: dict = raw
        try:
            return Limits.from_dict(raw)
        except Exception as exc:
            raise RuntimeError(
                f"safety_node: safety_limits.yaml is malformed: {exc}"
            ) from exc

    def _load_sources(self, path: str) -> Dict[str, CommandSource]:
        """Load config/topics.yaml's command_sources; raises on any failure (fatal)."""
        try:
            return load_command_sources(path)
        except Exception as exc:
            raise RuntimeError(
                f"safety_node: cannot load the command-source table from "
                f"{path!r}: {exc}"
            ) from exc

    def _apply_zone_switches(self, limits: Limits) -> Limits:
        """AND the `*_zone_enabled` parameters into the loaded limits.

        The YAML is the authority and the parameters are an override that can
        only ever REMOVE protection, never add it back. Composing with AND
        rather than letting the parameter win means a `safety_limits.yaml`
        that disables a zone cannot be silently re-enabled by a launch file,
        and vice versa — whichever file you happen to read, the protection it
        claims is at most what is actually running.
        """
        slow = (
            self.get_parameter("slow_zone_enabled").get_parameter_value().bool_value
        )
        stop = (
            self.get_parameter("stop_zone_enabled").get_parameter_value().bool_value
        )
        return dataclasses.replace(
            limits,
            estop=dataclasses.replace(
                limits.estop,
                slow_zone_enabled=limits.estop.slow_zone_enabled and slow,
                stop_zone_enabled=limits.estop.stop_zone_enabled and stop,
            ),
        )

    def _announce_zone_state(self) -> None:
        """Log the lidar-zone configuration at startup, loudly if degraded.

        A disabled e-stop must never be something an operator has to infer
        from a launch argument they did not type. This prints once at startup
        and then on a slow repeat from the control loop, and the state is also
        carried continuously in the /safety/status reason string so it reaches
        the dashboard and every recorded bag.
        """
        es = self._limits.estop
        if es.slow_zone_enabled and es.stop_zone_enabled:
            self.get_logger().info(
                f"safety_node: lidar e-stop ACTIVE — "
                f"slow {es.slow_zone_m} m / stop {es.stop_zone_m} m."
            )
            return

        disabled = []
        if not es.slow_zone_enabled:
            disabled.append("SLOW zone (velocity scaling)")
        if not es.stop_zone_enabled:
            disabled.append("STOP zone (e-stop + latch)")

        self.get_logger().warn(
            "=" * 72 + "\n"
            "  LIDAR E-STOP PARTIALLY DISABLED — OPERATOR-APPROVED OVERRIDE\n"
            f"  DISABLED: {'; '.join(disabled)}\n"
            "  This robot has NO PHYSICAL E-STOP. With the STOP zone off,\n"
            "  NOTHING in this stack will stop the robot before a collision.\n"
            "  Velocity clamps bound SPEED, not PROXIMITY.\n"
            "  STILL ACTIVE: velocity clamps, acceleration limits, /safety/kill,\n"
            "  killed-on-startup, and the command/scan/odom staleness watchdogs.\n"
            "  KEEP THE KILL COMMAND IN HAND AND WATCH THE ROBOT.\n"
            + "=" * 72
        )

    def _warn_zones_disabled_periodically(self, now_s: float) -> None:
        """Re-announce a disabled e-stop on a slow repeat while running.

        Startup logs scroll away, and a stack that has been up for an hour is
        exactly when someone forgets the e-stop is off. Throttled so it stays
        readable next to everything else the loop emits.
        """
        if self._limits.estop.slow_zone_enabled and self._limits.estop.stop_zone_enabled:
            return
        if now_s - self._zones_last_warned_s < _ZONES_DISABLED_WARN_PERIOD_S:
            return
        self._zones_last_warned_s = now_s
        es = self._limits.estop
        off = []
        if not es.slow_zone_enabled:
            off.append("slow")
        if not es.stop_zone_enabled:
            off.append("stop")
        self.get_logger().warn(
            f"safety_node: lidar e-stop still DISABLED ({'+'.join(off)} zone) — "
            f"no obstacle protection. min_range is being measured and reported "
            f"but NOT acted on."
        )

    def _read_kill_state(self) -> bool:
        """Read the persisted kill state from disk.  Fail-closed: any error → True."""
        try:
            with open(self._kill_state_path, "r") as fh:
                content = fh.read().strip()
            state = content == "killed"
            self.get_logger().info(
                f"safety_node: persisted kill state read: {content!r} → killed={state}"
            )
            return state
        except FileNotFoundError:
            self.get_logger().info(
                "safety_node: no persisted kill state file found; defaulting to KILLED."
            )
            return True
        except Exception as exc:
            self.get_logger().error(
                f"safety_node: failed to read kill state from "
                f"{self._kill_state_path!r}: {exc} — defaulting to KILLED."
            )
            return True

    def _write_kill_state(self, killed: bool) -> None:
        """Persist the kill state to disk.  Logs a warning and continues on failure."""
        if not self._persist_kill_state:
            return
        state_str = "killed" if killed else "active"
        try:
            parent = pathlib.Path(self._kill_state_path).parent
            parent.mkdir(parents=True, exist_ok=True)
            with open(self._kill_state_path, "w") as fh:
                fh.write(state_str + "\n")
        except Exception as exc:
            self.get_logger().warn(
                f"safety_node: failed to persist kill state to "
                f"{self._kill_state_path!r}: {exc}.  "
                f"Continuing; kill state is still active in memory."
            )

    @staticmethod
    def _age_ms(stamp: Optional[float], now: float) -> float:
        """Compute age in milliseconds from a monotonic timestamp; +inf if None."""
        if stamp is None:
            return math.inf
        return (now - stamp) * 1000.0

    def _publish_zero(self) -> None:
        """Publish a zero Twist to /cmd_vel immediately (best-effort).

        Also resets the accel-limiter reference (_last_vx/vy/wz) to zero so
        that the next normal-path tick uses zero as the starting reference,
        matching what was actually published.  Without this reset, an exception
        in the control loop would leave _last_* at a non-zero pre-exception
        value while zero has been sent to the robot, and the next tick's accel
        limiter would permit a step that is larger than max_a * dt relative to
        the true robot state.
        """
        self._cmd_vel_pub.publish(Twist())
        # Sync the accel-limiter reference to the zero we just emitted.
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0

    # ------------------------------------------------------------------
    # Subscription callbacks — record data + arrival time only
    # ------------------------------------------------------------------

    def _command_cb(self, name: str) -> Callable[[Twist], None]:
        """The subscription callback for one table row: record its latest Twist."""
        buffer = self._commands[name]

        def cb(msg: Twist) -> None:
            buffer.vx = msg.linear.x
            buffer.vy = msg.linear.y
            buffer.wz = msg.angular.z
            buffer.stamp = time.monotonic()

        return cb

    def _select_command(self) -> Tuple[CommandSource, _CommandBuffer]:
        """The active row and its buffer. The ONLY place a buffer is read.

        `command_source` is validated against the table at startup and on
        every runtime set, so the lookup cannot miss; a KeyError here would
        be a bug, and the control loop's catch-all would publish zero for it.
        """
        source = self._sources[self._command_source]
        return source, self._commands[self._command_source]

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_ranges = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_increment = msg.angle_increment
        self._scan_range_min = msg.range_min
        self._scan_range_max = msg.range_max
        self._scan_stamp = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        # We do not consume odom velocity directly into EvalInputs because
        # safety_logic.evaluate() does not accept an observed velocity argument
        # — it only uses last_vx/vy/wz for the acceleration-limit context, and
        # those come from the previously published output velocity.  Odom is
        # tracked here solely for the staleness watchdog: if odom is stale, we
        # flag it and treat the situation as a stop condition in the control loop.
        self._odom_stamp = time.monotonic()

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Validate and apply runtime parameter changes.

        Only `command_source` is settable at runtime. Everything else that
        matters — the limits, the loop rate, the kill-state path — is read
        once at startup by design: a safety layer whose clamps can be widened
        by a `ros2 param set` is not a safety layer. Those are therefore
        REJECTED here rather than silently ignored, so an attempt to change
        one fails visibly at the caller.

        A `command_source` change is applied immediately AND force-kills. The
        kill is not a precaution about the switch itself; it is what makes the
        switch unable to start motion. Going teleop -> policy while unkilled
        would otherwise hand a live robot to the policy in one step, with no
        operator act in between.
        """
        # VALIDATE EVERYTHING FIRST, APPLY ONCE AFTER. A set carrying several
        # parameters is accepted or rejected as a whole by rcl, so applying
        # command_source inside the loop and then rejecting a later parameter
        # would leave this node forwarding the new source while
        # `ros2 param get` (and the dashboard's read-back) still showed the
        # old one. Fail-closed either way (the kill would have fired), but a
        # page saying 'teleop' over a node forwarding a policy is exactly the
        # mismatch an operator arms into.
        new_source: Optional[str] = None
        for param in params:
            if param.name in ("slow_zone_enabled", "stop_zone_enabled"):
                # Called out separately from the generic rejection because
                # this is the one an operator is most likely to reach for
                # mid-run, and the reason they need is different: it is not
                # "we happen to read this at startup", it is "the obstacle
                # e-stop is not something a live parameter set — or an
                # unauthenticated dashboard on 0.0.0.0 — gets to switch off".
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"{param.name!r} is NOT settable at runtime. Disabling "
                        f"the lidar e-stop is a bring-up decision: set it in "
                        f"config/safety_limits.yaml or pass it to the launch "
                        f"file, and restart the stack. /safety/kill is the "
                        f"runtime stop."
                    ),
                )
            if param.name != "command_source":
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"{param.name!r} is not settable at runtime; it is "
                        f"read once at startup. Restart the node to change it."
                    ),
                )
            value = param.value
            if value not in self._sources:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"command_source must be one of "
                        f"{sorted(self._sources)} (config/topics.yaml "
                        f"command_sources), got {value!r}"
                    ),
                )
            new_source = value

        if new_source is None or new_source == self._command_source:
            # No-op (same row): do not kill a running robot for nothing.
            return SetParametersResult(successful=True)

        previous = self._command_source
        self._command_source = new_source
        self._is_killed = True
        self._write_kill_state(True)
        self.get_logger().warn(
            f"safety_node: command_source {previous!r} -> {new_source!r}; "
            f"KILLED as a consequence. Call /safety/reset to arm."
        )
        return SetParametersResult(successful=True)

    def _kill_srv(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._is_killed = True
        self._write_kill_state(True)
        self.get_logger().warn("safety_node: KILL service called — robot stopped.")
        response.success = True
        response.message = "killed"
        return response

    def _reset_srv(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._is_killed = False
        self._reset_latch = True  # consumed on the next evaluate() tick
        self._write_kill_state(False)
        self.get_logger().info("safety_node: RESET service called — kill cleared.")
        response.success = True
        response.message = "reset"
        return response

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Timer callback: build inputs, evaluate, publish, update state."""
        try:
            self._control_loop_inner()
        except Exception as exc:
            # Fail-closed: any unhandled exception must still publish zero.
            self.get_logger().error(
                f"safety_node: unhandled exception in control loop: {exc} — "
                f"publishing zero velocity."
            )
            self._publish_zero()

    def _control_loop_inner(self) -> None:
        now = time.monotonic()

        # Slow repeat of the disabled-e-stop warning. No-op while the e-stop
        # is whole, so this costs one boolean check per tick in normal
        # operation.
        self._warn_zones_disabled_periodically(now)

        # ------------------------------------------------------------------
        # Compute dt since last tick
        # ------------------------------------------------------------------
        if self._last_tick_time is None:
            dt = 1.0 / self._loop_rate_hz  # first tick: assume nominal period
        else:
            dt = now - self._last_tick_time
        self._last_tick_time = now

        # ------------------------------------------------------------------
        # Odom staleness check
        # ------------------------------------------------------------------
        odom_age_ms = self._age_ms(self._odom_stamp, now)
        odom_timeout_ms = self._limits.timeouts.odom_timeout_ms
        self._odom_stale = odom_age_ms > odom_timeout_ms

        if self._odom_stale and self._odom_stamp is not None:
            # Only warn if we've ever received odom (not at startup before first
            # message — that is expected and not worth spamming the log).
            self.get_logger().warn(
                f"safety_node: /odom stale ({odom_age_ms:.0f} ms > "
                f"{odom_timeout_ms:.0f} ms) — treating as stop condition."
            )

        # ------------------------------------------------------------------
        # Active source selection: the one row named by command_source. Its
        # KIND (not its name) decides whether the policy-only checks apply,
        # so a new policy row gets the lateral-velocity check for free.
        # ------------------------------------------------------------------
        source, command = self._select_command()
        is_policy_source = source.is_policy
        cmd_vx = command.vx
        cmd_vy = command.vy
        cmd_wz = command.wz

        command_age_ms = self._age_ms(command.stamp, now)

        # ------------------------------------------------------------------
        # Build EvalInputs
        # command_age_ms reflects the actual active-source age only; odom
        # staleness is passed as a dedicated field so evaluate() can return a
        # correctly labelled "odom_stale" reason rather than the misleading
        # "command_timeout" that the previous inflation workaround produced.
        # ------------------------------------------------------------------
        inputs = EvalInputs(
            cmd_vx=cmd_vx,
            cmd_vy=cmd_vy,
            cmd_wz=cmd_wz,
            is_policy_source=is_policy_source,
            command_age_ms=command_age_ms,
            last_vx=self._last_vx,
            last_vy=self._last_vy,
            last_wz=self._last_wz,
            dt=dt,
            scan_ranges=self._scan_ranges,
            scan_angle_min=self._scan_angle_min,
            scan_angle_increment=self._scan_angle_increment,
            scan_range_min=self._scan_range_min,
            scan_range_max=self._scan_range_max,
            scan_age_ms=self._age_ms(self._scan_stamp, now),
            is_killed=self._is_killed,
            estop_latched=self._estop_latched,
            consecutive_stop_count=self._consecutive_stop_count,
            reset_latch=self._reset_latch,
            odom_stale=self._odom_stale,
        )

        # Consume the reset_latch flag — it is a one-shot per reset service call.
        self._reset_latch = False

        # ------------------------------------------------------------------
        # Evaluate — the pure logic layer makes all decisions
        # ------------------------------------------------------------------
        decision = evaluate(inputs, self._limits)

        # Log policy vy violation if the active source is policy and clamping
        # is active (the violation detection and zeroing happens inside
        # safety_logic; the node is responsible for the warning log per the
        # policy_vy_check docstring).
        if is_policy_source and decision.clamping_active and abs(cmd_vy) > 1e-6:
            self.get_logger().warn(
                f"safety_node: {source.topic} had non-zero linear.y "
                f"({cmd_vy:.6f}) — this is a bug in the policy node (the "
                f"action space is [vx, wz] only).  Zeroed and warned."
            )

        # ------------------------------------------------------------------
        # Update carried state
        # ------------------------------------------------------------------
        self._estop_latched = decision.estop_latched
        self._consecutive_stop_count = decision.consecutive_stop_count
        self._last_vx = decision.out_vx
        self._last_vy = decision.out_vy
        self._last_wz = decision.out_wz

        # ------------------------------------------------------------------
        # Publish /cmd_vel — ALWAYS, every tick
        # Continuous-zero guarantee: even when killed, evaluate() returns zero
        # and we publish it.  This loop runs at >= zero_publish_rate_hz.
        # ------------------------------------------------------------------
        cmd_out = Twist()
        cmd_out.linear.x = decision.out_vx
        cmd_out.linear.y = decision.out_vy
        cmd_out.linear.z = 0.0
        cmd_out.angular.x = 0.0
        cmd_out.angular.y = 0.0
        cmd_out.angular.z = decision.out_wz
        self._cmd_vel_pub.publish(cmd_out)

        # ------------------------------------------------------------------
        # Publish /safety/status (throttled to status_rate_hz)
        # ------------------------------------------------------------------
        self._tick_counter += 1
        if self._tick_counter >= self._status_period_ticks:
            self._tick_counter = 0
            status = SafetyStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = ""
            status.state = int(decision.state)
            status.reason = decision.reason
            # float32 saturates at +inf; math.inf is a valid IEEE-754 float32 inf.
            status.min_range = float(decision.min_range)
            status.clamping_active = decision.clamping_active
            self._status_pub.publish(status)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """Initialise rclpy, spin SafetyNode, publish a final zero on shutdown."""
    rclpy.init(args=args)
    node: Optional[SafetyNode] = None
    try:
        node = SafetyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # If the node fails to construct (e.g. bad config), log and exit.
        # We cannot publish zero here because the publisher may not exist yet.
        if node is not None:
            node.get_logger().fatal(f"safety_node: fatal error: {exc}")
        else:
            print(f"safety_node: fatal error during init: {exc}")
        raise
    finally:
        if node is not None:
            # Best-effort final zero publish so the robot is left stopped.
            try:
                node._publish_zero()
            except Exception:
                pass
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
