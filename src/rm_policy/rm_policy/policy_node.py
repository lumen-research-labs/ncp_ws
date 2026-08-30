"""
policy_node.py — ROS2 node wrapper that runs the trained NCP policy at the
control rate and publishes a candidate command for the safety layer.

What it does:
    Every tick (at `control.rate_hz` from `config/interface.yaml`, currently
    5 Hz):
      1. Checks `/scan` and `/odom` staleness against the SAME timeout
         constants `rm_safety` uses (`config/safety_limits.yaml`
         `timeouts.scan_timeout_ms` / `odom_timeout_ms`), so this node goes
         quiet on exactly the same schedule the safety layer would already
         be treating the robot as blind.
      2. Confirms the goal is defined. The goal is a STATIC point in the
         odom frame, fixed ONCE from the first `/odom` message (start pose +
         the `(goal_x, goal_y)` launch-parameter offset, rotated by the
         start yaw). Until the first `/odom` message has ever arrived, the
         goal does not exist and this node MUST NOT drive — no default goal
         vector is ever substituted (interface.yaml `goal_local.failure_mode`
         is explicit that a plausible-looking goal is worse than no goal).
      3. Calls `rm_policy.observation_builder` to assemble the 36-element
         observation (32 lidar + 2 dynamic robot-frame goal + 2 previous
         action). This node does not build or normalize the vector itself —
         see the ASSUMED API note below.
      4. Calls `rm_policy.policy_runner.PolicyRunner.act()` to get `(v, w)`
         in PRE-SCALED PHYSICAL UNITS (m/s, rad/s). The trained actor's
         linear head + clip already produced final units — this node must
         NOT multiply by any additional scaling (`config/interface.yaml`
         `action.scaling.applies: false`; double-scaling here is a silent,
         no-runtime-error numerical bug).
      5. Publishes a `Twist` to `/policy/cmd_vel` — an INPUT to `rm_safety`,
         never `/cmd_vel` directly. `linear.y` is set to `0.0` EXPLICITLY on
         every message (never left at its zero default) because the action
         space has no lateral component and that must be a visible claim in
         the code and in the bag, not an accident of an unset field.
      6. Feeds back the REQUESTED action (what this node asked for, not
         what `rm_safety` may have clamped it to) as `previous_action` for
         the next tick's observation. Training had no safety layer, so
         emitted == applied there; feeding back the requested action keeps
         the observation distribution identical to training. Known,
         accepted consequence: during any period `rm_safety` clamps or
         zeroes the command, the policy's own model of "what I last did"
         disagrees with what the robot actually did.

    On ANY fault (stale scan, stale odom, undefined goal, observation-builder
    fault, inference fault, NaN in the final command, or an unhandled
    exception) the tick publishes a Twist of all zeros and does NOT update
    `previous_action` — a failed tick must not let a stale/garbage command
    persist, and must not corrupt the fed-back action history either.

Subscribes:  /scan   [sensor_msgs/LaserScan]   @ ~10.07 Hz (interface.yaml)
Subscribes:  /odom   [nav_msgs/Odometry]       @ ~10.74 Hz (interface.yaml)
Publishes:   /policy/cmd_vel  [geometry_msgs/Twist]  @ control.rate_hz (5 Hz)
Publishes:   /ncp_debug       [std_msgs/String]      @ control.rate_hz (5 Hz)
Parameters:  scan_topic       (str, "/scan")
Parameters:  odom_topic       (str, "/odom")
Parameters:  cmd_vel_topic    (str, "/policy/cmd_vel") — MUST NOT be /cmd_vel;
                              refused at startup if set to /cmd_vel.
Parameters:  debug_topic      (str, "/ncp_debug")
Parameters:  goal_x           (float, 2.5) — metres, robot-frame offset from
                              the start pose, per interface.yaml goal_default_m.
                              SETTABLE AT RUNTIME: a set re-fixes the goal from
                              the CURRENT pose (always, even to an unchanged
                              value) and resets the recurrent state.
Parameters:  goal_y           (float, 0.0) — metres, ditto
Parameters:  weights          (str, "") — path to a trained actor state_dict;
                              "" means random weights (no checkpoint applied).
                              Read once at startup; a runtime set is REJECTED.
                              Its sha256 is logged at startup and published on
                              /ncp_debug, so a run is traceable to exact bytes.
Parameters:  interface_config_path (str, <repo-relative default>) — path to
                              config/interface.yaml
Parameters:  safety_limits_path    (str, <repo-relative default>) — path to
                              config/safety_limits.yaml, for the scan/odom
                              timeout constants shared with rm_safety
Parameters:  pose_source      (str, "odom") — "odom" reads the pose from /odom
                              (dead reckoning, drifts); "map" looks up
                              map -> base_footprint via TF, which slam_toolbox
                              corrects. In "map" mode the goal is fixed in the
                              MAP frame on the first usable pose, so it stops
                              migrating with odometry drift. Read once at
                              startup; a runtime set is REJECTED.
Failure mode:
    - Node refuses to construct (raises, never starts spinning) if:
        * `config/interface.yaml` cannot be read, or its `interface_version`
          does not match `_EXPECTED_INTERFACE_VERSION` — a version mismatch
          means the model and this code disagree about vector layout, which
          has no runtime error otherwise.
        * `config/safety_limits.yaml` cannot be read.
        * `cmd_vel_topic` resolves to `/cmd_vel`.
        * `weights` is non-empty and fails to load (see policy_runner.py).
    - Before the first `/odom` message: goal undefined -> publish zero every
      tick, throttled warning. No motion is possible on startup.
    - `pose_source:="map"` and the map -> base_footprint transform missing,
      unconnected, un-extrapolatable or stale: publish zero, reason
      `pose_tf_unavailable:*` / `pose_stale:*`. NEVER falls back to /odom —
      a silent downgrade to the drifting pose is exactly the failure this
      mode exists to remove, and it would be invisible.
    - A pose DISCONTINUITY (a SLAM loop closure or relocalization) does not
      stop the robot: the corrected pose is adopted and the policy runner's
      recurrent state is reset, because the hidden state was accumulated
      toward a goal position that has just moved. See pose_source.py.
    - Stale `/scan` or `/odom` (age > the shared timeout from
      safety_limits.yaml): publish zero, throttled warning, and reset the
      policy runner's recurrent state (so a resumed session does not carry
      a hidden state built across a discontinuous input history).
    - `observation_builder` or `policy_runner` reporting a fault: publish
      zero, throttled warning, previous_action left unchanged.
    - NaN/inf anywhere in the final `(v, w)`: publish zero, throttled
      warning — belt-and-braces on top of `policy_runner`'s own validation.
    - Any unhandled exception inside the control-loop tick: caught, zero
      published, logged at ERROR. An exception must never skip the publish.
    - On shutdown: best-effort final zero Twist, matching `rm_safety`'s
      node-lifecycle convention.
"""

from __future__ import annotations

import json
import math
import pathlib
import time
from typing import Optional, Sequence, Tuple

import rclpy
import rclpy.node
import yaml
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros
from rm_policy.observation_builder import build_observation
from rm_policy.policy_runner import PolicyRunner
from rm_policy.pose_source import Pose2D, detect_pose_jump, pose_is_usable

# ---------------------------------------------------------------------------
# Interface-contract version this node was written against. Loaded-and-checked
# against config/interface.yaml's own `interface_version` at startup — see
# `_load_interface_config`. A mismatch means the observation/action layout
# this code assumes and the layout the checkpoint/contract actually specifies
# have silently diverged, which otherwise produces no runtime error.
# ---------------------------------------------------------------------------
_EXPECTED_INTERFACE_VERSION: str = "3.0.0"

# ---------------------------------------------------------------------------
# Default config paths — only valid in a standard colcon workspace layout
# where the package source tree is adjacent to config/. Matches
# rm_safety.safety_node's `_DEFAULT_LIMITS_PATH` resolution convention.
# Prefer passing these explicitly via a launch file parameter.
# ---------------------------------------------------------------------------
_THIS_DIR = pathlib.Path(__file__).parent.resolve()
_DEFAULT_INTERFACE_CONFIG_PATH = str(_THIS_DIR.parents[2] / "config" / "interface.yaml")
_DEFAULT_SAFETY_LIMITS_PATH = str(_THIS_DIR.parents[2] / "config" / "safety_limits.yaml")

# ---------------------------------------------------------------------------
# Default topic names.
# ---------------------------------------------------------------------------
_DEFAULT_SCAN_TOPIC: str = "/scan"
_DEFAULT_ODOM_TOPIC: str = "/odom"
_DEFAULT_CMD_VEL_TOPIC: str = "/policy/cmd_vel"
_DEFAULT_DEBUG_TOPIC: str = "/ncp_debug"

# ---------------------------------------------------------------------------
# `/cmd_vel` is the ONE topic this node must never publish to. `rm_safety` is
# its sole publisher (see CLAUDE.md ground rules). This is a hard-coded
# comparison target, not a magic number in the config sense — it names the
# one topic string that must never appear as this node's output.
# ---------------------------------------------------------------------------
_FORBIDDEN_CMD_VEL_TOPIC: str = "/cmd_vel"

# ---------------------------------------------------------------------------
# Default goal offset, metres, robot frame relative to the start pose.
# Sourced from config/interface.yaml `observation.layout[goal_local].
# goal_default_m: [2.5, 0.0]` (operator decision 2026-08-27) and re-declared
# here as ROS parameter defaults, which is the standard ROS2 mechanism for
# operator-overridable values — not a duplicate literal invented independently
# of the config.
# ---------------------------------------------------------------------------
_DEFAULT_GOAL_X_M: float = 2.5
_DEFAULT_GOAL_Y_M: float = 0.0

# Default weights path: "" means random weights (no trained checkpoint
# applied). Matches policy_runner.PolicyRunner's own default.
_DEFAULT_WEIGHTS_PATH: str = ""

# Where the goal block sits in the 36-dim observation, and the constant its
# normalization divides by. Both are fixed by the trained checkpoint
# (config/interface.yaml observation.layout: 32 lidar, then goal_local, then
# previous_action; `normalization: "xy / 5.0 - 1.0"`). Used ONLY to invert the
# normalization for the debug stream — never to build the vector.
# Pose sources. "odom" reads the pose straight from /odom (the pre-SLAM
# behaviour and still the default). "map" looks up map -> base_footprint via
# TF, which is what slam_toolbox corrects. See pose_source.py.
_POSE_SOURCE_ODOM: str = "odom"
_POSE_SOURCE_MAP: str = "map"

# Frames for the map pose lookup. base_footprint, not base_link: /odom's
# child_frame_id is base_footprint and that is what the EKF and slam_toolbox
# both terminate at (interface.yaml coordinate_frames).
_MAP_FRAME: str = "map"
_BASE_FRAME: str = "base_footprint"

_GOAL_SLICE_START: int = 32
_GOAL_NORM_SCALE_M: float = 5.0

# Throttle period (seconds) for repeated warning logs so a sustained fault
# does not flood the log at the full control rate.
_WARN_THROTTLE_SEC: float = 5.0


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (radians, REP-103 z-up) from a geometry_msgs quaternion.

    Plain message-field arithmetic, not an observation/normalization formula
    — this is quaternion-to-Euler conversion, not part of the frozen
    obs/action contract, so it belongs here rather than in
    observation_builder (which consumes an already-extracted yaw float).
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotate_2d(x: float, y: float, theta: float) -> Tuple[float, float]:
    """Rotate a 2D vector by `theta` radians (standard CCW rotation)."""
    c, s = math.cos(theta), math.sin(theta)
    return (c * x - s * y, s * x + c * y)


class PolicyNode(rclpy.node.Node):
    """ROS2 node: runs the trained NCP policy; publishes to /policy/cmd_vel."""

    def __init__(self) -> None:
        super().__init__("policy_node")

        # ------------------------------------------------------------------
        # Declare and read parameters
        # ------------------------------------------------------------------
        self.declare_parameter("scan_topic", _DEFAULT_SCAN_TOPIC)
        self.declare_parameter("odom_topic", _DEFAULT_ODOM_TOPIC)
        self.declare_parameter("cmd_vel_topic", _DEFAULT_CMD_VEL_TOPIC)
        self.declare_parameter("debug_topic", _DEFAULT_DEBUG_TOPIC)
        self.declare_parameter("goal_x", _DEFAULT_GOAL_X_M)
        self.declare_parameter("goal_y", _DEFAULT_GOAL_Y_M)
        self.declare_parameter("weights", _DEFAULT_WEIGHTS_PATH)
        self.declare_parameter("interface_config_path", _DEFAULT_INTERFACE_CONFIG_PATH)
        self.declare_parameter("safety_limits_path", _DEFAULT_SAFETY_LIMITS_PATH)
        self.declare_parameter("pose_source", _POSE_SOURCE_ODOM)

        scan_topic = self._get_str_param("scan_topic")
        odom_topic = self._get_str_param("odom_topic")
        cmd_vel_topic = self._get_str_param("cmd_vel_topic")
        debug_topic = self._get_str_param("debug_topic")
        self._goal_offset_x: float = self._get_double_param("goal_x")
        self._goal_offset_y: float = self._get_double_param("goal_y")
        weights_path = self._get_str_param("weights")
        interface_config_path = self._get_str_param("interface_config_path")
        safety_limits_path = self._get_str_param("safety_limits_path")
        self._pose_source = self._get_str_param("pose_source")
        if self._pose_source not in (_POSE_SOURCE_ODOM, _POSE_SOURCE_MAP):
            raise ValueError(
                f"policy_node: pose_source must be {_POSE_SOURCE_ODOM!r} or "
                f"{_POSE_SOURCE_MAP!r}, got {self._pose_source!r}"
            )

        # ------------------------------------------------------------------
        # HARD RULE: never publish to /cmd_vel from this node. rm_safety is
        # the sole publisher. Fail loudly at construction, not silently at
        # publish time.
        # ------------------------------------------------------------------
        if cmd_vel_topic == _FORBIDDEN_CMD_VEL_TOPIC:
            raise ValueError(
                f"policy_node: cmd_vel_topic must not be {_FORBIDDEN_CMD_VEL_TOPIC!r}. "
                f"This node publishes candidate commands to rm_safety's input "
                f"topics only (default /policy/cmd_vel); rm_safety is the sole "
                f"publisher of /cmd_vel."
            )

        # ------------------------------------------------------------------
        # Load and validate config/interface.yaml — fatal on version mismatch
        # or unreadable file. The node must not start with an unknown or
        # wrong observation/action contract.
        # ------------------------------------------------------------------
        interface_cfg = self._load_yaml(interface_config_path, "interface.yaml")
        interface_version = interface_cfg.get("interface_version")
        if interface_version != _EXPECTED_INTERFACE_VERSION:
            raise RuntimeError(
                f"policy_node: interface_version mismatch — this node was "
                f"written against {_EXPECTED_INTERFACE_VERSION!r}, but "
                f"{interface_config_path!r} declares {interface_version!r}. "
                f"Refusing to start: the model and the code disagree about "
                f"the observation/action vector layout."
            )
        try:
            self._control_rate_hz: float = float(interface_cfg["control"]["rate_hz"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f"policy_node: interface.yaml missing/invalid "
                f"control.rate_hz: {exc}"
            ) from exc

        # ------------------------------------------------------------------
        # Load config/safety_limits.yaml — ONLY for the scan/odom staleness
        # timeout constants, reused verbatim so this node goes quiet on
        # exactly the schedule rm_safety already treats as unsafe. This is a
        # config-file read, not a Python import of rm_safety — rm_tools/
        # rm_policy must never import rm_safety at runtime, and they don't
        # here; both packages just parse the same YAML file independently.
        # ------------------------------------------------------------------
        safety_cfg = self._load_yaml(safety_limits_path, "safety_limits.yaml")
        try:
            self._scan_timeout_ms: float = float(
                safety_cfg["timeouts"]["scan_timeout_ms"]
            )
            self._odom_timeout_ms: float = float(
                safety_cfg["timeouts"]["odom_timeout_ms"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f"policy_node: safety_limits.yaml missing/invalid "
                f"timeouts.scan_timeout_ms / odom_timeout_ms: {exc}"
            ) from exc

        # Freshness bound on the pose, whichever source it comes from. Reuses
        # the odom timeout rather than inventing a second constant: a pose is
        # an odometry-derived quantity in both modes (map mode corrects it,
        # but the odom -> base_footprint edge is still underneath), so the two
        # should go stale on the same schedule. Deriving it also means it
        # cannot drift out of step with rm_safety's own watchdog.
        self._pose_max_age_s: float = self._odom_timeout_ms / 1000.0

        # ------------------------------------------------------------------
        # Policy runner — constructed (and weights loaded, if given) before
        # any subscription/timer exists, so a bad weights file prevents the
        # node from starting at all.
        # ------------------------------------------------------------------
        self._runner = PolicyRunner(weights_path=weights_path)

        # ------------------------------------------------------------------
        # Sensor state — everything starts "never received" (infinitely
        # stale), matching rm_safety.safety_node's fail-closed convention.
        # ------------------------------------------------------------------
        self._scan_ranges: list = []
        self._scan_angle_min: float = 0.0
        self._scan_angle_increment: float = 0.0
        self._scan_stamp: Optional[float] = None

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._odom_stamp: Optional[float] = None

        # Goal is UNDEFINED until the first /odom message arrives. No default
        # is ever substituted — see interface.yaml goal_local.failure_mode.
        self._goal_world_x: Optional[float] = None
        self._goal_world_y: Optional[float] = None

        # Previous action fed back into the next observation: the REQUESTED
        # action (what this node asked for), not what rm_safety may have
        # clamped it to. See module docstring point 6 / interface.yaml
        # previous_action.clamped_action_note. Zero on the first tick, per
        # interface.yaml's init_note (a zero ACTION, not a "do nothing" flag).
        self._prev_action_v: float = 0.0
        self._prev_action_w: float = 0.0

        self._last_scan_stale: bool = True
        self._last_odom_stale: bool = True

        # Previous pose sample, for jump detection. A SLAM correction is a
        # step change in the pose and therefore in goal_local, which the CfC
        # reads — see pose_source.py for why a stateful network makes the
        # recurrent-state reset mandatory rather than tidy.
        self._prev_pose: Optional[Pose2D] = None

        # ------------------------------------------------------------------
        # QoS — best-effort/volatile, matching rm_safety's sensor QoS.
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------------------------------------------------
        # Publishers — NEVER /cmd_vel. cmd_vel_topic defaults to
        # /policy/cmd_vel, an INPUT to rm_safety.
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._debug_pub = self.create_publisher(String, debug_topic, 10)

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, sensor_qos)
        # /odom is subscribed in BOTH pose modes. In "map" mode the pose comes
        # from TF, but this subscription still drives the odom staleness
        # watchdog — which mirrors rm_safety's, and rm_safety zeroes every
        # command on odom_stale at precedence 3 regardless of what this node
        # thinks. Dropping it in map mode would let this node keep commanding
        # while the safety layer had already gone quiet, which is a confusing
        # disagreement to debug and buys nothing.
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)

        # TF, only in map mode. Creating a TransformListener spawns a
        # subscription to /tf and /tf_static, so it is not free; in odom mode
        # nothing here is used and it is left unbuilt.
        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._tf_listener = None
        if self._pose_source == _POSE_SOURCE_MAP:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------
        # Control loop timer — rate sourced from interface.yaml control.rate_hz.
        # ------------------------------------------------------------------
        loop_period = 1.0 / self._control_rate_hz
        self._control_timer = self.create_timer(loop_period, self._control_loop)

        # Runtime re-targeting. Registered AFTER the goal state and the runner
        # exist, since the callback touches both.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"policy_node ready: cmd_vel_topic={cmd_vel_topic!r}, "
            f"rate={self._control_rate_hz} Hz, weights={weights_path or '<random>'!r}, "
            f"goal_offset=({self._goal_offset_x}, {self._goal_offset_y}) m — "
            f"NOT YET APPLIED (undefined until the first /odom message)."
        )
        # Provenance, logged once at full length. The short form goes out on
        # /ncp_debug every tick; this is the line to grep when asking months
        # later which bytes produced a recorded run.
        if self._runner.weights_sha256:
            self.get_logger().info(
                f"policy_node checkpoint sha256={self._runner.weights_sha256}"
            )
        else:
            self.get_logger().warn(
                "policy_node is running RANDOM WEIGHTS — no checkpoint was "
                "loaded. Any behaviour observed is the PLUMBING, not a policy."
            )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _get_str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_double_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    @staticmethod
    def _load_yaml(path: str, label: str) -> dict:
        """Load a YAML config file; raises on any failure (fatal)."""
        try:
            with open(path, "r") as fh:
                return yaml.safe_load(fh)
        except Exception as exc:
            raise RuntimeError(
                f"policy_node: cannot load {label} from {path!r}: {exc}"
            ) from exc

    # ------------------------------------------------------------------
    # Subscription callbacks — record data + arrival time only; the control
    # loop does all the decision-making, matching rm_safety.safety_node's
    # split between I/O callbacks and the timer-driven evaluation.
    # ------------------------------------------------------------------

    def _current_pose(self, now: float) -> Tuple[Optional[Pose2D], Optional[str]]:
        """The robot's pose for this tick, or a fault reason.

        In "odom" mode this is whatever `_odom_cb` last recorded. In "map"
        mode it is a TF lookup of map -> base_footprint, which is the pose
        slam_toolbox corrects; `/odom` is still subscribed, but only to drive
        the staleness watchdog.

        Returns `(pose, None)` or `(None, reason)`. There is no third outcome
        and no default pose: a substituted pose is a plausible-looking lie
        about where the robot is, and every downstream number — goal_local,
        the observation, the command — would inherit it silently.
        """
        if self._pose_source == _POSE_SOURCE_ODOM:
            if self._odom_stamp is None:
                return None, "pose_missing"
            return Pose2D(
                x=self._robot_x, y=self._robot_y, yaw=self._robot_yaw,
                stamp=self._odom_stamp,
            ), None

        # --- map mode: TF ---
        if self._tf_buffer is None:  # unreachable; defensive
            return None, "pose_no_tf_buffer"
        try:
            # rclpy.time.Time() means "latest available" rather than a
            # specific stamp. Asking for `now` instead invites extrapolation
            # errors on a robot whose clock has been pushed by hand, and the
            # freshness of what comes back is checked below anyway — which is
            # the honest way round: take what exists, then judge its age.
            tf = self._tf_buffer.lookup_transform(
                _MAP_FRAME, _BASE_FRAME, rclpy.time.Time()
            )
        except Exception as exc:
            # Covers the whole tf2 exception family (LookupException,
            # ConnectivityException, ExtrapolationException) plus anything
            # else the buffer raises. All of them mean the same thing here:
            # no trustworthy pose, so do not drive.
            return None, f"pose_tf_unavailable:{type(exc).__name__}"

        t = tf.transform.translation
        q = tf.transform.rotation

        # TWO CLOCKS, EACH USED CONSISTENTLY WITH ITSELF. This is the whole
        # trick, and getting it wrong is not theoretical — an earlier revision
        # tried to map the transform's ROS stamp onto this node's monotonic
        # clock via an offset captured at the first lookup. If that first
        # lookup happened to return an already-old transform, the offset was
        # poisoned permanently and EVERY subsequent pose read as stamped in
        # the future. Observed on hardware: `pose_future_stamp` every tick,
        # forever, with a perfectly healthy SLAM underneath.
        #
        #   FRESHNESS  -> ROS stamp vs the ROS clock. Both wall time, so the
        #                 difference is a real age even on a robot whose clock
        #                 gets pushed by hand (bernard-1 has no NTP).
        #   JUMP dt    -> monotonic, below. Never wall time: a hand-corrected
        #                 clock would otherwise register as a pose teleport.
        stamp_s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        age_s = ros_now - stamp_s
        if age_s > self._pose_max_age_s:
            return None, f"pose_stale:{age_s * 1000.0:.0f}ms"

        # The Pose2D carries the MONOTONIC stamp, because the only thing that
        # reads it is the jump detector's dt.
        return Pose2D(
            x=float(t.x), y=float(t.y),
            yaw=_yaw_from_quaternion(q.x, q.y, q.z, q.w),
            stamp=now,
        ), None

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Runtime re-targeting: setting `goal_x`/`goal_y` re-fixes the goal
        from the robot's CURRENT pose.

        Only the goal offsets are settable. Topics, the weights path and the
        config paths are structural — changing them at runtime would mean a
        half-reconfigured node, so a set is REJECTED rather than accepted and
        ignored (the failure mode this project already hit once with
        rm_safety's command_source).

        Semantics worth being explicit about: a set ALWAYS re-fixes the goal,
        even to the value it already had. Writing a parameter is an explicit
        operator act, and "re-target 2.5 m ahead of wherever I am now" is the
        thing that act means during a debug run — treating an unchanged value
        as a no-op would make the obvious way to re-issue a goal do nothing.

        The recurrent state is reset on every re-target. A CfC carries its
        history, and the objective changing underneath it is exactly the
        discontinuity `PolicyRunner.reset()` exists for: without the reset the
        network would be steering with a hidden state accumulated toward a
        goal that no longer exists.
        """
        offsets = {}
        for param in params:
            if param.name not in ("goal_x", "goal_y"):
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"{param.name!r} is not settable at runtime; it is "
                        f"read once at startup. Restart the node to change it."
                    ),
                )
            value = float(param.value)
            if not math.isfinite(value):
                return SetParametersResult(
                    successful=False, reason=f"{param.name} must be finite"
                )
            offsets[param.name] = value

        self._goal_offset_x = offsets.get("goal_x", self._goal_offset_x)
        self._goal_offset_y = offsets.get("goal_y", self._goal_offset_y)

        if self._odom_stamp is None:
            # No pose yet, so there is nothing to re-fix against. The offsets
            # are stored and the normal first-/odom path will apply them —
            # this is not an error, and it must NOT invent a pose.
            self.get_logger().warn(
                "policy_node: goal offset updated to "
                f"({self._goal_offset_x}, {self._goal_offset_y}) but no /odom "
                "has arrived — goal stays UNDEFINED until it does."
            )
            return SetParametersResult(successful=True)

        dx, dy = _rotate_2d(
            self._goal_offset_x, self._goal_offset_y, self._robot_yaw
        )
        self._goal_world_x = self._robot_x + dx
        self._goal_world_y = self._robot_y + dy
        self._runner.reset()
        self.get_logger().info(
            f"policy_node: re-targeted to offset "
            f"({self._goal_offset_x}, {self._goal_offset_y}) m from the current "
            f"pose — goal now ({self._goal_world_x:.3f}, "
            f"{self._goal_world_y:.3f}) in "
            f"{_MAP_FRAME if self._pose_source == _POSE_SOURCE_MAP else 'odom'}"
            f"; recurrent state reset."
        )
        return SetParametersResult(successful=True)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_ranges = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_increment = msg.angle_increment
        self._scan_stamp = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)

        self._robot_x = pos.x
        self._robot_y = pos.y
        self._robot_yaw = yaw
        self._odom_stamp = time.monotonic()

        if self._goal_world_x is None and self._pose_source == _POSE_SOURCE_ODOM:
            # Fix the goal ONCE, from the first /odom message: start position
            # plus the launch-parameter (goal_x, goal_y) offset, rotated by
            # the start yaw into the world (odom) frame. This is a one-time
            # frame conversion done here at "start pose capture" time — it is
            # NOT the per-tick robot-frame `local_goal()` transform that
            # observation_builder performs every tick against this fixed
            # point (see interface.yaml goal_local.preprocessing). The two
            # are different operations and this one is not part of the
            # frozen per-tick observation formula, so computing it here does
            # not duplicate anything observation_builder owns.
            dx, dy = _rotate_2d(self._goal_offset_x, self._goal_offset_y, yaw)
            self._goal_world_x = pos.x + dx
            self._goal_world_y = pos.y + dy
            self.get_logger().info(
                f"policy_node: goal fixed at odom-frame "
                f"({self._goal_world_x:.3f}, {self._goal_world_y:.3f}) m "
                f"(start ({pos.x:.3f}, {pos.y:.3f}), yaw {yaw:.3f} rad, "
                f"offset ({self._goal_offset_x}, {self._goal_offset_y}) m)."
            )

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def _publish_zero(self, reason: str) -> None:
        """Publish a zero Twist to cmd_vel_topic and a debug message.

        Deliberately does NOT update `self._prev_action_{v,w}` — a failed
        tick must not corrupt the fed-back action history with a value the
        policy never asked for.
        """
        cmd = Twist()
        cmd.linear.y = 0.0  # explicit, never left at the field default
        self._cmd_pub.publish(cmd)
        self._publish_debug(fault=reason, action=None)

    def _publish_debug(
        self,
        *,
        fault: Optional[str],
        action: Optional[Tuple[float, float]],
        obs: Optional[Sequence[float]] = None,
        goal_local: Optional[Tuple[float, float]] = None,
    ) -> None:
        """Publish a one-line JSON status, mirroring the upstream /ncp_debug
        convention, so "is it seeing anything?" is answerable from the
        terminal without decoding Twist messages.

        This is also the ONLY place the observation vector the network
        actually consumed becomes visible outside this process. rm_dashboard
        renders it from here rather than rebuilding it, because a second
        implementation of the observation build would be a second thing that
        can be wrong — and it would be wrong silently, showing a plausible
        vector that is not the one the policy saw. `obs` and `goal_local` are
        None on every fault path, which is correct: on a fault no observation
        was consumed, and emitting the previous one would misreport the tick.
        """
        min_range = None
        if self._scan_ranges:
            finite = [r for r in self._scan_ranges if math.isfinite(r) and r > 0.0]
            if finite:
                min_range = round(min(finite), 3)

        status = {
            "fault": fault,
            "goal_defined": self._goal_world_x is not None,
            "min_range_m": min_range,
            "action": (
                [round(action[0], 3), round(action[1], 3)]
                if action is not None
                else None
            ),
            # Which pose the goal is being computed from. Worth publishing:
            # "the goal is drifting" and "the goal is fixed in a corrected
            # frame" look identical on a plot until you know which mode ran.
            "pose_source": self._pose_source,
            "weights": self._runner.weights_path or None,
            # Short hash is enough to tell two checkpoints apart at a glance
            # while keeping this message small at 5 Hz; the full hash is in
            # the startup log line. "" (random weights) publishes as None so
            # an untrained run is unmistakable rather than looking like a
            # missing field.
            "weights_sha256": (self._runner.weights_sha256 or None) and
                              self._runner.weights_sha256[:12],
            "obs": [round(float(x), 4) for x in obs] if obs is not None else None,
            "goal_local_m": (
                [round(goal_local[0], 3), round(goal_local[1], 3)]
                if goal_local is not None
                else None
            ),
            "pose": (
                [round(self._robot_x, 3), round(self._robot_y, 3),
                 round(self._robot_yaw, 4)]
                if self._goal_world_x is not None
                else None
            ),
            "goal_world_m": (
                [round(self._goal_world_x, 3), round(self._goal_world_y, 3)]
                if self._goal_world_x is not None
                and self._goal_world_y is not None
                else None
            ),
        }
        self._debug_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Timer callback: fail-closed wrapper around `_control_loop_inner`."""
        try:
            self._control_loop_inner()
        except Exception as exc:
            self.get_logger().error(
                f"policy_node: unhandled exception in control loop: {exc} — "
                f"publishing zero velocity."
            )
            self._publish_zero(reason=f"internal_error:{exc!r}")

    def _control_loop_inner(self) -> None:
        now = time.monotonic()

        # ------------------------------------------------------------------
        # Staleness watchdogs — same timeout constants rm_safety uses.
        # ------------------------------------------------------------------
        scan_age_ms = self._age_ms(self._scan_stamp, now)
        odom_age_ms = self._age_ms(self._odom_stamp, now)
        scan_stale = scan_age_ms > self._scan_timeout_ms
        odom_stale = odom_age_ms > self._odom_timeout_ms

        # Reset the recurrent state on the transition INTO a stale period
        # (not every tick while it remains stale) so a long-stale robot
        # doesn't spam resets, but a resumed session never carries a hidden
        # state built across the gap.
        if (scan_stale and not self._last_scan_stale) or (
            odom_stale and not self._last_odom_stale
        ):
            self._runner.reset()
        self._last_scan_stale = scan_stale
        self._last_odom_stale = odom_stale

        if scan_stale:
            self.get_logger().warn(
                f"policy_node: /scan stale ({scan_age_ms:.0f} ms > "
                f"{self._scan_timeout_ms:.0f} ms) — publishing zero.",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            self._publish_zero(reason="scan_stale")
            return

        if odom_stale:
            self.get_logger().warn(
                f"policy_node: /odom stale ({odom_age_ms:.0f} ms > "
                f"{self._odom_timeout_ms:.0f} ms) — publishing zero.",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            self._publish_zero(reason="odom_stale")
            return

        # ------------------------------------------------------------------
        # Pose. In map mode this is the SLAM-corrected TF pose; in odom mode
        # it is /odom's, exactly as before. Fail-closed either way.
        # ------------------------------------------------------------------
        pose, pose_fault = self._current_pose(now)
        if pose_fault is None:
            pose_fault = pose_is_usable(pose, now, self._pose_max_age_s)
        if pose_fault is not None or pose is None:
            self.get_logger().warn(
                f"policy_node: pose unavailable ({pose_fault}) — publishing "
                f"zero.",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            self._publish_zero(reason=str(pose_fault))
            return

        # JUMP DETECTION. A corrected pose is a BETTER pose, so it is adopted
        # — what is discarded is the recurrent state, which was accumulated
        # while steering toward where the goal appeared to be before the
        # correction. Not resetting here is the failure that would be hardest
        # to see: the policy would keep acting on a hidden history that no
        # longer corresponds to anything.
        jump = detect_pose_jump(self._prev_pose, pose)
        if jump is not None:
            self._runner.reset()
            self.get_logger().warn(
                f"policy_node: pose discontinuity ({jump}) — recurrent state "
                f"reset. Expected on a SLAM loop closure; investigate if it "
                f"repeats while stationary."
            )
        self._prev_pose = pose

        self._robot_x, self._robot_y, self._robot_yaw = pose.x, pose.y, pose.yaw

        # In map mode the goal is fixed on the first usable MAP pose, not on
        # the first /odom message — that is the entire point. A goal fixed in
        # the odom frame migrates with the odometry drift; one fixed in the
        # map frame does not.
        if self._goal_world_x is None and self._pose_source == _POSE_SOURCE_MAP:
            dx, dy = _rotate_2d(
                self._goal_offset_x, self._goal_offset_y, pose.yaw
            )
            self._goal_world_x = pose.x + dx
            self._goal_world_y = pose.y + dy
            self.get_logger().info(
                f"policy_node: goal fixed in the {_MAP_FRAME} frame at "
                f"({self._goal_world_x:.3f}, {self._goal_world_y:.3f}) from "
                f"pose ({pose.x:.3f}, {pose.y:.3f}, {pose.yaw:.3f})."
            )

        # ------------------------------------------------------------------
        # Goal must be defined. No default is ever substituted (interface.yaml
        # goal_local.failure_mode) — undefined goal is a hard stop, not a
        # degraded mode.
        # ------------------------------------------------------------------
        if self._goal_world_x is None or self._goal_world_y is None:
            self.get_logger().warn(
                "policy_node: goal undefined — no /odom message has ever "
                "arrived, so the start pose (and therefore the goal) is "
                "unknown. Publishing zero.",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            self._publish_zero(reason="goal_undefined")
            return

        # ------------------------------------------------------------------
        # Build the observation. See the observation_builder API note in the
        # module docstring.
        # ------------------------------------------------------------------
        obs_result = build_observation(
            ranges=self._scan_ranges,
            angle_min=self._scan_angle_min,
            angle_increment=self._scan_angle_increment,
            pose_xy=(self._robot_x, self._robot_y),
            yaw=self._robot_yaw,
            goal_xy=(self._goal_world_x, self._goal_world_y),
            previous_action=(self._prev_action_v, self._prev_action_w),
        )

        if obs_result.fault is not None or obs_result.obs is None:
            self.get_logger().warn(
                f"policy_node: observation_builder fault: "
                f"{obs_result.fault!r} — publishing zero.",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            self._publish_zero(reason=f"obs_fault:{obs_result.fault}")
            return

        # ------------------------------------------------------------------
        # Run inference. PolicyRunner.act() already validates shape/NaN/inf
        # on both the input vector and the raw network output.
        # ------------------------------------------------------------------
        inference = self._runner.act(obs_result.obs)
        if inference.fault is not None or inference.action is None:
            self.get_logger().warn(
                f"policy_node: policy_runner fault: {inference.fault!r} — "
                f"publishing zero.",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            self._publish_zero(reason=f"inference_fault:{inference.fault}")
            return

        v, w = inference.action

        # Belt-and-braces final guard: policy_runner already checks
        # finiteness, but a command must never reach a publisher unchecked.
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().error(
                f"policy_node: non-finite action after inference (v={v}, "
                f"w={w}) — publishing zero."
            )
            self._publish_zero(reason="action_nonfinite")
            return

        # ------------------------------------------------------------------
        # Action is PRE-SCALED PHYSICAL UNITS (m/s, rad/s). Do NOT multiply
        # by any max_vx/wz_max here — config/interface.yaml
        # action.scaling.applies is false, and that scaling is rm_safety's
        # clamp, not this node's. Publish exactly what the actor emitted.
        # ------------------------------------------------------------------
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.linear.y = 0.0  # EXPLICIT — see module docstring point 5 /
        # interface.yaml action.vy_removed_note. The action space has no
        # lateral component; leaving this field at its implicit zero default
        # would not be a visible claim in the code or the recorded bag.
        cmd.angular.z = float(w)
        self._cmd_pub.publish(cmd)

        # Feed back the REQUESTED action, not any achieved/clamped value —
        # rm_safety may clamp or zero this downstream; that clamping is
        # invisible to this node by design (interface.yaml
        # previous_action.clamped_action_note / module docstring point 6).
        self._prev_action_v = float(v)
        self._prev_action_w = float(w)

        # Recover the robot-frame goal by INVERTING the observation's own
        # normalization (`xy / 5.0 - 1.0`, interface.yaml goal_local) rather
        # than recomputing local_goal() here. Two reasons: it cannot drift out
        # of step with what the network consumed, and if it ever disagreed
        # with the pose, the vector is the truth — that is what was fed in.
        self._publish_debug(
            fault=None,
            action=(v, w),
            obs=obs_result.obs,
            goal_local=(
                (float(obs_result.obs[_GOAL_SLICE_START]) + 1.0) * _GOAL_NORM_SCALE_M,
                (float(obs_result.obs[_GOAL_SLICE_START + 1]) + 1.0) * _GOAL_NORM_SCALE_M,
            ),
        )

    @staticmethod
    def _age_ms(stamp: Optional[float], now: float) -> float:
        """Age in milliseconds since `stamp` (monotonic seconds); +inf if
        the topic has never been received."""
        if stamp is None:
            return math.inf
        return (now - stamp) * 1000.0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    """Initialise rclpy, spin PolicyNode, publish a final zero on shutdown."""
    rclpy.init(args=args)
    node: Optional[PolicyNode] = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"policy_node: fatal error: {exc}")
        else:
            print(f"policy_node: fatal error during init: {exc}")
        raise
    finally:
        if node is not None:
            try:
                node._publish_zero(reason="shutdown")
            except Exception:
                pass
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
