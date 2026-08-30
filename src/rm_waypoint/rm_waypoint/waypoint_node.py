"""
waypoint_node.py — drives the robot to an operator-chosen point. The ROS shell
around `waypoint_controller`: it owns the clock, the pose, the services and
the one publisher.

What it does:
    Accepts a waypoint over `/waypoint/goto`, then drives toward it at
    `rate_hz` until it arrives, times out, stops making progress, loses its
    pose, or is cancelled. It exists for one operator job — repositioning the
    robot between policy runs, "drive back to the start so I can go again" —
    and it is not part of the NCP pipeline. Nothing in that pipeline reads it.

    WHERE IT SITS, and this is the whole safety argument:

        /waypoint/goto -> [this node] -> /teleop/cmd_vel -> rm_safety -> /cmd_vel

    It publishes `/teleop/cmd_vel`, which is an INPUT to rm_safety, exactly
    like the gamepad. It therefore drives behind the same clamps, the same
    slow and stop arcs, the same acceleration limiter and the same kill as
    every other source. It has no path to `/cmd_vel` and cannot become a
    second publisher of it. The topic is a MODULE CONSTANT, not a parameter,
    so that remains true no matter how the node is launched — see
    `_CMD_TOPIC`.

    THREE THINGS MUST ALL BE TRUE before this node can move the robot, and no
    one of them is this node's to grant:

        1. an operator explicitly sends a waypoint (this node idles otherwise,
           publishing nothing at all)
        2. rm_safety's `command_source` is `teleop`
        3. rm_safety has been armed with /safety/reset

    So the button that starts a waypoint run is, by construction, incapable of
    starting motion on its own.

Subscribes:  /odom            [nav_msgs/Odometry]        (pose_source:=odom)
Subscribes:  /tf, /tf_static  via tf2                    (pose_source:=map)
Subscribes:  /safety/status   [rm_interfaces/SafetyStatus]
Publishes:   /teleop/cmd_vel  [geometry_msgs/Twist]  @ rate_hz, WHILE ACTIVE ONLY
Publishes:   /waypoint/status [rm_interfaces/WaypointStatus] @ rate_hz, always
Services:    /waypoint/goto   [rm_interfaces/SetWaypoint]
Services:    /waypoint/cancel [std_srvs/Trigger]
Parameters:  config_path      (str)   config/waypoint.yaml — THE authority
Parameters:  pose_source      (str, "odom") — "odom" or "map"; "map" needs SLAM
Parameters:  dry_run          (bool, False) — compute and report, publish no Twist
Parameters:  rate_hz          (float, 0.0) — 0 means "take it from the config"

Failure mode:
    Fail-closed on every path, and silent when idle.

    IDLE MEANS SILENT. With no waypoint active this node publishes no Twist at
    all — not zeros. That matters because `/teleop/cmd_vel` is shared with the
    gamepad: a node helpfully publishing zeros forever would fight the
    operator's stick for the whole session. On every transition INTO a
    terminal state it floods zeros (`_ZERO_FLOOD`) and then goes quiet, so the
    robot is definitely stopped before the topic is released.

    Anything that goes wrong stops the robot and latches idle with a reason:
    a missing or stale pose, a non-finite pose, a timeout, a stalled range, a
    kill, an unhandled exception in the control tick. The controller never
    retries on its own — an operator who wants another attempt presses the
    button again. Unattended retry into a failure nobody has looked at is how
    a robot grinds itself into a wall.

    A kill is watched for rather than assumed: if rm_safety reports KILLED
    while a run is active, the run is abandoned immediately. rm_safety would
    zero the command anyway, so this changes nothing about what the wheels do;
    what it changes is that the operator is told the run is over instead of
    watching a controller command a robot that is not listening.
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional, Tuple

import rclpy
import rclpy.node
import tf2_ros
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from rm_interfaces.msg import SafetyStatus, WaypointStatus
from rm_interfaces.srv import Jog, SetWaypoint
from rm_waypoint import waypoint_controller as wc

# THE ONLY TOPIC THIS NODE MAY PUBLISH A TWIST ON. A module constant, not a
# parameter, and asserted below. Making it configurable would put a launch-file
# typo between this node and rm_safety's sole ownership of /cmd_vel — the one
# invariant this entire workspace is built around. There is no operator need
# it would serve.
_CMD_TOPIC = "/teleop/cmd_vel"

# Hard ceiling on a jog's time-to-live, seconds. The caller supplies the TTL,
# but the caller is a browser and this is the deadman — so the node, not the
# browser, decides the longest the robot may keep moving on one command. Half
# a second is comfortably longer than a sane resend interval (~150 ms) and
# short enough that a dropped connection stops the robot almost immediately.
_JOG_MAX_TTL_S = 0.5
_STATUS_TOPIC = "/waypoint/status"

# The pose frames. base_footprint (not base_link) because that is what /odom's
# child_frame_id is and what the EKF publishes — see config/slam_toolbox.yaml.
_MAP_FRAME = "map"
_ODOM_FRAME = "odom"
_BASE_FRAME = "base_footprint"

_POSE_SOURCES = (_ODOM_FRAME, _MAP_FRAME)

# Zero Twists published on the way into a terminal state, before the node
# releases the topic and goes quiet. More than one because /teleop/cmd_vel is
# BEST_EFFORT-adjacent in practice and a single dropped stop message would
# leave rm_safety holding the last non-zero command until its own 300 ms
# command timeout expired. Cheap insurance in the one place it matters.
_ZERO_FLOOD = 5

# Default control rate if the config somehow lacks one. The config is the
# authority; this exists so a malformed file cannot produce a zero-rate timer.
_FALLBACK_RATE_HZ = 10.0

_STATE_NAMES = {
    WaypointStatus.IDLE: "IDLE",
    WaypointStatus.TURNING: "TURNING",
    WaypointStatus.DRIVING: "DRIVING",
    WaypointStatus.ARRIVED: "ARRIVED",
    WaypointStatus.ABORTED: "ABORTED",
}


class WaypointNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("waypoint_node")

        # The constant above is the safety argument; this asserts it rather
        # than trusting that nobody edits it to something convenient.
        assert _CMD_TOPIC != "/cmd_vel", (
            "waypoint_node must never publish /cmd_vel — rm_safety is its sole "
            "publisher. See the module docstring."
        )

        self.declare_parameter("config_path", "")
        self.declare_parameter("pose_source", _ODOM_FRAME)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("rate_hz", 0.0)

        config_path = self._str_param("config_path")
        self._pose_source = self._str_param("pose_source")
        if self._pose_source not in _POSE_SOURCES:
            raise ValueError(
                f"waypoint_node: pose_source must be one of {_POSE_SOURCES}, "
                f"got {self._pose_source!r}"
            )
        self._dry_run = bool(
            self.get_parameter("dry_run").get_parameter_value().bool_value
        )

        self._limits, self._pose_timeout_s, rate_hz = self._load_config(config_path)

        override = float(
            self.get_parameter("rate_hz").get_parameter_value().double_value
        )
        if override > 0.0:
            rate_hz = override

        # --- run state -----------------------------------------------------
        self._target: Optional[Tuple[float, float]] = None
        # Manual jog: (vx, wz) and the monotonic deadline after which it stops
        # being valid. None means no jog is in force. The deadline IS the
        # deadman — see _on_jog and the Jog.srv comments.
        self._jog: Optional[Tuple[float, float]] = None
        self._jog_deadline: float = 0.0
        self._started_at: float = 0.0
        self._progress: Optional[wc.ProgressWatchdog] = None
        self._state: int = WaypointStatus.IDLE
        self._reason: str = "no waypoint"
        self._last_cmd: Tuple[float, float] = (0.0, 0.0)
        self._last_range: float = 0.0
        self._last_heading_err: float = 0.0

        # --- pose ----------------------------------------------------------
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        # Two clocks, each used consistently with itself. ROS stamps are
        # compared against the ROS clock for FRESHNESS, monotonic against
        # monotonic for INTERVALS. Never mapped onto one another via a captured
        # offset: policy_node did that once and every pose read as stamped in
        # the future forever, because bernard-1 has no NTP and its clock is
        # pushed by hand mid-session (CLAUDE.md, 2026-08-29 s5).
        self._odom_stamp_ros: Optional[float] = None

        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._tf_listener = None
        if self._pose_source == _MAP_FRAME:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._safety_state: Optional[int] = None

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(
            SafetyStatus, "/safety/status", self._safety_cb, 10
        )

        self._cmd_pub = self.create_publisher(Twist, _CMD_TOPIC, 10)
        self._status_pub = self.create_publisher(WaypointStatus, _STATUS_TOPIC, 10)

        self.create_service(SetWaypoint, "/waypoint/goto", self._on_goto)
        self.create_service(Trigger, "/waypoint/cancel", self._on_cancel)
        self.create_service(Jog, "/waypoint/jog", self._on_jog)

        # Structural parameters only. Everything above is read once at
        # construction, so accepting a runtime set would leave a
        # half-reconfigured node — the accepted-and-silently-ignored failure
        # this project already hit with rm_safety's command_source. Reject
        # instead, loudly.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"waypoint_node ready: pose_source={self._pose_source}, "
            f"{rate_hz:.1f} Hz, publishing {_CMD_TOPIC} "
            f"(max_vx {self._limits.max_vx} m/s, max_wz {self._limits.max_wz} rad/s, "
            f"range ceiling {self._limits.max_range_m} m)"
            + (" — DRY RUN, no Twist will be published" if self._dry_run else "")
        )

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    def _load_config(self, path: str):
        """Load config/waypoint.yaml, or refuse to start.

        No fallback to defaults. A waypoint controller running on limits
        nobody chose is precisely the thing the operator cannot see and cannot
        check, and the numbers in that file are measured properties of this
        robot rather than taste.
        """
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(
                f"waypoint_node: config_path {path!r} does not exist. "
                f"Pass config/waypoint.yaml — there is no built-in default."
            )
        with open(path, "r") as handle:
            raw = yaml.safe_load(handle) or {}
        block = raw.get("waypoint")
        if not isinstance(block, dict):
            raise ValueError(f"waypoint_node: {path} has no 'waypoint:' block")

        limits, error = wc.limits_from_dict(block)
        if error:
            raise ValueError(f"waypoint_node: {error}")

        pose_timeout_s = float(block.get("pose_timeout_ms", 500)) / 1000.0
        rate_hz = float(block.get("rate_hz", _FALLBACK_RATE_HZ))
        if rate_hz <= 0.0:
            raise ValueError("waypoint_node: rate_hz must be positive")
        return limits, pose_timeout_s, rate_hz

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_set_parameters(self, params) -> SetParametersResult:
        names = ", ".join(p.name for p in params)
        return SetParametersResult(
            successful=False,
            reason=(
                f"waypoint_node parameters are structural and read once at "
                f"startup ({names}); restart the node instead. Waypoints are "
                f"sent through /waypoint/goto, not through parameters."
            ),
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom_x = float(p.x)
        self._odom_y = float(p.y)
        self._odom_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._odom_stamp_ros = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    def _safety_cb(self, msg: SafetyStatus) -> None:
        self._safety_state = int(msg.state)

    # ------------------------------------------------------------------
    # Pose
    # ------------------------------------------------------------------

    def _pose(self) -> Tuple[Optional[Tuple[float, float, float]], Optional[str]]:
        """Return `((x, y, yaw), None)` or `(None, reason)`.

        NEVER falls back to the other source. In map mode a silent downgrade
        to /odom would put the robot back on the drifting pose that SLAM was
        added to correct, without saying so — which is the exact failure
        policy_node's pose path is written to avoid.
        """
        ros_now = self.get_clock().now().nanoseconds * 1e-9

        if self._pose_source == _ODOM_FRAME:
            if self._odom_stamp_ros is None:
                return None, "pose_missing: no /odom yet"
            age = ros_now - self._odom_stamp_ros
            if age < 0.0:
                return None, "pose_future_stamp"
            if age > self._pose_timeout_s:
                return None, f"pose_stale: {age * 1000.0:.0f}ms"
            return (self._odom_x, self._odom_y, self._odom_yaw), None

        if self._tf_buffer is None:  # unreachable; defensive
            return None, "pose_no_tf_buffer"
        try:
            # Time() means "latest available" rather than a specific stamp.
            # Asking for `now` invites extrapolation errors on a robot whose
            # clock is pushed by hand; take what exists, then judge its age.
            tf = self._tf_buffer.lookup_transform(
                _MAP_FRAME, _BASE_FRAME, rclpy.time.Time()
            )
        except Exception as exc:
            # The whole tf2 exception family means one thing here: no
            # trustworthy pose, so do not drive.
            return None, f"pose_tf_unavailable: {type(exc).__name__}"

        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        age = ros_now - stamp
        if age < 0.0:
            return None, "pose_future_stamp"
        if age > self._pose_timeout_s:
            return None, f"pose_stale: {age * 1000.0:.0f}ms"

        t = tf.transform.translation
        q = tf.transform.rotation
        return (
            float(t.x), float(t.y),
            _yaw_from_quaternion(q.x, q.y, q.z, q.w),
        ), None

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _on_jog(self, request, response):
        """Accept a manual jog: drive directly for at most `ttl_s` more.

        A jog CANCELS any active waypoint. Manual input wins over an
        autonomous drive without argument — the operator reaching for the keys
        while the robot is driving itself is, in every case that matters, a
        correction.

        Accepted even while rm_safety is KILLED. That is not an oversight: the
        layering is that this node asks and rm_safety decides, and duplicating
        the kill check here would put a second, staler opinion in front of the
        real one. The published Twist simply goes nowhere while killed, and
        the status reports why so the operator is not left guessing.
        """
        vx = float(request.vx)
        wz = float(request.wz)
        ttl = float(request.ttl_s)

        if not all(math.isfinite(v) for v in (vx, wz, ttl)):
            response.accepted = False
            response.message = "vx, wz and ttl_s must be finite"
            return response
        if ttl <= 0.0:
            response.accepted = False
            response.message = "ttl_s must be positive — a jog must expire"
            return response

        ttl = min(ttl, _JOG_MAX_TTL_S)
        # Clamp to the SAME limits the waypoint controller uses, so the manual
        # path cannot ask for more than the operator configured for the
        # autonomous one. Reverse is permitted here and only here.
        vx = max(-self._limits.max_vx, min(self._limits.max_vx, vx))
        wz = max(-self._limits.max_wz, min(self._limits.max_wz, wz))

        if self._target is not None:
            self.get_logger().info("waypoint_node: jog received — cancelling waypoint")
            self._target = None

        self._jog = (vx, wz)
        self._jog_deadline = time.monotonic() + ttl
        response.accepted = True
        response.message = f"jog ({vx:+.3f}, {wz:+.3f}) for {ttl:.2f}s"
        return response

    def _on_goto(self, request, response):
        """Accept or refuse a waypoint, with a reason either way.

        Every refusal path returns `accepted=False` and says why. That is the
        whole reason this is a service: an operator who clicks a point and
        sees nothing happen has no way to tell a refused waypoint from a
        broken node, and would reasonably click again.
        """
        frame = (request.frame_id or "").strip()
        if frame != self._pose_source:
            response.accepted = False
            response.message = (
                f"waypoint is in frame {frame!r} but this node is driving in "
                f"{self._pose_source!r}. Refused rather than converted — the "
                f"two frames differ by the SLAM correction, so converting "
                f"would silently move the destination."
            )
            return response

        if self._safety_state == SafetyStatus.KILLED:
            # Not fatal to accept, but pointless and misleading: the run would
            # start, command nothing that reaches the wheels, and abort on the
            # progress watchdog several seconds later with a reason that
            # points at obstacles. Say the real thing now.
            response.accepted = False
            response.message = (
                "rm_safety is KILLED — arm it with /safety/reset first, and "
                "set command_source to 'teleop' so this node's commands are "
                "the ones it forwards."
            )
            return response

        pose, fault = self._pose()
        if pose is None:
            response.accepted = False
            response.message = (
                f"no usable pose ({fault}) — cannot tell how far away that is, "
                f"so it cannot be driven to."
            )
            return response

        refusal = wc.validate_target(
            pose[0], pose[1], float(request.x), float(request.y), self._limits
        )
        if refusal:
            response.accepted = False
            response.message = refusal
            return response

        self._target = (float(request.x), float(request.y))
        self._started_at = time.monotonic()
        self._progress = wc.ProgressWatchdog(
            self._limits.progress_window_s, self._limits.progress_min_m
        )
        self._state = WaypointStatus.TURNING
        self._reason = ""
        rng = math.hypot(self._target[0] - pose[0], self._target[1] - pose[1])
        self.get_logger().info(
            f"waypoint accepted: ({self._target[0]:.2f}, {self._target[1]:.2f}) "
            f"in {frame}, {rng:.2f} m away from "
            f"({pose[0]:.2f}, {pose[1]:.2f})"
        )
        response.accepted = True
        response.message = f"driving {rng:.2f} m to ({request.x:.2f}, {request.y:.2f})"
        return response

    def _on_cancel(self, request, response):
        del request
        was_active = self._target is not None or self._jog is not None
        self._jog = None
        self._finish(WaypointStatus.IDLE, "cancelled by operator")
        response.success = True
        response.message = (
            "waypoint cancelled; robot stopped" if was_active
            else "no waypoint was active"
        )
        return response

    # ------------------------------------------------------------------
    # Control tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """One control step. Never raises: an exception here stops the robot.

        The blanket handler is deliberate and is not hiding anything — the
        traceback is logged. What it prevents is a timer callback dying and
        leaving the node alive but not ticking, which on a publisher of
        velocity commands means the last command standing until rm_safety's
        own timeout notices. Stopping first and explaining second is the right
        order for this node.
        """
        try:
            self._tick_body()
        except Exception as exc:
            self.get_logger().error(
                f"waypoint_node: control tick failed: {exc!r} — stopping"
            )
            self._finish(WaypointStatus.ABORTED, f"internal error: {exc!r}")

    def _tick_body(self) -> None:
        # JOG FIRST. It is checked before the waypoint and before idle,
        # because a jog is a live operator holding a key and nothing should
        # come between that and the wheels except rm_safety.
        if self._jog is not None:
            if time.monotonic() >= self._jog_deadline:
                # EXPIRED: go silent rather than publishing zeros. The gamepad
                # shares this topic, and a stream of zeros from here would
                # fight it. Silence lets rm_safety's command_timeout stop the
                # robot, which is the layer that is supposed to.
                self._jog = None
                self._state = WaypointStatus.IDLE
                self._reason = "jog expired"
                self._last_cmd = (0.0, 0.0)
                self._publish_status()
                return
            self._state = WaypointStatus.DRIVING
            self._reason = (
                "jog (killed — rm_safety is not forwarding)"
                if self._safety_state == SafetyStatus.KILLED else "jog"
            )
            self._last_cmd = self._jog
            self._publish_cmd(self._jog[0], self._jog[1])
            self._publish_status()
            return

        if self._target is None:
            # Idle: report, publish NO Twist. See the failure-mode note — the
            # gamepad shares this topic and must not be fought.
            self._publish_status()
            return

        if self._safety_state == SafetyStatus.KILLED:
            self._finish(WaypointStatus.ABORTED, "rm_safety went KILLED")
            return

        pose, fault = self._pose()
        if pose is None:
            self._finish(WaypointStatus.ABORTED, fault or "pose unavailable")
            return

        command = wc.compute(
            robot_x=pose[0], robot_y=pose[1], robot_yaw=pose[2],
            target_x=self._target[0], target_y=self._target[1],
            elapsed_s=time.monotonic() - self._started_at,
            limits=self._limits,
            progress=self._progress,
        )

        self._last_range = command.range_m
        self._last_heading_err = command.heading_error_rad

        if command.is_terminal:
            self._finish(command.state, command.reason,
                         command.range_m, command.heading_error_rad)
            return

        self._state = command.state
        self._reason = command.reason
        self._last_cmd = (command.vx, command.wz)
        self._publish_cmd(command.vx, command.wz)
        self._publish_status()

    def _publish_cmd(self, vx: float, wz: float) -> None:
        """Publish one Twist, unless this is a dry run.

        `linear.y` is set to 0.0 explicitly and never computed. The base is
        mecanum and the driver accepts strafe, but the whole stack constrains
        the controller to [vx, wz] — and a stray lateral component from a
        repositioning tool is a bug, not a command.
        """
        if self._dry_run:
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _finish(self, state: int, reason: str,
                range_m: float = 0.0, heading_err: float = 0.0) -> None:
        """Stop the robot, latch a terminal state, release the topic.

        The zero flood is suppressed in a dry run, by `_publish_cmd` and
        correctly so: a dry run published no motion and so has none to undo.
        The state latch and the log happen either way, so a run that has ended
        is visibly over in both modes.
        """
        for _ in range(_ZERO_FLOOD):
            self._publish_cmd(0.0, 0.0)

        was_active = self._target is not None
        self._target = None
        self._progress = None
        self._state = state
        self._reason = reason
        self._last_cmd = (0.0, 0.0)
        self._last_range = range_m
        self._last_heading_err = heading_err
        self._publish_status()

        if was_active:
            log = (
                self.get_logger().info if state == WaypointStatus.ARRIVED
                else self.get_logger().warn
            )
            log(f"waypoint run ended {_STATE_NAMES.get(state, state)}: {reason}")

    def _publish_status(self) -> None:
        msg = WaypointStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._pose_source
        msg.state = int(self._state)
        msg.reason = self._reason
        msg.frame_id = self._pose_source
        msg.target_x = float(self._target[0]) if self._target else 0.0
        msg.target_y = float(self._target[1]) if self._target else 0.0
        msg.range_m = float(self._last_range)
        msg.heading_error_rad = float(self._last_heading_err)
        msg.elapsed_s = (
            float(time.monotonic() - self._started_at) if self._target else 0.0
        )
        msg.commanded_vx = float(self._last_cmd[0])
        msg.commanded_wz = float(self._last_cmd[1])
        msg.dry_run = bool(self._dry_run)
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        """Stop before letting go of the topic.

        A node being shut down mid-run is the one case where the robot is
        moving and nothing else is about to correct it: rm_safety's command
        timeout would catch it 300 ms later, but 300 ms of uncommanded motion
        on a robot with no physical E-stop is not something to lean on when a
        flood of zeros costs nothing.
        """
        try:
            for _ in range(_ZERO_FLOOD):
                self._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        return super().destroy_node()


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
