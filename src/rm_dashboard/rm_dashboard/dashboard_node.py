"""
dashboard_node.py — ROS2 node behind the web dashboard. Subscribe-only on
topics; the sole outbound effects are two safety services and two parameter
sets.

What it does:
    Subscribes to every stage of the NCP pipeline, keeps the latest message
    from each in a `StateStore` (which also derives rate/jitter/staleness), and
    runs a `DashboardServer` on a background thread that streams snapshots to a
    browser and forwards a fixed set of commands back.

    The point is to make the WHOLE CHAIN visible in one place, because the
    integration failures on this project have consistently been failures of
    the joins rather than of the parts. Each stage was individually correct
    when the scan bridge published BEST_EFFORT to a RELIABLE subscriber, when a
    second EKF was started on top of the first, when `command_source` accepted
    a value and ignored it, and when front-right was being dragged by the other
    three wheels. In each case the missing view was the same one: what does
    each stage think, side by side, right now.

    So the layout follows the signal:

        /scan -> /ncp/scan -> [observation] -> policy (v,w)
              -> /policy/cmd_vel -> rm_safety -> /cmd_vel -> rm_base -> wheels

    with rate and staleness on every hop, and the safety state and per-wheel
    telemetry alongside. The observation comes from `/ncp_debug` (published by
    policy_node) rather than being rebuilt here — see `_ncp_debug_cb`.

Subscribes:  /scan            [sensor_msgs/LaserScan]
Subscribes:  /ncp/scan        [sensor_msgs/LaserScan]   (bridge output, the policy's beams)
Subscribes:  /odom            [nav_msgs/Odometry]
Subscribes:  /policy/cmd_vel  [geometry_msgs/Twist]     (policy request)
Subscribes:  /teleop/cmd_vel  [geometry_msgs/Twist]     (gamepad request)
Subscribes:  /cmd_vel         [geometry_msgs/Twist]     (rm_safety output)
Subscribes:  /safety/status   [rm_interfaces/SafetyStatus]
Subscribes:  /base/debug      [rm_interfaces/BaseDebug]
Subscribes:  /ncp_debug       [std_msgs/String]         (JSON, incl. the obs)
Subscribes:  /voltage         [std_msgs/Float32]
Subscribes:  /scan_sanitized  [sensor_msgs/LaserScan]   (what SLAM actually reads)
Subscribes:  /map             [nav_msgs/OccupancyGrid]  (latched; SLAM's map)
Subscribes:  /waypoint/status [rm_interfaces/WaypointStatus]
Subscribes:  /tf, /tf_static  via tf2 — map -> odom (the SLAM correction) and
                              map -> base_footprint (the corrected pose)
Publishes:   NOTHING. Not one topic, ever. See the note above `_CONTROLS`.
Parameters:  host        (str,   "0.0.0.0")
Parameters:  port        (int,   8080)
Parameters:  stream_hz   (float, 10.0)
Parameters:  scan_stride (int,   6)    — decimation for the displayed /scan
Parameters:  enable_controls (bool, True) — false serves a read-only page
Parameters:  map_max_cells (int, 400) — longest side of the map image served
Parameters:  tf_poll_hz  (float, 5.0)  — how often the SLAM transforms are read

Failure mode:
    Diagnostic-grade, and structurally unable to move the robot. It publishes
    no Twist on any topic, so it cannot be a second `/cmd_vel` publisher —
    the violation this workspace is built around preventing. Its only writes
    are `/safety/kill`, `/safety/reset` and parameter sets on `safety_node` /
    `policy_node`, each of which is individually incapable of starting motion:
    kill only stops; a `command_source` change force-kills (see safety_node);
    a goal change only re-targets a policy that must still be armed
    separately. Arming therefore always remains an explicit, separate act.

    THE WAYPOINT CONTROL IS THE ONE ADDITION THAT CAN MOVE THE ROBOT, and it
    changes what this page is. `/waypoint/goto` is a service call into
    rm_waypoint, which owns the publisher; that node publishes
    `/teleop/cmd_vel`, which rm_safety forwards only when `command_source` is
    `teleop` AND the kill has been cleared.

    AN EARLIER VERSION OF THIS DOCSTRING CLAIMED those were "two prior operator
    acts that this page cannot perform on the operator's behalf". THAT WAS
    FALSE, and it is corrected here rather than quietly deleted because a false
    claim in safety documentation is worse than no claim. This page exposes
    BOTH of them — `source` and `reset` are in `_CONTROLS` — so a browser can
    set the source, clear the kill, and send a waypoint in three POSTs, with no
    physical operator involvement at any point.

    That is a deliberate operator decision (full control was chosen over
    observe-and-kill), not an oversight. But be clear about what it means: the
    server binds 0.0.0.0 with NO AUTHENTICATION, so on this network the
    boundary protecting the robot is the network itself, not anything in this
    node. On a lab network with no WAN uplink that is a reasonable trade. It
    would not be on any other network, and `enable_controls:=false` is the
    switch that takes it away.

    What remains structurally true: the dashboard is still a leaf with NO
    publisher of any kind, so it can never become a second `/cmd_vel` source.

    Every callback is wrapped: a malformed message degrades that one panel and
    never takes the node down. The HTTP server runs on daemon threads and is
    never awaited by the ROS thread, so a wedged browser cannot stall the
    subscriptions. If the port is already bound the node logs the error and
    keeps running headless rather than dying — the subscriptions are harmless
    and the operator may well be about to close the other tab.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional

import rclpy
import rclpy.node
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import (
    GetParameters, SetParameters, SetParametersAtomically,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_localization.srv import SetPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from rm_interfaces.msg import BaseDebug, SafetyStatus, WaypointStatus
from rm_interfaces.srv import Jog, SetWaypoint
from rm_dashboard.frames import world_to_body_offset
from rm_dashboard.slam_health import (
    OCCUPIED_THRESHOLD, SlamHealth, pack_grid, summarize_map,
)
from rm_dashboard.state_store import StateStore
from rm_dashboard.web_server import DashboardServer

# Expected publish rates, Hz. Sourced from the MEASURED figures recorded in
# config/interface.yaml (control.measured_source_rates_hz and the per-block
# measured_rate_hz entries) and from the rates measured on hardware
# 2026-08-29. They drive the staleness colouring and the expected-vs-actual
# readout only; nothing here is a control constant.
_EXPECTED_HZ: Dict[str, float] = {
    "/scan": 10.1,
    "/ncp/scan": 10.1,
    "/odom": 30.0,
    "/policy/cmd_vel": 5.0,
    "/teleop/cmd_vel": 5.0,
    "/cmd_vel": 20.0,
    "/safety/status": 10.0,
    "/base/debug": 20.0,
    "/ncp_debug": 5.0,
    # 30 Hz, not the ~9 Hz recorded in interface.yaml: that figure was
    # measured when the VENDOR Mcnamu_driver_X3 published it. rm_base
    # replaced that driver and republishes /voltage on its own sensor
    # tick (measured 30.0 Hz on bernard-1, 2026-08-29).
    "/voltage": 30.0,
    # SLAM's actual input. Watched separately from /scan because the sanitizer
    # sits between them, and "SLAM sees nothing" vs "the lidar sees nothing"
    # are different faults with the same symptom.
    "/scan_sanitized": 10.1,
    # slam_toolbox's map_update_interval is 2 s in config/slam_toolbox.yaml.
    "/map": 0.5,
    # rm_waypoint's status tick, config/waypoint.yaml rate_hz.
    "/waypoint/status": 10.0,
}

# Frames. base_footprint (not base_link) matches /odom's child_frame_id and
# what the EKF publishes — see config/slam_toolbox.yaml.
_MAP_FRAME = "map"
_ODOM_FRAME = "odom"
_BASE_FRAME = "base_footprint"

# How stale the SLAM correction may be before the page calls it dead.
# slam_toolbox republishes map -> odom with every scan-match update; several
# seconds of silence means it has stopped working, which it does without
# erroring (the 2026-08-29 silent scan rejection).
_SLAM_TF_MAX_AGE_S = 3.0

# How often to re-read rm_safety's command_source. It changes only when an
# operator changes it, so this is about noticing a change made elsewhere (a
# terminal, another tab), not about tracking a fast signal.
_COMMAND_SOURCE_POLL_SEC = 1.0

# How long a service call may take before the browser gets an error. Generous
# for a local call, but a hung service must not hold an HTTP worker forever.
_SERVICE_TIMEOUT_SEC = 3.0

# Poll interval while waiting on a future from the HTTP thread. The future is
# resolved by the executor on the ROS thread, so this thread can only wait.
_FUTURE_POLL_SEC = 0.02

# Battery thresholds, volts, for the page's colour bands. From the operating
# history in CLAUDE.md: runs start ~11.9 V freshly charged, ~10.3 V was called
# "bottom third", and the 2026-08-08 session died at 9.5 V.
_BATTERY_GOOD_V = 11.0
_BATTERY_LOW_V = 10.5

# The complete set of commands a browser may issue. Anything not named here is
# rejected. Kept as an explicit table next to the ROS handles, deliberately
# NOT in the HTTP layer: the decision about what a web page can do to a robot
# with no physical E-stop should be readable in one place, beside the code
# that actually does it.
#
# "waypoint" is the only entry that can lead to motion, and it cannot lead to
# motion ALONE: it is a service call into rm_waypoint, whose commands go to
# /teleop/cmd_vel and reach the wheels only if rm_safety is already armed and
# already set to the teleop source. Both of those are separate acts, and
# neither is performed here. "waypoint_cancel" only ever stops.
_CONTROLS = (
    "kill", "reset", "source", "goal", "goal_map", "waypoint",
    "waypoint_cancel", "reset_odom", "jog",
)


class DashboardNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("stream_hz", 10.0)
        self.declare_parameter("scan_stride", 6)
        self.declare_parameter("enable_controls", True)
        self.declare_parameter("map_max_cells", 400)
        self.declare_parameter("tf_poll_hz", 5.0)

        host = self.get_parameter("host").get_parameter_value().string_value
        port = int(self.get_parameter("port").get_parameter_value().integer_value)
        stream_hz = float(
            self.get_parameter("stream_hz").get_parameter_value().double_value
        )
        self._scan_stride = max(
            1, int(self.get_parameter("scan_stride").get_parameter_value().integer_value)
        )
        self._enable_controls = bool(
            self.get_parameter("enable_controls").get_parameter_value().bool_value
        )

        self._map_max_cells = max(
            32, int(self.get_parameter("map_max_cells").get_parameter_value().integer_value)
        )
        tf_poll_hz = float(
            self.get_parameter("tf_poll_hz").get_parameter_value().double_value
        ) or 5.0

        self._store = StateStore(expected_hz=_EXPECTED_HZ)
        self._slam = SlamHealth()
        self._started_at = time.monotonic()

        # The rendered map, kept OUT of the SSE snapshot. A 1000x1000 grid is
        # a megabyte; putting it on a 10 Hz stream would cost ~10 MB/s on a
        # Jetson to redraw a picture that changes every two seconds. The
        # browser fetches /api/map on its own slow schedule instead, and the
        # snapshot carries only the summary (coverage, resolution, extent),
        # which is the part worth watching continuously.
        self._map_payload: Optional[Dict[str, Any]] = None
        self._map_seq = 0

        # Sensor data is BEST_EFFORT, matching every other node in this stack.
        # /ncp/scan is the documented exception: the bridge publishes RELIABLE
        # (so upstream's depth-only, therefore RELIABLE, subscription works),
        # and a BEST_EFFORT subscriber accepts a RELIABLE publisher, so this
        # profile reads both. Do NOT "tidy" the two sides to match — see
        # CLAUDE.md, 2026-08-27 session 2.
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(
            LaserScan, "/ncp/scan", self._ncp_scan_cb, sensor_qos
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(
            Twist, "/policy/cmd_vel", self._twist_cb("/policy/cmd_vel"), 10
        )
        self.create_subscription(
            Twist, "/teleop/cmd_vel", self._twist_cb("/teleop/cmd_vel"), 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb("/cmd_vel"), 10)
        self.create_subscription(
            SafetyStatus, "/safety/status", self._safety_cb, 10
        )
        self.create_subscription(BaseDebug, "/base/debug", self._base_cb, 10)
        self.create_subscription(String, "/ncp_debug", self._ncp_debug_cb, 10)
        self.create_subscription(Float32, "/voltage", self._voltage_cb, sensor_qos)
        self.create_subscription(
            LaserScan, "/scan_sanitized", self._sanitized_cb, sensor_qos
        )
        self.create_subscription(
            WaypointStatus, "/waypoint/status", self._waypoint_cb, 10
        )

        # /map is LATCHED (TRANSIENT_LOCAL) and RELIABLE. Subscribing with the
        # sensor profile above would silently receive nothing: a BEST_EFFORT
        # VOLATILE subscriber is incompatible with a RELIABLE TRANSIENT_LOCAL
        # publisher, and rclpy reports that once and then never again — the
        # same class of failure as the scan bridge's QoS mismatch, which cost
        # a session (CLAUDE.md, 2026-08-27 s2). Transient local also means a
        # dashboard started AFTER slam_toolbox still gets the current map
        # immediately, instead of showing nothing until the next update.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)

        # TF, for the two SLAM transforms. Always created, even without SLAM
        # running: "map -> odom does not exist" is itself the answer to the
        # question this panel is asked, and a listener that only appears when
        # SLAM is healthy could never report that SLAM is not.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._map_pose: Optional[Dict[str, Any]] = None
        self._tf_fault: Optional[str] = None
        self.create_timer(1.0 / tf_poll_hz, self._poll_tf)

        # Service and parameter clients. Created unconditionally so the page
        # can report whether each is AVAILABLE — "the kill button is not
        # wired up" is exactly the thing you want to learn before you need it,
        # not at the moment you press it.
        self._kill_client = self.create_client(Trigger, "/safety/kill")
        self._reset_client = self.create_client(Trigger, "/safety/reset")
        self._safety_params = self.create_client(
            SetParameters, "/safety_node/set_parameters"
        )
        # ATOMIC for the goal. The plain SetParameters service applies each
        # parameter separately, so setting goal_x and goal_y through it fires
        # policy_node's on-set callback TWICE: once with the new x against the
        # OLD y, then again with both. Observed on hardware 2026-08-29 — two
        # "re-targeted" log lines per click, and two recurrent-state resets
        # for one operator action. The final goal was correct, but a goal is a
        # PAIR and applying half of one is not a state worth passing through.
        self._policy_params = self.create_client(
            SetParametersAtomically, "/policy_node/set_parameters_atomically"
        )
        # rm_waypoint. Absent unless that node was launched, which is the
        # normal case — the page reports the service as unavailable and greys
        # the button, rather than offering a control that silently does
        # nothing.
        # robot_localization's EKF pose reset. Zeroing the EKF is only sound
        # while the pose is being READ from /odom. In map mode slam_toolbox's
        # map->odom correction was computed against the CURRENT odom origin, so
        # moving that origin invalidates it — and slam_toolbox only recomputes
        # after minimum_travel_distance (0.2 m) of travel, which never happens
        # while the robot sits still. The map pose would be silently WRONG
        # until the robot drove that far. _reset_odom refuses in map mode.
        self._set_pose = self.create_client(SetPose, "/set_pose")

        self._waypoint_goto = self.create_client(SetWaypoint, "/waypoint/goto")
        self._waypoint_cancel = self.create_client(Trigger, "/waypoint/cancel")
        self._waypoint_jog = self.create_client(Jog, "/waypoint/jog")

        # READ BACK the active command source, rather than remembering what
        # this page last asked for. Those are different things: any node can
        # set it, a set can be rejected, and rm_safety force-kills on a change
        # — so the page must show what rm_safety IS doing, not what a browser
        # once requested. It is also the difference between "the waypoint
        # button will move the robot" and "it will be accepted and ignored",
        # which is exactly what an operator needs to know before pressing it.
        self._safety_get_params = self.create_client(
            GetParameters, "/safety_node/get_parameters"
        )
        self._command_source: Optional[str] = None
        self._command_source_pending = None
        self.create_timer(_COMMAND_SOURCE_POLL_SEC, self._poll_command_source)

        self._server = DashboardServer(
            snapshot_fn=self._snapshot,
            dispatch_fn=self._dispatch,
            map_fn=self._map_image,
            host=host,
            port=port,
            stream_hz=stream_hz,
            logger=lambda msg: self.get_logger().info(msg),
        )
        try:
            self._server.start()
        except OSError as exc:
            # Almost always "address already in use" — a second dashboard, or
            # a previous one whose socket has not been released. Not fatal:
            # the subscriptions are harmless and the operator is likely one
            # tab-close away from being able to retry.
            self.get_logger().error(
                f"dashboard_node: could not bind {host}:{port} ({exc}). "
                f"Running headless — no web UI. Is another dashboard up?"
            )

        self.get_logger().info(
            f"dashboard_node ready: http://<robot>:{port}/  "
            f"controls={'ENABLED' if self._enable_controls else 'read-only'}"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks. Each one only reduces a message to something
    # JSON-shaped and hands it to the store; no decisions are made here.
    # ------------------------------------------------------------------

    def _guard(self, label: str, fn) -> None:
        """Run a callback body, downgrading any failure to a logged warning.

        A dashboard must not be able to take itself down over one malformed
        message, and it must not go silently blank either — the panel keeps
        its last value and the reason appears in the ROS log.
        """
        try:
            fn()
        except Exception as exc:
            self.get_logger().warn(
                f"dashboard_node: {label} callback failed: {exc!r}",
                throttle_duration_sec=5.0,
            )

    def _scan_cb(self, msg: LaserScan) -> None:
        def body():
            ranges = list(msg.ranges)
            # Decimate for TRANSPORT ONLY. The TG30 delivers ~2000 points at
            # 10 Hz; shipping all of them as JSON is ~200 KB/s for a plot no
            # denser than the pixels available. The statistics below are
            # computed on the FULL scan first, so nothing derived is affected
            # by the decimation — only what is drawn.
            valid = [r for r in ranges if math.isfinite(r) and r > 0.0]
            self._store.update("/scan", {
                "ranges": [
                    _finite_or_none(r) for r in ranges[:: self._scan_stride]
                ],
                "stride": self._scan_stride,
                "n_full": len(ranges),
                "angle_min": float(msg.angle_min),
                "angle_increment": float(msg.angle_increment) * self._scan_stride,
                "range_max": float(msg.range_max),
                "frame_id": msg.header.frame_id,
                "min_range_m": min(valid) if valid else None,
                # The TG30's no-return sentinel is 0.0, not inf (measured:
                # ~39% of rays). Counting them is how you tell a dropout-heavy
                # but healthy scan from a genuinely blocked sensor.
                "zero_count": sum(1 for r in ranges if r == 0.0),
                "valid_count": len(valid),
            })
        self._guard("/scan", body)

    def _ncp_scan_cb(self, msg: LaserScan) -> None:
        def body():
            ranges = [_finite_or_none(r) for r in msg.ranges]
            self._store.update("/ncp/scan", {
                "ranges": ranges,
                "n": len(msg.ranges),
                "angle_min": float(msg.angle_min),
                "angle_increment": float(msg.angle_increment),
                # After the resampler no bin may be 0.0 — that mapping is the
                # only defence against a no-return latching the e-stop. If
                # this is ever non-zero, the resampler is not doing its job.
                "zero_count": sum(1 for r in msg.ranges if r == 0.0),
            })
        self._guard("/ncp/scan", body)

    def _odom_cb(self, msg: Odometry) -> None:
        def body():
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            t = msg.twist.twist
            self._store.update("/odom", {
                "x": float(p.x),
                "y": float(p.y),
                "yaw": _yaw_from_quat(o.x, o.y, o.z, o.w),
                "vx": float(t.linear.x),
                "vy": float(t.linear.y),
                "wz": float(t.angular.z),
            })
        self._guard("/odom", body)

    def _twist_cb(self, topic: str):
        def cb(msg: Twist) -> None:
            def body():
                self._store.update(topic, {
                    "vx": float(msg.linear.x),
                    "vy": float(msg.linear.y),
                    "wz": float(msg.angular.z),
                })
            self._guard(topic, body)
        return cb

    def _safety_cb(self, msg: SafetyStatus) -> None:
        def body():
            self._store.update("/safety/status", {
                "state": int(msg.state),
                "state_name": _SAFETY_STATE_NAMES.get(int(msg.state), "UNKNOWN"),
                "reason": msg.reason,
                # min_range is +inf when there is no valid scan; JSON has no
                # infinity, so it is sent as None and the page renders it as
                # "no perception" rather than as a number.
                "min_range": _finite_or_none(msg.min_range),
                "clamping_active": bool(msg.clamping_active),
            })
        self._guard("/safety/status", body)

    def _base_cb(self, msg: BaseDebug) -> None:
        def body():
            self._store.update("/base/debug", {
                "control_mode": msg.control_mode,
                "dry_run": bool(msg.dry_run),
                "command_stale": bool(msg.command_stale),
                "commanded": {
                    "vx": float(msg.commanded_twist.linear.x),
                    "vy": float(msg.commanded_twist.linear.y),
                    "wz": float(msg.commanded_twist.angular.z),
                },
                # Indexed [FL, BL, FR, BR] — the MEASURED motor->wheel map,
                # not the Yahboom sample order. The page labels them.
                "wheels": [
                    {
                        "target_mps": float(msg.target_speed_mps[i]),
                        "measured_mps": float(msg.measured_speed_mps[i]),
                        "feedforward_pwm": float(msg.feedforward_pwm[i]),
                        "integral_pwm": float(msg.integral_pwm[i]),
                        "target_pwm": int(msg.target_pwm[i]),
                        "applied_pwm": int(msg.applied_pwm[i]),
                        "feedback_valid": bool(msg.feedback_valid[i]),
                        "saturated": bool(msg.saturated[i]),
                        "stalled": bool(msg.stalled[i]),
                        "sign_fault": bool(msg.sign_fault[i]),
                    }
                    for i in range(4)
                ],
            })
        self._guard("/base/debug", body)

    def _ncp_debug_cb(self, msg: String) -> None:
        def body():
            # policy_node (and the upstream node) publish JSON here. This is
            # the ONLY source of the observation vector: rebuilding it in this
            # node would be a second implementation of the frozen contract,
            # and a wrong one would render as a plausible-looking vector that
            # is not what the network consumed. The whole value of showing the
            # observation is that it is the same numbers the policy saw.
            self._store.update("/ncp_debug", json.loads(msg.data))
        self._guard("/ncp_debug", body)

    def _voltage_cb(self, msg: Float32) -> None:
        def body():
            volts = float(msg.data)
            self._store.update("/voltage", {
                "volts": volts,
                "level": (
                    "good" if volts >= _BATTERY_GOOD_V
                    else "low" if volts >= _BATTERY_LOW_V
                    else "critical"
                ),
            })
        self._guard("/voltage", body)

    def _sanitized_cb(self, msg: LaserScan) -> None:
        def body():
            ranges = msg.ranges
            # Only the counts, no ray data: this scan is never drawn. What it
            # is here to answer is whether the sanitizer is doing its job
            # before slam_toolbox reads it. The TG30 encodes no-return as 0.0
            # (~31-39% of rays), and karto has no idea that means "nothing
            # there" — untreated they are phantom hits at the sensor origin
            # and scan matching is worthless. So `zero_count` on THIS topic
            # must be 0. If it is not, SLAM is being fed the sentinel.
            self._store.update("/scan_sanitized", {
                "n": len(ranges),
                "zero_count": sum(1 for r in ranges if r == 0.0),
                "inf_count": sum(1 for r in ranges if math.isinf(r)),
                "finite_count": sum(
                    1 for r in ranges if math.isfinite(r) and r > 0.0
                ),
            })
        self._guard("/scan_sanitized", body)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        def body():
            info = msg.info
            data = msg.data
            known = sum(1 for c in data if c >= 0)
            occupied = sum(1 for c in data if c >= OCCUPIED_THRESHOLD)
            summary = summarize_map(
                info.width, info.height, info.resolution, known, occupied
            )
            summary["origin"] = [
                float(info.origin.position.x),
                float(info.origin.position.y),
            ]
            summary["frame_id"] = msg.header.frame_id
            self._map_seq += 1
            summary["seq"] = self._map_seq
            self._store.update("/map", summary)

            # Render once, here, rather than on every /api/map request: the
            # map changes every couple of seconds and a browser polling it
            # must not be able to make the ROS thread re-decimate a million
            # cells on demand.
            self._map_payload = pack_grid(
                data, info.width, info.height, self._map_max_cells
            )
            self._map_payload.update({
                "resolution_m": float(info.resolution),
                "origin": summary["origin"],
                "frame_id": msg.header.frame_id,
                "seq": self._map_seq,
            })
        self._guard("/map", body)

    def _waypoint_cb(self, msg: WaypointStatus) -> None:
        def body():
            self._store.update("/waypoint/status", {
                "state": int(msg.state),
                "state_name": _WAYPOINT_STATE_NAMES.get(int(msg.state), "UNKNOWN"),
                "reason": msg.reason,
                "frame_id": msg.frame_id,
                "target": [float(msg.target_x), float(msg.target_y)],
                "range_m": float(msg.range_m),
                "heading_error_rad": float(msg.heading_error_rad),
                "elapsed_s": float(msg.elapsed_s),
                "commanded": {
                    "vx": float(msg.commanded_vx),
                    "wz": float(msg.commanded_wz),
                },
                "dry_run": bool(msg.dry_run),
                "active": int(msg.state) in (
                    WaypointStatus.TURNING, WaypointStatus.DRIVING
                ),
            })
        self._guard("/waypoint/status", body)

    def _poll_tf(self) -> None:
        """Read the two SLAM transforms on a timer.

        Polled rather than driven by /tf callbacks because tf2 delivers many
        transforms at many rates and only two of them matter here. A timer
        also means the panel updates at a predictable rate whether SLAM is
        publishing or not — which is the point, since its NOT publishing is
        the state most worth showing.
        """
        def body():
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            monotonic = time.monotonic()

            # map -> odom IS the SLAM correction: how far SLAM has had to move
            # the odometry's idea of where the robot is. See slam_health.py.
            try:
                tf = self._tf_buffer.lookup_transform(
                    _MAP_FRAME, _ODOM_FRAME, rclpy.time.Time()
                )
            except Exception as exc:
                self._tf_fault = f"{_MAP_FRAME}->{_ODOM_FRAME}: {type(exc).__name__}"
            else:
                self._tf_fault = None
                t = tf.transform.translation
                q = tf.transform.rotation
                self._slam.update(
                    float(t.x), float(t.y),
                    _yaw_from_quat(q.x, q.y, q.z, q.w),
                    monotonic,
                )

            # map -> base_footprint is the CORRECTED POSE — the one
            # policy_node consumes with pose_source:=map, and the one a
            # waypoint in the map frame is measured against. Shown beside the
            # raw /odom pose so the correction is legible as a difference
            # rather than as a number in isolation.
            try:
                tf = self._tf_buffer.lookup_transform(
                    _MAP_FRAME, _BASE_FRAME, rclpy.time.Time()
                )
            except Exception:
                self._map_pose = None
            else:
                t = tf.transform.translation
                q = tf.transform.rotation
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                self._map_pose = {
                    "x": round(float(t.x), 4),
                    "y": round(float(t.y), 4),
                    "yaw": round(_yaw_from_quat(q.x, q.y, q.z, q.w), 4),
                    # Freshness compares a ROS stamp against the ROS clock —
                    # both wall time, so the age is real even on bernard-1,
                    # whose clock is pushed by hand and has no NTP. Never
                    # mapped onto the monotonic clock via a captured offset;
                    # that bug made every pose read as future-stamped forever
                    # (CLAUDE.md, 2026-08-29 s5).
                    "age_s": round(max(0.0, ros_now - stamp), 3),
                }
        self._guard("tf", body)

    def _poll_command_source(self) -> None:
        """Refresh `command_source` from rm_safety, asynchronously.

        Runs on the ROS thread, so it must NOT block on the future — the same
        executor that would resolve it is the one being blocked. The pending
        future is held across ticks and read when it completes, which also
        naturally coalesces: a slow reply delays the next request rather than
        stacking up.
        """
        def body():
            if self._command_source_pending is not None:
                if not self._command_source_pending.done():
                    return
                result = self._command_source_pending.result()
                self._command_source_pending = None
                if result and result.values:
                    value = result.values[0].string_value
                    # An empty string means the parameter exists but is not a
                    # string — or does not exist, which rcl reports as a
                    # NOT_SET value rather than an error. Either way it is not
                    # an answer, so keep reporting "unknown" instead of "".
                    self._command_source = value or None
                return
            if not self._safety_get_params.service_is_ready():
                self._command_source = None
                return
            request = GetParameters.Request()
            request.names = ["command_source"]
            self._command_source_pending = self._safety_get_params.call_async(request)
        self._guard("command_source", body)

    def _map_image(self) -> Dict[str, Any]:
        """The decimated occupancy grid, for GET /api/map.

        Called on an HTTP worker thread. Returns the payload rendered by
        `_map_cb`; it never touches the raw grid, so a browser polling this
        cannot make the ROS thread do work.
        """
        if self._map_payload is None:
            return {"available": False, "reason": "no /map yet — is slam_toolbox up?"}
        payload = dict(self._map_payload)
        payload["available"] = True
        return payload

    # ------------------------------------------------------------------
    # Snapshot assembly (called on the broadcast thread)
    # ------------------------------------------------------------------

    def _snapshot(self) -> Dict[str, Any]:
        # Taken ONCE and threaded through: the store snapshot is the
        # expensive part of a broadcast frame, and taking it twice would also
        # risk the two halves of one payload describing different instants.
        topics = self._store.snapshot()
        return {
            "now": time.time(),
            "uptime_s": round(time.monotonic() - self._started_at, 1),
            "topics": topics,
            "controls_enabled": self._enable_controls,
            # rm_safety's ACTIVE source, read back from the node itself.
            # None means "not yet known", which the page renders as unknown
            # rather than guessing at a default.
            "command_source": self._command_source,
            "slam": {
                # The correction itself, plus its jump history — see
                # slam_health.py for why those two are the whole story.
                "correction": self._slam.snapshot(
                    time.monotonic(), _SLAM_TF_MAX_AGE_S
                ),
                "tf_fault": self._tf_fault,
                "map_pose": self._map_pose,
                # Which pose the POLICY is actually using. Published by
                # policy_node on /ncp_debug. Worth showing next to the SLAM
                # panel because a healthy map with `pose_source: odom` means
                # SLAM is running and being ignored — a deliberate and safe
                # intermediate state, but one nobody should be in by accident.
                "policy_pose_source": _slot_value(topics, "/ncp_debug")
                .get("pose_source"),
            },
            # Service readiness, so a disabled-looking button has a reason.
            "services": {
                "kill": self._kill_client.service_is_ready(),
                "reset": self._reset_client.service_is_ready(),
                "safety_params": self._safety_params.service_is_ready(),
                "policy_params": self._policy_params.service_is_ready(),
                "waypoint": self._waypoint_goto.service_is_ready(),
                "waypoint_cancel": self._waypoint_cancel.service_is_ready(),
            },
        }


    # ------------------------------------------------------------------
    # Command dispatch (called on an HTTP worker thread)
    # ------------------------------------------------------------------

    def _dispatch(self, command: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self._enable_controls:
            return {"ok": False, "error": "controls disabled (enable_controls:=false)"}
        if command not in _CONTROLS:
            return {"ok": False, "error": f"unknown command {command!r}"}

        if command == "kill":
            return self._call_trigger(self._kill_client, "/safety/kill")
        if command == "reset":
            return self._call_trigger(self._reset_client, "/safety/reset")
        if command == "source":
            value = payload.get("value")
            if value not in ("policy", "teleop"):
                return {"ok": False, "error": "value must be 'policy' or 'teleop'"}
            return self._set_params(
                self._safety_params, "safety_node",
                [_string_param("command_source", value)],
            )
        if command == "goal":
            try:
                gx = float(payload["x"])
                gy = float(payload["y"])
            except Exception:
                return {"ok": False, "error": "goal needs numeric x and y"}
            if not (math.isfinite(gx) and math.isfinite(gy)):
                return {"ok": False, "error": "goal must be finite"}
            result = self._set_params(
                self._policy_params, "policy_node",
                [_double_param("goal_x", gx), _double_param("goal_y", gy)],
            )
            # Optional "clean slate": zero the EKF as part of re-targeting.
            # Ordered AFTER the goal is fixed, not before. The goal is derived
            # from the CURRENT pose, so redefining that pose first would place
            # the goal using one frame and read it back in another.
            if result.get("ok") and payload.get("reset_odom"):
                odom = self._reset_odom()
                result["odom_reset"] = odom.get("ok", False)
                if not odom.get("ok"):
                    result["odom_reset_error"] = odom.get("error")
            return result
        if command == "goal_map":
            return self._set_goal_from_world(payload)
        if command == "reset_odom":
            return self._reset_odom()
        if command == "waypoint":
            return self._send_waypoint(payload)
        if command == "jog":
            return self._send_jog(payload)
        if command == "waypoint_cancel":
            return self._call_trigger(self._waypoint_cancel, "/waypoint/cancel")
        return {"ok": False, "error": f"unhandled command {command!r}"}

    def _set_goal_from_world(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Set the POLICY goal from a world point the operator clicked.

        policy_node takes `goal_x`/`goal_y` as a ROBOT-FRAME OFFSET and re-fixes
        the goal against its own current pose. A map click is a WORLD point, so
        something has to do the conversion. It is done HERE, against the pose
        policy_node itself published on /ncp_debug, so the offset is expressed
        in exactly the frame policy_node will rotate it back out of. Converting
        in the browser instead would use whatever pose that page last rendered,
        which is a different sample.

        REFUSED rather than converted on a frame mismatch, exactly as
        rm_waypoint refuses one: the map and odom frames differ by the SLAM
        correction, which GROWS as the robot drives, so quietly reinterpreting
        a point from one as a point in the other moves the goal without saying
        so.

        There is a residual race — the robot may move between the /ncp_debug
        sample and the parameter set, and policy_node re-fixes from ITS pose at
        that instant, so the goal lands displaced by whatever the robot
        travelled in between. At 5 Hz and stationary this is millimetres. It is
        NOT silent: the response carries the pose used and the world goal that
        was intended, and the map draws the goal policy_node actually adopted,
        so the two can be compared on screen.
        """
        try:
            wx = float(payload["x"])
            wy = float(payload["y"])
        except Exception:
            return {"ok": False, "error": "goal_map needs numeric x and y"}
        if not (math.isfinite(wx) and math.isfinite(wy)):
            return {"ok": False, "error": "goal must be finite"}

        debug = self._store.snapshot().get("/ncp_debug", {}).get("value") or {}
        pose = debug.get("pose")
        pose_source = debug.get("pose_source")
        if not isinstance(pose, (list, tuple)) or len(pose) != 3:
            return {
                "ok": False,
                "error": (
                    "no pose on /ncp_debug yet — policy_node has not fixed a "
                    "goal. It publishes a pose only once it has one; use the "
                    "robot-frame offset box, or wait for the first fix."
                ),
            }
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
            return {"ok": False, "error": "pose on /ncp_debug is not finite"}

        expected_frame = _MAP_FRAME if pose_source == "map" else _ODOM_FRAME
        frame = payload.get("frame", expected_frame)
        if frame != expected_frame:
            return {
                "ok": False,
                "error": (
                    f"refused: point is in '{frame}' but policy_node is driving "
                    f"in '{expected_frame}' (pose_source {pose_source!r}). Those "
                    "frames differ by the SLAM correction, which grows as the "
                    "robot drives — converting would move the goal silently."
                ),
            }

        rx, ry, ryaw = float(pose[0]), float(pose[1]), float(pose[2])
        # World -> body: the inverse of the rotation policy_node applies when
        # it turns the offset back into a world point. In its own pure module
        # so the round-trip can be pinned by a workstation test.
        offset_x, offset_y = world_to_body_offset(wx, wy, rx, ry, ryaw)

        result = self._set_params(
            self._policy_params, "policy_node",
            [_double_param("goal_x", offset_x), _double_param("goal_y", offset_y)],
        )
        if result.get("ok"):
            result["message"] = (
                f"policy goal set to ({wx:.2f}, {wy:.2f}) in {frame} "
                f"— offset ({offset_x:.2f}, {offset_y:.2f}) from pose "
                f"({rx:.2f}, {ry:.2f}, {math.degrees(ryaw):.0f} deg)"
            )
            result["goal_world"] = [round(wx, 3), round(wy, 3)]
            result["pose_used"] = [round(rx, 3), round(ry, 3), round(ryaw, 4)]
        return result

    def _reset_odom(self) -> Dict[str, Any]:
        """Zero the EKF pose. REFUSED in map mode — see the client comment."""
        debug = self._store.snapshot().get("/ncp_debug", {}).get("value") or {}
        if debug.get("pose_source") == "map":
            return {
                "ok": False,
                "error": (
                    "refused: pose_source is 'map'. Zeroing the EKF moves the "
                    "odom origin that slam_toolbox's map->odom correction was "
                    "computed against, and slam_toolbox only recomputes after "
                    "0.2 m of travel — so the map pose would be silently wrong "
                    "until the robot drove that far. Restart the stack for a "
                    "fresh map, or run with pose_source:=odom."
                ),
            }
        if not self._set_pose.service_is_ready():
            return {"ok": False, "error": "/set_pose is not available"}

        request = SetPose.Request()
        request.pose.header.frame_id = "odom"
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.orientation.w = 1.0
        # Explicitly TIGHT covariance. The EKF treats this as a measurement,
        # so a loose covariance would let it weight the reset against its own
        # prior and drift straight back rather than accepting the new origin.
        for i in range(6):
            request.pose.pose.covariance[i * 6 + i] = 1e-9

        future = self._set_pose.call_async(request)
        if self._await(future) is None:
            return {"ok": False, "error": "/set_pose timed out"}
        self.get_logger().warn("dashboard: EKF pose reset to origin via /set_pose")
        return {"ok": True, "message": "odom reset to origin"}

    def _send_jog(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Forward one manual jog. The browser re-sends while a key is held.

        Deliberately NOT rate-limited or de-duplicated here. A jog is only
        valid for its TTL, so the re-send IS the control signal: throttling it
        would make the robot stutter, and caching it would defeat the deadman.
        """
        if not self._waypoint_jog.service_is_ready():
            return {
                "ok": False,
                "error": "/waypoint/jog unavailable — rm_waypoint is not "
                         "running (launch with with_waypoint:=true)",
            }
        try:
            vx = float(payload.get("vx", 0.0))
            wz = float(payload.get("wz", 0.0))
            ttl = float(payload.get("ttl_s", 0.4))
        except Exception:
            return {"ok": False, "error": "jog needs numeric vx, wz, ttl_s"}
        if not all(math.isfinite(v) for v in (vx, wz, ttl)):
            return {"ok": False, "error": "jog values must be finite"}

        # Same reasoning as the waypoint pre-flight: command_source is a MODE,
        # so a cached copy cannot be stale in the way a pose can, and catching
        # it here saves the operator pressing keys at a robot that structurally
        # cannot respond.
        if self._command_source == "policy":
            return {
                "ok": False,
                "error": (
                    "refused: command_source is 'policy', so rm_safety is "
                    "forwarding /policy/cmd_vel and ignores the jog's "
                    "/teleop/cmd_vel. Switch to 'teleop' first."
                ),
            }

        request = Jog.Request()
        request.vx = vx
        request.wz = wz
        request.ttl_s = ttl
        future = self._waypoint_jog.call_async(request)
        result = self._await(future, timeout=1.0)
        if result is None:
            return {"ok": False, "error": "/waypoint/jog timed out"}
        return {"ok": bool(result.accepted), "message": result.message,
                "error": None if result.accepted else result.message}

    def _send_waypoint(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Forward a waypoint to rm_waypoint, and relay its answer verbatim.

        Note what this method does NOT do: it does not check the range, the
        frame, or whether the robot is armed. rm_waypoint checks all of those
        against the pose IT is driving on, and a second opinion here could
        only ever be the stale one — the dashboard's view of the pose is a
        snapshot up to 100 ms old and taken through a different path. So the
        browser's numbers are passed through unexamined except for being
        finite, and the refusal that comes back is shown word for word.
        """
        if not self._waypoint_goto.service_is_ready():
            return {
                "ok": False,
                "error": "/waypoint/goto unavailable — rm_waypoint is not "
                         "running (launch with with_waypoint:=true)",
            }
        try:
            wx = float(payload["x"])
            wy = float(payload["y"])
        except Exception:
            return {"ok": False, "error": "waypoint needs numeric x and y"}
        if not (math.isfinite(wx) and math.isfinite(wy)):
            return {"ok": False, "error": "waypoint must be finite"}

        # ONE pre-flight check, and only this one. command_source is a MODE,
        # not a measurement: it changes only when someone deliberately changes
        # it, so the dashboard's cached copy cannot be meaningfully stale in
        # the way its pose snapshot can. Everything else is still left to
        # rm_waypoint per the note above.
        #
        # Without this, a waypoint sent while the source is "policy" is
        # ACCEPTED, the robot never moves because rm_safety is forwarding
        # /policy/cmd_vel instead, and the attempt eventually times out with a
        # stall/obstacle reason that blames the hardware for a mode error.
        if self._command_source == "policy":
            return {
                "ok": False,
                "error": (
                    "refused: command_source is 'policy', so rm_safety is "
                    "forwarding /policy/cmd_vel and would ignore the waypoint "
                    "controller's /teleop/cmd_vel entirely. Switch the source "
                    "to 'teleop' first (that force-kills, so re-arm after)."
                ),
            }

        request = SetWaypoint.Request()
        # The frame comes from the BROWSER, not from a default here. A
        # waypoint means a different place in each frame, and silently
        # supplying "map" for a page that was reading an odom-frame plot would
        # send the robot somewhere the operator did not click.
        request.frame_id = str(payload.get("frame") or "")
        request.x = wx
        request.y = wy

        future = self._waypoint_goto.call_async(request)
        result = self._await(future)
        if result is None:
            return {"ok": False, "error": "/waypoint/goto timed out"}
        return {
            "ok": bool(result.accepted),
            "message": result.message,
            "error": None if result.accepted else result.message,
        }

    def _call_trigger(self, client, name: str) -> Dict[str, Any]:
        if not client.service_is_ready():
            return {"ok": False, "error": f"{name} is not available"}
        future = client.call_async(Trigger.Request())
        result = self._await(future)
        if result is None:
            return {"ok": False, "error": f"{name} timed out"}
        return {"ok": bool(result.success), "message": result.message}

    def _set_params(self, client, node_name: str, params) -> Dict[str, Any]:
        if not client.service_is_ready():
            return {"ok": False, "error": f"{node_name} parameter service unavailable"}
        request = (
            SetParametersAtomically.Request()
            if client.srv_type is SetParametersAtomically
            else SetParameters.Request()
        )
        request.parameters = params
        future = client.call_async(request)
        result = self._await(future)
        if result is None:
            return {"ok": False, "error": f"{node_name} parameter set timed out"}
        # The node's own on-set callback may REJECT the value. Surfacing its
        # reason verbatim is the point: a rejected set that reported success
        # is precisely the class of bug this project already hit once with
        # command_source, and the browser must never paper over it.
        # SetParameters returns a LIST of results, SetParametersAtomically a
        # single one. Normalising here keeps the rejection-reason path — which
        # is the safety-relevant part — identical for both.
        results = (
            result.results if hasattr(result, "results") else [result.result]
        )
        failures = [r.reason for r in results if not r.successful]
        if failures:
            return {"ok": False, "error": "; ".join(failures) or "rejected"}
        return {"ok": True, "message": f"{node_name} updated"}

    def _await(self, future, timeout: float = _SERVICE_TIMEOUT_SEC):
        """Wait for a future from a NON-executor thread.

        The executor spins on the main thread and is what completes this
        future, so this thread can only poll. Deliberately NOT
        `rclpy.spin_until_future_complete`: calling that from here would spin
        a second executor over the same node, which is undefined behaviour and
        can consume callbacks out from under the main loop.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(_FUTURE_POLL_SEC)
        future.cancel()
        return None

    def destroy_node(self) -> bool:
        try:
            self._server.stop()
        except Exception:
            pass
        return super().destroy_node()


_SAFETY_STATE_NAMES = {0: "OK", 1: "SLOW", 2: "STOPPED", 3: "KILLED"}

_WAYPOINT_STATE_NAMES = {
    WaypointStatus.IDLE: "IDLE",
    WaypointStatus.TURNING: "TURNING",
    WaypointStatus.DRIVING: "DRIVING",
    WaypointStatus.ARRIVED: "ARRIVED",
    WaypointStatus.ABORTED: "ABORTED",
}


def _slot_value(topics: Dict[str, Any], name: str) -> Dict[str, Any]:
    """The value of one topic slot, or an empty dict.

    Never None, so callers can `.get()` straight off it — an absent topic and
    a topic with no useful field want the same handling downstream.
    """
    slot = topics.get(name) or {}
    return slot.get("value") or {}


def _finite_or_none(value) -> Optional[float]:
    """JSON has no inf/nan. None means "not a number", which the page renders
    as an explicit absence rather than as a misleading 0."""
    v = float(value)
    return v if math.isfinite(v) else None


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _string_param(name: str, value: str) -> Parameter:
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    return p


def _double_param(name: str, value: float) -> Parameter:
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    return p


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
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
