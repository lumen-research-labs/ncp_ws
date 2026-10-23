"""
dashboard_node.py — ROS2 node behind the web dashboard. Subscribe-only on
topics; the sole outbound effects are two safety services and parameter
sets on safety_node and the policy nodes.

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
              -> /policy/<p>/cmd_vel -> rm_safety -> /cmd_vel -> rm_base -> wheels

    with rate and staleness on every hop, and the safety state and per-wheel
    telemetry alongside. Several policy nodes may run at once (one per row
    of config/topics.yaml `command_sources`, see ncp.launch.py profiles:=);
    the page follows ONE, the ACTIVE policy (rm_dashboard/sources.py), and
    offers one source button per row. The observation comes from that
    policy's debug stream rather than being rebuilt here — see `_debug_cb`.

Subscribes:  /scan            [sensor_msgs/LaserScan]
Subscribes:  /ncp/scan        [sensor_msgs/LaserScan]   (bridge output, first profile's beams)
Subscribes:  /odom            [nav_msgs/Odometry]
Subscribes:  <every command_sources row's topic>  [geometry_msgs/Twist]  (requests)
Subscribes:  <every policy row's debug stream>    [std_msgs/String]      (JSON, incl. the obs)
Subscribes:  /cmd_vel         [geometry_msgs/Twist]     (rm_safety output)
Subscribes:  /safety/status   [rm_interfaces/SafetyStatus]
Subscribes:  /base/debug      [rm_interfaces/BaseDebug]
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
Parameters:  topics_path (str, <in-repo config/topics.yaml>) — the command-source
                                         table; fatal if unreadable (a page that
                                         cannot name rm_safety's sources would
                                         offer buttons that do nothing)

Failure mode:
    Diagnostic-grade, and structurally unable to move the robot. It publishes
    no Twist on any topic, so it cannot be a second `/cmd_vel` publisher —
    the violation this workspace is built around preventing. Its only writes
    are `/safety/kill`, `/safety/reset` and parameter sets on `safety_node` /
    the policy nodes, each of which is individually incapable of starting
    motion: kill only stops; a `command_source` change force-kills (see
    safety_node); a goal change only re-targets policies that must still be
    armed separately; a roam switch only changes how those policies choose
    their next goal. Arming therefore always remains an explicit, separate
    act.

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
import os
import signal
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
from rm_safety.command_sources import DEFAULT_TOPICS_PATH, load_command_sources
from rm_dashboard.frames import world_to_body_offset
from rm_dashboard.slam_restart import find_slam_pids, reset_refusal
from rm_dashboard.sources import active_policy, is_live
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
    "/cmd_vel": 20.0,
    "/safety/status": 10.0,
    "/base/debug": 20.0,
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

# Every command-source row (and every policy row's debug stream) is added to
# the expected-rate table at this rate: policy nodes tick at control.rate_hz
# (5 Hz, config/interface.yaml) and the teleop publishers near it.
_SOURCE_HZ = 5.0

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

# RESET MAP: how long to wait for the respawned slam_toolbox to publish a
# map -> odom transform stamped after the kill, and how far past the kill
# instant a stamp must be to count as the NEW process's. The old process
# future-dates its last transform by slam_toolbox's transform_timeout
# (0.2 s), so a margin smaller than that would accept the dead map.
_MAP_RESET_TIMEOUT_SEC = 15.0
_MAP_RESET_FRESH_MARGIN_S = 0.5
# Goal offset used when the page sends none (config/interface.yaml
# goal_default_m, the same default policy_node declares).
_DEFAULT_GOAL_X_M = 2.5
_DEFAULT_GOAL_Y_M = 0.0

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
# neither is performed here. "waypoint_cancel" only ever stops. "reset_map"
# begins by killing the robot and ends with it still killed. "roam" is a
# parameter set on the same policy nodes a goal goes to; it cannot arm and
# cannot select a source. It changes what an ARMED, SELECTED policy does
# next (drive to goals nobody clicked), which is why the page confirms
# before turning it on and never before turning it off.
_CONTROLS = (
    "kill", "reset", "source", "goal", "goal_map", "roam", "waypoint",
    "waypoint_cancel", "reset_odom", "jog", "reset_map",
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
        self.declare_parameter("topics_path", DEFAULT_TOPICS_PATH)

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

        # The command-source table: one button per row, the policy rows'
        # debug streams and nodes. Loaded through rm_safety's own loader so
        # the names here are exactly the names safety_node accepts. Fatal if
        # it cannot be read: a page that cannot name the sources would offer
        # buttons that do nothing, the failure this page exists to show.
        topics_path = self.get_parameter("topics_path").get_parameter_value().string_value
        self._sources = load_command_sources(topics_path)
        # Policy rows in table order; the active-policy choice walks this.
        self._policies: List[str] = [
            name for name, source in self._sources.items() if source.is_policy
        ]
        expected = dict(_EXPECTED_HZ)
        for source in self._sources.values():
            expected[source.topic] = _SOURCE_HZ
            if source.debug:
                expected[source.debug] = _SOURCE_HZ
        self._store = StateStore(expected_hz=expected)
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
        # One request subscription per table row, and one debug subscription
        # per policy row. The page shows every request side by side and
        # follows the active policy's debug stream.
        for source in self._sources.values():
            self.create_subscription(
                Twist, source.topic, self._twist_cb(source.topic), 10
            )
            if source.debug:
                self.create_subscription(
                    String, source.debug, self._debug_cb(source.debug), 10
                )
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb("/cmd_vel"), 10)
        self.create_subscription(
            SafetyStatus, "/safety/status", self._safety_cb, 10
        )
        self.create_subscription(BaseDebug, "/base/debug", self._base_cb, 10)
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
        # ROS stamp (s) of the latest map -> odom transform. RESET MAP waits
        # for this to move past the kill instant, which is how it tells the
        # respawned slam_toolbox's first transform from the dead one's last
        # (tf2 keeps serving the old one from its buffer for ~10 s).
        self._slam_tf_stamp: Optional[float] = None
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
        #
        # One client per policy row that has a node, keyed by row name. A
        # goal is sent to EVERY one whose service is up (see _retarget), so
        # "the goal" stays one thing and the policy you switch to is already
        # aimed at it.
        self._policy_params = {
            name: self.create_client(
                SetParametersAtomically, f"/{source.node}/set_parameters_atomically"
            )
            for name, source in self._sources.items() if source.node
        }
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
        # The policy row most recently READ BACK as the active source. The
        # page keeps following it through a switch to teleop (sources.py).
        self._last_policy: Optional[str] = None
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

    def _debug_cb(self, topic: str):
        def cb(msg: String) -> None:
            def body():
                # Each policy node (and the upstream node) publishes JSON on
                # its own debug stream. This is the ONLY source of the
                # observation vector: rebuilding it in this node would be a
                # second implementation of the frozen contract, and a wrong
                # one would render as a plausible-looking vector that is not
                # what the network consumed. The whole value of showing the
                # observation is that it is the same numbers the policy saw.
                self._store.update(topic, json.loads(msg.data))
            self._guard(topic, body)
        return cb

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
                self._slam_tf_stamp = (
                    tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                )
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
                    if value in self._policies:
                        self._last_policy = value
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
    # The command-source table, as the page and the commands see it
    # ------------------------------------------------------------------

    def _source_is_policy(self, name: Optional[str]) -> bool:
        """True when `name` is a policy row of the table. Unknown is False."""
        source = self._sources.get(name) if name else None
        return bool(source and source.is_policy)

    def _live_sources(self, topics: Dict[str, Any]) -> set:
        """Rows whose command topic is being published right now."""
        return {
            name for name, source in self._sources.items()
            if is_live(topics.get(source.topic))
        }

    def _active_policy(self, topics: Dict[str, Any]) -> Optional[str]:
        """The policy row the page follows (rm_dashboard/sources.py)."""
        return active_policy(
            self._command_source, self._last_policy, self._policies,
            self._live_sources(topics),
        )

    def _policy_debug(self, name: Optional[str], topics=None) -> Dict[str, Any]:
        """The latest debug JSON of one policy row, or an empty dict."""
        if name is None:
            return {}
        source = self._sources[name]
        if source.debug is None:
            return {}
        if topics is None:
            topics = self._store.snapshot()
        return _slot_value(topics, source.debug)

    def _active_debug(self) -> Dict[str, Any]:
        topics = self._store.snapshot()
        return self._policy_debug(self._active_policy(topics), topics)

    def _lidar_zones_off(self) -> Optional[str]:
        """Why roam may not be switched on, or None when both lidar zones are
        on in rm_safety. Fail-closed: a zone that cannot be read is off."""
        names = ["stop_zone_enabled", "slow_zone_enabled"]
        if not self._safety_get_params.service_is_ready():
            return "roam refused: safety_node's parameter service is unavailable, so the lidar zones cannot be confirmed on"
        request = GetParameters.Request()
        request.names = names
        result = self._await(self._safety_get_params.call_async(request))
        if result is None:
            return "roam refused: safety_node did not answer the lidar-zone read in time"
        off = []
        for name, value in zip(names, result.values):
            if value.type != ParameterType.PARAMETER_BOOL:
                return f"roam refused: safety_node reports no boolean {name}"
            if not value.bool_value:
                off.append(name)
        if off:
            return (
                f"roam refused: {', '.join(off)} is off in rm_safety. Roam needs both "
                f"lidar zones on; relaunch with stop_zone_enabled:=true "
                f"slow_zone_enabled:=true (the launch defaults them off)."
            )
        return None

    def _retarget_targets(self) -> List[str]:
        """Policy rows with a node whose parameter service is up right now.

        A goal goes to every RUNNING policy, not to every row: a profile that
        was not launched has no service, and reporting it as a failure on
        every click would bury the real result.
        """
        return [
            name for name in self._policies
            if name in self._policy_params
            and self._policy_params[name].service_is_ready()
        ]

    def _set_on_policies(self, requests: Dict[str, Any], did: str) -> Dict[str, Any]:
        """Apply one parameter set per named policy node and aggregate.

        `requests` maps a row name to the list of Parameters to set on it, or
        to a dict carrying a refusal (`{"ok": False, "error": ...}`) decided
        by the caller, which is reported alongside the sets without being
        sent. The result is ok when at least one node took the set; every
        per-policy outcome is in `policies`, so a half-applied set is never
        reported as a clean one. `did` names the act in the messages
        ("goal set", "roam on").
        """
        results: Dict[str, Any] = {}
        done: List[str] = []
        for name, request in requests.items():
            if isinstance(request, dict):
                results[name] = request
                continue
            results[name] = self._set_params(
                self._policy_params[name], self._sources[name].node, request,
            )
            if results[name].get("ok"):
                done.append(name)
        failed = [n for n in results if n not in done]
        out: Dict[str, Any] = {
            "ok": bool(done), "policies": results, "applied": done,
        }
        if not results:
            out["error"] = (
                "no policy node is running (no parameter service is up); "
                "is a profile launched?"
            )
        elif not done:
            out["error"] = f"{did} on no policy: " + "; ".join(
                f"{n}: {results[n].get('error')}" for n in failed
            )
        else:
            out["message"] = f"{did} on {', '.join(done)}" + (
                f" (not on {', '.join(failed)}: "
                + "; ".join(str(results[n].get("error")) for n in failed) + ")"
                if failed else ""
            )
        return out

    def _retarget(self, offsets: Dict[str, Any]) -> Dict[str, Any]:
        """Set goal_x/goal_y on each named policy node (see _set_on_policies).

        `offsets` maps a row name to its robot-frame (x, y) offset, or to a
        caller-decided refusal dict.
        """
        return self._set_on_policies({
            name: offset if isinstance(offset, dict) else [
                _double_param("goal_x", offset[0]), _double_param("goal_y", offset[1]),
            ]
            for name, offset in offsets.items()
        }, "goal set")

    # ------------------------------------------------------------------
    # Snapshot assembly (called on the broadcast thread)
    # ------------------------------------------------------------------

    def _snapshot(self) -> Dict[str, Any]:
        # Taken ONCE and threaded through: the store snapshot is the
        # expensive part of a broadcast frame, and taking it twice would also
        # risk the two halves of one payload describing different instants.
        topics = self._store.snapshot()
        live = self._live_sources(topics)
        active = self._active_policy(topics)
        active_source = self._sources[active] if active else None
        return {
            "now": time.time(),
            "uptime_s": round(time.monotonic() - self._started_at, 1),
            "topics": topics,
            "controls_enabled": self._enable_controls,
            # rm_safety's ACTIVE source, read back from the node itself.
            # None means "not yet known", which the page renders as unknown
            # rather than guessing at a default.
            "command_source": self._command_source,
            # The table, in order, with what the page needs to draw a button
            # per row: liveness (a policy button greys until its node
            # publishes) and whether its goal can be set.
            "sources": [
                {
                    "name": name,
                    "kind": source.kind,
                    "topic": source.topic,
                    "node": source.node,
                    "debug": source.debug,
                    "description": source.description,
                    "live": name in live,
                    "params_ready": (
                        name in self._policy_params
                        and self._policy_params[name].service_is_ready()
                    ),
                }
                for name, source in self._sources.items()
            ],
            # The policy the header, observation panel and chain follow.
            "active_policy": (
                {
                    "name": active, "topic": active_source.topic,
                    "debug": active_source.debug, "node": active_source.node,
                }
                if active_source else None
            ),
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
                "policy_pose_source": self._policy_debug(active, topics)
                .get("pose_source"),
            },
            # Service readiness, so a disabled-looking button has a reason.
            "services": {
                "kill": self._kill_client.service_is_ready(),
                "reset": self._reset_client.service_is_ready(),
                "safety_params": self._safety_params.service_is_ready(),
                # Any running policy node: the goal buttons need one.
                "policy_params": any(
                    c.service_is_ready() for c in self._policy_params.values()
                ),
                "waypoint": self._waypoint_goto.service_is_ready(),
                "waypoint_cancel": self._waypoint_cancel.service_is_ready(),
                # Exactly one slam_toolbox process: the precondition RESET
                # MAP checks again at the moment it runs.
                "reset_map": len(find_slam_pids()) == 1,
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
            if value not in self._sources:
                return {
                    "ok": False,
                    "error": f"value must be one of {sorted(self._sources)} "
                             f"(config/topics.yaml command_sources)",
                }
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
            # The same offset to every running policy: it is an offset from
            # the robot, and the robot is where it is for all of them.
            result = self._retarget(
                {name: (gx, gy) for name in self._retarget_targets()}
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
        if command == "roam":
            on = payload.get("on")
            # A JSON boolean, strictly. "false", 0 and "" are refused rather
            # than coerced: a page bug sending the wrong type must never read
            # as an operator turning roam on or off.
            if not isinstance(on, bool):
                return {"ok": False, "error": f"roam needs boolean 'on', got {on!r}"}
            if on:
                # Refused unless rm_safety has BOTH lidar zones on. Roam draws
                # goals the lidar shows as free, but the policy's own obstacle
                # avoidance is what it is, and the launch defaults the zones
                # OFF: on 2026-10-23 the first live run drove into a wall
                # with nothing between a bad goal and the wall but KILL.
                # Read from safety_node itself, never from what this page
                # last saw. Off never checks: it only removes autonomy.
                refusal = self._lidar_zones_off()
                if refusal:
                    return {"ok": False, "error": refusal}
            # Every running policy, like a goal: the policy you switch to
            # should roam the same way as the one that is driving.
            return self._set_on_policies(
                {name: [_bool_param("roam", on)] for name in self._retarget_targets()},
                f"roam {'on' if on else 'off'}",
            )
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
        if command == "reset_map":
            return self._reset_map(payload)
        return {"ok": False, "error": f"unhandled command {command!r}"}

    def _set_goal_from_world(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Set EVERY running policy's goal from a world point the operator clicked.

        policy_node takes `goal_x`/`goal_y` as a ROBOT-FRAME OFFSET and re-fixes
        the goal against its own current pose. A map click is a WORLD point, so
        something has to do the conversion. It is done HERE, per policy,
        against the pose that policy node itself published on its debug
        stream, so each offset is expressed in exactly the frame its node will
        rotate it back out of. Converting in the browser instead would use
        whatever pose that page last rendered, which is a different sample.

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

        topics = self._store.snapshot()
        offsets: Dict[str, Any] = {}
        poses_used: Dict[str, Any] = {}
        for name in self._retarget_targets():
            debug = self._policy_debug(name, topics)
            pose = debug.get("pose")
            pose_source = debug.get("pose_source")
            node = self._sources[name].node
            # A pose from a stalled stream is a pose from the past, and the
            # offset would land the goal relative to where the robot WAS.
            if not is_live(topics.get(self._sources[name].debug)):
                offsets[name] = {
                    "ok": False,
                    "error": (
                        f"refused: {self._sources[name].debug} is stale, so "
                        f"{node}'s last pose cannot be trusted for the "
                        f"conversion. Use the robot-frame offset box, or wait."
                    ),
                }
                continue
            if not isinstance(pose, (list, tuple)) or len(pose) != 3:
                offsets[name] = {
                    "ok": False,
                    "error": (
                        f"no pose on {self._sources[name].debug} yet — {node} "
                        "has not fixed a goal. It publishes a pose only once "
                        "it has one; use the robot-frame offset box, or wait "
                        "for the first fix."
                    ),
                }
                continue
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
                offsets[name] = {"ok": False, "error": f"pose from {node} is not finite"}
                continue

            expected_frame = _MAP_FRAME if pose_source == "map" else _ODOM_FRAME
            frame = payload.get("frame", expected_frame)
            if frame != expected_frame:
                offsets[name] = {
                    "ok": False,
                    "error": (
                        f"refused: point is in '{frame}' but {node} is driving "
                        f"in '{expected_frame}' (pose_source {pose_source!r}). "
                        "Those frames differ by the SLAM correction, which "
                        "grows as the robot drives — converting would move "
                        "the goal silently."
                    ),
                }
                continue

            rx, ry, ryaw = float(pose[0]), float(pose[1]), float(pose[2])
            # World -> body: the inverse of the rotation policy_node applies
            # when it turns the offset back into a world point. In its own
            # pure module so the round-trip can be pinned by a workstation
            # test.
            offsets[name] = world_to_body_offset(wx, wy, rx, ry, ryaw)
            poses_used[name] = [round(rx, 3), round(ry, 3), round(ryaw, 4)]

        result = self._retarget(offsets)
        if result.get("ok"):
            result["message"] = (
                f"policy goal ({wx:.2f}, {wy:.2f}) set on "
                f"{', '.join(result['applied'])}"
                + (
                    f" (not on {', '.join(n for n in offsets if n not in result['applied'])})"
                    if len(result["applied"]) < len(offsets) else ""
                )
            )
            result["goal_world"] = [round(wx, 3), round(wy, 3)]
            result["pose_used"] = poses_used
        return result

    def _reset_odom(self) -> Dict[str, Any]:
        """Zero the EKF pose. REFUSED in map mode — see the client comment."""
        debug = self._active_debug()
        if debug.get("pose_source") == "map":
            return {
                "ok": False,
                "error": (
                    "refused: pose_source is 'map'. Zeroing the EKF moves the "
                    "odom origin that slam_toolbox's map->odom correction was "
                    "computed against, and slam_toolbox only recomputes after "
                    "0.2 m of travel — so the map pose would be silently wrong "
                    "until the robot drove that far. Use RESET MAP for a "
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

    def _reset_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """A fresh SLAM map in place: stop, restart slam_toolbox, re-fix the goal.

        slam_toolbox has no reset service, so this terminates its process and
        relies on slam.launch.py's respawn to bring it back empty. The order
        is the same principle as stack_down.sh: the robot is stopped FIRST
        and is still stopped when this returns, whatever happens in between.
        The map -> odom correction starts over at zero, so the new map is
        anchored on the odometry frame and every old map-frame target
        becomes meaningless: the waypoint run is cancelled and every running
        policy's goal is re-fixed from the current pose, with the offsets the
        page sent (its goal x / y fields).

        Called on an HTTP worker thread, so the waits below are plain sleeps;
        the executor keeps polling TF on its own thread meanwhile.
        """
        debug = self._active_debug()
        pids = find_slam_pids()
        refusal = reset_refusal(debug.get("pose_source"), pids)
        if refusal is not None:
            return {"ok": False, "error": refusal}
        try:
            gx = float(payload.get("x", _DEFAULT_GOAL_X_M))
            gy = float(payload.get("y", _DEFAULT_GOAL_Y_M))
        except Exception:
            return {"ok": False, "error": "reset_map needs numeric goal x and y"}
        if not (math.isfinite(gx) and math.isfinite(gy)):
            return {"ok": False, "error": "goal must be finite"}

        # 1. Stop, and stay stopped. A reset that cannot stop the robot first
        #    does not happen.
        killed = self._call_trigger(self._kill_client, "/safety/kill")
        if not killed.get("ok"):
            return {
                "ok": False,
                "error": f"refused: could not kill the robot first ({killed.get('error')})",
            }
        # 2. A waypoint target lives in the frame that is about to vanish.
        if self._waypoint_cancel.service_is_ready():
            self._call_trigger(self._waypoint_cancel, "/waypoint/cancel")

        # 3. Restart slam_toolbox. The health history belongs to the old map.
        pid = pids[0]
        kill_ros_time = self.get_clock().now().nanoseconds * 1e-9
        started = time.monotonic()
        self._slam.reset()
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass  # exited between the scan and the signal; respawn covers it
        except PermissionError as exc:
            return {"ok": False, "error": f"cannot signal slam_toolbox pid {pid}: {exc}"}
        self.get_logger().warn(
            f"dashboard: RESET MAP — slam_toolbox pid {pid} terminated; "
            f"waiting for the respawn. Robot is KILLED."
        )

        # 4. Wait for a NEW process and a transform it stamped.
        new_pid: Optional[int] = None
        deadline = started + _MAP_RESET_TIMEOUT_SEC
        while time.monotonic() < deadline:
            current = find_slam_pids()
            if current and pid not in current:
                new_pid = current[0]
            stamp = self._slam_tf_stamp
            if (
                new_pid is not None
                and stamp is not None
                and stamp > kill_ros_time + _MAP_RESET_FRESH_MARGIN_S
            ):
                break
            time.sleep(_FUTURE_POLL_SEC * 5)
        else:
            return {
                "ok": False,
                "error": (
                    f"slam_toolbox did not publish a fresh map->odom within "
                    f"{_MAP_RESET_TIMEOUT_SEC:.0f} s (respawned pid: {new_pid}). "
                    f"The robot is KILLED. Check the launch log; restart the "
                    f"stack if SLAM stays down."
                ),
            }
        elapsed = time.monotonic() - started

        # 5. Re-fix every running policy's goal from the robot's pose in the
        #    NEW map.
        goal = self._retarget({name: (gx, gy) for name in self._retarget_targets()})
        goal_note = (
            f"goal re-fixed {gx:.2f} m ahead, {gy:.2f} m left of here on "
            f"{', '.join(goal['applied'])}"
            if goal.get("ok") else f"goal NOT re-fixed ({goal.get('error')})"
        )
        message = (
            f"map reset: slam_toolbox restarted (pid {pid} -> {new_pid}) in "
            f"{elapsed:.1f} s; {goal_note}; robot is KILLED — arm to continue"
        )
        self.get_logger().warn("dashboard: " + message)
        return {
            "ok": True,
            "message": message,
            "slam_restart_s": round(elapsed, 1),
            "goal": goal,
        }

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
        if self._source_is_policy(self._command_source):
            return {
                "ok": False,
                "error": (
                    f"refused: command_source is '{self._command_source}' (a "
                    f"policy row), so rm_safety is forwarding "
                    f"{self._sources[self._command_source].topic} and ignores "
                    f"the jog's /teleop/cmd_vel. Switch to 'teleop' first."
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
        # Without this, a waypoint sent while the source is a policy row is
        # ACCEPTED, the robot never moves because rm_safety is forwarding that
        # policy's topic instead, and the attempt eventually times out with a
        # stall/obstacle reason that blames the hardware for a mode error.
        if self._source_is_policy(self._command_source):
            return {
                "ok": False,
                "error": (
                    f"refused: command_source is '{self._command_source}' (a "
                    f"policy row), so rm_safety is forwarding "
                    f"{self._sources[self._command_source].topic} and would "
                    f"ignore the waypoint controller's /teleop/cmd_vel "
                    f"entirely. Switch the source to 'teleop' first (that "
                    f"force-kills, so re-arm after)."
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


def _bool_param(name: str, value: bool) -> Parameter:
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
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
