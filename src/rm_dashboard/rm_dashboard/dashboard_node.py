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
Subscribes:  /ncp/scan        [sensor_msgs/LaserScan]   (bridge output, 32 beams)
Subscribes:  /odom            [nav_msgs/Odometry]
Subscribes:  /policy/cmd_vel  [geometry_msgs/Twist]     (policy request)
Subscribes:  /teleop/cmd_vel  [geometry_msgs/Twist]     (gamepad request)
Subscribes:  /cmd_vel         [geometry_msgs/Twist]     (rm_safety output)
Subscribes:  /safety/status   [rm_interfaces/SafetyStatus]
Subscribes:  /base/debug      [rm_interfaces/BaseDebug]
Subscribes:  /ncp_debug       [std_msgs/String]         (JSON, incl. the obs)
Subscribes:  /voltage         [std_msgs/Float32]
Publishes:   NOTHING. Not one topic, ever. See the note above `_CONTROLS`.
Parameters:  host        (str,   "0.0.0.0")
Parameters:  port        (int,   8080)
Parameters:  stream_hz   (float, 10.0)
Parameters:  scan_stride (int,   6)    — decimation for the displayed /scan
Parameters:  enable_controls (bool, True) — false serves a read-only page

Failure mode:
    Diagnostic-grade, and structurally unable to move the robot. It publishes
    no Twist on any topic, so it cannot be a second `/cmd_vel` publisher —
    the violation this workspace is built around preventing. Its only writes
    are `/safety/kill`, `/safety/reset` and parameter sets on `safety_node` /
    `policy_node`, each of which is individually incapable of starting motion:
    kill only stops; a `command_source` change force-kills (see safety_node);
    a goal change only re-targets a policy that must still be armed
    separately. Arming therefore always remains an explicit, separate act.

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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from rm_interfaces.msg import BaseDebug, SafetyStatus
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
}

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
_CONTROLS = ("kill", "reset", "source", "goal")


class DashboardNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("stream_hz", 10.0)
        self.declare_parameter("scan_stride", 6)
        self.declare_parameter("enable_controls", True)

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

        self._store = StateStore(expected_hz=_EXPECTED_HZ)
        self._started_at = time.monotonic()

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

        # Service and parameter clients. Created unconditionally so the page
        # can report whether each is AVAILABLE — "the kill button is not
        # wired up" is exactly the thing you want to learn before you need it,
        # not at the moment you press it.
        self._kill_client = self.create_client(Trigger, "/safety/kill")
        self._reset_client = self.create_client(Trigger, "/safety/reset")
        self._safety_params = self.create_client(
            SetParameters, "/safety_node/set_parameters"
        )
        self._policy_params = self.create_client(
            SetParameters, "/policy_node/set_parameters"
        )

        self._server = DashboardServer(
            snapshot_fn=self._snapshot,
            dispatch_fn=self._dispatch,
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

    # ------------------------------------------------------------------
    # Snapshot assembly (called on the broadcast thread)
    # ------------------------------------------------------------------

    def _snapshot(self) -> Dict[str, Any]:
        return {
            "now": time.time(),
            "uptime_s": round(time.monotonic() - self._started_at, 1),
            "topics": self._store.snapshot(),
            "controls_enabled": self._enable_controls,
            # Service readiness, so a disabled-looking button has a reason.
            "services": {
                "kill": self._kill_client.service_is_ready(),
                "reset": self._reset_client.service_is_ready(),
                "safety_params": self._safety_params.service_is_ready(),
                "policy_params": self._policy_params.service_is_ready(),
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
            return self._set_params(
                self._policy_params, "policy_node",
                [_double_param("goal_x", gx), _double_param("goal_y", gy)],
            )
        return {"ok": False, "error": f"unhandled command {command!r}"}

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
        request = SetParameters.Request()
        request.parameters = params
        future = client.call_async(request)
        result = self._await(future)
        if result is None:
            return {"ok": False, "error": f"{node_name} parameter set timed out"}
        # The node's own on-set callback may REJECT the value. Surfacing its
        # reason verbatim is the point: a rejected set that reported success
        # is precisely the class of bug this project already hit once with
        # command_source, and the browser must never paper over it.
        failures = [
            r.reason for r in result.results if not r.successful
        ]
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
