"""
scan_sanitizer_node.py — republish /scan with the TG30's no-return sentinel
rewritten to +inf, at FULL resolution, for consumers we do not control.

What it does:
    Subscribes to the raw /scan and republishes it unchanged except that every
    invalid range — 0.0, negative, below range_min, or NaN — becomes +inf, the
    universal "no return" encoding. Ray count, angles, frame and stamp are
    preserved exactly.

Why it exists:
    The YDLIDAR TG30 encodes a no-return as 0.0, NOT inf, and ~31-39% of rays
    in a normal room are no-returns (measured repeatedly on bernard-1; the
    driver's own `invalid_range_is_inf` flag is INERT here). Every consumer
    therefore has to know that 0.0 means "nothing there" rather than "an
    obstacle touching the sensor".

    Our own code does know: rm_policy.scan_resampler maps no-returns to
    range_max before the policy sees them, and rm_safety filters by the
    LaserScan's own range_min. THIRD-PARTY code does not. slam_toolbox reads
    the raw /scan with nothing of ours in between, and whether its
    `min_laser_range` DISCARDS a short reading or CLAMPS it is an
    implementation detail of a dependency — one we would be betting the pose
    estimate on. Measured on hardware 2026-08-29 with slam_toolbox on the raw
    scan: the map grew to 6.35 x 12.70 m with its origin 10.5 m from the robot
    in a room the robot had driven ~2 m across, and the goal visibly wandered.

    Rewriting the sentinel upstream removes the question entirely. inf is
    unambiguous in a LaserScan and every consumer handles it the same way.

    This is the same protection scan_resampler gives the policy path, applied
    to a path we do not own. It is deliberately NOT done by patching a
    dependency or the vendored policy code.

Subscribes:  <scan_topic>    [sensor_msgs/LaserScan]  raw, default /scan
Publishes:   <output_topic>  [sensor_msgs/LaserScan]  sanitized, default
                             /scan_sanitized
Parameters:  scan_topic    (str, "/scan")
Parameters:  output_topic  (str, "/scan_sanitized")

Failure mode:
    Fail-quiet and additive. It publishes nothing when it has received
    nothing, so a consumer's own staleness watchdog is what reacts — this node
    never fabricates a scan. A malformed message is dropped with a throttled
    warning rather than being forwarded half-converted, because a partially
    sanitized scan is worse than none: it would look valid. It publishes no
    command topic and cannot move the robot.
"""

from __future__ import annotations

import math

import rclpy
import rclpy.node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_DEFAULT_SCAN_TOPIC = "/scan"
_DEFAULT_OUTPUT_TOPIC = "/scan_sanitized"

_WARN_THROTTLE_SEC = 5.0


def sanitize_ranges(ranges, range_min: float):
    """Rewrite every invalid range to +inf. Pure, and the whole of the logic.

    "Invalid" is: NaN, non-positive (the TG30's 0.0 sentinel and any negative),
    or below the sensor's own advertised range_min. Values at or above
    range_min pass through untouched, including existing infs.

    range_max is deliberately NOT applied. A reading beyond range_max is a
    long reading, not an invalid one, and clamping it here would invent an
    obstacle at the ceiling distance.
    """
    out = []
    for r in ranges:
        if r is None:
            out.append(math.inf)
            continue
        v = float(r)
        if math.isnan(v) or v <= 0.0 or v < range_min:
            out.append(math.inf)
        else:
            out.append(v)
    return out


class ScanSanitizerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("scan_sanitizer")

        self.declare_parameter("scan_topic", _DEFAULT_SCAN_TOPIC)
        self.declare_parameter("output_topic", _DEFAULT_OUTPUT_TOPIC)
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        # Same asymmetry as scan_bridge_node, and for the same reason:
        # subscribe BEST_EFFORT to match the sensor driver, publish RELIABLE so
        # a depth-only subscriber (which is RELIABLE by default, as
        # slam_toolbox's is) will accept us. A RELIABLE subscriber REFUSES a
        # BEST_EFFORT publisher, and the symptom is silence that looks exactly
        # like a dead sensor. Do NOT "tidy" these to match.
        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(LaserScan, output_topic, pub_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sub_qos)

        self.get_logger().info(
            f"scan_sanitizer: {scan_topic} -> {output_topic}; "
            f"invalid ranges (0.0 / negative / NaN / < range_min) -> +inf"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            out = LaserScan()
            # Copy the geometry verbatim. Any divergence here would be a
            # silent frame/timing error in whatever consumes this.
            out.header = msg.header
            out.angle_min = msg.angle_min
            out.angle_max = msg.angle_max
            out.angle_increment = msg.angle_increment
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = msg.range_min
            out.range_max = msg.range_max
            out.intensities = msg.intensities
            out.ranges = sanitize_ranges(msg.ranges, msg.range_min)
            self._pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(
                f"scan_sanitizer: dropping malformed scan: {exc!r}",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSanitizerNode()
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
