"""
scan_rotate_node.py — republish a LaserScan turned about the sensor's axis.

What it does:
    Subscribes to the lidar driver's scan and republishes it unchanged except
    that `angle_min` and `angle_max` are shifted by `yaw_deg`, so a sensor
    whose zero direction does not point along the robot's +x reads as if it
    did. Ray count, ranges, intensities, frame and stamp are preserved
    exactly; nothing is resampled.

Why it exists:
    Every consumer of /scan in this stack reads the ray angles directly
    (scan_resampler, rm_safety's arcs, roam's free-space draw, the dashboard)
    and all assume angle 0 is the robot's forward, which the contract fixes
    with a laser_link that carries no rotation. On bernard-3 the RPLIDAR A1
    is mounted with its zero pointing BACKWARD (2026-11-18: the half of the
    scan the mount blocks sat at the scan's front, and the operator's hand
    at the robot's rear showed up ahead). Rotating the published angles is
    one change in one place; rotating laser_link in TF would fix SLAM alone
    and leave the policy and the e-stop reading the world back to front.

    Angles are only shifted, never wrapped per ray: consumers normalise each
    ray's angle to (-pi, pi] themselves, and `angle_min` is kept inside
    [-pi, pi) so a LaserScan reader that assumes a bounded start still works.

Subscribes:  <input_topic>   [sensor_msgs/LaserScan]  the driver, e.g. /scan_driver
Publishes:   <output_topic>  [sensor_msgs/LaserScan]  the rotated scan, e.g. /scan
Parameters:  input_topic   (str, "/scan_driver")
Parameters:  output_topic  (str, "/scan")
Parameters:  yaw_deg       (double, REQUIRED and non-zero: a zero offset means
                           this node should not be running; the launch starts
                           it only when lidar_yaw_deg is set)

Failure mode:
    Fail-quiet and additive, like scan_sanitizer_node: it publishes nothing
    until it has received something, a malformed message is dropped with a
    throttled warning, and it has no command topic. A missing or zero
    `yaw_deg` refuses to start.
"""

from __future__ import annotations

import math

import rclpy
import rclpy.node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_DEFAULT_INPUT_TOPIC = "/scan_driver"
_DEFAULT_OUTPUT_TOPIC = "/scan"
_WARN_THROTTLE_SEC = 5.0


def rotated_angle_min(angle_min: float, yaw_rad: float) -> float:
    """`angle_min` shifted by `yaw_rad`, wrapped into [-pi, pi)."""
    a = angle_min + yaw_rad
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ScanRotateNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("scan_rotate")
        self.declare_parameter("input_topic", _DEFAULT_INPUT_TOPIC)
        self.declare_parameter("output_topic", _DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("yaw_deg", 0.0)
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        yaw_deg = float(self.get_parameter("yaw_deg").get_parameter_value().double_value)
        if not math.isfinite(yaw_deg) or yaw_deg == 0.0:
            raise ValueError(
                f"scan_rotate: yaw_deg must be a finite, non-zero angle in degrees "
                f"(the lidar's zero direction measured from the robot's +x, +left), "
                f"got {yaw_deg!r}. With no offset this node must not run."
            )
        self._yaw_rad = math.radians(yaw_deg)
        # Sensor QoS both ways: the driver publishes best effort, and every
        # consumer of /scan in this stack subscribes best effort.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(LaserScan, output_topic, qos)
        self.create_subscription(LaserScan, input_topic, self._on_scan, qos)
        self.get_logger().info(
            f"scan_rotate: {input_topic} -> {output_topic}, angles shifted by "
            f"{yaw_deg:g} deg (the lidar's zero points {yaw_deg:g} deg left of the robot's +x)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            out = LaserScan()
            out.header = msg.header
            n = len(msg.ranges)
            out.angle_min = rotated_angle_min(float(msg.angle_min), self._yaw_rad)
            out.angle_increment = msg.angle_increment
            out.angle_max = out.angle_min + (n - 1) * float(msg.angle_increment) if n > 1 else out.angle_min
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = msg.range_min
            out.range_max = msg.range_max
            out.ranges = msg.ranges
            out.intensities = msg.intensities
            self._pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(
                f"scan_rotate: dropping malformed scan: {exc!r}",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRotateNode()
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
