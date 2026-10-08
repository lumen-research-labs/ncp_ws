"""
scan_bridge_node.py — republish the raw TG30 scan as the 720-beam/270 deg
LaserScan the trained NCP was trained on.

What it does:
    Subscribes to the robot's real `/scan` (YDLIDAR TG30: ~1982 points over
    a full 360 deg, no-returns encoded as 0.0) and publishes a SYNTHETIC
    `sensor_msgs/LaserScan` on `/ncp/scan` containing exactly
    `ncp_policy.BEAMS` (720) ranges across the forward `ncp_policy.FOV_DEG`
    (270) degrees, right -> left, in metres.

    This exists so that `NCP-on-Rosmaster/rosmaster/node.py` can run
    COMPLETELY UNMODIFIED against real hardware. That node hard-refuses any
    scan whose length is not `policy.BEAMS` (`if len(msg.ranges) != policy.BEAMS:
    return`), so pointed at the raw TG30 it never builds an observation, its
    1 s deadman never clears, and it publishes zero forever -- in BOTH
    `mode:=debug` and `mode:=ncp`. Rather than fork the upstream node (whose
    policy half is vendored byte-exact here on operator instruction), this
    node reshapes the input to match what upstream demands.

    All the reduction work is `rm_policy.scan_resampler`; this file is only
    the ROS wrapper around it. In particular the TG30's 0.0-no-return
    sentinel is handled there (mapped to range_max BEFORE pooling), which is
    the one thing upstream's `prepare_scan` -- which only maps non-finite --
    would get wrong on this hardware. The range bounds, the nearest-ray
    search window and the self-occlusion zones come from the lidar block of
    config/interface.yaml, read through `rm_policy.lidar_contract`, so this
    node and policy_node reduce the scan identically.

    Synthetic geometry is chosen so upstream's own `policy.check_scan()`
    passes cleanly: 720 bin CENTRES spaced 270/720 = 0.375 deg apart span
    269.625 deg (within its 10 deg tolerance of 270) and are centred on
    0 rad (straight ahead, REP-103).

Subscribes:  /scan      [sensor_msgs/LaserScan]  ~10.07 Hz, raw TG30
Publishes:   /ncp/scan  [sensor_msgs/LaserScan]  same rate, 720 beams
Parameters:  scan_topic             (str, "/scan")       raw input
Parameters:  output_topic           (str, "/ncp/scan")   720-beam output
Parameters:  interface_config_path  (str, <repo-relative default>) — path to
                            config/interface.yaml. Its lidar block supplies
                            range_min_m / range_max_m (the TG30 spec bounds,
                            NOT the 5.0 m training clip, which upstream's
                            build_obs applies itself), the search window and
                            the self-occlusion zones. Refuses to start if the
                            file cannot be read or its bin count / arc differ
                            from the vendored checkpoint module.

Failure mode:
    Fail-closed by SILENCE. On any resampler fault (empty scan, bad angle
    increment, an empty bin) this node publishes NOTHING for that tick and
    logs a throttled warning. It never emits a plausible-looking
    all-range_max scan, because "clear path everywhere" is the single most
    dangerous thing to fabricate. A sustained fault therefore starves the
    consumer, whose own deadman (upstream node.py's 1 s SCAN_TIMEOUT, or
    rm_safety's scan_timeout_ms) stops the robot -- which is the desired
    outcome and needs no cooperation from this node.
"""

from __future__ import annotations

import math
import pathlib

import rclpy
import rclpy.node
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from rm_policy import scan_resampler
from rm_policy.lidar_contract import lidar_contract_from
from rm_policy.vendor import ncp_policy

_DEFAULT_SCAN_TOPIC = "/scan"
_DEFAULT_OUTPUT_TOPIC = "/ncp/scan"

# Same resolution convention as policy_node: valid in a standard colcon
# workspace where the package source tree is adjacent to config/.
_THIS_DIR = pathlib.Path(__file__).parent.resolve()
_DEFAULT_INTERFACE_CONFIG_PATH = str(_THIS_DIR.parents[2] / "config" / "interface.yaml")

_WARN_THROTTLE_SEC = 5.0


class ScanBridgeNode(rclpy.node.Node):
    """Raw 360 deg TG30 scan in, 720-beam/270 deg synthetic scan out."""

    def __init__(self) -> None:
        super().__init__("ncp_scan_bridge")

        self.declare_parameter("scan_topic", _DEFAULT_SCAN_TOPIC)
        self.declare_parameter("output_topic", _DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("interface_config_path", _DEFAULT_INTERFACE_CONFIG_PATH)

        scan_topic = self.get_parameter("scan_topic").value
        output_topic = self.get_parameter("output_topic").value
        interface_config_path = self.get_parameter("interface_config_path").value

        # The lidar block of the contract. Fatal if unreadable or if it
        # disagrees with the vendored checkpoint module about the layout:
        # upstream's node would refuse every scan, and silently.
        try:
            with open(interface_config_path, "r", encoding="utf-8") as fh:
                interface_cfg = yaml.safe_load(fh)
            self._lidar = lidar_contract_from(interface_cfg)
        except (OSError, yaml.YAMLError, ValueError) as exc:
            raise RuntimeError(
                f"ncp_scan_bridge: cannot read the lidar contract from "
                f"{interface_config_path!r}: {exc}"
            ) from exc
        if (self._lidar.n_beams, self._lidar.arc_deg) != (ncp_policy.BEAMS, ncp_policy.FOV_DEG):
            raise RuntimeError(
                f"ncp_scan_bridge: interface.yaml declares {self._lidar.n_beams} beams "
                f"over {self._lidar.arc_deg} deg, but the vendored checkpoint module is "
                f"built for {ncp_policy.BEAMS} beams over {ncp_policy.FOV_DEG} deg."
            )
        self._range_min = self._lidar.range_min_m
        self._range_max = self._lidar.range_max_m

        self._n_beams = ncp_policy.BEAMS
        self._arc_deg = ncp_policy.FOV_DEG

        # Synthetic geometry: bin CENTRES, not bin edges. Bin i spans
        # [-135 + i*w, -135 + (i+1)*w] deg, so its centre is at
        # -135 + (i + 0.5)*w. Publishing centres is what makes the output
        # read as "a 720-beam lidar", which is what check_scan validates.
        bin_width = math.radians(self._arc_deg) / self._n_beams
        self._angle_min = -math.radians(self._arc_deg) / 2.0 + bin_width / 2.0
        self._angle_increment = bin_width

        # QoS, and why the two sides DIFFER (found on hardware 2026-08-27):
        #
        # SUBSCRIPTION: BEST_EFFORT, the usual sensor-data profile, matching
        # rm_safety's. A BEST_EFFORT subscriber accepts both RELIABLE and
        # BEST_EFFORT publishers, so this reads any lidar driver.
        #
        # PUBLICATION: RELIABLE, deliberately NOT the sensor default. Our
        # primary consumer is NCP-on-Rosmaster/rosmaster/node.py, whose
        # `create_subscription(LaserScan, scan_topic, cb, 10)` is a depth-only
        # profile and therefore RELIABLE. A RELIABLE subscriber REFUSES a
        # BEST_EFFORT publisher outright: rclpy logs "offering incompatible
        # QoS ... No messages will be received", the subscriber gets nothing,
        # and upstream's deadman then reports "no fresh scan — stopped"
        # forever. That looks exactly like a dead lidar while /ncp/scan is
        # visibly publishing at 10 Hz, which is a nasty thing to debug on
        # hardware. Publishing RELIABLE serves BOTH kinds of subscriber
        # (RELIABLE upstream, BEST_EFFORT rm_policy.policy_node), so it is the
        # compatible choice. Do not "tidy" this to match the subscription.
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, output_topic, pub_qos)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sub_qos)

        self.get_logger().info(
            f"ncp_scan_bridge: {scan_topic} -> {output_topic}, "
            f"{self._n_beams} beams over {self._arc_deg:.0f} deg, "
            f"range [{self._range_min}, {self._range_max}] m, "
            f"search window +/-{self._lidar.search_window_bins} bins, "
            f"{len(self._lidar.mask_zones_deg)} self-occlusion zone(s)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        result = scan_resampler.resample_forward_arc(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            n_bins=self._n_beams,
            arc_deg=self._arc_deg,
            range_min=self._range_min,
            range_max=self._range_max,
            sampling=scan_resampler.SAMPLING_NEAREST_RAY,
            search_window_bins=self._lidar.search_window_bins,
            mask_zones_deg=self._lidar.mask_zones_deg,
        )
        if result.fault is not None or result.bins is None:
            # Publish NOTHING. See the module docstring's failure mode: the
            # consumer's deadman is the stop mechanism, not a fabricated scan.
            self.get_logger().warn(
                f"ncp_scan_bridge: resampler fault {result.fault!r} — "
                f"publishing nothing this tick (consumer deadman will stop).",
                throttle_duration_sec=_WARN_THROTTLE_SEC,
            )
            return

        out = LaserScan()
        out.header = msg.header  # same stamp and frame_id as the real scan
        out.angle_min = self._angle_min
        out.angle_max = self._angle_min + self._angle_increment * (self._n_beams - 1)
        out.angle_increment = self._angle_increment
        out.time_increment = 0.0
        out.scan_time = msg.scan_time
        out.range_min = self._range_min
        out.range_max = self._range_max
        out.ranges = [float(r) for r in result.bins]
        out.intensities = []
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ScanBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
