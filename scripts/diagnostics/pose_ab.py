#!/usr/bin/env python3
"""
pose_ab.py — measure what SLAM is actually correcting, over a real driven run.

What it does:
    Answers the one question that decides `pose_source`: how much error is
    `/odom` accumulating per metre driven, and how often would SLAM's
    corrections reset the policy's recurrent state?

    `map -> odom` IS the SLAM correction — the transform slam_toolbox
    publishes to move the drifting odometry estimate onto the map. Its
    MAGNITUDE is therefore the odometry error SLAM has computed, in metres,
    and its STEPS are loop closures. This script samples it for a fixed
    duration alongside the distance actually driven, and writes both to a CSV.

    Intended as an A/B across two runs of the same stack:

      A. with_slam:=true pose_source:=odom   SLAM watches, policy drives on
                                             raw odometry. The correction is
                                             MEASURED but not APPLIED, so it
                                             is a clean reading of the error.
      B. with_slam:=true                     (pose_source follows to "map")
                                             correction applied; the run is
                                             for counting loop closures, i.e.
                                             how often policy_node resets the
                                             CfC recurrent state.

    Correction magnitude alone is not interpretable — 15 cm over 2 m is bad,
    over 30 m is fine — so distance driven is recorded in the same pass and
    reported as error-per-metre.

    READ-ONLY. Publishes NOTHING and calls no service, so it cannot move the
    robot and is safe to leave running across a kill/reset cycle. It is not a
    substitute for the operator's hand on /safety/kill.

Preconditions, CHECKED INSIDE THE MEASUREMENT (not asserted in a print):
    - no duplicate node names          a second stack means two EKFs and two
                                       slam_toolboxes writing the same TF
                                       edges, and the pose jumps between two
                                       independent estimators. That reads
                                       exactly like the drift this script is
                                       here to measure (2026-08-29 s7).
    - exactly one /cmd_vel publisher   the same duplicate-stack proxy, and the
                                       invariant this workspace exists for.
    - map -> odom resolves             absent means slam_toolbox is not
                                       running, or is silently rejecting every
                                       scan, and there is nothing to measure.
    - the robot actually MOVED         a stationary run measures nothing about
                                       drift. Reported, never assumed: a
                                       driven run was once written up as a
                                       stationary one and every number under
                                       the heading was wrong (2026-08-29 s7).

Subscribes:
    /tf, /tf_static via tf2   map -> odom (the correction), map -> base_footprint
    /odom            [nav_msgs/Odometry]      pose + twist, distance driven
    <policy row>     [geometry_msgs/Twist]    yaw sign flips (the limit cycle);
                                              --policy names the
                                              config/topics.yaml row
    /safety/status   [rm_interfaces/SafetyStatus]  reason distribution
    /scan_sanitized  [sensor_msgs/LaserScan]  zero count — SLAM's real input
    <its debug>      [std_msgs/String]        that policy node's own pose_source

Publishes:
    nothing. Ever.

Failure mode:
    Fails closed on the preconditions: refuses to record rather than produce a
    CSV that looks valid and is not. Once recording, it is best-effort — a
    topic that never publishes leaves its columns empty and is called out in
    the summary, rather than blocking or being silently filled with zeros.
    Rows are flushed as they are taken, so a Ctrl-C or a battery cut keeps
    every sample up to that point.
"""
from __future__ import annotations

import argparse
import pathlib
import csv
import json
import math
import sys
import time
from collections import Counter
from typing import Optional, Tuple

import rclpy
import rclpy.node
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from rm_interfaces.msg import SafetyStatus

# SlamHealth already implements correction accumulation and loop-closure
# detection, is unit-tested, and is what the dashboard reports from. Reusing
# it means the CSV and the dashboard panel cannot disagree about what counts
# as a loop closure.
from rm_dashboard.slam_health import SlamHealth

# The repo this script lives in: scripts/diagnostics/pose_ab.py -> parents[2].
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

_MAP_FRAME = "map"
_ODOM_FRAME = "odom"
_BASE_FRAME = "base_footprint"

# Sampling rate. Above the EKF's 30 Hz would oversample the pose; well below
# it would alias the correction steps that ARE the loop closures. 5 Hz also
# matches the control rate, so one row is roughly one policy decision.
_SAMPLE_HZ = 5.0

# How long to wait for map -> odom before giving up. DDS discovery on a fresh
# node plus slam_toolbox's first scan match is several seconds; a short wait
# turns "SLAM is starting" into "SLAM is broken" (velocity_accuracy.py made
# exactly this mistake with /scan and silently skipped its runway check).
_TF_WAIT_S = 15.0

# Below this peak speed the run did not meaningfully drive and its correction
# figure says nothing about drift per metre.
_MOVED_VX_MPS = 0.02
_MOVED_WZ_RPS = 0.05

# A policy yaw command smaller than this is noise, not a direction choice —
# excluded from the sign-flip statistic so a near-zero wobble is not counted
# as a limit cycle.
_WZ_SIGNIFICANT = 0.05

# Fraction of consecutive significant policy commands that must alternate sign
# before the period-2 limit cycle is called out in the summary.
_FLIP_FRACTION_ALERT = 0.6

_STATE_NAME = {
    SafetyStatus.OK: "OK",
    SafetyStatus.SLOW: "SLOW",
    SafetyStatus.STOPPED: "STOPPED",
    SafetyStatus.KILLED: "KILLED",
}

_COLUMNS = [
    "t_s",
    "corr_x", "corr_y", "corr_yaw", "corr_m",
    "odom_x", "odom_y", "odom_yaw",
    "map_x", "map_y", "map_yaw",
    "odom_path_m", "map_path_m",
    "vx", "wz",
    "policy_wz",
    "safety_state", "safety_reason", "min_range",
    "scan_zeros",
]


def _wrap_pi(a: float) -> float:
    """Signed angle difference in (-pi, pi]. A raw subtraction across the wrap
    reads as a ~6.28 rad spin — the same trap pose_source.py documents."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat(q) -> float:
    """Yaw only — the stack is planar and two_d_mode is on in the EKF."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PoseAB(rclpy.node.Node):
    def __init__(self, policy_topic: str, debug_topic: str) -> None:
        super().__init__("pose_ab")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._slam = SlamHealth()

        # --- odom ---------------------------------------------------------
        self._odom: Optional[Tuple[float, float, float]] = None
        self._odom_prev: Optional[Tuple[float, float]] = None
        self._odom_path_m = 0.0
        self._odom_first: Optional[Tuple[float, float]] = None
        self._vx = float("nan")
        self._wz = float("nan")
        self._peak_vx = 0.0
        self._peak_wz = 0.0
        self._odom_count = 0

        # --- map pose -----------------------------------------------------
        self._map_prev: Optional[Tuple[float, float]] = None
        self._map_path_m = 0.0
        # Extent of the map-frame pose, which is the ONLY honest drift measure
        # for a stationary robot. See _CORRECTION_IS_ROTATION_CONTAMINATED.
        self._map_min: Optional[list] = None
        self._map_max: Optional[list] = None
        self._corr_yaw_prev: Optional[float] = None
        self._corr_yaw_total = 0.0
        self._corr_yaw_seen = False

        # --- policy -------------------------------------------------------
        self._policy_wz = float("nan")
        self._policy_prev_sign = 0
        self._policy_significant = 0
        self._policy_flips = 0
        self._policy_count = 0

        # --- safety -------------------------------------------------------
        self._state = -1
        self._reason = ""
        self._min_range = float("nan")
        self._reasons: Counter = Counter()

        # --- scan ---------------------------------------------------------
        self._scan_zeros = -1
        self._scan_zeros_max = -1
        self._scan_count = 0

        # --- policy_node self-report ---------------------------------------
        self._pose_source: Optional[str] = None
        self._policy_fault: Optional[str] = None

        # Depth-10 (RELIABLE) throughout: every publisher here is depth-10 or
        # explicitly RELIABLE. A BEST_EFFORT profile copied from the sensor
        # path would silently receive nothing from /scan_sanitized — the
        # mismatch class that has cost this project two sessions.
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, policy_topic, self._on_policy, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_status, 10)
        self.create_subscription(LaserScan, "/scan_sanitized", self._on_scan, 10)
        self.create_subscription(String, debug_topic, self._on_debug, 10)

    # -- callbacks ---------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
            return
        self._odom_count += 1
        self._odom = (p.x, p.y, yaw)
        if self._odom_first is None:
            self._odom_first = (p.x, p.y)
        if self._odom_prev is not None:
            self._odom_path_m += math.hypot(p.x - self._odom_prev[0], p.y - self._odom_prev[1])
        self._odom_prev = (p.x, p.y)

        self._vx = msg.twist.twist.linear.x
        self._wz = msg.twist.twist.angular.z
        if math.isfinite(self._vx):
            self._peak_vx = max(self._peak_vx, abs(self._vx))
        if math.isfinite(self._wz):
            self._peak_wz = max(self._peak_wz, abs(self._wz))

    def _on_policy(self, msg: Twist) -> None:
        wz = msg.angular.z
        self._policy_wz = wz
        self._policy_count += 1
        if not math.isfinite(wz) or abs(wz) < _WZ_SIGNIFICANT:
            return
        sign = 1 if wz > 0 else -1
        if self._policy_prev_sign != 0:
            self._policy_significant += 1
            if sign != self._policy_prev_sign:
                self._policy_flips += 1
        self._policy_prev_sign = sign

    def _on_status(self, msg: SafetyStatus) -> None:
        self._state = msg.state
        self._reason = msg.reason
        self._min_range = msg.min_range
        self._reasons[msg.reason] += 1

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_count += 1
        zeros = sum(1 for r in msg.ranges if r == 0.0)
        self._scan_zeros = zeros
        self._scan_zeros_max = max(self._scan_zeros_max, zeros)

    def _on_debug(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(payload, dict):
            self._pose_source = payload.get("pose_source", self._pose_source)
            self._policy_fault = payload.get("fault", self._policy_fault)

    # -- tf ----------------------------------------------------------------

    def lookup(self, parent: str, child: str) -> Optional[Tuple[float, float, float]]:
        """Latest available transform, or None. Never raises."""
        try:
            tf = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        yaw = _yaw_from_quat(tf.transform.rotation)
        if not all(math.isfinite(v) for v in (t.x, t.y, yaw)):
            return None
        return (t.x, t.y, yaw)

    # -- one sample --------------------------------------------------------

    def sample(self, t_s: float) -> dict:
        corr = self.lookup(_MAP_FRAME, _ODOM_FRAME)
        if corr is not None:
            self._slam.update(corr[0], corr[1], corr[2], t_s)

        if corr is not None:
            # Summing WRAPPED increments, not last-minus-first: the correction
            # ratchets monotonically on this robot and a total past pi would
            # otherwise fold back on itself.
            if self._corr_yaw_prev is not None:
                self._corr_yaw_total += _wrap_pi(corr[2] - self._corr_yaw_prev)
            self._corr_yaw_prev = corr[2]
            self._corr_yaw_seen = True

        map_pose = self.lookup(_MAP_FRAME, _BASE_FRAME)
        if map_pose is not None:
            if self._map_min is None:
                self._map_min = list(map_pose)
                self._map_max = list(map_pose)
            else:
                for i in range(3):
                    self._map_min[i] = min(self._map_min[i], map_pose[i])
                    self._map_max[i] = max(self._map_max[i], map_pose[i])
            if self._map_prev is not None:
                self._map_path_m += math.hypot(
                    map_pose[0] - self._map_prev[0], map_pose[1] - self._map_prev[1]
                )
            self._map_prev = (map_pose[0], map_pose[1])

        row = {c: "" for c in _COLUMNS}
        row["t_s"] = round(t_s, 3)
        if corr is not None:
            row["corr_x"] = round(corr[0], 4)
            row["corr_y"] = round(corr[1], 4)
            row["corr_yaw"] = round(corr[2], 4)
            row["corr_m"] = round(math.hypot(corr[0], corr[1]), 4)
        if self._odom is not None:
            row["odom_x"] = round(self._odom[0], 4)
            row["odom_y"] = round(self._odom[1], 4)
            row["odom_yaw"] = round(self._odom[2], 4)
        if map_pose is not None:
            row["map_x"] = round(map_pose[0], 4)
            row["map_y"] = round(map_pose[1], 4)
            row["map_yaw"] = round(map_pose[2], 4)
        row["odom_path_m"] = round(self._odom_path_m, 4)
        row["map_path_m"] = round(self._map_path_m, 4)
        row["vx"] = "" if math.isnan(self._vx) else round(self._vx, 4)
        row["wz"] = "" if math.isnan(self._wz) else round(self._wz, 4)
        row["policy_wz"] = "" if math.isnan(self._policy_wz) else round(self._policy_wz, 4)
        row["safety_state"] = _STATE_NAME.get(self._state, "")
        row["safety_reason"] = self._reason
        row["min_range"] = "" if math.isnan(self._min_range) else round(float(self._min_range), 4)
        row["scan_zeros"] = "" if self._scan_zeros < 0 else self._scan_zeros
        return row

    # -- preconditions -----------------------------------------------------

    def check_preconditions(self) -> list:
        """Every reason this run would be uninterpretable. Empty means go."""
        problems = []

        names = [f"{ns.rstrip('/')}/{n}" for n, ns in
                 self.get_node_names_and_namespaces()]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            problems.append(
                "DUPLICATE NODES — a second stack is running, and its pose "
                "estimator will be mistaken for drift: " + ", ".join(duplicates)
                + ". Run scripts/stack_down.sh."
            )

        try:
            publishers = self.get_publishers_info_by_topic("/cmd_vel")
        except Exception:
            publishers = []
        if len(publishers) != 1:
            problems.append(
                f"/cmd_vel has {len(publishers)} publishers, expected exactly 1"
                + (" — is the stack up?" if not publishers else
                   " — TWO PUBLISHERS, stop and run scripts/stack_down.sh.")
            )

        return problems

    def wait_for_correction(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.lookup(_MAP_FRAME, _ODOM_FRAME) is not None:
                return True
        return False

    # -- summary -----------------------------------------------------------

    def summary(self, elapsed_s: float) -> str:
        snap = self._slam.snapshot(elapsed_s, max_age_s=5.0)
        lines = ["", "=" * 68, "pose_ab summary", "=" * 68]

        lines.append(f"  duration              {elapsed_s:.1f} s")
        lines.append(f"  policy pose_source    {self._pose_source or '(no /ncp_debug)'}")
        if self._policy_fault:
            lines.append(f"  policy fault          {self._policy_fault}")

        lines.append("")
        lines.append("  -- did it actually drive? ------------------------------")
        moved = self._peak_vx >= _MOVED_VX_MPS or self._peak_wz >= _MOVED_WZ_RPS
        lines.append(f"  peak |vx| / |wz|      {self._peak_vx:.4f} m/s  {self._peak_wz:.4f} rad/s")
        lines.append(f"  odom path length      {self._odom_path_m:.3f} m")
        if self._odom_first is not None and self._odom is not None:
            net = math.hypot(self._odom[0] - self._odom_first[0],
                             self._odom[1] - self._odom_first[1])
            lines.append(f"  net displacement      {net:.3f} m")
        if not moved:
            lines.append("  *** THE ROBOT DID NOT MOVE. Correction figures below say")
            lines.append("      nothing about drift per metre — rerun while driving.")

        lines.append("")
        lines.append("  -- the SLAM correction (map -> odom) -------------------")
        if not snap["available"]:
            lines.append("  *** map -> odom NEVER RESOLVED — no data.")
        else:
            corr_now = snap["correction_m"]
            corr_max = snap["max_correction_m"]
            lines.append(f"  correction |t| now/max {corr_now:.4f} m / {corr_max:.4f} m"
                         "   (see note below)")
            if self._corr_yaw_seen:
                dyaw = self._corr_yaw_total
                rate = math.degrees(dyaw) / elapsed_s if elapsed_s > 0 else 0.0
                lines.append(f"  correction YAW drift  {math.degrees(dyaw):+.1f} deg over the "
                             f"run  ({rate:+.2f} deg/s)")
            lines.append(f"  correction steps      {snap['jump_count']}"
                         "   (each one resets the CfC recurrent state)")
            lines.append(f"  odom path / map path  {self._odom_path_m:.3f} m / "
                         f"{self._map_path_m:.3f} m")
            if self._map_path_m > 0.10:
                lines.append(f"  odom linear over-read {(self._odom_path_m / self._map_path_m - 1.0) * 100:+.1f} %")
            lines.append("")
            lines.append("  NOTE: correction |t| is the translation of map -> odom, and it is")
            lines.append("  ROTATION-CONTAMINATED — the correction rotates about the ODOM")
            lines.append("  ORIGIN, so once the robot is metres away a pure yaw correction")
            lines.append("  sweeps |t| through metres while the robot has not moved at all.")
            lines.append("  Do NOT divide it by distance driven. The honest drift measure for")
            lines.append("  a stationary robot is the map-frame pose extent below.")

        lines.append("")
        lines.append("  -- map-frame pose extent (the honest drift measure) ----")
        if self._map_min is None:
            lines.append("  *** map -> base_footprint never resolved.")
        else:
            dx = self._map_max[0] - self._map_min[0]
            dy = self._map_max[1] - self._map_min[1]
            dyaw = self._map_max[2] - self._map_min[2]
            lines.append(f"  x / y / yaw range     {dx:.4f} m  {dy:.4f} m  {dyaw:.4f} rad")
            if not moved:
                lines.append("  Stationary, so this IS the drift. Tens of mm is SLAM working;")
                lines.append("  a yaw range of tenths of a radian means odom yaw is drifting")
                lines.append("  and SLAM is sawtoothing to cancel it.")

        lines.append("")
        lines.append("  -- policy yaw (the limit-cycle question) ---------------")
        if self._policy_significant == 0:
            lines.append(f"  {self._policy_count} commands, none above |wz| "
                         f"{_WZ_SIGNIFICANT} — nothing to judge.")
        else:
            frac = self._policy_flips / self._policy_significant
            lines.append(f"  sign flips            {self._policy_flips}/"
                         f"{self._policy_significant} consecutive ({frac:.0%})")
            if frac >= _FLIP_FRACTION_ALERT:
                lines.append("  *** ALTERNATING — the period-2 limit cycle is present")
                lines.append("      IN MOTION, which offline analysis predicted it")
                lines.append("      would not be. Worth reporting.")
            else:
                lines.append("  committed direction — the limit cycle resolved in motion.")

        lines.append("")
        lines.append("  -- sanity ----------------------------------------------")
        if self._scan_count == 0:
            lines.append("  *** /scan_sanitized NEVER PUBLISHED — SLAM has no input.")
        else:
            flag = "" if self._scan_zeros_max == 0 else "   *** MUST BE 0"
            lines.append(f"  /scan_sanitized zeros max {self._scan_zeros_max}{flag}")
        if self._odom_count == 0:
            lines.append("  *** /odom NEVER PUBLISHED — the odom chain is not running.")
        if self._reasons:
            top = ", ".join(f"{r} x{n}" for r, n in self._reasons.most_common(5))
            lines.append(f"  safety reasons        {top}")
        else:
            lines.append("  *** /safety/status NEVER PUBLISHED.")

        lines.append("=" * 68)
        return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Measure the SLAM correction over a driven run (read-only).")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="seconds to record (default 60)")
    parser.add_argument("--out", default="pose_ab.csv",
                        help="CSV path (appended to if it exists)")
    parser.add_argument("--label", default="",
                        help="free text stamped into the summary, e.g. 'A odom'")
    parser.add_argument("--force", action="store_true",
                        help="record even if a precondition fails. Do not use "
                             "to get past a duplicate stack — fix the stack.")
    parser.add_argument("--policy", default="m8v2",
                        help="which config/topics.yaml policy row to watch "
                             "(its command topic and debug stream); default m8v2")
    args = parser.parse_args(argv)

    # The row's topics come from the same table rm_safety and the launch use.
    sys.path.insert(0, str(_REPO_ROOT / "src" / "rm_safety"))
    from rm_safety.command_sources import load_command_sources
    sources = load_command_sources(str(_REPO_ROOT / "config" / "topics.yaml"))
    source = sources.get(args.policy)
    if source is None or not source.is_policy or not source.debug:
        print(f"--policy must name a policy row with a debug stream: "
              f"{sorted(n for n, s in sources.items() if s.is_policy and s.debug)}",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = PoseAB(source.topic, source.debug)
    try:
        # Let discovery settle before judging what is running: a fresh node
        # sees an empty graph for the first second or so, and "0 /cmd_vel
        # publishers" would be a lie about a healthy stack.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        problems = node.check_preconditions()
        if not node.wait_for_correction(_TF_WAIT_S):
            problems.append(
                f"map -> odom did not resolve in {_TF_WAIT_S:.0f} s. Either "
                "slam_toolbox is not running (with_slam:=false) or it is "
                "rejecting every scan, which it does SILENTLY.")

        if problems:
            print("PRECONDITIONS FAILED:", file=sys.stderr)
            for p in problems:
                print(f"  - {p}", file=sys.stderr)
            if not args.force:
                print("\nRefusing to record. Re-run with --force to override.",
                      file=sys.stderr)
                return 2
            print("\n--force given: recording anyway. The data is suspect.",
                  file=sys.stderr)

        header_needed = True
        try:
            with open(args.out, "r", encoding="utf-8") as existing:
                header_needed = not existing.readline().strip()
        except FileNotFoundError:
            header_needed = True

        print(f"recording {args.duration:.0f} s to {args.out} — drive now.")
        started = time.monotonic()
        period = 1.0 / _SAMPLE_HZ
        with open(args.out, "a", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=_COLUMNS)
            if header_needed:
                writer.writeheader()
            next_tick = started
            while True:
                elapsed = time.monotonic() - started
                if elapsed >= args.duration:
                    break
                next_tick += period
                while time.monotonic() < next_tick:
                    rclpy.spin_once(node, timeout_sec=0.02)
                writer.writerow(node.sample(time.monotonic() - started))
                handle.flush()

        elapsed = time.monotonic() - started
        if args.label:
            print(f"\nlabel: {args.label}")
        print(node.summary(elapsed))
        print(f"\nCSV: {args.out}")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted — rows written so far are kept.", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
