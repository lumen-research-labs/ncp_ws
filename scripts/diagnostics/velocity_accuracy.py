#!/usr/bin/env python3
"""
velocity_accuracy.py — does the robot actually go as fast as it was told?

WHEELS DOWN, floor test, OPERATOR PRESENT WITH A KILL SWITCH. This script
commands real motion across a matrix of speeds and asks the operator to read a
tape measure / protractor after each leg. It is the acceptance test for
"commanded vx and wz match achieved vx and wz".

Why a whole script rather than `drive_test.sh straight 0.10 5`:

  1. ONE SPEED PROVES NOTHING. The calibration model is
     |pwm| = dead_pwm + gain*|v|, two unknowns per wheel per direction. A
     single-speed check passes for infinitely many wrong (dead_pwm, gain)
     pairs — that exact mistake made a perfect 1.00 m straight line coexist
     with a 79 deg "180 deg" turn (2026-08-29). Accuracy is only established
     across a RANGE, in BOTH directions, on BOTH axes.

  2. `vx * secs` IS THE WRONG REFERENCE. rm_safety acceleration-limits every
     command (max_ax), so a 5 s leg at 0.25 m/s never travels 1.25 m even when
     the base is perfect — the ramp alone costs ~0.04 m. And any slow-zone
     scaling silently steals more. So the reference here is the TIME INTEGRAL
     OF /cmd_vel, the velocity actually demanded of the base after clamping
     and ramping. Tape-vs-integral is a clean statement about the base;
     tape-vs-(vx*secs) confounds the base with the safety layer.

  3. ODOMETRY CANNOT CHECK ITSELF. /odom is integrated from rm_base's own
     encoder reconstruction, whose scale (counts_per_rev 2360) was FITTED to a
     tape measure with any wheel slip baked in. It is recorded here beside the
     tape reading — as a second thing under test, never as the ruler.

Subscribes:
    /cmd_vel        [geometry_msgs/Twist]      the commanded reference
    /odom           [nav_msgs/Odometry]        under test, not ground truth
    /base/debug     [rm_interfaces/BaseDebug]  per-wheel behaviour
    /safety/status  [rm_interfaces/SafetyStatus]
    /scan           [sensor_msgs/LaserScan]    pre-run runway check
Publishes:
    /policy/cmd_vel [geometry_msgs/Twist]  — an INPUT to rm_safety. This
        script never publishes /cmd_vel; rm_safety alone owns it, so every
        clamp, arc and /safety/kill still applies to everything below.
Services called:
    /safety/reset, /safety/kill  [std_srvs/Trigger]

Usage (on the robot, base stack already up):
    bash scripts/bench_up.sh --mode closed_loop --live --source policy

    NOT ncp.launch.py: that always starts a policy consumer, and in debug mode
    the consumer publishes /policy/cmd_vel continuously — the same topic this
    script commands on. Two publishers would fight mid-leg.

    velocity_accuracy.py                      # the default matrix
    velocity_accuracy.py --legs straight      # linear legs only
    velocity_accuracy.py --legs spin
    velocity_accuracy.py --vx 0.20 --secs 5   # one ad-hoc leg
    velocity_accuracy.py --summary out.csv    # re-print a finished sheet

Failure mode:
    Fails closed. Every leg's publisher, zero flood and /safety/kill live in
    one try/finally, so no exit path — normal, Ctrl-C or exception — leaves
    the robot commanded. A leg whose runway is too short, or whose safety
    reason was ever not 'ok', is recorded and flagged rather than silently
    folded into the fit. Results are appended to the CSV after each leg, so an
    aborted session keeps the legs it already measured.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time

# ROS is imported softly so that --summary works on the WORKSTATION, which has
# no rclpy (ROS lives in a container here). Reading a finished sheet is pure
# arithmetic on a CSV and should not require the robot's environment; driving a
# leg obviously does, and _require_ros() refuses that path with the real error.
try:
    import rclpy
    import rclpy.node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Trigger

    from rm_interfaces.msg import BaseDebug, SafetyStatus

    _ROS_ERROR: ImportError | None = None
    _NodeBase = rclpy.node.Node
except ImportError as _exc:                    # workstation, or ROS not sourced
    _ROS_ERROR = _exc
    _NodeBase = object

import yaml

WHEELS = ["FL", "BL", "FR", "BR"]
RATE_HZ = 20.0
FLOOD_SECS = 2.0        # zero flood after each leg; also captures the ramp-down
SETTLE_SECS = 1.5       # zeros before arming, so rm_safety has a fresh source
RUNWAY_ARC_DEG = 30.0   # half-arc searched ahead for the runway check
RUNWAY_MARGIN_M = 0.25  # clearance demanded beyond slow_zone + travel
SCAN_WAIT_SECS = 8.0    # how long to wait for the first /scan before giving up
_STATE = {0: "OK", 1: "SLOW", 2: "STOPPED", 3: "KILLED"}


def _sensor_qos():
    """/scan is BEST_EFFORT; a RELIABLE subscriber silently receives nothing
    from it — the trap that once made a live lidar look dead. Built lazily so
    the module still imports without ROS (see the soft import above)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )


def _require_ros() -> None:
    if _ROS_ERROR is not None:
        print(f"ERROR: this needs ROS, which did not import: {_ROS_ERROR}\n"
              "Run it on the robot with the workspace sourced:\n"
              "  source /opt/ros/humble/setup.bash\n"
              "  source ~/ncp_ws/install/setup.bash\n"
              "(--summary works anywhere; it only reads the CSV.)")
        sys.exit(1)


# Default matrix. Straights span a 4x speed range so dead_pwm and gain are
# separable; each has a reverse partner because the calibration is
# per-direction (back-left is asymmetric, front-right more so). Spins are all
# below the ~0.14 m/s wheel speed at which front-right's breakaway kick
# engages, so they double as the kick's first hardware exercise.
DEFAULT_LEGS = [
    ("straight", 0.06, 6.0),
    ("straight", 0.12, 6.0),
    ("straight", 0.25, 5.0),
    ("straight", -0.12, 6.0),
    ("spin", 0.30, 5.0),
    ("spin", 0.60, 5.0),
    ("spin", -0.30, 5.0),
]

CSV_FIELDS = [
    "kind", "commanded_val", "secs", "naive", "cmd_integral", "odom",
    "measured", "ratio", "lateral", "safety_ok", "runway_m", "wheel_flags",
    # The calibration the leg was driven with. Recorded because the CSV
    # accumulates across sessions and the natural workflow is measure ->
    # change a constant -> measure again: without this, summarise() averages
    # legs from before and after the change together and reports a spread
    # that is really just the edit. It bit us the first time this was used.
    "lever_arm",
]


class Leg(_NodeBase):
    """One commanded leg: arm, drive, stop, kill. Reused across legs."""

    def __init__(self) -> None:
        super().__init__("velocity_accuracy")
        self.vx = 0.0
        self.wz = 0.0
        self.recording = False

        self.cmd_track: list[tuple[float, float, float]] = []   # t, vx, wz
        self.odom_track: list[tuple[float, float, float]] = []   # x, y, yaw
        self.samples: list[BaseDebug] = []
        self.reasons: dict[str, int] = {}
        self.status: SafetyStatus | None = None
        self.scan: LaserScan | None = None

        self.pub = self.create_publisher(Twist, "/policy/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(BaseDebug, "/base/debug", self._on_debug, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_status, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, _sensor_qos())
        self.kill_cli = self.create_client(Trigger, "/safety/kill")
        self.reset_cli = self.create_client(Trigger, "/safety/reset")
        self.create_timer(1.0 / RATE_HZ, self._tick)

    # ---- callbacks --------------------------------------------------------

    def _on_cmd(self, msg: Twist) -> None:
        # The reference. Stamped on ARRIVAL: rm_safety's own header stamp comes
        # from the robot clock, which on bernard-1 is months adrift and reset
        # by hand each session. A monotonic local clock cannot be wrong about
        # an elapsed interval, which is all this integral needs.
        if self.recording:
            self.cmd_track.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _on_odom(self, msg: Odometry) -> None:
        if not self.recording:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_track.append((p.x, p.y, yaw))

    def _on_debug(self, msg: BaseDebug) -> None:
        if self.recording:
            self.samples.append(msg)

    def _on_status(self, msg: SafetyStatus) -> None:
        self.status = msg
        if self.recording:
            self.reasons[msg.reason] = self.reasons.get(msg.reason, 0) + 1

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _tick(self) -> None:
        t = Twist()
        if self.recording:
            t.linear.x = self.vx
            t.angular.z = self.wz
        self.pub.publish(t)

    # ---- helpers ----------------------------------------------------------

    def reset_run(self, vx: float, wz: float) -> None:
        self.vx, self.wz = vx, wz
        self.cmd_track.clear()
        self.odom_track.clear()
        self.samples.clear()
        self.reasons.clear()

    def call(self, cli, name: str) -> bool:
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"!! {name} not available")
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None

    def min_range(self, centre_rad: float) -> float | None:
        """Min real return within +-RUNWAY_ARC_DEG of `centre_rad`, metres.

        Centre is 0 for a forward leg and pi for a reverse one. Checking the
        direction of TRAVEL matters more here than anywhere else in the stack:
        rm_safety's e-stop arcs are forward and lateral only (forward_arc_deg,
        lateral_arc_deg -- there is no rear arc), so when this script commands
        a negative vx NOTHING downstream will stop the robot for an obstacle
        behind it. This check is the only thing looking that way.

        Applies the same 0.0-is-a-no-return rule the observation builder does:
        the TG30 reports no-return as 0.0, never inf, so an untreated 0.0 reads
        as an obstacle at zero distance and would abort every run.
        """
        s = self.scan
        if s is None:
            return None
        half = math.radians(RUNWAY_ARC_DEG)
        best = math.inf
        for i, r in enumerate(s.ranges):
            ang = s.angle_min + i * s.angle_increment - centre_rad
            if abs(math.atan2(math.sin(ang), math.cos(ang))) > half:
                continue
            if r <= s.range_min or math.isnan(r) or math.isinf(r):
                continue
            best = min(best, r)
        return None if math.isinf(best) else best


def spin_for(node: rclpy.node.Node, secs: float) -> None:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for_scan(node: Leg, timeout: float) -> bool:
    """Block until a /scan has arrived, or `timeout` elapses.

    Returns whether one arrived. The scan is kept across legs, so this only
    ever actually waits on the first leg of a run.
    """
    end = time.monotonic() + timeout
    while node.scan is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return node.scan is not None


def integrate(track: list[tuple[float, float, float]]) -> tuple[float, float]:
    """Trapezoidal integral of the /cmd_vel track -> (metres, radians)."""
    dist = yaw = 0.0
    for (t0, vx0, wz0), (t1, vx1, wz1) in zip(track, track[1:]):
        dt = t1 - t0
        if dt <= 0.0 or dt > 0.5:      # a gap that large is a dropout, not motion
            continue
        dist += 0.5 * (vx0 + vx1) * dt
        yaw += 0.5 * (wz0 + wz1) * dt
    return dist, yaw


def odom_delta(track: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """(forward m, lateral m, yaw deg) in the BODY frame at leg start.

    Rotated out of the odom frame deliberately: the odom frame's orientation
    is frozen when the EKF starts, so after the robot is repositioned between
    legs a raw odom dx/dy splits pure forward motion across both axes and
    reads as metres of drift that never happened.
    """
    if len(track) < 2:
        return 0.0, 0.0, 0.0
    x0, y0, yaw0 = track[0]
    x1, y1, yaw1 = track[-1]
    odx, ody = x1 - x0, y1 - y0
    fwd = odx * math.cos(-yaw0) - ody * math.sin(-yaw0)
    lat = odx * math.sin(-yaw0) + ody * math.cos(-yaw0)
    dyaw = math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
    return fwd, lat, dyaw


def ask(prompt: str) -> float | None:
    """Read one number from the operator; blank means 'not measured'."""
    while True:
        raw = input(prompt).strip()
        if raw == "":
            return None
        try:
            return float(raw)
        except ValueError:
            print("   (enter a number, or blank to skip)")


def run_leg(node: Leg, kind: str, val: float, secs: float, limits: dict,
            lever_arm: float | str = "") -> dict | None:
    """Drive one leg and return its result row, or None if the operator skipped."""
    vx = val if kind == "straight" else 0.0
    wz = val if kind == "spin" else 0.0
    naive_m = vx * secs
    naive_deg = math.degrees(wz * secs)

    print("\n" + "=" * 68)
    print(f"LEG: {kind}  {'vx' if kind == 'straight' else 'wz'}={val}  for {secs}s")
    if kind == "straight":
        print(f"     open-loop expectation ~{naive_m:+.3f} m "
              f"(less the accel ramp — the /cmd_vel integral is the reference)")
        print("     Mark the floor at the wheel contact point before and after.")
    else:
        print(f"     open-loop expectation ~{naive_deg:+.1f} deg")
        print("     Mark a heading line on the floor and read the achieved angle.")
    print("=" * 68)

    # ---- runway check -----------------------------------------------------
    # The slow zone scales vx down proportionally and says nothing about it in
    # the motion itself, so an obstacle inside it turns a calibration run into
    # a meaningless under-drive. Check BEFORE arming, not after wondering.
    # WAIT for a scan rather than spinning a fixed 0.5 s and hoping. On the
    # FIRST leg of a run the node has just been created and DDS discovery plus
    # the first 10 Hz message routinely take longer than that -- which silently
    # turned the check into "no /scan, type YES to run anyway" on every
    # single-leg invocation of the first session. Losing the check is worst
    # exactly when it is skipped without being noticed.
    if not wait_for_scan(node, SCAN_WAIT_SECS):
        print(f"  (no /scan yet after {SCAN_WAIT_SECS:.0f} s)")
    # The slow zone is a lidar range built from THIS robot's profile
    # (lidar.min_range_m) plus the file's clearance; build it the way
    # rm_safety does rather than reading a key that no longer exists.
    from rm_safety.robot_profile import DEFAULT_ROBOTS_DIR, load_robot_profile, resolve_profile_path
    from rm_safety.safety_logic import Limits
    mount = load_robot_profile(resolve_profile_path(DEFAULT_ROBOTS_DIR)).mount
    slow = Limits.from_dict(limits, mount).estop.slow_zone_m
    reversing = vx < 0.0
    where = "behind" if reversing else "ahead"
    runway = node.min_range(math.pi if reversing else 0.0)
    # A reverse leg gets no slow-zone scaling (the arcs face forward), so the
    # clearance it needs is the travel plus a margin -- but for the same reason
    # nothing downstream will stop it, so say so out loud.
    if kind != "straight":
        need = slow
    elif reversing:
        need = abs(naive_m) + RUNWAY_MARGIN_M
    else:
        need = slow + abs(naive_m) + RUNWAY_MARGIN_M
    if reversing:
        print("  NOTE: reversing. rm_safety has forward and lateral e-stop arcs "
              "only —\n        nothing will stop this leg for an obstacle "
              "behind the robot. Watch it.")
    if runway is None:
        print(f"!! no /scan — cannot check the runway {where}. Is the lidar up?")
        if input("   type YES to run anyway: ").strip() != "YES":
            return None
        runway = float("nan")
    else:
        print(f"  runway {where}: {runway:.2f} m   (need > {need:.2f} m)")
        if runway < need:
            print(f"  !! TOO SHORT — reposition the robot."
                  + ("" if reversing else " Inside the slow zone the command is "
                     "scaled\n     down, which would corrupt the measurement."))
            if input("   type YES to run anyway: ").strip() != "YES":
                return None

    print("\n  THE ROBOT IS ABOUT TO MOVE. There is no physical E-stop.")
    print("  Have this typed and ready in another terminal:")
    print("      ros2 service call /safety/kill std_srvs/srv/Trigger")
    if input("  Enter to go, or 's' to skip this leg: ").strip().lower() == "s":
        return None

    node.reset_run(vx, wz)

    try:
        spin_for(node, SETTLE_SECS)            # zeros, so arming finds a source
        print("-- arming --")
        if not node.call(node.reset_cli, "/safety/reset"):
            return None
        spin_for(node, 1.0)
        st = node.status
        if st is None:
            print("!! no /safety/status — is safety_node running?")
            return None
        if st.state == 3:
            print(f"!! still KILLED after reset, reason={st.reason!r} — skipping leg")
            return None
        print(f"   armed: {_STATE.get(st.state)}  reason={st.reason!r}")

        print("-- driving --")
        node.recording = True
        spin_for(node, secs)
        node.vx = node.wz = 0.0                # ramp down, still recording
        spin_for(node, FLOOD_SECS)
        node.recording = False
    finally:
        # The only stop path, taken on every exit including Ctrl-C.
        try:
            node.vx = node.wz = 0.0
            node.recording = False
            print("-- stopping: flooding zeros --")
            spin_for(node, 1.5)
            node.call(node.kill_cli, "/safety/kill")
            spin_for(node, 0.5)
            print("-- killed --")
        except Exception as exc:               # noqa: BLE001 — never mask the stop
            print(f"!! stop path error: {exc}")

    # ---- what was actually demanded --------------------------------------
    cmd_m, cmd_rad = integrate(node.cmd_track)
    cmd_ref = cmd_m if kind == "straight" else math.degrees(cmd_rad)
    o_fwd, o_lat, o_yaw = odom_delta(node.odom_track)
    odom_ref = o_fwd if kind == "straight" else o_yaw
    unit = "m" if kind == "straight" else "deg"

    safety_ok = set(node.reasons) <= {"ok"}
    print(f"\n  /cmd_vel integral (REFERENCE): {cmd_ref:+.3f} {unit}")
    print(f"  /odom             (under test): {odom_ref:+.3f} {unit}"
          + (f"   lateral {o_lat:+.3f} m" if kind == "straight" else ""))
    if node.reasons:
        print("  safety reasons: "
              + ", ".join(f"{k}x{v}" for k, v in node.reasons.items()))
        if not safety_ok:
            print("  !! NOT 'ok' throughout — this leg was clamped and is NOT a "
                  "clean accuracy number.")

    flags = []
    if node.samples:
        print(f"\n  {'wheel':6}{'target':>9}{'measured':>10}{'pwm':>6}  flags")
        tail = node.samples[int(len(node.samples) * 0.4):] or node.samples
        for i, w in enumerate(WHEELS):
            tgt = statistics.mean(m.target_speed_mps[i] for m in tail)
            mes = statistics.mean(m.measured_speed_mps[i] for m in tail)
            pwm = statistics.mean(abs(m.applied_pwm[i]) for m in tail)
            f = [n for n, arr in (("SIGN", "sign_fault"), ("stall", "stalled"),
                                  ("sat", "saturated"))
                 if any(getattr(m, arr)[i] for m in node.samples)]
            flags += [f"{w}:{x}" for x in f]
            print(f"  {w:6}{tgt:9.3f}{mes:10.3f}{pwm:6.0f}  {' '.join(f)}")
    else:
        print("  !! no /base/debug samples — is rm_base_node running?")

    # ---- the ruler --------------------------------------------------------
    # Ask for a MAGNITUDE and apply the commanded sign here. Asking the
    # operator to sign it by hand produced a wrong-signed entry twice in the
    # first session -- a reverse leg recorded +0.69 against a -0.72 command
    # reads as ratio -0.959, which is not a small error but a robot that drove
    # the wrong way. Reading a tape gives a magnitude; the direction is already
    # known from what was commanded, so the tool should not ask twice.
    print("\n  MEASURE IT. Tape measure / protractor — not the numbers above.")
    unit_name = "distance (m)" if kind == "straight" else "yaw (deg)"
    if kind == "straight":
        went = "FORWARD" if cmd_ref >= 0 else "BACKWARD"
    else:
        went = "LEFT/CCW" if cmd_ref >= 0 else "RIGHT/CW"
    magnitude = ask(f"  achieved {unit_name} — how far it went, unsigned "
                    f"(commanded {went}), blank to skip: ")
    measured = None
    if magnitude is not None:
        if magnitude < 0:
            print("     (negative entered — taking the magnitude; the "
                  "commanded direction is used)")
        measured = math.copysign(abs(magnitude), cmd_ref)
        print(f"     recorded as {measured:+.3f} "
              f"{'m' if kind == 'straight' else 'deg'}")
    lateral = None
    if kind == "straight" and measured is not None:
        lateral = ask("  sideways drift off the start line (m, blank if none): ")

    ratio = (measured / cmd_ref) if (measured is not None and abs(cmd_ref) > 1e-6) else None
    if ratio is not None:
        print(f"\n  >>> achieved / commanded = {ratio:.3f}"
              f"   ({(ratio - 1.0) * 100:+.1f}%)")
        if abs(odom_ref) > 1e-6 and measured is not None:
            print(f"  >>> odom / achieved      = {odom_ref / measured:.3f}"
                  f"   (1.000 would mean /odom is honest)")

    return {
        "kind": kind,
        "commanded_val": val,
        "secs": secs,
        "naive": round(naive_m if kind == "straight" else naive_deg, 3),
        "cmd_integral": round(cmd_ref, 4),
        "odom": round(odom_ref, 4),
        "measured": "" if measured is None else measured,
        "ratio": "" if ratio is None else round(ratio, 4),
        "lateral": "" if lateral is None else lateral,
        "safety_ok": safety_ok,
        "runway_m": round(runway, 2),
        "lever_arm": lever_arm,
        "wheel_flags": " ".join(sorted(set(flags))),
    }


def summarise(rows: list[dict]) -> None:
    print("\n" + "=" * 78)
    print("VELOCITY ACCURACY SUMMARY")
    print("=" * 78)
    print(f"{'leg':>16} {'cmd(int)':>10} {'achieved':>10} {'ratio':>7} "
          f"{'odom':>9} {'sfty':>5} {'lever':>6}  flags")
    for r in rows:
        unit = "m" if r["kind"] == "straight" else "d"
        name = f"{r['kind'][:4]} {float(r['commanded_val']):+.2f} {float(r['secs']):.0f}s"
        ref = f"{float(r['cmd_integral']):+.3f}{unit}"
        meas = f"{float(r['measured']):+.3f}{unit}" if r["measured"] != "" else "--"
        ratio = f"{float(r['ratio']):.3f}" if r["ratio"] != "" else "--"
        odom = f"{float(r['odom']):+.3f}"
        sfty = "ok" if str(r["safety_ok"]) == "True" else "CLAMP"
        lever = r.get("lever_arm") or "--"
        print(f"{name:>16} {ref:>10} {meas:>10} {ratio:>7} {odom:>9} {sfty:>5}"
              f" {lever:>6}  {r['wheel_flags']}")

    # Analyse ONE lever_arm at a time. The workflow is measure -> change the
    # constant -> measure again, all appending to the same sheet, so averaging
    # every spin leg reports a "spread" that is really the edit.
    #
    # Only SPIN legs are filtered: lever_arm_m scales the wheel speeds a yaw
    # command asks for and has no effect on straight-line driving, so straight
    # legs stay valid across the change and are all kept. (A change to
    # dead_pwm/gain WOULD invalidate them, and is not tracked here — start a
    # fresh --out after one.)
    levers = [r.get("lever_arm") or "" for r in rows if r["kind"] == "spin"]
    current = next((l for l in reversed(levers) if l != ""), "")
    older = {l for l in levers if l != current}
    if older:
        shown = ", ".join(sorted(x or "untracked" for x in older | {current}))
        print(f"\n  NOTE: spin legs here span more than one lever_arm ({shown}). "
              f"Analysing only\n        the most recent, "
              f"{current or 'untracked'} — earlier spins are listed above but "
              f"excluded.\n        Straight legs are unaffected by lever_arm "
              f"and are all kept.")
    rows = [r for r in rows
            if r["kind"] != "spin" or (r.get("lever_arm") or "") == current]

    # Group by DIRECTION as well as axis. The calibration is per-direction
    # (dead_pwm_rev / gain_pwm_per_mps_rev), and the robot has a measured
    # forward/reverse and CW/CCW asymmetry, so pooling the two directions turns
    # a direction difference into a fake "varies with speed" verdict — which is
    # exactly what it did to a 0.959 reverse leg sitting beside three accurate
    # forward ones.
    def _clean(kind: str, sign: int) -> list[float]:
        return [float(r["ratio"]) for r in rows
                if r["kind"] == kind and r["ratio"] != ""
                and str(r["safety_ok"]) == "True"
                and (float(r["commanded_val"]) >= 0) == (sign > 0)]

    clean = {(k, s): _clean(k, s)
             for k in ("straight", "spin") for s in (1, -1)}

    # Whether the per-wheel PWM model is exonerated. The straight legs drive
    # the wheels over the same speed range a spin does (wz * lever_arm_m), so
    # if they are accurate the wheels demonstrably CAN hit those speeds and no
    # wheel-level explanation of a yaw error survives. Judged on the FORWARD
    # straights: they are the group with several speeds in it.
    fwd = clean[("straight", 1)]
    wheels_proven = (
        len(fwd) >= 2
        and abs(statistics.mean(fwd) - 1.0) <= 0.05
        and max(fwd) - min(fwd) <= 0.05
    )

    for (kind, sign), label in (
        (("straight", 1), "LINEAR  vx forward"),
        (("straight", -1), "LINEAR  vx reverse"),
        (("spin", 1), "ANGULAR wz left/CCW"),
        (("spin", -1), "ANGULAR wz right/CW"),
    ):
        ratios = clean[(kind, sign)]
        if not ratios:
            continue
        mean = statistics.mean(ratios)
        spread = max(ratios) - min(ratios)
        print(f"\n{label}: {len(ratios)} clean leg{'s' if len(ratios) > 1 else ''}, "
              f"achieved/commanded mean {mean:.3f}"
              + (f", spread {spread:.3f}" if len(ratios) > 1 else ""))
        # A consistent ratio is a SCALE error and is one constant away from
        # fixed. A ratio that varies with speed is a modelling error, and WHICH
        # model is wrong depends on whether the straights already cleared the
        # wheels — that cross-check is why both axes are summarised together.
        # With only one leg there is no speed dependence to see either way.
        if len(ratios) < 2:
            verdict = ("within 5%" if abs(mean - 1.0) <= 0.05
                       else f"{(mean - 1.0) * 100:+.1f}% off")
            print(f"  -> {verdict}, but ONE leg only — a single speed cannot "
                  f"separate a scale\n     error from a dead-band. Add a second "
                  f"speed before trusting it.")
        elif abs(mean - 1.0) <= 0.05 and spread <= 0.05:
            print("  -> accurate: within 5% at every speed tested.")
        elif spread <= 0.05:
            factor = ("wheel_circumference_m / counts_per_rev" if kind == "straight"
                      else "lever_arm_m")
            print(f"  -> CONSISTENT {(mean - 1.0) * 100:+.1f}% error at all speeds: a "
                  f"pure SCALE error.\n     Scale {factor} by {mean:.3f}.")
        elif kind == "spin" and wheels_proven:
            # Do NOT send the operator to refit dead_pwm/gain here. The
            # straights just proved those are right across this very speed
            # range, so the loss is in the rotation itself: mecanum rollers
            # scrubbing sideways. lever_arm_m is the term that absorbs scrub —
            # but it is ONE CONSTANT and the loss is not constant, so it can
            # only be made exact at one wz. Fit it where the robot will spin.
            print("  -> ratio VARIES WITH SPEED, but the straight legs were "
                  "accurate over the\n     same wheel-speed range — so the "
                  "wheels are fine and this is NOT a\n     dead_pwm/gain "
                  "mis-split. It is rotational SCRUB, which lever_arm_m "
                  "absorbs.")
            print("     lever_arm_m implied per leg (one constant cannot fit "
                  "them all):")
            for r in rows:
                if r["kind"] != "spin" or r["ratio"] == "":
                    continue
                if str(r["safety_ok"]) != "True":
                    continue
                if (float(r["commanded_val"]) >= 0) != (sign > 0):
                    continue
                print(f"       wz {float(r['commanded_val']):+.2f}  "
                      f"scale current lever_arm_m by {1.0 / float(r['ratio']):.3f}")
            print("     Pick the wz the robot actually spins at and fit there; "
                  "expect the\n     other end of the range to stay off.")
        else:
            print("  -> ratio VARIES WITH SPEED (spread > 5%): dead_pwm and gain "
                  "are mis-split.\n     Refit with fit_reverse.py from two "
                  "STRAIGHT legs — never scale a constant to paper over this.")

    if clean[("straight", 1)] and not clean[("straight", -1)]:
        print("\nNOTE: no REVERSE straight leg was measured. The calibration is "
              "per-direction\n      (dead_pwm_rev / gain_pwm_per_mps_rev), so "
              "forward accuracy says nothing\n      about reverse.")


def load_rows(path: str) -> list[dict]:
    if not os.path.exists(path):
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def append_row(path: str, row: dict) -> None:
    new = not os.path.exists(path)
    with open(path, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--legs", choices=["all", "straight", "spin"], default="all")
    ap.add_argument("--vx", type=float, help="single ad-hoc straight leg")
    ap.add_argument("--wz", type=float, help="single ad-hoc spin leg")
    ap.add_argument("--secs", type=float, default=5.0, help="duration of an ad-hoc leg")
    ap.add_argument("--out", default="velocity_accuracy.csv")
    ap.add_argument("--calibration",
                    default=os.path.expanduser(
                        "~/ncp_ws/config/base_calibration.yaml"))
    ap.add_argument("--limits",
                    default=os.path.expanduser("~/ncp_ws/config/safety_limits.yaml"))
    ap.add_argument("--summary", metavar="CSV",
                    help="re-print the summary for a finished sheet and exit")
    args = ap.parse_args()

    if args.summary:
        rows = load_rows(args.summary)
        if not rows:
            print(f"no rows in {args.summary}")
            return 1
        summarise(rows)
        return 0

    _require_ros()

    with open(args.limits) as fh:              # no magic numbers: the zones the
        limits = yaml.safe_load(fh)            # runway check needs come from here
    try:
        with open(args.calibration) as fh:
            lever_arm = yaml.safe_load(fh).get("lever_arm_m", "")
    except OSError:                            # not fatal — it is only a label
        lever_arm = ""

    if args.vx is not None:
        legs = [("straight", args.vx, args.secs)]
    elif args.wz is not None:
        legs = [("spin", args.wz, args.secs)]
    else:
        legs = [l for l in DEFAULT_LEGS if args.legs in ("all", l[0])]

    print(__doc__.split("\n")[1])
    print(f"\n{len(legs)} legs, results appended to {args.out}")
    print("Bring the robot back to the start line between legs; each leg is "
          "measured on its own,\nso repositioning between them is expected.")

    rclpy.init()
    node = Leg()
    rows: list[dict] = []
    try:
        for kind, val, secs in legs:
            row = run_leg(node, kind, val, secs, limits, lever_arm)
            if row is None:
                print("   (leg skipped)")
                continue
            append_row(args.out, row)
            rows.append(row)
    except KeyboardInterrupt:
        print("\n-- interrupted; the current leg's stop path already ran --")
    finally:
        try:
            node.call(node.kill_cli, "/safety/kill")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if rows:
        summarise(load_rows(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
