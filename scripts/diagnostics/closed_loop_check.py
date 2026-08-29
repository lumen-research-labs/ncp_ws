#!/usr/bin/env python3
"""
closed_loop_check.py — the wheels-UP acceptance test for rm_base closed loop.

What it does:
    Arms rm_safety, commands one small velocity for a fixed duration while
    recording /base/debug, then stops and re-kills. Prints a per-wheel verdict
    on the three closed-loop guards.

    WHEELS UP ONLY, operator present. It commands real motion.

    Everything is one process so that stopping cannot be forgotten: the
    command publisher, the zero flood, and the /safety/kill call all live in
    the same try/finally. There is no path out of main() -- normal end,
    Ctrl-C, or an exception -- that does not flood zeros and re-kill.

Why it exists rather than `ros2 topic pub`:
    A `--once` zero does not stop this robot (dropped, or sent before the
    subscription matched), and a bare `topic pub` in another terminal leaves
    stopping to a human under time pressure. It also samples /base/debug at
    full rate during the run, which `ros2 topic echo` cannot do without
    scrolling past the moment of interest.

Subscribes:
    /base/debug     [rm_interfaces/BaseDebug]
Publishes:
    /policy/cmd_vel [geometry_msgs/Twist]   -- an INPUT to rm_safety, never
                                               /cmd_vel, which rm_safety owns.
Services called:
    /safety/reset, /safety/kill  [std_srvs/Trigger]

Usage:
    closed_loop_check.py [--vx 0.05] [--wz 0.0] [--secs 4.0]

What to look for:
    SIGN    must be clean on all four. A latched sign_fault means the encoder
            polarity assumption is wrong for that wheel and closed loop would
            drive it the wrong way -- STOP and go back to open_loop.
    STALL   expected on front-right (its encoder only reads once turning fast).
    SAT     expected on front-right at higher commands (weak motor, high
            dead_pwm). On any other wheel it means the feedforward is far off.
    measured vs target speed: closed loop should pull measured toward target.

Failure mode:
    Fails closed. Any exception aborts into the same stop path. If rm_safety
    never arms (e.g. /odom stale, obstacle in the stop zone) the run reports
    the blocking reason and commands nothing.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

import math

import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from rm_interfaces.msg import BaseDebug, SafetyStatus

WHEELS = ["FL", "BL", "FR", "BR"]
RATE_HZ = 20.0
_STATE = {0: "OK", 1: "SLOW", 2: "STOPPED", 3: "KILLED"}


class Check(rclpy.node.Node):
    def __init__(self, vx: float, wz: float) -> None:
        super().__init__("closed_loop_check")
        self.vx = vx
        self.wz = wz
        self.samples: list[BaseDebug] = []
        self.status: SafetyStatus | None = None
        self.recording = False
        # Odometry track. NOT ground truth -- /odom is integrated from
        # rm_base's reconstructed /vel_raw, which is exactly what the tape
        # measure is here to check (BUILD-PLAN item 4). Recorded so the two
        # numbers can be compared.
        self.odom_track: list[tuple[float, float, float]] = []
        self.reasons: dict[str, int] = {}

        self.pub = self.create_publisher(Twist, "/policy/cmd_vel", 10)
        self.create_subscription(BaseDebug, "/base/debug", self._on_debug, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_status, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.kill_cli = self.create_client(Trigger, "/safety/kill")
        self.reset_cli = self.create_client(Trigger, "/safety/reset")
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _on_debug(self, msg: BaseDebug) -> None:
        if self.recording:
            self.samples.append(msg)

    def _on_status(self, msg: SafetyStatus) -> None:
        self.status = msg
        if self.recording:
            self.reasons[msg.reason] = self.reasons.get(msg.reason, 0) + 1

    def _on_odom(self, msg: Odometry) -> None:
        if not self.recording:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        # two_d_mode EKF: yaw from the z/w components only.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_track.append((p.x, p.y, yaw))

    def _tick(self) -> None:
        t = Twist()
        if self.recording:
            t.linear.x = self.vx
            t.angular.z = self.wz
        self.pub.publish(t)

    def call(self, cli, name: str) -> bool:
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"!! {name} service not available")
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None


def spin_for(node: rclpy.node.Node, secs: float) -> None:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.05)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--secs", type=float, default=4.0)
    args = ap.parse_args()

    rclpy.init()
    node = Check(args.vx, args.wz)

    try:
        print(f"== WHEELS UP CHECK  vx={args.vx} wz={args.wz} for {args.secs}s ==")
        # Publish zeros first so rm_safety has a fresh command source before
        # it is armed -- otherwise it arms straight into command_timeout.
        spin_for(node, 1.5)

        print("-- arming (/safety/reset) --")
        if not node.call(node.reset_cli, "/safety/reset"):
            return 1
        spin_for(node, 1.0)

        st = node.status
        if st is None:
            print("!! no /safety/status — is safety_node running?")
            return 1
        if st.state == 3:
            print(f"!! still KILLED after reset, reason={st.reason!r} — aborting")
            return 1
        print(f"   armed: state={_STATE.get(st.state)} reason={st.reason!r}")

        print("-- commanding --")
        node.recording = True
        spin_for(node, args.secs)
        node.recording = False

    finally:
        # The only stop path, taken on every exit including Ctrl-C.
        try:
            print("\n-- stopping: flooding zeros --")
            spin_for(node, 1.5)          # _tick now publishes zeros
            node.call(node.kill_cli, "/safety/kill")
            spin_for(node, 0.5)
            print("-- killed --")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    # ---- report -----------------------------------------------------------
    s = node.samples
    if not s:
        print("!! no /base/debug samples — is rm_base_node running?")
        return 1

    # Report the STEADY-STATE tail, not the whole run: the integral starts at
    # zero and spends the first seconds winding toward the feedforward error,
    # so a whole-run mean understates it and would under-correct the
    # calibration. The tail is the converged estimate worth folding back.
    tail = s[int(len(s) * 0.6):] or s

    print(f"\n==== {len(s)} samples ({len(tail)} in steady-state tail) ====")
    print(f"{'wheel':6} {'target':>8} {'measured':>9} {'ff':>7} {'integ':>7} "
          f"{'pwm':>5} {'fb%':>5}  flags")
    bad = []
    for i, w in enumerate(WHEELS):
        tgt = statistics.mean(m.target_speed_mps[i] for m in tail)
        mes = statistics.mean(m.measured_speed_mps[i] for m in tail)
        ff = statistics.mean(m.feedforward_pwm[i] for m in tail)
        integ = statistics.mean(m.integral_pwm[i] for m in tail)
        pwm = statistics.mean(m.applied_pwm[i] for m in tail)
        flags = []
        if any(m.sign_fault[i] for m in s):
            flags.append("SIGN-FAULT")
            bad.append(w)
        if any(m.stalled[i] for m in s):
            flags.append("stalled")
        if any(m.saturated[i] for m in s):
            flags.append("saturated")
        # Percentage of ticks with a FRESH encoder reading. Low is expected --
        # get_motor_encoder returns a value cached by the board's auto-report
        # thread, so polling at 20 Hz repeats samples. Near 0% means the loop
        # is running on feedforward alone.
        fb = 100.0 * sum(1 for m in s if m.feedback_valid[i]) / len(s)
        if fb < 100.0:
            flags.append(f"fb-gaps")
        print(f"{w:6} {tgt:8.3f} {mes:9.3f} {ff:7.1f} {integ:7.1f} {pwm:5.0f} "
              f"{fb:5.0f}  {' '.join(flags)}")

    # ---- odometry + safety summary ---------------------------------------
    if node.odom_track:
        x0, y0, yaw0 = node.odom_track[0]
        x1, y1, yaw1 = node.odom_track[-1]
        # Rotate the displacement into the body frame AT RUN START. The odom
        # frame's origin and orientation are fixed when the EKF starts, so
        # after any manual reposition of the robot it no longer lines up with
        # the robot's heading -- and a raw odom-frame dx/dy then splits pure
        # forward motion across both axes, which reads as enormous "lateral
        # drift" that never happened. Yaw change is frame-independent and is
        # the trustworthy veer signal; this makes forward/lateral one too.
        odx, ody = x1 - x0, y1 - y0
        dx = odx * math.cos(-yaw0) - ody * math.sin(-yaw0)
        dy = odx * math.sin(-yaw0) + ody * math.cos(-yaw0)
        straight = math.hypot(dx, dy)
        path = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(node.odom_track, node.odom_track[1:])
        )
        dyaw = math.degrees(math.atan2(
            math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)
        ))
        print(f"\n---- /odom (NOT ground truth -- compare to tape) ----")
        print(f"  displacement : {straight:.3f} m  "
              f"(body-frame fwd={dx:+.3f} lat={dy:+.3f})")
        print(f"  path length  : {path:.3f} m")
        print(f"  yaw change   : {dyaw:+.1f} deg")
        print(f"  commanded    : {args.vx * args.secs:+.3f} m , "
              f"{math.degrees(args.wz * args.secs):+.1f} deg")
        if abs(args.wz) < 1e-6 and straight > 1e-3:
            print(f"  LATERAL DRIFT: dy/dx = {dy / dx:+.3f}" if abs(dx) > 1e-3
                  else "  LATERAL DRIFT: dx ~ 0")

    if node.reasons:
        print(f"\n  safety reasons during run: "
              f"{', '.join(f'{k}x{v}' for k, v in node.reasons.items())}")
        if set(node.reasons) - {"ok"}:
            print("  !! not 'ok' for the whole run -- the robot was being "
                  "clamped, so distance is NOT a clean calibration number.")

    # Feedforward correction implied by the converged integral. The integral
    # exists to absorb feedforward error, so folding it in is what stops the
    # loop having to rediscover the same offset on every start -- and the
    # start transient is where a straight line is lost.
    print("\n---- implied feedforward correction (fold into dead_pwm) ----")
    for i, w in enumerate(WHEELS):
        ff = statistics.mean(m.feedforward_pwm[i] for m in tail)
        integ = statistics.mean(m.integral_pwm[i] for m in tail)
        print(f"  {w:3}  ff {ff:6.1f}  {integ:+6.1f}  ->  {ff + integ:6.1f}"
              f"   (dead_pwm shift {integ:+.1f})")

    print()
    if bad:
        print(f"!! SIGN FAULT on {', '.join(bad)} — encoder polarity is wrong "
              f"for those wheels. Do NOT run closed_loop on the floor.")
        return 1
    print("OK: no sign faults. Check measured is tracking target above.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
