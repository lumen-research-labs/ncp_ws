#!/usr/bin/env python3
"""
fit_reverse.py — measure each wheel's REVERSE dead_pwm / gain.

What it does:
    Drives the base backwards at two speeds, records the steady-state applied
    PWM and the speed each wheel actually reached, and solves the two-point
    fit per wheel:

        |pwm| = dead_pwm_rev + gain_pwm_per_mps_rev * |measured_speed|

    Prints the YAML to paste into config/base_calibration.yaml.

Why it exists:
    The PWM model was symmetric until 2026-08-29, and on the floor both LEFT
    wheels delivered only 23-32% of their commanded speed while running in
    reverse during a spin (the right pair, going forward, hit 99-106%). A
    spin drives one side backwards, so a wheel that under-performs in reverse
    makes the robot arc instead of turning on the spot. The closed loop
    cannot absorb it: the integrator is reset every time the target crosses
    zero, which is every direction flip.

    Fitting against MEASURED speed rather than the target is the point — it
    maps "this PWM produced this actual speed", which is what the model needs
    to invert.

!!! READ BEFORE RUNNING ON THE FLOOR !!!
    rm_safety monitors a FORWARD arc plus lateral arcs. The rear is
    chassis-occluded and NOT monitored — driving backwards, the robot is
    blind, and neither the slow zone nor the stop zone will fire for anything
    behind it. Clear the space behind the robot and watch it yourself.

    Wheels-UP is the safe alternative and still captures the dead-band
    asymmetry (static friction), which is the term the closed loop cannot
    fix. It will NOT capture the gain under load; the loop trims that.

Subscribes:  /base/debug, /safety/status
Publishes:   /policy/cmd_vel  (an INPUT to rm_safety, never /cmd_vel)
Services:    /safety/reset, /safety/kill

Usage:
    fit_reverse.py [--slow 0.06] [--fast 0.12] [--secs 6]

Failure mode:
    Fails closed. Publisher, zero-flood and /safety/kill live in one
    try/finally, so no exit path — normal, Ctrl-C or exception — leaves the
    robot moving. A wheel whose two points are too close together to invert
    is reported as unfittable rather than given a fabricated number.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from rm_interfaces.msg import BaseDebug, SafetyStatus

WHEELS = ["front_left", "back_left", "front_right", "back_right"]
SHORT = ["FL", "BL", "FR", "BR"]
RATE_HZ = 20.0


class Fit(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("fit_reverse")
        self.vx = 0.0
        self.recording = False
        self.samples: list[BaseDebug] = []
        self.status: SafetyStatus | None = None
        self.pub = self.create_publisher(Twist, "/policy/cmd_vel", 10)
        self.create_subscription(BaseDebug, "/base/debug", self._d, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._s, 10)
        self.kill = self.create_client(Trigger, "/safety/kill")
        self.reset = self.create_client(Trigger, "/safety/reset")
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _d(self, m: BaseDebug) -> None:
        if self.recording:
            self.samples.append(m)

    def _s(self, m: SafetyStatus) -> None:
        self.status = m

    def _tick(self) -> None:
        t = Twist()
        t.linear.x = self.vx
        self.pub.publish(t)

    def call(self, cli, name: str) -> bool:
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"!! {name} unavailable")
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None


def spin_for(node, secs: float) -> None:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def run_leg(node: Fit, vx: float, secs: float) -> list[tuple[float, float]]:
    """One reverse leg. Returns (|measured speed|, |applied pwm|) per wheel."""
    print(f"\n-- leg: vx={vx:+.3f} for {secs}s --")
    node.vx = 0.0
    node.samples = []
    spin_for(node, 1.5)                      # fresh zeros before arming
    if not node.call(node.reset, "/safety/reset"):
        raise RuntimeError("could not arm")
    spin_for(node, 1.0)
    if node.status is None or node.status.state == 3:
        raise RuntimeError(
            f"still KILLED after reset "
            f"(reason={node.status.reason if node.status else 'no status'})"
        )
    node.vx = vx
    spin_for(node, 1.5)                      # let it reach steady state
    node.recording = True
    spin_for(node, secs)
    node.recording = False
    node.vx = 0.0
    spin_for(node, 1.5)
    node.call(node.kill, "/safety/kill")
    spin_for(node, 0.5)

    s = node.samples
    if not s:
        raise RuntimeError("no /base/debug samples")
    tail = s[int(len(s) * 0.4):] or s        # steady state only
    out = []
    for i in range(4):
        mps = abs(statistics.mean(m.measured_speed_mps[i] for m in tail))
        pwm = abs(statistics.mean(m.applied_pwm[i] for m in tail))
        out.append((mps, pwm))
        print(f"   {SHORT[i]:3}  measured {mps:.4f} m/s   applied pwm {pwm:5.1f}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--slow", type=float, default=0.06)
    ap.add_argument("--fast", type=float, default=0.12)
    ap.add_argument("--secs", type=float, default=6.0)
    args = ap.parse_args()

    print(__doc__.split("Usage:")[0].split("!!! READ")[1][:400])
    print("\nThe robot will drive BACKWARDS. Ctrl-C aborts.\n")

    rclpy.init()
    node = Fit()
    try:
        slow = run_leg(node, -abs(args.slow), args.secs)
        fast = run_leg(node, -abs(args.fast), args.secs)
    except Exception as exc:  # noqa: BLE001
        print(f"\n!! aborted: {exc}")
        slow = fast = None
    finally:
        node.vx = 0.0
        print("\n-- stopping --")
        spin_for(node, 1.5)
        node.call(node.kill, "/safety/kill")
        spin_for(node, 0.5)
        stop_ok = True
        node.destroy_node()
        rclpy.shutdown()

    if not slow or not fast:
        return 1

    print("\n" + "=" * 62)
    print("Paste into config/base_calibration.yaml under each wheel:")
    print("=" * 62)
    for i, name in enumerate(WHEELS):
        v1, p1 = slow[i]
        v2, p2 = fast[i]
        if abs(v2 - v1) < 0.005:
            print(f"  # {name}: UNFITTABLE — the two legs measured "
                  f"{v1:.4f} and {v2:.4f} m/s, too close to invert.")
            print(f"  #   Widen --slow/--fast, or this wheel barely moved.")
            continue
        gain = (p2 - p1) / (v2 - v1)
        dead = p1 - gain * v1
        if gain <= 0.0:
            print(f"  # {name}: UNFITTABLE — negative slope "
                  f"({p1:.1f}@{v1:.3f} -> {p2:.1f}@{v2:.3f}).")
            continue
        print(f"  {name}:")
        print(f"    dead_pwm_rev: {max(dead, 0.0):.1f}")
        print(f"    gain_pwm_per_mps_rev: {gain:.1f}")
    print("=" * 62)
    print("Omit a wheel entirely to keep it symmetric (forward values).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
