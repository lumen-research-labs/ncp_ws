#!/usr/bin/env python3
"""
watch_stack.py — one-line live view of the safety + base stack.

What it does:
    Subscribes to /safety/status, /base/debug and /cmd_vel and rewrites a
    single terminal line at 5 Hz. Built for wheels-down testing, where
    `ros2 topic echo` scrolls faster than anyone can read and the operator
    has one hand on the kill switch.

    Read-only. Publishes NOTHING — it can never move the robot, and it is
    safe to leave running across a kill/reset cycle.

Subscribes:
    /safety/status  [rm_interfaces/SafetyStatus]  FSM state + reason
    /base/debug     [rm_interfaces/BaseDebug]     per-wheel PWM / speed
    /cmd_vel        [geometry_msgs/Twist]         what rm_safety emitted

Publishes:
    nothing.

The line reads:

    KILLED  min=0.42 | cmd vx=+0.100 wz=+0.000 | pwm[ 15  15  59  21] | ...
    ^state  ^lidar     ^what rm_safety let through  ^applied per wheel

`min=` reads `inf` whenever the state is KILLED. That is NOT a dead lidar:
evaluate() takes the KILLED branch first and returns before it processes the
scan, so a real min_range only appears after /safety/reset. Confirm the lidar
separately with `ros2 topic hz /scan` if in doubt.

Flags at the end appear only when set, so an empty tail is a clean run:
    DRY     dry_run — nothing is being written to the motors
    STALE   no /cmd_vel inside rm_base's watchdog
    SAT:..  wheels at their PWM ceiling
    STALL:. wheels commanded but reporting no motion (front-right expected)
    SIGN:.  ENCODER POLARITY FAULT — stop the test

Failure mode:
    Read-only and best-effort. A topic that never publishes shows as "--"
    rather than blocking, so a missing node is visible instead of silent.
"""
from __future__ import annotations

import sys

import rclpy
import rclpy.executors
import rclpy.node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from rm_interfaces.msg import BaseDebug, SafetyStatus

_STATE = {0: "OK    ", 1: "SLOW  ", 2: "STOP  ", 3: "KILLED"}
_WHEEL_INITIALS = "FL BL FR BR".split()


class Watcher(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("watch_stack")
        self._status: SafetyStatus | None = None
        self._debug: BaseDebug | None = None
        self._cmd: Twist | None = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # /safety/status and /base/debug are published RELIABLE; /cmd_vel is
        # subscribed BEST_EFFORT so this tool can never be the reason a
        # publisher blocks.
        self.create_subscription(SafetyStatus, "/safety/status", self._on_status, 10)
        self.create_subscription(BaseDebug, "/base/debug", self._on_debug, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, sensor_qos)
        self.create_timer(0.2, self._render)

    def _on_status(self, msg: SafetyStatus) -> None:
        self._status = msg

    def _on_debug(self, msg: BaseDebug) -> None:
        self._debug = msg

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = msg

    def _render(self) -> None:
        parts: list[str] = []

        if self._status is None:
            parts.append("safety:--    ")
        else:
            state = _STATE.get(self._status.state, f"?{self._status.state}")
            rng = self._status.min_range
            rng_s = f"{rng:5.2f}" if rng == rng and rng != float("inf") else "  inf"
            parts.append(f"{state} min={rng_s} {self._status.reason[:18]:18s}")

        if self._cmd is None:
            parts.append("cmd --")
        else:
            parts.append(
                f"cmd vx={self._cmd.linear.x:+.3f} wz={self._cmd.angular.z:+.3f}"
            )

        flags: list[str] = []
        if self._debug is None:
            parts.append("base --")
        else:
            d = self._debug
            pwm = " ".join(f"{v:4d}" for v in d.applied_pwm)
            mps = " ".join(f"{v:+.2f}" for v in d.measured_speed_mps)
            parts.append(f"pwm[{pwm}] mps[{mps}]")
            if d.dry_run:
                flags.append("DRY")
            if d.command_stale:
                flags.append("STALE")
            for name, arr in (
                ("SAT", d.saturated), ("STALL", d.stalled), ("SIGN", d.sign_fault)
            ):
                hit = [_WHEEL_INITIALS[i] for i, v in enumerate(arr) if v]
                if hit:
                    flags.append(f"{name}:{','.join(hit)}")

        line = " | ".join(parts)
        if flags:
            line += "  << " + " ".join(flags)
        sys.stdout.write("\r\033[K" + line[:200])
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = Watcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Ctrl-C and SIGTERM are the normal ways this tool ends.
        sys.stdout.write("\n")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
