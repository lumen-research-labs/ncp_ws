#!/usr/bin/env python3
"""
fr_threshold.py — find the PWM at which ONE motor actually starts turning.

WHEELS UP. Operator present. Drives one motor directly via Rosmaster_Lib,
bypassing rm_base entirely, so nothing else can be dragging the wheel — which
is the whole point: on the floor a dead wheel is turned by the other three and
its encoder reports the robot's motion, which reads exactly like a working
motor.

    python3 fr_threshold.py [--motor 3] [--from 30] [--to 80] [--step 5]

Motor numbering is the MEASURED map, not Yahboom's sample order:
    M1 = front-left, M2 = back-left, M3 = front-right, M4 = back-right
set_motor(+) drives a wheel FORWARD; a forward wheel makes its encoder count
DOWN, which this script corrects for.

For each PWM step it drives forward briefly, then reverse, and reports the
encoder delta for that motor. The threshold is the first step with motion
clearly above the noise floor.

    rm_base_node / driver_node / rosmaster_main must NOT hold the serial port.
    Stop the stack first (scripts/bench_down.sh, or kill rm_base_node).

Fail-closed: every exit path — normal, Ctrl-C, or exception — floods
set_motor(0,0,0,0) before releasing the port.

HEAT: a stalled motor at high PWM gets hot. Steps are short (default 1.5 s)
and there is a cooling pause between them. Stop if you smell or feel heat.
"""
from __future__ import annotations

import argparse
import sys
import time

from Rosmaster_Lib import Rosmaster

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
NAMES = {1: "front-left", 2: "back-left", 3: "front-right", 4: "back-right"}
# Counts below this over a step are indistinguishable from noise / jitter.
NOISE_COUNTS = 20


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", type=int, default=3, choices=[1, 2, 3, 4])
    ap.add_argument("--from", dest="lo", type=int, default=30)
    ap.add_argument("--to", dest="hi", type=int, default=80)
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--secs", type=float, default=1.5)
    ap.add_argument("--curve", action="store_true",
                    help="Do not stop at the first moving step. Sweep the "
                         "whole range and report speed at each PWM, to fit "
                         "the RUNNING curve above breakaway.")
    ap.add_argument("--counts-per-rev", type=float, default=2360.0)
    ap.add_argument("--circumference", type=float, default=0.248)
    a = ap.parse_args()

    def mps(counts: float) -> float:
        """Encoder counts over one step -> wheel surface speed."""
        return (counts / a.counts_per_rev) * a.circumference / a.secs

    idx = a.motor - 1
    print(f"\n  WHEELS UP. Sweeping M{a.motor} ({NAMES[a.motor]}) "
          f"from {a.lo} to {a.hi} in steps of {a.step}.")
    print("  Watch the wheel. Ctrl-C aborts.\n")
    input("  Enter to start: ")

    car = Rosmaster(com=PORT, delay=0.002, debug=False)
    car.set_car_type(1)
    car.create_receive_threading()
    car.set_auto_report_state(True, forever=False)
    time.sleep(0.6)

    def stop(secs: float = 1.0) -> None:
        end = time.time() + secs
        while time.time() < end:
            car.set_motor(0, 0, 0, 0)
            time.sleep(0.03)

    def drive(pwm: int, secs: float) -> int:
        """Drive this motor alone at pwm; return |encoder delta| for it."""
        vals = [0, 0, 0, 0]
        vals[idx] = pwm
        before = car.get_motor_encoder()[idx]
        end = time.time() + secs
        while time.time() < end:
            car.set_motor(*vals)
            time.sleep(0.05)
        after = car.get_motor_encoder()[idx]
        stop(0.6)
        return abs(after - before)

    first_moving = None
    try:
        print(f"  {'pwm':>5} {'fwd m/s':>9} {'rev m/s':>9}   verdict")
        curve: list[tuple[int, float, float]] = []
        for pwm in range(a.lo, a.hi + 1, a.step):
            fwd = drive(pwm, a.secs)
            rev = drive(-pwm, a.secs)
            moving = fwd > NOISE_COUNTS and rev > NOISE_COUNTS
            verdict = "TURNS" if moving else (
                "one direction only" if (fwd > NOISE_COUNTS) != (rev > NOISE_COUNTS)
                else "still"
            )
            print(f"  {pwm:5d} {mps(fwd):9.3f} {mps(rev):9.3f}   {verdict}")
            if moving:
                curve.append((pwm, mps(fwd), mps(rev)))
                if first_moving is None:
                    first_moving = pwm
                    if not a.curve:
                        break
            time.sleep(1.0)   # let it cool between steps

        if a.curve and len(curve) >= 2:
            # Least-squares over the RUNNING points only. Breakaway is a
            # separate, higher number and must not be fitted from these.
            for label, col in (("forward", 1), ("reverse", 2)):
                xs = [c[col] for c in curve]
                ys = [float(c[0]) for c in curve]
                n = len(xs)
                mx, my = sum(xs) / n, sum(ys) / n
                den = sum((x - mx) ** 2 for x in xs)
                if den <= 0:
                    continue
                gain = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
                dead = my - gain * mx
                print(f"\n  running fit ({label}): "
                      f"dead {max(dead, 0.0):.1f}  gain {gain:.1f}")
            print(f"  breakaway PWM: {first_moving} "
                  f"(NOT the same as the fitted dead — stiction)")
    except KeyboardInterrupt:
        print("\n  aborted by operator")
    except Exception as exc:  # noqa: BLE001
        print(f"\n  !! {exc}")
    finally:
        stop(1.5)
        print("  motors stopped.")

    print()
    if first_moving is None:
        print(f"  M{a.motor} never turned in both directions up to PWM {a.hi}.")
        print("  That is a motor/wiring fault, not a calibration problem —")
        print("  no dead_pwm value will make it drive. Do NOT raise its")
        print("  dead_pwm to compensate; that only heats a stalled motor.")
    else:
        print(f"  M{a.motor} ({NAMES[a.motor]}) starts turning at PWM "
              f"{first_moving}.")
        print(f"  Use that as its dead_pwm, then re-run fit_reverse.py with a")
        print(f"  speed range high enough to keep it above the threshold so")
        print(f"  its gain can be fitted too.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
