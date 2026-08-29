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
    a = ap.parse_args()

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
        print(f"  {'pwm':>5} {'fwd counts':>11} {'rev counts':>11}   verdict")
        for pwm in range(a.lo, a.hi + 1, a.step):
            fwd = drive(pwm, a.secs)
            rev = drive(-pwm, a.secs)
            moving = fwd > NOISE_COUNTS and rev > NOISE_COUNTS
            verdict = "TURNS" if moving else (
                "one direction only" if (fwd > NOISE_COUNTS) != (rev > NOISE_COUNTS)
                else "still"
            )
            print(f"  {pwm:5d} {fwd:11d} {rev:11d}   {verdict}")
            if moving and first_moving is None:
                first_moving = pwm
                break
            time.sleep(1.0)   # let it cool between steps
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
