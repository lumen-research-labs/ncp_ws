#!/usr/bin/env python3
"""Wheels-up encoder + velocity-decode diagnostic for ROSMASTER X3.

Purpose
    Determine whether the fixed ~0.8 m/s wheel speed (any nonzero /cmd_vel) and
    the garbage /vel_raw are caused by dead/erratic wheel encoders or by a
    firmware velocity-decode/scale problem.

What it does
    Takes over the base-board serial link directly via Rosmaster_Lib
    (driver_node MUST be stopped first - only one writer on the port). Commands
    a low steady speed in two short phases, logs raw per-wheel encoder counts
    and the firmware's own velocity estimate at 10 Hz, then floods stop.

Fail-closed
    Any exception, Ctrl-C, SIGTERM, or the hard time cap -> repeated
    set_car_motion(0,0,0) for ~2 s before exit. The base driver has NO
    watchdog; if this process is hard-killed the wheels keep spinning and the
    only stop left is a battery pull.

NOT a ROS node. Operator + kill switch required - it moves the wheels.
"""
import argparse
import signal
import sys
import time

from Rosmaster_Lib import Rosmaster

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
LOOP_HZ = 10.0
WHEELS = ("front_left", "front_right", "back_left", "back_right")  # m1..m4 order per Rosmaster_Lib


class Stopping(Exception):
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--vx1", type=float, default=0.05, help="phase A forward speed (m/s)")
    ap.add_argument("--vx2", type=float, default=0.15, help="phase B forward speed (m/s)")
    ap.add_argument("--phase", type=float, default=4.0, help="seconds per phase")
    ap.add_argument("--cpr", type=float, default=1980.0,
                    help="assumed encoder counts per wheel revolution (for the rev estimate only; raw counts are the real output)")
    args = ap.parse_args()

    car = Rosmaster(com=args.port, delay=0.002, debug=False)
    car.set_car_type(1)  # X3
    car.create_receive_threading()
    car.set_auto_report_state(True, forever=False)
    time.sleep(0.6)

    samples = []  # (phase, t, m1,m2,m3,m4, fw_vx, fw_vy, fw_vz)
    t0 = time.time()
    HARD_CAP = 6.0 + 3 * args.phase  # absolute upper bound on runtime

    def flood_stop(secs=2.0):
        end = time.time() + secs
        while time.time() < end:
            try:
                car.set_car_motion(0.0, 0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.03)

    def on_signal(signum, frame):
        raise Stopping()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def run_phase(name, vx, secs):
        dt = 1.0 / LOOP_HZ
        end = time.time() + secs
        while time.time() < end:
            if time.time() - t0 > HARD_CAP:
                raise Stopping()
            if vx != 0.0:
                car.set_car_motion(vx, 0.0, 0.0)
            m1, m2, m3, m4 = car.get_motor_encoder()
            fx, fy, fz = car.get_motion_data()
            samples.append((name, round(time.time() - t0, 3), m1, m2, m3, m4, fx, fy, fz))
            time.sleep(dt)

    try:
        print("# baseline (wheels should be still)")
        run_phase("baseline", 0.0, 2.0)
        print(f"# phase A: vx={args.vx1}")
        run_phase("A", args.vx1, args.phase)
        print(f"# phase B: vx={args.vx2}")
        run_phase("B", args.vx2, args.phase)
        print("# coast-down (commanding zero)")
        run_phase("coast", 0.0, 2.0)
    except Stopping:
        print("\n!! stop requested / cap hit - flooding zero", file=sys.stderr)
    except Exception as e:
        print(f"\n!! error: {e!r} - flooding zero", file=sys.stderr)
    finally:
        flood_stop(2.0)
        try:
            car.set_auto_report_state(True, forever=False)  # leave enabled for driver_node
        except Exception:
            pass

    # ---- report ----
    print("\n=== raw samples (phase, t, enc m1..m4, firmware vx/vy/vz) ===")
    for s in samples:
        print(f"{s[0]:>8}  t={s[1]:6.2f}  enc=({s[2]:>10},{s[3]:>10},{s[4]:>10},{s[5]:>10})  fw=({s[6]:+.3f},{s[7]:+.3f},{s[8]:+.3f})")

    def phase_slice(name):
        return [s for s in samples if s[0] == name]

    print("\n=== per-wheel encoder deltas ===")
    print(f"(assumed CPR = {args.cpr:.0f} counts/rev - rev numbers scale with this, raw counts do not)")
    for name in ("A", "B"):
        sl = phase_slice(name)
        if len(sl) < 2:
            continue
        span = sl[-1][1] - sl[0][1]
        print(f"\nphase {name}  ({span:.2f} s):")
        deltas = []
        for i, w in enumerate(WHEELS):
            d = sl[-1][2 + i] - sl[0][2 + i]
            deltas.append(d)
            cps = d / span if span else 0.0
            rps = cps / args.cpr
            print(f"  {w:<12} dcounts={d:>10}   {cps:>10.1f} counts/s   ~{rps:+.2f} rev/s   ~{abs(rps)*span:.1f} rev in phase")
        mx = max((abs(x) for x in deltas), default=0)
        for w, d in zip(WHEELS, deltas):
            if mx and abs(d) < 0.10 * mx:
                print(f"  !! {w} moved <10% of the fastest wheel - possible dead/disconnected encoder")
        fw = [s[6] for s in sl]
        print(f"  firmware vx over phase: min={min(fw):+.3f} max={max(fw):+.3f} mean={sum(fw)/len(fw):+.3f}  (commanded {args.vx1 if name=='A' else args.vx2:+.3f})")

    bl = phase_slice("baseline")
    if len(bl) >= 2:
        moved = [bl[-1][2 + i] - bl[0][2 + i] for i in range(4)]
        print(f"\nbaseline encoder drift (should be ~0): {dict(zip(WHEELS, moved))}")


if __name__ == "__main__":
    main()
