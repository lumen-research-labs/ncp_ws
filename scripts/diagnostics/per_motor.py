#!/usr/bin/env python3
"""Per-motor identification test for ROSMASTER X3.

Drives ONE motor at a time, forward then reverse, pausing between each so the
operator can see which physical wheel turns and which way. Logs ALL FOUR
encoders each spin (so we catch cross-wiring: 'motor N' command turning a
different wheel or moving a different encoder).

  python3 per_motor.py [--pwm 20] [--secs 3]

driver_node / rosmaster_main must NOT hold the serial port.
Fail-closed: exception / Ctrl-C -> flood set_motor(0,0,0,0).
Wheels UP, operator + kill switch.
"""
import argparse, signal, sys, time
from Rosmaster_Lib import Rosmaster

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
HZ = 10.0


class Stop(Exception):
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pwm", type=int, default=20)
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()

    car = Rosmaster(com=PORT, delay=0.002, debug=False)
    car.set_car_type(1)
    car.create_receive_threading()
    car.set_auto_report_state(True, forever=False)
    time.sleep(0.6)
    print("firmware:", car.get_version(), " PID:", car.get_motion_pid())
    print(f"PWM={a.pwm}  secs={a.secs}\n")

    def flood_stop(secs=1.5):
        end = time.time() + secs
        while time.time() < end:
            try:
                car.set_motor(0, 0, 0, 0)
            except Exception:
                pass
            time.sleep(0.03)

    signal.signal(signal.SIGINT, lambda s, f: (_ for _ in ()).throw(Stop()))
    signal.signal(signal.SIGTERM, lambda s, f: (_ for _ in ()).throw(Stop()))

    results = {}

    def spin(idx, val):
        pwm = [0, 0, 0, 0]
        pwm[idx] = val
        t0 = time.time()
        first = list(car.get_motor_encoder())
        last = first
        while time.time() - t0 < a.secs:
            car.set_motor(*pwm)
            last = list(car.get_motor_encoder())
            time.sleep(1.0 / HZ)
        flood_stop(1.2)
        span = time.time() - t0
        return [round((last[i] - first[i]) / span) for i in range(4)]

    try:
        for idx in range(4):
            mid = idx + 1
            print(f"\n================  MOTOR M{mid}  (index {idx})  ================")
            input(f"  wheels UP. Enter to drive ONLY M{mid} at +{a.pwm} for {a.secs:.0f}s...")
            fwd = spin(idx, a.pwm)
            print(f"  +{a.pwm}: all-4 encoders c/s = {fwd}")
            input(f"  Enter to drive ONLY M{mid} at -{a.pwm} for {a.secs:.0f}s...")
            rev = spin(idx, -a.pwm)
            print(f"  -{a.pwm}: all-4 encoders c/s = {rev}")
            note = input(f"  Which WHEEL moved? did + look FORWARD or BACKWARD?  > ")
            results[f"M{mid}"] = {"+": fwd, "-": rev, "note": note}
    except Stop:
        print("\n!! stop - flooding zero", file=sys.stderr)
    except Exception as e:
        print(f"\n!! {e!r} - flooding zero", file=sys.stderr)
    finally:
        flood_stop(2.0)

    print("\n\n================  SUMMARY  ================")
    for m, r in results.items():
        print(f"  {m}:  +cmd encoders {r['+']}   -cmd encoders {r['-']}")
        print(f"       operator: {r['note']}")


if __name__ == "__main__":
    main()
