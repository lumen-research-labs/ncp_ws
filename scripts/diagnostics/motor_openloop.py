#!/usr/bin/env python3
"""Open-loop motor + PID-readout diagnostic for ROSMASTER X3.

Why
    wheel_diag.py showed the closed-loop speed control runs away: any nonzero
    /cmd_vel drives all wheels to ~full PWM, and /vel_raw reports the wrong sign.
    This script bypasses the firmware speed PID entirely (set_motor = raw PWM,
    no encoder feedback) to prove whether the motors/wiring are healthy and to
    see which direction each encoder counts for a known PWM sign.

What it does
    1. Prints firmware version, car type, and the stored motor PID gains.
    2. Wheels-up: drives all four motors open-loop at low PWM, briefly, in
       three steps (+small, +bigger, -small), logging raw per-wheel encoder
       counts + the firmware's own velocity estimate at 10 Hz.

Fail-closed
    Any exception / Ctrl-C / SIGTERM / hard time cap -> repeated set_motor(0,0,0,0)
    AND set_car_motion(0,0,0). No watchdog on the base board: a hard kill leaves
    the wheels spinning and only a battery pull stops them.

driver_node MUST be stopped first (single writer on the serial port).
NOT a ROS node. Operator + kill switch required - it moves the wheels.
"""
import signal, sys, time
from Rosmaster_Lib import Rosmaster

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
HZ = 10.0
WHEELS = ("m1", "m2", "m3", "m4")   # per Rosmaster_Lib get_motor_encoder order
STEPS = [("+small", (6, 6, 6, 6), 2.0),
         ("+big",   (12, 12, 12, 12), 2.0),
         ("-small", (-6, -6, -6, -6), 2.0)]


class Stop(Exception):
    pass


def main():
    car = Rosmaster(com=PORT, delay=0.002, debug=False)
    car.set_car_type(1)
    car.create_receive_threading()
    car.set_auto_report_state(True, forever=False)
    time.sleep(0.6)

    print("firmware version :", car.get_version())
    print("car type         :", car.get_car_type_from_channel() if hasattr(car, "get_car_type_from_channel") else "n/a")
    print("motor PID [kp,ki,kd]:", car.get_motion_pid())
    print()

    t0 = time.time()
    HARD_CAP = 4.0 + sum(s[2] for s in STEPS) + 4.0
    rows = []

    def flood_stop(secs=2.0):
        end = time.time() + secs
        while time.time() < end:
            try:
                car.set_motor(0, 0, 0, 0)
                car.set_car_motion(0.0, 0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.03)

    def on_sig(s, f):
        raise Stop()
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    def phase(name, pwm, secs):
        dt = 1.0 / HZ
        end = time.time() + secs
        while time.time() < end:
            if time.time() - t0 > HARD_CAP:
                raise Stop()
            if any(pwm):
                car.set_motor(*pwm)
            m = car.get_motor_encoder()
            fx, fy, fz = car.get_motion_data()
            rows.append((name, round(time.time() - t0, 2), *m, fx, fy, fz))
            time.sleep(dt)

    try:
        phase("baseline", (0, 0, 0, 0), 1.5)
        for name, pwm, secs in STEPS:
            print(f"# step {name}: set_motor{pwm}")
            phase(name, pwm, secs)
            phase(f"{name}_coast", (0, 0, 0, 0), 1.0)
    except Stop:
        print("\n!! stop / cap - flooding zero", file=sys.stderr)
    except Exception as e:
        print(f"\n!! {e!r} - flooding zero", file=sys.stderr)
    finally:
        flood_stop(2.0)

    print("\n=== samples (phase, t, enc m1..m4, fw vx/vy/vz) ===")
    for r in rows:
        print(f"{r[0]:>12} t={r[1]:5.2f}  enc=({r[2]:>9},{r[3]:>9},{r[4]:>9},{r[5]:>9})  fw=({r[6]:+.3f},{r[7]:+.3f},{r[8]:+.3f})")

    print("\n=== per-step encoder count/s (sign = direction) ===")
    for name, _, _ in STEPS:
        sl = [r for r in rows if r[0] == name]
        if len(sl) < 2:
            continue
        span = sl[-1][1] - sl[0][1]
        cps = [ (sl[-1][2+i] - sl[0][2+i]) / span for i in range(4) ]
        print(f"{name:>8}: " + "  ".join(f"{w}={c:>+9.0f}" for w, c in zip(WHEELS, cps)))
        signs = [ "+" if c > 50 else "-" if c < -50 else "0" for c in cps ]
        if len(set(signs)) > 1:
            print(f"          !! encoders disagree on direction: {dict(zip(WHEELS, signs))}  -> reversed/dead encoder")
        for w, c in zip(WHEELS, cps):
            if abs(c) < 50:
                print(f"          !! {w} barely moved ({c:+.0f} c/s) -> dead/disconnected encoder or motor")


if __name__ == "__main__":
    main()
