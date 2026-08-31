# Base-board bench diagnostics

**Bringing up a NEW robot? Follow [`docs/calibration.md`](../../docs/calibration.md)**,
which puts these scripts in the order they must be run and states what each
step must prove before the next one is safe. This file is the reference for
the individual tools.

Standalone Python scripts (no ROS, no colcon) that talk to the ROSMASTER X3
control board directly via `Rosmaster_Lib`. Written 2026-08-28 to root-cause
the velocity / clamp failure (see CLAUDE.md §3).

**These MOVE THE WHEELS. Run only wheels-UP, with the operator holding a kill
switch / near the battery.** Each script floods `set_motor(0,0,0,0)` on exit,
on Ctrl-C, and on any exception — but the base board has no watchdog, so a
hard kill leaves the wheels spinning until a battery pull.

Before running any of them: stop whatever holds the base-board serial port
(the vendor `Mcnamu_driver_X3`, `rm_base_node`, or the `rosmaster_main.py`
autostart). Only one process may own `/dev/ttyUSB1`.

| script | what it answers |
|---|---|
| `wheel_diag.py` | closed-loop (`set_car_motion`) behaviour: does commanded speed reach the wheels? encoder health at rest and under command. |
| `motor_openloop.py` | open-loop (`set_motor`) behaviour: are the motors + PWM path healthy independent of the firmware speed PID? prints stored PID gains. |
| `per_motor.py` | drives one motor at a time — maps motor channel → physical wheel + direction, isolates a dead/weak motor or encoder. |

Findings from the 2026-08-28 run are in CLAUDE.md §3; the resulting workaround
is the `rm_base` package.

## ROS-level scripts

These run against the live stack instead of the serial port, so `rm_base` and
`rm_safety` must already be up. They publish `/policy/cmd_vel` — an INPUT to
`rm_safety`, never `/cmd_vel` — so the clamps, the e-stop arcs and
`/safety/kill` all still apply, and each one arms and re-kills `rm_safety`
inside a single `try/finally`.

| script | what it answers | wheels |
|---|---|---|
| `closed_loop_check.py` | do the closed-loop guards behave — sign, stall, saturation — and is the loop pulling measured toward target? | UP |
| `fr_threshold.py` | front-right's breakaway and running thresholds, motor driven directly so the other wheels cannot drag it. | UP |
| `fit_reverse.py` | fits `dead_pwm` / `gain_pwm_per_mps` per direction from two straight legs. | DOWN |
| `velocity_accuracy.py` | **does achieved speed match commanded speed**, linear and angular, against a tape measure across a range of speeds. | DOWN |
| `watch_stack.py` | live rates and states across the whole stack. | either |

`velocity_accuracy.py` is the acceptance test for the clamps in physical
units. Two things about it are deliberate and should not be "simplified":

- The reference is the **time integral of `/cmd_vel`**, not `vx * secs`.
  `rm_safety` acceleration-limits every command, so a 5 s leg at 0.25 m/s
  never travels 1.25 m even when the base is perfect.
- It runs **several speeds in both directions**. A one-speed check passes for
  infinitely many wrong `(dead_pwm, gain)` splits — the mistake that let a
  perfect 1.00 m straight line coexist with a 79 deg "180 deg" turn.

`/odom` is recorded next to the tape reading as a second thing under test. It
is never the ruler: its scale was fitted to a tape with wheel slip baked in.
