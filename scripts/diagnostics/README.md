# Base-board bench diagnostics

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
