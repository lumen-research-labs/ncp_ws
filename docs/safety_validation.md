# Safety validation — Phase 2 sign-off record

Phase 2 acceptance requires every test below to pass, recorded here with date
and operator. **Phase 4 does not begin until this document is signed off.**

> **This unit has no physical E-stop.** `/safety/kill` is the only stop
> mechanism; battery disconnect is the last resort. Every hardware test below
> is run with the wheels off the ground first, and with a human hand on the
> power switch.

Nothing below has been run yet — Phase 2 has not started.

## 1. Unit tests (no ROS runtime)

Pure functions in `rm_safety/safety_logic.py`.

| Case | Expected | Result | Date | Operator |
|---|---|---|---|---|
| Step input | acceleration clamped | | | |
| NaN in command | zero velocity | | | |
| NaN in scan | treated as no-return, no spurious stop | | | |
| dt = 0 | no divide-by-zero, no unbounded accel | | | |
| Out-of-range velocity | clamped to limits | | | |
| Single-scan noise spike | does NOT latch (N=3 required) | | | |
| N consecutive below stop zone | latches | | | |

## 2. Replay tests

Against the Phase 1 baseline bag in `tests/fixtures/`.

| Case | Expected | Result | Date | Operator |
|---|---|---|---|---|
| Known-safe segments | no spurious e-stops | | | |

## 3. Hardware tests — no policy, wheels off ground first

| Case | Expected | Result | Date | Operator |
|---|---|---|---|---|
| Teleop | clamping visibly limits speed | | | |
| Walk toward lidar | slow zone, then stop zone | | | |
| Kill service mid-motion | immediate stop | | | |
| Kill the command publisher | watchdog zeroes output | | | |
| Cover the lidar | stale-scan e-stop | | | |
| Restart node while killed | comes up killed, does not move | | | |

## 4. Acceptance criteria

- [ ] Every test above passes, recorded with date and operator.
- [ ] Verified by inspection that nothing other than `rm_safety` publishes `/cmd_vel`.
- [ ] E-stop latches and requires explicit `/safety/reset`.

## Sign-off

```
Signed off by:  ____________________
Date:           ____________________
Limits in force at sign-off (config/safety_limits.yaml commit): ____________
```
