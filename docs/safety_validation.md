# Safety validation — Phase 2 sign-off record

Phase 2 acceptance requires every test below to pass, recorded here with date
and operator. **Phase 4 does not begin until this document is signed off.**

> **This unit has no physical E-stop.** `/safety/kill` is the only stop
> mechanism; battery disconnect is the last resort. Every hardware test below
> is run with the wheels off the ground first, and with a human hand on the
> power switch.

Software tiers (unit + replay) are automated and passing as of 2026-08-13.
The hardware tier (§3) has NOT been run — it requires the operator, wheels off
the ground, and a hand on the power switch, and cannot be run by an agent.

## 1. Unit tests (no ROS runtime)

Pure functions in `rm_safety/safety_logic.py`. Run: `pytest tests/unit/`
(81 tests, all passing 2026-08-13).

| Case | Expected | Result | Date | Operator |
|---|---|---|---|---|
| Step input | acceleration clamped | PASS | 2026-08-13 | automated |
| NaN in command | zero velocity | PASS | 2026-08-13 | automated |
| NaN in scan | treated as no-return, no spurious stop | PASS | 2026-08-13 | automated |
| dt = 0 | no divide-by-zero, no unbounded accel | PASS | 2026-08-13 | automated |
| Out-of-range velocity | clamped to limits | PASS | 2026-08-13 | automated |
| Single-scan noise spike | does NOT latch (N=3 required) | PASS | 2026-08-13 | automated |
| N consecutive below stop zone | latches | PASS | 2026-08-13 | automated |

## 2. Replay tests

Against the Phase 1 baseline bag and TG30 scan captures in `tests/fixtures/`,
run ROS-free through the real `safety_logic.evaluate()` and the real
`config/safety_limits.yaml`. Run: `pytest tests/replay/` (9 tests, all passing
2026-08-13). The bag is read directly from its sqlite3 store by a minimal,
self-checking CDR decoder (`tests/replay/laserscan_bag_reader.py`) — no ROS
runtime, no container.

| Case | Expected | Result | Date | Operator |
|---|---|---|---|---|
| 600-scan static bag streamed | 0.0 no-returns never cause perception loss (min_range always finite, never 0.0) | PASS | 2026-08-13 | automated |
| 600-scan static bag streamed | real ~0.21 m forward obstacle correctly latches STOPPED and holds | PASS | 2026-08-13 | automated |
| Open-space capture | forward arc clear (~0.49 m), no false stop | PASS | 2026-08-13 | automated |
| Cluttered capture, 1 frame | does NOT latch (noise filter) | PASS | 2026-08-13 | automated |
| Cluttered capture, N frames | latches after `consecutive_scans_to_trigger` | PASS | 2026-08-13 | automated |
| All-0.0 synthetic scan | fails closed → STOPPED (no-perception) | PASS | 2026-08-13 | automated |
| Single real return in a field of 0.0 | min_range = the real return, not 0.0 | PASS | 2026-08-13 | automated |

### 2a. What the baseline bag does and does NOT cover

`tests/fixtures/ncp_static_bag` (600 scans, 60 s) was recorded in a **cluttered
room with a real object ~0.21 m dead-ahead (+6.7°, inside the 0.35 m stop
zone)**. Its e-stop latching is therefore **correct behaviour, not a spurious
stop**. This bag validates the TG30 `0.0`-no-return handling and the latch
path — it is **not** a clear-path recording. The "no spurious e-stop on a clear
path" case still needs a **new bag recorded in genuinely open space** (operator
task; noted in CLAUDE.md next-session list).

### 2b. RESOLVED 2026-08-14 (operator decision) — pre-latch full-speed window

Surfaced by replay 2026-08-13: during the consecutive-scan noise-filter
window, a **sustained stop-zone obstacle produced OK / FULL commanded
velocity for the first (`consecutive_scans_to_trigger` − 1) control ticks**,
then latched to zero. A *slow*-zone obstacle scaled velocity down
immediately, but a closer *stop*-zone obstacle did **not** slow the robot
until the latch tripped. At 5 Hz control and N=3 that was ~400 ms of
un-slowed motion before the stop (≈15 cm at the 0.25 m/s clamp).

**Decision: apply slow-zone scaling to un-latched stop-zone readings.** Fixed
in `safety_logic.evaluate()` — the SLOW-ZONE branch (precedence 5) now covers
`zone in ("slow", "stop")`; `slow_scale()` already evaluates to 0.0 for
`min_r <= stop_zone_m`, so an un-latched stop-zone reading is scaled to zero
immediately (state SLOW, reason `stop_zone_prelatch`) instead of passing
through at full commanded velocity. Removes the counter-intuitive "closer
obstacle brakes later" inversion. The previously-pinning test
`test_prelatch_window_emits_full_velocity` was replaced with
`test_prelatch_window_emits_zero_velocity`, which pins the new behaviour;
`test_sample_scan_latches_after_consecutive_window` updated to expect `SLOW`
(not `OK`) on pre-latch ticks. Both pass.

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
