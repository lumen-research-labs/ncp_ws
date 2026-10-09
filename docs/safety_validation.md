# Safety validation — Phase 2 sign-off record

Phase 2 acceptance requires every test below to pass, recorded here with date
and operator. **Phase 4 does not begin until this document is signed off.**

> **This unit has no physical E-stop.** `/safety/kill` is the only stop
> mechanism; battery disconnect is the last resort. Every hardware test below
> is run with the wheels off the ground first, and with a human hand on the
> power switch.

Software tiers (unit + replay) are automated and passing as of 2026-08-13;
**210 tests as of 2026-08-29** (unit + replay, workstation tier).
The hardware tier (§3) has NOT been run — it requires the operator, wheels off
the ground, and a hand on the power switch, and cannot be run by an agent.

## 1. Unit tests (no ROS runtime)

Pure functions in `rm_safety/safety_logic.py`. Run: `pytest tests/unit/`
(83 tests, all passing 2026-08-14 — 81 as of 2026-08-13 plus 2 added for the
self-occlusion masking fix, §2c).

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

### 2c. RESOLVED 2026-08-14 (operator decision) — chassis standoffs permanently latching the e-stop, accepted blind sectors

First live hardware session: `rm_safety` latched STOPPED within seconds of
every launch and every `/safety/reset` call, with no way to clear it —
correct behaviour for a genuine obstacle, but the robot was in an open room
with nothing within 1 m at the time. Root-caused via multiple raw `/scan`
captures (analyzed offline, not through `rm_safety`): two chassis-mounted
mounting standoffs — one on each side of the robot — produce stable, real
(non-zero, not the TG30 `0.0` no-return case) lidar returns well inside the
0.35 m stop zone. Confirmed self-hit, not an environmental object: identical
angle and range across 5+ independent scans, unchanged before and after
physically repositioning the robot.

- LEFT standoff: ~97.3–117.7°, range ~0.25–0.26 m
- RIGHT standoff: ~-108.2 to -130.0°, range ~0.106–0.125 m

Both fall inside the (then-unmasked) 60° lateral arcs centred at ±90°
(`estop.lateral_arc_deg`), so `min_range_in_arcs()` always read < 0.35 m on
both sides. Because `estop.latching: true`, and `/safety/reset` re-latches
immediately whenever `is_stop_zone` is still true (by design — see
`safety_logic.py`'s reset-with-obstacle-still-present branch), the robot
could never actually clear.

**Considered and REJECTED: symmetric fix.** Narrowing `lateral_arc_deg` to
clear both standoffs would have required shrinking it to ~10–12° (from 60°)
on each side — gutting real lateral obstacle coverage on both sides to work
around a mounting artifact. Rejected as an unacceptable safety trade-off.

**Decision: explicit self-occlusion mask in `min_range_in_arcs()`,
mirroring `config/interface.yaml`'s lidar-observation mask but as a
separate, independently-configured mask** (different function, different
arc definition — the two do not interact; see the note in
`safety_limits.yaml`). New `estop.self_occlusion_zones_deg` config:
`[[92.0, 123.0], [-135.0, -103.0]]` (~5° margin beyond the measured extent
on every edge — widened from an initial ~2–3° pass after safety review
flagged that margin as too tight against measurement jitter, ~±1° scan-to-
scan plus the TG30's 0.178° angular quantization). A ray inside either zone
is dropped before the forward/lateral arc membership check — masked out
entirely, not counted as "clear" — so a monitored arc whose only returns are
masked still correctly falls back to no-signal-in-zone. Reviewed:
`min_range_in_arcs()`'s existing `+inf`-on-no-valid-returns path, combined
with `evaluate()`'s existing no-perception → `scan_stale` → `STOPPED`
handling, means "everything in an arc is masked" still fails closed, not
open. Two new unit tests pin the masking (`test_self_occlusion_zone_masks_chassis_standoff`,
`test_just_outside_self_occlusion_zone_still_detected`); unit 83/83 and
replay 9/9 pass.

**Accepted risk — this is a permanent, angle-only blind sector, not just a
standoff-range filter.** Any real obstacle at a bearing inside
`[92°,123°]` or `[-135°,-103°]`, *at any range*, is now invisible to the
e-stop, not only returns near the standoffs' own ~0.11–0.26 m. In practice
this is very likely a non-issue: at those exact bearings the beam already
terminated on the standoff before this fix, so there was no real detection
capability there to give up — the mask formalizes an existing blind spot
rather than creating a new one. Recorded here alongside the existing
accepted blind spot (rear hemisphere, chassis-occluded, "the robot must be
treated as blind behind" — CLAUDE.md §1/§4) as the same category of
decision. Not yet validated on real hardware beyond the hardware tier in
§3 below — the "walk toward lidar" test should specifically include an
approach from directly behind each standoff's bearing to confirm the accepted
blind sector behaves as expected in practice, not just in the scan math.

## 2c. Slow-zone arc split and radius reduction (2026-08-29) — NOT YET VALIDATED ON HARDWARE

**Change.** The slow and stop zones previously shared one arc pair. They no
longer do:

| zone | forward arc | lateral arc | radius |
|---|---|---|---|
| slow | **30 deg** (+-15) | **disabled** (0.0) | **0.55 m** (was 0.65) |
| stop | 90 deg (unchanged) | 60 deg (unchanged) | 0.35 m (unchanged) |

**Why.** Operator observation across several driven runs: returns from walls
and furniture BESIDE the robot were scaling forward velocity down with a
completely clear path ahead. Narrowing the single shared arc would have fixed
that by also narrowing the STOP zone — buying comfort with protection. The
split lets the slow zone become a narrow forward cone while stop-zone coverage
is untouched.

**Sizing, not picking.**

* Cone: at the slow radius the half-cone must subtend the robot's half-extent
  in the scan plane (~0.13 m, from the ~0.26 m measured chassis extent behind
  `self_occlusion_zones_deg`). 0.55 * tan(15 deg) = 0.147 m. At the 0.35 m
  STOP radius the same angle covers only 0.094 m — narrower than the robot,
  which is precisely why the stop arc must stay at 90 deg.
* Radius: braking 0.042 m + 3-scan filter travel 0.075 m = 0.117 m needed;
  the 0.55/0.35 band is 0.200 m, a 1.71x margin. 0.50 was rejected at 1.29x —
  the same thin margin that got 0.50 reverted once already.

Both criteria are pinned by tests (`tests/unit/test_safety_arc_split.py`), so
a later change to `max_vx`, the radius or the angle has to face them rather
than silently eating the margin. The three are coupled.

**Verified by review and by test, NOT by hardware.**

* Reviewed by `rm-safety-reviewer`: no fail-open path. Confirmed by reading
  the code that stop coverage and the latch derive only from the wide-arc
  minimum; that the 2026-08-14 pre-latch-to-zero behaviour survives even for a
  stop-zone obstacle OUTSIDE the narrow cone (`safety_logic.py`, the
  `slow_scale(min_r if zone == "stop" else min_r_slow, ...)` ternary); that the
  no-perception and scan-stale paths are untouched; and that
  `/safety/status.min_range` still reports the wide-arc value.
* 266 unit + replay tests pass, including the pre-existing pre-latch
  regression test and 33 new tests for the split.
* Misconfiguration is fatal: a slow arc wider than the stop arc raises at
  config load rather than being clamped.

**Outstanding.** This geometry is reasoned from the swept-path argument, not
measured. No recorded run has been scored against it — the operator declined
a baseline capture, so the before/after comparison the replay tier could have
provided does not exist. What is NOT known: how often the old wide arc was
actually triggering, at what bearings, and whether any of those triggers were
doing useful work. **Sign-off row below stays open until a driven run is
observed with the new geometry.**

## 3. Hardware tests — no policy, wheels off ground first

| Case | Expected | Result | Date | Operator |
|---|---|---|---|---|
| Teleop | clamping visibly limits speed | | | |
| Walk toward lidar | slow zone, then stop zone | | | |
| Kill service mid-motion | immediate stop | | | |
| Kill the command publisher | watchdog zeroes output | | | |
| Cover the lidar | stale-scan e-stop | | | |
| Restart node while killed | comes up killed, does not move | | | |

### 3a. Evidence accumulated 2026-08-29 (NOT a sign-off)

Two floor sessions with the operator present exercised much of the table
above incidentally. Recording it so the eventual sign-off run knows what is
already evidenced and what is genuinely untested:

| Observed | Evidence |
|---|---|
| Clamping limits speed | policy asked `vx 0.400` / `wz 1.200`; `/cmd_vel` carried 0.250 / 0.800 continuously |
| Starts KILLED, does not move | every launch this session; `min_range` reads `inf` while KILLED because `evaluate()` returns before processing the scan |
| Kill/reset services work | used dozens of times across both sessions, including mid-motion |
| E-stop latches, needs explicit reset | 2026-08-13 replay tier; not re-exercised on hardware this session |
| Acceleration limiter holds | 343 consecutive `/cmd_vel` samples: max |dvx/dt| within limit except one **deceleration to zero** (5.6 m/s², fail-closed paths deliberately bypass the limiter — braking hard is always safe); max |dwz/dt| 1.063 vs 1.0 limit, within timing jitter |
| Sole `/cmd_vel` publisher | `ros2 topic info /cmd_vel` → `Publisher count: 1`, checked before every arm |
| Command timeout fails closed | `reason: command_timeout` whenever `command_source` pointed at an unpublished topic |

**Still genuinely untested on hardware:** walking toward the lidar to trigger
the slow→stop zones under real motion, covering the lidar for a stale-scan
e-stop, and killing the command publisher mid-motion to prove the watchdog.
Those three are the gap.

### 3c. LIDAR ZONES CAN NOW BE SWITCHED OFF (2026-08-30) — NOT SIGNED OFF

`estop.slow_zone_enabled` / `estop.stop_zone_enabled` pause the lidar zones.
They default `true` in `config/safety_limits.yaml`, but **since 2026-08-31
`ncp.launch.py` passes `false` for both by default** (operator decision), so
a full-stack policy run has no proximity protection unless it is asked for.
Every other entry point keeps the config default. Added at operator request and with operator approval:
the slow zone's velocity scaling brakes before the trained policy can
demonstrate obstacle avoidance, so the policy could not be evaluated with it
active.

**A run with either switch `false` is OUTSIDE every tier recorded in this
document.** The unit and replay tiers above were all scored with the zones
active, and none of them says anything about a run without them. Nothing here
should be read as a sign-off for that configuration.

What a disabled stop zone actually removes: all obstacle-based protection.
This robot has no physical E-stop, so `/safety/kill` becomes the only stop.
The velocity clamps bound SPEED, not PROXIMITY.

What is unaffected, and still covered by the tiers above: every velocity clamp,
every acceleration limit, the policy `vy` check, kill and killed-on-startup,
and all three staleness watchdogs. `min_range` is still measured and published.

Conditions under which the override is acceptable, per the operator decision:

* operator present, watching the robot, with `/safety/kill` typed and ready
* never unattended, and never combined with a raised `max_vx`
* the default (`true`) is restored by dropping the launch arguments

Pinned by `tests/unit/test_safety_zone_switches.py` (26 tests) and
`tests/unit/test_safety_node_zone_switches.py` (15 tests): defaults enabled,
AND composition, everything that must survive, the latch release, the reason
strings, and runtime rejection.

### 3b. LIMITS IN FORCE ARE ABOVE THE LAST SIGNED VALUES

Raised 2026-08-29 at operator request to make the NCP debug test run at a
watchable pace, ahead of any sign-off:

| | signed-off basis | in force |
|---|---|---|
| `max_vx` | 0.15 m/s | **0.25** |
| `max_wz` | 0.4 rad/s | **0.8** |
| `slow_zone_m` | 0.80 m | **0.65** |

The slow zone was widened back from an interim 0.50 specifically to pay for
the higher speed. Stopping distance at 0.25 m/s is ~0.117 m (0.042 braking +
0.075 travelled during the 3-scan noise filter) against a 0.30 m band — about
2.5× margin. **`max_vx` and `slow_zone_m` are coupled: raising one requires
redoing that sum.** Revert to 0.15 / 0.4 for unattended or long-running use.

Threshold ordering that must be preserved, policy reacting before the safety
layer has to: **0.90 m** (debug policy `obstacle_dist`) > **0.65 m** (safety
scales) > **0.35 m** (safety zeroes and latches).

### 3d. CLAMP SEMANTICS CHANGED 2026-10-09 — NOT SIGNED OFF

`clamp_velocity` now scales the whole command by one factor instead of
capping each axis on its own. Measured cause: the M8-v2 policy asks for
~1.2 m/s with ~0.36 rad/s (a 3.3 m arc) and the per-axis clamp turned that
into 0.25 m/s with 0.36 rad/s (a 0.7 m circle), so the robot looped on the
floor where the policy meant to arc. Uniform scaling is never looser than
per-axis (the factor is bounded by every axis's cap, and a per-axis clamp
still guards rounding), so every stopping-distance sum above still holds.
The slow zone keeps scaling `vx` alone so the robot can turn away from an
obstacle. Unit tests pin: curvature preserved, never looser, non-finite axes
zeroed. Hardware rows for this change are unsigned like the rest of §3.

## 4. Acceptance criteria

- [ ] Every test above passes, recorded with date and operator.
- [ ] Verified by inspection that nothing other than `rm_safety` publishes `/cmd_vel`.
- [ ] E-stop latches and requires explicit `/safety/reset`.
- [ ] Limits in force at sign-off recorded below, and §3b reconciled — either
      the raised values are signed off on their own evidence, or reverted.

## Sign-off

```
Signed off by:  ____________________
Date:           ____________________
Limits in force at sign-off (config/safety_limits.yaml commit): ____________
```
