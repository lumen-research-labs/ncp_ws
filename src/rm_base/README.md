# `rm_base` — the mecanum base driver

**The calibration numbers in `config/base_calibration.yaml` are bernard-1's and do NOT transfer to another robot.** To bring the base up on a new unit, follow [`docs/calibration.md`](../../docs/calibration.md).

**Runs on: the robot.** Replaces the Yahboom `Mcnamu_driver_X3`. Owns the
base-board serial link and turns `/cmd_vel` into four wheel PWM values.

> `rm_base` is a pure **consumer** of `/cmd_vel`. It never publishes it.

---

## Why this package exists

**The STM32 firmware's closed-loop speed control is unusable on this robot.**

Measured per-motor on bernard-1 (2026-08-28): every working wheel reports its
encoder with the **wrong sign** relative to what the firmware's PID expects.
Drive a wheel forward and `get_motor_encoder` counts *down*. So
`err = target − measured` never shrinks, `pwm_output` integrates straight to
the ±3600 rail, and the wheel runs at ~100% PWM for **any** nonzero command.

That single fact explains everything that was previously mysterious:

- commanding 0.02 / 0.05 / 0.15 m/s each produced the *same* ~30 wheel-rev per
  10 s (~0.8–1 m/s) — the magnitude is simply thrown away;
- `/vel_raw` reading a constant −1.10 m/s **wheels-up**, uncorrelated with the
  command in magnitude, sign, or axis;
- the phantom yaw.

Gains are stock (`[0.8, 0.06, 0.5]`, firmware v3.6) — **not corrupted, and a
factory reset will not help.** The encoder polarity is not a swappable wire. A
reflash needs the image on a USB stick plus a BOOT0 jumper or ST-Link, on a
robot with no internet, and carries a bricking risk. `rm_base` is the working
workaround and it is proven on the floor, so the reflash is no longer on the
critical path.

Note what this does and does *not* condemn: the firmware's **loop** was fine,
its **feedback polarity** was not. Correcting the sign in ROS makes the same
feedback usable — which is why `closed_loop` mode is possible at all.

---

## What's in here

| File | Kind | What it is |
|---|---|---|
| `mecanum.py` | pure | Body twist ↔ four wheel ground speeds, REP-103. |
| `pwm_map.py` | pure | Wheel speed ↔ open-loop PWM, per wheel, with a dead-band. |
| `wheel_control.py` | pure | Per-wheel feedforward + PI on sign-corrected encoder feedback, with three guards. |
| `base_node.py` | ROS2 node | The serial owner and control loop. |

### Topics

| | |
|---|---|
| Subscribes | `/cmd_vel` (Twist) — the safety-clamped command |
| Publishes | `/imu/data_raw`, `/imu/mag`, `/voltage`, `/edition` — **byte-identical to the vendor driver**, so `imu_filter_madgwick` and the EKF are unaffected · `/vel_raw` — reconstructed from the encoders with the sign bug corrected · `/base/debug` (`rm_interfaces/BaseDebug`) |

---

## The measured motor→wheel map

```
M1 = front-left    M2 = back-left    M3 = front-right    M4 = back-right
```

**This is NOT the Yahboom sample order** (`M1,M2,M3,M4 = FL,FR,BL,BR`). Do not
"fix" it back. For every wheel, a positive `set_motor` value drives it
**forward**, and a forward-turning wheel makes `get_motor_encoder` count
**down** — hence the negation in the odometry path.

Every fixed-size array in `BaseDebug` is indexed `[FL, BL, FR, BR]`.

---

## Two control modes

| mode | what it does |
|---|---|
| `open_loop` | PWM straight from `base_calibration.yaml`. No feedback. |
| `closed_loop` | That same calibration as **feedforward**, plus a per-wheel PI trim on the sign-corrected encoder speed. |

**`closed_loop` is the validated path** (2026-08-29 floor tests) and is what
`ncp.launch.py` defaults to. It exists because open loop gives every wheel the
PWM its calibration row says and they still end up at different speeds under
real load. `base.launch.py` still defaults to `open_loop` — the more
conservative default for base-only bench work.

### The three guards, all failing toward *less* authority

Because the failure mode being worked around is a runaway with no physical
E-stop to catch it:

1. **Anti-windup** — the integrator never grows in the direction that deepens a
   saturation, and is hard-clamped to `i_max_pwm`.
2. **Stall detection** — a wheel commanded to move that reports no motion for
   `stall_ticks` stops integrating and holds feedforward. Without this,
   front-right (whose encoder only reads once it is already turning fast) would
   wind the integrator to the ceiling on every start, **reproducing the
   firmware bug in ROS**.
3. **Latched sign guard** — a wheel measurably turning *against* the command
   means the feedback polarity is wrong, so the loop is unsafe: that wheel is
   forced to zero PWM and the node logs ERROR. This is the tripwire for the
   firmware sign bug reappearing, or being inverted by a future reflash.

The sign guard is gated on the **actually applied PWM**, not the target. The
node slew-limits PWM (8/tick) *after* the controller returns, so coming out of
cruise the wheel is still being driven the old way while the target already
says reverse — judging polarity there condemns the slew limiter, not the
wiring. (That bug latched all four wheels to zero on the first full-stack run.)

`EncoderTracker` handles the fact that `get_motor_encoder` returns a value
cached by the auto-report thread: polling at 20 Hz yields repeated samples, so
an unchanged count is reported **not-fresh** and dt accumulates, rather than
being divided into a fabricated zero speed. Invalid feedback degrades that
wheel to feedforward — i.e. to the validated open-loop behaviour. Feedback is
read in `open_loop` mode too, so `/base/debug` shows the four diverging wheel
speeds even when nothing is correcting them.

---

## Calibration — `config/base_calibration.yaml`

The model is linear with a static-friction dead-band, **per wheel and per
direction**:

```
|pwm| = dead_pwm + gain_pwm_per_mps * |v|      (v above v_min)
pwm   = 0                                       (|v| <= v_min)
```

`dead_pwm_rev` / `gain_pwm_per_mps_rev` are optional and default to "same as
forward", so an uncalibrated wheel keeps its previous symmetric behaviour.

### Hard-won calibration rules

- **The numbers are TAPE-FITTED, not derived.** `wheel_circumference_m` 0.248
  is measured (a 79 mm wheel). `counts_per_rev` 2360 and `lever_arm_m` 0.227
  are *fitted* to make commanded match achieved. **Re-tune against a tape
  measure, never against odometry** — odometry cannot check itself.
- **Fit `dead_pwm` from TWO speeds, and err HIGH.** A one-speed fit cannot
  separate `dead_pwm` from `gain_pwm_per_mps`. Getting it wrong is silent: an
  intermediate fit drove perfectly straight at 0.10 m/s and then produced only
  79° from a commanded 180°, because both right wheels never *started* at the
  0.052 m/s a turn asks of them — and a wheel that fails to start stays stuck,
  since the stall guard (correctly) freezes the integrator. Too much starting
  push is trimmed by the loop; too little means no motion at all.
- **Never fit `dead_pwm`/`gain` from a measurement that includes a TURN.** A
  spin carries rotational scrub load, which the fit misattributes to the
  dead-band. Use two **straight** legs (`scripts/diagnostics/fit_reverse.py`).
- **`lever_arm_m` 0.227 is a SCRUB FUDGE, not a measurement.** Mecanum rollers
  scrub sideways turning on the spot. Do **not** "correct" it back to the
  physical 164.555 mm half-wheelbase. And it **cannot be right everywhere**:
  there is a rotational dead-band (~the first 0.107 rad/s of any commanded spin
  is lost to scrub) and a multiplier cannot cancel an offset. It is fitted at
  `wz` 0.60; **`wz` ~0.30 under-rotating ~20% is expected, not a regression.**
- **`max_vx` is coupled to `max_pwm`.** At 0.25 m/s with the floor-test ceiling
  of 35 three of four wheels ran saturated and could not reach the commanded
  speed. Ceilings are now 60 (FR 95).
- **`closed_loop.guards` in the YAML SHADOWS the module defaults** in
  `wheel_control.Guards`. A stale key there silently overrode the code for
  several hardware runs. Change both, or delete the key. Pinned by a test.

### Front-right (M3) is STICKY, not weak

Settled after a correction and a re-correction — read the history before
reopening.

- breakaway **PWM 60 forward / 55 reverse** — to *start* it turning
- running **dead 30.2 / gain 132.1 fwd, 27.7 / 145.4 rev** — to *keep* it going

Its running gain is normal, so the motor has torque; it needs roughly twice the
PWM to break loose as to sustain. **The RUNNING fit is what is configured** —
using breakaway as `dead_pwm` would command 66 PWM for a 0.05 m/s target and
the wheel would take off at ~0.28 m/s. A bounded **breakaway kick**
(`breakaway_pwm`, `breakaway_pwm_rev`) covers the start: it *replaces* the loop
output with a fixed open-loop nudge and never feeds the integrator, and is
bounded three ways — 8 ticks per attempt, 20-tick cooldown, 3 attempts then it
gives up until the command clears. Holding a blocked motor at breakaway PWM is
how it burns out.

> **A dead wheel reads like a working one on the floor.** The other three drag
> it and its encoder reports the *robot's* motion — front-right showed 0.250
> against a 0.254 target while contributing nothing. Suspiciously *perfect*
> tracking is the tell. Diagnose a suspect wheel **wheels-up** with the motor
> driven directly (`scripts/diagnostics/fr_threshold.py`).

---

## Failure mode

Fail-closed everywhere — there is no physical E-stop.

- Serial port cannot be opened → raise in `__init__`; the node does not start.
- **`dry_run` defaults `true`.** Motion needs `dry_run:=false` **and** an armed
  `rm_safety` — two explicit acts.
- No `/cmd_vel` within `command_timeout_ms` (300) → zero PWM every tick. **The
  vendor driver had no watchdog at all**; this closes the "the robot doesn't
  stop on its own" hazard.
- Any exception in the control tick → `set_motor(0,0,0,0)`, log ERROR, never
  leave the last PWM applied.
- On SIGINT/shutdown → flood stop *before* releasing the serial port.

`/base/debug` is populated in `dry_run` too — everything except `applied_pwm` —
so a wheels-up dry run previews the real command. (The old message packed four
PWM values into a Twist and reported only what was written to the board, so in
`dry_run` it showed four zeros and told the operator nothing.)

---

## Operational note

**The base-board serial port is single-writer.** The vendor
`Mcnamu_driver_X3` and the Yahboom `rosmaster_main.py` factory autostart (which
**recurs every boot**) must both be killed before this node can open it.
`scripts/prep_robot.sh` does that. If something else holds the port, the
driver's serial thread crashes and `/voltage`, `/imu/data_raw` and `/vel_raw`
all read **zero** while the topics keep publishing normally — which looks like
dead hardware, not a conflict.
