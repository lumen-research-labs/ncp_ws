# Calibrating `rm_base` on a new robot

**The numbers in `config/base_calibration.yaml` do not transfer.** They were
fitted to bernard-1 with a tape measure, and one of them compensates for a
known-bad motor on that specific unit. Copying them onto another ROSMASTER X3
will produce a robot that drives — badly, and in a way that looks like a
controller problem rather than a calibration one.

This document is the procedure. The *values* and the reasoning behind each one
are documented in `config/base_calibration.yaml` itself; read that file
alongside this one.

Everything here needs **an operator present with `/safety/kill` typed and ready
in its own terminal**. There is no physical E-stop on this platform.

---

## 0. First decide whether you need `rm_base` at all

`rm_base` exists to work around a **firmware fault on bernard-1**: the STM32's
per-wheel speed PID reads its encoders with the wrong sign, so `err` never
shrinks, the PWM integrates to the rail, and every nonzero `/cmd_vel` runs the
wheel at ~100% regardless of magnitude. See CLAUDE.md 2026-08-28 session 2.

**Check the new robot before assuming it has the same fault.** Wheels up,
vendor driver only:

```bash
python3 scripts/diagnostics/wheel_diag.py
```

Command 0.02, 0.05 and 0.15 m/s in turn. If the wheel speed scales with the
command, **the firmware is healthy and this whole document is optional** — use
the vendor driver and skip to §6 to verify. If all three produce the same
flat-out spin, the fault is present and `rm_base` is the path.

Confirm the diagnosis rather than inferring it:

```bash
python3 scripts/diagnostics/motor_openloop.py   # raw PWM, no PID, no feedback
```

Open loop being clean while closed loop rails is the signature. It also prints
the stored PID gains — on bernard-1 they were stock, so a factory reset does
not help.

---

## 1. Map motor channels to physical wheels

**Do not assume the Yahboom sample order.** On bernard-1 the map is
`M1=front-left, M2=back-left, M3=front-right, M4=back-right`, which is *not*
the documented `FL, FR, BL, BR`.

```bash
python3 scripts/diagnostics/per_motor.py     # wheels UP
```

Drives one motor at a time. For each, record which wheel turns and which way
`set_motor(+)` drives it. Record separately whether `get_motor_encoder` counts
**up or down** for forward rotation — on bernard-1 forward is *decreasing*, and
`rm_base` negates the delta to correct it.

Write both into `rm_base/pwm_map.py` and the calibration file. Getting this
wrong does not fail loudly; it produces a robot that drives diagonally when
told to go straight.

---

## 2. Prove the closed loop wheels-UP before touching the floor

```bash
bash scripts/prep_robot.sh
bash scripts/bench_up.sh --mode closed_loop --live
python3 scripts/diagnostics/closed_loop_check.py
```

What must be true before you put it down:

- **No `sign_fault` on any wheel in either direction.** A sign fault means the
  encoder polarity assumption in §1 is wrong. Stop and fix that first.
- Mecanum kinematics behave: `vx=0.05, wz=0.3` should give the left wheels
  roughly `vx - wz*L` and the right `vx + wz*L`, right-faster for CCW.
- Encoder feedback fresh on most ticks. `get_motor_encoder` returns a value
  cached by the auto-report thread, so gaps are expected; a not-fresh read
  degrades that wheel to feedforward rather than fabricating a zero.

---

## 3. Find any sick wheel — WHEELS UP, driven directly

**A dead wheel reads like a working one on the floor.** The other three drag
it, its encoder reports the robot's motion rather than its own drive, and it
tracks its target *perfectly*. Suspiciously exact tracking is the tell. This
cost a full session on bernard-1, where front-right was declared healthy on
floor data and then measured 0.0000 m/s wheels-up at PWM 42–49.

```bash
python3 scripts/diagnostics/fr_threshold.py --curve    # wheels UP
```

Despite the name it works on any wheel. It reports two different things and
the difference matters:

- **breakaway** PWM — what it takes to *start* a stopped wheel
- **running** dead/gain — what it takes to *keep it going*

If the two are close, the wheel is normal: fit the running numbers and move
on. If breakaway is roughly double the running requirement, the wheel is
**sticky**, and it needs `breakaway_pwm` / `breakaway_pwm_rev` set as a bounded
kick. **Do not put the breakaway figure in `dead_pwm`** — the wheel would then
be commanded ~66 PWM for a 0.05 m/s target and take off at ~0.28 m/s.

A wheel whose *running gain* is normal has torque; it has stiction, not
weakness, and does not need replacing.

---

## 4. Fit `dead_pwm` and `gain_pwm_per_mps` — wheels DOWN, two STRAIGHT legs

```bash
python3 scripts/diagnostics/fit_reverse.py      # wheels DOWN, tape measure
```

Four rules, each of which was learned by getting it wrong:

1. **Two speeds, minimum.** One speed cannot separate `dead_pwm` from
   `gain_pwm_per_mps` — infinitely many wrong splits fit a single point.
   bernard-1 used 0.06 and 0.12 m/s.
2. **Never include a TURN in the fit.** A spin carries rotational scrub load,
   and the fit misattributes that extra PWM to the dead-band: `dead_pwm` comes
   out far too high, gain far too low, and the model then matches only at the
   speed it was fitted at. A robot fitted this way drove 1.00 m perfectly and
   then turned 79° when asked for 180°.
3. **Err HIGH on `dead_pwm`.** Too much starting push overshoots and the loop
   trims it. Too little and the wheel never starts — and the stall guard
   correctly freezes the integrator, so it stays stuck forever.
4. **Fit against MEASURED speed**, not commanded.

Fit forward and reverse separately. If they agree within ~10% the wheel is
symmetric and no `_rev` overrides are needed. A directionally asymmetric wheel
(bernard-1's back-left turns at PWM 7 forward but is dead until ~16 in
reverse) gets `dead_pwm` set for the **harder** direction; the loop trims the
overshoot in the easier one.

---

## 5. Fit `lever_arm_m` — and know what it is

`lever_arm_m` is **not** the physical half-wheelbase. Mecanum rollers scrub
sideways when turning on the spot, so the geometric value under-rotates: on
bernard-1, wheels running at exactly the speed the geometry asked for produced
139° for a commanded 180°. The constant is a scrub compensation.

Fit it by bracketing, not by calculation: run a spin at your operating `wz`,
read the achieved angle off a floor line with a protractor, adjust, repeat.
Two runs that straddle 1.000 interpolate the answer.

**Fit it at the `wz` your policy actually commands** — usually near `max_wz`,
since a saturating policy lives at the top of the range.

Two things to expect rather than chase:

- **Low `wz` will under-rotate**, by ~20% at half the fitted speed. There is a
  rotational dead-band (~0.107 rad/s lost to scrub on bernard-1) and a pure
  multiplier cannot cancel an offset. Raising the lever to fix low `wz`
  over-rotates high `wz`. The real fix is an affine yaw term, which nobody has
  needed yet.
- **CW/CCW asymmetry** of several percent, if any wheel has direction-dependent
  stiction.

---

## 6. Verify against a tape measure — the acceptance test

```bash
bash scripts/bench_up.sh --mode closed_loop --live --source policy
python3 scripts/diagnostics/velocity_accuracy.py
```

Use `bench_up.sh`, **not `ncp.launch.py`** — the latter always starts a policy
consumer, which publishes on the same topic this test commands on, and the two
would fight mid-leg.

Run several speeds in **both** directions, plus spins both ways. Two design
points in the tool are load-bearing:

- The reference is the **time integral of `/cmd_vel`**, never `vx * secs`.
  `rm_safety` acceleration-limits every command, so the ramp alone is ~4% of a
  5 s leg at 0.25 m/s and would otherwise be booked against your calibration.
- **Never pool forward with reverse, or CW with CCW.** The calibration is
  per-direction; pooling reads a direction difference as "varies with speed"
  and sends you to refit the wrong thing.

Target: achieved/commanded within a few percent on the linear axis across a
3–4x speed range, and near 1.000 on the angular axis *at the speed you fitted*.

**Cross-check before refitting anything.** A spin at `wz` asks each wheel for
`wz * lever_arm_m` m/s. If a straight leg at that same wheel speed is accurate,
the wheels can hit the speed and the yaw shortfall is scrub — not a
`dead_pwm`/`gain` problem. Without that check the obvious and wrong move is to
refit the wheels.

---

## 7. Things that will bite

- **`closed_loop.guards` in the YAML SHADOWS the module defaults** in
  `wheel_control.Guards`. A stale key there silently overrode the code for
  several hardware runs. Change both, or delete the key.
- **`max_vx` is coupled to `max_pwm` and to `slow_zone_m`.** Raising the speed
  limit without raising the PWM ceiling leaves wheels saturated and unable to
  reach the commanded speed. Raising it also means redoing the stopping-distance
  sum for the slow zone.
- **`counts_per_rev` is fitted to floor distance, so wheel SLIP is baked in as
  if it were encoder scale.** Treat `/odom` as "matches the tape", not
  "correct" — on bernard-1 it over-reads linear distance by ~5%. A wheels-up
  revolution count would separate slip from scale; nobody has run one.
- **`/odom` yaw drifts** (~0.6 deg/s at rest on bernard-1, measured
  2026-08-31). Do not calibrate anything against odometry. The tape measure is
  the only ruler in this document, deliberately.
- **Check the runway before every floor leg.** `/scan`'s forward-arc minimum
  tells you whether the slow zone will silently scale `vx` down mid-run and
  corrupt the measurement. A clean run shows `safety reasons: ok` throughout.

---

## Summary — the order

| step | wheels | tool | produces |
|---|---|---|---|
| 0 | up | `wheel_diag.py`, `motor_openloop.py` | is `rm_base` needed at all |
| 1 | up | `per_motor.py` | motor→wheel map, encoder sign |
| 2 | up | `closed_loop_check.py` | no sign faults, kinematics sane |
| 3 | up | `fr_threshold.py --curve` | per-wheel breakaway vs running |
| 4 | down | `fit_reverse.py` | `dead_pwm`, `gain_pwm_per_mps`, per direction |
| 5 | down | protractor | `lever_arm_m` |
| 6 | down | `velocity_accuracy.py` | acceptance: achieved vs commanded |

Steps 0–3 are wheels-up and safe to iterate. Everything from 4 on needs floor
space, a tape measure, and a hand on the kill switch.
