> ## SUPERSEDED — historical only
>
> This is the **2.0.0 draft**, describing a **41-dim observation that was never
> implemented**. It was written before a trained checkpoint existed, when the
> interface was a design proposal rather than a description.
>
> **The live contract is [`interface_contract.md`](interface_contract.md)
> (version 3.0.0, 36 dims), and `../config/interface.yaml` is the authority.**
>
> Kept because the reasoning behind two operator decisions is recorded here and
> nowhere else: dropping lateral strafe from the action space, and dropping the
> IMU block from the observation. Both still hold in 3.0.0.
>
> **Do not implement anything from this file.**

# Interface contract (human-readable)

Machine-readable source of truth: `config/interface.yaml`. If the two ever
disagree, the YAML wins and this file is the bug.

**Status: DRAFT, version 2.0.0, not yet frozen.** Freezing happens in Phase 3,
which blocks on Phase 1 measured sensor rates.

**Changed in 2.0.0** (operator decision, 2026-08-12), both breaking:
- Action is **2 dims `[vx, wz]`** — lateral strafe removed from the policy's
  action space to shrink the exploration problem. The base is still mecanum.
- **IMU block removed** from the observation. 48 → 41 dims.

## Context

No policy has been trained yet and no task is defined. This interface exists to
unblock Phases 1–3 (sensor bring-up, safety layer, schema freeze) ahead of
training. Phases are exercised with a random or hand-scripted placeholder
policy, not a trained one.

Consequences of that, deliberately: no goal pose, no localization source, no
camera and no depth in the observation. Adding any of them is an
`interface_version` bump.

## Observation — 41 dims, float32

Order is load-bearing. Built in exactly this order by the shared schema loader.

| # | Component | Dims | Source | Units |
|---|---|---|---|---|
| 1 | Lidar ranges | 36 | `/scan` | normalized [0,1] |
| 2 | Base velocity (vx, vy, wz) | 3 | `/odom` | m/s, m/s, rad/s |
| 3 | Previous commanded action (vx, wz) | 2 | internal node state | [-1,1] |

Base velocity keeps all three components even though the action space no longer
has `vy`. Observed lateral velocity is now purely a *disturbance* signal —
roller drift, a push, a caught wheel, a slope — and with the IMU gone it is the
only place that information enters the observation. Symmetry between the
observation and action spaces is not a goal; don't "tidy" it to 2 dims.

### Lidar preprocessing

**The sensor is a 360° RPLidar A1**, not a forward-only unit. This is confirmed
from the previous team's `cpt_b2_engine`, which decomposed the full scan into
FRONT/RIGHT/REAR/LEFT sectors and used the rear sector for reverse-motion and
rotation obstacle avoidance.

Restricting the observation to the forward 180° is therefore our **deliberate
choice**, not a hardware constraint — the rear half is discarded, not absent.
Re-confirmed by the operator on 2026-08-07 after the 360° finding came to
light. Widening the arc later is an `interface_version` bump, and the data is
there if we want it.

The robot is consequently **blind behind** for both the policy and the e-stop.
The safety layer must treat reverse motion as unguarded — that is the reason
`safety_limits.yaml` monitors only forward and lateral arcs, and it is an
argument for keeping reverse limits especially low.

That arc is min-pooled into 36 bins of 10° each. Min, not mean: a narrow
obstacle inside a 10° sector must survive pooling. An average would dilute a
table leg into open space, which is exactly the failure that hurts. (The
previous team used *mean* range for their CPT sensor channel — different
purpose, and not a precedent to copy for a safety-relevant observation.)

#### Self-occlusion — the thing that will bite

The lidar sees the robot's own body at several angles. The previous team
calibrated three zones from a diagnostic scan on 2026-02-23:

| Zone | What it is |
|---|---|
| **−6° to +6°** | **Astra Pro camera body, directly ahead of the lidar** |
| 18° to 24° | cable/bracket at ~20° |
| 38° to 41° | structure at ~39° |

The first one is dead centre of our forward arc, in the most safety-critical
bin. Unmasked, a min-pool over that bin returns the distance to the robot's own
camera — a permanent phantom obstacle straight ahead, which would latch the
e-stop the instant it is enabled and would teach any policy trained on this
observation that there is always a wall in front of it.

They also found rear-sector data corrupted by self-occlusion badly enough to
drop it from their sensor channel entirely, while keeping it for safety. Our
forward-only choice sidesteps that, but does not sidestep the front zone.

These values are re-verified against **our** unit in Phase 1 from a static scan
in open space — mounting differs between builds.

#### Order of operations, and it matters

1. Take the forward 180° arc.
2. **Drop rays inside the self-occlusion zones.**
3. Clip to `[range_min, range_max]` (datasheet limits of the fitted lidar —
   TODO Phase 1, cross-check against a live `LaserScan`).
4. Replace non-finite values (`inf`/`NaN`, i.e. no-return) with `range_max`.
   A no-return means "nothing out there", not "obstacle at zero" — getting this
   backwards turns open space into a permanent e-stop.
5. Min-pool into 36 × 10° bins.
6. Divide by `range_max` → [0, 1].

Substituting non-finites *before* pooling is what keeps NaN out of the
observation vector entirely.

**A bin left with zero valid rays after masking is a fault, not "clear."** The
observation builder returns `None` and the safety layer treats it as a stale
scan. Filling an empty bin with `range_max` would be fail-open and is
forbidden. At 10° bins over [−90°, +90°] no zone covers a whole bin — the
widest takes 60% of the two bins adjacent to 0° — so this should never fire.
It is asserted rather than assumed.

### Previous action

The action the policy last emitted, in network output space (pre-scaling),
zeros on the first cycle and after any reset.

It is **not** read back from `/cmd_vel`. The safety layer may have clamped that
value, and feeding a clamped command back as "the previous action" would
silently shift the observation distribution away from training.

### Normalization

Only the lidar block is normalized, by the fixed `/range_max` scaling above.
The base velocity component stays in raw physical units.

No learned normalizer exists yet because no training has happened. Placeholder
mean/std statistics are **not** to be fabricated — a wrong normalizer is far
worse than none, because it fails silently. When training begins and a running
mean/std normalizer is used, its statistics are exported alongside the ONNX
model and loaded by the inference node. The hook for that is marked with a TODO
in `src/rm_policy/rm_policy/observation_builder.py`. Adding it is an
`interface_version` bump.

## Action — 2 dims, float32

| # | Name | Network range | Scale | Physical |
|---|---|---|---|---|
| 1 | `cmd_vx` | [-1, 1] tanh | `vx_max` = 0.3 | m/s |
| 2 | `cmd_wz` | [-1, 1] tanh | `wz_max` = 0.5 | rad/s |

`physical = tanh_output * max`.

### No lateral strafe — what that does and doesn't mean

The X3 is mecanum and physically omnidirectional. Removing `vy` is a
constraint on the **controller**, chosen to shrink the exploration problem, not
a statement about the hardware. Three consequences that are easy to get wrong:

1. **The policy node sets `linear.y = 0.0` explicitly** on every published
   Twist rather than leaving the field at its default. The zero is a claim the
   node makes; it should be visible in the code and in the bag.
2. **`rm_safety` keeps its `max_vy` clamp.** `/teleop/cmd_vel` is also an input
   to the safety layer and an operator may legitimately strafe while
   repositioning. Deleting lateral handling from the safety layer because "the
   policy doesn't strafe" would leave the teleop path unclamped on an axis the
   robot can physically drive.
3. **A non-zero `linear.y` on `/policy/cmd_vel` is a bug, not a command.** The
   safety layer zeroes it *and warns* rather than clamping silently — silent
   clamping would hide a real interface mismatch, e.g. a stale 3-dim model
   loaded against a 2-dim contract.

The lateral e-stop arcs in `safety_limits.yaml` are likewise retained: the
robot can still be pushed or drift sideways on its rollers, and monitoring is
not conditional on the policy's action space.

Those maxima are **provisional and deliberately conservative**, matching the
safety layer's start-low principle. They are raised only after Phase 2 safety
validation is signed off in `docs/safety_validation.md`. They live in
`config/interface.yaml`, never hardcoded in a node.

Note the safety layer's own clamps (`config/safety_limits.yaml`: 0.25 vx /
0.4 wz) are *tighter* than the action scaling. That is intentional — the safety
layer is always the binding constraint, so a mistake in the interface scaling
cannot outrun it.

## Base velocity source — resolved (Phase 1, 2026-08-08)

**`/odom`.** It exists — published by the `robot_localization` EKF that
`yahboomcar_bringup_X3_launch.py` launches, fusing wheel encoders with the IMU.
The previous team simply never used it.

Chosen over `/vel_raw` because `nav_msgs/Odometry` carries a header and a bare
`geometry_msgs/Twist` does not. Staleness inferred from arrival time at the
subscriber cannot distinguish a stalled driver from a stalled network — and
"the driver died but the last velocity looks recent" is precisely the case the
watchdog exists to catch.

## IMU — removed from the observation (2.0.0)

There is no IMU block any more. The 6 raw IMU dims were dropped in 2.0.0.

The IMU's contribution has not vanished: `/odom` is EKF-fused from the encoders
*and* this IMU, published with `child_frame_id: base_footprint`, i.e. already
rotated into the body frame by `robot_localization`'s own TF lookup. What is
genuinely given up is high-frequency linear acceleration and roll/pitch rate —
the least valuable part of the vector for a flat-ground indoor base.

This also retired a live blocker. `imu_link` is rotated RPY −180°/0°/−90° from
`base_link`, so those 6 dims were the only part of the observation in a
non-`base_link` frame, and feeding them in unrotated would have silently
mislabelled axes and flipped at least one sign.

**That rotation is sidestepped, not solved.** It remains live for the EKF
(which handles it correctly, which is exactly why `/odom` is safe and raw
`/imu/data_raw` was not), and it comes back the moment anyone re-adds a raw IMU
channel. Re-adding the IMU is an `interface_version` bump **and** requires
resolving the rotation first. The `imu_link` entry in `interface.yaml` stays
for that reason — do not delete it as dead config.

## Control rate

**TODO(phase1).** Must be at or below the slowest sensor in the observation,
with headroom.

Strong prior: the previous team's engine ran its decision cycle at
**`PIPELINE_HZ = 5`**, with a 200 ms latency target and 400 ms flagged as "very
slow", on this exact hardware. The RPLidar A1 spins at 5.5–10 Hz. The answer
almost certainly lands in 5–10 Hz — but Phase 1 task 4 measures it rather than
inheriting it.

Reasoning to be recorded here, against the measured numbers, before the
interface is frozen. If lidar measures 10 Hz, control does not run at 50 Hz —
that would feed the policy four stale scans out of every five.

## Version assertion

The policy node reads the `interface_version` embedded in the exported model's
metadata and refuses to start if it does not match `config/interface.yaml`.
No model exists yet, so this path is implemented against a stub and will be
verified against a real artifact in Phase 4.
