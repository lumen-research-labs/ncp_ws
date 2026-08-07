# Interface contract (human-readable)

Machine-readable source of truth: `config/interface.yaml`. If the two ever
disagree, the YAML wins and this file is the bug.

**Status: DRAFT, version 1.0.0, not yet frozen.** Freezing happens in Phase 3,
which blocks on Phase 1 measured sensor rates.

## Context

No policy has been trained yet and no task is defined. This interface exists to
unblock Phases 1–3 (sensor bring-up, safety layer, schema freeze) ahead of
training. Phases are exercised with a random or hand-scripted placeholder
policy, not a trained one.

Consequences of that, deliberately: no goal pose, no localization source, no
camera and no depth in the observation. Adding any of them is an
`interface_version` bump.

## Observation — 48 dims, float32

Order is load-bearing. Built in exactly this order by the shared schema loader.

| # | Component | Dims | Source | Units |
|---|---|---|---|---|
| 1 | Lidar ranges | 36 | `/scan` | normalized [0,1] |
| 2 | Base velocity (vx, vy, wz) | 3 | `/odom` `twist.twist` | m/s, m/s, rad/s |
| 3 | IMU angular velocity | 3 | `/imu` `angular_velocity` | rad/s |
| 4 | IMU linear acceleration | 3 | `/imu` `linear_acceleration` | m/s² |
| 5 | Previous commanded action | 3 | internal node state | [-1,1] |

### Lidar preprocessing

The X3's lidar is **occluded to the rear by the chassis** — only the forward
180° carries usable signal. That arc is min-pooled into 36 bins of 10° each.

Min, not mean: a narrow obstacle inside a 10° sector must survive pooling. An
average would dilute a table leg into open space, which is exactly the failure
that hurts.

Order of operations, and it matters:

1. Take the forward 180° arc.
2. Clip to `[range_min, range_max]` (datasheet limits of the fitted lidar —
   TODO Phase 1, cross-check against a live `LaserScan`).
3. Replace non-finite values (`inf`/`NaN`, i.e. no-return) with `range_max`.
   A no-return means "nothing out there", not "obstacle at zero" — getting this
   backwards turns open space into a permanent e-stop.
4. Min-pool into 36 × 10° bins.
5. Divide by `range_max` → [0, 1].

Substituting non-finites *before* normalizing is what keeps NaN out of the
observation vector entirely.

### Previous action

The action the policy last emitted, in network output space (pre-scaling),
zeros on the first cycle and after any reset.

It is **not** read back from `/cmd_vel`. The safety layer may have clamped that
value, and feeding a clamped command back as "the previous action" would
silently shift the observation distribution away from training.

### Normalization

Only the lidar block is normalized, by the fixed `/range_max` scaling above.
Velocity and IMU components stay in raw physical units.

No learned normalizer exists yet because no training has happened. Placeholder
mean/std statistics are **not** to be fabricated — a wrong normalizer is far
worse than none, because it fails silently. When training begins and a running
mean/std normalizer is used, its statistics are exported alongside the ONNX
model and loaded by the inference node. The hook for that is marked with a TODO
in `src/rm_policy/rm_policy/observation_builder.py`. Adding it is an
`interface_version` bump.

## Action — 3 dims, float32

Mecanum. The lateral DOF `vy` is real and is included; this is not
differential-drive.

| # | Name | Network range | Scale | Physical |
|---|---|---|---|---|
| 1 | `cmd_vx` | [-1, 1] tanh | `vx_max` = 0.3 | m/s |
| 2 | `cmd_vy` | [-1, 1] tanh | `vy_max` = 0.3 | m/s |
| 3 | `cmd_wz` | [-1, 1] tanh | `wz_max` = 0.5 | rad/s |

`physical = tanh_output * max`.

Those maxima are **provisional and deliberately conservative**, matching the
safety layer's start-low principle. They are raised only after Phase 2 safety
validation is signed off in `docs/safety_validation.md`. They live in
`config/interface.yaml`, never hardcoded in a node.

Note the safety layer's own clamps (`config/safety_limits.yaml`: 0.25 / 0.25 /
0.4) are *tighter* than the action scaling. That is intentional — the safety
layer is always the binding constraint, so a mistake in the interface scaling
cannot outrun it.

## Control rate

**TODO(phase1).** Must be at or below the slowest sensor in the observation,
with headroom. Expected to be lidar-bound but that is a guess until Phase 1
task 4 measures it.

Reasoning to be recorded here, against the measured numbers, before the
interface is frozen. If lidar measures 10 Hz, control does not run at 50 Hz —
that would feed the policy four stale scans out of every five.

## Version assertion

The policy node reads the `interface_version` embedded in the exported model's
metadata and refuses to start if it does not match `config/interface.yaml`.
No model exists yet, so this path is implemented against a stub and will be
verified against a real artifact in Phase 4.
