# Interface contract — version 3.0.0 (SUPERSEDED)

> **Superseded by [`interface_contract.md`](interface_contract.md) (4.0.0, 2026-10-08):**
> 720 lidar beams over 270° and the M8-v2 checkpoint. Kept because the
> rationale for everything 4.0.0 leaves unchanged (the `0.0` no-return
> sentinel, the two clip distances, goal normalization, the literal `3.14`,
> the authority gap) is written here and linked from there. Numbers specific
> to the 32-beam layout (36 dims, 5.625° bins, 16% dropout-centre bins) are
> historical.


**Machine-readable source of truth: [`../config/interface.yaml`](../config/interface.yaml).**
If the two ever disagree, the YAML wins and this file is the bug.

The 2.0.0 draft is kept at
[`interface_contract_2.0.0_superseded.md`](interface_contract_2.0.0_superseded.md)
for history. It described a 41-dim observation that was **never implemented**.

---

## What changed, and why

| version | observation | action | driver |
|---|---|---|---|
| 1.0.0 | 48 dims (36 lidar + 3 base vel + 3 prev action + IMU) | 3 dims | initial draft |
| 2.0.0 | 41 dims (36 lidar + 3 base vel + 2 prev action) | 2 dims `[vx, wz]` | operator decisions: drop lateral strafe, drop IMU |
| **3.0.0** | **36 dims (32 lidar + 2 goal + 2 prev action)** | 2 dims `[vx, wz]` | **the trained checkpoint arrived and it defines the contract** |

3.0.0 is not a design choice. A trained checkpoint exists, and the observation
layout, every normalization constant, and the action bounds are **frozen by
what that network was trained against**. This document describes the
checkpoint; it does not propose anything.

> ### The 36 that is not the other 36
>
> **36 is the TOTAL observation width in 3.0.0.** It was also the **lidar bin
> count** in 2.0.0. These are unrelated numbers that happen to coincide. In
> 3.0.0 the lidar block is **32**.
>
> **Any code or document saying "36 bins" is pre-3.0.0.**

`policy_node` reads `interface_version` at startup and **refuses to construct**
on a mismatch. A layout disagreement between the model and the code has no
runtime error otherwise — it just produces wrong numbers, quietly, forever.

---

## Observation — 36 dims, float32

Order is load-bearing.

| slice | block | dims | source | units |
|---|---|---|---|---|
| `obs[0:32]` | lidar ranges | 32 | `/scan` | normalized, [-1, 1] |
| `obs[32:34]` | goal `(x, y)` in the **robot** frame | 2 | `/odom` pose, or TF `map → base_footprint` | normalized |
| `obs[34:36]` | previous action | 2 | internal node state | normalized |

**Every block is normalized** — unlike 2.0.0, there is no raw-units block left,
because the `base_velocity` block that carried raw units was removed. Every
formula is a fixed affine map whose constants came from the training
environment. **No normalizer statistics are fabricated.**

Assembly order is upstream's own — `prepare_scan → local_goal → build_obs` —
and **all the arithmetic is done by the vendored, byte-exact
`vendor/ncp_policy.py`.** `observation_builder.py` contributes only two things
upstream has no equivalent for: reducing a ~2000-point 360° TG30 scan to 32
forward beams, and fail-closed validation. **No formula is reimplemented.**

---

### Block 1 — lidar, `obs[0:32]`

| property | value |
|---|---|
| arc | forward **180°** only (the robot is blind behind, by choice) |
| bins | **32**, 5.625° each |
| ordering | **right → left** (ascending angle; bin 0 is starboard) |
| sampling | **`nearest_valid_ray`** — not min-pooling |
| resample range | `range_min` 0.01 m, `range_max` 30.0 m |
| normalization clip | **`laser_clip_m` 5.0 m** |
| normalization | `clip(r, 0, 5) → (r − 2.5) / 5 × 2`, giving [-1, 1] |
| empty-bin policy | **`fault`** — never `range_max` |

−1 means "touching"; +1 means "clear to the clip distance". **This is BarnEnv's
formula and must not be altered.**

#### `nearest_valid_ray`, not min-pooling

The policy trained on `--beams 32 --fov 180`: **exactly one ray per beam
direction.** The TG30 delivers ~31 real rays per output bin. We take the single
**real return nearest the bin's centre angle**, reconstructing what a clean
32-beam lidar would have reported.

Min-pooling reports obstacles closer and wider than any 32-beam lidar would.
Measured against `nearest_valid_ray` on real TG30 scans, it shifts the
normalized observation by **0.03–0.19 mean** on a [-1, 1] range, with
individual bins differing by up to **1.05**. That is a systematic distribution
shift away from training, not a rounding detail.

2.0.0 specified `pooling: min` for a 36-bin layout that was never implemented.
It is **superseded, not violated**.

> **This fidelity is only affordable because this block feeds the POLICY
> ONLY.** `rm_safety` computes its e-stop independently from the
> **full-resolution** scan, so a thin obstacle falling between sampled rays is
> invisible to the policy — as it was in simulation — but still stops the
> robot.
>
> **NEVER wire this block's output into a safety path.** `min_pool` remains
> available in `scan_resampler` for any consumer that wants conservatism
> instead of fidelity.

#### The `0.0` no-return sentinel — the safety-critical detail

The TG30 encodes a no-return as **`0.0`, NOT `inf`**, and the driver's
`invalid_range_is_inf` flag is **inert on this hardware** (measured 2026-08-27:
1982 points, 781 zeros = 39.4%, **0 inf**, 0 NaN).

Untreated, `0.0` reads as an obstacle at zero distance, min-pools to 0, and
would **latch the e-stop in every bin containing a dropout**.

`scan_resampler` maps **all** of `0.0`, negative values, anything `< range_min`,
`+inf`, `−inf` and `NaN` to `range_max` **before** pooling. **This mapping is
the only defence in the policy path.** The earlier "belt-and-braces" advice to
also launch with `invalid_range_is_inf:=true` is **retracted** — it does
nothing on this driver.

"**Real** return" is load-bearing in `nearest_valid_ray`. Accepting whatever
ray sits nearest the centre would inherit the ~39% dropout rate: measured on
`tg30_open_scan.yaml`, **5 of 32 bins (16%)** have a dropout as their centre
ray and would report `range_max` ("clear") while neighbouring rays in the same
bin see a real obstacle at 0.5–3 m. Simulation had **no dropout**, so skipping
dropouts in favour of the nearest real return is the **more faithful** choice,
not a safety compromise. A bin with no real return anywhere falls back to
`range_max` — genuinely nothing detected.

#### Two clip distances, both correct

- **`range_max_m: 30.0`** — the sensor ceiling used for resampling, so a
  no-return maps to something unambiguously far.
- **`laser_clip_m: 5.0`** — the **training** clip, applied by `build_obs`,
  which defines the normalization. Anything beyond 5 m was indistinguishable to
  the policy during training and must stay indistinguishable now.

A no-return therefore resolves to 30 m, is clipped to 5.0, and normalizes to
**+1.0 ("clear")**. **Do not reconcile these two numbers.**

#### Self-occlusion

`self_occlusion_zones_deg: []` — **none.** The operator confirmed against live
scans (2026-08-14) that there is no persistent close return anywhere in the
forward 180° arc. `camera_link` sits 7.2 cm below `laser_link`, so it likely
never occluded the scan plane. The previous team's Astra-Pro-era values are
kept as a comment for reference only.

Note `config/safety_limits.yaml` **does** carry occlusion zones — those are the
chassis standoffs at ±90–135°, outside the forward arc, and are a separate,
accepted decision for the *safety* path.

---

### Block 2 — `goal_local`, `obs[32:34]`

| property | value |
|---|---|
| source | `/odom` **pose** (`pose_source: odom`) or TF `map → base_footprint` (`map`) |
| frame | fixed in the pose frame at start |
| default offset | `[2.5, 0.0]` m — 2.5 m ahead, no lateral offset |
| normalization | `xy / 5.0 − 1.0` |
| failure mode | **fail-closed. Never substitute a default.** |

The goal is a **static point**, fixed **once** from the first usable pose as
`start_pose + (goal offset rotated by start yaw)`. The **observation** is
dynamic: every tick, `local_goal()` re-expresses that fixed point in the
robot's **current** frame. So the policy never sees an absolute location, only
"where the goal is relative to me, right now" — equivalent to
`(dist, bearing)`.

#### The normalization is only valid ahead and near

`xy/5 − 1` maps cleanly to [-1, 1] **only for coordinates in [0, 10] m**. A
goal 45° off the bow gives **−1.71**; a goal directly behind gives **−2.00**.

BARN spawned goals ahead and within ~10 m, so the policy **never saw those
values in training.** Turning away from the goal — or overshooting it — pushes
this block **out of distribution**. That is a property of the trained
checkpoint, not a bug to fix here; fixing it means retraining. **Keep goals
modest and ahead.**

This is also why `policy_node` **latches** on goal-reached: past the goal the
observation is off-distribution, so post-overshoot behaviour is not evidence
about the policy.

#### Odometry drift

`/odom` is dead reckoning — an EKF over wheel encoders and the IMU, with no
map, no global localization and no loop closure. It drifts, mecanum rollers
slip constantly, and it **over-reads linear distance by ~5%** (measured). Since
the goal is defined in that same drifting frame, "distance to goal" degrades
over a run and is fiction at long range. The odom origin also resets when the
EKF restarts, so a goal is only meaningful **within one run**. 2.5 m is chosen
to sit inside the drift budget.

**This is not a navigation system.**

`pose_source: map` (slam_toolbox) adds a correcting frame above the EKF so the
goal stops migrating. It does **not** repair the odometry. See
[`architecture.md`](architecture.md) §8.

#### Fail-closed

If no pose has arrived, the goal is **not yet defined** and the policy **must
not drive**. **Do not substitute a default vector** — a plausible-looking goal
is worse than no goal, because it silently commands motion in an arbitrary
direction.

---

### Block 3 — `previous_action`, `obs[34:36]`

The action emitted on the previous tick, normalized to [-1, 1] by
`(a − bias) / scale`, where `bias = (high + low)/2` and `scale = (high − low)/2`:

```
v_norm = (v − 0.5) / 1.5
w_norm = w / 3.14
```

**Initialised to a zero ACTION on the first tick**, which normalizes to a
**non-zero vector** (`v=0 → −0.333`, `w=0 → 0.0`). **Do not "fix" this to a
zero vector.**

#### The requested action is fed back, not the achieved one

**OPEN QUESTION, and it is now informed by live data.** `rm_safety` may clamp
or zero the command, so what the robot *did* is not what the policy asked for.

- Feeding back the **requested** action keeps the observation distribution
  identical to training (which had no safety layer, so emitted == applied).
  This is what upstream does and what we do.
- Feeding back the **achieved** action is off-distribution.

The accepted consequence: during any clamped period the policy believes it is
moving faster than it is.

**Measured behaviour (2026-08-29):** stationary, this feedback produces a
**period-2 limit cycle in `wz`** — alternating roughly −0.36 / +1.07 every tick
— which `rm_safety`'s acceleration limiter filters to a ±0.15 wobble around
zero. Proven by A/B against the live observation: with the block held at zero
it settles to a steady `wz ≈ −0.15`. **It largely resolves in motion** — fed a
goal updated each tick as a moving robot would, it holds a steady `wz ≈ +0.98`
for 14 consecutive ticks.

So the limit cycle is an artifact of a **frozen** observation, i.e. exactly the
condition of a dry-run bench test. **Do not judge yaw behaviour from a
stationary run.** Left as upstream has it — the decision is the operator's.

---

## Action — 2 dims, float32

| # | name | range | units |
|---|---|---|---|
| 0 | `cmd_vx` | `[-1.0, 2.0]` | m/s |
| 1 | `cmd_wz` | `[-3.14, 3.14]` | rad/s |

| property | value |
|---|---|
| activation | **none** — a linear head + clip, NOT tanh |
| `pre_scaled` | **true** |
| `scaling.applies` | **false** |

**The action is already in physical units.** The trained head produces final
m/s and rad/s. Multiplying by any additional scaling in `policy_node` is a
**silent numerical bug with no runtime error.**

> ### `3.14` is not `math.pi`, and that is not a typo
>
> The yaw bound is **literally 3.14**, not 3.14159…. That is what
> `ncp_policy.ACTION_HIGH` contains and therefore what the network was trained
> against: `_ACTION_SCALE = 3.14` exactly, and `previous_action` is normalized
> by dividing by it.
>
> **Do NOT "correct" this to `math.pi`.** It looks like a typo; it is not.
> Replacing it changes the observation the policy sees by ~0.05% with no error
> anywhere — precisely the kind of silent drift that invalidates a checkpoint.

### No lateral strafe

The action space has no `vy` component (operator decision, 2026-08-12). Three
things follow, and none may be "tidied":

1. **`policy_node` sets `linear.y = 0.0` EXPLICITLY on every published Twist**,
   rather than leaving the field at its default. A structurally-zero field is a
   claim the node makes, and it should be visible in the code and in the bag.
2. **`rm_safety` keeps its `max_vy` clamp and lateral e-stop arcs.**
   `/teleop/cmd_vel` is also an input to it, and an operator may legitimately
   strafe while repositioning. The safety layer still has to clamp an axis the
   policy never uses.
3. **A non-zero `linear.y` on `/policy/cmd_vel` is a BUG, not a command** —
   zero it and warn, never clamp it silently.

The base is still mecanum and the driver still accepts `linear.y`. This
constrains the **controller**, not the robot.

### The authority gap is intentional

| | policy range | safety clamp | ratio |
|---|---|---|---|
| `vx` | 2.0 m/s | 0.25 m/s | **8×** |
| `wz` | 3.14 rad/s | 0.8 rad/s | **3.9×** |

**This is correct and the clamps must not be raised to close it.** The safety
layer is deliberately the binding constraint; raising it is gated on Phase 2
hardware sign-off. Both current figures are already **above** the last
signed-off values (0.15 / 0.4) — revert for anything unattended.

**Correction to an earlier expectation in this document:** it predicted the
policy would run "almost entirely saturated". On a **real** TG30 scan it does
not — forward `v` comes out **0.23–0.42 m/s**, scaling sensibly with goal
distance (1 m goal → 0.23, 8 m → 0.42), and `wz` runs ~0.1–0.2 against a 0.8
clamp, i.e. not clamped at all.

> **Probe this checkpoint with a REAL scan, never a synthetic one.** An
> all-beams-at-5.0 m synthetic scan drives the policy far off-distribution and
> produces alarming full-reverse/full-yaw output. That is the **probe** being
> wrong, not the policy. Fixtures are in `tests/fixtures/`; note
> `tg30_open_scan.yaml` is **multi-document** — use `yaml.safe_load_all`.

---

## Control rate — 5 Hz

**Decided**, not inherited. All sensors run ~10 Hz (`/scan` measured
10.07–10.17 Hz, std dev ~1.5 ms), so a 10 Hz control rate has **no headroom**.
5 Hz gives a 2:1 margin and matches the previous team's stack on the same
hardware.

This is the **policy** rate. It is a different number from `rm_safety`'s 20 Hz
republish rate, which exists to guarantee continuous zeros when killed. Do not
conflate them.

---

## Coordinate frames

```
odom -> base_footprint -> base_link -> { imu_link, camera_link, laser_link }
```

REP-103 throughout: **x forward, y left, z up, yaw CCW-positive.**

| frame | transform from `base_link` | note |
|---|---|---|
| `laser_link` | `[0.043, 0.000, 0.110]` m, **no rotation** | lidar 0° == robot forward, so the forward-180° arc maps onto scan angles with **no offset** |
| `imu_link` | `[-0.060, 0.010, 0.010]` m, rpy `[-180, 0, -90]°` | see below |

`/odom`'s `child_frame_id` is **`base_footprint`, not `base_link`**.

With `pose_source: map`, slam_toolbox adds `map → odom` above this chain.

### The `imu_link` rotation is still live

Dropping the IMU from the observation in 2.0.0 **sidestepped** the 180/0/−90
rotation — it did not resolve it. The EKF still fuses `/imu/data` for yaw and
yaw-rate, so that rotation still reaches `/odom`, and therefore still reaches
`goal_local`. Anyone adding an IMU channel back to the observation inherits the
unresolved question.

---

## Version assertion

`interface_version: "3.0.0"`.

`policy_node` compares it against `_EXPECTED_INTERFACE_VERSION` at startup and
**raises rather than starting** on a mismatch. Any change to the observation
layout, the action layout, or a normalization constant is a version bump.

Preprocessing details that do not change the vector layout — the occlusion mask
going to `[]`, for example — are not bumps.

Related invariants pinned by tests:

- `tests/unit/test_vendor_is_pristine.py` — the vendored `ncp_policy.py` still
  matches its upstream hash.
- `tests/unit/test_ncp_policy_vendored.py` — the constants in this contract
  still match the vendored module's (including the `3.14` check).
- `PolicyRunner.load_state_dict` stays **`strict=True`**, so a checkpoint that
  disagrees with the network shape is a **startup crash**, not a
  wrong-numbers run.
