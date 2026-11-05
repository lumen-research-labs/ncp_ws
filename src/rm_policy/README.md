# `rm_policy` — observation building and policy inference

**Runs on: the robot.** Turns raw sensor data into the trained network's
observation, runs the network, and publishes a *candidate* command.

> `rm_policy` publishes its `cmd_vel_topic` (`/policy/<profile>/cmd_vel` under
> `ncp.launch.py`, one node per profile). It has **no route to `/cmd_vel`** —
> that topic belongs to `rm_safety`. `policy_node` refuses to start if its
> `cmd_vel_topic` parameter resolves to `/cmd_vel`.

---

## What's in here

| File | Kind | What it is |
|---|---|---|
| `vendor/ncp_policy.py` | **vendored, byte-exact** | The CfC/AutoNCP network, copied unmodified from the checkpoint's source repo. Never edited. See `vendor/VENDOR.md`. |
| `scan_resampler.py` | pure module | Reduces a ~2000-point 360° TG30 scan to a fixed-length forward-arc vector, in metres. |
| `observation_builder.py` | pure module | Assembles the 724-element observation. Thin: the vendored module does the normalization, and the previous-action block is re-normalized by the profile's action bounds. |
| `policy_runner.py` | pure module | Loads the checkpoint, owns the recurrent state, runs one inference tick, clips to the contract's action bounds. No ROS. |
| `pose_source.py` | pure module | Validates a pose sample and detects discontinuities (SLAM loop closures). |
| `roam.py` | pure module | ROAM mode: the limits in `config/roam.yaml` and the hand-off clock that draws the next random goal on arrival or timeout. See [`docs/roam.md`](../../docs/roam.md). |
| `retreat.py` | pure module | ROAM's manoeuvres: the straight reverse out of a stop-zone e-stop (the one command rm_safety passes under its latch) and the turn in place toward a goal off the nose. |
| `policy_node.py` | ROS2 node | The control loop. The production path. |
| `lidar_contract.py` | pure module | Reads the lidar block of `config/interface.yaml` (bins, arc, range bounds, search window, self-occlusion zones, body floor) for both nodes. |
| `action_contract.py` | pure module | Reads the action block (the trained bounds) of a profile's interface file; 5.0.0 (M9) differs from the vendored constants. See [`docs/interface_versions.md`](../../docs/interface_versions.md). |
| `scan_bridge_node.py` | ROS2 node | `/scan` → `/ncp/scan`, 720 beams. Exists only so the *upstream* node can run unmodified. |
| `scan_sanitizer_node.py` | ROS2 node | `/scan` → `/scan_sanitized`, full resolution, `0.0` → `+inf`, and returns under the body radius (the chassis corners) → `+inf`. For third-party consumers (slam_toolbox). |

---

## The observation — interface 4.0.0, 724 dims

`config/interface.yaml` is the authority. `policy_node` reads
`interface_version` at startup and **refuses to construct** on a mismatch,
because a layout disagreement between the model and this code has no runtime
error otherwise.

| slice | block | source |
|---|---|---|
| `obs[0:720]` | lidar, 720 beams across the forward 270°, right→left | `/scan` |
| `obs[720:722]` | goal `(x, y)` in the **robot** frame | `/odom` or TF `map→base_footprint` |
| `obs[722:724]` | previous action | internal node state |

Every block is normalized by a formula frozen by the trained checkpoint. **No
formula is reimplemented here.** An earlier revision duplicated the
normalization in pure Python so the tests could run without numpy on the
workstation; that was removed on operator instruction — silent drift against
the checkpoint outranks test-runner convenience. The cost is that
`observation_builder`'s tests run on the robot tier.

### Lidar: `nearest_valid_ray`, not min-pooling

The policy was trained on `--beams 720 --fov 270` — exactly **one ray per beam
direction**, with no dropout. The TG30 gives about 2 rays per 0.375° bin and
drops 28–39% of them. `scan_resampler` takes the single **real return nearest
the bin's centre angle**, reconstructing what a clean 720-beam lidar would have
reported. A bin with no real return at all borrows the nearest real return
within ±2 bins (`nearest_ray_search_window_bins` in `interface.yaml`); on the
recorded fixtures that cuts empty bins from 5–8% to about 1.3%. Rays inside
the bin always win, so the window is a fallback, never a smoothing.

The 270° arc also sees the chassis standoffs, which the training sensor's
0.1 m `min_range` hid. `self_occlusion_zones_deg` blanks those sectors before
sampling, so they read "clear" as they did in simulation.

Min-pooling instead would report obstacles closer and wider than the training
sensor did: measured at 32 bins against real TG30 scans it shifts the
normalized observation by 0.03–0.19 mean on a [-1, 1] range, up to 1.05 in
individual bins. That is a systematic distribution shift away from training,
not a rounding detail.

**This is only affordable because this block feeds the POLICY ONLY.**
`rm_safety` computes its e-stop independently from the full-resolution scan, so
a thin obstacle falling between sampled rays is invisible to the policy — as it
was in simulation — but still stops the robot. **Never wire this block's output
into a safety path.** `min_pool` remains available in `scan_resampler` for any
consumer that wants conservatism instead of fidelity.

### The TG30's no-return sentinel is the safety-critical detail

The YDLIDAR TG30 encodes a no-return as **`0.0`, not `inf`** — regardless of
the driver's `invalid_range_is_inf` flag, which is **inert on this driver**
(measured: 1982 points, 781 zeros, **0 inf**). Untreated, `0.0` reads as an
obstacle at zero distance.

`scan_resampler` maps *all* of `0.0`, negative, sub-`range_min`, `±inf` and
`NaN` to `range_max` **before** pooling. This is the only defence in the policy
path. `scan_sanitizer_node` does the same job for consumers we do not own.

"Real return" is load-bearing in `nearest_valid_ray`: accepting whatever ray
sits nearest the centre would inherit the TG30's ~39% dropout rate — 16% of
bins would report "clear" while a neighbouring ray in the same bin sees an
obstacle at 0.5–3 m. Simulation had no dropout, so skipping dropouts is the
*more* faithful choice, not a compromise.

### Two clip distances, both correct

- `range_max_m: 30.0` — the sensor ceiling used for resampling, so a no-return
  maps to something unambiguously far.
- `laser_clip_m: 5.0` — the **training** clip, applied by `build_obs`, which
  defines the normalization. Anything beyond 5 m was indistinguishable to the
  policy during training and must stay indistinguishable now.

**Do not reconcile these two numbers.**

### The goal

A **static point in the pose frame**, fixed once from the first usable pose as
`start_pose + (goal_x, goal_y) rotated by start yaw`. The *observation* is
dynamic: every tick the fixed point is re-expressed in the robot's current
frame.

- **No default goal is ever substituted.** Until a pose has arrived, the goal
  does not exist and the node publishes zero. A plausible-looking goal silently
  commands motion in an arbitrary direction — worse than not driving.
- **The normalization is `xy/5 - 1`**, which maps cleanly to [-1, 1] only for
  coordinates in [0, 10] m. A goal directly behind gives −2.0. BARN never
  produced those values, so **turning away from the goal pushes the policy
  off-distribution.** Behaviour observed after an overshoot says nothing about
  the policy.
- **`goal_x` / `goal_y` are runtime-settable.** A set re-fixes the goal from
  the *current* pose — always, even to an unchanged value, because a set is an
  explicit "re-target now". A new goal is a new episode: the arrival latch
  clears, the recurrent state resets and the fed-back previous action returns
  to zero. `policy_node._fix_goal` is the one place the goal is written.

### Previous action: the *requested* action is fed back

Not what `rm_safety` actually allowed. Training had no safety layer, so
emitted == applied there; feeding back the request keeps the observation
distribution identical to training. The accepted consequence: during any
clamped period the policy's model of "what I last did" disagrees with what the
robot did. This is `interface.yaml`'s open `clamped_action_note` and it is
faithful to upstream — do not "fix" it unilaterally.

**Known behaviour:** stationary, this checkpoint enters a **period-2 limit
cycle in `wz`** (alternating roughly −0.36 / +1.07 every tick), driven by that
feedback, which `rm_safety`'s acceleration limiter filters to near-zero net
yaw. Fed a *moving* goal it resolves to a steady turn. **Do not diagnose yaw
behaviour from a stationary dry-run bench test.**

---

## `pose_source` — odom or map

| value | pose from | behaviour |
|---|---|---|
| `odom` (default) | `/odom` | dead reckoning; drifts, over-reads linear distance ~5% |
| `map` | TF `map → base_footprint` | slam_toolbox-corrected; the goal stops migrating |

In `map` mode a missing, unconnected, un-extrapolatable or stale transform
publishes **zero** with reason `pose_tf_unavailable:*` / `pose_stale:*`. It
**never falls back to `/odom`** — a silent downgrade to the drifting pose is
exactly the failure this mode exists to remove.

**A corrected pose is a discontinuous pose.** Every loop closure steps
`goal_local`, and the network is a CfC carrying recurrent state — untreated it
would keep steering on a hidden history built toward where the goal *used* to
be. `pose_source.py` detects a jump by **implied speed** (so the test scales
with dt rather than assuming a tick rate) and **wraps yaw** (a heading crossing
±π would otherwise read as a 6.28 rad spin). On a jump, `policy_node` adopts
the corrected pose and resets the recurrent state.

**Never map a ROS stamp onto the monotonic clock via a captured offset.** The
node keeps two clocks used consistently with themselves: ROS stamp vs ROS clock
for freshness, monotonic for intervals. bernard-1 has no NTP and its clock is
pushed by hand, so the two disagree by months.

---

## The goal-reached latch

The trained policy **has no stop-at-the-goal behaviour and could not have**:
BarnEnv terminated the episode at `goal_dist < SUCCESS_DIST` (0.4 m), so every
trajectory it learned from was *cut* at the moment of arrival. It was never
once observed sitting at a goal.

`policy_node` supplies that termination: it stops at `goal_reached_dist` and
**latches**. The latch is not optional — unlatched, drift or a SLAM correction
could carry the goal back outside the threshold and re-engage the policy, which
is exactly the drive-past-turn-back oscillation. Arrival is one-way, as in
training. Re-targeting clears it, and so does [roam mode](../../docs/roam.md),
which draws the next goal after a dwell at the goal or after a timeout.

---

## `PolicyRunner` — and why `strict=True` stays

Wraps `vendor.ncp_policy.NcpActor` via torch/ncps (not ONNX/TensorRT — no
export of this checkpoint exists yet; that is a flagged divergence from
BUILD-PLAN Phase 4, not an omission). It owns the CfC's recurrent state across
ticks, hashes the weights file for provenance, and validates every input and
output for shape/NaN/inf.

`load_state_dict` is called at its default **`strict=True`**. On 2026-08-29 a
stale vendor pin meant `NcpActor` had a different head shape than the
checkpoint — and because of strict mode that was a **startup crash, never a
wrong-numbers run**. Under `strict=False` the missing tensors would have been
left at random init and the robot would have run a half-trained network with no
error anywhere.

**The action is already in physical units.** The trained linear head + clip
produces final m/s and rad/s (`interface.yaml action.scaling.applies: false`).
Multiplying by any additional scaling here is a silent numerical bug with no
runtime error.

---

## The two bridge nodes

**`scan_bridge_node`** exists so `NCP-on-Rosmaster/rosmaster/node.py` can run
**completely unmodified**. That node hard-refuses any scan whose length isn't
32, so pointed at the raw TG30 it never builds an observation, its 1 s deadman
never clears, and it publishes zero forever — in *both* `debug` and `ncp` mode.
The bridge publishes 32 bin **centres** (span 174.375°, centred 0 rad) so
upstream's own `check_scan()` passes untouched.

> **QoS trap — do not "tidy" this.** The bridge publishes **RELIABLE** and
> subscribes **BEST_EFFORT**. Upstream's `create_subscription(..., 10)` is a
> depth-only profile and therefore RELIABLE, and a RELIABLE subscriber
> *refuses* a BEST_EFFORT publisher: rclpy logs the incompatibility once, then
> upstream's deadman reports "no fresh scan" forever *while `/ncp/scan` is
> visibly publishing at 10 Hz*. That looks exactly like a dead lidar.

**Only the upstream consumer reads `/ncp/scan`.** `policy_node` resamples
internally from the raw `/scan` — feeding it the bridge output would
double-resample and fault every tick.

**`scan_sanitizer_node`** republishes `/scan` at full resolution with every
invalid range rewritten to `+inf`, for slam_toolbox. Whether a dependency's
`min_laser_range` *discards* or *clamps* a short reading is its implementation
detail, and we were betting the pose estimate on it. `range_max` is
deliberately **not** clamped — a long reading is long, not invalid.

---

## Failure modes

Every module follows the same convention: a result object carrying either a
value or a `fault` string, **never a partial or plausible-looking answer**. The
caller cannot reach the value without branching on `fault` first.

`policy_node` publishes **zero** and does **not** update `previous_action` on
any of: stale scan, stale odom, undefined goal, unavailable/stale/jumped pose,
observation-builder fault, inference fault, NaN in the final command, or any
unhandled exception. A failed tick must not let a stale command persist, and
must not corrupt the fed-back action history either.

`scan_bridge_node` fails closed **by silence** — it publishes nothing on a
resampler fault rather than emitting a plausible all-`range_max` scan, because
"clear path everywhere" is the single most dangerous thing to fabricate. The
consumer's own deadman then stops the robot, which needs no cooperation from
this node.

---

## Provenance

The checkpoint's sha256 is logged at `policy_node` startup, published on
`/ncp_debug`, and printed by `scripts/deploy_upstream.sh` after verifying it on
both ends. All three must agree. Running with `weights:=''` logs a **warning**
that the network is randomly initialised.

**Probe this checkpoint with a real scan, never a synthetic one.** An
all-beams-at-max synthetic scan is far off-distribution and produces alarming
full-reverse/full-yaw output that says nothing about the policy. Fixtures are
in `tests/fixtures/`; note `tg30_open_scan.yaml` is **multi-document** — use
`yaml.safe_load_all`.
