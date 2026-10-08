# Interface contract — version 4.0.0

**Machine-readable source of truth: [`../config/interface.yaml`](../config/interface.yaml).** If the two disagree, the YAML wins and this file is the bug.

Earlier versions: [3.0.0](interface_contract_3.0.0_superseded.md), the M4 checkpoint with 32 beams over 180°, and [2.0.0](interface_contract_2.0.0_superseded.md), never implemented. The 3.0.0 document holds the rationale for everything 4.0.0 keeps unchanged and is linked below instead of repeated.

## What changed

| version | observation | action | driver |
|---|---|---|---|
| 3.0.0 | 36 dims (32 lidar over 180° + 2 goal + 2 prev action) | 2 dims `[vx, wz]` | the M4 checkpoint |
| **4.0.0** | **724 dims (720 lidar over 270° + 2 goal + 2 prev action)** | 2 dims `[vx, wz]` | **the M8-v2 checkpoint, trained on BARN's native sensor** |

4.0.0 is dictated by the checkpoint, as 3.0.0 was. `policy_node` reads `interface_version` at startup and refuses to construct on a mismatch. It also refuses if the YAML's bin count or arc differs from the vendored module's `BEAMS` and `FOV_DEG`, and so does `scan_bridge_node`.

## The checkpoint

| property | value |
|---|---|
| file | `NCP-on-Rosmaster/weights/actor.pt` |
| sha256 | `71cc50aeb734…` |
| run, snapshot | M8-v2 phase B, `s1436792` |
| trained on | `--beams 720 --fov 270`, 5 Hz, v in [-1, 2] m/s, w in [-3.14, 3.14] rad/s |
| held-out success | 63% (100 BARN worlds, Gazebo) |
| official BARN metric | 0.219 |

`policy_node` logs the sha256 it loaded and publishes the first 12 characters on `/ncp_debug`. `scripts/deploy_upstream.sh` verifies the same hash on both ends. All three must agree.

## Observation, 724 dims, float32

| slice | block | dims | source |
|---|---|---|---|
| `obs[0:720]` | lidar ranges | 720 | `/scan` |
| `obs[720:722]` | goal `(x, y)` in the robot frame | 2 | `/odom` pose, or TF `map → base_footprint` |
| `obs[722:724]` | previous action | 2 | internal node state |

Assembly order and all arithmetic are upstream's own, `prepare_scan → local_goal → build_obs` in the vendored `vendor/ncp_policy.py`. `observation_builder.py` reduces the scan and validates, nothing more. The goal and previous-action blocks are unchanged from 3.0.0, including the `xy/5 − 1` normalization that leaves the policy out of distribution behind or beside the goal, the requested-action feedback, and the literal `3.14` yaw bound. See the 3.0.0 document, blocks 2 and 3.

### Block 1, lidar, `obs[0:720]`

| property | value |
|---|---|
| arc | forward **270°**, BARN's native UST-10LX span |
| bins | **720**, 0.375° each, ordered right → left |
| sampling | `nearest_valid_ray` with a **±2-bin search window** |
| self-occlusion zones | `[92°, 123°]` and `[-135°, -97°]` |
| resample range | 0.01 m to 30.0 m. `0.0`, negatives, `inf`, `NaN` and masked rays become 30 m |
| normalization | `clip(r, 0, 5) → (r − 2.5) / 5 × 2` |
| empty-bin policy | `fault` |

Three facts about the real sensor shape this block.

**The TG30 drops 28 to 39% of rays, and a 0.375° bin holds about 2 of them.** Strict per-bin sampling leaves 5.4 to 8.3% of bins with no real return on the recorded fixtures. Each reads "clear at 5 m" while a real return sits a fraction of a degree away, and simulation never produced that. A bin with no real return therefore borrows the real return nearest its centre within ±2 bins (reach 0.9375°), which leaves 1.2 to 1.4% of bins empty on the same fixtures. Rays inside the bin always win. The window is a fallback and never a smoothing.

**The 270° arc sees the chassis.** The 180° arc stopped short of the standoffs. The wider arc includes them at 0.11 to 0.25 m. The training sensor's 0.1 m `min_range` put the Jackal chassis in a dead zone, so those directions read "clear" in simulation. The two zones above, the same sectors `safety_limits.yaml` masks for the e-stop, are blanked before sampling. Their rays count as members, contribute no range, and never feed the window.

**No-return is `0.0`, not `inf`,** and the driver flag that should change that is inert. The mapping in `scan_resampler` is the only defence in the policy path. The measurements are in the 3.0.0 document.

`rm_safety` computes its e-stop from the full-resolution scan, independently. Never wire this block into a safety path.

## Action, 2 dims, unchanged

`[vx, wz]` in physical units from a linear head with clipping, v in [-1, 2] m/s and w in [-3.14, 3.14] rad/s. `pre_scaled: true`, `scaling.applies: false`. The authority gap against the safety clamps (0.25 m/s, 0.8 rad/s) is intentional, and the clamps are not raised to close it. The rationale and the `3.14` note are in the 3.0.0 document under "Action".

## Control rate, 5 Hz, unchanged

The M7 and M8 runs stepped at 0.2 s of sim time, the same cadence as M4. All sensors run about 10 Hz, so 5 Hz keeps a 2:1 margin.

## Version assertion

`interface_version: "4.0.0"`. Invariants pinned by tests:

- `tests/unit/test_vendor_is_pristine.py`: the vendored module matches upstream commit `bc1099d`.
- `tests/unit/test_lidar_contract.py`: the YAML, `policy_node`'s version pin and the vendored constants agree.
- `tests/unit/test_scan_resampler.py`: the window and mask measurements above hold on the recorded fixtures.
- `PolicyRunner.load_state_dict` stays `strict=True`, so a 3.0.0 checkpoint is a startup crash, never a wrong-numbers run.
