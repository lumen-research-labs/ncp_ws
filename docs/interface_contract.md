# Interface contract, the shared layout

**Machine-readable source of truth: the profile's file under [`../config/`](../config/)**: `interface.yaml` (4.0.0), `interface_m9.yaml` (5.0.0), `interface_m4.yaml` (3.0.0). When they disagree, the YAML wins.

This page describes the layout 4.0.0 and 5.0.0 share. [`interface_versions.md`](interface_versions.md) lists every version with its checkpoint. [3.0.0](interface_contract_3.0.0_superseded.md) holds the rationale for the goal and previous-action blocks.

**Profiles.** `ncp.launch.py` runs one policy node per profile in `profiles:=` (default `m8v2,m4,m9`), each on its own contract file and checkpoint. `rm_safety` forwards the one named by `command_source`. `policy_node` builds the network to its contract's width and clips to its bounds. Another width, an unknown version, or a lidar block that differs from the vendored `BEAMS` and `FOV_DEG` is a startup crash.

## Observation, 724 dims, float32

| slice | block | dims | source |
|---|---|---|---|
| `obs[0:720]` | lidar ranges | 720 | `/scan` |
| `obs[720:722]` | goal `(x, y)` in the robot frame | 2 | `/odom` pose, or TF `map → base_footprint` |
| `obs[722:724]` | previous action | 2 | internal node state |

The arithmetic is upstream's own, `prepare_scan → local_goal → build_obs` in the vendored `vendor/ncp_policy.py`. `observation_builder.py` reduces the scan, validates, and re-normalizes the previous-action block by the profile's action bounds (see Action). The goal block (`xy/5 − 1`) and the requested-action feedback are 3.0.0's, blocks 2 and 3 there.

### Block 1, lidar, `obs[0:720]`

| property | value |
|---|---|
| arc | forward **270°**, BARN's native UST-10LX span |
| bins | **720**, 0.375° each, ordered right → left |
| sampling | `nearest_valid_ray` with a **±2-bin search window** |
| self-occlusion zones | `[92°, 123°]` and `[-135°, -97°]` |
| resample range | 0.01 m to 30.0 m. `0.0`, negatives, `inf`, `NaN`, masked rays and returns under the body floor become 30 m |
| body floor | `body_floor_m` 0.12 m: nearer than this is the chassis (standoffs 0.06 to 0.11 m, 2026-11-04) |
| normalization | `clip(r, 0, 5) → (r − 2.5) / 5 × 2` |
| empty-bin policy | `fault` |

[`lidar_sampling.md`](lidar_sampling.md) explains the window, the zones and the `0.0` mapping, each a measured decision. `rm_safety` computes its e-stop from the full-resolution scan, independently. Never wire this block into a safety path.

## Action, 2 dims

`[vx, wz]` in physical units from a linear head with clipping. `pre_scaled: true`, `scaling.applies: false`: the node multiplies by nothing. The bounds are the profile's `action.layout` ranges, read by `rm_policy/action_contract.py`: v in [-1, 2] m/s and w in [-3.14, 3.14] rad/s for 3.0.0 and 4.0.0 (the vendored constants), v in [-0.25, 0.25] and w in [-0.8, 0.8] for 5.0.0. `policy_runner.py` clips every action to them and `observation_builder.py` normalizes the previous-action block by them, as the training env did. The authority gap of 3.0.0 and 4.0.0 against the safety clamps (0.25 m/s, 0.8 rad/s) is intentional. The clamps scale `v` and `wz` by one factor (`safety_logic.clamp_velocity`), so the commanded path shape survives at the capped speed.

## Control rate, 5 Hz

Every run stepped at 0.2 s of sim time. The sensors run about 10 Hz, so 5 Hz keeps a 2:1 margin.

## Version assertion

`interface_version` is one of `3.0.0`, `4.0.0`, `5.0.0` (`policy_node._SUPPORTED_INTERFACE_VERSIONS`). Invariants pinned by tests:

- `tests/unit/test_vendor_is_pristine.py`: the vendored module matches upstream commit `bc1099d`.
- `tests/unit/test_lidar_contract.py`: each YAML, `policy_node`'s version pin and the vendored constants agree.
- `tests/unit/test_action_contract.py`: 3.0.0 and 4.0.0 declare the vendored action bounds exactly; 5.0.0 declares the robot's clamps and the 4.0.0 observation layout.
- `tests/unit/test_scan_resampler.py`: the window and mask measurements above hold on the recorded fixtures.
- `PolicyRunner.load_state_dict` stays `strict=True`: a checkpoint of another width is a startup crash.
