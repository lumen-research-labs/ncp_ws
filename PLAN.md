# Plan: a trained checkpoint becomes a dashboard button (first case: M9)

> Status: APPROVED 2026-11-04 (Daniel); BUILT the same day on `feat/roam-mode` (29c3cc5, a6350d5 + docs) and upstream `feat/720-beam-270fov` (0e1bbe1, `weights/actor_m9.pt`); 742 workstation tests. OPEN: the deploy, rebuild and restart on bernard-1 (the robot left the network mid-deploy) and the floor test. Written to `~/lumen/optomotive/CODING_PROMPT.md` (2026-10-22). Revised in place as implementation finds things.

## Context

Daniel: "make it so that I can have a m9 button on the dashboard." M9 is the rooms-trained NCP that follows the goal (sim success ahead/beside/behind 0.76/0.74/0.71; M8-v2 0.35/0.34/0.24). Its two training stages finished on 2026-10-13 (phase A, fast 2D) and 2026-10-17 (phase B, Gazebo with the X3). It was never exported.

The general capability this plan builds: **any exported checkpoint becomes a profile by adding one contract file, one weights file and one `config/topics.yaml` row.** The dashboard builds one button per row, so the button is free. Two existing profiles (m8v2, m4) already work this way, with one gap. Both were trained with the same action bounds ([-1, 2] m/s, [-3.14, 3.14] rad/s), and those bounds are hardcoded in the vendored `rm_policy/vendor/ncp_policy.py`. M9 was trained at the robot's clamps (±0.25 m/s, ±0.8 rad/s), and the bounds shape two things the network sees and does. This plan moves the bounds into the contract, where the beam count already lives. M9 is the test case.

Facts, read from the checkpoints on the box and the code:
- Both M9 snapshots are `obs_dim 724, units 256, motor 128`, the shape of M8-v2, with the 4.0.0 observation layout (720 beams over 270 deg, `LASER_CLIP` 5.0, goal `xy/5 - 1`). `barn_gym/env.py` on optomotive `feat/m9-gazebo` confirms the layout.
- M9's action space is `[-0.25, 0.25] x [-0.8, 0.8]`. The previous-action block is normalised by `(a - bias) / scale` with bias 0 and scale (0.25, 0.8). The training env clips the action to the space in `step()` and feeds the **clipped** action back as the observation (`env.py` lines 153-157 and 325). The node must do the same.
- `observation_builder.build_observation` calls the vendored `ncp_policy.build_obs`, and `PolicyRunner.act` returns `NcpActor.act`'s output clipped to the vendored bounds. `policy_node` publishes that and stores it as `_prev_action_v/w` (policy_node.py:1396). The bounds enter at exactly those two points. The vendored file stays pristine (`tests/unit/test_vendor_is_pristine.py`).
- Snapshot: `runs/m9_gz/archive/s152000` (phase B, the rendered lidar at the real mount and the fitted X3 dynamics; 52/100 Gazebo held-out, the 2D best scores 46/100 there). M8-v2 was taken from its phase B the same way. The 2D best `s1627328` stays on the box.
- The export runs on the box (torch and sb3 live only there). `weights/export_from_sb3.py` infers obs_dim/units/motor from the checkpoint and asserts step-for-step equivalence against SB3 before saving. It is copied up for the run. The DynaBARN controller (not ours, 32 envs) is running there; the export is a few hundred forward passes under `nice`.
- Jetson headroom: 8 cores, 68 % idle with two policy nodes. A third fits. `profiles:=` drops one if it does not.

## Existing libraries and code reused

No external library covers this. Reused: `weights/export_from_sb3.py` (upstream, the M8-v2 and M4 exports), the contract-reader pattern of `rm_policy/lidar_contract.py` (`lidar_contract_from(interface_cfg)`), the `command_sources` table (`rm_safety/command_sources.py`), the `_PROFILES` table in `ncp.launch.py`, `scripts/deploy_upstream.sh` checksum list, and the dashboard's `renderSources`. Nothing new is vendored.

## Design

| Topic | Decision |
|---|---|
| Export | On the box: `scp` the exporter up, `nice -n 15 .venv/bin/python export_from_sb3.py runs/m9_gz/archive/s152000 /tmp/actor_m9.pt`, expect `obs_dim=724 units=256 motor=128` and `max action diff vs SB3` under 1e-5, `scp` back to `~/lumen/ncp/weights/actor_m9.pt`, record the sha256. Commit on upstream `feat/720-beam-270fov` (the pattern of 97f293b), push to lumen-research-labs/NCP-on-Rosmaster. |
| Contract | `config/interface_m9.yaml`, `interface_version: "5.0.0"`, made as `interface_m4.yaml` was (a full copy of `interface.yaml`). Differences: `checkpoint:` (file, sha `[after export]`, run `m9_gz`, snapshot `s152000`, trained_on "rooms + BARN corridors, X3 dynamics, 720 beams over 270 deg, v in [-0.25, 0.25], w in [-0.8, 0.8], 5 Hz", held-out 0.52 Gazebo and 0.53 fast2d, goal-following 0.76/0.74/0.71), `previous_action.normalization` ("v / 0.25, w / 0.8"), `action.layout` ranges `[-0.25, 0.25]` and `[-0.8, 0.8]`, and the `authority_gap_warning` rewritten: the trained range equals the clamps, so M9 is the first policy whose behaviour on the floor is evidence about the policy. |
| Bounds reader | New pure module `src/rm_policy/rm_policy/action_contract.py`: `ActionBounds(low, high)` and `action_bounds_from(interface_cfg, path)` from `action.layout[cmd_vx/cmd_wz].range`. For 3.0.0 and 4.0.0 files the result equals the vendored `ACTION_LOW/HIGH` exactly (3.14, not pi), test-pinned, which keeps the existing cross-check automated. |
| Observation | `build_observation(..., action_low, action_high)` with the vendored constants as defaults. After `ncp_policy.build_obs`, the last two entries are recomputed as `(prev - (hi+lo)/2) / ((hi-lo)/2)` in float32, the formula `build_obs` uses, so the default path is unchanged bit for bit. |
| Runner | `PolicyRunner(..., action_low, action_high)`, defaults the vendored constants. After `NcpActor.act` (the looser vendored clip) the action is clipped to the contract bounds. The node publishes and stores that value. A no-op for m8v2 and m4. |
| policy_node | `_SUPPORTED_INTERFACE_VERSIONS` gains `"5.0.0"`. The bounds are read at startup, passed to the runner and to `build_observation`, named in the startup line, and published in `/ncp_debug/<p>` as `action_bounds`. |
| Launch and table | `ncp.launch.py`: `_PROFILES["m9"] = ("interface_m9.yaml", "actor_m9.pt")`, `_DEFAULT_PROFILES = "m8v2,m4,m9"` (m8v2 stays first for the upstream consumer and the bridge). `config/topics.yaml`: row `m9` with `/policy/m9/cmd_vel`, `policy_node_m9`, `/ncp_debug/m9`, description "M9, 720 beams over 270 deg, interface 5.0.0 (steers to the goal; trained at the robot's clamps)". The button, the source switch and the roam pill come from the row. No dashboard HTML change. |
| Deploy | `scripts/deploy_upstream.sh` `CRITICAL` += `weights/actor_m9.pt`. On the robot: `deploy_upstream.sh`, `deploy_to_robot.sh`, `colcon build --symlink-install --packages-select rm_policy rm_bringup`, `stack_down.sh`, `prep_robot.sh`, `run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true`. |
| Roam on m9 | Unchanged. The shared `roam.yaml` cone (±45 deg) applies. M9 trained on random bearings, so a wider cone per profile is a follow-up once it has driven. |

## Failure modes and how each surfaces

| Failure | Where it is caught | What the operator sees |
|---|---|---|
| Export does not match SB3 (wrong snapshot layout, a changed exporter) | `export_from_sb3.py` asserts `worst < 1e-5` | The export fails on the box with the max diff printed. Nothing is copied. |
| Weights and contract disagree (a 724-wide file with a 36-wide contract, or a checkpoint of another shape) | `PolicyRunner` loads with `strict=True` | `policy_node_m9` refuses to start, naming the weights path and the contract file. The stack stays up without it. |
| Checkpoint missing on the robot | `deploy_upstream.sh` `CRITICAL` check and sha256 compare | The deploy exits nonzero and names the file; the launch refuses to start the node for a missing default checkpoint. |
| `action:` block missing, a range inverted, non-finite, or `pre_scaled` not true | `action_bounds_from` | `RuntimeError` at startup: "policy_node: `<file>`: action.layout cmd_vx range [..] ... (what was read, what is required)". No node, no silent default. |
| Contract bounds wider than the vendored clip (a future checkpoint trained past ±2 m/s) | `action_bounds_from` | Refused at startup with both ranges in the message. The vendored clip would otherwise bite first and the fed-back action would differ from training. |
| Contract bounds wider than `safety_limits.yaml` clamps | Allowed (m8v2 and m4 are that case). The contract's authority note records it. | The policy runs saturated, as today. |
| `interface_version` not in the supported list | `policy_node` (existing) | Refused at startup, naming the version and the list. |
| The m9 row and the `_PROFILES` entry drift apart | `test_source_per_policy_launch.py` | A failing test on the workstation before deploy. |
| Third node starves the Jetson | None automatic; `/api/state` tick rates and the policy debug `fault` field | Lower policy rates on the dashboard; `profiles:=m8v2,m9` drops m4. |

## Changes by file

**Upstream `~/lumen/ncp` (feat/720-beam-270fov)**: `weights/actor_m9.pt` (new, about 6 MB).

**`config/interface_m9.yaml`** (new). **`config/topics.yaml`**: the `m9` row.

**`src/rm_policy/rm_policy/action_contract.py`** (new, pure). **`observation_builder.py`**: the two keyword arguments and the recompute. **`policy_runner.py`**: the two constructor arguments and the clip after `act`; the docstring sentence "already clipped to the trained action bounds" becomes "clipped to the contract's bounds". **`policy_node.py`**: supported versions, bounds read and passed, startup log, debug field, docstring row.

**`src/rm_bringup/launch/ncp.launch.py`**: `_PROFILES`, `_DEFAULT_PROFILES`, docstring. **`scripts/deploy_upstream.sh`**: `CRITICAL`.

**Docs** (each under 600 words by `wc -w`, Google style, the writing rules):
- `docs/add_a_policy_profile.md` (new how-to): export on the box, the contract file and which blocks a new checkpoint changes, the table row, the `_PROFILES` entry, the deploy list, the tests that pin each, the robot checks, the floor test. The reader is a fresh agent with a checkpoint directory and nothing else. Indexed in `docs/README.md` and the repo README layout.
- `docs/interface_contract.md` (735 words before, over the cap): split. The checkpoint tables and the version history move to `docs/interface_versions.md` (reference; rows for 3.0.0, 4.0.0, 5.0.0 with each checkpoint's file, sha, bounds, held-out figures); the contract doc keeps the layout and the normalisation rules and gains the sentence that action bounds and the previous-action scale come from each profile's file. Both indexed.
- `src/rm_bringup/README.md` (profile row, diagram line), `docs/operating_guide.md` (button list, source table, profile table, switch sentence), `src/rm_policy/README.md` (row for `action_contract.py`), `CLAUDE.md` (dated entry: why the bounds move into the contract, the clipped-feedback fact, the snapshot choice), `PLAN.md` (this file, status line), memory.

## Tests

- `tests/unit/test_action_contract.py` (new): `interface_m9.yaml` gives (-0.25, -0.8)/(0.25, 0.8); `interface.yaml` and `interface_m4.yaml` give the vendored constants exactly; each refusal (missing block, inverted, non-finite, `pre_scaled: false`, wider than the vendored clip) raises with a message naming the file and the key; M9's bounds sit within `safety_limits.yaml` `max_vx`/`max_wz`.
- `tests/unit/test_observation_builder.py`: with M9 bounds prev (0, 0) gives (0, 0), (0.25, -0.8) gives (1, -1), (0.1, 0.4) gives (0.4, 0.5); the default path equals `ncp_policy.build_obs` bitwise on a fixture scan.
- Runner tests: random weights with M9 bounds never exceed them; with the defaults the action equals the actor's own output.
- `tests/unit/test_lidar_contract.py` `PROFILES` += m9 (5.0.0, the m8v2 lidar block); `test_source_per_policy_launch.py` set `{"m8v2","m4","m9"}` and the default string; `test_command_sources.py` table order; a pin that `interface_m9.yaml`'s observation block equals `interface.yaml`'s apart from the previous-action text; pins for the supported-versions tuple and the deploy `CRITICAL` list.
- Doc tests: every `docs/*.md` is linked from `docs/README.md`; each doc under 600 words (today `test_roam.py` pins only `roam.md`; the cap test becomes one parametrised test over `docs/`, with the files already over the cap listed as known exceptions until they are split); the how-to names every registration point the profile tests check.
- No agent-eval harness exists in this repo. The how-to is proven by this task: the m9 profile is added by following it, and the doc test above keeps the how-to and the code pins in step.

## Commits

Upstream `feat/720-beam-270fov`: `feat: ship the M9 checkpoint (weights/actor_m9.pt, m9_gz s152000)`.
ncp_ws `feat/roam-mode` (continuing the chain):
1. `feat(rm_policy): action bounds come from the contract; interface 5.0.0 for M9` (action_contract, observation_builder, policy_runner, policy_node, interface_m9.yaml, tests).
2. `feat(rm_bringup): the m9 profile, third policy node and dashboard button` (launch, topics.yaml, deploy_upstream, tests).
3. `docs: how to add a policy profile; the contract doc split at the cap` (how-to, interface_versions, indexes, READMEs, operating guide, CLAUDE.md, PLAN.md).
Suite green after each. Push both repos to the org (pushing is allowed; merging stays Daniel's; PR #2 picks the commits up).

## Verification

- Box: the export prints `obs_dim=724 units=256 motor=128` and `max action diff vs SB3` under 1e-5. The sha256 on the workstation, in `interface_m9.yaml` and on the robot agree (`deploy_upstream.sh` prints OK for `weights/actor_m9.pt`).
- Workstation: `python3 -m pytest tests/` green; `wc -w` under 600 for every doc touched.
- Robot, killed: three policy nodes; the m9 start line logs interface 5.0.0, the sha and bounds ±0.25/±0.8; `/api/state` lists the `m9` source and `/ncp_debug/m9` with `action_bounds`; the dashboard shows an `m9` button beside `m4` and `m8v2`; pressing it force-kills like the others; `/ncp_debug/m9` `action` stays inside ±0.25/±0.8 while killed.
- Floor (Daniel, KILL in hand, zones on): select m9, ARM, a goal 1.5 to 3 m ahead; expect a drive to the goal and the arrival latch. Then a goal 90 deg to the side: M8-v2 ignores it, M9 should turn to it. Then ROAM on m9. The comparison with m4 on the same legs goes into CLAUDE.md.

## Merge and revert

Merge: Daniel merges PR #2 (feat/roam-mode into main) when he chooses; the upstream branch merges the same way. Revert: `git revert` the three ncp_ws commits in reverse order, and `git rm weights/actor_m9.pt` upstream. The vendored file is untouched, so m8v2 and m4 behave the same before and after.

## Not in this plan

- Per-profile roam limits (a wider cone for m9).
- Exporting the 2D best `s1627328` as a second profile (one contract copy and one row, the same path).
- Any dashboard HTML change.
