# Interface versions and checkpoints

Every observation/action contract this repository has run, with the checkpoint each one serves. The machine-readable authority for a version is its file under `config/`. [`interface_contract.md`](interface_contract.md) explains the layout the current versions share.

## Versions

| version | file | observation | action bounds (v m/s, w rad/s) | checkpoint | profile |
|---|---|---|---|---|---|
| 2.0.0 | [archived](../archive/docs/interface_contract_2.0.0_superseded.md) | 41 dims | tanh, scaled by the node | none | never implemented |
| 3.0.0 | `config/interface_m4.yaml` | 36 dims: 32 lidar over 180° + 2 goal + 2 previous action | [-1, 2], [-3.14, 3.14] | M4 | `m4` ([rationale](interface_contract_3.0.0_superseded.md)) |
| 4.0.0 | `config/interface.yaml` | 724 dims: 720 lidar over 270° + 2 goal + 2 previous action | [-1, 2], [-3.14, 3.14] | M8-v2 | `m8v2` |
| 5.0.0 | `config/interface_m9.yaml` | 724 dims, as 4.0.0 | [-0.25, 0.25], [-0.8, 0.8] | M9 | `m9` |

A checkpoint dictates its version: its input layer is wired to one vector and its action head to one range. A profile is a version plus a checkpoint plus a `config/topics.yaml` row ([`add_a_policy_profile.md`](add_a_policy_profile.md)).

## Checkpoints

| property | M4 | M8-v2 | M9 |
|---|---|---|---|
| file | `NCP-on-Rosmaster/weights/actor_m4.pt` | `weights/actor.pt` | `weights/actor_m9.pt` |
| sha256 | `3c9bf0ee112b…` | `71cc50aeb734…` | `bf4e1ba21b73…` |
| run, snapshot | M4, `final` | M8-v2 phase B, `s1436792` | M9 phase B (`m9_gz`), `s152000`, hot from `m9_cold s1627328` |
| trained on | BARN corridors, 32 beams over 180°, 5 Hz | BARN corridors, 720 beams over 270°, 5 Hz | procedural rooms and BARN corridors, X3 dynamics, goals at random bearings, 720 beams over 270°, 5 Hz |
| action bounds | [-1, 2], [-3.14, 3.14] | same | [-0.25, 0.25], [-0.8, 0.8], the robot's clamps |
| held-out success | 30 % | 63 % (100 BARN worlds, Gazebo) | 52 % (100 episodes, Gazebo); 53 % (300 episodes, fast 2D) |
| official BARN metric | 0.1132 | 0.219 | not run |
| follows the goal | yes, ahead only | no (2026-10-09: the same action for a goal ahead, beside or behind) | yes: 0.76 / 0.74 / 0.71 ahead / beside / behind in fast 2D (M8-v2: 0.35 / 0.34 / 0.24) |
| exported | 2026-08-27 | 2026-10-08 | 2026-11-04, `obs_dim=724 units=256 motor=128`, max action diff vs SB3 0.0 |

`NCP-on-Rosmaster/weights/export_from_sb3.py` produced all three and asserts equivalence with SB3 before saving. `policy_node` logs the sha256 it loaded and publishes 12 characters of it on `/ncp_debug/<profile>`; `scripts/deploy_upstream.sh` verifies the hash on both machines.

## What 5.0.0 changes

The action bounds, and nothing in the layout. M9 trained inside the authority rm_safety grants (`max_vx` 0.25, `max_wz` 0.8), so two things differ from 4.0.0 per tick. The previous-action block is normalized by `(a - bias) / scale` over the M9 bounds, which with symmetric bounds is `v / 0.25` and `w / 0.8`. The emitted action is clipped to the M9 bounds before it is published and fed back, because the training env clips in `step()` and feeds the clipped action back. `rm_policy/action_contract.py` reads the bounds from the profile's file; `observation_builder.py` and `policy_runner.py` apply them. The vendored `vendor/ncp_policy.py` keeps the 4.0.0 constants. 3.0.0 and 4.0.0 declare those constants exactly (the literal 3.14), pinned by `tests/unit/test_action_contract.py`, so the two older profiles produce the same numbers as before.

M9 runs unsaturated, so its floor behaviour is evidence about the policy. A clamp below 0.25 / 0.8 (the signed-off 0.15 / 0.4 for anything unattended) puts it back in the saturated regime.
