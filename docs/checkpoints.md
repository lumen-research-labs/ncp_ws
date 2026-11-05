# Checkpoints

The trained network behind each policy profile. The profile's contract file and version are in [`interface_versions.md`](interface_versions.md); adding a profile is [`add_a_policy_profile.md`](add_a_policy_profile.md).

| property | M4 | M8-v2 | M9 | DynaBARN-X3 |
|---|---|---|---|---|
| file | `NCP-on-Rosmaster/weights/actor_m4.pt` | `weights/actor.pt` | `weights/actor_m9.pt` | `weights/actor_dynabarn.pt` |
| sha256 | `3c9bf0ee112b…` | `71cc50aeb734…` | `bf4e1ba21b73…` | `defd46dbdc04…` |
| run, snapshot | M4, `final` | M8-v2 phase B, `s1436792` | M9 phase B (`m9_gz`), `s152000`, hot from `m9_cold s1627328` | gauntlet `dynabarn_x3_v3`, `s1248000` |
| trained on | BARN corridors, 32 beams over 180°, 5 Hz | BARN corridors, 720 beams over 270°, 5 Hz | procedural rooms and BARN corridors, X3 dynamics, goals at random bearings, 720 beams over 270°, 5 Hz | DynaBARN at X3 scale (moving obstacles), X3 dynamics, 720 beams over 270° with the mask, 5 Hz |
| action bounds | [-1, 2], [-3.14, 3.14] | same | [-0.25, 0.25], [-0.8, 0.8], the robot's clamps | as M9 |
| held-out success | 30 % | 63 % (100 BARN worlds, Gazebo) | 52 % (100 episodes, Gazebo); 53 % (300 episodes, fast 2D) | 35 % (240 DynaBARN-X3 episodes; M9 23 %) |
| official BARN metric | 0.1132 | 0.219 | not run | not run |
| follows the goal | yes, ahead only | no (2026-10-09: the same action for a goal ahead, beside or behind) | yes: 0.76 / 0.74 / 0.71 ahead / beside / behind in fast 2D (M8-v2: 0.35 / 0.34 / 0.24) | yes, any bearing |
| exported | 2026-08-27 | 2026-10-08 | 2026-11-04, `obs_dim=724 units=256 motor=128`, max action diff vs SB3 0.0 | 2026-11-05, `--bounds`, diff 1.4e-7 |

`NCP-on-Rosmaster/weights/export_from_sb3.py` produced all four and asserts equivalence with SB3 before saving. `policy_node` logs the sha256 it loaded and publishes 12 characters of it on `/ncp_debug/<profile>`; `scripts/deploy_upstream.sh` verifies the hash on both machines.
