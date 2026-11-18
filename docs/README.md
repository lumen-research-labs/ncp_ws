# Documentation index

| document | what it is | current? |
|---|---|---|
| [`architecture.md`](architecture.md) | **Start here.** How the system fits together, the data flow, and the reasoning behind every major design decision. | current |
| [`operating_guide.md`](operating_guide.md) | **The runbook.** Connect, deploy, start the stack, arm the safety layer, run the policy, use the dashboard, shut down. | current |
| [`roam.md`](roam.md) | **How to run roam mode.** The robot draws its own next goal at random on arrival or timeout and keeps driving; the keys of `config/roam.yaml`, how to switch it on and off, and how to read the counters. | current |
| [`interface_contract.md`](interface_contract.md) | The observation/action layout the profiles share, in prose. The profile's file under `config/` is the machine-readable authority. | current |
| [`interface_versions.md`](interface_versions.md) | Every contract version (3.0.0 to 5.1.0), its layout and bounds, and what 5.0.0 changed. | current |
| [`checkpoints.md`](checkpoints.md) | The network behind each profile (M4, M8-v2, M9, DynaBARN-X3): files, hashes, runs, held-out figures, exports. | current |
| [`add_a_policy_profile.md`](add_a_policy_profile.md) | **How to add a trained checkpoint as a profile** with its own dashboard button: export, contract file, table row, pins, deploy. | current (4.0.0) |
| [`lidar_sampling.md`](lidar_sampling.md) | Why the 720-beam lidar block is reduced the way it is: the search window, the self-occlusion zones and the `0.0` no-return mapping, with the measurements behind each. | current |
| [`add_a_robot.md`](add_a_robot.md) | **How to add a robot.** One profile file per unit (`config/robots/<name>.yaml`): hostname, ROS domain, lidar model and yaw, mount measurements, base calibration; how to measure each and verify the deploy. | current |
| [`calibration.md`](calibration.md) | **How to bring `rm_base` up on a NEW robot.** The per-robot measurement procedure, in order, wheels-up before wheels-down. The numbers in `config/base_calibration.yaml` are bernard-1's and do not transfer. | current |
| [`safety_validation.md`](safety_validation.md) | The Phase 2 sign-off record: which test tiers have passed, what is still operator-only, and which limits are running above their signed values. | current |
| [`workstation_setup.md`](workstation_setup.md) | **Set up a fresh PC.** Packages, the two clones side by side, tests, the workstation build, SSH and the wired link to the robot, deploy. | current |
| [`deployment.md`](deployment.md) | Historical cold-start runbook and the measured rate/TF tables. | **partly superseded** — see its banner |
| [`interface_contract_3.0.0_superseded.md`](interface_contract_3.0.0_superseded.md) | The 3.0.0 contract (M4, 32 beams) with the rationale for the goal and previous-action blocks every later version keeps. | superseded, still deployable as profile `m4` |
| [`../archive/docs/phase1_session_2026-08-08.md`](../archive/docs/phase1_session_2026-08-08.md) | Phase 1 session log. | archived |

Per-package documentation lives next to the code, in `src/rm_*/README.md`.

`../CLAUDE.md` is the project brain — the append-only progress log and the live
list of settled decisions and open items. It is the most current document in
the repository and supersedes anything here on a conflict.
