# Documentation index

| document | what it is | current? |
|---|---|---|
| [`architecture.md`](architecture.md) | **Start here.** How the system fits together, the data flow, and the reasoning behind every major design decision. | current |
| [`operating_guide.md`](operating_guide.md) | **The runbook.** Connect, deploy, start the stack, arm the safety layer, run the policy, use the dashboard, shut down. | current |
| [`interface_contract.md`](interface_contract.md) | The frozen observation/action contract in prose. `config/interface.yaml` is the machine-readable authority. | current (4.0.0) |
| [`calibration.md`](calibration.md) | **How to bring `rm_base` up on a NEW robot.** The per-robot measurement procedure, in order, wheels-up before wheels-down. The numbers in `config/base_calibration.yaml` are bernard-1's and do not transfer. | current |
| [`safety_validation.md`](safety_validation.md) | The Phase 2 sign-off record: which test tiers have passed, what is still operator-only, and which limits are running above their signed values. | current |
| [`dev_env_setup.md`](dev_env_setup.md) | Workstation ROS2 setup (containerised). | mostly current |
| [`deployment.md`](deployment.md) | Historical cold-start runbook and the measured rate/TF tables. | **partly superseded** — see its banner |
| [`phase1_session_2026-08-08.md`](phase1_session_2026-08-08.md) | Session log, kept as a record. | historical |

Per-package documentation lives next to the code, in `src/rm_*/README.md`.

`../CLAUDE.md` is the project brain — the append-only progress log and the live
list of settled decisions and open items. It is the most current document in
the repository and supersedes anything here on a conflict.
