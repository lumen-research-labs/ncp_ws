# Archive: retired code and docs

Nothing here is part of the codebase. No package builds it (`COLCON_IGNORE` keeps colcon out), no module imports it, no test exercises it and no current document links to it except this index; `tests/unit/test_archive_is_dead.py` pins all of that. It is kept readable so the reasoning behind a retired piece is one `cat` away. Git history holds the blame.

| item | archived | last live commit | why |
|---|---|---|---|
| `scripts/check_interface.py` | 2026-10-08 | `0cd2e67` | A stub whose own docstring says it validates nothing. The contract is pinned by `tests/unit/test_lidar_contract.py` instead. |
| `src/rm_tools/` | 2026-10-08 | `0cd2e67` | Placeholder ROS package (a 0-byte module) for offline analysis that was never written. It was only ever a workstation build target. |
| `docs/interface_contract_2.0.0_superseded.md` | 2026-10-08 | `0cd2e67` | A 41-dim contract that was never implemented. 3.0.0 (profile m4) and 4.0.0 (profile m8v2) are the real ones. |
| `docs/phase1_session_2026-08-08.md` | 2026-10-08 | `0cd2e67` | Phase 1 session log. `CLAUDE.md` §3 carries the record. |
| `config/topics.yaml` (three keys: `debug.policy_cmd_vel_debug`, `status.policy`, `foreign.cpt_namespace`) | 2026-10-13 | `7238808` | Named topics nothing publishes or reads (no dry-run debug topic exists, no `/policy/status`, and `/cpt/*` is the previous team's). The live file became the command-source table that `rm_safety`, the launch and the dashboard read. |
