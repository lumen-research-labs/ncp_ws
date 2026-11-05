# Plan: the robot's own body never reaches a scan consumer

> Status: the body-floor plan below is BUILT and DEPLOYED (3f8448c, 2026-11-04). Revised the same day on Daniel's decision "pinned is any situation where it is stopped for a bit": `pinned` is now a drivable status plus the robot stopped for `pinned_after_s` (5 s, `StoppedClock` in retreat.py; killed time never counts), whatever the cause; `retreat_pinned_range_m` is gone; `roam.yaml` carries `body_floor_m` so roam's turn checks drop the standoffs too. BUILT (789 workstation tests); deploy and the m9 floor test OPEN. Written to `~/lumen/optomotive/CODING_PROMPT.md` (2026-10-22).

## Context

M9's first drive (2026-11-04) was tight left circles into things, every roam goal "stalled after 10 s", and Daniel judged it much worse than m4. The live observation had ten lidar bins around −95° reading 0.10 m while nothing real was nearer than 0.59 m. A full-resolution scan (`tests/fixtures/tg30_scan_2026-11-04_standoffs.yaml`) shows the right chassis standoffs returning 0.06 to 0.11 m continuously from −138° to −94°. The self-occlusion mask was measured on 2026-08-14 and ends at −97°; the mount has moved since, and the band from −97° to −94° leaks through. Measured on the fixture with the shipped 4.0.0 lidar block: nine bins under 0.3 m (0.085 to 0.114 m at −97° to −95°); with every return under 0.12 m dropped, none (nearest bin 0.584 m). The safety stop arcs see the same leak: `min_range_in_arcs` 0.085 m on that scan, 0.575 m with the floor.

A return at 0.10 m on the right is "collision now" to a policy trained with collision ending the episode, so M9 holds full left yaw. m4's ±90° arc never sees −95°; m8v2 saw it and "circled left" on 2026-10-09. The same leak sits in rm_safety's wide-arc minimum, so the retreat's release ("clear to 0.30 m") cannot fire. The SLAM map's slow rotation is most likely the rear chassis corners (0.17 to 0.22 m, above slam_toolbox's 0.15 m floor) feeding points fixed in the robot frame to the scan matcher; not proven, fixed in the same pass.

## The general rule

Nothing in the world can be nearer the lidar than the chassis is, so a return under a measured body floor is the robot and is dropped before any consumer sees it. Two numbers, both measured: the standoffs end at 0.11 m (floor 0.12 m, the forward-arc consumers), the corners at 0.22 m (radius 0.25 m, the all-round consumers). The angle masks stay for anything the floor cannot cover.

## Design

| Consumer | Where | Change |
|---|---|---|
| Policy observation (all profiles) | `config/interface*.yaml` lidar block, `rm_policy/lidar_contract.py`, `policy_node`, `scan_bridge_node` | New key `body_floor_m: 0.12`. `LidarContract.body_floor_m` (absent = 0.0, off, like the window). The nodes pass `max(range_min_m, body_floor_m)` as the resampler's floor, so a body return takes the existing no-return path (`range_max`, clear). No resampler change. |
| Safety arcs | `config/safety_limits.yaml` `estop.body_floor_m: 0.12`, `safety_logic.Limits.estop`, `min_range_in_arcs` | Absent = 0.0 (off), finite, `>= 0`, `< stop_zone_m`. Rays under the floor are skipped like `<= 0`. |
| Third parties (slam_toolbox) | `config/safety_limits.yaml` `estop.body_radius_m: 0.25`, `scan_sanitizer_node`, `ncp.launch.py` | `sanitize_ranges(ranges, range_min, body_radius_m)`: returns under the radius become `+inf`. The node takes `body_radius_m` as a parameter; the launch reads it from the limits file. |
| Masks | the three contract files, `safety_limits.yaml` | Numbers unchanged; the measurement comments say what the standoffs read today and that the floor is the first line. |

## Failure modes

| Failure | Surfaces as |
|---|---|
| `body_floor_m` negative, non-finite or at or above the stop zone | `Limits.from_dict` / `lidar_contract_from` raise naming the key and value; no node starts. |
| A contract file without the key | Floor off for that profile; `tests/unit/test_body_floor.py` pins that every shipped profile sets 0.12 and that it equals the safety file's. |
| The mount moves again and a standoff reads above the floor | The fixture test fails only on a new fixture; the live check is `/ncp_debug/<p>` `obs` minimum, documented in `lidar_sampling.md`. |
| A real object under 0.12 m from the lidar | Physically inside the chassis; the stop zone owns anything that close anyway. |

## Tests

- `tests/unit/test_body_floor.py` (workstation): on the fixture, the 4.0.0 block without the floor yields bins under 0.12 m (the bug) and with it none under 0.3 m; `min_range_in_arcs` 0.085 → 0.575; the sanitizer turns returns under 0.25 m into `+inf` and leaves 0.26 m alone; refusals for bad values; the three contract files and the safety file agree on 0.12; `body_radius_m` equals `roam.yaml` `self_radius_m`; the launch passes it.
- `test_lidar_contract.py` PROFILES gain `body_floor_m`; `test_scan_sanitizer.py` the new argument; `test_safety_logic.py` `from_dict` cases.

## Docs

`docs/lidar_sampling.md` (a "Body floor" section with the 2026-11-04 measurement and the live check), `docs/interface_contract.md` Block 1 row, `src/rm_safety/README.md` and `src/rm_policy/README.md` one line each, `CLAUDE.md` dated entry, this file.

## Verification

Workstation suite green. Robot: rebuild `rm_policy rm_safety rm_bringup`, restart with the zones on; `/ncp_debug/m9` `obs` minimum above 0.3 m with nothing near; `/safety/status` `min_range` the real nearest object; the sanitizer start line names the radius. Floor: m9 armed with a goal ahead, then beside; the SLAM map holds its orientation over a few minutes of driving (Daniel's eye).
