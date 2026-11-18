# Plan: one stack, several robots: a robot profile per unit

> Status: BUILT 2026-11-18 on `feat/dynabarn-profile` (commits 6343b60, 6df144d, 91f8b8a, db1fdd8 and the docs commit); suite 850 green; DEPLOY to bernard-3 next, bernard-1 when it is back. The previous plans this file held (M9 profile, body floor, pinned-is-stopped) are built and deployed; git history keeps them. Written to `~/lumen/optomotive/CODING_PROMPT.md` (2026-10-22). Revised in place as implementation finds things.

## Context

Daniel runs two robots from one codebase and will run them at the same time: bernard-1 (the unit everything was measured on) and bernard-3, a stock Yahboom X3 brought up on 2026-11-17. Two days of bringing bernard-3 up turned every "constant" that was really a measurement of bernard-1 into a per-robot fact, each patched in a different place:

| Fact | Where it lives today | bernard-1 | bernard-3 |
|---|---|---|---|
| ROS domain | `scripts/run_stack.sh` (`export ROS_DOMAIN_ID=28`) | 28 | 28, so both units on one network share `/cmd_vel` and every dashboard shows both |
| lidar driver | `ncp.launch.py lidar:=` | TG30 | RPLIDAR A1 |
| lidar zero direction | `ncp.launch.py lidar_yaw_deg:=` | 0 | 180 (mounted zero-backward) |
| self-occlusion zones | `config/interface*.yaml` and `config/safety_limits.yaml`, same numbers twice | measured 2026-08-14 | wrong for this mount, nothing measured |
| body floor, body radius | the same two files, plus `config/roam.yaml` | 0.12 / 0.25 | the same numbers happen to fit |
| base calibration | `config/base_calibration.yaml` via `calibration_path:=` | tape-fitted, with a sticky-wheel workaround | bernard-1's file, wrong |
| encoder sign | hardcoded negation in `rm_base/wheel_control.py` | inverted (firmware bug) | normal: the polarity guard latched all four wheels to zero on the first jog, 2026-11-18 |

Daniel asked whether two dashboards or a dashboard option is the answer. Neither: the dashboard is served from each robot and reads only that robot's topics, so there are already two. The differences are launch-time facts about the unit, and they need one home.

The general capability: **a robot is a profile file. The stack reads it by the robot's hostname and configures itself; nothing about a unit is hardcoded anywhere else.** Adding a fourth robot is one file and one measurement session.

## Existing code reused

`rm_safety/command_sources.py` is the pattern for a validated config table loaded by several packages (the launch and the dashboard already import it). `lidar_contract_from` (`rm_policy/lidar_contract.py`) and `Limits.from_dict` (`rm_safety/safety_logic.py`) are the two readers that gain a mount argument. `base.launch.py calibration_path` and `EncoderTracker` (`rm_base/wheel_control.py`, the negation is documented there) already carry the base side. `ncp.launch.py`'s `OpaqueFunction` refusals (`_refuse_if_stack_already_running`, `_refuse_unknown_lidar`) are the pattern for fail-fast checks. `policy_node` already reads `safety_limits.yaml` directly for its timeouts, which is the precedent for reading a YAML without importing `rm_safety`.

## Design

**The profile.** `config/robots/<name>.yaml`, one per unit:

```yaml
robot:
  name: bernard-3
  hostnames: [bernard-3, yahboom]      # what `hostname` returns on that unit
  ros_domain_id: 29
lidar:
  model: a1                            # tg30 | a1, the driver block in ncp.launch.py
  yaw_deg: 180                         # where its zero points, degrees left of +x
  self_occlusion_zones_deg: []         # mount measurements, were in interface*.yaml and safety_limits.yaml
  body_floor_m: 0.12
  body_radius_m: 0.25
base:
  calibration: config/base_calibration_bernard3.yaml
```

`config/robots/bernard-1.yaml` carries today's values exactly (domain 28, tg30, yaw 0, the two zones, 0.12, 0.25, `config/base_calibration.yaml`), so bernard-1's behaviour is unchanged bit for bit. The encoder sign stays in the base calibration file (`encoder_sign: -1` in bernard-1's, `+1` in bernard-3's, a required key), because it is a base-board fact the calibration already owns.

**The loader.** `rm_safety/robot_profile.py` (pure): `load_robot_profile(path) -> RobotProfile` (frozen dataclass), `resolve_profile_path(robots_dir, hostname) -> path` by the `hostnames` lists, refusing no match or two matches by name, and `check_profiles(robots_dir)`: every file loads, names unique, hostnames unique, domains unique, calibration files exist, model in `("tg30", "a1")`, yaw finite, zones well-formed, floor below radius. Every refusal names the file and the key.

**Where the facts go, by consumer:**

| Consumer | Change |
|---|---|
| `scripts/robot_env.sh` (new, sourced by `run_stack.sh` and `stack_down.sh`) | Resolves the profile from `hostname` (or `$ROBOT`), exports `ROS_DOMAIN_ID` and `RM_ROBOT_PROFILE`, prints `robot: bernard-3 (domain 29)`. Refuses an unknown hostname by listing the profiles. `stack_down.sh` needs the domain to reach `/safety/kill`. |
| `ncp.launch.py` | New `robot_profile:=` argument, default resolved from the hostname. Reads the profile once: the lidar block picks the driver (replaces `lidar:=`) and the yaw (replaces `lidar_yaw_deg:=`, both removed), `calibration_path` defaults to the profile's file, every node below receives `robot_profile_path`. `SetEnvironmentVariable("ROS_DOMAIN_ID", profile.domain)` for every child, and a refusal when the launching shell's `ROS_DOMAIN_ID` disagrees with the profile (the stack and its stop button must share a domain). |
| `lidar_contract_from(interface_cfg, mount)` | Zones and body floor come from the profile's `lidar` block; the contract files lose `self_occlusion_zones_deg` and `body_floor_m` (a test pins their absence). `policy_node` and `scan_bridge_node` read the profile by path and pass the block. |
| `Limits.from_dict(cfg, mount)` | The same two facts from the profile; `safety_limits.yaml` loses them; `safety_node` reads the profile by path. |
| roam | `roam.yaml` loses `body_floor_m`; `policy_node` sets `RoamLimits.body_floor_m` from the profile (`dataclasses.replace`). |
| `scan_sanitizer_node` | `body_radius_m` from the profile; `slam.launch.py` reads the profile instead of `safety_limits.yaml`. |
| `rm_base` | `EncoderTracker` gains `sign` (from the calibration file's `encoder_sign`, required, `+1` or `-1`); `base_node` refuses a file without it. |
| `rm_dashboard` | `robot_profile_path` parameter; the snapshot carries `robot: {name, domain, lidar}`; the header and the page title show the robot's name, so two open tabs cannot be confused. |
| hostname | bernard-3 is still `yahboom`. The profile lists both names, so nothing depends on renaming it; renaming (`hostnamectl set-hostname bernard-3`, `/etc/hosts`) is a one-line operator step recorded in the how-to. bernard-1's hostname is confirmed on its next boot and added to its list. |

## Failure modes

| Failure | Surfaces as |
|---|---|
| No profile lists this hostname | `robot_env.sh` and the launch refuse, printing the hostname and the profiles' names; nothing starts. |
| Two profiles claim one hostname or one domain | `check_profiles` raises naming both files; pinned by a test over `config/robots/`. |
| A profile missing a key, an unknown lidar model, a non-finite yaw, a floor at or above the radius | The loader raises naming the file and the key; the node or launch that asked refuses to start. |
| The calibration file lacks `encoder_sign` | `rm_base` refuses at construction, naming the file. |
| The shell's domain differs from the profile's | The launch refuses before starting anything, naming both. |
| Both robots on one network | Distinct domains: each stack, dashboard and stop button sees only its own robot. Workstation-side `ros2` tools need the matching domain; documented. |
| A contract or limits file still carrying zones | A test fails: the fact has one home. |

## Changes by file

**New:** `config/robots/bernard-1.yaml`, `config/robots/bernard-3.yaml`, `config/base_calibration_bernard3.yaml` (encoder_sign +1, bernard-1's generic wheel model on all four wheels as the starting point, no front-right stiction overrides), `rm_safety/robot_profile.py`, `scripts/robot_env.sh`, `docs/add_a_robot.md`, `tests/unit/test_robot_profile.py`.

**Changed:** `scripts/run_stack.sh`, `scripts/stack_down.sh`, `ncp.launch.py` (profile loading, driver and yaw from it, `robot_profile_path` to every node, domain env, `lidar`/`lidar_yaw_deg` removed), `slam.launch.py`, `rm_policy/lidar_contract.py`, `policy_node.py`, `scan_bridge_node.py`, `roam.py` (KEYS without `body_floor_m`), `rm_safety/safety_logic.py`, `safety_node.py`, `rm_base/wheel_control.py`, `base_node.py`, `rm_dashboard/dashboard_node.py`, `index.html` (header, title), `config/interface*.yaml`, `config/safety_limits.yaml`, `config/roam.yaml`, `config/base_calibration.yaml` (`encoder_sign: -1` with the 2026-08-28 measurement).

**Docs** (each under 600 words): `docs/add_a_robot.md` (how-to: hostname, profile, calibration, the measurements to take, the first jog); `docs/operating_guide.md` (the robots table becomes "one profile per robot", the launch lines lose their arguments); `docs/workstation_setup.md` (domain per robot for workstation ROS tools); `docs/interface_contract.md` and `docs/lidar_sampling.md` (zones and floor now live in the robot profile); `src/rm_bringup/README.md`, `src/rm_safety/README.md`, `src/rm_base/README.md`, `src/rm_dashboard/README.md` (one line each); `.claude/skills/robot-ssh/SKILL.md`; `README.md` layout; `docs/README.md` index; `CLAUDE.md`; `PLAN.md`.

## Tests

- `test_robot_profile.py`: both shipped profiles load; `check_profiles` passes on `config/robots/`; refusals (missing key, unknown model, duplicate hostname across two temp files, duplicate domain, missing calibration file, floor >= radius) name the file and key; `resolve_profile_path` by hostname, unknown hostname refused with the names listed.
- `test_lidar_contract.py`: `PROFILES` rows take the mount from the matching robot profile; the bernard-1 profile reproduces today's `LidarContract` values exactly; the contract files no longer contain the keys.
- `test_safety_logic.py` / `test_body_floor.py`: `real_limits` built from `safety_limits.yaml` plus bernard-1's profile reproduces today's `Limits` exactly; the fixture tests unchanged in outcome; the cross-file equalities become profile-internal checks.
- `test_rm_base_wheel_control.py`: `EncoderTracker(sign=-1)` is today's behaviour, `sign=+1` the un-negated one, anything else refused; a calibration file without `encoder_sign` is refused by the loader.
- `test_lidar_launch.py`: the driver block and the relay are selected from the profile; `lidar:=` and `lidar_yaw_deg:=` are gone; `run_stack.sh` and `stack_down.sh` source `robot_env.sh`; the domain refusal exists.
- `test_dashboard_sources.py` (or a new file): the snapshot carries `robot`; the page renders it.
- `test_docs.py` keeps every new doc indexed and under the cap.

## Commits (ncp_ws `feat/dynabarn-profile`, continuing)

1. `feat(rm_safety): robot profiles, one file per unit, resolved by hostname` (loader, the two profiles, tests, `robot_env.sh`, `run_stack.sh`, `stack_down.sh`).
2. `refactor(config): the mount measurements move from the contracts and the limits into the robot profile` (lidar_contract, safety_logic, roam, the nodes, the YAML files, tests).
3. `feat(rm_base): encoder_sign in the calibration file; bernard-3's calibration` (wheel_control, base_node, both calibration files, tests).
4. `feat(rm_bringup, rm_dashboard): the launch configures itself from the profile; the dashboard names its robot` (ncp.launch, slam.launch, dashboard, tests).
5. `docs: add a robot; the profile in every doc that named a unit`.
Suite green after each; push to the org (merging stays Daniel's).

## Verification

- Workstation: `python3 -m pytest tests/` green; `wc -w` under 600 for every doc touched.
- bernard-3: `deploy_to_robot.sh bernard-3`, rebuild `rm_safety rm_policy rm_base rm_bringup rm_dashboard`, `run_stack.sh` with NO arguments beyond the zones. Expect `robot: bernard-3 (domain 29)` from the scripts, the A1 driver and the 180° relay from the profile, the dashboard header reading `bernard-3`, `/base/debug` showing no `sign_fault` after a short forward jog, and the jog turning both ways. Battery above 11 V first.
- bernard-1, when it is back: confirm its hostname, deploy, rebuild, relaunch; the start lines must show domain 28, TG30, the same zones and floors as before; `/ncp_debug/m9` observation on a parked scan identical to the pre-change build.
- Both on: from bernard-3's shell `ros2 topic list` on domain 29 shows no bernard-1 topic; each dashboard shows only its own robot.

## Not in this plan

- The lidar remount on bernard-3 (its rear half is blocked at the mount; a profile cannot fix a blind sensor).
- Fitting bernard-3's base calibration from the tape (`docs/calibration.md`); the starting file gets it moving, not accurate.
- Per-robot roam limits beyond the body floor.
