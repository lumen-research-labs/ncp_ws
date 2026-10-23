> Status: approved 2026-10-23 (Daniel); BUILT the same day on `feat/roam-mode` (commits 64f1bda, b6e2322, 9b39e6c, 97e77f2, 25152f9, 75aa34e, c53afba; 611 workstation tests; pushed to GitHub). OPEN: the hardware verification below (deployed to bernard-1 for the dry run; the live run on `m4` is the operator's). Revised in place as implementation finds things.

# Plan: roam mode (random waypoints, the robot keeps running)

## Context

Daniel wants the real robot, running the deployed policies (`m8v2`, `m4`), to drive on its own: waypoints set at random, one after another, no operator click per goal, and **if the robot takes too long on a waypoint, the goal switches anyway**. Today the goal is one static point. `policy_node` fixes it from the first pose, latches at `goal_reached_dist` and holds zero until someone re-targets it (dashboard "re-target from here", a map click, or `ros2 param set`). Every waypoint costs a human action, and a goal the robot never reaches is held forever.

What the two policies do with a goal (measured, CLAUDE.md): `m4` steers to a goal ahead of it (weak response, off-distribution behind); `m8v2` ignores the goal and runs corridors from the lidar. Roam serves both: for `m4` it is random goal-seeking; for `m8v2` it keeps the arrival latch from parking the robot and gives a run structure (waypoint count, time per waypoint). The mode is policy-agnostic; a goal-trained policy (M9, not deployed, not in this plan) widens the bearing cap in config and nothing else changes.

Robot state: bernard-1 is back on the lab link (uptime 0 min at 07:40 UTC, no stack running, clock five months behind). Done already this session, outside this plan: the paused dataset pipeline on the box was resumed; the three repos were pushed to GitHub (`lumen-research-labs/ncp_ws` 5 branches, `NCP-on-Rosmaster` `feat/720-beam-270fov`, new private `optomotive` with every branch, default `feat/m9-gazebo`).

## Conventions

- Repo `~/lumen/ncp_physical/ncp_ws`, new branch `feat/roam-mode` off `feat/source-per-policy` (4c39f2f). Suite `python3 -m pytest tests/` (570 pass, 6 skip today; `policy_node.py` cannot be imported there, so byte-compile it with `python3 -m py_compile` before the robot build). `CODING_PROMPT.md`: suckless/DRY, no feature flags, config numbers in `config/*.yaml` with a rationale per number and a missing key is an error, loud errors, docs <= 600 words updated with the code, cruft archived never deleted, `PLAN.md` in the repo is the current plan, trailer `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`. Never merge; push the branch (Daniel asked for GitHub).
- Safety invariants unchanged: `/cmd_vel` has one publisher (rm_safety); policies publish on their own row topics; `/safety/kill` is the only stop; the dashboard publishes no topic; arming stays a separate act; no obstacle logic outside rm_safety.

## Design decisions

| Topic | Decision |
|---|---|
| Where roam lives | In `policy_node`, as a runtime-settable bool parameter `roam` beside `goal_x`/`goal_y`. A hand-off is a re-target through the one goal-writing function; no new node, package, topic, publisher, message or service. A separate sequencer node was rejected (a package, a service and a status message for the same behaviour). |
| **Two hand-off triggers** | **Arrived**: the latch is set and `dwell_s` has passed. **Timeout**: the current goal has been held for `timeout_s` without arrival, for any reason (blocked, stuck, the policy ignores it, killed), and the goal switches anyway. A timed-out goal is abandoned, never retried. Each hand-off logs its ordinal, reason, elapsed seconds, offset and world goal, and is counted on the debug stream. |
| Sampling | A robot-frame offset at the hand-off instant: distance U(`range_min_m`, `range_max_m`), bearing U(-cap, +cap), from the node's own seeded RNG. Starting values in `config/roam.yaml`, each with its rationale: 1.0-2.5 m (above the 0.4 m latch; inside the drift budget and the static default goal), `bearing_cap_deg` 45 (both policies only saw goals ahead; a 1.0 m goal at 45 deg normalises to -0.86, inside BARN's -0.92 floor), `dwell_s` 1.0 (rm_safety's limiter stops the robot in ~0.33 s; 5 ticks of `goal_reached` are visible), `timeout_s` 30 (the longest leg is 10 s at the clamp; M4 converges ~4x slower; the log carries elapsed seconds so the number is set from a real run), `seed` 0. |
| Clocks | The pure sequencer owns them: `goal_fixed(now)` on every fix, `set_on(on, now)` on the switch, `step(now, goal_reached)` once per tick after the latch. The timeout reference is `max(fixed_at, enabled_at)` and the dwell reference `max(reached_at, enabled_at)`, so switching roam on gives the current goal a full timeout and a latched robot a full dwell, with no spurious timeout. A repeated `on` moves no clock. A hand-off restarts the goal clock itself, so it cannot storm. |
| One goal writer | `_fix_goal(offset_x, offset_y, why)` becomes the only place the goal is written (today: first `/odom`, first map pose, parameter set; roam is the fourth). It rotates the offset by the current yaw, clears the latch, resets the recurrent state, tells the sequencer, logs one uniform line. Roam passes its drawn offset; it never overwrites the `goal_x`/`goal_y` node state, so `ros2 param get` stays truthful and a later one-axis set pairs with the operator's other axis. It also zeroes the fed-back previous action (training starts an episode at 0; "a new goal is a new episode"). Own refactor commit, suite green before and after. |
| Multiple policies | Each running policy node has its own sequencer and RNG (same seed, same offset sequence, different world goals because each hands off at its own moments). The live one (rm_safety's `command_source`) drives. The dashboard's ROAM button sets `roam` on every running policy, the way goals go to every running policy. |
| Kill | `policy_node` does not subscribe to `/safety/status` (pinned by a test): the proposer's behaviour must not depend on the arbiter's state, and "not driving" is also true of the unselected node, so a pause would be a half-truth. While KILLED, timeouts keep drawing goals (one INFO line per `timeout_s`, counted) and re-arming drives to the current one. Documented: read `timeouts` over an armed window. |
| On and off | ROAM on is confirmed on the page (an armed robot on a policy source moves at the next hand-off). ROAM off never confirms, keeps the current goal and does not stop the robot; KILL is the stop. A manual re-target during roam is accepted, keeps roam on and restarts the clocks. No launch argument for `roam` (a runtime operator act after bring-up, like arming); the launch passes only `roam_config_path`. |
| Config | `config/roam.yaml` is the authority: `range_min_m`, `range_max_m`, `bearing_cap_deg`, `dwell_s`, `timeout_s`, `seed`. Read at construction whether or not roam is on; a missing file, block or key, a non-number, incoherent limits, or `range_min_m <= goal_reached_dist` refuse to start with the path and key named. |
| Status | `/ncp_debug/<profile>` gains `"roam": {"on", "n", "timeouts", "last_reason", "goal_age_s"}`; the dashboard shows it for the active policy. |

## Changes by file

**`config/roam.yaml` (new).** The six keys above with a rationale paragraph per number and the "what it is not" preamble (roam only moves the goal; no obstacle logic, no retry, no command authority).

**`src/rm_policy/rm_policy/roam.py` (new, pure, no ROS).** `RoamLimits` (frozen dataclass of the six keys), `limits_from_dict(block, source)`, `check_limits(limits) -> str | None`, `load_roam_limits(path)`; `Handoff(reason, offset_x, offset_y, n, elapsed_s)`; `RoamSequencer(limits)` with `on/n/timeouts/last_reason` properties, `set_on(on, now) -> changed`, `goal_fixed(now)`, `draw()`, `step(now, goal_reached) -> Handoff | None` (order: no goal ever fixed -> None; record the arrival edge even while off; off -> None; reached and dwell elapsed -> ARRIVED; else goal older than timeout -> TIMEOUT), `status(now)`.

**`src/rm_policy/rm_policy/policy_node.py`.** Declare `roam` (bool, False) and `roam_config_path`; load and cross-check the limits before the runner is built; build the sequencer before any subscription. `_fix_goal` replaces the three goal-writing bodies (the two assignment lines and `_rotate_2d(` stay verbatim: `test_dashboard_map_goal.py` pins them). `_on_set_parameters` validates the whole set, accepts `roam` beside `goal_x`/`goal_y`, applies the goal first then `_set_roam` (INFO on/off lines with the limits and the totals). In the tick, directly after the GOAL REACHED latch block and before `_publish_zero("goal_reached")`: `handoff = self._roam.step(now, self._goal_reached)`; on a hand-off `_fix_goal(offset, why=f"roam #{n} {reason} after {elapsed:.0f}s; totals ...")` and the same tick drives to the new goal. Every fault return (stale scan/odom, pose, undefined goal) precedes it, so no hand-off happens without a trusted pose. `_publish_debug` adds `"roam": self._roam.status(now)`; docstring rows and failure-mode bullets.

**`src/rm_bringup/launch/ncp.launch.py`.** `_DEFAULT_ROAM_PATH` beside `_DEFAULT_TOPICS_PATH`; `"roam_config_path": _DEFAULT_ROAM_PATH` in every policy node's `params`; a docstring row. A test pins that the node declares the name (no inert override, the `command_source` failure class).

**`src/rm_dashboard/rm_dashboard/dashboard_node.py`.** Own refactor commit first: the per-policy loop of `_retarget(offsets)` becomes `_set_on_policies(requests, did)` (row name -> Parameter list or a caller refusal dict; result keys `ok`, `policies`, `applied`, `message`/`error`); `_retarget` keeps its name and signature and wraps it; the result key `retargeted` becomes `applied` at its four readers. Then `_CONTROLS` += `"roam"`; the `roam` command (after the `goal` branch) refuses a non-bool `payload["on"]` by name and applies `_bool_param("roam", on)` (new helper beside `_double_param`) to `_retarget_targets()` with `did="roam on|off"`. No snapshot change.

**`src/rm_dashboard/rm_dashboard/static/index.html`.** A row under the goal controls: button `ROAM: on|off|—` (class `on` when on) and pill `m4 · roam on · goals n · timeouts t · last arrived`, from the ACTIVE policy's debug `roam` field, plus `differs: m8v2 off` when the other running policy disagrees. `renderRoam(snap)` from `render`; enabled under the same `svc.policy_params` as the goal buttons. The handler sends an explicit `{on: true|false}`; enabling confirms, disabling never does.

**Fresh-PC docs (from the GitHub request).** `docs/dev_env_setup.md` rewritten for the verified workstation (Ubuntu 24.04 / Pop!_OS, ROS 2 Jazzy native, apt `python3-pytest python3-numpy python3-yaml python3-colcon-common-extensions`, clone `ncp_ws` and `NCP-on-Rosmaster` as siblings on their branch tips, `python3 -m pytest tests/`, `colcon build --packages-select rm_interfaces`, ssh config for the robots, the two deploy scripts); the old Docker/Arch version moves to `archive/docs/` with an index row. optomotive: a root `README.md` on `feat/m9-gazebo` naming the branches and pointing at `docs/README.md`, `docs/SIM_HOST.md`, `docs/M9_ROOMS.md`, `docs/M9_GAZEBO.md`.

## Tests

- `tests/unit/test_roam.py` (pure, stdlib + yaml): draws stay inside the range and cone (2000 draws, all x >= 0 at cap <= 90); same seed same sequence, different seeds differ; off never hands off; arrival hands off after the dwell, not before, with `elapsed_s`; timeout hands off and is counted; enabling while latched dwells from the switch-on; enabling with an old goal gives it a full timeout; re-enabling moves no clock; a hand-off restarts the goal clock (no storm); `goal_fixed` clears the arrival and restarts the timeout; no goal ever fixed means no hand-off; `status()` keys; the shipped config loads, passes its checks, `range_min_m` exceeds `SUCCESS_DIST` (regex over `vendor/ncp_policy.py`) and `range_max_m` is within `goal_default_m`; a missing key names the key and the file; non-numeric and non-integer seed refused; incoherent limits parametrized; a missing file names the path; no `roam:` block refused.
- Source pins (the repo's style, same file or `test_roam_wiring.py`): policy_node declares both parameters, loads the limits before the runner, the goal is written in exactly one place and all four sites call `_fix_goal`, `_roam.step(` sits between `goal_undefined` and `goal_reached` in the tick, `_on_set_parameters` accepts `roam` and still rejects the rest, `"roam": self._roam.status(` is published, no `SafetyStatus`/`/safety/status` in policy_node, still two publishers; the launch passes `roam_config_path` inside the per-profile loop and declares no `roam` argument; `_CONTROLS` has `roam`; the command refuses a non-bool before any set, goes through `_set_on_policies` to `_retarget_targets()` and touches neither arming nor the source; `_bool_param` uses `PARAMETER_BOOL`; `index.html` has `roambtn` and `roamstat`, `renderRoam` reads `r.on/n/timeouts/last_reason`, the handler confirms exactly once and only on enable, the button is gated on `policy_params`, KILL and cancel still never confirm; `config/roam.yaml` keys equal the dataclass fields; `docs/roam.md` exists, is indexed, is under 600 words and has no em-dash; every relative doc link in `README.md`, `docs/*.md`, `src/*/README.md` resolves (49 today, 0 broken); roam is linked from the operating guide and the three package READMEs.
- Existing suite stays green.

## Docs

- `docs/roam.md` (new how-to, <= 600 words): what it does, before you start (zones off, which policy steers, goals ahead, open floor), turn it on (dashboard or `ros2 param set /policy_node_m4 roam true`, arm, read the pill, the real log line from a run), the limits table, reading the counters (`ros2 topic echo /ncp_debug/m4`), turn it off and what will look like a bug (the dwell pause, timeouts counting while killed, `m8v2` roams but its path does not change).
- `docs/README.md` row; `docs/operating_guide.md`: the controls sentence, a usage line, a paragraph under "Re-targeting the goal", two bullets in "Things that will look like bugs"; `src/rm_policy/README.md`: a layout row for `roam.py` and one sentence in the latch section (2053 words already, so no more); `src/rm_dashboard/README.md` and `src/rm_bringup/README.md`: one paragraph and one row; `README.md`: `roam.yaml` in the config list; `CLAUDE.md`: a progress-log entry and two settled bullets (ROAM on asks, off never asks and never stops; policy_node does not read `/safety/status`).

## Commits (branch `feat/roam-mode` off `feat/source-per-policy`)

1. `docs: PLAN.md for roam mode` (this plan, revised in place as implementation finds things).
2. `refactor(rm_policy): one place fixes the goal (_fix_goal)`.
3. `feat(rm_policy): roam — the node draws its own next goal (config/roam.yaml, roam.py, policy_node, launch path, tests)`.
4. `refactor(dashboard): one aggregation for per-policy parameter sets`.
5. `feat(dashboard): ROAM toggle and pill; roam goes to every running policy`.
6. `docs: roam how-to, index rows, operating guide, brain entry`.
7. `chore: archive the Docker workstation doc` and `docs: workstation setup on a fresh PC`.
8. optomotive `feat/m9-gazebo`: `docs: root README for the GitHub repo`.
Suite green after each; `py_compile` the node; push `feat/roam-mode` and optomotive at the end.

## First action after approval

Bring the robot stack up: `prep_robot.sh` (base-board serial trap, clock push) then `run_stack.sh` with the deployed branch and the default `m8v2,m4` profiles. rm_safety starts KILLED; nothing moves. Dashboard at `http://192.168.4.100:8080`.

## Verification

- Workstation: `python3 -m pytest tests/` green; `py_compile` of `policy_node.py`.
- Robot, operator with KILL in hand: deploy the branch, restart the stack; the policy logs show the roam limits line; with the source on `teleop`, ROAM on from the dashboard gives hand-offs on timeouts only and no motion; switch to `m4`, arm: the robot drives to the goal, the log shows `roam #1 arrived after Ns`, the pill counts; block it: `roam #2 timeout after 30s` and the goal moves; KILL stops it; re-arm resumes; ROAM off leaves the current goal. Set `timeout_s` from the elapsed-seconds distribution. Record the result in `CLAUDE.md`.

## Found during implementation

- The `roam` launch argument proposed by the operator-side pass was dropped: roam is a runtime parameter only (no feature flags; switching it on is an operator act after bring-up).
- `docs/dev_env_setup.md` was not rewritten in place but replaced by `docs/workstation_setup.md`, so the Docker recipe could be archived in its own commit with no dangling link in between.
- The timeout starts at 30 s (not 60): the longest leg is 10 s at the clamp; the hand-off log line carries the elapsed seconds so the number is set from a real run.

## Not in this plan

- M9 on the robot (its export is a separate, known open item).
- A matching roam episode mode in barn-gym.
- Obstacle logic in roam (rm_safety owns it).
