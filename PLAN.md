> Status: approved 2026-10-23 (Daniel); BUILT the same day on `feat/roam-mode` (commits 64f1bda, b6e2322, 9b39e6c, 97e77f2, 25152f9, 75aa34e, c53afba; 611 workstation tests; pushed to GitHub). DEPLOYED to bernard-1 (70f4b77) and dry-run verified 2026-10-23; the first LIVE run drove into a wall (blind draw, zones off), fixed the same day (free-space draw, progress watchdog, dashboard refuses ROAM on with the zones off; see Found during implementation). OPEN: the live run on `m4` with the zones on and KILL in hand; `timeout_s` and `progress_window_s` set from a real run. Revised in place as implementation finds things.

> Status: the roam-mode plan (approved 2026-10-23) is BUILT and deployed. The retreat plan below (approved 2026-10-23 after a safety review) is BUILT and deployed (e3f758b, fd68947, 7b92dbd, 24ba505, 8738ef9), with two revisions found on the floor: a pure straight reverse also passes capped in the slow band and a retreat starts when pinned there without progress; and a retreat ends in a recovery turn toward the deepest free strip (any angle, operator's rule) before the next goal is drawn, with no per-goal retreat limit and no clearance gate on turning (operator's decisions). OPEN: the live run of the whole sequence.

# Plan: retreat after an e-stop, then a new goal (roam)

## Context

Roam now runs with both lidar zones on. When the robot creeps to within 0.15 m of something, rm_safety's stop zone latches: everything is zeroed and the latch holds until `/safety/reset`, which re-latches while the obstacle is still inside the zone. A roaming robot that reaches a wall is stuck until a human moves it. Daniel asked that on an e-stop the robot back along its most recent path a short way and then reset the goal, on the basis that the scene is static ("things won't be moving"), the path was just driven, and he is present with KILL.

Facts that shape the design (read from the code, and a safety-review pass over the draft):
- `safety_logic._evaluate_inner` precedence: KILLED, scan stale, odom stale, **stop zone (latched) returns a fully zeroed STOPPED decision and returns before the command-timeout check, the clamp and the acceleration limiter**, command stale, slow zone, nominal. Nothing distinguishes reverse from forward.
- The rear ~60 degrees of the scan is chassis-blocked, so a rear arc is impossible on this mounting. `rm_waypoint` never reverses for this reason; the jog allows reverse because a human watches.
- The stop radius (0.15 m) is INSIDE the chassis corners (0.19-0.23 m from the lidar), so any yaw while latched sweeps a corner through the obstacle's sector. The actor's own output can be `v < 0`, so a reverse tolerance on `wz` would let a network tick pass through a latched e-stop with no retreat involved. Reverse under the latch must be exactly straight.
- The wide-arc `min_r` in the room roam runs in sits at 0.41 m (objects beside the robot), so a release rule of "above the slow zone, 0.55 m" would never fire.
- Phase 2 states "latching: do not auto-resume when the obstacle clears". A release that depends on the obstacle leaving is an auto-resume. A release that fires only while a commanded retreat is being passed and the scan has read clear for N ticks is "a commanded retreat succeeded", a different claim; it is still a Phase 2 exception and is recorded as unsigned.

## Design

| Topic | Decision |
|---|---|
| Reverse under the latch (`safety_logic`, stop-zone branch only) | A command passes when ALL hold: the stop zone is enabled and latched; `estop.retreat_max_vx > 0`; the command is fresh (`command_age_ms` within `command_timeout_ms`); `cmd_vx` finite and `< 0`; `abs(cmd_vy)` and `abs(cmd_wz)` within the policy lateral epsilon, for every source. Output: `vx = max(cmd_vx, -retreat_max_vx)` then `apply_accel_limit` against `last_*` and `dt` (a bad `dt` gives zeros), `vy = wz = 0`. State SLOW (STOPPED always means zero output), reason `stop_zone_retreat`, latch kept, counter carried, `min_range` the wide arc, `clamping_active` True when the cap or ramp altered the command. Anything else in the branch stays the zeroed STOPPED decision. The pre-latch window is unchanged (a reverse there is scaled to zero, then latches within 150 ms and becomes a retreat). Kill, scan stale, odom stale keep precedence by position. |
| Release | A carried `retreat_clear_count` (the `consecutive_stop_count` pattern): incremented only on a tick where that reverse is being passed AND `min_r >= estop.retreat_release_range_m`; reset to 0 on any other tick. At `estop.retreat_release_ticks` the latch clears, both counters zero, and the tick falls through to the normal pipeline. A clear scan with no reverse commanded never releases (the auto-resume guard); a reverse that stops before the scan clears leaves the latch and the reset rule as today. `safety_node` logs the latched True to False edge with the reason and `min_r`. After release the node's own reverse command (-0.12) still passes through the normal pipeline for one or two status periods, under the clamp; accepted and written down. |
| Keys (`config/safety_limits.yaml` `estop:`) | `retreat_max_vx: 0.12` (the only measured reverse speed, 0.959 at 0.12 m/s; 0.08 sits at back-left's reverse dead band; absent key reads 0.0 = off, negative or non-finite refused), `retreat_release_range_m: 0.30` (twice the stop radius, inside the slow band so the policy resumes throttled; refused if not above `stop_zone_m`), `retreat_release_ticks: 10` (0.5 s at 20 Hz, five distinct scans; not the 3 used for latching). Every source gets the provision, the jog included: a latched robot can be backed out with S without RESET. Documented as a manual-path change. |
| Who reverses | `policy_node`, when roam is on, it is the SELECTED source, and its own last published `vx >= 0` (it drove forward into this). It subscribes to `/safety/status`, age-bounded by `odom_timeout_ms`, and that status feeds exactly one decision: the retreat. Nothing else in the node reads it (no pause on kill, no arm). `SafetyStatus.msg` gains `string command_topic` (the input topic rm_safety forwards), filled by `safety_node`, so a node compares it with its own `cmd_vel_topic`; the unselected node never retreats in bookkeeping. |
| The retreat | Straight back: publish `(-retreat_vx, 0, 0)` every tick without running the network and without touching `prev_action`. Under the uniform clamp the forward leg's curvature is preserved, so 0.3 m straight back departs from the driven path by 1-2 cm, inside the 0.2 m corridor the draw assumed; no trail, no steering. Success: displacement from the retreat start pose reaches `retreat_m` (0.3), or the latched set {STOPPED/stop_zone, SLOW/stop_zone_retreat} is left for OK or a non-retreat SLOW. Then the sequencer draws a new goal (reason `retreated`, through `_fix_goal`, the one goal writer). Timeout `retreat_timeout_s` (8): hold and warn. Abort (zero, no hand-off, current goal kept): KILLED, scan or odom stale, command timeout, a pose fault or pose jump, roam switched off, a stale or never-received status, the source switched away. At most one retreat per goal (reset in `goal_fixed`) and `retreat_cooldown_s` (20) between retreats; past either, hold and warn. A lateral latch backs the robot along the obstacle, the wide arc never clears, the retreat times out: accepted, documented. `dry_run`: nothing moves, timeout, hold: documented. |
| Status | `/ncp_debug` roam block gains `retreating`, `retreats`, `retreat_aborts`; the pill shows them. No new dashboard control. |

## Changes by file

**`config/safety_limits.yaml`**: the three `retreat_*` keys under `estop:` with rationale (the 2026-10-23 incident, the reverse-speed measurement, the corner geometry, the clutter range) and the Phase 2 exception named.

**`src/rm_safety/rm_safety/safety_logic.py`**: `Limits.estop` fields and `from_dict` (absent `retreat_max_vx` = 0.0; refusals as above); `EvalInputs.retreat_clear_count: int = 0`, `Decision.retreat_clear_count`; the stop-zone branch rewritten as: release test first, then the pure-reverse pass-through, else the existing zeroed decision; module docstring precedence text and reason vocabulary (`stop_zone_retreat`); the stale `safety_logic.py` comment that justifies unscaled yaw by "the swept circle sits inside the 0.35 m stop zone" gets a note that it is false at 0.15 m (an open item, not fixed here).

**`src/rm_interfaces/msg/SafetyStatus.msg`**: `string command_topic` with its comment. Robot rebuild of `rm_interfaces`.

**`src/rm_safety/rm_safety/safety_node.py`**: carries `retreat_clear_count`; fills `command_topic`; logs the release edge; startup line names the provision (speed, range, ticks).

**`src/rm_policy/rm_policy/retreat.py` (new, pure)**: `RetreatLimits` (from the `roam:` block: `retreat_vx`, `retreat_m`, `retreat_timeout_s`, `retreat_cooldown_s`), `RetreatController.step(now, status, pose, roam_on, selected, last_cmd_vx) -> Step(vx, wz, done, reason)` with the start conditions, success, timeout, abort and bounds above; `status()` counters. `status` is a small value object `(state, reason, age_s, command_topic)`.

**`src/rm_policy/rm_policy/policy_node.py`**: `SafetyStatus` subscription (depth 10) into `(state, reason, stamp, command_topic)`; the retreat step placed after the pose block and the first-map-pose fix and before the goal block; on `done` with success the sequencer's `retreated(now, scan)` draws and `_fix_goal(why=...)` fixes; on abort `_publish_zero(reason="retreat_aborted:<why>")`; the debug block fields; `goal_fixed` resets the per-goal retreat allowance; docstring rows and failure-mode bullets.

**`src/rm_policy/rm_policy/roam.py`**: reason `RETREATED`; `RoamSequencer.retreated(now, scan)`; the four `retreat_*` keys in `KEYS` and `check_limits` (`retreat_vx > 0`, `retreat_m > 0`, times `> 0`); `status()` carries `retreats`, `retreat_aborts`, `retreating` (fed by the node).

**`config/roam.yaml`**: the four keys with rationale; `retreat_vx <= safety max_vx` and `<= retreat_max_vx` pinned by a test across the two files.

**Dashboard** `index.html` `renderRoam`: `retreating` marker and the counters.

**Docs**: `docs/safety_validation.md` new §3e "Stop-zone retreat and scoped auto-release (2026-10-23), NOT SIGNED OFF" naming the auto-resume exception, the blind reverse under operator presence, the manual-path change; `src/rm_safety/README.md` precedence row for `stop_zone_retreat` and the release rule; `docs/roam.md` (what the robot does on an e-stop, what a hold after a retreat means, `dry_run`); `src/rm_waypoint/README.md` one line on jog reverse under the latch; `src/rm_policy/README.md` a row for `retreat.py`; `CLAUDE.md`: the re-opened decision scoped to the retreat, the new settled item ("a latched robot may back straight out under a fresh pure-reverse command; the latch releases only on N clear ticks during that reverse; nothing else re-arms it"), the footnote on "ROAM off never stops the robot" (it ends a reverse), and the yaw-at-0.15 m finding as an open item.

## Tests

- `tests/unit/test_safety_logic.py` (the `real_limits` / `_make_inputs` style, a latched close scan): pure reverse passes capped and ramped as SLOW/`stop_zone_retreat` with the latch kept and `min_range` the wide arc; any `wz` or `vy` (parametrized, including 1e-3) stays zero; forward and zero commands stay zero; a stale reverse stays zero; `dt = 0` gives zero; the key absent or 0 keeps today's behaviour bit for bit, negative or non-finite refused; steady-state never exceeds `retreat_max_vx`; kill, scan stale, odom stale win over the retreat; the pre-latch reverse is zeroed, not retreated; teleop source gets it too; release needs N clear ticks WITH an active reverse, the counter resets when the reverse stops, a clear scan without reverse never releases, `retreat_release_range_m <= stop_zone_m` refused, an obstacle returning after release re-latches after three; a sweep pinning `state == STOPPED` implies zero output; `test_safety_zone_switches.py`: unreachable with the stop zone disabled.
- Replay: the hold-scan fixture and the 600-scan bag through `evaluate()` with a reverse while latched never release (they are not clear to 0.30 m).
- `tests/unit/test_retreat.py`: starts only on fresh STOPPED/stop_zone with roam on, selected, last `vx >= 0`; commands exactly `(-retreat_vx, 0, 0)`; success on displacement or on leaving the latched set for a driving state; abort on each listed cause; timeout into hold; one per goal and the cooldown; counters; the shipped keys load and `retreat_vx` is within both safety bounds.
- Source pins: the status subscription exists and is used in exactly one method; the retreat step's position in the tick; `_fix_goal(` with `why="retreated"`; `prev_action` untouched on the retreat path; `SafetyStatus.msg` has `command_topic`; `safety_node` fills it; `rm_dashboard` still has no publisher. The existing pin "no `SafetyStatus` in policy_node" is rewritten as the narrower invariant (this is the explicit re-opening).

## Commits (branch `feat/roam-mode`)

1. `feat(rm_interfaces, rm_safety): a fresh pure-reverse command passes under the stop-zone latch; the latch releases after N clear ticks during that reverse` (msg, config, logic, node, tests, README, safety_validation).
2. `feat(rm_policy): roam backs straight out of a stop-zone e-stop, then draws a new goal` (retreat.py, policy_node, roam.py, config/roam.yaml, dashboard pill, tests, docs).
3. `docs: brain and PLAN.md`.
Suite green after each; `py_compile` both nodes; deploy with an `rm_interfaces` rebuild on the robot.

## Verification

- Workstation: `python3 -m pytest tests/` green.
- Robot, killed, roam on: `/safety/status` reads KILLED, so no retreat can start; the pill shows no retreat; the safety start line names the provision; `ros2 topic echo /safety/status` shows `command_topic`.
- Live (operator, KILL in hand, zones on): on `m4` with roam, let it reach a wall. Expect in order: `stop_zone` latched, the m4 log `retreat: start`, `/safety/status` reason `stop_zone_retreat` with `/cmd_vel` vx about -0.12, the safety log `latch released after N clear ticks, min_range X`, the m4 log `goal fixed (roam #n retreated ...)` and a goal beside or behind. Record the measured reverse distance and the release `min_range` in CLAUDE.md; set `retreat_m` and the release range from them.

## Not in this plan

- A rear lidar arc (impossible on this mounting).
- Steered or trail-following reverse (refused by the review: a yaw tolerance under the latch is a scrape path and a network-reachable fail-open).
- Fixing the stale yaw-at-0.15 m justification in `safety_logic` (recorded as an open item).
