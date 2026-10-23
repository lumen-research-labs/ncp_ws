# Roam: the robot picks its own goals

Roam makes each running policy node draw its next goal at random. When the robot arrives at the current goal and has stood there for `dwell_s`, or when a goal has been current for `timeout_s` without an arrival, the node draws a new robot-frame offset within the limits in `config/roam.yaml`, fixes it from its current pose, resets the recurrent state and keeps going. A goal the robot takes too long to reach switches anyway, and is never retried.

Roam only moves the goal. The policy drives to it as to any goal, behind rm_safety's clamps, watchdogs and kill. Nothing here arms the robot or selects a source.

## Before you start

- Both lidar zones are off by default in `ncp.launch.py`, so `/safety/kill` is the only stop.
- `m4` steers to the goal. `m8v2` ignores it (measured 2026-10-09), so with `m8v2` roam only advances the counters.
- The draw stays within `bearing_cap_deg` of straight ahead. Both policies are off-distribution with a goal behind them.
- Give the robot open floor `range_max_m` deep all round.

## Turn it on

1. Bring the stack up with `bash scripts/run_stack.sh`. Each policy node logs its roam limits at startup.
2. Switch roam on: press ROAM on the dashboard and confirm (every running policy), or from a robot shell, one node at a time:
   ```bash
   ros2 param set /policy_node_m4 roam true
   ```
   The node logs `roam ON`. The current goal is kept.
3. Select the policy and ARM from the dashboard. A robot already at its goal sets off after the dwell.
4. Watch the policy node's log. Each hand-off is a `goal fixed` line whose reason starts with `roam #`: the ordinal, `arrived` or `timeout`, the seconds the finished goal was current, the running totals, and the new goal.
5. Read the counters on the debug stream:
   ```bash
   ros2 topic echo /ncp_debug/m4 --once | grep -o '"roam": {[^}]*}'
   ```
   The block holds `on`, `n`, `timeouts`, `last_reason` and `goal_age_s`, which counts up toward `timeout_s`.

## The limits

| key | starting value | what the node does with it |
|---|---|---|
| `range_min_m` | 1.0 | Shortest leg. Must exceed `goal_reached_dist` (0.4 m) or the node refuses to start. |
| `range_max_m` | 2.5 | Longest leg. Inside the odometry drift budget. |
| `bearing_cap_deg` | 45 | The draw stays within this many degrees of straight ahead. |
| `dwell_s` | 1.0 | Hold at the goal before the next draw. |
| `timeout_s` | 30 | Give up on a goal this old and draw the next one. |
| `seed` | 0 | Every node draws the same offset sequence. Change it for a different course. |

The file is read at startup, so a change needs a relaunch. Set `timeout_s` from the elapsed seconds the hand-off lines report over a real run.

## Turn it off, and what will look like a bug

- ROAM off (the button, or `roam false`) keeps the current goal. The robot drives there and latches. KILL is the stop.
- The pause at each goal is the dwell.
- Timeouts keep counting while the robot is KILLED, and on a policy node that is not the selected source. The node cannot see either. Read `timeouts` over an armed window.
- A manual re-target during roam is accepted. Roam stays on and both clocks restart.
- Each policy node roams on its own. Two nodes draw the same offsets but hand off at their own moments. The pill names the other node when their switches differ.
