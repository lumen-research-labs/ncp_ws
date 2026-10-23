# Roam: the robot picks its own goals

Roam makes each running policy node draw its next goal at random, in free space the lidar can see. A goal is finished when the robot arrives and has stood there for `dwell_s`, when it has been current for `timeout_s`, or when the distance to it has not shrunk by `progress_min_m` within `progress_window_s`. The node then draws a new robot-frame offset within `config/roam.yaml`, fixes it from its current pose, resets the recurrent state and keeps going. A finished goal is never retried.

Roam only moves the goal; the policy drives to it behind rm_safety's clamps, watchdogs and kill. It reads the lidar only to keep the next goal out of walls. It arms nothing.

## Before you start

- **Launch with both lidar zones on.** The launch defaults them off; on 2026-10-23 the first live run drove into a wall behind a blind-drawn goal. With the zones on a wall stops the robot, the goal stalls and switches.
  ```bash
  bash scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true
  ```
- `m4` steers to the goal. `m8v2` ignores it (measured 2026-10-09), so with `m8v2` roam only advances the counters.

## Turn it on

1. Press ROAM on the dashboard and confirm, or from a robot shell `ros2 param set /policy_node_m4 roam true`. The node logs `roam ON`; the current goal is kept.
2. Select the policy and ARM. A robot already at its goal sets off after the dwell.
3. Each hand-off is a `goal fixed (roam #n ...)` line in the policy log: the reason (`arrived`, `timeout`, `stalled`), the seconds the goal was current, the draws tried, the totals.
4. `ros2 topic echo /ncp_debug/m4 --once` carries a `roam` block: `on`, `n`, `timeouts`, `stalls`, `held`, `holding`, `last_reason`, `goal_age_s`.

## The limits

| key | starting value | what it does |
|---|---|---|
| `range_min_m`, `range_max_m` | 1.0, 2.5 | Leg length. The shortest must exceed `goal_reached_dist` (0.4 m). |
| `bearing_cap_deg` | 45 | The draw stays within this of straight ahead. |
| `corridor_half_width_m`, `clearance_m` | 0.20, 0.5 | The strip to the goal plus the clearance must hold no return. |
| `min_valid_fraction` | 0.5 | No-return rays are unknown; a strip needs this share of valid rays. |
| `draw_attempts` | 32 | Random candidates, then as many at the shortest leg, before holding. |
| `dwell_s` | 1.0 | Hold at the goal before the next draw. |
| `progress_window_s`, `progress_min_m` | 10, 0.05 | No progress for this long: `stalled`, the goal switches. |
| `timeout_s` | 30 | The hard ceiling on one goal. |
| `seed` | 0 | Every node draws the same candidate sequence. |

The file is read at startup. Set the two windows from the elapsed seconds the hand-off lines report over a real run.

## Turn it off, and what will look like a bug

- ROAM off keeps the current goal. The robot drives there and latches. KILL is the stop.
- The pause at each goal is the dwell.
- **The robot stops and the log says no candidate goal is in free space.** It is boxed in: `holding` is true and the draw retries once a second. Jog it into open floor.
- Timeouts and stalls keep counting while the robot is KILLED or the node is not the selected source. Read the counters over an armed window.
- A manual re-target during roam is accepted; the clocks restart.
- Each node roams on its own scan, so two nodes' goals differ; the pill names the other node when their switches differ.
