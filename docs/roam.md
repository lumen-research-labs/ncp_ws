# Roam: the robot picks its own goals

Roam makes each running policy node draw its next goal at random, in free space the lidar can see. A goal is finished when the robot arrives and has stood there for `dwell_s`, when it has been current for `timeout_s`, or when the distance to it has not shrunk by `progress_min_m` within `progress_window_s`. The node then draws a new robot-frame offset within `config/roam.yaml`, fixes it from its current pose, resets the recurrent state and keeps going. A finished goal is never retried.

Roam only moves the goal; the policy drives to it behind rm_safety's clamps, watchdogs and kill. It reads the lidar only to keep the next goal out of walls. It arms nothing.

## Before you start

- **Launch with both lidar zones on.** The launch defaults them off; on 2026-10-23 the first live run drove into a wall behind a blind-drawn goal. The dashboard refuses ROAM on while either zone is off.
  ```bash
  bash scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true
  ```
- **On a stop-zone e-stop the robot backs out by itself.** rm_safety passes one thing under its latch, a straight reverse at `retreat_vx`, and releases the latch after half a second of it with a clear scan. The node reverses up to `retreat_m` or `retreat_timeout_s`, then draws the next goal. The rear is unmonitored: a blind reverse, KILL in hand, never unattended. One retreat per goal, `retreat_cooldown_s` between retreats.

## Turn it on

1. Press ROAM on the dashboard and confirm, or from a robot shell `ros2 param set /policy_node_m4 roam true`. The current goal is kept.
2. Select the policy and ARM. A robot already at its goal sets off after the dwell.
3. Each hand-off is a `goal fixed (roam #n ...)` log line: reason (`arrived`, `timeout`, `stalled`, `retreated`), seconds the goal lasted, draws tried, totals.
4. `ros2 topic echo /ncp_debug/m4 --once` carries a `roam` block: the switch, the counters, `holding`, `retreating`, `goal_age_s`.

## The limits

| key | value | what it does |
|---|---|---|
| `range_min_m`, `range_max_m` | 0.6, 2.5 | Leg length; the shortest exceeds the 0.4 m latch. |
| `bearing_cap_deg` | 45 | Draw within this of straight ahead. |
| `corridor_half_width_m`, `clearance_m` | 0.20, 0.3 | The strip to the goal plus clearance holds no return. |
| `min_valid_fraction`, `self_radius_m` | 0.5, 0.25 | No-returns are unknown; the robot's own body is not an obstacle. |
| `draw_attempts` | 32 | Random candidates, then the shortest leg all round, before holding. |
| `dwell_s`, `timeout_s` | 1.0, 30 | Hold at the goal; the ceiling on one goal. |
| `progress_window_s`, `progress_min_m` | 10, 0.05 | No progress for this long: `stalled`. |
| `retreat_vx`, `retreat_m`, `retreat_timeout_s`, `retreat_cooldown_s` | 0.12, 0.3, 8, 20 | The reverse out of an e-stop. |
| `seed` | 0 | Every node draws the same sequence. |

Read at startup; tune from the logs of a real run.

## What will look like a bug

- ROAM off keeps the current goal; the robot drives there and latches. KILL is the stop. ROAM off during a retreat ends the reverse.
- **No candidate goal is in free space.** Boxed in: `holding` is true, the draw retries once a second. Jog it into open floor.
- **`retreat TIMEOUT`.** The latch did not release: wheels that did not move, `dry_run`, or an obstacle beside the robot. KILL, reset and jog it out.
- Counters climb while the robot is KILLED or the node is not the selected source; read them over an armed window.
