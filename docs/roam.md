# Roam: the robot picks its own goals

Roam makes each running policy node draw its next goal at random, in free space the lidar can see. A goal is finished when the robot arrives and has stood there for `dwell_s`, after `timeout_s`, or when the distance to it has not shrunk by `progress_min_m` within `progress_window_s`. The node then draws a new offset within `config/roam.yaml` from its current pose, resets the recurrent state and keeps going.

Roam only moves the goal; the policy drives to it behind rm_safety's clamps, watchdogs and kill. It reads the lidar to keep goals out of walls and to turn safely. It arms nothing.

## Before you start

- **Launch with both lidar zones on.** The launch defaults them off; the dashboard refuses ROAM on while either zone is off.
  ```bash
  bash scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true
  ```
- **A goal off the nose is faced first.** The node turns in place toward a goal more than `turn_tolerance_deg` off; M4 cannot steer to one beside or behind it.
- **Turns mind the corners.** They sweep 0.22 m from the lidar, outside the stop zone, so each turn goes the way whose swept arcs are clear (else the long way round, else no turn) and stops if anything comes within `turn_clear_m`.
- **Stopped by the stop zone, or pinned at its boundary with no progress, the robot backs out by itself.** rm_safety passes one thing there, a straight reverse at `retreat_vx`, and releases a latch after half a second of clear scan. The node reverses `retreat_m`, on to `retreat_max_m` while a latch holds, turns toward the most open floor, then draws the next goal. The rear is unmonitored: a blind reverse, KILL in hand, never unattended. Pinned retreats wait `retreat_cooldown_s`.

## Turn it on

1. Press ROAM on the dashboard and confirm, or from a robot shell `ros2 param set /policy_node_m4 roam true`. The current goal is kept.
2. Select the policy and ARM.
3. Each hand-off is a `goal fixed (roam #n ...)` log line with its reason, duration, draws tried and totals.
4. `ros2 topic echo /ncp_debug/m4 --once` carries a `roam` block with the switch, counters and states.

## The limits

| key | value | what it does |
|---|---|---|
| `range_min_m`, `range_max_m` | 0.6, 2.5 | Leg length. |
| `bearing_cap_deg` | 45 | Draw within this of ahead. |
| `corridor_half_width_m`, `clearance_m` | 0.20, 0.4 | The strip to the goal plus clearance holds no return. |
| `min_valid_fraction`, `self_radius_m` | 0.5, 0.25 | No-returns are unknown; the body is not an obstacle. |
| `draw_attempts` | 32 | Random candidates, then the shortest leg all round. |
| `dwell_s`, `timeout_s` | 1.0, 30 | Hold at the goal; the ceiling on one goal. |
| `progress_window_s`, `progress_min_m` | 10, 0.05 | No progress for this long: `stalled`. |
| `retreat_vx`, `retreat_m`, `retreat_max_m`, `retreat_timeout_s`, `retreat_cooldown_s` | 0.12, 0.3, 0.8, 8, 20 | The reverse out of an e-stop. |
| `turn_wz`, `turn_tolerance_deg`, `turn_timeout_s`, `turn_clear_m` | 0.5, 15, 12, 0.30 | Turning in place. |

Read at startup; tune from a real run.

## What will look like a bug

- ROAM off keeps the current goal and ends a retreat or a turn. KILL is the stop.
- **No candidate goal is in free space.** Boxed in; the draw retries once a second. Jog it into open floor.
- **`turn REFUSED`.** Something inside the corner sweep both ways; the policy gets the goal as drawn.
- **`retreat TIMEOUT`.** Wheels that did not move, `dry_run`, or an obstacle beside the robot. Jog it out.
