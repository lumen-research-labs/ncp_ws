# Roam: the robot picks its own goals

Roam makes each running policy node draw its next goal at random, in free space the lidar can see. A goal is finished when the robot arrives and has stood there for `dwell_s`, after `timeout_s`, or when the distance to it has not shrunk by `progress_min_m` within `progress_window_s`. The node then draws a new offset within `config/roam.yaml`, fixes it from its current pose, resets the recurrent state and keeps going.

Roam only moves the goal; the policy drives to it behind rm_safety's clamps, watchdogs and kill. It reads the lidar only to keep the next goal out of walls. It arms nothing.

## Before you start

- **Launch with both lidar zones on.** The launch defaults them off; on 2026-10-23 the first live run drove into a wall behind a blind-drawn goal. The dashboard refuses ROAM on while either zone is off.
  ```bash
  bash scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true
  ```
- **A goal off the nose is faced first.** The node turns in place toward a goal more than `turn_tolerance_deg` off, only with nothing within `turn_clear_m`; M4 cannot steer to a goal beside or behind it.
- **Stopped by the stop zone, or pinned at its boundary with no progress, the robot backs out by itself.** rm_safety passes one thing there, a straight reverse at `retreat_vx`, and releases a latch after half a second of it with a clear scan. The node reverses up to `retreat_m` or `retreat_timeout_s`, turns in place toward the most open floor (`recovery_turn_min_deg` to `max`), then draws the next goal. The rear is unmonitored: a blind reverse, KILL in hand, never unattended. `retreat_cooldown_s` between retreats.

## Turn it on

1. Press ROAM on the dashboard and confirm, or from a robot shell `ros2 param set /policy_node_m4 roam true`. The current goal is kept.
2. Select the policy and ARM.
3. Each hand-off is a `goal fixed (roam #n ...)` log line: reason (`arrived`, `timeout`, `stalled`, `retreated`), seconds the goal lasted, draws tried, totals.
4. `ros2 topic echo /ncp_debug/m4 --once` carries a `roam` block: the switch, the counters, `holding`, `retreating`, `turning`.

## The limits

| key | value | what it does |
|---|---|---|
| `range_min_m`, `range_max_m` | 0.6, 2.5 | Leg length; the shortest exceeds the 0.4 m latch. |
| `bearing_cap_deg` | 45 | Draw within this of straight ahead. |
| `corridor_half_width_m`, `clearance_m` | 0.20, 0.3 | The strip to the goal plus clearance holds no return. |
| `min_valid_fraction`, `self_radius_m` | 0.5, 0.25 | No-returns are unknown; the robot's own body is not an obstacle. |
| `draw_attempts` | 32 | Random candidates, then the shortest leg all round. |
| `dwell_s`, `timeout_s` | 1.0, 30 | Hold at the goal; the ceiling on one goal. |
| `progress_window_s`, `progress_min_m` | 10, 0.05 | No progress for this long: `stalled`. |
| `retreat_vx`, `retreat_m`, `retreat_timeout_s`, `retreat_cooldown_s` | 0.12, 0.3, 8, 20 | The reverse out of an e-stop. |
| `turn_wz`, `turn_tolerance_deg`, `turn_timeout_s`, `turn_clear_m` | 0.5, 15, 12, 0 | Turning in place; 0 clearance: a pure rotation is accepted near things. |

Read at startup; tune from a real run.

## What will look like a bug

- ROAM off keeps the current goal; the robot drives there and latches. KILL is the stop. ROAM off ends a retreat or a turn.
- **No candidate goal is in free space.** Boxed in; the draw retries once a second. Jog it into open floor.
- **`retreat TIMEOUT`.** Wheels that did not move, `dry_run`, or an obstacle beside the robot. KILL, reset and jog it out.
