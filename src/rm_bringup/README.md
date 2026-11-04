# `rm_bringup` — launch files

**Runs on: the robot.** No nodes of its own; it composes everything else.

`ncp.launch.py` is the one command that brings the whole stack up.

---

## The launch files

| File | Starts | Notes |
|---|---|---|
| **`ncp.launch.py`** | **the whole stack** | lidar + base + odom + safety + bridge + policy consumer + dashboard, behind toggles. **Refuses to start if a `safety_node` is already running.** |
| `safety.launch.py` | `rm_safety/safety_node` | Comes up KILLED regardless of arguments. |
| `base.launch.py` | `rm_base/rm_base_node` | Defaults `dry_run:=true`, `control_mode:=open_loop`. |
| `odom.launch.py` | `base_node_X3` + `imu_filter_madgwick` + EKF + `robot_state_publisher` | The vendor odom chain **without** the vendor driver. |
| `slam.launch.py` | `slam_toolbox` + `scan_sanitizer_node` | Publishes `map → odom`. `slam_toolbox` respawns on exit, which is what the dashboard's RESET MAP relies on; a crash therefore also comes back with a fresh map. |
| `dashboard.launch.py` | `rm_dashboard/dashboard_node` | For attaching to an already-running stack. |
| `waypoint.launch.py` | `rm_waypoint/waypoint_node` | Off by default in `ncp.launch.py`. |
| `teleop_joy.launch.py` | Yahboom gamepad, **remapped to `/teleop/cmd_vel`** | The vendor's own copy must be killed first. |

---

## `ncp.launch.py` — the arguments that matter

```
/scan (TG30, ~2000 pts, 360°)
      │
      ├─ scan_bridge_node ──► /ncp/scan (first profile's beams)  [upstream consumer only]
      │
      ├─ policy_node_m8v2 ──► /policy/m8v2/cmd_vel ──┐
      ├─ policy_node_m4   ──► /policy/m4/cmd_vel   ──┼──► rm_safety ──► /cmd_vel ──► rm_base ──► wheels
      └─ policy_node_m9   ──► /policy/m9/cmd_vel   ──┘    (command_source picks ONE row)
```

Node names, command topics and debug streams come from `config/topics.yaml`
`command_sources`, the table `rm_safety` validates `command_source` against
and the dashboard draws its source buttons from (loader:
`rm_safety/command_sources.py`). Every listed policy runs at once; switching
the live one is a parameter set, never a restart.

| argument | default | what it does |
|---|---|---|
| `command_source` | **`teleop`** | A `config/topics.yaml` row: `teleop`, `policy`, `m8v2`, `m4` or `m9`. Defaults to `teleop` so bringing the stack up to watch it **cannot** reach the base driver. Any other name is refused. |
| `consumer` | `policy_node` | Or `upstream` — runs `NCP-on-Rosmaster/rosmaster/node.py` unmodified, the only way to get its `mode:=debug` reactive rule. |
| `mode` | `debug` | Only meaningful for `consumer:=upstream`. `debug` = reactive rule drives, NCP runs in shadow. `ncp` = the network drives. |
| `profiles` | **`m8v2,m4,m9`** | Which trained policies to run, each as its own node on its own row: `m8v2` (720 beams over 270°, interface 4.0.0, `actor.pt`), `m4` (32 over 180°, 3.0.0, `actor_m4.pt`) and `m9` (720 over 270°, 5.0.0, `actor_m9.pt`, trained at the robot's clamps; see [`docs/add_a_policy_profile.md`](../../docs/add_a_policy_profile.md) for adding one). The scan bridge serves the first listed. `consumer:=upstream` needs `m8v2` first. The dashboard header shows which policy is live. |
| `interface_config_path` | from the table | Contract file override for one profile; pass `profiles:=<one>` with it. |
| `weights` | the profile's checkpoint | For one profile (`profiles:=<one>`). `weights:=''` is the explicit opt-in to **random weights**. A checkpoint of the other profile's width refuses to load. |
| `dry_run` | **`false`** | Note this differs from `base.launch.py`'s own default of `true`: bringing up the full stack implies intent to drive. |
| `control_mode` | `closed_loop` | The floor-validated path. |
| `with_lidar` / `with_base` / `with_odom` | `true` | Turn off only if already running. |
| `with_slam` | `true` | Default TRUE since 2026-08-31 — `/odom` yaw drifts ~0.6 deg/s at rest. Also switches `pose_source` to `map` unless overridden. |
| `pose_source` | follows `with_slam` | Set explicitly to `odom` **with** `with_slam:=true` to run SLAM while still driving on odometry — the safe way to watch a map build before trusting it. |
| `with_dashboard` | `true` | `dashboard_controls:=false` serves a read-only page. |
| `with_waypoint` | **`false`** | The only autonomous motion path here that is not the policy. |
| (roam) | runtime | Not a launch argument. Every policy node reads `config/roam.yaml`; `roam` is switched from the dashboard or `ros2 param set`. See [`docs/roam.md`](../../docs/roam.md). |
| `obstacle_dist` | `0.9` | Debug rule only, overriding upstream's 1.5. |

### The threshold ordering that must be preserved

```
0.90 m   policy spins away        (obstacle_dist)
0.55 m   safety scales velocity   (slow_zone_m)
0.15 m   safety zeroes + latches  (stop_zone_m)
```

The policy must react *before* the safety layer has to. Upstream's 1.5 m is
measured across the whole 270° arc **including the sides**, so in a normal room
something is nearly always inside it and the robot spins continuously instead
of ever cruising (measured: 380 spin samples to 49 cruise).

---

## Key design decisions

### `odom.launch.py` exists because the vendor bundle is not separable

The vendor `yahboomcar_bringup_X3_launch.py` bundles the odometry chain
together with two nodes we must **not** run, and gives no argument to disable
either:

- **`Mcnamu_driver_X3`** — `rm_base` replaces it, and the base-board serial
  port is single-writer, so running both makes both read garbage.
- **`yahboom_joy_X3`** — publishes `/cmd_vel` **directly**, which only
  `rm_safety` may do.

So this file starts the wanted subset directly:

```
rm_base /vel_raw      ──► base_node_X3        ──► /odom_raw
rm_base /imu/data_raw ──► imu_filter_madgwick ──► /imu/data
/odom_raw + /imu/data ──► robot_localization  ──► /odom
```

**Without `/odom` nothing moves.** `odom_stale` sits at precedence 3 in
`evaluate()`, *above* the zone checks, so every command is zeroed with reason
`odom_stale` regardless of what is sent.

`pub_odom_tf` defaults `false`: the EKF publishes `odom → base_footprint`, and
letting the integrator publish it too gives two writers on one TF.

### `teleop_joy.launch.py` turns a violation into a feature

The remap to `/teleop/cmd_vel` does not merely avoid the `/cmd_vel` clash — it
is where the architecture already wanted the gamepad. `/teleop/cmd_vel` is a
declared **input** to `rm_safety`, so manual driving now sits behind the same
clamps and e-stop arcs as the policy, and the gamepad becomes a genuine manual
override. That matters more than usual with no physical E-stop.

**The vendor's own copy still starts with every bringup — stop that PID first:**

```bash
pgrep -af '[y]ahboom_joy_X3'   # bracketed, so the pattern cannot match your own shell
kill <PID>
ros2 launch rm_bringup teleop_joy.launch.py
ros2 topic info /cmd_vel          # -> 0 publishers (before rm_safety starts)
ros2 topic info /teleop/cmd_vel   # -> 1 publisher
```

### `slam.launch.py` reads `/scan_sanitized`, never the raw `/scan`

~31–39% of TG30 rays are the `0.0` no-return sentinel, and slam_toolbox does
not know that. Untreated they are phantom hits at the sensor origin and scan
matching is worthless. It is also **not** `/ncp/scan` — that is 720 beams
reduced for the policy; scan matching wants every ray and the full 360°.

SLAM is additive: it publishes `map → odom` *above* the untouched EKF chain. It
**does not repair the odometry** — `/odom` remains untrustworthy for anything
reading it directly. Only `policy_node` with `pose_source:=map` benefits.

> **The lidar must run with `fixed_resolution:=true` whenever slam_toolbox
> does.** The TG30's ray count varies scan to scan (9 distinct values across 60
> consecutive scans); karto latches the first count and then **silently rejects
> every scan** — no map, no `map → odom`, no crash, no warning. It reads as
> SLAM failing to converge.

### Two safety_nodes is the failure this launch file guards against

`ncp.launch.py` **hard-aborts** if a `safety_node` is already running. Two of
those means two `/cmd_vel` publishers — the violation this workspace exists to
prevent — and it is a reliable proxy for "a stack is already up". A hard abort
rather than a warning, because a warning scrolls past in a launch printing
hundreds of lines. If the check itself cannot run it does not block the launch.

**Never run `bench_up.sh` and `ncp.launch.py` together.** `bench_up.sh` starts
its own safety, base, odom and lidar; it is for base-calibration work alone.

---

## Bringing the whole stack up

`ncp.launch.py` cannot do the host-level cleanup — killing processes and
creating a root-owned directory. Run `scripts/prep_robot.sh` first. The full
sequence is in [`docs/operating_guide.md`](../../docs/operating_guide.md).

```bash
bash scripts/stack_down.sh && bash scripts/prep_robot.sh   # must print "== ready =="
bash scripts/run_stack.sh command_source:=teleop with_slam:=true
```

> `pkill -f ncp.launch.py` **does not stop the stack.** It kills the launch
> parent and orphans every node. Use `scripts/stack_down.sh`.
