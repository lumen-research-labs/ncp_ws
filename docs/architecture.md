# Architecture — how the whole thing fits together

For someone joining the project. Read this before the package docs.

- **What to *do*** → [`operating_guide.md`](operating_guide.md)
- **The frozen data contract** → [`interface_contract.md`](interface_contract.md)
- **Per-package detail** → `src/rm_*/README.md`
- **Phase plan and sign-off gates** → [`../BUILD-PLAN.md`](../BUILD-PLAN.md)

---

## 1. What this is

A ROS2 pipeline that runs a trained reinforcement-learning policy on a Yahboom
ROSMASTER X3 (a mecanum-wheeled robot on a Jetson Orin Nano), behind a hard
safety layer, with a frozen observation/action interface and full logging.

Two machines:

- **the dev workstation** — authoring, training, analysis. ROS2 runs in a
  container here.
- **the robot** — a Jetson running ROS2 Humble **natively** on the host. No dev
  tooling is installed on it. Code arrives by push over LAN, never edited in
  place.

The policy is a **CfC / AutoNCP** (liquid neural network) trained in the BARN
navigation benchmark. It is stateful — that recurrence turns out to drive
several design decisions further down.

---

## 2. The one invariant

> **`/cmd_vel` has exactly one publisher: `rm_safety`.**

Every other source of motion publishes to an *input* topic and reaches the
wheels only through the safety layer:

```
/policy/cmd_vel  ── policy_node
/teleop/cmd_vel  ── gamepad, rm_waypoint (waypoints + WASD jog)
```

There is **no bypass for anyone**, including the operator. Manual driving sits
behind the same clamps, arcs, acceleration limiter and kill as the policy.

This is enforced in four places, deliberately overlapping:

1. `policy_node` **refuses to start** if its `cmd_vel_topic` resolves to
   `/cmd_vel`.
2. `rm_waypoint`'s output topic is a **module constant with an assert**, not a
   parameter, so no launch argument can point it elsewhere.
3. `rm_dashboard` has **no publisher of any kind** — a test pins that it
   contains no `create_publisher` call.
4. `ncp.launch.py` **hard-aborts** if a `safety_node` is already running, since
   two of those is two publishers.

The vendor bringup violates it twice — `yahboom_joy_X3` publishes `/cmd_vel`
directly, and `Mcnamu_driver_X3` contends for the serial port — which is why
`rm_bringup` starts its own subset instead of the vendor bundle.

---

## 3. The governing principle: fail closed

Any error — a stale message, a NaN, an empty scan, an unhandled exception —
publishes **zero velocity**. Never a pass-through of the last command, never a
plausible-looking substitute.

There is **no physical E-stop on this robot.** `/safety/kill` is the only stop
mechanism. So:

- `rm_safety` comes up **KILLED** and there is no argument to override it.
  Launching a node can never make the robot move.
- The kill state **persists to disk**, and an unreadable state file is read as
  KILLED.
- `rm_base` defaults to `dry_run: true`. Motion needs `dry_run:=false` **and**
  an armed `rm_safety` — two explicit acts.
- Changing `command_source` **force-kills**, so switching sources can only
  stop the robot, never start it.

Three specific fail-closed choices are worth internalising, because each looks
like over-engineering until you know the incident behind it:

| choice | why |
|---|---|
| **"No valid perception" is a stop, not a clear path.** An empty-but-fresh scan produces `min_range = inf`, which is treated as blindness. | A review round found it bypassing the e-stop entirely. A real fail-OPEN bug. |
| **The scan bridge publishes NOTHING on a fault**, rather than an all-`range_max` scan. | "Clear path everywhere" is the single most dangerous thing to fabricate. The consumer's own deadman then stops the robot, needing no cooperation. |
| **No default goal is ever substituted.** Until a pose arrives, the goal does not exist and the policy does not drive. | A plausible-looking goal silently commands motion in an arbitrary direction — worse than not driving. |

---

## 4. The data flow

```
YDLIDAR TG30  ──►  /scan  (~2000 pts, 360°, 10.1 Hz)
                     │
      ┌──────────────┼───────────────────────────┐
      │              │                           │
      ▼              ▼                           ▼
 scan_sanitizer  policy_node                rm_safety
      │         (resamples                 (full-resolution
      ▼          internally)                 e-stop arcs)
/scan_sanitized       │                           ▲
      │               ▼                           │
      ▼          observation                      │
 slam_toolbox     724 dims                        │
      │               │                           │
      ▼               ▼                           │
  map → odom      NcpActor (CfC)                  │
      │               │                           │
      └───► pose ─────┤                           │
                      ▼                           │
              /policy/cmd_vel ────────────────────┤
                                                  │
   gamepad ──┐                                    │
   waypoint ─┴──► /teleop/cmd_vel ────────────────┘
                                                  │
                                                  ▼
                                             /cmd_vel  (20 Hz, ONE publisher)
                                                  │
                                                  ▼
                                    rm_base  ── mecanum IK ──► 4× raw PWM
                                                  │
                                                  ▼
                            /vel_raw, /imu/data_raw ──► EKF ──► /odom
```

`rm_dashboard` subscribes to **every arrow above** and serves it as one page.
It is a leaf; nothing depends on it.

### Rates

| topic | rate | why that rate |
|---|---|---|
| `/scan` | 10.1 Hz | the sensor |
| `/policy/cmd_vel` | **5 Hz** | half the sensor rate — 10 Hz has no headroom, 5 Hz gives 2:1 margin and matches the previous team's stack |
| `/cmd_vel` | 20 Hz | the safety republish rate; must be ≥ `zero_publish_rate_hz` for the continuous-zero-when-killed guarantee |
| `/odom` | 30 Hz | the EKF |
| `/base/debug` | 20 Hz | the base control tick |

**These are independent numbers.** The 5 Hz *policy* rate and the 20 Hz
*safety* rate are not the same clock and must not be conflated.

---

## 5. The packages

| package | runs on | one-line job |
|---|---|---|
| [`rm_safety`](../src/rm_safety/README.md) | robot | **The sole `/cmd_vel` publisher.** Clamps, e-stop arcs, kill switch, staleness watchdogs. |
| [`rm_policy`](../src/rm_policy/README.md) | robot | Observation building, the vendored network, inference, the two scan bridges. |
| [`rm_base`](../src/rm_base/README.md) | robot | Mecanum driver: raw PWM, bypassing the broken firmware speed PID. |
| [`rm_bringup`](../src/rm_bringup/README.md) | robot | Launch files. `ncp.launch.py` starts everything. |
| [`rm_dashboard`](../src/rm_dashboard/README.md) | robot | Live web view. **Subscribe-only, publishes nothing.** |
| [`rm_waypoint`](../src/rm_waypoint/README.md) | robot | Go-to-waypoint and WASD jog, for repositioning. Publishes `/teleop/cmd_vel` only. |
| [`rm_interfaces`](../src/rm_interfaces/README.md) | **both** | Custom msg/srv definitions. |
| [`rm_tools`](../src/rm_tools/README.md) | **workstation only** | Offline analysis. Never a runtime dependency of the robot packages. |

### A recurring structural pattern

Every package splits into a **pure core** and a **ROS shell**:

| pure (no ROS, no I/O, no clock) | ROS shell |
|---|---|
| `safety_logic.py` | `safety_node.py` |
| `scan_resampler.py`, `observation_builder.py`, `policy_runner.py`, `pose_source.py` | `policy_node.py` |
| `mecanum.py`, `pwm_map.py`, `wheel_control.py` | `base_node.py` |
| `waypoint_controller.py` | `waypoint_node.py` |
| `state_store.py`, `slam_health.py`, `web_server.py` | `dashboard_node.py` |

This is what makes 340+ tests runnable with no robot and no ROS runtime. Every
safety decision in the system is reachable from a unit test.

Pure modules share one convention: they return a **result object carrying
either a value or a `fault` string**, never a partial or plausible-looking
answer. The caller cannot reach the value without branching on `fault` first.

---

## 6. Why the robot's hardware shaped the software

Four hardware facts drove more design than any software consideration.

### The lidar reports no-returns as `0.0`, not `inf`

The YDLIDAR TG30 encodes "nothing there" as **`0.0`** — and the driver's
`invalid_range_is_inf` flag is **inert on this unit** (measured: 1982 points,
781 zeros, **0 inf**). Untreated, `0.0` reads as an obstacle touching the
sensor: it min-pools to zero and would latch the e-stop in every bin containing
a dropout. ~31–39% of rays in a normal room are dropouts.

Two nodes exist because of this: `scan_resampler` maps every invalid value to
`range_max` before pooling (the policy path), and `scan_sanitizer_node`
rewrites them to `+inf` at full resolution for **consumers we do not own**
(slam_toolbox), rather than betting the pose estimate on a dependency's
implementation detail.

### The base firmware's speed PID is broken

Every wheel reports its encoder with the **wrong sign** relative to what the
STM32's PID expects, so any nonzero command integrates the output straight to
the PWM rail and the wheel runs flat out **regardless of the commanded value**.
Not fixable in ROS; not fixable in firmware without a reflash on a robot with
no internet.

`rm_base` is the workaround: it does the mecanum kinematics in ROS, drives raw
PWM, and — because the firmware's *loop* was fine and only its *feedback
polarity* was wrong — runs its **own** closed loop on the sign-corrected
encoders, with three guards that all fail toward less authority.

### One wheel is sticky, and a dead wheel lies

Front-right needs roughly twice the PWM to break loose as to sustain. It gets
per-direction calibration and a bounded breakaway kick.

The diagnostic lesson generalises: **a dead wheel reads like a working one on
the floor**, because the other three drag it and its encoder reports the
*robot's* motion. It showed 0.250 m/s against a 0.254 target while contributing
nothing. Suspiciously *perfect* tracking is the tell. Diagnose wheels-up.

### The calibration is tape-fitted, not derived

`wheel_circumference_m` is measured; `counts_per_rev` and `lever_arm_m` are
**fitted to make commanded match achieved**. `lever_arm_m` is explicitly a
scrub compensation, not a geometric half-wheelbase — do not "correct" it.

**Odometry cannot check itself.** `/odom` over-reads linear distance by ~5%,
and a converging controller against wrong feedback looks exactly like success
in every ROS-side number. Only a tape measure caught it. Every calibration
claim in this project traces back to a physical measurement.

---

## 7. Consequences of the policy being a *trained* artifact

The checkpoint is fixed. Several things that look like bugs are properties of
what it learned, and fixing them means retraining.

| observation | why |
|---|---|
| **It does not stop at the goal.** | BarnEnv **terminated** the episode at `goal_dist < 0.4 m`, so every trajectory it learned from was cut at arrival. It was never once observed sitting at a goal. `policy_node`'s goal-reached **latch** supplies that termination. |
| **Past the goal it drives erratically.** | The goal normalization `xy/5 − 1` goes below −1 (to −2.0 dead astern) — values BARN never produced. It is **off-distribution**, so post-overshoot behaviour is not evidence about the policy. |
| **Stationary, `wz` limit-cycles every tick.** | The `previous_action` feedback with a frozen observation. It resolves with a moving goal. Do not diagnose yaw from a bench run. |
| **The action range is ~8× wider than the clamps allow.** | Intentional. The safety layer is the binding constraint by design. |

Three fidelity rules follow, and they are absolute:

1. **`vendor/ncp_policy.py` stays byte-exact.** Every hardware fix goes in our
   own modules — that is what `scan_bridge_node` and `rm_base` *are*. A test
   pins the vendored file against its upstream hash.
2. **No formula is reimplemented.** An earlier revision duplicated the
   normalization in pure Python for test convenience; it was removed. Fidelity
   to the checkpoint outranks the test runner.
3. **`load_state_dict` stays `strict=True`.** A stale vendor pin once meant a
   shape mismatch — and strict mode made it a **startup crash**, not a
   wrong-numbers run. Under `strict=False` the missing tensors would have been
   left at random init with no error anywhere.

Even the apparent typos are load-bearing: the yaw bound is **literally 3.14,
not `math.pi`**, because that is what the network was trained against.
"Correcting" it shifts the observation by 0.05% with no error anywhere — which
is precisely the kind of silent drift that invalidates a checkpoint.

---

## 8. Pose, SLAM, and why a stateful policy makes this delicate

`goal_local` is **2 of the 36 numbers the network reads**, and both are derived
from the robot's pose. With `pose_source:=odom` that pose is dead reckoning
with slip baked in.

`slam_toolbox` adds a **correcting frame above** the untouched EKF chain:

```
map --(slam_toolbox)--> odom --(EKF)--> base_footprint --> base_link
```

It **does not repair the odometry.** `/odom` remains untrustworthy for anything
reading it directly.

**A corrected pose is a discontinuous pose.** Every loop closure steps
`goal_local` — and the network carries recurrent state, so untreated it would
keep steering on a hidden history built toward where the goal *used* to be.
`policy_node` detects the jump and **resets the recurrent state**, the same
response it gives a stale-sensor gap and for the same reason: the input history
is no longer continuous.

The failure mode is **never** a silent fallback to `/odom`. A missing or stale
transform publishes zero with a `pose_*` reason.

Two operational traps live here:

- **slam_toolbox silently rejects every scan** if the lidar's ray count varies
  (karto latches the first count). No crash, no warning, no map. The lidar must
  run `fixed_resolution:=true`.
- **Picking the robot up breaks SLAM completely** — odometry reports no motion,
  so no scan is processed and the pose freezes at the old spot. Mapping mode
  has no relocalization. Reposition by **driving**, which is why `rm_waypoint`
  exists.

---

## 9. Testing tiers

| tier | where | needs |
|---|---|---|
| **unit** | workstation **and** robot | nothing. Pure modules only. |
| **replay** | workstation | recorded bags in `tests/fixtures/`. A self-checking CDR reader decodes rosbag2 sqlite3 directly, so no rclpy is needed. |
| **hardware** | robot | **a human operator with the kill command in hand. Never run by an agent.** |

The workstation tier is smaller than the robot's — the workstation has no
torch, numpy or ncps, so checkpoint tests skip there.

> **The workstation tier cannot catch a whole class of bug.** It never imports
> the ROS node modules (there is no rclpy here), so an undefined name in a node
> is invisible until it is launched on the robot. That has happened.

Run order is strict: unit → replay → hardware.

---

## 10. Operational hazards worth knowing before you touch anything

| hazard | signature |
|---|---|
| **A duplicate stack looks like a sensor or SLAM fault.** Two EKFs / two slam_toolboxes write the same TF edges, so the pose jumps between two estimators and the goal appears to teleport while the robot sits still. | Doubled rates on the dashboard. Check `ros2 node list \| sort \| uniq -d` **first**. |
| **`pkill -f ncp.launch.py` does not stop the stack.** It kills the parent and orphans every node. | The next launch silently gives you two of everything. Use `stack_down.sh`. |
| **The base-board serial port is single-writer**, and a Yahboom factory autostart grabs it **every boot**. | `/voltage`, IMU and `/vel_raw` all read **zero** while the topics keep publishing normally — which looks like dead hardware. `prep_robot.sh` clears it. |
| **QoS mismatch is silent after one log line.** A RELIABLE subscriber refuses a BEST_EFFORT publisher. | "No fresh scan" forever while the topic visibly publishes at 10 Hz. Looks exactly like a dead lidar. |
| **`set -u` breaks ROS's `setup.bash`.** | The launch silently never starts. |
| **`/tmp` is cleared on every Jetson reboot.** | Nothing operational may live there — that is why `run_stack.sh` is in the repo. |
| **bernard-1 has no internet and its clock resets ~3 months back on every boot.** | Cross-machine staleness checks fail closed permanently; bags carry wrong stamps. Push the clock by hand. |

---

## 11. Method lessons that cost real time

Kept because each one produced a *confident wrong answer*, not an obvious
failure.

- **Verify the precondition inside the measurement.** A drift probe printed
  "STATIONARY for 149 s" as a heading while never checking stationarity — the
  robot had been driven, and every number under it was wrong. The final probe
  asserts peak |vx| and |wz| and says so.
- **Odometry cannot check itself.** A converging controller against wrong
  feedback looks like success in every ROS-side number.
- **Map extent is not evidence of pose drift.** It measures what the lidar can
  *see*, not where the robot has been.
- **Never pool forward with reverse, or CW with CCW, in one accuracy fit.** The
  robot has a measured 4% forward/reverse and 9.7% CW/CCW asymmetry, so pooling
  reads a direction difference as "varies with speed".
- **Never fit a dead-band from a measurement that includes a turn.** Rotational
  scrub load gets misattributed to the dead-band, and the model then matches
  only at the speed it was fitted at.
- **Probe a trained checkpoint with a real scan, never a synthetic one.** An
  all-beams-at-max scan is far off-distribution and produces alarming output
  that says nothing about the policy.
- **A safety check that quietly degrades to a prompt is worse than no check.**
  A runway check silently became a Y/N prompt on 4 of 12 legs because it did
  not wait for DDS discovery.

---

## 12. Where the project stands

The **plumbing is done and proven on hardware**: one command brings up ~12
nodes with exactly one `/cmd_vel` publisher, the trained checkpoint runs at
5 Hz, and the base tracks commanded velocity to within 3% on all four wheels,
tape-verified on both axes.

What has **not** happened: the policy has never been evaluated *as a policy*.
Every observation so far is about the pipeline. The checkpoint is a 30/100
held-out BARN run, so mediocre navigation is expected — **judge the stack, not
the policy.**

See §4 of the project brain (`CLAUDE.md`) for the live open-items list.
