# Operating guide — connect, start the NCP policy, open the dashboard

Everything an operator needs to take the robot from powered-off to a running
policy, and to stop it again. Written to be followed top to bottom by someone
who has not seen this project before.

**Read [`architecture.md`](architecture.md) first if you want to know *why*
the steps are what they are.** This document is the *what*.

> ## Before anything else
>
> **There is no physical E-stop on this robot.** `/safety/kill` is the only
> stop mechanism short of pulling the battery.
>
> - **Keep a terminal open with the kill command already typed.** Do not
>   compose it under pressure.
> - **Never run a motion test without a person watching the robot.**
> - Hardware tests are **operator-only**. An agent must never run one.

---

## 0. The stop button

Open this in its own terminal, before you start anything, and leave it there:

```bash
ssh bernard-1
source /opt/ros/humble/setup.bash && source ~/ncp_ws/scripts/robot_env.sh
ros2 service call /safety/kill std_srvs/srv/Trigger
```

`robot_env.sh` puts the shell on THIS robot's ROS domain, from its profile
(`config/robots/<name>.yaml`). A stop button on another domain stops nothing.

The dashboard also has a **KILL** button, which does the same thing.

Killing is **latching**. The robot stays stopped, publishing continuous zeros
at 20 Hz, until someone explicitly calls `/safety/reset`.

---

## 1. Connect

### The three robots

| | `bernard-1` | `bernard-2` | `bernard-3` |
|---|---|---|---|
| address | `192.168.4.100` (static, `RobotLab` WiFi) | `bernard-2.local` (mDNS — its IP moves) | `192.168.4.102` (wired RobotLab segment; factory hostname `yahboom`, mDNS `yahboom.local`) |
| SSH key | `~/.ssh/lumen` | `~/.ssh/bernard-2` | `~/.ssh/lumen` |
| user | `jetson` | `jetson` | `jetson` |
| profile | `config/robots/bernard-1.yaml`: domain 28, TG30, yaw 0, the measured zones | none yet | `config/robots/bernard-3.yaml`: domain 29, RPLIDAR A1 mounted zero-backward (yaw 180), no zones, its own calibration file |
| status | the working robot; all validation is on this unit | not brought up on the current stack | stack and dashboard up 2026-11-17; base calibration a copy, lidar rear half blocked at the mount ([`add_a_robot.md`](add_a_robot.md)) |

**One profile per robot.** `run_stack.sh` resolves it from the hostname,
puts the stack on the profile's ROS domain and configures the lidar driver,
its yaw and the base calibration from it; no per-unit launch arguments.
bernard-1 and bernard-3 run on distinct domains, so both can be on at once,
each with its own dashboard and stop button. Workstation `ros2` tools see one
robot at a time: `export ROS_DOMAIN_ID=<its domain>`.

```bash
ssh bernard-1        # the alias is already in ~/.ssh/config
```

> **Load the `robot-ssh` skill before running any command on either robot.** It
> carries the full trap list — the `pkill -f` self-match, the by-id lidar port,
> the base-board serial contention, the DDS peers file. This guide covers the
> happy path only.

### Health check on arrival

```bash
# battery — below ~10.5 V, stop and charge. Motors misbehave on a low pack.
ros2 topic echo /voltage --once

# the clock. bernard-1 has no NTP and no internet; it resets to ~3 months
# behind on every boot. Bags recorded with a wrong clock are near-useless.
date
```

Push the clock from the workstation before recording anything:

```bash
# on the workstation
ssh bernard-1 "sudo date -s '$(date -u '+%Y-%m-%d %H:%M:%S')' -u"
```

### If you also want ROS topics on the workstation

DDS across this network needs `config/fastdds_unicast_peers.xml` applied on
**both ends**, with `maxInitialPeersRange` large enough for the node count.
A passing `ros2 multicast receive` is **not** evidence that DDS works — the
access point forwards `225.0.0.1` but drops `239.255.0.1`, which is the group
FastDDS actually uses for discovery.

**You do not need this to run the stack.** The dashboard is served from the
robot over plain TCP, so it works whenever SSH does.

---

## 2. Deploy code (workstation → robot)

Code is authored on the workstation and **never edited on the robot**.
bernard-1 has no internet, so GitHub is not in the path.

```bash
# on the workstation, from the repo root
bash scripts/deploy_to_robot.sh            # pushes this repo to ~/ncp_ws
bash scripts/deploy_upstream.sh            # pushes NCP-on-Rosmaster + actor.pt
```

`deploy_upstream.sh` verifies the sha256 of `policy.py`, `node.py` and
`actor.pt` on **both ends** and exits non-zero on any mismatch. That hash is
the same one `policy_node` logs at startup and publishes on `/ncp_debug` —
eyeball all three against each other.

Then build on the robot:

```bash
ssh bernard-1
cd ~/ncp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

A change to `rm_interfaces` (any `.msg` or `.srv`) **requires** this rebuild
before any node using it will run.

---

## 3. Start the stack

### Step 3a — tear down and prepare

**Always, every time, even on a fresh boot.**

```bash
cd ~/ncp_ws
bash scripts/stack_down.sh && bash scripts/prep_robot.sh
```

`prep_robot.sh` **must print `== ready ==`.** If it does not, read what it
says and fix it — do not proceed.

What it clears, and why each one matters:

| | |
|---|---|
| `Mcnamu_driver_X3` | the vendor base driver. The base-board serial port is single-writer; `rm_base` cannot open it until this is gone. |
| `rosmaster_main.py` | a Yahboom factory autostart holding the same port. **Recurs on every boot.** If it holds the port, `/voltage`, `/imu/data_raw` and `/vel_raw` all read **zero** while the topics keep publishing normally — which looks like dead hardware, not a conflict. |
| `yahboom_joy_X3` | publishes `/cmd_vel` **directly**, which only `rm_safety` may do. |
| `/var/lib/rm_safety` | created, so `rm_safety` can persist its kill state instead of degrading to memory-only. |

> **`pkill -f ncp.launch.py` DOES NOT STOP THE STACK.** It kills the launch
> parent and orphans every node. The orphans survive the next `ros2 launch`,
> giving you **two safety_nodes — two `/cmd_vel` publishers**, two EKFs
> fighting over one TF, and two lidar drivers racing for the port (which kills
> both, so the symptom you notice is a stale `/scan`, several layers from the
> cause). **Use `stack_down.sh`.**

### Step 3b — bring it up

```bash
bash scripts/run_stack.sh command_source:=teleop with_slam:=true
```

`run_stack.sh` sources the four ROS setup files a non-interactive shell does
not get from `~/.bashrc`, sources `robot_env.sh` (which prints
`robot: <name> (domain N, <profile>)` and refuses an unknown hostname), and
passes every argument straight through to `ncp.launch.py`.

This starts about 14 nodes: lidar → `rm_base` → odom chain → `rm_safety` →
scan bridge → `policy_node_m8v2`, `policy_node_m4`, `policy_node_m9` and `policy_node_dynabarn` →
dashboard, plus SLAM. All three trained policies run; `rm_safety` forwards only the one named by
`command_source` (§6).

**Nothing can move yet.** `rm_safety` comes up **KILLED**, and
`command_source` is `teleop` while nothing is publishing `/teleop/cmd_vel`.

### Step 3c — verify before arming

```bash
ros2 topic info /cmd_vel        # MUST say "Publisher count: 1"
ros2 node list | sort | uniq -d # MUST be empty — a duplicate is a second stack
ros2 topic echo /safety/status --once
```

Expected rates (check them on the dashboard; **doubled rates mean a duplicate
stack**):

| topic | expected |
|---|---|
| `/scan` | ~10.1 Hz |
| `/ncp/scan` | ~10.1 Hz |
| `/odom` | 30 Hz |
| `/policy/cmd_vel` | 5.0 Hz |
| `/cmd_vel` | 20.0 Hz |
| `/base/debug` | 20.0 Hz |

Watch it live with `python3 scripts/diagnostics/watch_stack.py`.

---

## 4. The dashboard

Open **`http://192.168.4.100:8080/`** in a browser on the workstation.

It is served from the robot over plain TCP, so it works whenever SSH does — it
does **not** depend on DDS reaching the workstation.

### What it shows

| panel | what to look at |
|---|---|
| **Scan** | raw `/scan` overlaid with the 32 policy beams, on one polar plot |
| **Observation** | the actual 724-dim vector the network consumed, as a bar strip |
| **Velocity chain** | asked-vs-allowed at every hop: policy → safety → base |
| **Safety** | state, `reason`, `min_range`, `clamping_active` |
| **Base** | per-wheel target/measured speed, PWM, and the `feedback_valid` / `saturated` / `stalled` / `sign_fault` flags |
| **SLAM** | the `map → odom` correction — magnitude, loop-closure count, age — plus the map |
| **Rates** | rate, jitter and staleness on **every** topic |

Two things worth knowing how to read:

- **`sign_fault` true on any wheel means STOP.** The encoder-polarity
  assumption the whole `rm_base` package rests on no longer holds.
- **The SLAM correction should *grow* while driving.** Flat zero means scan
  matching is contributing nothing. `/scan_sanitized` zero count **must be 0**.

### What the controls can do

The page can **kill**, **reset (arm)**, **switch `command_source`** (one
button per source: `teleop`, `policy`, `m8v2`, `m4`, `m9`, `dynabarn`; a policy button is
greyed until its node is publishing), **re-target the goal** on every running
policy, **switch ROAM** on every running policy (random goals; see
[`roam.md`](roam.md)), **send a waypoint**, **jog** with WASD, and **reset the SLAM map**
(which kills first and leaves the robot killed; see §8). The header names the
policy the page is following next to its checkpoint hash.

> **The server binds `0.0.0.0` with NO AUTHENTICATION.** Anyone who can route
> to the robot can arm and drive it. That is an accepted operator decision for
> an isolated lab network with no WAN uplink. On any other network, launch with
> `dashboard_controls:=false`, which serves a read-only page.

The dashboard **publishes no topic of any kind**. Everything above goes out as
a service call or a parameter set.

---

## 5. Arming the safety layer — the two explicit acts

Motion requires **two separate deliberate acts**, and no single button, launch
argument or parameter grants both.

```bash
# Act 1 — choose the source (m8v2 or m4 for a policy). This FORCE-KILLS, every time.
ros2 param set /safety_node command_source m8v2

# Act 2 — arm.
ros2 service call /safety/reset std_srvs/srv/Trigger
```

The dashboard's source buttons do Act 1; ARM does Act 2.

**Order matters.** Changing `command_source` force-kills, so doing it *after*
arming just disarms you again. Source first, then arm.

### What each state means

| `SafetyStatus.state` | meaning |
|---|---|
| `KILLED` (3) | hard kill. Continuous zeros at 20 Hz. Only `/safety/reset` clears it. |
| `STOPPED` (2) | a stop condition. See `reason`. |
| `SLOW` (1) | obstacle in the slow zone; velocity scaled down proportionally. |
| `OK` (0) | nominal; the command is passing through (possibly clamped). |

### Reading the `reason` field

Checks run in **precedence order** — the first one that fires wins:

| `reason` | what it means | typical fix |
|---|---|---|
| `killed` | not armed | `/safety/reset` |
| `scan_stale` | `/scan` old, **or no valid return anywhere in the arcs** | lidar down, or two lidar drivers racing for the port |
| `odom_stale` | `/odom` old | the odom chain is not running. **This is above the zone checks, so it zeroes everything you send.** |
| `stop_zone` | obstacle inside 0.15 m, **latched** | clear the obstacle, then `/safety/reset` |
| `command_timeout` | the active source went quiet | nothing is publishing on the source you selected |
| `slow_zone` / `stop_zone_prelatch` | obstacle in the 0.55 m slow cone | expected; velocity is being scaled |
| `ok` | nominal | — |
| `internal_error` | an exception inside `evaluate()` | a bug. Zero is published regardless. |

**A latched `stop_zone` needs `/safety/reset` after the obstacle is cleared.**
The reset is refused while an obstacle is still inside the stop zone.

---

## 6. Policy vs teleop — how the command sources work

**Only one command source is live at a time.** They are not merged, arbitrated
between, or fallen back to. `rm_safety` reads the selected one and ignores the
others entirely. The sources are the rows of `config/topics.yaml`
`command_sources`; `rm_safety` subscribes to all of them and refuses any
`command_source` that is not a row.

```
                                                command_source
/policy/m8v2/cmd_vel ── policy_node_m8v2 ──────────┐   selects ONE
/policy/m4/cmd_vel   ── policy_node_m4   ──────────┤
/policy/m9/cmd_vel   ── policy_node_m9   ──────────┤
/policy/dynabarn/cmd_vel ── policy_node_dynabarn ─┤
/policy/cmd_vel      ── upstream consumer / bench ─┼──► rm_safety ──► /cmd_vel ──► rm_base
/teleop/cmd_vel  ──┬── gamepad ────────────────────┘         ▲
                   └── rm_waypoint (goto + jog)              │
                                                      clamps, arcs,
                                                      accel limit, kill
```

| `command_source` | live input | who publishes it |
|---|---|---|
| `teleop` **(default)** | `/teleop/cmd_vel` | the gamepad (`teleop_joy.launch.py`) and `rm_waypoint` |
| `m8v2` | `/policy/m8v2/cmd_vel` | `policy_node_m8v2` (M8-v2, 720 beams, interface 4.0.0) |
| `m4` | `/policy/m4/cmd_vel` | `policy_node_m4` (M4, 32 beams, interface 3.0.0) |
| `m9` | `/policy/m9/cmd_vel` | `policy_node_m9` (M9, 720 beams, interface 5.0.0, trained at the robot's clamps) |
| `dynabarn` | `/policy/dynabarn/cmd_vel` | `policy_node_dynabarn` (DynaBARN-X3 from the lumen gauntlet, 720 beams, interface 5.1.0) |
| `policy` | `/policy/cmd_vel` | the upstream node (`consumer:=upstream`) or a bench tool |

### Why `teleop` is the default

Bringing the stack up to look at it must not be able to connect a policy to
the base driver. Switching to a policy row is a deliberate act — and because
the switch **force-kills**, it can only ever *stop* the robot, never start it.

### Manual driving is not a bypass

The gamepad and the waypoint controller publish `/teleop/cmd_vel`, which is an
**input** to `rm_safety`. Manual driving therefore sits behind exactly the same
clamps, slow and stop arcs, acceleration limiter and kill as the policy. There
is no path around the safety layer for anyone.

### Pausing the lidar zones (operator override)

The slow zone brakes before the policy has a chance to demonstrate obstacle
avoidance, which makes the policy impossible to evaluate. Both zones can be
switched off at bring-up:

**Since 2026-08-31 both zones default OFF in `ncp.launch.py`** (operator
decision), so `run_stack.sh` with no zone arguments has no proximity
protection. `config/safety_limits.yaml` still says `true`, so `bench_up.sh`,
`safety.launch.py` on its own and base-only work all keep the full e-stop —
only the full-stack policy launch opts out.

```bash
# the DEFAULT: both zones off, no proximity braking and no e-stop
bash scripts/run_stack.sh

# keep the e-stop, pause only the velocity scaling (the safer intermediate)
bash scripts/run_stack.sh stop_zone_enabled:=true

# restore the full safety layer
bash scripts/run_stack.sh slow_zone_enabled:=true stop_zone_enabled:=true
```

> **With `stop_zone_enabled:=false` there is no obstacle protection at all.**
> The clamps bound *speed*, not *proximity* — the robot hits the wall at
> `max_vx` instead of at 2 m/s. That is a smaller impact, not the absence of
> one. **`/safety/kill` is your only stop. Keep it typed in its own terminal
> and watch the robot.**

**What still works**, and is untouched by these switches:

| still active | still enforced |
|---|---|
| every velocity clamp (`max_vx` / `max_vy` / `max_wz`) | yes |
| every acceleration limit | yes |
| the policy `vy` violation check | yes |
| `/safety/kill` and killed-on-startup | yes |
| command / scan / odom staleness watchdogs | yes |

`min_range` is still **measured and published**, so the dashboard shows
proximity throughout — the layer simply does not brake on it.

**How to tell it is off:** `/safety/status` `reason` reads `ok_estop_disabled`
(or `ok_slow_zone_disabled` / `ok_stop_zone_disabled`) instead of `ok`, the
dashboard shows that verbatim, and `safety_node` logs a warning at startup and
again every 10 s.

**Turning it back on is the default** — drop the launch arguments and restart.
There is no way to re-enable it without a restart, and no way to *disable* it
at runtime: `ros2 param set` on either switch is **rejected**, so an
unauthenticated dashboard cannot switch the e-stop off mid-run.

One interaction worth knowing: with **both** zones off, an all-no-return scan
no longer stops the robot. That check asks "is anything visible in the
monitored arcs", and the arcs exist to feed the zones. Scan *staleness* — a
lidar that has stopped publishing — still stops the robot either way, because
`policy_node` reads the same scan.

### Current limits

| | value | note |
|---|---|---|
| `max_vx` | 0.25 m/s | **above** the signed-off 0.15 — revert for anything unattended |
| `max_vy` | 0.25 m/s | kept for teleop strafe; the policy never uses it |
| `max_wz` | 0.8 rad/s | **above** the signed-off 0.4 |
| `max_ax` | 0.75 m/s² | deceleration to zero deliberately bypasses this |
| slow zone | 0.55 m, 30° forward cone | lateral monitoring disabled |
| stop zone | 0.15 m, 90° forward + 60° lateral | latching |

The trained policy's own range is `vx ∈ [-1, 2]`, `wz ∈ [-3.14, 3.14]` — about
8× and 4× wider. **That gap is the design. Do not raise the clamps to "let the
policy through."** Since 2026-10-09 the clamps scale the whole command by one
factor, so a policy asking for 1.2 m/s with 0.36 rad/s gets 0.25 m/s with
0.075 rad/s: the same arc, five times slower. Before that, `vx` was capped
alone and the robot drove a five-times-tighter circle than the policy meant.

---

## 7. Running the policy

```bash
bash scripts/stack_down.sh && bash scripts/prep_robot.sh
bash scripts/run_stack.sh with_slam:=true

# verify (§3c), then:
ros2 param set /safety_node command_source m8v2
ros2 service call /safety/reset std_srvs/srv/Trigger
```

Watch `/ncp_debug/m8v2` and the dashboard.

### Options you will actually use

```bash
# prove the sensor path end to end with the reactive rule, no network involved
bash scripts/run_stack.sh consumer:=upstream mode:=debug

# exercise the plumbing with an untrained network (logs a warning); one profile
bash scripts/run_stack.sh profiles:=m8v2 weights:=''

# compute everything, publish everything, send only zero PWM
bash scripts/run_stack.sh dry_run:=true

# add the waypoint controller for repositioning between runs
bash scripts/run_stack.sh with_slam:=true with_waypoint:=true

# let the robot pick its own goals: the stack as usual, then ROAM on the
# dashboard (or `ros2 param set /policy_node_m4 roam true`); see docs/roam.md
```

### Choosing the policy

Two trained policies are deployed, each with the contract it was exported
against. By default both run, each as its own node on its own command topic,
and `command_source` picks which one `rm_safety` forwards.

| source | beams | contract | checkpoint | held-out / official BARN |
|---|---|---|---|---|
| `m8v2` | 720 over 270° | `config/interface.yaml` (4.0.0) | `actor.pt`, sha `71cc50ae…` | 63% / 0.219 |
| `m4` | 32 over 180° | `config/interface_m4.yaml` (3.0.0) | `actor_m4.pt`, sha `3c9bf0ee…` | 30% / 0.113 |
| `m9` | 720 over 270° | `config/interface_m9.yaml` (5.0.0) | `actor_m9.pt`, sha `bf4e1ba2…` | 52% (Gazebo rooms) / not run; follows the goal |
| `dynabarn` | 720 over 270° | `config/interface_dynabarn.yaml` (5.1.0) | `actor_dynabarn.pt`, sha `defd46db…` | 35% (DynaBARN-X3, fast 2D) / not run; moving obstacles |

Switch from the dashboard (press `m4`, `m8v2`, `m9` or `dynabarn`, confirm, then ARM) or from a
terminal:

```bash
ros2 param set /safety_node command_source m4     # force-kills
ros2 service call /safety/reset std_srvs/srv/Trigger
```

No restart: both nodes keep running and the switch is a parameter set. Goal
re-targets go to every running policy, so the one you switch to is already
aimed. The dashboard header reads `<policy> · checkpoint <hash> · interface
<version> · <n> beams`, which is how you confirm which one the page is
following; the safety panel's `command source` says which one drives.
`policy_node` builds the network to its contract's width and loads its
checkpoint strictly, so a mismatched pair refuses to start instead of driving
on wrong numbers.

To run one policy only, list it: `run_stack.sh profiles:=m4 with_slam:=true`.
The scan bridge serves the first listed profile's beams. `consumer:=upstream`
(the reactive debug rule) needs `m8v2` first and is selected with
`command_source:=policy`.

### Re-targeting the goal

The goal is a **static point**, fixed once from the first usable pose at
`(goal_x, goal_y)` ahead of the robot (default 2.5 m). Setting either parameter
re-fixes it from the **current** pose — always, even to an unchanged value —
and resets the network's recurrent state.

```bash
ros2 param set /policy_node goal_x 1.8
```

With ROAM on the node re-targets itself: after a dwell at each goal, or when
a goal has been held for `timeout_s` without arrival, it draws the next one at
random. See [`roam.md`](roam.md).

### Things that will look like bugs and are not

- **The robot stops at the goal and stays stopped.** That is the goal-reached
  latch. The trained policy has no stop-at-the-goal behaviour — BarnEnv
  terminated the episode on arrival, so it never learned one. Re-target to
  clear the latch, or switch ROAM on.
- **With ROAM on, the robot stops at each goal, then sets off somewhere
  else.** That is the dwell followed by the next random draw. ROAM off keeps
  the current goal; KILL is the stop.
- **`m8v2` roams but its path does not change.** It ignores the goal
  (2026-10-09), so roam only advances its counters.
- **ROAM on is refused with "zones off".** Roam needs both lidar zones;
  relaunch with `stop_zone_enabled:=true slow_zone_enabled:=true`.
- **Standing still, `wz` oscillates every tick and nets ~zero rotation.** A
  period-2 limit cycle driven by the previous-action feedback, which the
  acceleration limiter filters out. It largely resolves once the observation
  updates with real motion. **Do not diagnose yaw from a stationary bench run.**
- **In `mode:=debug`, the robot turns one way then equally far the other.** The
  debug rule picks spin direction by comparing scan halves with no hysteresis,
  so in a symmetric spot it flips every ~2 s. Put the robot somewhere clearly
  asymmetric. The vendored policy stays byte-exact, so this is not "fixed".
- **`wz` around 0.30 rad/s under-rotates by ~20%.** A measured rotational
  dead-band; a single `lever_arm_m` multiplier cannot cancel an offset.
  Inherent, not a regression.

---

## 8. Repositioning between runs

> **DO NOT PICK THE ROBOT UP.** slam_toolbox gates on odometry-reported motion,
> so a lift-and-place reports zero movement: no scan is processed, and the map
> and pose stay frozen where you lifted it. Mapping mode has no relocalization.
> **Recovery is the dashboard's RESET MAP button** (or a full stack restart).
> It kills the robot, restarts slam_toolbox with an empty map anchored on the
> odometry frame (the correction starts over at zero), cancels any waypoint run, and
> re-fixes the policy goal from the goal x / y fields. It takes a few seconds
> and leaves the robot KILLED; arm again afterwards. The button is greyed out
> unless the policy is in map mode and exactly one slam_toolbox is running.

Drive instead. Launch with `with_waypoint:=true`, then either:

- **click the map** on the dashboard and press **DRIVE THERE**; or
- **hold WASD / arrow keys** on the dashboard to jog.

Both go through `rm_waypoint` → `/teleop/cmd_vel` → `rm_safety`, so both need
`command_source:=teleop` **and** an armed robot. Jog allows reverse; an
autonomous waypoint does not (there is **no rear e-stop arc**).

The jog is a **deadman**: releasing a key sends nothing at all, and the command
simply expires after ≤0.5 s. A browser that closes, sleeps or loses WiFi
mid-keypress stops the robot for the same reason.

---

## 9. Recording a run

```bash
bash scripts/record_run.sh <name>     # runs alongside a live stack; starts nothing
```

**Push the clock first** (§1) or the bag carries wrong stamps.

The script prompts for **tape-measured start and end marks** and writes them
next to the bag. That is not optional bookkeeping: the bag can tell you what
`/odom` *claimed*, not whether it was true. Odometry cannot check itself — that
is the lesson that cost this project a stop-the-line and a full calibration
refit.

Pull bags back with `scripts/fetch_bags.sh`.

---

## 10. Shutting down

```bash
bash scripts/stack_down.sh
```

It removes the **command sources first**, while `rm_safety` is still alive and
flooding zeros, and stops `rm_safety` last. Stopping safety first would leave a
policy publishing into a base driver with no clamp in front of it.

It verifies at the end and reports anything it could not stop, rather than
claiming success.

> `bench_down.sh` **restarts** the vendor `Mcnamu_driver_X3`, which then holds
> the base-board serial port. Always re-run `prep_robot.sh` after it.

Then power the robot down at the switch.

---

## Quick reference

```bash
# stop
ros2 service call /safety/kill std_srvs/srv/Trigger

# full bringup
cd ~/ncp_ws
bash scripts/stack_down.sh && bash scripts/prep_robot.sh   # -> "== ready =="
bash scripts/run_stack.sh with_slam:=true

# verify
ros2 topic info /cmd_vel          # -> 1 publisher
ros2 node list | sort | uniq -d   # -> empty

# arm (source FIRST — it force-kills); m8v2 or m4
ros2 param set /safety_node command_source m8v2
ros2 service call /safety/reset std_srvs/srv/Trigger

# the other policy: a parameter set, not a restart (or the dashboard button)
ros2 param set /safety_node command_source m4

# dashboard
http://192.168.4.100:8080/

# teardown
bash scripts/stack_down.sh
```

---

## When something is wrong

| symptom | first thing to check |
|---|---|
| commands ignored, `reason: odom_stale` | is the odom chain running? `with_odom:=true` |
| commands ignored, `reason: killed` | you did not arm, or a `command_source` change re-killed you |
| `reason: scan_stale` while `/scan` publishes | two lidar drivers racing for the port — a duplicate stack |
| pose or goal drifts/teleports while stationary | **duplicate stack.** `ros2 node list \| sort \| uniq -d` **first** |
| rates doubled on the dashboard | duplicate stack |
| `/voltage`, IMU and `/vel_raw` all read zero | `rosmaster_main.py` is holding the base-board serial port |
| SLAM builds no map, no errors | the lidar is not running `fixed_resolution:=true` |
| upstream reports "no fresh scan" while `/ncp/scan` publishes | QoS mismatch — the bridge must publish RELIABLE |
| a launch "silently did nothing" | `set -u` in a script that sources ROS, or `/tmp` was cleared on reboot |
| `sign_fault` on any wheel | **stop.** The encoder-polarity assumption has broken. |
