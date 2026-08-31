# ncp_ws — ROSMASTER X3 policy deployment pipeline

Runs a trained RL policy (a CfC / AutoNCP "liquid" network, trained in the BARN
navigation benchmark) on a Yahboom ROSMASTER X3 behind a hard safety layer,
with a frozen observation/action interface and full logging.

> **There is no physical E-stop on this robot.** `/safety/kill` is the only
> stop mechanism short of pulling the battery. Everything in this workspace is
> shaped by that fact.

---

## Start here

| I want to… | read |
|---|---|
| **run the robot** — connect, start the policy, open the dashboard | [`docs/operating_guide.md`](docs/operating_guide.md) |
| **understand the system** — data flow, packages, design decisions | [`docs/architecture.md`](docs/architecture.md) |
| know the exact observation/action layout | [`docs/interface_contract.md`](docs/interface_contract.md) |
| work on one package | `src/rm_*/README.md` |
| see the phase plan and sign-off gates | [`BUILD-PLAN.md`](BUILD-PLAN.md) |
| set up the dev workstation | [`docs/dev_env_setup.md`](docs/dev_env_setup.md) |
| check the safety sign-off record | [`docs/safety_validation.md`](docs/safety_validation.md) |

`BUILD-PLAN.md` is the authority on scope and phase gates.
`config/interface.yaml` is the authority on the data contract.
`config/safety_limits.yaml` is the authority on every safety number.

---

## The one invariant

**`/cmd_vel` has exactly one publisher: `rm_safety`.**

Command sources (`/policy/cmd_vel`, `/teleop/cmd_vel`) are *inputs* to the
safety node and have no route to the base driver. That includes manual
driving — the gamepad, waypoints and the WASD jog all publish
`/teleop/cmd_vel`, so they sit behind the same clamps, e-stop arcs,
acceleration limiter and kill as the policy.

Everything **fails closed**: any error publishes zero velocity rather than
letting the last command persist.

---

## The stack

```
YDLIDAR TG30 ──► /scan ──┬──► policy_node ──► /policy/cmd_vel ──┐
                         │                                       ├──► rm_safety ──► /cmd_vel ──► rm_base ──► wheels
                         ├──► scan_sanitizer ──► slam_toolbox     │
                         │                       (pose)           │
                         └──► rm_safety (e-stop, full resolution)  │
                                                                   │
  gamepad / rm_waypoint ──────► /teleop/cmd_vel ───────────────────┘
```

`rm_dashboard` subscribes to every stage and serves it at
`http://<robot>:8080/`. It publishes nothing.

## Packages

| package | target | job |
|---|---|---|
| [`rm_safety`](src/rm_safety/README.md) | robot | **Sole `/cmd_vel` publisher.** Clamps, e-stop, kill switch, watchdogs. |
| [`rm_policy`](src/rm_policy/README.md) | robot | Observation building, the vendored network, inference, scan bridges. |
| [`rm_base`](src/rm_base/README.md) | robot | Mecanum driver: raw PWM, bypassing the broken firmware speed PID. |
| [`rm_bringup`](src/rm_bringup/README.md) | robot | Launch files. `ncp.launch.py` starts everything. |
| [`rm_dashboard`](src/rm_dashboard/README.md) | robot | Live web view. Subscribe-only. |
| [`rm_waypoint`](src/rm_waypoint/README.md) | robot | Waypoints + WASD jog, for repositioning. |
| [`rm_interfaces`](src/rm_interfaces/README.md) | both | Custom msg/srv. |
| [`rm_tools`](src/rm_tools/README.md) | workstation | Offline analysis. Never a robot runtime dependency. |

---

## Platform

| | |
|---|---|
| Controller board | Jetson Orin Nano Super — L4T R36.4.3 / JetPack 6.2 |
| ROS2 distro | Humble, running **natively on the Jetson host** (not in a container) |
| Physical E-stop | **None** — `/safety/kill` only |
| `ROS_DOMAIN_ID` | **28** (both machines) |
| Lidar | YDLIDAR **TG30** (TOF), 360°, ~10.1 Hz. Encodes no-returns as `0.0`, not `inf`. |
| Robots | `bernard-1` at `192.168.4.100` (the working unit) · `bernard-2` at `bernard-2.local` |
| Interface version | **3.0.0** — 36-dim observation, 2-dim action `[vx, wz]` |

`bernard-1` has **no internet** and its clock resets ~3 months behind on every
boot. Everything reaches it from the workstation over LAN.

---

## Two machines, one repo

Code is authored on the **workstation** and delivered to the **robot** by push
over LAN SSH — not GitHub, which neither robot can reach.

```bash
bash scripts/deploy_to_robot.sh    # this repo -> ~/ncp_ws on the robot
bash scripts/deploy_upstream.sh    # NCP-on-Rosmaster + the trained actor.pt
```

**Never edit files on the robot.** If a hotfix is made there to unblock a debug
session, revert it and re-apply from the workstation before the session ends.

### Build targets are separate — do not co-install

```bash
# Robot
colcon build --symlink-install --packages-select \
  rm_interfaces rm_safety rm_policy rm_base rm_bringup rm_dashboard rm_waypoint

# Workstation
colcon build --symlink-install --packages-select rm_interfaces rm_tools
```

---

## Quick start (on the robot)

```bash
ssh bernard-1
cd ~/ncp_ws
bash scripts/stack_down.sh && bash scripts/prep_robot.sh   # must print "== ready =="
bash scripts/run_stack.sh with_slam:=true
```

Then open `http://192.168.4.100:8080/`.

Nothing can move yet — `rm_safety` starts **KILLED** and `command_source`
defaults to `teleop`. Arming takes two deliberate acts:

```bash
ros2 param set /safety_node command_source policy   # this force-kills
ros2 service call /safety/reset std_srvs/srv/Trigger
```

Stop button, in its own terminal:

```bash
ros2 service call /safety/kill std_srvs/srv/Trigger
```

Full procedure, including verification and troubleshooting:
[`docs/operating_guide.md`](docs/operating_guide.md).

---

## Layout

```
BUILD-PLAN.md   the authority on scope and phase gates
CLAUDE.md       the working-state project brain and session log
config/         interface.yaml (THE contract), safety_limits.yaml,
                base_calibration.yaml, waypoint.yaml, slam_toolbox.yaml,
                topics.yaml, fastdds_unicast_peers.xml
docs/           architecture, operating_guide, interface_contract,
                safety_validation, deployment, dev_env_setup
src/            the eight rm_* packages (see the table above)
scripts/        deploy, bringup/teardown, recording + diagnostics/
tests/          unit (no ROS runtime) -> replay (fixtures) -> hardware (human only)
```

## Testing

```bash
python3 -m pytest tests/          # unit + replay; no ROS runtime needed
```

Robot tier: **408 tests.** Workstation tier is smaller — no torch, numpy or
ncps here, so checkpoint tests skip.

**Hardware tests require a human operator with the kill command in hand and are
never run by an agent.**

---

## Status

Phases 1–3 are complete. The full pipeline runs on `bernard-1` hardware: one
command brings up ~12 nodes with exactly one `/cmd_vel` publisher, the trained
checkpoint (m4 BARN run, sha256 `3c9bf0ee112b…`) runs at 5 Hz, and the base
tracks commanded velocity to within 3% on all four wheels, tape-verified on
both axes.

**The plumbing is proven; the policy has not been evaluated as a policy.** The
checkpoint is a 30/100 held-out BARN run — judge the stack, not the policy.

Current clamps (`max_vx` 0.25, `max_wz` 0.8) are **above** the last signed-off
values (0.15 / 0.4). Revert for anything unattended. See
[`docs/safety_validation.md`](docs/safety_validation.md).
