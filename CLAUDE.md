# ROSMASTER X3 Policy Deployment (ncp_ws) — Project Brain

> Sections 1, 2, and 4 get OVERWRITTEN each time they change — do not append.
> Section 3 is the only append-only section.

## 1. What & Why

Lumen Research project (Harry Gandhi). Build a ROS2 pipeline that runs a
trained RL policy on a Yahboom ROSMASTER X3, behind a hard safety layer, with
a frozen observation/action interface and full logging. Two machines: dev
workstation (authoring, training, ONNX export) and the robot itself (Docker
container only, no dev tooling installed on it). Code moves workstation →
robot via `git pull`, never edited directly on the robot.

Full spec lives in `BUILD-PLAN.md` at repo root (note the hyphen — the plan
refers to itself as `BUILD_PLAN.md`, the on-disk name wins) — that document is
the authority; this file is the working-state summary of it.

### Platform facts (operator-confirmed 2026-08-07)

| | |
|---|---|
| Controller board | Jetson Orin Nano Super — TensorRT available, JetPack 6.x/L4T r36 |
| ROS2 distro on robot | Humble |
| Physical E-stop | **NONE.** `/safety/kill` is the only stop; battery pull is last resort |
| Network | Same subnet, direct DDS — no NAT/VLAN, no discovery server |
| `ROS_DOMAIN_ID` | **TBD** — dictated by the Jetson host, candidates 20 / 28 |

### Prior art — `LumenLabs/firmware` (cpt_b2)

The previous team ran a full stack on this same unit. Their repo is the best
source of ground truth about the hardware; read it before guessing.

- SSH: `ssh -i ~/.ssh/lumen jetson@<HOSTNAME>.local`, checkout at
  `~/Desktop/firmware`. (That key is **not** on this workstation yet.)
- **ROS2 Humble runs natively on the Jetson host.** Yahboom drivers launch on
  the host (`ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py`),
  not in a container. Our container joins their domain; it does not own it.
- Real topics: `/scan`, `/imu/data_raw`, `/imu/data`, `/imu/mag`, `/vel_raw`
  (bare Twist, **no timestamp**), `/voltage`, `/camera/*`. **`/odom` presence
  is unverified** — they never used it.
- Lidar is a **360° RPLidar A1**, not forward-only. Forward-180° is our
  choice, not a hardware limit. Self-occlusion zones `[-6,6]`, `[18,24]`,
  `[38,41]` deg — the first is the Astra Pro camera body **dead ahead** and
  must be masked or the e-stop latches permanently.
- Their engine ran at **5 Hz** (200 ms latency target) — strong prior for our
  control rate.
- Host↔container needs a FastDDS profile with `useBuiltinTransports=false`.
  Their docs call it a multicast fix; it is actually disabling shared-memory
  transport, which cannot cross the container boundary. Multicast discovery
  is unaffected, so workstation↔robot DDS should work normally.
- `cpt_b2_engine` publishes `/cmd_vel` **directly, with no safety layer**, and
  moves the robot the instant it starts. It must never run concurrently with
  `rm_safety` — that is two publishers on `/cmd_vel`.

**No policy has been trained and no task is defined.** The interface exists to
unblock Phases 1–3; those phases are exercised with a random/hand-scripted
placeholder policy and a stub inference call, not a real model. No ONNX
artifact exists yet.

Observation = 48 dims, in this order: lidar 36, base velocity 3, IMU angular
velocity 3, IMU linear acceleration 3, previous action 3. Action = 3 dims
`[vx, vy, wz]`, tanh `[-1,1]`, mecanum lateral DOF included. Lidar is
chassis-occluded to the rear: only the forward 180° is real, min-pooled into
36×10° bins. Only the lidar block is normalized (fixed `/range_max`); velocity
and IMU stay in raw units and **no placeholder normalizer stats are to be
fabricated**.

## 2. Structure & Architecture

```
ncp_ws/
├── BUILD-PLAN.md
├── docker/            # Dockerfile.jetson, docker-compose.yml — ROBOT side only
├── docs/               # interface_contract, dev_env_setup, deployment, safety_validation
├── config/
│   ├── interface.yaml      # THE FROZEN CONTRACT — obs/action shape, units, control rate
│   ├── safety_limits.yaml  # clamps, e-stop distances, timeouts
│   └── topics.yaml         # canonical topic names
├── src/
│   ├── rm_interfaces/  # custom msg/srv defs (ROBOT + WORKSTATION)
│   ├── rm_safety/      # ROBOT — sole publisher of /cmd_vel
│   ├── rm_policy/      # ROBOT — split: observation_builder / policy_runner / policy_node
│   ├── rm_bringup/     # ROBOT — launch files, params
│   └── rm_tools/       # WORKSTATION only — bag analysis, export, schema check
├── scripts/            # deploy.sh, fetch_bags.sh, check_interface.py
└── tests/
    ├── unit/           # pure-python, no ROS runtime
    ├── replay/         # nodes run against recorded bags
    └── fixtures/       # committed bags / synthetic data
```

Key interfaces / contracts:
- **`/cmd_vel`** — published ONLY by `rm_safety`. All other sources
  (`/policy/cmd_vel`, `/teleop/cmd_vel`) are inputs to the safety node, not
  direct paths to the base driver.
- **`config/interface.yaml`** — the frozen obs/action contract. A shared
  schema loader (imported by both robot and workstation code) is the single
  source of ordering — never hand-written twice. `interface_version` bumps
  on any change; the policy node refuses to start on a version mismatch.
- **`rm_tools`** — never a runtime dependency of `rm_safety` or `rm_policy`.

Fail-closed is the governing principle everywhere: any error (stale message,
NaN, exception) → publish zero velocity, never let the last command persist.

## 3. Progress Log

- 2026-08-07: Received full BUILD_PLAN.md. Set up Claude Code subagent
  scaffolding (rm-node-writer, rm-safety-reviewer, rm-test-runner) and two
  hooks (/cmd_vel ownership enforcement — blocking; interface_version bump
  reminder — warning). Phase 1 not yet started.
- 2026-08-07: All five open questions answered by the operator (see §1
  Platform facts). Created the repo skeleton per BUILD-PLAN §1: `git init`,
  five `rm_*` packages with empty manifests, all three `config/*.yaml`, all
  four `docs/*.md`, `docker/` (Dockerfile pinned to `TODO_PHASE1`),
  `scripts/` (deploy + fetch_bags real but gated on the robot address;
  check_interface.py a documented stub for Phase 3), `tests/` dirs,
  `.gitignore` blocking `.engine`/`.plan` and untrimmed bags. Nothing
  committed yet. `interface.yaml` is DRAFT not frozen — `control.rate_hz`,
  lidar `range_min/max`, and all TF transforms are `null` pending Phase 1
  measurements. Phase 1 task 1 blocked on robot SSH details.

- 2026-08-07: Committed the skeleton (f1c6b6f). Mined `LumenLabs/firmware`
  (previous team's cpt_b2 stack on this same robot) and the operator's
  Obsidian ROS notes. Corrected several assumptions: real topic names,
  360° lidar with front-dead-ahead self-occlusion, 5 Hz prior control rate,
  domain ID is host-dictated not free, `/odom` may not exist. Wrote
  `docs/dev_env_setup.md` from the Obsidian notes (workstation ROS2 is a
  Docker container, `osrf/ros:humble-desktop-full-jammy`, `--net=host`,
  X11 via XWayland). Robot unreachable — stale `~/.ssh/config` entries point
  at `10.0.0.149`, a different subnet from this workstation's 192.168.1.0/24.

## 4. Current Goal

Phase 1, task 1 — SSH + DDS connectivity, no X11 forwarding.

**Blocked on the operator for:** the robot's current hostname/IP on
192.168.1.0/24, and SSH auth (the `~/.ssh/lumen` key the firmware docs use is
not on this workstation). Everything else is now known — see §1 prior art.

Once unblocked, in order:
1. `printenv ROS_DOMAIN_ID` on the Jetson **host**; set both machines to match.
2. Talker on robot / `ros2 topic echo` on workstation. On failure check subnet
   → multicast → workstation firewall, in that order, before anything else.
3. `cat /etc/nv_tegra_release` on the robot; pin `docker/Dockerfile.jetson`.
4. Sensor bringup, verify all five streams in RVIZ2 on the workstation.
5. `ros2 topic hz` / `ros2 topic delay` for every topic → the table in
   `docs/deployment.md` §6. **This determines the Phase 3 control rate — it is
   measured, never guessed.**
6. Record a ~2 min teleop baseline bag, commit a trimmed fixture.

---

## Working conventions

- Build (workstation and robot targets are separate — don't co-install):
  `colcon build --symlink-install`
- Never build/cross-compile TensorRT artifacts on the workstation — TRT
  engines are built on-robot as a deploy step. Never commit a `.engine` file.
- Test order: unit (no ROS runtime) → replay (against `tests/fixtures/`
  bags) → hardware (requires human operator + kill switch in hand — never
  run by an agent).
- Docstring standard for every module: what it does, subscribes to,
  publishes, and its failure mode. ROS2 nodes additionally need the
  Subscribes/Publishes/Parameters/Failure mode header block.
- No magic numbers — timeouts/limits/rates are named constants sourced from
  `config/*.yaml`.
- Phase order is strict (see BUILD-PLAN.md execution table): Phase 2 and
  Phase 4 require explicit human sign-off before proceeding.
- With no physical E-stop, `rm_safety` must default to **killed on startup**
  (`safety_limits.yaml: startup.start_killed`). A node launching must never be
  able to make the robot move.
- Safety clamps are deliberately tighter than the interface action scaling
  (0.25/0.25/0.4 vs 0.3/0.3/0.5) so the safety layer is always the binding
  constraint. Both sets are provisional and raised only after Phase 2 sign-off.
