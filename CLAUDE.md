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
| `ROS_DOMAIN_ID` | **28** — confirmed from the host banner 2026-08-08 |
| Robot address | `bernard-2` / `192.168.1.154` (mDNS `bernard-2.local`) |
| Camera | `astraplus` per host banner — NOT the Astra Pro the previous team documented |

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

Observation = **41 dims** (interface 2.0.0), in this order: lidar 36, base
velocity 3, previous action 2. Action = **2 dims `[vx, wz]`**, tanh `[-1,1]`.
Lidar is chassis-occluded to the rear: only the forward 180° is real,
min-pooled into 36×10° bins. Only the lidar block is normalized (fixed
`/range_max`); base velocity stays in raw units and **no placeholder normalizer
stats are to be fabricated**.

Two operator decisions on 2026-08-12 took this from 48/3 to 41/2:

- **No lateral strafe in the action space.** Shrinks the exploration problem.
  The X3 is still mecanum and the driver still accepts `linear.y` — this
  constrains the *controller*, not the robot. Three things must not be
  "tidied": the policy node sets `linear.y = 0.0` explicitly; `rm_safety` keeps
  its `max_vy` clamp and lateral e-stop arcs, because `/teleop/cmd_vel` is also
  an input and an operator may strafe while repositioning; and a non-zero
  `linear.y` on `/policy/cmd_vel` is a **bug, not a command** — zero it and
  warn, never clamp it silently.
- **No IMU block in the observation.** `/odom` is EKF-fused from the encoders
  and this same IMU, and arrives already rotated into the body frame; raw
  `/imu/data_raw` is in `imu_link`. What is given up is high-frequency linear
  acceleration and roll/pitch rate.

Base velocity keeps all 3 components (`vx, vy, wz`) despite the 2-dim action.
Observed lateral velocity is now purely a disturbance signal — roller drift, a
push, a caught wheel — and with the IMU gone it is the only route by which that
reaches the policy. Observation/action symmetry is not a goal.

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

- 2026-08-08: Phase 1 session, ended early on a dead battery. Full log in
  `docs/phase1_session_2026-08-08.md`. SSH working (`ssh bernard-2`, key
  `~/.ssh/bernard-2`). L4T R36.4.3 / JetPack 6.2 confirmed. Resolved: `/odom`
  exists (robot_localization EKF) and is now the base-velocity source;
  `/imu/data` is Madgwick-fused so we stay on `/imu/data_raw`; lidar is a
  separate `sllidar_ros2` launch; camera is `astro_pro_plus.launch.xml`.
  DDS root cause found: the AP drops multicast 239.255.0.1 (FastDDS SPDP)
  while forwarding 225.0.0.1, so `ros2 multicast receive` passing proves
  nothing. Fix committed as `config/fastdds_unicast_peers.xml`; both-ends
  application UNVERIFIED. Lidar never started (timeout) at 9.5 V battery.

- 2026-08-08 (session 2): DDS **working end to end** — workstation receives
  robot data. Fix was `maxInitialPeersRange` (default 4 < 8 nodes) in
  `config/fastdds_unicast_peers.xml`, applied on both ends. Rate table and TF
  tree measured (`docs/deployment.md` §6/§6b). Camera confirmed on
  `astro_pro_plus.launch.xml`. Two silent-error traps found: lidar frame is
  `laser_link` not `laser`, and **the IMU is rotated 180/0/-90 deg vs
  base_link** — blocks the Phase 3 freeze. Lidar is a **hardware fault**:
  zero bytes over raw serial to STOP/GET_INFO/GET_HEALTH.

- 2026-08-12: **Interface 2.0.0** — two operator decisions, both breaking.
  Action 3 -> 2 dims `[vx, wz]`: lateral strafe removed from the policy's
  action space to shrink the exploration problem (the base is still mecanum;
  this constrains the controller, not the robot). IMU block removed from the
  observation, 48 -> 41 dims, which retired the `imu_link` rotation blocker by
  sidestepping it — `/odom` is EKF-fused from the encoders and the same IMU and
  already arrives body-frame. Base velocity deliberately keeps all 3 components
  as a disturbance signal. Updated `config/interface.yaml` (authority),
  `config/safety_limits.yaml`, `docs/interface_contract.md`, `BUILD-PLAN.md`
  and this file. The safety layer keeps its `max_vy` clamp and lateral e-stop
  arcs — teleop is also an input to it. Contract is still DRAFT, still blocked
  on `/scan` for `control.rate_hz`.

  Also this session: fixed the Claude Code config. `build-check.sh` had never
  run (unregistered, and gated on a nonexistent agent name); the `/cmd_vel`
  ownership hook moved PostToolUse -> PreToolUse so violations never land; added
  a `robot-ssh` skill encoding the SSH/DDS/battery traps. Commit 5bf166e.


## 4. Current Goal

Phase 1 is **substantially complete except the lidar**. Tasks 1 (SSH + DDS),
3 (partial), 4 (rates) done; tasks 3 (lidar), 4 (`/scan` rate), 5 (baseline
bag) blocked.

**BLOCKER — the lidar is a hardware fault, not a config or power problem.**
Raw serial to `/dev/ttyUSB0` at 115200 returns **zero bytes** to `STOP`
(`A5 25`), `GET_INFO` (`A5 50`) and `GET_HEALTH` (`A5 52`), including with the
DTR motor line toggled. Those queries do not need the motor spinning, so this
is not a brownout. The CP2102 adapter enumerates fine, so USB and the cable to
the Jetson are good — the fault is between the adapter board and the lidar.
Operator to check, in order: (1) does it physically spin? (2) the JST connector
between lidar and adapter board, (3) try a different USB port to rule out the
adapter itself.

Battery is a secondary concern, not the cause: 9.5 V → 10.1 V after a short
charge → 9.9 V under load by session end. Still low for a 3S pack. Charge fully
to remove it as a confounder.

**RESOLVED 2026-08-12 by removal — the IMU rotation no longer blocks Phase 3.**
`imu_link` is rotated 180°/0°/−90° (RPY) from `base_link`, so the 6 raw IMU
observation dims were the only part of the vector in a non-`base_link` frame:
unrotated, forward acceleration lands on the wrong component and at least one
sign flips, silently. Interface 2.0.0 drops the IMU block entirely, so nothing
we build reads `/imu/data_raw` any more.

**Sidestepped, not solved.** The rotation is still live for
`robot_localization`'s EKF, which consumes the IMU to produce `/odom` and does
its own TF lookup — that is exactly why `/odom` is safe to use and raw
`/imu/data_raw` was not. It comes back the moment anyone re-adds a raw IMU
channel, which is an `interface_version` bump **and** requires resolving the
rotation first. The `imu_link` entry in `config/interface.yaml` stays for that
reason; it is not dead config.

Settled, do not re-open:
- Observation stays **forward-180°, 36 bins** despite the lidar being a 360° A1
  (operator, 2026-08-07). The robot is therefore blind behind — the safety
  layer treats reverse as unguarded motion.
- Base velocity is **`/odom`** (EKF, has a header) not `/vel_raw` (bare Twist,
  no timestamp — cannot support honest staleness detection).
- IMU source, *if one is ever re-added*, is **`/imu/data_raw`**; `/imu/data` is
  Madgwick-fused and its value is the orientation quaternion we do not use.
  Moot as of 2.0.0 — nothing subscribes to either.
- `ROS_DOMAIN_ID` is **28**, dictated by the Jetson host.
- Action space is **2 dims `[vx, wz]`**, no lateral strafe (operator,
  2026-08-12). The safety layer nonetheless keeps lateral clamps and arcs.
- **No IMU in the observation** (operator, 2026-08-12). Proprioception comes
  via `/odom`, which is EKF-fused from the encoders and IMU.

### Next session, in order

1. **Lidar hardware check** (above). Everything below except 5 depends on it.
2. Relaunch: bringup, then lidar **with `frame_id:=laser_link`** — the launch
   file defaults to `laser`, which is not in the TF tree and makes every scan
   untransformable.
3. Measure `/scan` rate → `docs/deployment.md` §6. **This is the last input to
   the Phase 3 control-rate decision.** Everything else sensor-side is ~10 Hz,
   so 10 Hz control would leave zero headroom; 5 Hz is the conservative option.
4. **Record a short static bag** — all sensor topics, robot stationary, ~60 s,
   no teleop, so an agent can do it unattended. Gives re-analyzable rate/jitter
   data (the current table is summary numbers only — `topic hz` discards its
   samples) and an early replay fixture. Trim and commit to `tests/fixtures/`.
   Also run `ros2 topic delay`, which was never captured; BUILD-PLAN task 4
   asks for latency as well as rate.
5. **Capture one raw `/scan` message to a file** and commit it to
   `tests/fixtures/`. A few KB. This is what the self-occlusion mask must be
   derived from — the previous team's `[-6,+6]` zone is NOT inheritable (their
   unit had an Astra Pro; ours is an Astra Plus, and on our measured geometry
   `camera_link` sits 1.4 cm forward of `laser_link` but 7.2 cm below it, so it
   may not occlude the scan plane at all).
6. Verify all five sensor streams in RVIZ2 from the workstation, no X11.
7. Teleop baseline bag — **operator only, never an agent**.
8. ~~Resolve the IMU rotation before touching Phase 3.~~ Closed 2026-08-12 by
   removing the IMU from the observation (interface 2.0.0). Nothing to do.

### Robot state at session end (2026-08-08)

I tried to stop the nodes I had launched (bringup + camera) at wrap-up, but the
robot had already gone off the network — either powered down or the battery
gave out again after reading 9.9 V under load. So they were not stopped
cleanly; they died with the machine. No harm done: none of them run at boot and
nothing persists, but the disorderly stop is worth knowing if the next session
finds anything odd.

The robot is otherwise untouched: no files written outside `/tmp`, no packages
installed, no repo cloned on it yet.

**Check the battery first thing next session** — this is twice now that it has
dropped off mid-session. If a full charge does not hold, the pack itself is
suspect, and that would need settling before any Phase 2 hardware testing,
where losing power mid-run with a policy driving is a genuine safety issue.

`/tmp/ncp_fastdds_peers.xml` is a copy of `config/fastdds_unicast_peers.xml`
placed there for this session; `/tmp` does not survive reboot, so re-copy it
next session (`scp config/fastdds_unicast_peers.xml bernard-2:/tmp/ncp_fastdds_peers.xml`).
The durable fix is to clone this repo onto the robot and point
`FASTRTPS_DEFAULT_PROFILES_FILE` at the in-repo copy.

### Remote-command gotchas (cost real time — do not rediscover)

- **Non-interactive SSH does not source `~/.bashrc`.** Ubuntu's bashrc returns
  early for non-interactive shells, before the ROS lines. `ssh bernard-2
  'ros2 topic list'` fails with an empty `ROS_DISTRO`. Every remote command
  needs the explicit source prelude — see `docs/phase1_session_2026-08-08.md`.
- **Never `pkill -f <pattern>` where the pattern appears in your own SSH
  command line.** It matches the remote shell running it and kills the session.
- `ros2 multicast send/receive` passing proves **nothing** about DDS. It uses
  group 225.0.0.1; FastDDS discovery uses 239.255.0.1, which this AP drops.
- A portless FastDDS initial-peer locator only covers participant IDs
  0..`maxInitialPeersRange`, **default 4**. The robot runs 8 nodes, so the
  default gives partial, silent discovery failure that looks like a flaky
  network. Set to 20 in our profile.

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
