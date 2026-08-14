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
| Robot address | `bernard-2` / `10.0.0.107` (mDNS `bernard-2.local`) — updated 2026-08-12 after network move |
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

- 2026-08-12: Moved project to a new PC and new WiFi network. Robot now at
  `10.0.0.107` (was `192.168.1.154`); workstation WiFi at `10.0.0.117` (was
  `192.168.1.144`). Added `bernard-2` entry to `~/.ssh/config` (key
  `~/.ssh/bernard-2`, user `jetson`, mDNS hostname). Updated IP addresses in
  `config/fastdds_unicast_peers.xml`. SSH and mDNS resolution confirmed working.
  DDS not yet re-verified on the new network — do this at the start of the next
  robot session.

- 2026-08-12 (session 2): Robot session. DDS **re-verified on the new network**
  (echoed `/voltage`, `/imu/data_raw`, `/vel_raw` from the workstation). Two
  findings, one new. **NEW — base-board serial contention:** on first bringup
  every STM32 value read zero (`/voltage` 0.0, IMU "free fall", `/vel_raw` 0)
  because a Yahboom factory **autostart**, `Rosmaster/rosmaster/rosmaster_main.py`,
  was holding `/dev/myserial` (ttyUSB1). Our driver's serial-receive thread had
  crashed with `SerialException: … multiple access on port`. Killed the app by
  PID, relaunched bringup → real data (IMU gravity −9.879, voltage 9.8 V). This
  **recurs every boot** until disabled; `sudo fuser -v /dev/ttyUSB1` is the check.
  Documented in the robot-ssh skill. **Lidar re-confirmed a hardware fault,
  now battery-independent AND port-contention-independent:** with ttyUSB0 free
  and the app dead, the sllidar driver opened the port (SDK banner) then failed
  `GET_INFO`/`GET_HEALTH` with `SL_RESULT_OPERATION_TIMEOUT`. Non-motor handshake,
  so not the 9.8 V battery. Also: this install ships **no** `sllidar_a1_launch.py`
  — use `sllidar_launch.py frame_id:=laser_link scan_mode:=Standard` (the generic
  default `Sensitivity` is an A2/A3 mode the A1 rejects). Skill launch reference
  fixed. Robot left clean (ports free, no procs), powered on for the operator to
  charge.

- 2026-08-12 (session 2, cont.): **Phase 2 safety-layer code written** (not yet
  hardware/replay validated). Built via Sonnet subagents with rm-safety-reviewer
  gating each step. `rm_interfaces/msg/SafetyStatus.msg` (state OK/SLOW/STOPPED/
  KILLED, reason, min_range, clamping_active; kill/reset use std_srvs/Trigger, no
  custom srv). `rm_safety/safety_logic.py` — pure, ROS-free decision core, all
  constants from `safety_limits.yaml`, fail-closed; **81 unit tests pass**.
  `rm_safety/safety_node.py` — sole `/cmd_vel` publisher, killed-on-startup,
  continuous-zero-when-killed, kill-state persistence (fail-closed to KILLED),
  std_srvs kill/reset, staleness watchdogs, `/safety/status` publishing; entry
  point `safety_node` wired. Two review rounds fixed real fail-OPEN bugs: an
  empty-but-fresh `/scan` bypassing the no-perception e-stop, and `/safety/reset`
  permitting motion with an obstacle still in the stop zone. evaluate() precedence:
  KILLED → scan_stale/no-perception → odom_stale → stop_zone(latched) →
  command_timeout → slow_zone → ok. Remaining Phase 2 (replay + hardware tests,
  safety_validation.md sign-off) is BLOCKED on the lidar/baseline bag.

- 2026-08-12 (session 3): **THE LIDAR WORKS — the "hardware fault" was a
  three-session misdiagnosis.** The unit is a **YDLIDAR TG30** (TOF; serial
  2025092500100084), NOT an RPLIDAR A1. It was being driven the whole time by
  SLAMTEC's `sllidar_ros2`, whose protocol a YDLidar cannot answer — so every
  `GET_INFO`/`GET_HEALTH` returned `SL_RESULT_OPERATION_TIMEOUT`. The port always
  opened (generic CP2102 USB-UART, VID:PID `10c4:ea60` — the same chip RPLIDAR
  uses, which is why both `rplidar.rules` and `ydlidar.rules` symlink it and the
  A1 label stuck). Two things had to change together: (1) **driver** —
  `ydlidar_ros2_driver` (SDK + driver already installed on the robot) at
  **baudrate 512000**, `lidar_type:=0` (TOF), `isSingleChannel:=false`,
  `support_motor_dtr:=true`; (2) **power** — the TG30 needs external motor power
  the A1 never did; the operator plugged the adapter's **micro-USB (power-only)
  into the powered USB hub** while USB-C stays the data link to the Jetson. Yahboom
  calls this the **"4ROS"** lidar variant; ground-truth config is in
  `yahboomcar_ws/.../yahboomcar_multi/launch/4ROS_multi.launch.xml`. Result:
  `/scan` (`sensor_msgs/LaserScan`) at a **rock-steady 10.17 Hz** (std dev ~1.5 ms),
  360° (−π..π), **2020 pts/scan**, range 0.01–64 m, `frame_id:=laser_link` (our
  override; the launch default `laser` is not in the TF tree). Battery 10.6 V,
  base board healthy. **This unblocks Phase 1 tasks 3/4/5 and Phase 2 validation.**
  Working launch (plain node, matches the 4ROS xml):
  `ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p port:=/dev/rplidar
  -p baudrate:=512000 -p lidar_type:=0 -p device_type:=0 -p sample_rate:=20
  -p isSingleChannel:=false -p support_motor_dtr:=true -p reversion:=true
  -p inverted:=true -p range_max:=64.0 -p range_min:=0.01 -p frequency:=10.0
  -p frame_id:=laser_link`.

- 2026-08-12 (session 3, cont.): Locked Phase 1 measurements into the contract
  (commit 94cad3e). **Control rate DECIDED = 5 Hz** (`/scan` 10.17 Hz, all
  sensors ~10 Hz → 10 Hz has no headroom; 5 Hz = 2:1 margin, matches cpt_b2).
  Set `interface.yaml` lidar `range_min_m 0.01 / range_max_m 30.0` (TG30 spec
  reach; the driver's advertised 64.0 is a generic Yahboom default, ignored for
  normalization) and corrected `sensor: rplidar_a1 → ydlidar_tg30`. **NEW
  SAFETY-CRITICAL FINDING** from a live scan (`tests/fixtures/tg30_scan_sample.yaml`,
  28% no-return in a static room): the TG30 encodes **no-return as `0.0`, NOT
  inf** (`invalid_range_is_inf:=false`). Untreated, 0.0 reads as an obstacle at
  zero distance, min-pools to 0, and would **latch the e-stop in every bin with
  a no-return**. The observation builder MUST map `0.0` and any `< range_min` to
  no-return → range_max, exactly like inf/NaN; preferred belt-and-braces is to
  also launch the driver with `invalid_range_is_inf:=true`. Documented in the
  `interface.yaml` preprocessing block. Also captured the first scan as a polar
  plot (`docs/tg30_first_scan.png`) and the raw message as a fixture. Robot left
  running (operator was about to power it down manually).

- 2026-08-13: **Phase 2 replay validation + safety launch file — offline
  session, robot untouched.** Three deliverables, all committed to the software
  test tiers (no hardware). (1) **Replay tests** (`tests/replay/`, 9 tests, pass):
  real recorded scans fed through the exact `safety_logic.evaluate()` with the
  real `safety_limits.yaml`, ROS-free. The 600-scan baseline bag is decoded
  straight from its sqlite3 store by a minimal, self-checking little-endian CDR
  reader (`tests/replay/laserscan_bag_reader.py`) — no rclpy/container needed on
  the workstation. **Validated the TG30 `0.0`-no-return safety finding: across
  all 600 scans (~27% zeros each) `safety_logic` never loses perception and
  never reads a `0.0` as a zero-distance obstacle** — `min_range` always finite,
  always the real ~0.21 m return. Also proved the all-`0.0` scan fails closed
  (STOPPED) and the latch engages after N=3 sustained stop-zone scans. (2) NOTE:
  the baseline bag was recorded in a **cluttered room with a real object ~0.21 m
  dead-ahead** (+6.7°, inside the 0.35 m stop zone), so its e-stop latch is
  CORRECT, not spurious — the bag validates 0.0-handling + latching, NOT a
  clear path. A **clear-path open-space bag is still an operator task**. (3) **New
  FINDING flagged, not fixed:** during the consecutive-scan filter window a
  sustained *stop*-zone obstacle emits OK/FULL velocity for the first (N−1)
  ticks (slow-zone scaling only fires on *slow*-zone readings), i.e. ~400 ms
  un-slowed at 5 Hz before the latch. Pinned by a regression test; written up in
  `docs/safety_validation.md` §2b for operator decision (recommendation: apply
  slow-scaling to un-latched stop-zone readings). (4) **`rm_bringup` safety
  launch file written** — `src/rm_bringup/launch/safety.launch.py` (Phase 2 task
  a), params from `safety_limits.yaml`, killed-on-startup preserved, `setup.py`
  now installs `launch/`. Byte-compiled + path-resolution checked offline; NOT
  colcon-built (ROS is containerised here). `docs/safety_validation.md` updated:
  unit (81) + replay (9) tiers marked PASS 2026-08-13; hardware tier still
  operator-only. Stray empty `src/rm_safety/logic.md` (0 bytes, untracked) left
  in place — origin unknown, flagged for the operator to delete.


## 4. Current Goal

Phase 1 is **substantially complete and the lidar is now UNBLOCKED.** Tasks 1
(SSH + DDS, re-verified on the new network), 3 (lidar — now working), 4 (rates,
incl. `/scan` at 10.17 Hz) done; task 5 (baseline bag) still to record but no
longer blocked. **Phase 2 code is written and reviewed** (see §3); its replay +
hardware validation is now unblocked too (needs the baseline bag recorded).

**RESOLVED 2026-08-12 (session 3) — the lidar was never a hardware fault; it is a
YDLIDAR TG30 driven by the wrong driver.** Full detail in §3. Summary: it needs
`ydlidar_ros2_driver` at **baudrate 512000** (`lidar_type:=0`, TOF, non-single-
channel, `support_motor_dtr:=true`) PLUS **external motor power** via the adapter's
micro-USB into the powered USB hub (USB-C = data to Jetson). Every prior session
used SLAMTEC's `sllidar_ros2`, which a YDLidar cannot answer — hence the
`SL_RESULT_OPERATION_TIMEOUT` that looked like dead hardware. `/scan` now streams
at 10.17 Hz, 360°, 2020 pts, `frame_id:=laser_link`. The device is a generic
CP2102 (`10c4:ea60`), so `/dev/rplidar` and `/dev/ydlidar` both symlink it — the
name means nothing; only the physical label (YDLidar) and the working driver do.
**Do not reintroduce `sllidar` for this unit.** Yahboom's name for this variant
is "4ROS"; ground-truth config is `4ROS_multi.launch.xml` in `yahboomcar_multi`.

Still TODO on the lidar, now that it runs: record the baseline/static bag and a
single `/scan` fixture (tasks 4/5 below); set `lidar.range_max` and the self-
occlusion mask in `config/interface.yaml` from a real captured scan (2020 pts,
0.01–64 m); write the `rm_bringup` lidar launch using the 4ROS params above.

**STANDING TRAP — Yahboom autostart grabs the base-board serial.** A factory
autostart, `python3 …/Rosmaster/rosmaster/rosmaster_main.py`, holds
`/dev/myserial` (ttyUSB1, the STM32) and recurs **every boot**. If it is up when
`yahboomcar_bringup` starts, the driver's serial-receive thread crashes
("multiple access on port") and **all** base data — `/voltage`, `/imu/data_raw`,
`/vel_raw` — silently reads zero while topics keep publishing at normal rate.
Preflight: `sudo fuser -v /dev/ttyUSB1`; if `rosmaster_main.py` holds it, kill
that PID (exact PID, not `pkill -f`) and relaunch bringup. See the robot-ssh
skill. (The durable fix is to disable the autostart on the Jetson, not yet done.)

Battery **charged — 10.6 V** under bringup at the start of session 3 (above the
10.0 V floor). Base board healthy after the standing-trap autostart kill.

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
- Observation stays **forward-180°, 36 bins** despite the lidar being 360°
  (a YDLIDAR TG30, 2020 pts/scan; operator, 2026-08-07). The robot is therefore
  blind behind — the safety layer treats reverse as unguarded motion.
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

0. **Robot preflight:** battery ≥ 10 V; re-copy the DDS profile to `/tmp`; after
   bringup, run `sudo fuser -v /dev/ttyUSB1` and kill `rosmaster_main.py` if it
   holds the port (the standing trap above), then confirm `/voltage` is non-zero.
1. ~~Lidar hardware check.~~ **DONE 2026-08-12 s3 — not a fault; YDLIDAR TG30,
   wrong driver. Use `ydlidar_ros2_driver` at baud 512000, see §3.**
2. ~~Relaunch lidar with `frame_id:=laser_link`.~~ **DONE — `/scan` streams with
   `frame_id:=laser_link`.** (The `laser` default is still the trap to avoid.)
3. ~~Measure `/scan` rate.~~ **DONE — 10.17 Hz, std dev ~1.5 ms.** Everything
   sensor-side is ~10 Hz, so 10 Hz control leaves zero headroom; **5 Hz is the
   conservative option** and now the standing recommendation for Phase 3. Still
   to write the rate into `docs/deployment.md` §6.
4. ~~Record a short static bag.~~ **DONE 2026-08-12 s3** —
   `tests/fixtures/ncp_static_bag` (60 s, 6125 msgs, all sensor topics, real
   data). Still TODO: `ros2 topic delay` for latency (BUILD-PLAN task 4 wants it).
5. ~~Capture one raw `/scan` message.~~ **DONE** — two fixtures:
   `tg30_scan_sample.yaml` (cluttered room) and `tg30_open_scan.yaml` (open
   space, the better basis). **PRELIMINARY occlusion finding (open-space scan,
   1 frame):** ZERO close returns (<0.35 m) anywhere in the forward 180° arc —
   all 43 near-returns were rear/side. Strong hint that on our unit the camera/
   body does NOT occlude the forward scan plane (matches geometry: `camera_link`
   is 7.2 cm BELOW `laser_link`), so the previous team's `[-6,+6]` Astra-Pro zone
   likely does not apply and the forward arc may need NO mask. **NOT yet
   final** — confirm with a multi-scan capture (~20 frames) that no forward angle
   has a *persistent* close return before removing the `self_occlusion_zones_deg`
   from `interface.yaml`. That multi-scan check is the one remaining robot task
   for the mask.
6. Verify all five sensor streams in RVIZ2 from the workstation, no X11.
7. Teleop baseline bag — **operator only, never an agent**.
8. ~~Resolve the IMU rotation before touching Phase 3.~~ Closed 2026-08-12 by
   removing the IMU from the observation (interface 2.0.0). Nothing to do.

**Phase 2 (safety layer) — software tiers done; hardware tier + one decision
remain.** `rm_interfaces`, `rm_safety/safety_logic.py` and
`rm_safety/safety_node.py` are written and reviewed. Status of the four items:
(a) **DONE 2026-08-13** — `rm_bringup` launch file
`src/rm_bringup/launch/safety.launch.py` (params from `safety_limits.yaml`,
killed-on-startup preserved; `setup.py` installs `launch/`; byte-compiled +
path-checked offline, NOT colcon-built). (b) **DONE 2026-08-13** — replay tests
`tests/replay/` (9 pass) against the baseline bag + TG30 captures, ROS-free via
a self-checking CDR reader; unit tier still 81 pass (total **90**). (c) hardware
safety checklist (BUILD-PLAN Phase 2 §Testing item 3) — operator + kill switch
in hand, wheels off the ground first, **never an agent** — STILL TODO. (d)
sign-off in `docs/safety_validation.md` — unit + replay rows filled PASS; the
hardware rows and final signature are the operator's — STILL TODO. Do not
`colcon build` on the workstation shell — ROS lives in a container here.

**OPERATOR DECISION PENDING (pre-hardware):** replay surfaced that a sustained
*stop*-zone obstacle emits full velocity for the first (N−1) filter ticks
(~400 ms un-slowed at 5 Hz) before latching, while a farther *slow*-zone
obstacle slows immediately. Written up in `docs/safety_validation.md` §2b.
Recommendation: apply slow-zone scaling to un-latched stop-zone readings. Not
changed by an agent — it is a safety-logic design call.

Also still open for Phase 2/1: a **clear-path open-space bag** (the current
baseline bag has a real 0.21 m forward obstacle, so it cannot serve as the
"no spurious e-stop on a clear path" replay case) — operator recording task.

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
