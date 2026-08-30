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
| Robot address | `bernard-2.local` (mDNS) — IP has changed 3x this project (last seen `192.168.0.190`, 2026-08-14); `~/.ssh/config` now resolves by mDNS, not a hardcoded IP |
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

- 2026-08-14: **Both pending operator decisions closed; robot clone attempted.**
  (1) Slow-scaling fix applied to `safety_logic.evaluate()`: the SLOW-ZONE
  branch (precedence 5) now covers `zone in ("slow", "stop")`, not just
  `"slow"`. `slow_scale()` already returns 0.0 for `min_r <= stop_zone_m`, so
  an un-latched stop-zone reading now scales to zero immediately (state SLOW,
  reason `stop_zone_prelatch`) instead of passing through at full commanded
  velocity for the pre-latch window. Removes the "closer obstacle brakes
  later" inversion flagged 2026-08-13. `docs/safety_validation.md` §2b marked
  RESOLVED. Three tests updated to match (`test_prelatch_window_emits_full_velocity`
  renamed to `test_prelatch_window_emits_zero_velocity`;
  `test_sample_scan_latches_after_consecutive_window` and
  `test_bag_latches_stop_on_real_forward_obstacle` updated to expect SLOW, not
  OK, on pre-latch ticks). Unit 81/81 + replay 9/9 still pass. (2) Occlusion
  mask dropped: operator confirmed (live scans, this session) no persistent
  close return anywhere in the forward 180° arc, consistent with the
  preliminary open-space-scan finding from 2026-08-12 s3 (camera_link sits
  7.2 cm below laser_link, so it likely never occluded the scan plane).
  `config/interface.yaml` `self_occlusion_zones_deg` set to `[]`; previous
  team's Astra-Pro-era values kept in a comment for reference only, not
  reinstated. `empty_bin_policy: fault` unchanged — still guards against
  genuine sensor fault/dropout, independent of masking. Not an
  `interface_version` bump (preprocessing detail, not a layout change). (3)
  **Robot clone attempted, blocked on GitHub access.** `~/ncp_ws` does not
  exist on the robot. `git remote` for this repo switched from HTTPS to SSH
  on the workstation (`git@github.com:devpatel2003/ncp_ws.git`) since HTTPS
  password auth is retired GitHub-side. The robot's existing deploy keys
  (`lumen_deploy`, `cradle_deploy`) are scoped to other repos, so a new
  dedicated deploy key was generated on the robot (`~/.ssh/ncp_ws_deploy`,
  alias `github-ncp` in the robot's `~/.ssh/config`), matching the existing
  per-repo pattern. **Waiting on the operator to register the pubkey as a
  read-only deploy key on `devpatel2003/ncp_ws`** before the clone can
  complete. Also confirmed live: `bernard-2`'s IP has moved again (now
  reachable only by `bernard-2.local` mDNS — the workstation's
  `~/.ssh/config` was repointed from a hardcoded IP to the mDNS hostname to
  stop this from recurring). The standing ttyUSB1 autostart trap
  (`rosmaster_main.py`) was confirmed still live and holding the base-board
  serial port; not killed, no nodes touched, robot left clean. `colcon` is
  present on the robot and the workspace layout matches
  `yahboomcar_ros2_ws/yahboomcar_ws` — build was not attempted, blocked on
  the clone.


- 2026-08-27: **Second robot `bernard-1` brought online; lidar verified.** New
  network: `RobotLab` router, robot on wifi at `192.168.4.100`, workstation
  cabled to the same router at `192.168.4.150`. Set up SSH (`~/.ssh/bernard-1`,
  config entry, key installed by operator). **Auto-connect-on-boot fixed:** the
  cause was not DHCP — `RobotLab` sat at `autoconnect-priority 0` while
  `Aikoada1`, `Aikoada1_2GEXT` and `robotics` were all at 10, so any of those in
  range won the boot race. Raised RobotLab to 100 and pinned
  `ipv4.method manual / 192.168.4.100/24` (no router access, so a DHCP
  reservation was impossible). **Verified through a real unplanned reboot** —
  came back on RobotLab at the static address unattended. Workstation
  `enp0s31f6` set `ipv4.never-default`: athean-wifi (`192.168.4.129/22`) and
  RobotLab (`192.168.4.150/24`) are *different physical routers sharing gateway
  192.168.4.1*, so both default routes had to be disambiguated; robot traffic
  reaches eth by longest-prefix match. Operator declined renumbering.
  **LIDAR WORKS on bernard-1** — YDLIDAR TG30 serial 2025092500100082
  (bernard-2 is ...0084, consecutive units), `/scan` 10.07 Hz, std dev 1.1 ms,
  1982 pts, +-pi, `laser_link`, 0.062-14.23 m. Two *sequential* faults had to be
  cleared and the second masked the first: (1) the TG30's micro-USB motor-power
  lead was not in the powered hub — the CP2102 adapter is powered by the data
  cable, so the port enumerates and the driver logs "LiDAR successfully
  connected" against a dead sensor; (2) **NEW STANDING TRAP — the
  `/dev/rplidar` symlink races between devices across reboots.** Yahboom's
  `rplidar.rules` maps CP2102->rplidar while cradle's `99-cpt-lidar.rules` maps
  CH340->rplidar; both match, and the winner flips per boot. Observed live:
  after a reboot `/dev/rplidar` pointed at the **base board**, so the lidar
  driver was talking to the motor controller. Ground truth is chip-based —
  CP2102 `10c4:ea60` = lidar, CH340 `1a86:7523` = base board — so address the
  lidar by `/dev/serial/by-id/usb-Silicon_Labs_CP2102_...`. `/dev/yahboom_motor`
  currently points at the *lidar*, which would misdirect cradle on this robot.
  **CORRECTION to the 2026-08-12 s3 contract note: `invalid_range_is_inf:=true`
  is INERT on this driver.** Launched with the flag explicitly true and
  measured 1982 points / 781 zeros (39.4%) / **0 inf** / 0 nan. The
  "belt-and-braces" recommendation is retracted in `config/interface.yaml`; the
  observation builder's `0.0 -> range_max` mapping is the ONLY defence against
  a no-return latching the e-stop. Also found: bernard-1 has **no internet**
  (RobotLab has no WAN uplink) and its **clock is ~3 months behind** with RTC
  at 1970 and NTP unreachable — cross-machine staleness checks would fail
  closed permanently and bags would carry wrong stamps. Both robots are
  `ROS_DOMAIN_ID=28`, so running them on one network shares `/cmd_vel`.
  `robot-ssh` skill rewritten to cover both robots and all of the above. Robot
  left clean (lidar driver stopped, ports free); `rosmaster_main.py` autostart
  trap confirmed live here too and left untouched. Session ended with the robot
  taken offline to charge.

- 2026-08-27 (session 2): **FULL NCP PIPELINE RUNNING ON bernard-1 HARDWARE.**
  Battery 11.8 V (freshly charged). Clock pushed from the workstation (was
  2026-05-24, ~3 months behind). **THE torch BLOCKER IS CLEARED:** the robot
  already has torch 2.5.0a0+nv24.08 (JetPack build, CUDA available), ncps
  0.0.2, numpy 1.23.5 — so no internet is needed to run the NCP.
  `/dev/rplidar` pointed at the **base board** again this boot (the documented
  symlink race); the by-id CP2102 path was used throughout. `rosmaster_main.py`
  (PID 3485) held ttyUSB1 and was killed by exact PID before bringup.

  **NEW — the 32-beam blocker and its fix.** `NCP-on-Rosmaster/rosmaster/node.py`
  hard-refuses any scan whose length != `policy.BEAMS` (32); the TG30 delivers
  ~2020. Pointed at the raw `/scan` that node NEVER builds an observation, its
  1 s deadman never clears, and it publishes zero forever — in BOTH `mode:=debug`
  and `mode:=ncp`. Fixed WITHOUT touching upstream (the vendored copy stays
  byte-exact) by `rm_policy/scan_bridge_node.py`: `/scan` -> `/ncp/scan`, 32
  beams over the forward 180 deg via the existing `scan_resampler`, publishing
  bin CENTRES so upstream's own `check_scan()` passes unmodified (span 174.375
  deg, centred 0 rad — both inside its 10 deg tolerance). Measured live:
  `/ncp/scan` 10.141 Hz, 32 ranges, **0 zeros** (the TG30 no-return sentinel
  fully absorbed), min 0.406 m. Plus `rm_bringup/launch/ncp.launch.py`
  (safety + bridge + one consumer).

  **NEW STANDING TRAP — QoS.** The bridge first published BEST_EFFORT (copied
  from rm_safety's sensor QoS). Upstream's `create_subscription(..., 10)` is a
  depth-only profile and therefore **RELIABLE**, and a RELIABLE subscriber
  REFUSES a BEST_EFFORT publisher: rclpy logs "offering incompatible QoS … No
  messages will be received" once, then upstream's deadman reports "no fresh
  scan — stopped" forever *while `/ncp/scan` is visibly publishing at 10 Hz*.
  That looks exactly like a dead lidar. The bridge now publishes RELIABLE
  (serves both upstream and policy_node's BEST_EFFORT sub) and subscribes
  BEST_EFFORT. **Do not "tidy" the two sides to match.**

  **Validated live, all three consumers:**
  * upstream `node.py mode:=debug` — `SEES OBSTACLE -> SPIN | min=0.4m |
    cmd=(+0.00,+1.20) | ncp=(+0.03,-0.09)` at 5 Hz, NCP running in shadow.
    Spin direction verified correct (left-half mean 5.5 m vs right 3.0 m ->
    +1.2 rad/s toward the open side).
  * `rm_policy/policy_node` — interface_version 3.0.0 check passed, goal fixed
    from first `/odom` at (2.500, -0.007), `/policy/cmd_vel` at **4.999 Hz**
    (std dev 1.7 ms), `/ncp_debug` `fault: null`.
  * **`rm_safety` FIRST EVER HARDWARE RUN — passes.** Starts KILLED, publishes
    continuous zeros at exactly **20.000 Hz** (std dev 0.19 ms), kill/reset
    services work (state 3<->2), receives `/scan` and computes a real
    `min_range` 0.374 m, fails closed to `command_timeout` with no command
    source. End-to-end with `command_source:=policy`: IN `/policy/cmd_vel`
    (0.008186, -0.10555) -> OUT (0.000309, -0.001843), `reason: slow_zone`,
    `clamping_active: true`, min_range 0.367 m. The 0.0377 linear.x ratio is
    exactly the slow-zone scale for that reading.

  **NEW BLOCKER — two publishers on `/cmd_vel`.** Yahboom's own bringup runs
  `joy_ctrl`, which **already publishes `/cmd_vel`**. rm_safety therefore
  cannot be the sole publisher while standard bringup is up — the same
  violation the plan forbids for `cpt_b2_engine`, but from a node we launch
  every session. All of this session's rm_safety testing was done with
  `-r /cmd_vel:=/cmd_vel_test` to avoid it, so **NOTHING MOVED**. Needs an
  operator decision before any motion test (recommendation: remap joy_ctrl to
  `/teleop/cmd_vel`, which is where the plan already wants it — it is an INPUT
  to rm_safety, and that also makes the gamepad a real manual override).

  **NEW — `command_source` is read once at init** (`safety_node.py:153`) with
  no `add_on_set_parameters_callback`, so `ros2 param set /safety_node
  command_source policy` prints "Set parameter successful" and changes
  NOTHING. Fails safe, but silently. Recommendation: declare it read-only so
  the attempt is rejected rather than accepted-and-ignored.

  **NEW — kill-state persistence was broken on the robot:**
  `/var/lib/rm_safety/kill_state` was `[Errno 13] Permission denied`, so the
  node warned and degraded to memory-only every kill/reset. Fixed on bernard-1
  (`install -d -o jetson -g jetson /var/lib/rm_safety`); 0 errors after. This
  is a **deploy step that must be scripted** — it will recur on bernard-2 and
  on any reflash.

  Robot tier runs **183 tests** (workstation 145: no numpy/torch/ncps there).
  Left clean: all rm_* nodes stopped; Yahboom bringup + lidar deliberately
  LEFT RUNNING (sensing only, no motion) for the operator's motion tests.
  Motion tests NOT attempted — operator + kill switch only, per BUILD-PLAN.

- 2026-08-28: **`joy_ctrl` /cmd_vel conflict RESOLVED; `ncp.launch.py` proven
  end to end.** (1) The vendor bringup hardcodes `yahboom_joy_X3` (node
  `joy_ctrl`) publishing `/cmd_vel`, with no launch arg to disable it. Measured:
  idle it ADVERTISES but publishes nothing, so the clash was latent, not active.
  Resolved by remap, not by deleting teleop —
  `rm_bringup/launch/teleop_joy.launch.py` runs it with
  `/cmd_vel -> /teleop/cmd_vel`, which is where the plan already wanted the
  gamepad: an INPUT to rm_safety, so manual driving now sits BEHIND the same
  clamps and e-stop arcs as the policy. Verified live: `/cmd_vel` 0 publishers,
  `/teleop/cmd_vel` 1 (`joy_ctrl`), `/policy/cmd_vel` 1 (`ncp_rosmaster`).
  **The vendor launch still starts its own copy every bringup — stop that PID
  first** (`pgrep -af '[y]ahboom_joy_X3'`, then `kill <PID>`); the sequence is
  documented in the launch file's docstring.
  (2) **`ncp.launch.py` had never actually been run** — the 2026-08-27 session
  exercised the nodes individually. Running it found two real bugs. **rcl
  cannot parse a parameter override with no value:** an empty `weights` (the
  DEFAULT, meaning random weights) emitted `-p weights:=` and killed upstream
  node.py at `rclpy.init()` with "Couldn't parse parameter override rule". The
  upstream process is now built by an `OpaqueFunction` that omits the argument
  when empty. **And there was no `command_source` passthrough**, so
  `safety.launch.py` always came up `teleop` and rm_safety ignored
  `/policy/cmd_vel` — the NCP could never drive however it was launched. Added
  as an explicit argument, still defaulting to `teleop` so bringing the stack
  up to watch it cannot reach the base driver. After both fixes one command
  brings up all three processes and debug mode runs:
  `SEES OBSTACLE -> SPIN | min=0.89m | cmd=(+0.00,+1.20) | ncp=(+0.04,-0.08)`.
  Left clean: all NCP-stack nodes stopped, `/cmd_vel` 0 publishers; Yahboom
  bringup, remapped `joy_ctrl` and the lidar left RUNNING for the operator.

- 2026-08-28 (floor test): **STOP-THE-LINE — the velocity clamps do not hold in
  physical units, and `/odom` is unusable.** Two bags analysed
  (`/tmp/drift_test`, `/tmp/spin_test` on bernard-1).

  **(a) RETRACTED — "the robot moves faster than commanded" was NOT established.**
  It was inferred from `/odom` reporting ~6 m of travel in the floor bag against
  a commanded ceiling of ~1.7 m. That ruler is the very sensor (b) shows to be
  broken: the second bag was recorded **WHEELS UP**, and `/odom_raw` still
  accumulated **21.0 m** (and `/odom` 24.3 m) over 25 s in the air. The 6 m
  figure is therefore worthless as evidence about achieved speed. The operator's
  "6 m is right" was agreement with that number, not an independent
  tape-measured distance.
  What REMAINS true: the achieved-vs-commanded velocity ratio is **UNMEASURED**,
  so the clamps are **unverified, not proven broken**. The operator did observe
  real drift on the floor; that eyeball observation stands on its own.
  **Measure it before trusting `max_vx` in physical units** (see the wheels-up
  revolution count below). With no physical E-stop, an unverified clamp still
  deserves caution.

  **(b) `/vel_raw` and `/odom_raw` are not measurements. THIS IS THE FIRM
  FINDING.** Across a 25 s **wheels-up** run (chassis held off the ground, so
  the true body velocity was ZERO throughout) they sat at **≈ −1.10 m/s and +0.28 rad/s CONSTANT**, uncorrelated with the
  command in magnitude, sign, or axis: commanded `+0.150` forward -> −1.10;
  commanded `0.000` with a pure `wz = −0.400` spin -> still −1.10. They snap to
  exactly 0.000 only after the command has been zero ~0.5 s. Values jitter in
  ~0.004 (vx) and ~0.024 (wz) steps, which looks like a fixed-point decode
  artefact rather than a signal.
  NOT the known base-board serial trap: `/voltage` read a healthy 10.7 V
  throughout, `rosmaster_main.py` was killed before bringup, and every scale
  parameter is at its default (`base_node.linear_scale_x/y`,
  `angular_scale` all 1.0; `driver_node.car_type` correctly `X3`). The fault is
  inside the Yahboom driver/firmware velocity decode, upstream of `base_node`.

  **(c) The EKF inherits it rather than rejecting it.** `ekf_x1_x3.yaml` fuses
  `odom0: /odom_raw` for x/y/yaw/vx/vy/wz and `imu0: /imu/data` for **yaw and
  yaw-rate only** — no linear acceleration. So `/odom` is a low-pass of
  `/odom_raw`: its vx visibly converges toward −1.1 over several seconds and
  decays slowly after motion stops. **`/odom` is not trustworthy on this robot.**

  **Consequences that must not be forgotten:**
  * `policy_node` derives pose and the goal from `/odom`, so **2 of the NCP's
    36 observation dims are currently garbage**, as are the logged
    `goal_dist`/`reward`/`episode_return` in debug mode.
  * A heading-hold controller for the drift symptom **must not be built on
    `/odom`** — it would chase a lying sensor. Resolve (a)/(b) first.
  * The earlier note that the `imu_link` 180/0/−90 rotation was "sidestepped"
    by dropping the IMU from the observation is **incomplete**: the EKF still
    fuses `/imu/data` for yaw, so that rotation still reaches `/odom`.

  **Next diagnostic (zero risk, no floor travel):** wheels UP, command a steady
  `vx = 0.15`, and COUNT WHEEL REVOLUTIONS on slow-mo video over ~10 s. Wheel
  surface speed is exactly what the command maps to, so this is ground truth
  independent of the lying odometry: X3 mecanum ~97 mm dia -> ~0.305 m
  circumference, so 0.15 m/s = ~0.49 rev/s (~5 revs in 10 s). ~3.6 rev/s (~36
  revs) would mean ~1.1 m/s and a ~7x clamp error.
  Also observe, in the same run, whether the wheels visibly change pattern
  between the forward phase and the spin phase (forward = all four same
  direction; spin = left pair opposing right pair). Identical patterns would
  mean the command is not reaching the wheels — a different fault from a
  decode bug.

  Confirmed working in the same bags: the 2026-08-28 yaw fix — `wz` reached the
  full ±0.400 clamp while `vx` was scaled to 0.000 in the slow zone, exactly the
  intended asymmetry.

- 2026-08-28 (session 2): **ROOT-CAUSED the velocity/clamp failure — it is the
  STM32 firmware's closed-loop speed control, not anything in ROS.** Bench
  session on bernard-1, wheels up, operator + kill switch, three diagnostic
  scripts now committed at `scripts/diagnostics/` (`wheel_diag.py` closed-loop,
  `motor_openloop.py` raw PWM, `per_motor.py` one motor at a time).
  * **`/cmd_vel` magnitude is ignored.** Commanding 0.02 / 0.05 / 0.15 m/s each
    produced the SAME ~30 wheel-rev / 10 s (~0.8-1 m/s) on a tape-marked wheel.
    Not proportional, not a gain error — the number is thrown away.
  * **Why:** the board runs a per-wheel incremental speed PID (firmware v3.6,
    gains stock `[0.8, 0.06, 0.5]` — NOT corrupted, a factory reset won't help).
    `err = target - measured`. On this robot every working wheel reports its
    encoder with the **wrong sign** relative to what the PID expects: drive the
    wheel forward and `get_motor_encoder` counts DOWN. So `err` never shrinks,
    `pwm_output` integrates straight to the ±3600 rail, and the wheel runs at
    ~100% PWM for any nonzero command. This single fact explains the ignored
    magnitude, the constant `-1.1 m/s` `/vel_raw`, and the phantom yaw.
  * **Motor→wheel map, MEASURED (per-motor open-loop `set_motor`):**
    `M1=front-left, M2=back-left, M3=front-right, M4=back-right`. This is NOT
    the Yahboom sample order (`M1,M2,M3,M4 = FL,FR,BL,BR`). `get_motor_encoder`
    returns counts in M1..M4 order. For every wheel `set_motor(+)` drives it
    FORWARD.
  * **Front-right motor (M3) is FAILING.** Dead at PWM ≤ 25; moves but slow at
    PWM 50; its encoder only reads once it is turning fast. The fault FOLLOWS
    THE MOTOR through an M1↔M3 plug swap → it is the motor, not the board
    channel or wiring. Operator cannot replace it now.
  * Open-loop `set_motor` (raw PWM, no PID, no feedback) is CLEAN on the three
    good wheels — proportional-ish, correct direction, no runaway. Rough scale
    (wheels up): `set_motor(6)` ≈ 0.05 m/s, `set_motor(12)` ≈ 0.10 m/s.
  * Not firmware-fixable in place: bernard-1 has no internet for the correct
    image, and the encoder polarity is not a swappable wire (keyed / integral
    to the motor). Reflash deferred; needs the image on a USB stick + BOOT0
    jumper or ST-Link, and it is a bricking risk.

- 2026-08-28 (session 2, cont.): **Built `rm_base` — open-loop mecanum base
  driver — to work around the firmware. Offline, not colcon-built, not
  hardware-tested.** New package `src/rm_base/` (`rm_base_node`). Owns the
  base-board serial via `Rosmaster_Lib` and REPLACES the vendor
  `Mcnamu_driver_X3` (single-writer port — the vendor driver must be killed
  first; bringup does not respawn it). Does the mecanum inverse kinematics in
  ROS with the measured map + sign, drives raw `set_motor` PWM (bypasses the
  firmware speed PID AND the dead M3 encoder), and republishes
  `/imu/data_raw` `/imu/mag` `/voltage` `/edition` byte-identically to the
  vendor driver (so `imu_filter_madgwick` + the EKF are unaffected) plus a
  sign-corrected `/vel_raw` reconstructed from the encoders (negated per the
  firmware bug). Fail-closed: `dry_run` defaults **true** (motion needs
  `dry_run:=false` AND an armed rm_safety — two explicit acts, no physical
  E-stop); a `/cmd_vel` watchdog zeros PWM after `command_timeout_ms`
  (**the vendor driver had NO watchdog — this closes the "robot doesn't stop
  on its own" hazard in the robot-ssh skill**); zero-on-exception; flood-stop
  on SIGINT/shutdown before releasing the port. Pure modules
  `rm_base/mecanum.py` + `rm_base/pwm_map.py` with 15 unit tests (pass, part
  of the workstation tier — 155 pass / 4 skip total). `config/base_calibration.yaml`
  holds the per-wheel PWM dead-band model — **STARTING GUESSES from the
  wheels-up probes, must be re-tuned wheels-DOWN against a tape measure**;
  front-right carries a big dead_pwm + gain + raised ceiling to nurse the weak
  motor. `src/rm_bringup/launch/base.launch.py`. Still `/cmd_vel` is published
  ONLY by rm_safety; rm_base is a pure consumer of it.
  Robot: bernard-1 rebooted mid-session (dropped off the network ~20 min),
  came back on the static IP unattended; clock reset to May again, RTC still
  1970; `rosmaster_main.py` autostart re-killed by PID. Left with `driver_node`
  manually restarted and `safety_node` KILLED holding `/cmd_vel` at zero — the
  full stack needs a clean re-bringup next session.

- 2026-08-28 (session 3): **`rm_base` gained a CLOSED-LOOP mode, and
  `/base/debug` became readable.** Offline, not colcon-built, not
  hardware-tested. Motivated by two operator observations: the wheels run at
  different speeds on the ground (the expected open-loop failure — each wheel
  gets the PWM its calibration row says, not the speed the kinematics asked
  for), and `/base/debug` showed nothing useful.
  * **Why closed loop is possible at all:** the firmware PID's *loop* was
    never the problem, its *feedback polarity* was. rm_base already negates
    the encoder delta (forward = count DOWN). Feeding that sign-corrected
    speed to a loop in ROS is the fix the firmware could not apply.
  * New pure module `rm_base/wheel_control.py`: per-wheel feedforward + PI.
    Feedforward is the existing `base_calibration.yaml` model, so the
    integrator only trims the residual and `i_max_pwm` bounds the total extra
    authority a bad measurement can command. Three guards, all failing toward
    LESS authority: **anti-windup** (never integrate deeper into saturation,
    hard clamp), **stall detect** (a wheel commanded but reporting no motion
    freezes its integrator — without this, front-right's read-only-at-speed
    encoder would rail the output on every start, reproducing the firmware bug
    in ROS), and a **latched sign guard** (a wheel measurably turning against
    the command zeroes that wheel and logs ERROR — the tripwire for the
    polarity assumption breaking). Also `EncoderTracker`: `get_motor_encoder`
    reads a value cached by the auto-report thread, so polling it at 20 Hz
    yields repeated samples; an unchanged count is reported NOT-fresh and dt
    accumulates, rather than being divided into a fabricated zero speed.
    Invalid feedback degrades a wheel to feedforward, i.e. to the validated
    open-loop behaviour. **22 new unit tests** (workstation tier now 186).
  * `control_mode` parameter, `"open_loop"` (DEFAULT, unchanged behaviour) |
    `"closed_loop"`. Also a `base.launch.py` arg. Closed loop must be proven
    wheels-up before the floor.
  * Encoders now read in the CONTROL tick, not the sensor tick — one reader,
    one clock. `/vel_raw` republishes whatever the control tick last measured.
    Feedback is read in open_loop mode too, so `/base/debug` shows the four
    diverging wheel speeds even when nothing is correcting them.
  * **`/base/debug` is now `rm_interfaces/msg/BaseDebug`**, not a Twist with
    four PWM values packed into `linear.x/y/z` + `angular.x`. Per-wheel
    target/measured speed, feedforward, integral, target and applied PWM, and
    the feedback_valid / saturated / stalled / sign_fault flags, indexed
    `[FL, BL, FR, BR]`. **The old message reported only what was written to
    the board, so in `dry_run` (the default) it was four zeros** — the likely
    reason it "showed no PWM values". Everything except `applied_pwm` is now
    populated in dry_run, so a wheels-up dry run previews the real command.
    Requires a `rm_interfaces` rebuild on the robot.

- 2026-08-29: **THE ROBOT DRIVES ACCURATELY. Straight line and turn both
  calibrated wheels-DOWN against a tape measure** (operator present, floor
  test, `control_mode:=closed_loop`). Final verified results: commanded 1.00 m
  -> measured **1.00 m**, visually straight; commanded 180 deg -> achieved
  **172 deg** with 4 mm of drift off the spot. This closes BUILD-PLAN items
  2b, 3 and (partly) 4. **Purpose of all of it: the NCP debug test — a rough
  end-to-end pipeline run of the complete stack to verify everything works as
  expected.** The base was the last blocker; motion is no longer gated.

  **NEW — `/odom` had NO publisher at all, and it would have blocked every
  command.** `bench_up.sh` killed the vendor `Mcnamu_driver_X3` (correct —
  rm_base replaces it) but nothing ever started the REST of the vendor
  odometry chain, which lives in the same bundled launch file. The chain is
  `driver -> /vel_raw -> base_node_X3 -> /odom_raw -> EKF -> /odom`; rm_base
  replaces only the first stage and already republishes `/vel_raw` +
  `/imu/data_raw` byte-identically, so the last three nodes drop straight in.
  Fixed with **`src/rm_bringup/launch/odom.launch.py`** — runs
  `base_node_X3` + `imu_filter_madgwick` + EKF + robot_state_publisher,
  deliberately WITHOUT `Mcnamu_driver_X3` (single-writer serial port) and
  WITHOUT `yahboom_joy_X3` (that is `teleop_joy.launch.py`'s job). `/odom` now
  publishes at a steady **30 Hz**. This is the item-5 wiring.
  Note `odom_stale` sits at **precedence 3** in `evaluate()`, ABOVE the zone
  checks, so without this every command is zeroed with `reason: odom_stale`
  regardless of what is sent. `_odom_cb` only stamps the staleness watchdog and
  never feeds velocity into `EvalInputs`, so odom ACCURACY cannot change a
  safety decision — but odom PRESENCE gates everything.

  **THE BIG ONE — `/vel_raw` and `/odom` were lying by ~1/3, and the closed
  loop tracked the lie perfectly.** Commanded 1.00 m, robot moved **0.75 m**,
  odom claimed **1.11 m**, while all four wheels held 0.100 m/s to within 3%.
  A converging controller against wrong feedback looks EXACTLY like success in
  every ROS-side number. **Only the operator's tape measure caught it —
  odometry cannot check itself.** Two independent constants were both wrong and
  the errors partly cancelled:
  * `wheel_circumference_m` 0.204 -> **0.248**. The wheel measures **79 mm**
    across with a ruler (NOT the ~97 mm the earlier CLAUDE.md note assumed),
    so one turn = 0.248 m. The 0.204 was an unmeasured guess and the file
    itself said "measure the real wheel and correct".
  * `counts_per_rev` 1320 -> **2360** (fitted in two rounds, 1320 -> 2140 ->
    2360, each against a fresh tape reading). Near 48*11*4 = 2112, i.e. a 48:1
    gearbox rather than the assumed 30:1 — plausible, unconfirmed.
  CAVEAT: fitted from floor distance, so any wheel SLIP is baked in as if it
  were encoder scale. A wheels-up revolution count would separate the two.
  Treat as "matches the tape", not "physically correct".

  **NEW STANDING TRAP — a one-speed fit cannot separate `dead_pwm` from
  `gain_pwm_per_mps`, and getting it wrong breaks turning silently.**
  `dead_pwm` is the push needed to START a stopped wheel; gain is the extra
  push per unit speed once rolling. An intermediate fit folded the whole
  measured correction into `dead_pwm` — it drove straight perfectly at
  0.10 m/s and then a 180 deg turn produced only **79 deg**, because BOTH
  right wheels never started at the ~0.052 m/s a turn asks of them. A wheel
  that fails to start **stays** stuck: the stall guard (correctly) freezes the
  integrator so the controller never pushes harder. Refitted from TWO speeds
  (0.100 m/s straight + 0.052 m/s turn = two points, two unknowns).
  **Always err HIGH on `dead_pwm`** — too much starting push overshoots and is
  trimmed by the loop, too little means the wheel never moves at all.

  **CORRECTION to 2026-08-28 s2: front-right (M3) is NOT a failing motor.**
  The "weak, needs ~5x PWM, dead below PWM 25" diagnosis was measured
  WHEELS-UP. On the floor FR was the FASTEST wheel and over-driving it is what
  curved the robot left. It does still need the highest starting push of the
  four (dead_pwm 34) and its encoder reads unreliably at low speed. Motor
  replacement is NOT needed. Final per-wheel model (both `base_calibration.yaml`
  and `..._floor.yaml`, now identical): FL 12.0/62, BL 20.0/42, FR 34.0/60,
  BR 13.0/60; `max_pwm` kept at the reduced floor-test ceilings (35, FR 80) —
  the model asks ~26 at the 0.15 m/s clamp, so 35 is ample and a low ceiling
  bounds what a bad reading can command.

  **Back-left is directionally asymmetric:** turns forward at PWM 7, DEAD in
  reverse at PWM 11, breaks loose in reverse ~PWM 16. `pwm_map`'s model is
  symmetric (`|pwm| = dead_pwm + gain*|v|`) and cannot hold two dead-bands, so
  `dead_pwm` is set for the REVERSE threshold (the harder one) and the loop
  trims the forward overshoot. Reverse is exactly what turning on the spot
  needs from the left side. Per-direction `dead_pwm` is a small `pwm_map.py`
  change if this ever needs to be tighter.

  **`lever_arm_m` 0.165 -> 0.213 is a SCRUB FUDGE, not a measurement.** With
  all four wheels running at exactly the speed the geometry asks for, a
  commanded 180 deg produced 139 deg — mecanum rollers scrub sideways when
  turning on the spot. 0.165 * (180/139.3) = 0.213 makes the commanded angle
  come out right (172 deg achieved). Do NOT "correct" it back to the physical
  164.555 mm half-wheelbase. Affects turning only; straight-line driving has no
  yaw component.

  **Closed loop validated wheels-UP first (item 2b): no `sign_fault` on any
  wheel in any direction** — the encoder polarity assumption holds. Mecanum
  kinematics verified: `vx=0.05, wz=0.3` gave left 0.002 / right 0.097, exactly
  `vx -/+ wz*L`, right-faster = correct CCW. Encoder feedback fresh on ~95% of
  ticks (it is a cached auto-report value, so gaps are expected and a
  not-fresh read degrades that wheel to feedforward rather than fabricating a
  zero).

  **Tooling, all new/changed this session:**
  * `scripts/bench_up.sh` REWRITTEN as the single parameterised bringup
    (BUILD-PLAN item 7): `--mode open_loop|closed_loop`, `--cal PATH|floor`,
    `--live` (dry_run false), `--source policy|teleop`. Kills the vendor
    driver AND the `rosmaster_main.py` autostart, ensures
    `/var/lib/rm_safety`, starts lidar (by-id CP2102) + rm_base + odom chain +
    rm_safety. All defaults are the safe end.
  * **BUG FIXED IN IT THE SAME SESSION:** its stale-process cleanup did not
    cover the odom chain, so re-running it stacked a SECOND EKF and odom
    integrator on the first — `/odom`'s measured rate jumped 30 -> 56 -> 49 Hz
    and two nodes fought over the `odom->base_footprint` TF. Cleanup now
    covers `base_node_X3`, `imu_filter_madgwick_node`, `ekf_node`,
    `robot_state_publisher`, `joint_state_publisher`.
  * `scripts/diagnostics/closed_loop_check.py` NEW — the acceptance test used
    for everything above. Arms rm_safety, commands one velocity for a fixed
    time while recording `/base/debug` + `/odom` + safety reasons, then stops
    and re-kills. Publisher, zero-flood and `/safety/kill` all live in one
    `try/finally`, so there is no exit path — normal, Ctrl-C or exception —
    that does not stop the robot. Reports the STEADY-STATE TAIL (last 40%),
    not the whole-run mean, because the integral starts at zero and a
    whole-run mean under-states the converged value and under-corrects.
  * `scripts/drive_test.sh` NEW — timed operator-run velocity command, floods
    zeros on every exit path.
  * `scripts/bench_down.sh` now also stops the odom chain.
  * **REPORTING BUG FIXED:** displacement was being split in the ODOM frame,
    whose orientation is frozen when the EKF starts. After any manual
    reposition of the robot that no longer matches its heading, and pure
    forward motion then reads as a metre of phantom "lateral drift". Now
    rotated into the body frame at run start. **Yaw change was never affected
    (frame-independent) and is the trustworthy veer signal.**

  Method note worth keeping: `/scan` gives an objective runway check before
  every floor run (forward-arc min range) — the 0.80 m `slow_zone_m` silently
  scales `vx` down and would corrupt a calibration measurement. Every accepted
  run above had `safety reasons: ok x100`, i.e. nothing was clamped. The close
  returns at +-90..150 deg are the CHASSIS (symmetric, rotate with the robot)
  and never enter the forward arc.

  Robot left: bringup + lidar + odom chain running, rm_safety KILLED holding
  `/cmd_vel` at zero, battery ~11.7 V.

- 2026-08-29 (session 2): **THE NCP DEBUG TEST RAN END TO END ON HARDWARE, and
  the base now drives accurately on all four wheels.** One command brings up 11
  nodes with exactly one `/cmd_vel` publisher; measured live: `/scan` 10.13,
  `/ncp/scan` 10.14, `/odom` 30.0, `/policy/cmd_vel` 5.0, `/cmd_vel` 20.0,
  `/base/debug` 20.0 Hz. Final cruise, all four wheels, target 0.250 m/s:
  FL 0.256 / BL 0.257 / FR 0.257 / BR 0.257, **no flags, integrals under 2.3
  PWM** — i.e. the feedforward is genuinely right rather than being propped up
  by the loop. Getting there took five real bugs, below.

  **`ncp.launch.py` now brings up the WHOLE stack** (lidar + rm_base + odom
  chain + safety + bridge + consumer) behind `with_lidar` / `with_base` /
  `with_odom`, plus `control_mode`, `dry_run`, `calibration_path` and
  `obstacle_dist`. `dry_run` defaults FALSE here (unlike `base.launch.py`),
  because bringing up the full stack implies intent to drive; rm_safety still
  starts KILLED and `command_source` still defaults to `teleop`.
  **`scripts/prep_robot.sh`** is the host-level cleanup a launch file cannot
  do (vendor driver, `rosmaster_main.py`, `yahboom_joy_X3`, `/var/lib/rm_safety`),
  shared with `bench_up.sh`. **Do NOT run `bench_up.sh` and `ncp.launch.py`
  together — two safety_nodes means two `/cmd_vel` publishers.** bench_up is
  for base-only work now.

  **BUG 1 — config silently shadowed a module default.** `base_node` splats
  `closed_loop.guards` straight into `wheel_control.Guards`, and both YAML
  files still said `sign_fault_ticks: 3`. Raising the module default to 12
  therefore did NOTHING on the robot for several runs. A test now pins every
  config guard key against the module default. **Any key in that block wins
  over the code — check both when changing a guard.**

  **BUG 2 — the sign guard was judging the SLEW LIMITER, not the wiring.**
  `base_node` slew-limits PWM (8/tick) AFTER the controller returns, so coming
  out of cruise at +33 the applied PWM needs ~8 ticks just to reach zero.
  Throughout that the wheel is still being DRIVEN the old way while the target
  says reverse, so of course it keeps moving the old way. Measured: FR target
  -0.131, feedforward -35.6, **applied PWM +6**, measured +0.032 -> latched.
  All four wheels latched to zero PWM on the first full-stack run and the robot
  could not move. Fixed in three steps, all needed: reset the counter on a
  reversal; suspend the guard for `sign_settle_ticks` (10) after one; and
  **gate it on the ACTUAL applied PWM** — `update()` now takes `applied_pwm`
  and will not judge polarity until the wheel is genuinely being driven the
  commanded way, past its own dead band. A reversal is +1<->-1 only; starting
  from rest still arms the guard immediately.

  **BUG 3 — `max_pwm` was left at the floor-test ceiling when `max_vx` rose.**
  At 0.25 m/s three of four wheels ran SATURATED at 33 against a ceiling of 35
  and could not reach the commanded speed. Ceilings restored to 60 (FR 95),
  sized for feedforward + `i_max`. **`max_vx` and `max_pwm` are coupled.**

  **BUG 4 — THE CALIBRATION METHOD ITSELF WAS WRONG.** The 2026-08-29 s1 fit
  used two speeds — but one was a STRAIGHT run and the other a SPIN. A spin
  carries **rotational scrub load** (mecanum rollers must slide sideways), so
  the extra PWM that load demands was misattributed to the dead-band: dead_pwm
  came out far too high and gain far too low. The model then matched only at
  the single speed it was fitted at — straight-line distance at 0.10 m/s was
  correct while cruise at 0.25 m/s undershot to ~87%. **NEVER mix a turning
  measurement into a dead_pwm/gain fit.** Refitted from TWO STRAIGHT LEGS
  (0.06 and 0.12 m/s) via `scripts/diagnostics/fit_reverse.py`, solving
  against MEASURED speed:
  `FL 12.0/62 -> 1.3/162.9`, `BL 20.0/42 -> 3.1/143.6`, `BR 13.0/60 -> 6.5/126.6`.
  Forward and reverse fits agreed within ~10%, so these wheels are effectively
  symmetric and no reverse overrides are set for them.

  **BUG 5 / CORRECTION OF A CORRECTION — front-right IS bad, and a dead wheel
  reads like a working one on the floor.** Earlier the same day this file
  recorded "front-right is NOT a failing motor" because it appeared to track
  its target exactly on the floor (0.250 against 0.254). **It was being DRAGGED
  by the other three** — its encoder reported the robot's motion, not its own
  drive, and the suspiciously PERFECT tracking was the tell. Wheels-up, with
  nothing to drag it, FR read 0.0000 m/s across four 6-second legs at applied
  PWM 42-49, in both directions. **Measure a suspect wheel WHEELS-UP with the
  motor driven directly** (`scripts/diagnostics/fr_threshold.py`), or the other
  wheels will lie to you on your behalf.

  **FRONT-RIGHT IS STICKY, NOT WEAK** (measured, `fr_threshold.py --curve`):
  * breakaway **PWM 60 forward, 55 reverse** — to START it turning
  * running **dead 30.2 / gain 132.1 fwd, 27.7 / 145.4 rev** — to KEEP it going
  Its running gain is NORMAL (132/145 vs the others' 163/144/127), so the motor
  is not weak; it needs roughly twice the PWM to break loose as to sustain, and
  once loose runs freely at ~0.22 m/s. The RUNNING fit is what is configured —
  using breakaway as `dead_pwm` would command 66 PWM for a 0.05 m/s target and
  the wheel would take off at ~0.28 m/s.

  **New: per-direction calibration** (`dead_pwm_rev`, `gain_pwm_per_mps_rev`)
  and a **bounded breakaway kick** (`breakaway_pwm`, `breakaway_pwm_rev`).
  All four keys are optional and default to "same as forward" / "no kick", so
  an unconfigured wheel is untouched — only front-right has them set. The kick
  REPLACES the loop output with a fixed open-loop nudge; it never feeds the
  integrator (pinned by a test), because the stall guard freezing the
  integrator is what stops the firmware runaway and must not be weakened.
  Bounded three ways: 8 ticks per attempt, 20-tick cooldown, 3 attempts then
  it gives up until the command clears — holding a blocked motor at breakaway
  PWM is how it burns out. Measured: exactly 24 kick ticks then `XXX` forever.
  **The kick is DEPLOYED BUT NOT YET EXERCISED ON HARDWARE** — it only fires
  below ~0.14 m/s, and cruise never goes there (at 0.17 m/s the proportional
  term already asks ~63 against the 60 threshold).

  **Tuning this session, all operator-requested:** `obstacle_dist` 1.5 -> 0.9
  (upstream's 1.5 is measured across the whole 180 deg arc INCLUDING the sides,
  so in a normal room something is nearly always inside it — measured 380 spin
  samples to 49 cruise, i.e. it spun continuously and never drove);
  `slow_zone_m` 0.80 -> 0.50 -> 0.65; `max_vx` 0.15 -> **0.25**;
  `max_wz` 0.4 -> **0.8**. Ordering that must be preserved:
  **0.90 policy spins > 0.65 safety scales > 0.35 safety latches.**
  `slow_zone_m` and `max_vx` are COUPLED — at 0.25 m/s stopping needs ~0.117 m
  (0.042 braking + 0.075 travelled during the 3-scan filter) and the band is
  0.30 m. Raising `max_vx` again means redoing that sum.
  **These clamps are AHEAD OF THE PHASE 2 HARDWARE SIGN-OFF** — revert to
  0.15 / 0.4 for anything unattended.

  **Not a bug, verified:** the acceleration limiter holds. The one violation
  measured (`vx 0.25 -> 0.0` in a tick, 5.6 m/s^2) is a DECELERATION TO ZERO,
  which fail-closed paths deliberately bypass the limiter for. Braking hard is
  always safe.

  **Standing behaviour of the debug rule, not a fault:** it picks spin
  direction by comparing the left and right halves of the scan with NO
  hysteresis, so in a near-symmetric spot the choice flips every ~2 s and the
  robot turns one way then equally far the other, netting zero. Measured
  policy `wz` mean -0.004 over 301 samples while individual reads were +-1.2.
  Put the robot somewhere clearly asymmetric to get a committed spin. The
  vendored policy stays byte-exact, so this is not "fixed".

  Robot: rebooted mid-session; clock reset to May (pushed from the workstation),
  `/dev/rplidar` pointed at the BASE BOARD again, `rosmaster_main.py` back and
  holding ttyUSB1, and `/tmp` cleared (the ad-hoc probes had to be re-copied).
  Battery ended at **10.3 V** — bottom third, operator charging.

- 2026-08-29 (session 3): **COMMANDED vs ACHIEVED VELOCITY VERIFIED AGAINST A
  TAPE MEASURE, both axes.** Operator-run floor session on bernard-1, three
  rounds of legs. This is the answer to "do the clamps hold in physical units",
  open since the 2026-08-28 stop-the-line. Final state, `lever_arm_m` 0.227:

  | | legs | achieved/commanded |
  |---|---|---|
  | linear forward (0.06 / 0.12 / 0.25 m/s) | 3 | 1.000 / 0.992 / 1.016 |
  | linear reverse (0.12 m/s) | 1 | 0.959 |
  | angular at wz 0.60 | 1 | 0.972 |

  **`max_vx` NOW HOLDS IN PHYSICAL UNITS.** Linear is accurate across a 4x
  speed range forward; reverse is 4% short on a single sample, consistent with
  only front-right having reverse overrides configured. Angular at the
  operating point is 4.8 deg short of 171.8 commanded, i.e. ~1.7 sigma against
  a protractor read off a floor line (worth ~+-3 deg) — at the noise floor,
  not worth chasing further.

  **NEW TOOL — `scripts/diagnostics/velocity_accuracy.py`.** Drives a matrix of
  straight and spin legs, prompts for the tape/protractor reading after each,
  appends to a CSV, and reports achieved/commanded per leg. Two design points
  that are load-bearing and must not be "simplified":
  * **The reference is the TIME INTEGRAL OF `/cmd_vel`, not `vx * secs`.**
    rm_safety acceleration-limits every command, so a 5 s leg at 0.25 m/s never
    travels 1.25 m even with a perfect base — the ramp alone is ~0.04 m, 4% of
    the run, which would be booked against the calibration. Integrating what
    was actually demanded separates the base from the safety layer.
  * **Several speeds, both directions.** A one-speed check passes for
    infinitely many wrong `(dead_pwm, gain)` splits — the 2026-08-29 s2 BUG 4.

  **`lever_arm_m` 0.213 -> 0.236 -> 0.227**, by measured bracket: 0.213 gave
  ratio 0.904 at wz 0.60, 0.236 overshot to 1.060, so 1.000 interpolates at
  0.227 (confirmed live at 0.972). It is an interpolation between two measured
  runs, not a new fit. The pre-existing 0.213 was fitted before the s2
  calibration refit, which is why it needed revisiting (next-session item 3).

  **NEW — THERE IS A ROTATIONAL DEAD-BAND, and one constant cannot cover the
  wz range.** Raising the lever by 10.8% moved wz 0.60 by 17.2% but wz +-0.30
  by only 4.5% / 1.6%. Fitting all six spin legs gives
  `achieved_yaw_rate ~ 311 * (wheel_speed - 0.025 m/s)` — roughly **the first
  0.107 rad/s of any commanded spin is lost to scrub before the chassis turns
  at all**. `lever_arm_m` is a pure multiplier and a multiplier cannot cancel
  an offset, so it can be exact at ONE spin speed and nowhere else. Fitted at
  wz 0.60 because `max_wz` is 0.8 and the debug rule saturates it. **EXPECT
  wz ~0.30 TO STAY ~20% SHORT — that is inherent, not a regression.** The real
  fix is an affine yaw term (gain + per-direction dead-band, the same shape as
  `dead_pwm`/`gain` one level up); NOT built, because no trained policy
  commands low wz yet.

  **The wheels were exonerated by cross-checking the two axes, and this is the
  method worth keeping.** A spin at wz 0.30 asks each wheel for
  `wz * lever_arm_m` = 0.064 m/s — which is the 0.06 m/s straight leg that
  measured 1.000. So the wheels demonstrably CAN hit those speeds and no
  wheel-level explanation of the yaw shortfall survives. Without that
  cross-check the obvious (wrong) move is to refit `dead_pwm`/`gain`.

  **NEW — CW/CCW asymmetry ~9.7%**, CCW 0.797 vs CW 0.727 at |wz| 0.30, two
  samples each, consistent in sign across two different `lever_arm_m` values.
  Suspected cause is front-right's direction-dependent stiction (breakaway 60
  fwd / 55 rev), which a spin drives one way for CCW and the other for CW.

  **`/odom` over-reads linear distance by ~5%** (1.047 / 1.058 / 1.051 across
  the three forward legs) — unlike the yaw error, a clean scale error. NOT
  corrected: `counts_per_rev` is dual-purpose (it is also the closed loop's
  feedback) and the feedforward was fitted against measured speed, so changing
  it moves both and would put the just-verified straight-line accuracy at risk.
  Logged as a known bias; the wheels-up revolution count still separates slip
  from scale.

  **Four bugs found IN THE TEST TOOL, all fixed the same session.** Worth
  recording because three of them produced confident wrong output:
  1. **The summary told the operator to refit `dead_pwm`/`gain`** when linear
     was accurate and angular varied — see the cross-check above. It now
     considers both axes together.
  2. **The runway check silently degraded to a Y/N prompt on 4 of 12 legs**
     (`runway_m` = nan). It spun a fixed 0.5 s before reading `/scan`, which is
     not enough for DDS discovery on a fresh node, so every single-leg
     invocation skipped the check. It now waits up to 8 s for a real scan. A
     safety check that quietly becomes a prompt you say yes to is worse than
     no check.
  3. **It asked the operator to SIGN the tape reading**, and got a wrong sign
     twice — a reverse leg entered as +0.69 against a -0.72 command reads as
     ratio -0.959, i.e. "the robot drove the wrong way", not a typo. It now
     takes an unsigned magnitude and applies the commanded sign itself.
  4. **The analysis pooled forward and reverse legs**, turning a direction
     difference into a fake "varies with speed" verdict. It now groups by
     direction as well as axis, and says when a group has only one leg instead
     of reporting spread 0.000 as though a range had been tested.
  Each CSV row now also records the `lever_arm_m` it ran with, because the
  workflow is measure -> change a constant -> measure again into the same
  sheet, and the summary was averaging pre- and post-change legs together.

  **Reverse legs have NO e-stop coverage.** rm_safety's arcs are forward and
  lateral only — there is no rear arc — so nothing downstream stops the robot
  on a negative-vx leg. The tool now checks the runway in the direction of
  travel and warns explicitly.

  Method note: `bench_up.sh` is the correct bringup for this test, NOT
  `ncp.launch.py` — the latter always starts a policy consumer, and in debug
  mode that consumer publishes `/policy/cmd_vel` continuously, the same topic
  the test commands on. Two publishers would fight mid-leg.

  **The breakaway kick is still NOT CONFIRMED on hardware.** All spin legs ran
  at wheel speeds (0.064-0.142 m/s) that straddle its ~0.14 m/s trigger and
  front-right did turn, so it very likely fired — but no `rm_base` log was
  captured during the runs, so this is inference, not evidence. Next-session
  item 2 stays open; capture the node log while commanding a slow turn.

  Robot: found powered off at session start, came up mid-session on its static
  IP. Clock back to May 24 (~3 months behind, not pushed this session — the
  test uses a monotonic clock deliberately, but push it before recording bags).
  `rosmaster_main.py` autostart live again and holding ttyUSB1 as always.
  Battery 11.9 V at start, freshly charged. NOTE: early in the session an
  agent-side shell-quoting error executed several repo scripts on the robot
  unintentionally (`bench_up.sh`, `bench_down.sh`, `prep_robot.sh`); verified
  after the fact that nothing moved — safety was KILLED and `rm_base` was in
  `dry_run` throughout — and the stack was stopped cleanly before the operator
  ran the real tests.

- 2026-08-29: **THE TRAINED POLICY RUNS THE PIPELINE ON HARDWARE, and there is
  now a web dashboard.** Weights `actor.pt` (m4 BARN run, sha256 `3c9bf0ee112b…`)
  landed upstream and the full stack ran with the NCP deciding actions:
  `ncp(+1.19,-0.37) -> /policy/cmd_vel -> rm_safety -> /cmd_vel(+0.250,-0.100)`,
  12 nodes, exactly **1 publisher on /cmd_vel**, all rates nominal (`/scan`
  10.6, `/ncp/scan` 11.4, `/odom` 32.9, `/policy/cmd_vel` 5.007, `/cmd_vel`
  20.0, `/base/debug` 20.0). Run was `dry_run:=true` throughout — **NOTHING
  MOVED**; motion is still an operator task.

  **THE BLOCKER WAS A STALE VENDOR PIN, and it failed in the right direction.**
  `vendor/ncp_policy.py` was pinned at `0ad548c`, which predates the weights.
  When the checkpoint landed (`a9af7af`) `NcpActor` CHANGED SHAPE: `head`
  `nn.Linear(motor,2)` -> the SB3 stack `Linear-Tanh-Linear-Tanh-Linear`, plus
  `obs_mean`/`obs_var` buffers carrying the checkpoint's VecNormalize stats
  (applied in `act()`, clipped ±10). `actor.pt` is a state_dict for the NEW
  shape. Because `PolicyRunner` calls `load_state_dict` at its default
  `strict=True`, this was a **startup crash, never a wrong-numbers run** —
  which is exactly why strict must stay. Re-vendored to `69785c8`; hash,
  VENDOR.md and `interface.yaml`'s provenance line updated; upstream's two new
  tests ported plus tests that load the real `actor.pt` and roll it 10
  recurrent ticks. **Robot tier now 277 tests** (workstation 233 — no torch
  there, so the checkpoint tests skip).

  **THE FINDING — the policy locks into a PERIOD-2 LIMIT CYCLE in `wz` when
  the robot is STATIONARY, and the safety layer filters it to ~zero yaw.**
  Measured live: `/policy/cmd_vel` `wz` alternating `-0.359, +1.068, -0.353,
  +1.062, …` every tick at 5 Hz, and `/cmd_vel` `wz` a ±0.15 wobble around
  zero — the robot would drive straight and never turn. `max_wz` is 0.8, so a
  clamp would have given +0.8; this is not a clamp. **Cause proven by A/B
  offline against the live observation:** with `previous_action` fed back (what
  `policy_node` does, and what upstream does) it locks to +1.03/-0.40
  alternating; with that block held at zero it settles to a steady `wz ≈
  -0.15`. It is the previous-action feedback, not our wiring — `policy_node`
  is faithful to upstream here.
  **AND IT LARGELY RESOLVES IN MOTION.** Re-run with the goal updated each tick
  as a moving robot would update it, the policy holds a steady `wz ≈ +0.98` for
  14 consecutive ticks, breaking down only intermittently as the goal passes
  abeam. So the limit cycle is an artifact of a FROZEN observation, which is
  precisely the condition of a dry-run bench test. **Do not judge the yaw
  behaviour from a stationary run.** Caveat on that simulation: it stepped the
  acceleration limiter once per POLICY tick (5 Hz) where rm_safety runs it at
  20 Hz, so the real ramp is ~4x faster than it showed.
  This is live data for the `clamped_action_note` OPEN QUESTION in
  `interface.yaml` (feed back requested vs achieved). Feeding back the ACHIEVED
  action also damps the cycle, but is off-distribution vs training. **Left as
  upstream has it — the decision is the operator's and it is now informed.**

  **CORRECTION to the authority-gap expectation.** `interface.yaml` predicted
  the policy would run "almost entirely saturated". On a REAL TG30 scan it does
  not: forward `v` comes out **0.23–0.42 m/s**, scaling sensibly with goal
  distance (goal 1 m -> 0.23, 8 m -> 0.42), against `max_vx` 0.25 — mildly
  clamped, not 8x saturated. `wz` on a real scan is ~0.1–0.2 vs a 0.8 clamp,
  i.e. not clamped at all. **A METHOD NOTE THAT COST ME AN HOUR:** a synthetic
  "all beams at 5.0 m" scan drives the policy far off-distribution and produces
  alarming full-reverse/full-yaw output. That is the PROBE being wrong, not the
  policy. **Always probe this checkpoint with a real scan** (there are fixtures
  in `tests/fixtures/`; note `tg30_open_scan.yaml` is MULTI-DOCUMENT — use
  `yaml.safe_load_all`).

  **NEW PACKAGE `rm_dashboard`** — live web view of the whole chain at
  `http://<robot>:8080/`, verified from the workstation. Shows the raw scan vs
  the 32 policy beams on one polar plot, the 36-dim observation as a bar strip,
  asked-vs-allowed velocity at every hop, safety state/reason/min_range,
  per-wheel base telemetry with the flags, and **rate + jitter + staleness on
  every topic** — the class of signal that has caused nearly every integration
  failure on this project (the QoS mismatch, the doubled EKF, the dead FR
  wheel) and that is invisible in the values alone.
  * **Stdlib only** (`http.server` + SSE). Not a preference — bernard-1 has no
    WAN uplink, so anything needing `pip install` cannot be deployed at all.
    That rules out flask/rosbridge/websockets.
  * **Served FROM THE ROBOT** so it needs only TCP. Workstation-side would put
    the diagnostic behind DDS discovery, i.e. sharing a failure mode with the
    thing it diagnoses.
  * **Subscribes only — publishes NO topic, ever**, so it can never become a
    second `/cmd_vel` publisher.
  * The observation comes from `/ncp_debug` (policy_node now publishes the
    vector, goal and pose there), NOT rebuilt in the dashboard — a second
    implementation of the frozen contract would render a plausible vector that
    is not the one the network consumed.

  **Controls (operator chose full control), each unable to start motion alone:**
  * **`command_source` is now genuinely runtime-settable, and EVERY CHANGE
    FORCE-KILLS.** This closes next-session item 7, in the better direction:
    it previously accepted `ros2 param set` and silently ignored it. Verified
    live — switching to `policy` while killed kept it KILLED, and after an
    explicit `/safety/reset` the policy's command really did reach `/cmd_vel`
    (it never could before). Arming stays a separate act.
  * **`goal_x`/`goal_y` are runtime-settable on `policy_node`**, re-fixing the
    goal from the CURRENT pose (always, even to an unchanged value — a set is
    an explicit "re-target now") and resetting the CfC recurrent state, since
    the objective changing under a stateful network is exactly the
    discontinuity `reset()` exists for. Verified: set (1.8, 0.4) -> `goal_local`
    read back exactly (1.8, 0.4).
  * Every OTHER parameter on both nodes is now **REJECTED** at runtime rather
    than accepted-and-ignored, and the node's own rejection reason is surfaced
    verbatim to the browser.

  **Checkpoint provenance is now traceable end to end.** `PolicyRunner` hashes
  the weights file; `policy_node` logs the full sha256 at startup and publishes
  the first 12 chars on `/ncp_debug`; `scripts/deploy_upstream.sh` prints the
  same hash after verifying it on both ends. All three agreed live
  (`3c9bf0ee112b…`). A run with no checkpoint now logs a WARNING that it is
  random weights.

  **`scripts/deploy_upstream.sh` is NEW and is the only route the checkpoint
  has onto bernard-1** (no internet, and the robot's `~/NCP-on-Rosmaster` is a
  plain directory, not a git repo — so `deploy_to_robot.sh`'s push-to-deploy
  does not apply to it). `tar | ssh tar -x` plus sha256 verification of
  `policy.py`, `node.py` and `actor.pt` on both ends; it refuses to report
  success on a mismatch, and warns if the vendored copy has drifted from what
  it is shipping.

  **`ncp.launch.py` defaults changed:** `consumer` is now `policy_node` (was
  `upstream`) and `weights` now defaults to the real checkpoint. `weights:=''`
  is the explicit opt-in to random weights — a run that silently exercises an
  untrained network is no longer the default. `with_dashboard` (default true)
  starts the dashboard.

  **Doc fix:** `ncp.launch.py` and `interface.yaml` still quoted `max_vx` 0.15 /
  `max_wz` 0.4; `safety_limits.yaml` has been 0.25 / 0.8 since the 2026-08-29
  tuning. The gap is 8x / 3.9x, not 13x / 7.9x.

  Robot: clock pushed from the workstation (was May 24 again). A leftover
  `rm_base_node` from the previous session was holding ttyUSB1 and
  `prep_robot.sh` correctly refused to declare ready until `bench_down.sh`
  cleared it — note `bench_down.sh` then RESTARTS the vendor `Mcnamu_driver_X3`,
  so `prep_robot.sh` must be re-run after it. Battery 11.3 V. Left clean: all
  nodes stopped, both serial ports free, safety KILLED before shutdown.

- 2026-08-29 (session 5): **SLAM WIRED IN — the goal is now fixed in a
  corrected `map` frame instead of drifting odometry.** Operator decision after
  confirming on driven runs that the goal drifts and that `/odom` is not to be
  trusted. Running on bernard-1: `map -> base_footprint` resolving, `/map` at
  2.0 Hz, `policy_node` `pose_source: map`, `fault: null`, goal fixed at
  (2.500, 0.000) in the map frame, one `/cmd_vel` publisher. Robot tier 331
  tests. NOT yet driven under it.

  **Why:** `goal_local` is 2 of the 36 numbers the network reads and both were
  derived from `/odom` — dead reckoning with a tape-fitted `counts_per_rev`,
  slip baked in, ~5% linear over-read. `slam_toolbox` (async online, already
  installed — no internet needed) publishes `map -> odom` on top of the
  untouched EKF chain. **This does not repair the odometry**, it adds a
  correcting frame above it; `/odom` remains untrustworthy for anything
  reading it directly.

  **`pose_source` ("odom" default | "map")** on `policy_node`, driven by
  `ncp.launch.py`'s `with_slam` (default false). `pose_source:=odom` WITH
  `with_slam:=true` runs SLAM while still driving on odometry — the safe way
  to watch a map build before trusting it. Fail-closed: a missing,
  unconnected, un-extrapolatable or stale transform publishes ZERO with reason
  `pose_tf_unavailable:*` / `pose_stale:*`. It NEVER falls back to `/odom` —
  a silent downgrade to the drifting pose is the exact failure this removes.

  **THE HAZARD, and why a stateful policy makes it mandatory:** a corrected
  pose is a DISCONTINUOUS pose. Every loop closure steps `goal_local`, and the
  CfC carries recurrent state — untreated it keeps steering on a hidden
  history built toward where the goal used to be. `rm_policy/pose_source.py`
  detects a jump by **implied speed** (scales with dt rather than assuming a
  tick rate) and **wraps yaw** (a heading crossing +-pi would otherwise read
  as a 6.28 rad spin and reset the net every time the robot pointed south).
  `policy_node` adopts the corrected pose and resets the recurrent state.

  **TWO HARDWARE BUGS, both caught fail-closed (nothing moved):**
  * **slam_toolbox was rejecting EVERY scan.** The TG30's ray count VARIES
    scan to scan — measured **9 distinct values (1967..1975) across 60
    consecutive scans**. karto latches the first count and refuses anything
    else: `LaserRangeScan contains 1970 range readings, expected 1951`, once
    per scan forever, **no map, no `map -> odom`, no crash and no warning that
    it has stopped working**. It reads as SLAM failing to converge. Fixed with
    the driver's **`fixed_resolution:=true`** -> a constant 2020 rays, verified
    over 60 scans. **This flag IS live on this driver, unlike
    `invalid_range_is_inf` which is inert here** — they look equally plausible
    in the parameter list, so it was MEASURED, not assumed.
  * **`pose_future_stamp` every tick.** `policy_node` mapped the transform's
    ROS stamp onto its monotonic clock via an offset captured at the first
    lookup; if that first lookup returned an already-old transform the offset
    was poisoned permanently and every later pose read as stamped in the
    FUTURE — the stalest possible pose passing as the freshest. The
    future-stamp guard caught its own bug. Now **two clocks, each used
    consistently with itself**: freshness compares ROS stamp vs ROS clock
    (both wall, so the age is real even after a hand clock push), jump dt
    stays monotonic (so a hand-corrected clock cannot read as a teleport).

  **TG30 note for SLAM:** slam_toolbox reads the RAW `/scan` with nothing of
  ours between it and the driver, so `min_laser_range: 0.15` in
  `config/slam_toolbox.yaml` is what discards the 0.0 no-return sentinel
  (~39% of rays). Untreated they are phantom hits at the sensor origin and
  scan matching is worthless. `scan_resampler` protects the POLICY path only —
  it is not in SLAM's path.

## 4. Current Goal

**THE NCP DEBUG TEST HAS RUN END TO END ON HARDWARE (2026-08-29 s2).** One
command brings up the whole stack, the policy drives the wheels through
rm_safety, and the base tracks commanded speed on all four wheels to within 3%.
The pipeline is proven. What is proven is the PLUMBING — there is still no
trained checkpoint.

**COMMANDED vs ACHIEVED VELOCITY IS NOW VERIFIED (2026-08-29 s3)**, tape-measured
on both axes: linear 1.000 / 0.992 / 1.016 forward across 0.06-0.25 m/s, 0.959
reverse, angular 0.972 at wz 0.60 with `lever_arm_m` 0.227. The clamps hold in
physical units — the question open since the 2026-08-28 stop-the-line is closed.

Still outstanding from that test:
* **The breakaway kick is STILL not confirmed on hardware.** The s3 spin legs
  straddled its ~0.14 m/s trigger and front-right turned, so it very likely
  fired — but no `rm_base` log was captured, so that is inference. Command a
  slow turn (`wz` under ~0.65 rad/s) with the node log captured.
* **`wz` ~0.30 under-rotates by ~20% and this is INHERENT** — a rotational
  dead-band against a single multiplier (§3). Do not turn `lever_arm_m` up to
  fix it; that over-rotates wz 0.60, as measured. The fix is an affine yaw
  term, not yet worth building.
* **Reverse is one sample.** 0.959 could be a scale error or a dead-band; a
  second reverse speed separates them.
* Battery was 11.9 V at the start of s3; check before the next run.

**The full NCP pipeline runs on bernard-1 hardware** (2026-08-27 s2): lidar ->
32-beam bridge -> observation -> CfC -> `/policy/cmd_vel` -> rm_safety clamp,
with all three consumers validated. Phases 1-3 are effectively done.

**MOTION IS NO LONGER GATED (2026-08-29).** The base drives accurately:
commanded 1.00 m -> 1.00 m measured, commanded 180 deg -> 172 deg, both
tape-verified wheels-down with the operator present. `rm_base` in
`control_mode:=closed_loop` is the validated path. Cruise now tracks 0.250 m/s
on all four wheels to within 3% with the integrals under 2.3 PWM (2026-08-29
s2, after the calibration refit).

Bring the FULL stack up with `prep_robot.sh` then `ncp.launch.py`; use
`bench_up.sh` only for base-only work, and **never both at once** (two
safety_nodes = two `/cmd_vel` publishers). rm_safety still starts KILLED and
`command_source` still defaults to `teleop`, so motion still takes two
explicit acts.

**The velocity failure is root-caused (2026-08-28 s2): the STM32 firmware's
closed-loop speed control is broken on this robot** — encoder feedback is
sign-inverted vs the PID's expectation, so any nonzero command rails the PWM
to ~100% and the wheel runs flat out regardless of the commanded value. Not
fixable in ROS, not fixable in firmware without a reflash (no image, no
internet on bernard-1). Full detail and the measured motor→wheel map in §3.
**`rm_base` is the working workaround and it is now proven on the floor**, so
the reflash is no longer on the critical path.

**What is still NOT trustworthy, and must not be forgotten during the debug
test:** the `/odom` chain is now live and safety-clean, but its scale was
FITTED to the tape rather than derived, and any wheel slip is baked into
`counts_per_rev` as if it were encoder scale. **Measured 2026-08-29 s3: it
over-reads linear distance by ~5%** (1.047 / 1.058 / 1.051 against the tape at
three speeds) — a clean scale error, deliberately NOT corrected because
`counts_per_rev` is also the closed loop's feedback and moving it would put the
just-verified straight-line accuracy at risk. It is good enough to satisfy
rm_safety's staleness watchdog (which is all safety uses it for) and good
enough for rough distance. It is NOT a basis for a heading-hold controller or
for trusting `policy_node`'s derived pose/goal. A wheels-up revolution count
would separate slip from scale and settle it.

**THE TRAINED CHECKPOINT IS IN AND RUNNING (2026-08-29).** `actor.pt`, the m4
BARN run, sha256 `3c9bf0ee112b…`, deployed to bernard-1 and exercised through
the full stack with `policy_node` as the consumer. On a REAL scan it produces
sensible commands — forward `v` 0.23–0.42 m/s scaling with goal distance,
small `wz` — not the saturated output the authority-gap note predicted.
`ACTION_HIGH` v = 2.0 m/s vs safety `max_vx` 0.25 is still an 8x ceiling; do
not raise the clamp.

**But the policy has NEVER DRIVEN THE WHEELS.** Every run so far has been
`dry_run:=true` or safety-KILLED. The remaining unknown is behavioural, not
structural: see the stationary limit cycle in §3 (2026-08-29), which resolves
when the observation updates but has not been confirmed in real motion. The
checkpoint is also only a 30/100 held-out BARN policy — judge the pipeline,
not the policy.

### Next session, in order

Items 1, 2b, 3, 4 and 7 of the previous list are DONE. What remains:

1. **Bring the stack up** — clock, battery, then:
   ```
   cd ~/ncp_ws
   bash scripts/prep_robot.sh                 # must print "== ready =="
   ros2 launch rm_bringup ncp.launch.py mode:=debug command_source:=policy
   ros2 service call /safety/reset std_srvs/srv/Trigger
   ```
   `ncp.launch.py` now starts everything (lidar, rm_base, odom chain, safety,
   bridge, consumer). **Do NOT also run `bench_up.sh`** — two safety_nodes is
   two `/cmd_vel` publishers. Verify `ros2 topic info /cmd_vel` says exactly
   1 publisher before arming. Stop button:
   `ros2 service call /safety/kill std_srvs/srv/Trigger`, in its own terminal.
   Watch with `scripts/diagnostics/watch_stack.py`.
   For BASE-ONLY work (including `velocity_accuracy.py`) use `bench_up.sh`
   instead — `ncp.launch.py` always starts a policy consumer, and in debug mode
   that consumer publishes `/policy/cmd_vel` continuously, which fights any
   script commanding on that topic.
2. **Exercise the breakaway kick — STILL OPEN.** The 2026-08-29 s3 spin legs
   almost certainly fired it (wheel speeds 0.064-0.142 m/s, straddling the
   ~0.14 m/s trigger, and front-right turned) but no `rm_base` log was
   captured, so it remains inference. Command a slow turn (`wz` under
   ~0.65 rad/s) WITH THE NODE LOG CAPTURED, and confirm front-right starts and
   that `still not turning after 3 breakaway kicks` does NOT appear.
3. ~~Measure a clean spin since the calibration refit.~~ **DONE 2026-08-29 s3**
   via `velocity_accuracy.py`, which commands the spin directly and so sidesteps
   the debug rule's direction flipping entirely. `lever_arm_m` refitted
   0.213 -> 0.227. Remaining spin work is the dead-band, not the constant.
3b. **A second REVERSE straight leg.** Reverse is one sample (0.959); a second
   speed separates a scale error from a dead-band there, the same way two
   speeds did for forward.
4. **A wheels-UP revolution count.** The cheapest experiment left: it separates
   wheel SLIP from encoder SCALE in `counts_per_rev`, which is currently fitted
   to the tape with slip baked in. Until then `/odom` is "matches the tape",
   not "correct".
5. `policy_node` derives pose + goal from `/odom`. The chain is live, but see
   the caveat in §4 — decide whether it is good enough, or give the policy a
   goal source that needs no odometry.
6. Do NOT build a heading-hold drift controller until 4/5 give a trustworthy
   heading signal.
7. ~~Make `command_source` read-only on `safety_node`.~~ **DONE 2026-08-29**,
   in the other direction: it is now genuinely runtime-settable and every
   change FORCE-KILLS, so a switch can only ever stop the robot. Verified live.
   Every other parameter on `safety_node` and `policy_node` is now rejected at
   runtime rather than accepted-and-ignored.
8. Hardware safety checklist (`safety_validation.md` §3); sign off its hardware
   rows and the 2026-08-28 yaw fix (Phase 2 gate). The 2026-08-29 floor runs
   exercised most of it — the e-stop arcs under real motion are the gap. Note
   the clamps are currently ABOVE their signed-off values.
9. Replace the front-right motor to remove the stiction workaround entirely.
10. Check whether bernard-2 has the same firmware sign fault (if yes it is
   systemic).

### Robot state at end of 2026-08-29 session

bernard-1 up on its static IP, battery **11.3 V**, clock pushed from the
workstation. Left **clean**: every stack node stopped, `/safety/kill` called
before shutdown, both serial ports free, no vendor driver running. `~/ncp_ws`
is at the deployed branch and colcon-built (6 packages); `~/NCP-on-Rosmaster`
holds the verified checkpoint at `weights/actor.pt`.

To bring it back up:

```
ssh bernard-1
cd ~/ncp_ws && bash scripts/prep_robot.sh          # must print "== ready =="
ros2 launch rm_bringup ncp.launch.py               # trained policy is the default
```
then open `http://192.168.4.100:8080/` from the workstation. Arm from the
dashboard or with `ros2 service call /safety/reset std_srvs/srv/Trigger`.
Add `dry_run:=true` for any run that must not move.

### Robot state at end of 2026-08-29 session 3

bernard-1 was off at session start and came up partway through, on its static
IP. Clock back to **May 24** (~3 months behind) and NOT pushed this session —
`velocity_accuracy.py` uses a monotonic clock deliberately, but push the clock
before recording any bag. `rosmaster_main.py` autostart live and holding
ttyUSB1 as always. Battery **11.9 V**, freshly charged, at the start.

Left with the base stack from `bench_up.sh --mode closed_loop --live --source
policy` RUNNING (lidar, rm_base, odom chain, rm_safety) after the operator's
last leg — run `scripts/bench_down.sh` if it is still up. All code is committed
and deployed; workstation and robot verified byte-identical for every file
touched. Results sheet is `~/ncp_ws/velocity_accuracy.csv` on the robot
(pre-correction copy at `.csv.bak`).

Offline work available any time: the 3.0.0 contract doc refresh, a clear-path
open-space bag (operator), `ros2 topic delay` latency capture.

Settled, do not re-open:
- Observation stays **forward-180 deg** despite the lidar being 360 deg
  (operator, 2026-08-07).
- **No self-occlusion mask** (operator, 2026-08-14, confirmed against live
  scans).
- **Pre-latch stop-zone reads soft-stop to zero, not full velocity**
  (operator, 2026-08-14).
- Base velocity is **`/odom`** (EKF, has a header), not `/vel_raw`.
- Action space is **2 dims `[vx, wz]`**, no lateral strafe (operator,
  2026-08-12). The safety layer nonetheless keeps lateral clamps and arcs.
- **No IMU in the observation** (operator, 2026-08-12).
- **`invalid_range_is_inf` is inert on this driver.** The `0.0 -> range_max`
  mapping in `scan_resampler` is the only defence.
- **The vendored `ncp_policy.py` stays byte-exact.** Every hardware fix goes in
  our own modules — that is what `scan_bridge_node` and `rm_base` are.
- **The base is driven via `rm_base`, in `control_mode:=closed_loop`**
  (open_loop 2026-08-28; closed_loop is the validated path as of 2026-08-29 —
  it is what holds the four wheels at a common speed). The firmware speed PID
  is not trusted on this robot. Do not go back to `set_car_motion` / the vendor
  `Mcnamu_driver_X3` for motion without first proving the firmware is fixed.
- **The vendor odom chain runs WITHOUT the vendor driver**, via
  `rm_bringup/launch/odom.launch.py` (2026-08-29). rm_base feeds it `/vel_raw`
  and `/imu/data_raw`. Never start `yahboomcar_bringup_X3_launch.py` while
  rm_base is up — it drags in `Mcnamu_driver_X3` (single-writer serial port)
  and `yahboom_joy_X3` (publishes `/cmd_vel` directly).
- **The calibration numbers are TAPE-FITTED, not derived** (2026-08-29).
  `wheel_circumference_m` 0.248 is measured (79 mm wheel); `counts_per_rev`
  2360 and `lever_arm_m` **0.227** are fitted to make commanded match achieved.
  The lever arm is a scrub compensation — do NOT "correct" it back to the
  physical 164.555 mm. Re-tune against a tape, never against odometry.
- **`lever_arm_m` 0.227 is fitted at wz 0.60 AND CANNOT BE RIGHT EVERYWHERE**
  (2026-08-29 s3). There is a rotational dead-band — about the first
  0.107 rad/s of a commanded spin is lost to scrub — and a multiplier cannot
  cancel an offset. `wz` ~0.30 under-rotating ~20% is EXPECTED. Raising the
  lever to fix it over-rotates wz 0.60, as measured (0.236 gave 1.060 there
  while the 0.30 legs had only reached 0.815/0.733). The fix is an affine yaw
  term, not a bigger constant.
- **Never pool FORWARD and REVERSE, or CW and CCW, in one accuracy fit**
  (2026-08-29 s3). The calibration is per-direction and the robot has a
  measured 4% forward/reverse and 9.7% CW/CCW asymmetry, so pooling reads a
  direction difference as "varies with speed" and sends you to refit the wrong
  thing.
- **Achieved-vs-commanded is measured against the TIME INTEGRAL OF `/cmd_vel`,
  never `vx * secs`** (2026-08-29 s3). rm_safety acceleration-limits every
  command, so the ramp alone is ~4% of a 5 s leg at 0.25 m/s and would be
  booked against the base. `velocity_accuracy.py` is the tool.
- **`dead_pwm` must be fitted from TWO speeds, and erring HIGH is the safe
  direction** (2026-08-29). A one-speed fit cannot separate it from
  `gain_pwm_per_mps`, and setting it too low means a wheel never starts —
  which the stall guard then makes permanent.
- **Front-right (M3) IS bad — it is STICKY, not weak** (settled 2026-08-29 s2,
  after a correction and a re-correction; read §3 before reopening).
  Breakaway PWM 60 fwd / 55 rev to START, running dead 30.2/gain 132.1 fwd and
  27.7/145.4 rev to KEEP GOING. Its running gain is normal, so the motor has
  torque — it has stiction. The RUNNING fit is configured plus a bounded
  breakaway kick; do NOT set breakaway as dead_pwm.
- **A dead wheel reads like a working one on the floor.** The other three drag
  it and its encoder reports the ROBOT's motion — front-right showed 0.250
  against a 0.254 target while contributing nothing. Suspiciously PERFECT
  tracking is the tell. Diagnose a suspect wheel WHEELS-UP with the motor
  driven directly (`scripts/diagnostics/fr_threshold.py`).
- **Never fit dead_pwm/gain from a measurement that includes a TURN.** A spin
  carries rotational scrub load, which the fit misattributes to the dead-band —
  dead_pwm comes out far too high, gain far too low, and the model then only
  matches at the one speed it was fitted at. Two STRAIGHT legs at different
  speeds (`fit_reverse.py`).
- **`closed_loop.guards` in the calibration YAML SHADOWS the module defaults**
  in `wheel_control.Guards`. A stale key there silently overrode the code for
  several hardware runs. Change both, or delete the key.
- **`max_vx` is coupled to `slow_zone_m` AND to `max_pwm`.** Raising it means
  redoing the stopping-distance sum and checking the wheels do not saturate.
- **The debug rule has no spin-direction hysteresis** — in a symmetric spot it
  flips every ~2 s and nets zero rotation. Environmental, not a stack fault;
  the vendored policy stays byte-exact.
- **Motor→wheel map is `M1=FL, M2=BL, M3=FR, M4=BR`** (measured 2026-08-28),
  NOT the Yahboom sample order. `set_motor(+)` = forward; forward wheel =
  DECREASING `get_motor_encoder` count.
- **The scan bridge publishes RELIABLE and subscribes BEST_EFFORT.** Asymmetric
  on purpose (2026-08-27 s2); matching them breaks upstream silently.
- **`PolicyRunner.load_state_dict` stays STRICT.** The 2026-08-29 vendor-pin
  mismatch was a startup crash precisely because of it. Under `strict=False`
  the missing tensors would have been left at random init and the robot would
  have run a half-trained network with no error anywhere.
- **Probe this checkpoint with a REAL scan, never a synthetic one.** An
  all-beams-at-max synthetic scan is far off-distribution and produces alarming
  full-reverse/full-yaw output that says nothing about the policy (2026-08-29).
  Fixtures are in `tests/fixtures/`; `tg30_open_scan.yaml` is MULTI-DOCUMENT.
- **The policy limit-cycles in `wz` when the robot is STATIONARY** — a period-2
  oscillation driven by the `previous_action` feedback, which rm_safety's
  acceleration limiter filters to ~zero net yaw. It largely resolves once the
  observation updates with motion (2026-08-29). Do not diagnose yaw behaviour
  from a dry-run bench test, and do not "fix" the feedback convention — it is
  faithful to upstream and the choice is the open `clamped_action_note`.
- **`pkill -f ncp.launch.py` DOES NOT STOP THE STACK — use
  `scripts/stack_down.sh`.** It kills the launch PARENT and leaves every node
  running. The orphans survive the next `ros2 launch`, giving two full stacks:
  two safety_nodes (TWO `/cmd_vel` PUBLISHERS), two EKFs fighting over the
  `odom->base_footprint` TF, two rm_base_nodes on the base-board port, and two
  lidar drivers racing for ttyUSB0 — which kills BOTH, so the symptom you
  actually notice is **a stale `/scan`**, several layers from the cause.
  Happened twice on 2026-08-29. Doubled rates on the dashboard (`/odom` 61 vs
  30, `/cmd_vel` 41 vs 20) are the giveaway, and are exactly why the dashboard
  shows expected-vs-actual rather than just a number.
- **The slow zone and the stop zone have SEPARATE arcs** (2026-08-29). Slow is
  a 30 deg forward cone with lateral monitoring disabled at 0.55 m; stop keeps
  90 deg forward / 60 deg lateral at 0.35 m. Do not collapse them back into one
  arc pair: the cone is deliberately too narrow for the stop radius
  (0.35*tan(15 deg) = 0.094 m, narrower than the robot), and applying it there
  would gut stop coverage. `from_dict` refuses a slow arc wider than the stop
  arc.
- **`slow_zone_m`, `max_vx` and `slow_forward_arc_deg` are all coupled.** The
  band must cover braking + filter travel (0.117 m at max_vx 0.25, band 0.200 m
  = 1.71x at slow 0.55), AND the cone must still subtend the robot's ~0.13 m
  half-extent at the slow radius (0.55*tan(15 deg) = 0.147 m). Shrinking the
  radius without widening the cone eventually stops covering the robot's own
  width. Both are pinned by tests in `tests/unit/test_safety_arc_split.py`.
- **The arc split and the 0.55 radius are REASONED, NOT MEASURED** (2026-08-29).
  Reviewed clean and test-covered, but no recorded run has been scored against
  them — the operator declined a baseline capture, so there is no before/after.
  Unknown: how often the old wide arc actually fired and at what bearings.
  `scripts/record_run.sh` + the replay tier can still answer that later.
- **`fixed_resolution:=true` is REQUIRED on the lidar whenever slam_toolbox
  runs.** Without it the TG30's ray count varies and karto silently rejects
  every scan after the first. Do not remove it to "simplify" the lidar params.
- **Never map a ROS stamp onto the monotonic clock via a captured offset.**
  Use each clock consistently with itself: ROS stamp vs ROS clock for
  freshness, monotonic for intervals. bernard-1 has no NTP and its clock is
  pushed by hand, so the two disagree by months.
- **SLAM corrects the pose; it does NOT fix the odometry.** `/odom` is
  unchanged and still untrustworthy for anything reading it directly. Only
  `policy_node` with `pose_source:=map` benefits.
- **rm_dashboard publishes NO topic.** It is a leaf: subscribe-only, plus
  /safety/kill, /safety/reset and parameter sets. Never give it a publisher —
  that is how it stays structurally incapable of being a second `/cmd_vel`
  source.
- **`bench_down.sh` RESTARTS the vendor `Mcnamu_driver_X3`**, which then holds
  the base-board serial port. Always re-run `prep_robot.sh` after it
  (2026-08-29).

### Remote-command gotchas

Full detail lives in the `robot-ssh` skill — now covering **both** robots, the
`/dev/rplidar` symlink race, the by-id lidar port, the TG30's two-cable power
requirement, the base-board serial trap, bernard-1's missing internet and wrong
clock, the `pkill -f` self-match trap, multicast-proves-nothing, and the
`maxInitialPeersRange` default-4 trap. Load that skill before any robot SSH
command rather than duplicating it here.

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
  so the safety layer is always the binding constraint. Both sets are
  provisional and raised only after Phase 2 sign-off. Current values
  (2026-08-14, live teleop tuning): safety `max_vx/vy/wz` = 0.15/0.25/0.4,
  `max_ax` = 0.75 (was 0.25/0.25/0.4 and 0.5 respectively before today);
  interface action scaling `vx_max/wz_max` = 0.3/0.5 unchanged.
