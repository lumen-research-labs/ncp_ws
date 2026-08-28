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

## 4. Current Goal

**The full NCP pipeline runs on bernard-1 hardware** (2026-08-27 s2): lidar ->
32-beam bridge -> observation -> CfC -> `/policy/cmd_vel` -> rm_safety clamp,
with all three consumers validated. Phases 1-3 are effectively done. **Nothing
has moved yet** — every rm_safety test used `-r /cmd_vel:=/cmd_vel_test`.

**The velocity failure is root-caused (2026-08-28 s2): the STM32 firmware's
closed-loop speed control is broken on this robot** — encoder feedback is
sign-inverted vs the PID's expectation, so any nonzero command rails the PWM
to ~100% and the wheel runs flat out regardless of the commanded value. The
front-right motor (M3) is also failing. Not fixable in ROS, not fixable in
firmware without a reflash (no image, no internet on bernard-1). Full detail
and the measured motor→wheel map in §3.

**The chosen workaround is `rm_base`** (built 2026-08-28 s2, offline, untested):
an open-loop mecanum driver that replaces the vendor `Mcnamu_driver_X3`, does
the kinematics in ROS, and drives raw `set_motor` PWM — bypassing the firmware
PID and the dead encoder. `dry_run` defaults true; it has a real `/cmd_vel`
watchdog. Needs colcon build + wheels-up smoke test + wheels-down PWM
calibration before it can drive.

**MOTION IS GATED.** `/odom`, `/odom_raw`, `/vel_raw` from the vendor driver
are garbage (§3). rm_base publishes a sign-corrected `/vel_raw`, but that is
unverified and only ~3.5 wheels of data. Do not drive on the floor until
rm_base is calibrated and its `/vel_raw` checked against a tape measure.

**Still no trained checkpoint.** `mode:=ncp` and `policy_node` both run RANDOM
weights; observed output was a near-constant (+0.03, -0.09). What has been
proven is the PLUMBING, not a policy. `ACTION_HIGH` v = 2.0 m/s vs safety
`max_vx` 0.15 — the ~13x clamp is deliberate; do not raise it.

### Next session, in order

1. **Clean re-bringup of bernard-1** (it rebooted): push the clock, kill
   `rosmaster_main.py` by PID, start yahboom bringup, `teleop_joy.launch.py`,
   lidar (by-id CP2102 path). Charge first — was ~10.5 V.
2. **colcon build `rm_base`** on the robot, then **wheels-up smoke test**:
   kill the vendor `Mcnamu_driver_X3`, `ros2 launch rm_bringup base.launch.py`
   (dry_run defaults true), confirm `/imu/data_raw` `/voltage` `/vel_raw`
   `/edition` still publish and match the vendor driver. Then `dry_run:=false`
   with rm_safety armed, wheels UP, kill switch in hand: command small
   `/policy/cmd_vel` (or teleop) values and watch `/base/debug` PWM + the
   wheels. Verify direction (forward / strafe / yaw) matches the command.
3. **Wheels-DOWN PWM calibration**: drive a known `/cmd_vel`, tape-measure the
   distance, tune `config/base_calibration.yaml` per-wheel `dead_pwm` /
   `gain_pwm_per_mps` until commanded ≈ achieved. The front-right entry will
   need the most work (weak motor).
4. Check rm_base's reconstructed `/vel_raw` against ground truth, and decide
   whether it is good enough to feed the EKF / be the policy's odom source
   (item 5).
5. `policy_node` derives pose + goal from `/odom` — still garbage until the
   EKF has a trustworthy velocity input. Either wire rm_base `/vel_raw` -> the
   yahboom odom integrator -> EKF, or give the policy a goal source that does
   not need odometry.
6. Do NOT build a heading-hold drift controller until 4/5 give a trustworthy
   heading signal.
7. Fold the vendor `joy_ctrl` kill + `teleop_joy.launch.py` + the vendor
   `Mcnamu_driver_X3` kill + `base.launch.py` + `install -d -o jetson -g jetson
   /var/lib/rm_safety` into ONE bringup script — the per-session ritual is now
   long enough to be error-prone.
8. Make `command_source` read-only on `safety_node`.
9. Hardware safety checklist (`safety_validation.md` §3); sign off its hardware
   rows and the 2026-08-28 yaw fix (Phase 2 gate).
10. When possible: replace the front-right motor; check whether bernard-2 has
    the same firmware sign fault (if yes it is systemic — pursue the reflash).

### Robot state at end of 2026-08-28 session 2

bernard-1 rebooted mid-session and came back on its static IP. Left with the
vendor `driver_node` manually restarted and `safety_node` KILLED holding
`/cmd_vel` at zero — **not a clean stack**, needs the full re-bringup above.
Clock reset to May (RTC 1970); `rosmaster_main.py` re-killed by PID, WILL
return on the next boot. Lidar was NOT confirmed back up after the reboot.
Battery ~11.6 V mid-session. `rm_base` is committed but only exists on the
workstation until the robot pulls + builds it.

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
- **The base is driven OPEN-LOOP via `rm_base`** (2026-08-28). The firmware
  speed PID is not trusted on this robot. Do not go back to `set_car_motion` /
  the vendor `Mcnamu_driver_X3` for motion without first proving the firmware
  is fixed.
- **Motor→wheel map is `M1=FL, M2=BL, M3=FR, M4=BR`** (measured 2026-08-28),
  NOT the Yahboom sample order. `set_motor(+)` = forward; forward wheel =
  DECREASING `get_motor_encoder` count.
- **The scan bridge publishes RELIABLE and subscribes BEST_EFFORT.** Asymmetric
  on purpose (2026-08-27 s2); matching them breaks upstream silently.

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
