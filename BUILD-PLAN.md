# ROSMASTER X3 — Policy Deployment Pipeline: Build Plan

**Target reader:** Claude Code (or any implementing agent/developer).
**Purpose:** Build a ROS2 pipeline that runs a trained RL policy on a Yahboom ROSMASTER X3, behind a hard safety layer, with a frozen observation/action interface and full logging.

---

## 0. Ground rules (read before writing any code)

These constraints are non-negotiable and shape every phase below.

1. **The robot is company-provided.** Do not install development tooling on it. No agents, no editors, no ad-hoc `pip install` into the system Python. All robot-side dependencies go inside a Docker container defined by a Dockerfile in this repo.
2. **Two machines, one repo.** Code is authored on the dev workstation and delivered to the robot via `git pull`. Never edit files directly on the robot. If a hotfix is made on the robot to unblock a debug session, it must be reverted and re-applied from the workstation before the session ends.
3. **The safety layer is the only thing that talks to the base driver.** No node may publish to the robot's velocity command topic except the safety node. This is enforced by topic naming (see Phase 3) and must be verified in Phase 2.
4. **Fail closed.** Any error — dropped message, stale observation, NaN in policy output, exception in the inference node — results in zero velocity, not in the last command persisting.
5. **Nothing is trusted until it is tested against replayed data.** Logic that can be tested offline against a rosbag must be tested offline before it is tested on hardware.

### Documentation standard

Every module must have:
- A module-level docstring stating **what it does, what it subscribes to, what it publishes, and what its failure mode is.**
- Type hints on all public functions.
- Inline comments only where the *why* is non-obvious. Do not comment what the code already says.
- Any magic number (timeout, clamp limit, rate) must be a named constant sourced from config, never a literal in the body.

Every ROS2 node must additionally document, at the top of the file:
```
Subscribes:  <topic>  [<msg type>]  @ <expected rate>
Publishes:   <topic>  [<msg type>]  @ <rate>
Parameters:  <name> (<type>, default) — <meaning>
Failure mode: <what happens when inputs go stale or invalid>
```

### Environment note

If ROS2 environment setup on the **dev workstation** blocks progress, the human operator has a personal Obsidian note covering their local ROS install. Ask for it rather than guessing at the setup. Once retrieved, copy the relevant content into `docs/dev_env_setup.md` so it is version-controlled and the pipeline is reproducible.

---

## 1. Repository layout

Create this structure. Packages prefixed `rm_` (ROSMASTER). Two build targets: things that run **on the robot** and things that run **on the workstation** — kept separate so the robot container installs only what it needs.

```
rosmaster_rl/
├── BUILD_PLAN.md                  # this file
├── README.md                      # quickstart: how to build, deploy, run
├── docker/
│   ├── Dockerfile.jetson          # ROS2 Humble + TensorRT, pinned to on-robot L4T
│   ├── docker-compose.yml         # mounts workspace, --network host, device passthrough
│   └── README.md                  # how to build/run the container ON THE ROBOT
├── docs/
│   ├── interface_contract.md      # human-readable version of the frozen interface
│   ├── dev_env_setup.md           # workstation ROS2 setup (from operator's notes)
│   ├── deployment.md              # ssh + git pull + colcon build + launch runbook
│   └── safety_validation.md       # signed-off checklist results from Phase 2
├── config/
│   ├── interface.yaml             # THE FROZEN CONTRACT — see Phase 3
│   ├── safety_limits.yaml         # clamps, e-stop distances, timeouts
│   └── topics.yaml                # canonical topic names, single source of truth
├── src/
│   ├── rm_interfaces/             # custom msg/srv definitions (ROBOT + WORKSTATION)
│   ├── rm_safety/                 # safety layer node (ROBOT)
│   ├── rm_policy/                 # ONNX/TensorRT inference node (ROBOT)
│   ├── rm_bringup/                # launch files, param files (ROBOT)
│   └── rm_tools/                  # offline: bag analysis, export, schema check (WORKSTATION)
├── scripts/
│   ├── deploy.sh                  # workstation → robot: push, pull, build
│   ├── fetch_bags.sh              # robot → workstation: rsync rosbags
│   └── check_interface.py         # validates a running system against interface.yaml
└── tests/
    ├── unit/                      # pure-python logic tests, no ROS runtime
    ├── replay/                    # tests that run nodes against recorded bags
    └── fixtures/                  # small committed bags / synthetic data
```

**Rule:** `rm_tools` must never be a runtime dependency of `rm_safety` or `rm_policy`.

---

## Phase 1 — Sensor verification & remote workflow

**Goal:** Prove every sensor works and that the workstation can see robot topics over the network, without X11 forwarding.

### Tasks

1. **Establish SSH + DDS connectivity.**
   - Document the robot's hostname/IP and SSH access in `docs/deployment.md`.
   - Set a shared `ROS_DOMAIN_ID` (pick a non-zero value, record it in `config/interface.yaml`) on both machines.
   - Verify: run a talker on the robot, `ros2 topic echo` from the workstation. If this fails, check that both are on the same subnet and that DDS multicast is not blocked before debugging anything else.
   - **Do not use X11 forwarding for RVIZ.** RVIZ2 runs on the workstation, subscribing over DDS.

2. **Build the Jetson container.** Write `docker/Dockerfile.jetson` pinned to the L4T/JetPack version actually present on the robot (check with `cat /etc/nv_tegra_release`). Include ROS2 Humble, TensorRT, ONNX Runtime. Build it **on the robot** — do not attempt to build or cross-compile TensorRT artifacts on the workstation.

3. **Bring up each sensor and confirm in RVIZ2 from the workstation:**

   | Sensor   | Verify                                                              |
   |----------|---------------------------------------------------------------------|
   | Lidar    | Scan renders, no dropouts, angular range matches datasheet, correct frame |
   | Depth    | Point cloud renders, depth values sane at known distances            |
   | Camera   | Image stream, check actual achieved FPS vs. advertised               |
   | IMU      | Orientation responds correctly to physical rotation; check for drift  |
   | Encoders | Odometry advances correctly; push robot a measured 1m, compare       |

4. **Record the actual rate and latency of every topic.** Use `ros2 topic hz` and `ros2 topic delay`. Write results into `docs/interface_contract.md`. **This data determines the control frequency chosen in Phase 3** — do not skip it or guess.

5. **Record a baseline bag** with all sensors while teleoperating the robot for ~2 minutes. Commit a trimmed version to `tests/fixtures/`. Every later phase tests against this.

### Acceptance criteria
- [ ] RVIZ2 on workstation shows all five sensor streams live, no X11.
- [ ] Measured rates for all topics documented.
- [ ] Baseline bag recorded and a trimmed fixture committed.
- [ ] Container builds and runs on robot; `ros2 topic list` inside it matches host.

### Deliverable
`docs/deployment.md` with a working step-by-step "cold start to live RVIZ" runbook.

---

## Phase 2 — Low-level safety layer

**Goal:** A node that sits between any command source and the base driver, and that is proven to stop the robot. Built and validated **before** any policy exists.

### Architecture

```
   /policy/cmd_vel  ──┐
                      ├──► [ rm_safety ] ──► /cmd_vel  ──► base driver ──► STM32
   /teleop/cmd_vel  ──┘         ▲
                                │
                    /scan, /odom, /safety/enable
```

The safety node is the **sole publisher** of `/cmd_vel`. Command sources publish to namespaced input topics and have no route to the base driver.

### Components

1. **Velocity & acceleration clamping**
   - Clamp linear x, linear y (mecanum — X3 is omnidirectional, do not forget lateral), and angular z to limits from `config/safety_limits.yaml`.
   - Clamp *acceleration* by comparing against the last published command and the elapsed time. This is what protects the STM32 and the drivetrain from step commands.
   - Start limits deliberately low. Raise them only after Phase 4 is stable.

2. **Lidar-based emergency stop**
   - Maintain a rolling minimum range within a configurable forward arc (and, for a mecanum base, lateral arcs too).
   - Two-tier: **slow zone** (scale commanded velocity down proportionally) and **stop zone** (zero velocity, latch).
   - Filter lidar noise — require N consecutive scans below threshold before triggering, so a single bad return doesn't latch a stop. Make N configurable.
   - **Latching:** once stopped, stay stopped until explicitly reset via service call. Do not auto-resume when the obstacle clears.

3. **Software kill switch**
   - A service (`/safety/kill` and `/safety/reset`) plus a persistent state.
   - When killed: publish zero velocity continuously (do not simply stop publishing — a stale-command watchdog downstream may not exist).
   - Kill state survives node restart if feasible; if not, default to killed on startup and require explicit enable.

4. **Staleness watchdog**
   - If no command arrives on the active input topic within `command_timeout_ms`, publish zero.
   - If `/scan` goes stale beyond `scan_timeout_ms`, treat as e-stop condition — you cannot guarantee safety without perception.

5. **Status publishing**
   - Publish `/safety/status` (custom msg in `rm_interfaces`) at a fixed rate with: current state (OK / SLOW / STOPPED / KILLED), triggering reason, min observed range, and whether clamping is active. This is what you inspect in Phase 5.

### Testing (in this order)

1. **Unit tests, no ROS runtime.** Extract clamping and e-stop decision logic into pure functions in a `safety_logic.py` module that takes plain inputs and returns plain outputs. Test: step inputs, NaN inputs, zero dt, out-of-range values, single-scan noise spikes.
2. **Replay tests.** Run the node against the Phase 1 baseline bag. Confirm no spurious e-stops during known-safe segments.
3. **Hardware test — no policy.** Wheels off the ground first. Then:
   - Teleop, verify clamping visibly limits speed.
   - Walk toward the lidar, verify slow zone then stop zone.
   - Call kill service mid-motion, verify immediate stop.
   - Kill the command publisher entirely, verify watchdog zeroes output.
   - Cover the lidar, verify stale-scan e-stop.

### Acceptance criteria
- [ ] Every test above passes and results are recorded in `docs/safety_validation.md` with date and operator.
- [ ] Verified by inspection that nothing other than `rm_safety` publishes `/cmd_vel`.
- [ ] E-stop latches and requires explicit reset.

**Do not proceed to Phase 4 until this checklist is signed off.**

---

## Phase 3 — Freeze the interface

**Goal:** A single machine-readable contract that both the training side and the robot side conform to. This is the highest-leverage artifact in the project — everything downstream breaks silently if it's wrong.

### `config/interface.yaml`

Define, explicitly and with units:

```yaml
interface_version: "1.0.0"     # bump on ANY change; nodes must assert on this

control:
  rate_hz: <chosen>            # justified by Phase 1 measured rates
  frame_id: base_link

observation:
  # Order is load-bearing. The vector is built in exactly this order.
  layout:
    - name: lidar_ranges
      source_topic: /scan
      shape: [<N>]
      preprocessing: <downsample method, clipping, normalization>
      units: meters
    - name: base_velocity
      source_topic: /odom
      shape: [3]               # vx, vy, wz — mecanum
      units: [m/s, m/s, rad/s]
    - name: imu_orientation
      source_topic: /imu
      shape: [<N>]
      note: <sin/cos representation if angles are used>
  total_dim: <sum>
  dtype: float32

action:
  layout:
    - name: cmd_vx
      range: [<min>, <max>]
      units: m/s
    - name: cmd_vy
      range: [<min>, <max>]
      units: m/s
    - name: cmd_wz
      range: [<min>, <max>]
      units: rad/s
  total_dim: 3
  dtype: float32
  scaling: <how [-1,1] network output maps to physical units>

coordinate_frames:
  base_link: <definition, axis convention>
  odom: <...>
  laser: <transform from base_link>
  camera_link: <...>
```

### Tasks

1. **Choose the control rate deliberately.** It must be at or below the slowest sensor in the observation, with headroom. If lidar is 10 Hz, do not run control at 50 Hz — you'd be feeding the policy four stale scans out of five. Document the reasoning in `docs/interface_contract.md`.
2. **Angle representation.** If any orientation enters the observation, use sin/cos rather than raw angles to avoid wraparound discontinuity at ±π.
3. **Write a schema loader** in `rm_interfaces` (or a shared Python module) that parses `interface.yaml` and exposes it as typed objects. **Both the observation builder and the offline export tooling import this same loader.** Never hand-write the ordering twice.
4. **Write `scripts/check_interface.py`** — introspects a live system (or a bag) and asserts every declared topic exists, publishes the declared message type, and runs at the declared rate. Run this before every hardware session.
5. **Version assertion.** The policy inference node reads the `interface_version` embedded in the exported model metadata and refuses to start if it mismatches `interface.yaml`.

### Acceptance criteria
- [ ] `interface.yaml` complete with units on every field.
- [ ] Schema loader written, imported by both robot and tooling code.
- [ ] `check_interface.py` passes against the live robot.
- [ ] Chosen control rate justified in writing against measured sensor rates.

---

## Phase 4 — Policy inference node

**Goal:** Load an exported model, build observations conforming to the frozen interface, run at the control rate, publish through the safety layer.

### Structure (`rm_policy`)

Split into three testable pieces — do not write this as one monolithic node:

1. **`observation_builder.py`** — subscribes to sensor topics, maintains latest values with timestamps, assembles the observation vector per `interface.yaml`. Returns `None` if any component is stale. **Pure enough to unit test against synthetic messages.**
2. **`policy_runner.py`** — wraps the ONNX Runtime / TensorRT session. Takes a vector, returns an action. Knows nothing about ROS. Validates output for NaN/inf and shape.
3. **`policy_node.py`** — the ROS2 node. Timer at the control rate. Calls builder, calls runner, publishes to `/policy/cmd_vel`. Handles all failure paths.

### Required behaviours

- **Dry-run mode** (`~dry_run` parameter, **default true**). Computes everything, publishes to `/policy/cmd_vel_debug` instead of the live topic. This is how you validate before ever moving the robot.
- **Stale observation → publish zero**, and log at WARN with which component was stale. Do not skip the publish.
- **Inference exception → publish zero**, log at ERROR, increment a failure counter published on `/policy/status`.
- **Timing instrumentation.** Measure inference latency every cycle; publish rolling p50/p99 on `/policy/status`. If p99 exceeds the control period, that is a design failure to surface loudly, not to silently absorb.
- **TensorRT engines are built on the target device.** The pipeline is: train on workstation → export ONNX on workstation → commit ONNX → build TRT engine on the robot as a deploy step. Never commit a `.engine` file.

### Bring-up sequence (follow exactly)

1. Run in dry-run against the **replayed baseline bag** on the workstation. Confirm observation vector shape and that actions are in range.
2. Run in dry-run on the **robot, stationary**. Confirm timing meets the control rate.
3. Run in dry-run on the robot while **teleoperating manually**. Compare `/policy/cmd_vel_debug` against what you'd expect. This is the sanity check that the observation is actually correct.
4. **Wheels off the ground.** Disable dry-run. Confirm wheels respond and safety clamps engage.
5. On the floor, in an open area, with the kill switch in hand and reachable.

### Acceptance criteria
- [ ] Runs at target rate with p99 latency under the control period.
- [ ] All three failure paths (stale, NaN, exception) verified to produce zero output.
- [ ] Interface version assertion verified to reject a mismatched model.
- [ ] Dry-run is the default; live mode requires explicit opt-in.

---

## Phase 5 — Logging & replay

**Goal:** Every hardware run is reconstructable offline.

### Tasks

1. **Recording launch file** in `rm_bringup` that captures, at minimum: all raw sensor topics, `/policy/cmd_vel`, `/cmd_vel`, `/policy/status`, `/safety/status`, `/tf`. Recording both the policy's request and the safety layer's output is what lets you tell "bad policy" from "safety intervened" after the fact.
2. **Use a consistent bag naming scheme**: `<date>_<time>_<model_hash>_<interface_version>_<short_description>`. Ambiguous bags are worthless three weeks later.
3. **`scripts/fetch_bags.sh`** — rsync from robot to workstation. Record locally on the robot (fast disk), pull afterward. Never stream bags over the network during a run.
4. **Analysis tooling in `rm_tools`**: load a bag, plot commanded vs. actual velocity, plot inference latency distribution, list every safety intervention with timestamp and reason.
5. **Replay harness** in `tests/replay/` — run the observation builder and policy runner against a bag and diff against what was logged live. This catches drift between training-time and deployment-time preprocessing, which is the single most common silent failure in this kind of pipeline.

### Acceptance criteria
- [ ] A recorded run can be replayed and produces the same observation vectors as logged live.
- [ ] Safety interventions are visible and attributable in the analysis output.

---

## Execution order & checkpoints

Work strictly in phase order. After each phase, stop and report status rather than proceeding automatically.

| Phase | Blocks on | Human sign-off required |
|-------|-----------|-------------------------|
| 1 | — | Sensor verification |
| 2 | 1 | **Yes — safety checklist** |
| 3 | 1 (needs measured rates) | Control rate decision |
| 4 | 2 and 3 | **Yes — before first live run** |
| 5 | 3 | No |

### Open questions to resolve with the operator before starting

1. Which controller board is on this X3 (Jetson Nano / Orin Nano Super / Orin NX / RPi 5)? Determines the container base image and whether TensorRT is available.
2. What ROS2 distro is on the robot as shipped — Foxy or Humble? Affects everything.
3. What is the trained policy's actual observation and action space? Phase 3 cannot be completed without this.
4. Is there a physical E-stop on the unit, or is the software kill switch the only stop mechanism? Changes how conservative Phase 2 limits must be.
5. Is the robot on a network the workstation can reach directly, or is there a NAT/VLAN boundary that will break DDS discovery?

### First actions

1. Ask the operator the five questions above.
2. Create the repo skeleton from Section 1, with README and empty package manifests.
3. Begin Phase 1, task 1 (SSH + DDS connectivity).
