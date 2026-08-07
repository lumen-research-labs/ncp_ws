# Deployment runbook — cold start to live RVIZ

Phase 1 deliverable. Fill in every `TODO(operator)` as it is established;
this file is the thing someone follows at 9am with no context.

## 0. Known platform facts

| | |
|---|---|
| Board | Jetson Orin Nano Super |
| ROS2 distro on robot | Humble |
| Physical E-stop | **None.** `/safety/kill` only; battery disconnect is the last resort. |
| Network topology | Workstation and robot on the same subnet, direct — no NAT/VLAN boundary, no discovery server needed |
| `ROS_DOMAIN_ID` | **28** — confirmed from the host banner 2026-08-08 |
| Robot ROS2 | Runs **natively on the Jetson host**, not only in Docker |
| Base image (previous team) | `yahboomtechnology/ros-humble:4.1.2` |

## 1. Access

Mostly established from the previous team's runbook
(`LumenLabs/firmware/DOCS/`) — only the current address is missing.

```
Robot hostname:  bernard-2       # bernard-2.local resolves via mDNS
Robot IP:        192.168.1.154   # same subnet as the workstation (192.168.1.0/24)
SSH user:        jetson          # per firmware/DOCS — VERIFY, not yet confirmed on this unit
Identity file:   ~/.ssh/lumen    # not yet on this workstation
Firmware path:   ~/Desktop/firmware   (previous team's checkout on the robot)

Host banner reports:

```
ROS_DOMAIN_ID: 28
my_robot_type: x3 | my_lidar: a1 | my_camera: astraplus
```

Password auth is enabled. Do not put the password in this file or anywhere in
the repo — set up key auth instead (below).
```

```bash
ssh -i ~/.ssh/lumen jetson@<HOSTNAME>.local
```

Find the address from the robot's own console:

```bash
hostname       # -> <HOSTNAME>, use <HOSTNAME>.local
hostname -I    # -> the IP
```

### Two open items

1. **The `~/.ssh/lumen` key is not yet on this workstation** (present keys are
   `id_ed25519` and `id_ed25519_github`). Decided 2026-08-07: the operator
   copies the Lumen key across, matching the previous team's setup exactly —
   user `jetson`, key `~/.ssh/lumen`. Then:

   ```bash
   chmod 600 ~/.ssh/lumen        # ssh refuses a key readable by others
   ```

   A `~/.ssh/config` entry is set up so nobody has to remember the address,
   user or key flag ever again — `ssh bernard-2` is the whole command:

   ```
   Host bernard-2 bernard-2.local 192.168.1.154
     HostName 192.168.1.154
     User jetson                    # <- correct this if wrong
     IdentityFile ~/.ssh/lumen
     IdentityFile ~/.ssh/id_ed25519
     IdentitiesOnly yes
     ServerAliveInterval 30
   ```

   The alias works everywhere ssh does, including `scp bernard-2:...`,
   `rsync ... bernard-2:...` and `git`. `scripts/deploy.sh` and
   `scripts/fetch_bags.sh` therefore only need `ROBOT_HOST=bernard-2`.

   Two `IdentityFile` lines are listed deliberately: the Lumen key, and this
   workstation's own key in case it was added to the robot's
   `authorized_keys` instead. `IdentitiesOnly yes` stops ssh offering every
   other key in the agent, which some servers reject as too many attempts.

   To avoid retyping a passphrase (if the key has one):

   ```bash
   eval "$(ssh-agent -s)"
   ssh-add ~/.ssh/lumen
   ```

   Status 2026-08-08: `~/.ssh/lumen` is **not present** on this workstation,
   and this workstation's `id_ed25519` is not authorized for user `jetson`.
   Stale entries for `jetson.local` and `10.0.0.149` (user `devpatel`, a
   different subnet) were removed; backup at `~/.ssh/config.bak.20260807`.
2. **`~/.ssh/config` already has stale entries** for `jetson.local` and
   `10.0.0.149`, both with user `devpatel` — not the `jetson` user the
   firmware docs use. `10.0.0.149` is also on a different subnet from this
   workstation (`192.168.1.0/24`), so it is from another network. Neither
   host answered a ping or a TCP connect on port 22 as of 2026-08-07.
   Clean these up once the real address is known.

## 2. Network / DDS sanity check

First, find out what the robot host actually uses — this is not ours to pick.
The Yahboom drivers run natively on the host and are the source of `/scan`,
`/imu/*`, `/vel_raw` and the sink for `/cmd_vel`, so we must join their domain.

**Answered: `28`**, straight off the host's login banner. Set it identically on
the workstation and in our container — in the shell profile / `docker run -e`,
never interactively, so it can't be half-applied. A mismatched domain ID looks
exactly like a broken network.

```bash
export ROS_DOMAIN_ID=28
```

### Verified so far (2026-08-08)

- Robot reachable at `192.168.1.154`, `bernard-2.local` resolves via mDNS,
  port 22 open.
- Workstation is on `192.168.1.0/24` — same subnet, as expected.
- No firewall active on the workstation (no ufw, no firewalld).
- From the workstation container (`--net=host`, `ROS_DOMAIN_ID=28`),
  `ros2 topic list` returns only `/parameter_events` and `/rosout`, and
  `ros2 node list` is empty. **This is not a DDS failure** — nothing is
  publishing, because the Yahboom drivers have not been launched. The DDS
  path is untested until something is up on the robot to discover.

Talker on the robot, echo on the workstation:

```bash
# robot
ros2 run demo_nodes_cpp talker

# workstation
ros2 topic echo /chatter
```

If this fails, check **in this order** before debugging anything else:

1. Both machines on the same subnet (`ip addr`, compare prefixes).
2. DDS multicast not blocked — `ros2 multicast receive` / `ros2 multicast send`.
3. Firewall on the workstation (`sudo ufw status`) — this is the usual culprit
   on a fresh Arch/Ubuntu install.

**Do not use X11 forwarding for RVIZ.** RVIZ2 runs on the workstation and
subscribes over DDS. If you find yourself typing `ssh -X`, stop.

## 3. Container on the robot

```bash
# On the robot — confirm the L4T version the image must be pinned to
cat /etc/nv_tegra_release
```

```
L4T version:     TODO(phase1)
JetPack version: TODO(phase1)
Base image tag:  TODO(phase1)
```

Build **on the robot**. Never build or cross-compile TensorRT artifacts on the
workstation.

The previous team's proven container recipe on this unit, for reference:

```bash
docker run -it --network host --privileged \
  -v /dev:/dev \
  -v ~/Desktop/firmware:/root/cpt_ws/src/cpt_b2 \
  -e ROS_DOMAIN_ID=28 \
  --name cpt_b2 yahboomtechnology/ros-humble:4.1.2 bash
```

`yahboomtechnology/ros-humble:4.1.2` is known to work on this hardware, but
whether it carries TensorRT and a Jetson-appropriate ONNX Runtime is
**unverified** — Phase 4 needs both. Check inside it before committing to a
base image:

```bash
python3 -c "import tensorrt, onnxruntime; print(tensorrt.__version__, onnxruntime.get_available_providers())"
```

Ours:

```bash
cd ~/ncp_ws/docker
docker compose build
docker compose up -d
docker compose exec robot bash
```

Inside the container, `ros2 topic list` must match what the host sees.

### The SHM transport fix

`--network host` is not sufficient on its own. The previous team had to disable
the FastDDS shared-memory transport on **both** the host and inside the
container — shared memory cannot cross the container boundary. Symptom without
it: `ros2 topic list` shows topics but `ros2 topic echo` hangs silently.

Their profile (`/root/fastdds_unicast.xml`, exported via
`FASTRTPS_DEFAULT_PROFILES_FILE`) declares a plain UDPv4 transport and sets
`useBuiltinTransports=false`. Their docs explain this as "bypassing multicast
discovery" — that is not what it does; a UDPv4 transport still multicasts.
What it removes is SHM. The fix is right, the stated reason is not, and the
distinction matters: **multicast discovery stays intact, so the workstation
can still find the robot over plain DDS with no discovery server.**

## 4. Deliver code

```bash
# workstation
git push

# robot
cd ~/ncp_ws && git pull
docker compose exec robot colcon build --symlink-install \
  --packages-select rm_interfaces rm_safety rm_policy rm_bringup
```

`scripts/deploy.sh` wraps this once the robot's address is filled in above.

## 5. Sensor bringup

These are the previous team's working commands
(`firmware/DOCS/4_robot_start_up.md`). They run on the **Jetson host**, not in
a container, and **order matters**. One SSH session per terminal.

```bash
# Terminal 1 — Yahboom base drivers (motors, IMU, encoders)
source /opt/ros/humble/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py

# Terminal 2 — camera. NOTE: this unit reports my_camera: astraplus, but the
# previous team's command is for the Astra *Pro*. Check which launch file
# applies before running it:
#   ls /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/astra_camera/share/astra_camera/launch/
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
ros2 launch astra_camera astra_pro.launch.xml uvc_product_id:=0x050f   # Astra Pro variant
```

The `my_lidar: a1` in the banner confirms the RPLidar A1 independently.

**Lidar: launch command unknown.** `cpt_b2_engine` subscribes to `/scan`, so it
exists, but `/scan` does not appear in the topic list the previous team
recorded after running the two launches above — which suggests the lidar is a
separate launch they did not document. Find it in Phase 1:

```bash
ros2 pkg list | grep -i -E 'lidar|rplidar|ydlidar'
ls /opt/ros/humble/share/*lidar*/launch/ 2>/dev/null
ros2 topic list | grep scan
```

We do **not** need rosbridge, the HTTP server, or the CLIP vision server —
those are the previous stack's browser plumbing. Skip them.

### Do not run `cpt_b2_engine`

It publishes `/cmd_vel` directly with no safety layer, and its own docs warn
"**Robot will start moving as soon as engine is started.**" Two publishers on
`/cmd_vel` breaks the project's central invariant. If it is ever running,
stop it before bringing up `rm_safety`.

Then from the workstation:

```bash
ros2 topic list
rviz2
```

Verify all five streams — lidar, depth, camera, IMU, encoders — per the Phase 1
task 3 table in `BUILD-PLAN.md`.

## 6. Measured rates

Phase 1 task 4. These determine the control rate in `config/interface.yaml`;
they are measured, never guessed.

Reference point: the previous team's engine ran its decision cycle at **5 Hz**
with a 200 ms latency target. The lidar is an RPLidar A1 (5.5–10 Hz spin). So
expect the answer to land in 5–10 Hz — but measure it.

| Topic | Type | Measured `hz` | QoS | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | **not measured** | — | lidar did not start — hardware fault |
| `/odom` | `nav_msgs/Odometry` | **10.74** | RELIABLE / VOLATILE | EKF-fused; `child_frame_id: base_footprint` |
| `/odom_raw` | `nav_msgs/Odometry` | 10.74 | — | pre-EKF |
| `/vel_raw` | `geometry_msgs/Twist` | 9.22 | RELIABLE / VOLATILE | no header/timestamp |
| `/imu/data` | `sensor_msgs/Imu` | 9.41 | — | Madgwick-fused |
| `/imu/data_raw` | `sensor_msgs/Imu` | **10.74** | RELIABLE / VOLATILE | our source |
| `/imu/mag` | `sensor_msgs/MagneticField` | 10.01 | — | unused |
| `/joint_states` | `sensor_msgs/JointState` | 20.01 | — | |
| `/tf` | `tf2_msgs/TFMessage` | 29.99 | — | `odom -> base_footprint` only |
| `/voltage` | `std_msgs/Float32` | 9.05 | — | battery |
| `/cmd_vel` | `geometry_msgs/Twist` | n/a | RELIABLE / VOLATILE | 1 pub (joy), 1 sub (driver) |

Measured 2026-08-08 from the workstation over DDS. **Everything sensor-side is
~10 Hz.** Nothing is faster. Note `/imu/data_raw` publishes RELIABLE, so a
BEST_EFFORT subscriber will still match, but the reverse would not — worth
remembering when writing the observation builder.

## 6b. TF tree — measured

```
odom -> base_footprint -> base_link -> { imu_link, camera_link, laser_link }
```

| Transform | Translation (m) | Rotation RPY (deg) |
|---|---|---|
| `base_link -> laser_link` | `[0.043, 0.000, 0.110]` | `[0, 0, 0]` |
| `base_link -> imu_link` | `[-0.060, 0.010, 0.010]` | `[-180, 0, -90]` |
| `base_link -> camera_link` | `[0.057, 0.000, 0.038]` | `[0, 0, 0]` |

Three consequences:

1. **The lidar frame is `laser_link`, not `laser`.** `sllidar_launch.py`
   defaults `frame_id` to `laser`, which is not in the TF tree. Launch it as
   `frame_id:=laser_link` or scans cannot be transformed and RVIZ shows
   nothing.
2. **The lidar has zero rotation relative to `base_link`**, so lidar 0 deg is
   robot-forward and the forward-180 arc maps onto scan angles with no offset.
   One less thing to get wrong.
3. **The IMU is rotated: roll 180 deg, yaw -90 deg.** See the warning block in
   `config/interface.yaml` — the raw IMU vectors are in `imu_link`, so their
   axes do not line up with `base_link`. This must be resolved before the
   interface is frozen.

Also worth noting: `camera_link` sits 1.4 cm *forward* of `laser_link` but
7.2 cm *below* it. Whether the camera body actually occludes the lidar plane
on this unit is therefore not obvious from geometry alone — it depends on the
Astra Plus body height. The previous team's `[-6, +6]` mask was measured on a
unit with a different camera and cannot be assumed. Settle it with a real
static scan once the lidar works.

Also record, for each: `ros2 topic info <topic> --verbose` QoS. The previous
team used `sensor_qos` (best-effort) for `/scan` and `/vel_raw` and RELIABLE
for `/imu/data_raw`. A QoS mismatch is a silent no-data failure.

### Two things to settle here

1. **`/odom` present or not.** `interface.yaml` observation component 2 is
   base velocity. `/vel_raw` is a bare `Twist` with **no header and no
   timestamp**, which makes real staleness detection impossible — arrival time
   at the subscriber cannot distinguish a stalled driver from a stalled
   network. Fail-closed needs a timestamp. Prefer `/odom` if it exists.
2. **Lidar self-occlusion.** Take a static scan in open space and confirm the
   previous team's masked zones against our unit:
   `[-6, +6]` (Astra Pro camera body, **dead ahead**), `[18, 24]`, `[38, 41]`.
   Unmasked, the first one puts a permanent phantom obstacle straight in front
   of the robot and the e-stop latches the moment it is enabled.

Copy the conclusions into `docs/interface_contract.md`.
