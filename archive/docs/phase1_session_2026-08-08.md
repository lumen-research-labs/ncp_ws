# Phase 1 session log — 2026-08-08

Operator: Dev Patel. Robot: `bernard-2` (192.168.1.154). Session ended when the
robot's battery died mid-run.

## Access — WORKING

```bash
ssh bernard-2          # user jetson, key ~/.ssh/bernard-2, config alias set up
```

**Non-interactive SSH does not source `~/.bashrc`** — Ubuntu's default bashrc
returns early for non-interactive shells, before the ROS lines at 150-165. So
`ssh bernard-2 'ros2 topic list'` fails with an empty `ROS_DISTRO`. Every remote
command needs this prelude:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=28
```

Also: never `pkill -f` a pattern that appears in your own SSH command line —
it matches the remote shell running it and kills the session. Cost 10 minutes.

## Platform — CONFIRMED

| | |
|---|---|
| L4T | **R36.4.3** (Jan 2025) → JetPack 6.2. Pin `Dockerfile.jetson` to this. |
| OS | Ubuntu 22.04.5 LTS, aarch64 |
| RAM / disk | 15 GiB / 35 GiB free of 147 GiB |
| `ROS_DOMAIN_ID` | 28, set in `~/.bashrc` line 155 |
| `/dev/rplidar` | → ttyUSB0, CP2102 (10c4:ea60) |
| `/dev/myserial` | → ttyUSB1, CH340 (1a86:7523), base board |
| Joystick | none connected — bringup cannot command motion |
| Containers | `cpt_b2`, `cpt_b2_cradle`, both exited 2 months ago |

## What launches what

`yahboomcar_bringup_X3_launch.py` contains: `robot_state_publisher`,
`joint_state_publisher`, the Mcnamu driver + base node, **`imu_filter_madgwick`**,
**`robot_localization` EKF** (`ekf_x1_x3_launch.py`), and the joy nodes.

**It does not launch the lidar.** That is separate:

```bash
ros2 launch sllidar_ros2 sllidar_launch.py     # defaults: /dev/rplidar, 115200, frame_id=laser
```

Camera: use **`astro_pro_plus.launch.xml`** (sic — the file is spelled that way),
matching the banner's `my_camera: astraplus`. Not `astra_pro.launch.xml`, which
is what the previous team documented for a different camera.

## Topics observed on the robot

```
/Buzzer /JoyState /RGBLight /cmd_vel /diagnostics /edition /imu/data
/imu/data_raw /imu/mag /joint_states /joy /joy/set_feedback
/move_base/cancel /odom /odom_raw /parameter_events /robot_description
/rosout /set_pose /tf /tf_static /vel_raw /voltage
```

Two open interface questions closed by this:

- **`/odom` exists** — published by the robot_localization EKF, so it is
  encoder+IMU fused and carries a proper header. Chosen as the base-velocity
  source over `/vel_raw`, which has no timestamp and therefore cannot support
  honest staleness detection.
- **`/imu/data` is Madgwick-fused** from `/imu/data_raw`. We stay on
  `/imu/data_raw`: the observation uses angular velocity and linear
  acceleration, and Madgwick's contribution is the orientation quaternion,
  which we do not use.

## DDS — root cause found

Symptom: robot publishing, workstation sees nothing. Everything obvious checked
out — same subnet (both WiFi, 192.168.1.0/24), no firewall on the workstation,
both ends `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=28` on both, no
`ROS_LOCALHOST_ONLY`, unicast fine in both directions (ssh proves it).

`ros2 multicast send` / `receive` **passed** — which is exactly what makes this
worth writing down, because it is misleading. That pair uses group `225.0.0.1`.
FastDDS discovery uses **`239.255.0.1`**. A raw-socket test on
`239.255.0.1:14400` (the SPDP port for domain 28) receives nothing. The access
point forwards one group and drops the other.

**A passing `ros2 multicast receive` is not evidence that DDS discovery works.**

Fix: `config/fastdds_unicast_peers.xml` — explicit unicast initial-peers list,
no discovery server needed.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/config/fastdds_unicast_peers.xml
```

Applied on the workstation alone, `ros2 topic list` from the workstation
returned the robot's topics — discovery crossed for the first time. Data still
did not flow, and discovery was unstable, because the robot's own nodes still
announce only via the blocked multicast group. **The profile must be set on
both ends.** That was being applied when the battery died, so it is
**unverified**. First thing to redo next session.

Related, separate finding: the previous team's `/root/fastdds_unicast.xml` is
not readable by user `jetson`, so their host-side SHM fix has never actually
been in effect for host-side nodes. FastDDS silently ignores an unreadable
profile.

## Lidar — did not start

```
[sllidar_node] Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!
process has died [exit code 255]
```

Tried `scan_mode:=Sensitivity` (the launch default) and `scan_mode:=Standard`.
Same failure at the same point — before the device ever answers. Device path
and identity are correct (CP2102 on ttyUSB0), permissions are `crw-rw-rw-`.

**Most likely cause: battery.** `/voltage` read **9.5 V**, against the previous
team's own calibrated battery range of 9.0–11.0 V — i.e. nearly flat. The A1's
motor has to spin up before the device responds, and that is the first thing to
brown out. The robot dropped off the network entirely about 15 minutes later,
which supports it.

Not yet ruled out: a genuinely dead lidar, or an unplugged motor lead. Recharge
first, then retest before investigating anything else.

## Next session

1. **Charge the battery**, confirm `/voltage` reads near 12 V.
2. Retest the lidar. If it still times out on a full battery, it is a hardware
   fault, not power.
3. Put `fastdds_unicast_peers.xml` on **both** ends and confirm data actually
   flows — not just `topic list`, but `topic echo` and `topic hz`.
4. Then the rate/QoS table (`docs/deployment.md` §6), which is still empty and
   which the Phase 3 control rate depends on.
5. Camera bringup via `astro_pro_plus.launch.xml`, verify in RVIZ2.
6. Static lidar scan in open space → re-derive self-occlusion zones for this
   unit. The previous team's `[-6,+6]` zone was attributed to an Astra *Pro*
   body; this robot has an Astra *Plus*, so that mask is not inheritable.
7. Teleop baseline bag — **requires the operator**, not an agent.
