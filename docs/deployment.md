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
| `ROS_DOMAIN_ID` | `42` |

## 1. Access

```
Robot hostname:  TODO(operator)
Robot IP:        TODO(operator)
SSH user:        TODO(operator)
Auth:            TODO(operator)  # key path, or note that password auth is in use
```

```bash
ssh <user>@<host>
```

If the IP is DHCP-assigned and moves, note the reservation or mDNS name here
rather than re-discovering it every session.

## 2. Network / DDS sanity check

Both machines:

```bash
export ROS_DOMAIN_ID=42
```

Put that in the shell profile on both ends so it is never half-set — a
mismatched domain ID looks exactly like a broken network.

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

```bash
cd ~/ncp_ws/docker
docker compose build
docker compose up -d
docker compose exec robot bash
```

Inside the container, `ros2 topic list` must match what the host sees
(`--network host` is why).

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

```
TODO(phase1): the vendor bringup launch command, once confirmed on the robot.
```

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

| Topic | Type | Advertised | Measured `hz` | Measured `delay` | Notes |
|---|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | | | | |
| `/odom` | `nav_msgs/Odometry` | | | | |
| `/imu` | `sensor_msgs/Imu` | | | | |
| `/camera/color/image_raw` | `sensor_msgs/Image` | | | | |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | | | | |

Copy the conclusions into `docs/interface_contract.md`.
