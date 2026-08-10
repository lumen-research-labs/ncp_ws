---
name: robot-ssh
description: REQUIRED before running any command on the ROSMASTER X3 robot (bernard-2 / 192.168.1.154) over SSH. Use for any remote ros2 command, launch, topic query, bag recording, or file copy to the Jetson. Triggers - ssh bernard-2, ros2 on the robot, ros2 topic/launch/bag on the Jetson, check the robot, is the robot up, robot battery/voltage, start bringup, start the lidar, start the camera, scp to the robot, FASTRTPS profile on the robot, DDS not working, remote node won't start.
---

# Running commands on bernard-2

Four things have each cost a real session's time on this robot. All four are
avoidable and all four are below. Read this before the first remote command of
a session, not after something fails.

## 0. Session preflight — do this first, every session

Two sessions have now died mid-run on a flat battery. Check power before
starting anything, so you don't diagnose a brownout as a software fault.

```bash
ssh -o ConnectTimeout=5 bernard-2 'echo up' || echo "ROBOT UNREACHABLE"
```

If it's up, re-place the DDS profile — `/tmp` does not survive a reboot:

```bash
scp config/fastdds_unicast_peers.xml bernard-2:/tmp/ncp_fastdds_peers.xml
```

Then read the battery (needs bringup running to publish `/voltage`; if it isn't
up yet, take the reading right after step 2 and before anything else):

```
Previous team's calibrated range: 9.0 V (flat) – 11.0 V. 3S pack.
< 10.0 V   →  STOP. Charge. Do not diagnose hardware at this voltage.
10.0-11.0  →  usable, but say so in the log; expect dropouts under motor load.
```

The lidar's motor is the first thing to brown out, so a flat pack looks exactly
like a dead lidar. (As of 2026-08-08 the lidar is a *confirmed* hardware fault —
zero bytes to raw serial with the motor irrelevant — so a good battery will not
fix it. Don't re-diagnose that one; see CLAUDE.md §4.)

## 1. Every remote command needs the source prelude

**Non-interactive SSH does not source `~/.bashrc`.** Ubuntu's bashrc returns
early for non-interactive shells, before the ROS lines at 150–165. So
`ssh bernard-2 'ros2 topic list'` fails with an empty `ROS_DISTRO`.

Use this wrapper for every remote ROS command:

```bash
ssh bernard-2 'source /opt/ros/humble/setup.bash && \
  source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && \
  source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && \
  export ROS_DOMAIN_ID=28 && \
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/ncp_fastdds_peers.xml && \
  <COMMAND>'
```

`ROS_DOMAIN_ID` is dictated by the Jetson host — 28, not negotiable.
The `FASTRTPS_DEFAULT_PROFILES_FILE` line matters as much as the sources: the
robot end of the unicast-peers fix is what makes workstation↔robot DDS work.

## 2. Never `pkill -f` a pattern that is in your own SSH command line

The pattern matches the remote shell running it and kills your own session.
Cost 10 minutes once already.

```bash
# WRONG — kills the ssh session executing it
ssh bernard-2 'pkill -f yahboomcar_bringup'

# Right — match the actual process, exclude your shell
ssh bernard-2 "pgrep -af '[y]ahboomcar_bringup'"      # look first
ssh bernard-2 "pkill -f '[y]ahboomcar_bringup'"       # bracket breaks self-match
```

Prefer stopping nodes by the launch process you started, and always confirm
with `pgrep -af` before and after.

## 3. `ros2 multicast send/receive` proves nothing about DDS

It uses group 225.0.0.1. FastDDS SPDP discovery uses **239.255.0.1**, which
this access point drops. A passing multicast test alongside broken discovery is
the expected result here, not a contradiction — do not use it as evidence.

Real verification is data actually flowing, in this order:

```bash
ros2 topic list          # necessary, not sufficient — lists without data
ros2 topic echo /odom --once
ros2 topic hz /imu/data_raw
```

If discovery is partial/flaky: check `maxInitialPeersRange` in
`config/fastdds_unicast_peers.xml` is **20**, not the default 4. The robot runs
8 nodes; a portless initial-peer locator only covers participant IDs
0..maxInitialPeersRange, so the default silently discovers about half of them.
This was the actual root cause, found 2026-08-08.

## Launch reference

```bash
# Base bringup (Yahboom drivers, runs natively on the host — not in a container)
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py

# Lidar — MUST override the frame_id; the launch file defaults to "laser",
# which is not in the TF tree and makes every scan untransformable.
ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=laser_link

# Camera — this unit is an Astra Plus, not the Astra Pro the old team documented
ros2 launch astra_camera astro_pro_plus.launch.xml
```

Launches are long-running: start them with `run_in_background`, then poll with
`ros2 node list` rather than blocking on the foreground process.

## Hard limits — do not cross these unattended

- **Never start `cpt_b2_engine`.** It publishes `/cmd_vel` directly with no
  safety layer and moves the robot the instant it starts. Concurrent with
  `rm_safety` it is two publishers on `/cmd_vel`.
- **Never publish to `/cmd_vel` or `/policy/cmd_vel`** from a remote command.
- **Never run teleop or any test that moves the robot.** There is no physical
  E-stop on this unit; `/safety/kill` is the only stop and battery pull is the
  last resort. Anything that moves the base requires a human operator with a
  kill switch in hand — that is a BUILD-PLAN requirement, not a preference.

If a task seems to need one of these, stop and hand it to the operator.

## Leaving the robot clean

Nothing on this robot runs at boot and nothing persists, but stop what you
started before wrapping up, and record in the session log whether the stop was
clean. A session that ended by the robot dropping off the network did **not**
stop cleanly — say so, so the next session knows why something looks odd.
