---
name: robot-ssh
description: REQUIRED before running any command on either ROSMASTER X3 robot (bernard-1 or bernard-2) over SSH. Use for any remote ros2 command, launch, topic query, bag recording, or file copy to a Jetson. Triggers - ssh bernard-1, ssh bernard-2, ros2 on the robot, ros2 topic/launch/bag on the Jetson, check the robot, is the robot up, robot battery/voltage, start bringup, start the lidar, start the camera, scp to the robot, FASTRTPS profile on the robot, DDS not working, remote node won't start, lidar not working, no /scan.
---

# Running commands on the robots

There are now **two** robots. They are NOT interchangeable — different
networks, different peripherals, different failure modes. Establish which one
you are on before the first remote command.

Every trap below has cost a real session's time. Read this before starting,
not after something fails.

## Which robot am I on?

| | bernard-1 | bernard-2 |
|---|---|---|
| Address | `192.168.4.100` static, wifi `RobotLab` | `bernard-2.local` (mDNS; IP has moved 3x) |
| SSH key | `~/.ssh/bernard-1` | `~/.ssh/bernard-2` |
| Board | Jetson Orin **NX** 16GB | Jetson Orin **Nano** Super |
| Internet | **NONE** — RobotLab has no WAN uplink | via lab wifi |
| Clock | **~3 months behind, no NTP** (see §5) | synced |
| Camera | Microdia USB webcam (`0c45:6340`) | Astra Plus (depth) |
| Extra HW | mic, gamepad, LED badge (cradle-era) | — |
| Other repos | `~/Desktop/cradle`, `~/Desktop/firmware`, `~/cpt_ws` | `~/Desktop/firmware` |
| Lidar | YDLIDAR TG30, serial …0082 | YDLIDAR TG30, serial …0084 |

Both are `ROS_DOMAIN_ID=28`, both run ROS2 Humble natively on the host, both
have the same lidar model (consecutive units from one batch).

**DOMAIN ID COLLISION:** both robots default to domain 28. If they are ever
powered on the same network they share every topic — including `/cmd_vel`, so
one robot's commands drive the other. Give one of them a different domain
before running them together.

## 0. Session preflight — do this first, every session

Sessions have died mid-run on a flat battery. Check power before starting, so
you don't diagnose a brownout as a software fault.

```bash
ssh -o ConnectTimeout=5 bernard-1 'echo up' || echo "ROBOT UNREACHABLE"
```

Battery (needs bringup running to publish `/voltage`):

```
Calibrated range: 9.0 V (flat) – 11.0 V. 3S pack.
< 10.0 V   →  STOP. Charge. Do not diagnose hardware at this voltage.
10.0-11.0  →  usable, but log it; expect dropouts under motor load.
```

The lidar's motor browns out first, so a flat pack looks like a dead lidar.

## 1. Every remote command needs the source prelude

**Non-interactive SSH does not source `~/.bashrc`** — Ubuntu's bashrc returns
early for non-interactive shells, before the ROS lines. So
`ssh bernard-1 'ros2 topic list'` fails with an empty `ROS_DISTRO`.

```bash
ssh bernard-1 'source /opt/ros/humble/setup.bash && \
  source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && \
  source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && \
  export ROS_DOMAIN_ID=28 && \
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_unicast.xml && \
  <COMMAND>'
```

Sourcing the Yahboom workspace also matters for *checking whether a package
exists*: `ydlidar_ros2_driver` looks absent until that workspace is sourced.
Don't conclude something isn't installed from an unsourced shell.

## 2. THE `/dev/rplidar` SYMLINK RACE — read before touching the lidar

**`/dev/rplidar` is not a reliable path to the lidar.** Two conflicting udev
rule sets are installed on bernard-1 and both match:

- Yahboom `rplidar.rules` / `ydlidar.rules`: CP2102 `10c4:ea60` → `/dev/rplidar`
- Cradle `99-cpt-lidar.rules`: CH340 `1a86:7523` → `/dev/rplidar`,
  and CP2102 → `/dev/yahboom_motor`

Whichever device wins the enumeration race that boot gets the symlink, and it
**flips across reboots** — observed live 2026-08-27, where `/dev/rplidar`
moved from the lidar to the base board and the lidar driver was then talking
to the motor controller. `/dev/yahboom_motor` currently points at the *lidar*.

Ground truth, independent of symlinks:

| chip | is | `lsusb` id |
|---|---|---|
| CP2102 | **the lidar** | `10c4:ea60` |
| CH340 | **the base board** (STM32, `/dev/myserial`) | `1a86:7523` |

Always address the lidar by its stable by-id path:

```
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Check the mapping any time the lidar misbehaves:

```bash
ssh bernard-1 'ls -l /dev/serial/by-id/ /dev/rplidar /dev/ydlidar'
```

## 3. Lidar: what "broken" actually looks like

The unit is a **YDLIDAR TG30**, not an RPLIDAR — despite `RPLIDAR_TYPE=a1` in
the robot's own `.bashrc` banner, which is wrong on both robots. Never use
SLAMTEC `sllidar` on it; its protocol times out and that red herring cost
three sessions.

The TG30 needs **two cables at once**: micro-USB (power-only) into the powered
USB hub, USB-C (data) into the Jetson. The CP2102 adapter is powered by the
data cable, so **the port enumerates and opens even when the lidar module has
no power** — you get a live serial port attached to a dead sensor, and the
driver still logs "LiDAR successfully connected".

Distinguish the three failure modes by raw probe rather than guesswork. The
TG30 answers `A5 90` (device info), `A5 91` (health), `A5 60` (scan) at
**512000** baud, replying with an `a55a` descriptor:

| symptom | meaning |
|---|---|
| only `0x00`/`0xF8`, no `55aa` syncs | not powered, or wrong baud |
| structured int16-ish data, no `a55a` | wrong port — you're on the base board |
| `a55a…` replies, health bytes `00 00 00` | lidar healthy |

Working launch (plain node, matches Yahboom's "4ROS" config):

```bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  -p baudrate:=512000 -p lidar_type:=0 -p device_type:=0 -p sample_rate:=20 \
  -p isSingleChannel:=false -p support_motor_dtr:=true -p reversion:=true \
  -p inverted:=true -p range_max:=64.0 -p range_min:=0.01 -p frequency:=10.0 \
  -p frame_id:=laser_link
```

Yields `/scan` at ~10.1 Hz, 360°, ~2000 pts, 0.01–64 m. `frame_id:=laser_link`
is mandatory — the driver default `laser` is not in the TF tree.

**`invalid_range_is_inf:=true` DOES NOTHING** on this driver (measured with
the flag set: 1982 points, 781 zeros, **0 inf**). No-returns always arrive as
`0.0`. Handling `0.0 → range_max` in the observation builder is the only
protection; a `0.0` left untreated min-pools to zero and latches the e-stop.

## 4. The base-board serial trap — recurs every boot

A Yahboom factory autostart, `Rosmaster/rosmaster/rosmaster_main.py`, holds
the base-board port. If it's up when `yahboomcar_bringup` starts, the driver's
serial-receive thread crashes and **all** base data — `/voltage`,
`/imu/data_raw`, `/vel_raw` — silently reads zero while topics keep publishing
normally.

```bash
ssh bernard-1 'fuser -v /dev/myserial'      # look first
```

If a `python3 …rosmaster_main.py` holds it, kill **that exact PID** and
relaunch bringup. Never `pkill -f` (see §6).

## 5. bernard-1 has no internet and a wrong clock

RobotLab has no WAN uplink — no `apt install`, no `git clone` from GitHub.
Move code across from the workstation over the LAN instead.

Its clock is ~3 months behind with `System clock synchronized: no` and RTC at
1970, because NTP can't reach anything. Consequence: message stamps from this
robot are months off workstation time, so **any staleness check comparing
robot stamps against workstation time fails closed permanently**, and recorded
bags carry wrong timestamps. Push time from the workstation before trusting
either.

## 6. Never `pkill -f` a pattern that is in your own SSH command line

The pattern matches the remote shell running it and kills your own session.

```bash
ssh bernard-1 "pgrep -af '[y]ahboomcar_bringup'"      # look first
ssh bernard-1 "pkill -f '[y]ahboomcar_bringup'"       # bracket breaks self-match
```

Prefer stopping nodes by the PID you started; confirm with `pgrep -af` before
and after.

## 7. `ros2 multicast send/receive` proves nothing about DDS

It uses group 225.0.0.1. FastDDS SPDP discovery uses **239.255.0.1**, which
the lab AP drops. A passing multicast test alongside broken discovery is the
expected result, not a contradiction.

Real verification is data actually flowing:

```bash
ros2 topic list          # necessary, not sufficient — lists without data
ros2 topic echo /odom --once
ros2 topic hz /scan
```

If discovery is partial/flaky, check `maxInitialPeersRange` in
`config/fastdds_unicast_peers.xml` is **20**, not the default 4. The robot runs
8 nodes; a portless initial-peer locator only covers participant IDs
0..maxInitialPeersRange, so the default silently discovers about half. Note
that file's addresses are stale (`10.0.0.x`, two networks ago) — fix before
relying on it. RobotLab is a dedicated router, so plain multicast may work
there; test before assuming the unicast workaround is needed.

## Launch reference

```bash
# Base bringup (Yahboom drivers, native on the host — not in a container)
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py

# Camera — bernard-2 only (Astra Plus). bernard-1 has a plain USB webcam.
ros2 launch astra_camera astro_pro_plus.launch.xml
```

Launches are long-running: start them with `run_in_background` or `nohup`,
then poll with `ros2 node list` rather than blocking on the foreground process.

## THE STOP BUTTON — the base driver has NO watchdog

`Mcnamu_driver_X3` holds its last `/cmd_vel` **indefinitely**. Stop publishing
and the wheels keep turning. There is no command timeout in the driver and
there is no physical E-stop on either robot.

**`ros2 topic pub --once ... "{}"` DOES NOT STOP THE ROBOT.** It sends a single
message that is easily missed — dropped in transit, or sent before the driver's
subscription matched. This happened live on 2026-08-28: a `timeout 10 ros2
topic pub` test ended, the `--once` zero did nothing, and the wheels kept
running.

Only **continuous** zeros stop it. Two ways, in order of preference:

```bash
# 1. BEST — rm_safety is purpose-built for this. It comes up KILLED and floods
#    zeros at 20 Hz. Starting it IS the stop button.
ros2 run rm_safety safety_node --ros-args \
  -p safety_limits_path:=/home/jetson/ncp_ws/config/safety_limits.yaml

# 2. Manual, when rm_safety is not available. Leave it RUNNING until the
#    wheels are still, then Ctrl-C.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}"
```

Killing the process that was commanding motion is NOT sufficient on its own —
it removes the command source but leaves the driver holding the last value.
Kill the publisher AND flood zeros.

Before ANY test that commands velocity: have a dedicated stop terminal open
with the stop command typed and ready to press Enter. Nothing else goes in
that terminal.

Note the ordering trap: `rm_safety` must be stopped to run a direct
`/cmd_vel` test (else two publishers), which means the stop button is exactly
what you just turned off. That is why the manual form above must be ready
first.

## Hard limits — do not cross these unattended

- **Never start `cpt_b2_engine`.** It publishes `/cmd_vel` directly with no
  safety layer and moves the robot the instant it starts. Concurrent with
  `rm_safety` it is two publishers on `/cmd_vel`.
- **Never publish to `/cmd_vel` or `/policy/cmd_vel`** from a remote command.
- **Never run teleop or any test that moves the robot.** There is no physical
  E-stop; `/safety/kill` is the only stop and battery pull is the last resort.
  Anything that moves the base requires a human operator with a kill switch in
  hand — a BUILD-PLAN requirement, not a preference.

If a task seems to need one of these, stop and hand it to the operator.

## Leaving the robot clean

Stop what you started before wrapping up, and record in the session log
whether the stop was clean. A session that ended by the robot dropping off the
network did **not** stop cleanly — say so, so the next session knows why
something looks odd.
