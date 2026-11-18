# Adding a robot

**A robot is one file: `config/robots/<name>.yaml`.** The stack reads it by
the unit's hostname and configures itself from it. Nothing about a unit is
written anywhere else: not the ROS domain, not the lidar model or which way
it points, not the mount measurements, not the base calibration.

```yaml
robot:
  name: bernard-4
  hostnames: [bernard-4, yahboom]   # what `hostname` prints on that unit
  ros_domain_id: 30                 # unique: two robots on one network
lidar:
  model: a1                         # tg30 | a1 (the driver blocks in ncp.launch.py)
  yaw_deg: 180                      # where its zero points, degrees left of +x
  self_occlusion_zones_deg: []      # [[lo, hi], ...] bearings the chassis blocks
  body_floor_m: 0.12                # a return nearer than this is the robot
  body_radius_m: 0.25               # the sanitizer hides returns under this
base:
  calibration: config/base_calibration_bernard4.yaml
```

The file name must equal `robot.name`. `rm_safety/robot_profile.py` refuses a
malformed file naming the file and the key, two profiles sharing a hostname or
a domain, and a hostname no profile lists (it prints the known names).
`tests/unit/test_robot_profile.py` checks every shipped profile.

## 1. The unit's names and its domain

On the robot, `hostname`. The factory name is `yahboom`; one profile may list
it. Rename when you can (`sudo hostnamectl set-hostname bernard-4`, then
`/etc/hosts`) and list both names. Pick a `ros_domain_id` no other profile
uses. Running two robots on one domain shares `/cmd_vel`: one robot's
commands drive the other. From the workstation, `ros2` tools see one robot at
a time: `export ROS_DOMAIN_ID=<its domain>` first.

## 2. The lidar

Launch once with the profile's guess and read the raw scan:

```bash
bash scripts/run_stack.sh                       # on the robot
python3 scripts/diagnostics/scan_view.py        # http://<robot>:8090
```

- **model**: `tg30` answers at 512000 baud, `a1` at 115200 (`GET_INFO`). The
  wrong driver opens the same CP2102 port and reads nothing.
- **yaw_deg**: stand ahead of the robot. If you appear behind it on the page,
  the lidar's zero points backward: 180. The relay turns the scan at the
  source, so every consumer reads the robot's frame.
- **self_occlusion_zones_deg**: bearings where the chassis returns under
  `body_floor_m` in every scan. On an A1 (minimum range 0.15 m) usually none.
- **body_floor_m** and **body_radius_m**: the nose and the corner sweep; 0.12
  and 0.25 on the X3. The floor must be below `estop.stop_zone_m`.

## 3. The base

Copy `config/base_calibration_bernard3.yaml` to a new file, set
`encoder_sign` (`+1` if `get_motor_encoder` counts up for a forward-turning
wheel, `-1` if down; `scripts/diagnostics/per_motor.py`, wheels up; with the
wrong sign every wheel latches `sign_fault` on the first jog), then fit the
wheel model with [`calibration.md`](calibration.md). Until it is fitted the
robot drives, somewhat off.

## 4. Deploy and verify

```bash
bash scripts/deploy_to_robot.sh <name>
ssh <name> 'cd ~/ncp_ws && colcon build --symlink-install --packages-select rm_safety rm_bringup'
ssh <name> 'bash ~/ncp_ws/scripts/run_stack.sh'
```

The scripts print `robot: <name> (domain N, <path>)`. The dashboard header
and tab title show the name. `ros2 topic list` on the other robot's domain
shows none of this one's topics. Jog forward and back from the dashboard
(`with_waypoint:=true`): `/base/debug` shows no `sign_fault`.

## What the profile does not fix

A blind sector at the mount (bernard-3's rear half) is a remount. A
calibration that is a copy is a copy: fit it.
