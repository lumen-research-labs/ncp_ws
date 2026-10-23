# Dev workstation ROS2 setup

Workstation is Arch Linux (omarchy/Hyprland). ROS2 does not run natively here —
it runs in a Docker container, which is how the operator's other ROS projects
work. Sourced from the operator's Obsidian notes at
`~/Documents/Obsidian/03 Projects/ROS/`.

## The image

Built from `~/Documents/Projects/docker/ros2_humble/Dockerfile`:

```dockerfile
FROM osrf/ros:humble-desktop-full-jammy
# + python3-colcon-common-extensions, git, ros-humble-simulation
# + "source /opt/ros/humble/setup.bash" appended to /root/.bashrc
```

`humble-desktop-full` includes RVIZ2, which is what Phase 1 needs. Distro
matches the robot (Humble) — good, no cross-distro DDS.

Present locally as both `ros2_humble:latest` and `ros2-humble-gazebo:latest`.

## Running it

Existing runner, `~/Documents/Projects/ros/run_docker_ros.sh`:

```bash
docker run --rm -it \
  --net=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/Documents/Projects/ros/$PROJECT/dev_ws:/workspaces/dev_ws \
  ros2-humble-gazebo:latest
```

Before the first run of a session:

```bash
xhost +SI:localuser:$USER
```

`--net=host` is the important part for us: DDS traffic reaches the LAN directly,
so the container can discover the robot with no extra plumbing. `-e DISPLAY` +
the X11 socket mount give RVIZ2 a display **locally** — this is not X11
forwarding from the robot, which BUILD-PLAN forbids. RVIZ2 runs here and
subscribes over DDS.

Hyprland is Wayland; `QT_QPA_PLATFORM=xcb` routes RVIZ2 through XWayland, which
is why that variable is there. Keep it.

## TODO — adapt the runner for this project

The existing script hardcodes `~/Documents/Projects/ros/$PROJECT/dev_ws`. This
repo is at `~/Documents/Projects/LumenLabs/ncp_ws`, and the operator's own note
says "one ROS project = one workspace, never mix unrelated projects in the same
`dev_ws`". So this project needs its own invocation mounting `ncp_ws` — do not
symlink it into the `ros/` tree.

Also still to add, once the robot's domain ID is known:

```bash
  -e ROS_DOMAIN_ID=<see config/interface.yaml>
```

## Shared with the robot

`ROS_DOMAIN_ID` must match the **Jetson host**, since the Yahboom drivers run
natively on the host and are the source of `/scan`, `/imu/*`, `/vel_raw` and
the sink for `/cmd_vel`. It is not a free choice. See `config/interface.yaml`
`network:` — candidates are 20 (baked into the Yahboom image) and 28 (what the
previous team set their container to). Verify on the robot.

Set it in the container invocation, not interactively, so it can never be
half-applied.

## FastDDS profile

The previous team needed a FastDDS profile disabling the shared-memory
transport on both the Jetson host and inside their container. That fix is
about the **container/host boundary on the robot** — shared memory cannot
cross it.

The workstation talks to the robot over the LAN, where SHM was never a
candidate transport, so the workstation probably does not need the profile.
If `ros2 topic list` shows robot topics but `ros2 topic echo` hangs, that is
the SHM signature and the profile is the fix. Note the symptom now so nobody
spends an afternoon on it.

## Workstation build target

```bash
colcon build --symlink-install --packages-select rm_interfaces rm_tools
```

Robot-side packages (`rm_safety`, `rm_policy`, `rm_bringup`) are not built here.
The two targets are kept separate on purpose — do not co-install.
