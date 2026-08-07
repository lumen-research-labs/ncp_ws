# Dev workstation ROS2 setup

Workstation is Arch Linux (omarchy/Hyprland). The robot runs Ubuntu 22.04 +
ROS2 Humble inside a container; the workstation needs a matching Humble
environment to talk to it over DDS.

**TODO(operator):** BUILD-PLAN.md §"Environment note" says there is a personal
Obsidian note covering the local ROS install. Paste its relevant content here
rather than re-deriving the setup — this file is the version-controlled copy so
the pipeline is reproducible.

Needed here, at minimum:

- How ROS2 Humble is installed on Arch (AUR? `ros2-humble` via distrobox? conda/RoboStack? a container?)
- The exact `source` line that activates it
- Any `RMW_IMPLEMENTATION` setting, and whether it matches the robot's
- Where `rviz2` comes from in that environment

## Shared with the robot

```bash
export ROS_DOMAIN_ID=42
```

Set on both machines, in the shell profile so it can never be half-applied.

## Workstation build target

```bash
colcon build --symlink-install --packages-select rm_interfaces rm_tools
```

Robot-side packages (`rm_safety`, `rm_policy`, `rm_bringup`) are not built here.
The two targets are kept separate on purpose — do not co-install.
