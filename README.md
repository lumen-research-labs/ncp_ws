# ncp_ws — ROSMASTER X3 Policy Deployment Pipeline

Runs a trained RL policy on a Yahboom ROSMASTER X3 behind a hard safety
layer, with a frozen observation/action interface and full logging.

`BUILD-PLAN.md` is the authority. This README is the quickstart.

## Hardware / platform

| | |
|---|---|
| Controller board | Jetson Orin Nano Super |
| ROS2 distro | Humble |
| Physical E-stop | **None** — `/safety/kill` is the only stop mechanism |
| Networking | Workstation and robot on the same subnet, direct DDS |
| `ROS_DOMAIN_ID` | `42` (both machines) |

## Two machines, one repo

Code is authored on the **dev workstation** and delivered to the **robot** via
`git pull`. Never edit files on the robot. If a hotfix is made there to unblock
a debug session, revert it and re-apply from the workstation before the session
ends.

The robot runs everything inside the Docker container defined in `docker/`.
No development tooling is installed on the robot itself.

### Build targets are separate — do not co-install

```bash
# Robot
colcon build --symlink-install --packages-select rm_interfaces rm_safety rm_policy rm_bringup

# Workstation
colcon build --symlink-install --packages-select rm_interfaces rm_tools
```

`rm_tools` is never a runtime dependency of `rm_safety` or `rm_policy`.

## The one invariant

`/cmd_vel` has exactly one publisher: `rm_safety`. Command sources
(`/policy/cmd_vel`, `/teleop/cmd_vel`) are *inputs* to the safety node and have
no route to the base driver. Everything fails closed — any error publishes zero
velocity rather than letting the last command persist.

## Layout

```
config/     interface.yaml (the frozen contract), safety_limits.yaml, topics.yaml
docker/     robot-side container definition
docs/       interface_contract, dev_env_setup, deployment, safety_validation
src/        rm_interfaces, rm_safety, rm_policy, rm_bringup (robot) + rm_tools (workstation)
scripts/    deploy.sh, fetch_bags.sh, check_interface.py
tests/      unit (no ROS runtime) -> replay (against fixtures) -> hardware (human only)
```

## Status

Phase 1 (sensor verification & remote workflow) — in progress.
See `BUILD-PLAN.md` for the phase table and sign-off gates.

No policy has been trained yet. Phases 1–3 are built and tested against a
placeholder/random-action policy; `config/interface.yaml` exists to unblock
them ahead of training.
