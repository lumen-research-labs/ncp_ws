#!/usr/bin/env bash
# Bring the NCP stack up, with every environment prerequisite already set.
#
# WHY THIS EXISTS: a non-interactive shell does not source ~/.bashrc, so the
# four `source` lines below are mandatory for every remote launch — and the
# ad-hoc wrapper that used to hold them lived in /tmp, which the Jetson clears
# on every reboot. That cost two silent "the launch didn't start" cycles
# (2026-08-29). This is the same wrapper, kept in the repo where a battery
# swap cannot delete it.
#
# It does NOT run prep_robot.sh or stack_down.sh — those are separate,
# deliberate steps and chaining them here would hide a teardown inside a
# bringup. Run them first:
#
#     bash scripts/stack_down.sh && bash scripts/prep_robot.sh
#     bash scripts/run_stack.sh [launch args...]
#
# Any argument is passed straight through to ncp.launch.py, e.g.
#     bash scripts/run_stack.sh with_slam:=true command_source:=policy
#
# Failure mode: exec's ros2 launch, so the exit status is the launch's own.
# ncp.launch.py refuses to start if a stack is already running, so re-running
# this cannot duplicate one.
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source /home/jetson/ncp_ws/install/setup.bash
export ROS_DOMAIN_ID=28

cd /home/jetson/ncp_ws
exec ros2 launch rm_bringup ncp.launch.py "$@"
