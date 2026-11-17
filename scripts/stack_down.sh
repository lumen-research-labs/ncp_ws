#!/usr/bin/env bash
# Stop the ENTIRE NCP stack, children included.
#
# WHY THIS EXISTS: `pkill -f ncp.launch.py` DOES NOT WORK. It kills the launch
# PARENT and leaves every node it started running. Those orphans then have no
# supervisor, survive the next `ros2 launch`, and you end up with two full
# stacks: two safety_nodes (TWO PUBLISHERS ON /cmd_vel -- the one thing this
# workspace exists to prevent), two EKFs fighting over the odom->base_footprint
# TF, two rm_base_nodes contending for the base-board serial port, and two
# lidar drivers racing for /dev/ttyUSB0 -- which kills BOTH, so the symptom you
# actually notice is a stale /scan.
#
# That exact sequence happened twice on 2026-08-30. Killing the parent is not
# stopping the stack; this script is.
#
# ORDER IS DELIBERATE, and it is the same principle as the robot-ssh skill's
# stop-button note: remove the COMMAND SOURCES first, while rm_safety is still
# alive and flooding zeros, and only then stop rm_safety itself. Stopping
# safety first would leave the policy publishing into a base driver with no
# clamp in front of it. rm_base is stopped after safety because its own
# /cmd_vel watchdog then zeroes PWM on timeout, so there is no window where a
# last command persists.
#
# Failure mode: best-effort and idempotent. Every kill is tolerant of "not
# running", so this is safe to run when the stack is already down, and safe to
# run twice. It verifies at the end and reports anything it could not stop
# rather than claiming success.
set -uo pipefail

echo "== stopping the NCP stack =="

# 1. Kill the safety layer's inputs FIRST, then ask safety to stop the robot,
#    while it is still running and able to.
if command -v ros2 >/dev/null 2>&1; then
  timeout 5 ros2 service call /safety/kill std_srvs/srv/Trigger >/dev/null 2>&1 \
    && echo "   /safety/kill called" \
    || echo "   /safety/kill unavailable (safety may already be down)"
fi

# 2. Launch parents. Bracketed so the pattern cannot match this script's own
#    command line over SSH (the pkill self-match trap in the robot-ssh skill).
pkill -f "[n]cp.launch.py"      2>/dev/null
pkill -f "[b]ench_up.sh"        2>/dev/null
sleep 1

# 3. The children, in dependency order. Matched on the INSTALLED PATH
#    (a leading slash) so a pattern can never match an editor, a log tail, or
#    this script.
# THIRD-PARTY nodes, which have to be named individually because they live
# outside our install tree. Order matters (see above).
ORDER=(
  # the odom chain
  ekf_node
  base_node_X3
  imu_filter_madgwick
  robot_state_publisher
  joint_state_publisher
  # localization
  async_slam_toolbox_node
  # sensors last: nothing downstream is left to be confused by their absence
  ydlidar_ros2_driver_node
  sllidar_node              # the RPLIDAR A1 driver (lidar:=a1, bernard-3)
)

# OUR nodes are matched by INSTALL PREFIX rather than by name, so a node added
# to the stack later is torn down automatically. Naming them individually is
# what already failed once: scan_sanitizer_node and slam_toolbox were added to
# the stack on 2026-08-29 and not added here, survived a teardown, and the
# next launch produced TWO scan_sanitizers — the duplicate-stack bug this
# script exists to prevent, reintroduced by extending the stack.
#
# Ordered the same way: command sources first while rm_safety still floods
# zeros, then safety, then the base.
OUR_NODES=(
  policy_node
  scan_bridge_node
  scan_sanitizer_node
  scan_rotate_node
  dashboard_node
  safety_node
  rm_base_node
)
OUR_PREFIX="/home/jetson/ncp_ws/install"
for n in "${OUR_NODES[@]}"; do
  if pgrep -f "[/]${n}" >/dev/null 2>&1; then
    pkill -f "[/]${n}" 2>/dev/null
    echo "   stopped ${n}"
  fi
done
for n in "${ORDER[@]}"; do
  if pgrep -f "[/]${n}" >/dev/null 2>&1; then
    pkill -f "[/]${n}" 2>/dev/null
    echo "   stopped ${n}"
  fi
done

# CATCH-ALL for anything of ours the list above missed. This is the line that
# makes the script survive the stack growing: any process running out of our
# colcon install tree is part of this stack by definition.
if pgrep -f "${OUR_PREFIX}" >/dev/null 2>&1; then
  echo "   stopping remaining ${OUR_PREFIX} processes"
  pkill -f "${OUR_PREFIX}" 2>/dev/null
fi

sleep 3

# 4. Anything that ignored SIGTERM gets SIGKILL. A node still holding a serial
#    port is worse than an ungraceful exit -- the next bringup cannot start.
for n in "${OUR_NODES[@]}" "${ORDER[@]}"; do
  if pgrep -f "[/]${n}" >/dev/null 2>&1; then
    echo "   ${n} ignored SIGTERM, sending SIGKILL"
    pkill -9 -f "[/]${n}" 2>/dev/null
  fi
done
pkill -9 -f "${OUR_PREFIX}" 2>/dev/null
sleep 2

# 5. Verify, and be honest about what is left.
echo "== verify =="
LEFT="$(pgrep -af "[/](policy_node|scan_bridge_node|scan_sanitizer_node|dashboard_node|safety_node|rm_base_node|ekf_node|base_node_X3|imu_filter_madgwick|robot_state_publisher|joint_state_publisher|async_slam_toolbox_node|ydlidar_ros2_driver_node)" 2>/dev/null)"
LEFT="${LEFT}$(pgrep -af "${OUR_PREFIX}" 2>/dev/null)"
if [[ -n "$LEFT" ]]; then
  echo "   !! STILL RUNNING:"
  echo "$LEFT" | sed 's/^/      /'
  exit 1
fi
echo "   all stack nodes stopped"

for dev in /dev/ttyUSB0 /dev/ttyUSB1; do
  if [[ -e "$dev" ]]; then
    if fuser "$dev" >/dev/null 2>&1; then
      echo "   !! ${dev} still held by:"
      fuser -v "$dev" 2>&1 | sed 's/^/      /'
      exit 1
    fi
    echo "   ${dev} free"
  fi
done

echo "== stack down =="
echo "   Run scripts/prep_robot.sh before the next bringup."
