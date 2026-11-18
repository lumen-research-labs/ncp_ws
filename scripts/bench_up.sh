#!/usr/bin/env bash
# rm_base bench/floor bringup: lidar + rm_base + rm_safety, one command.
#
# Runs ON the robot (paths are bernard-1's). Folds the whole per-session
# ritual into one script (BUILD-PLAN "next session" item 7): kill the vendor
# base driver and the factory autostart, ensure the kill-state directory
# exists, start the lidar on its by-id port, start rm_base, start rm_safety.
#
# SAFETY MODEL — nothing moves unless THREE things are true:
#   1. this script was run with --live      (otherwise rm_base is dry_run)
#   2. /safety/reset has been called        (rm_safety starts KILLED)
#   3. something is publishing the command source topic
# Defaults are the safe end of all three.
#
# Usage:
#   bench_up.sh [--mode open_loop|closed_loop] [--cal PATH|floor]
#               [--live] [--source policy|teleop]
#
#   --mode    rm_base control mode. open_loop (default) drives PWM straight
#             from the calibration table. closed_loop adds the per-wheel PI
#             trim on the sign-corrected encoders -- prove it wheels-UP first.
#   --cal     calibration file. Default: the robot profile's base.calibration
#             (config/robots/<name>.yaml). A path, or the word "floor" for
#             config/base_calibration_floor.yaml (cut PWM ceilings, for the
#             first wheels-DOWN run; bernard-1's numbers).
#   --live    dry_run:=false. Without this rm_base computes and publishes its
#             intended PWM on /base/debug but writes ZERO to the motors, so a
#             wheels-up dry run previews the real command safely.
#   --source  which topic rm_safety accepts commands from. Default policy.
#
# SCOPE: this is for BASE work -- calibration, wheels-up checks, driving a
# known velocity. For the full NCP pipeline use ncp.launch.py, which now
# starts the lidar, rm_base, the odom chain and rm_safety itself. Do NOT run
# both: two safety_nodes means two publishers on /cmd_vel.
#
# Tear down with bench_down.sh (restores the vendor Mcnamu_driver_X3).
# NOTE: no `set -u`. ROS's own setup.bash chain reads unbound variables
# (AMENT_TRACE_SETUP_FILES and friends) and aborts the script under it.
set -o pipefail

MODE=open_loop
CAL=""     # empty: the robot profile's base.calibration (set after robot_env.sh)
DRY_RUN=true
SOURCE=policy

while [ $# -gt 0 ]; do
  case "$1" in
    --mode)   MODE="$2"; shift 2 ;;
    --cal)
      if [ "$2" = floor ]; then
        CAL=/home/jetson/ncp_ws/config/base_calibration_floor.yaml
      else
        CAL="$2"
      fi
      shift 2 ;;
    --live)   DRY_RUN=false; shift ;;
    --source) SOURCE="$2"; shift 2 ;;
    -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done

case "$MODE" in open_loop|closed_loop) ;; *)
  echo "ERROR: --mode must be open_loop or closed_loop, got '$MODE'" >&2; exit 1 ;;
esac
case "$SOURCE" in policy|teleop) ;; *)
  echo "ERROR: --source must be policy or teleop, got '$SOURCE'" >&2; exit 1 ;;
esac
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /home/jetson/ncp_ws/install/setup.bash
# Which robot: the profile sets the domain and names this unit's calibration.
source "$(dirname "$0")/robot_env.sh"
if [ -z "$CAL" ]; then
  CAL="$(python3 -c "import sys; sys.path.insert(0, '/home/jetson/ncp_ws/src/rm_safety')
from rm_safety.robot_profile import load_robot_profile
print(load_robot_profile(sys.argv[1]).calibration_path)" "$RM_ROBOT_PROFILE")" || exit 1
fi
[ -f "$CAL" ] || { echo "ERROR: calibration file not found: $CAL" >&2; exit 1; }

# by-id, never /dev/rplidar: that symlink races between the lidar (CP2102)
# and the base board (CH340) across reboots and has pointed at the WRONG
# device on multiple boots. See the robot-ssh skill.
LIDAR=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
BASE=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0

echo "== config =="
echo "   mode=$MODE  dry_run=$DRY_RUN  source=$SOURCE"
echo "   cal=$CAL"
[ "$DRY_RUN" = false ] && echo "   *** LIVE: motors will be driven once /safety/reset is called ***"

echo "== clearing stale processes =="
[ -f /tmp/rm_base.pid ] && kill "$(cat /tmp/rm_base.pid)" 2>/dev/null
pkill -f '[r]m_base_node'
pkill -f '[y]dlidar_ros2_driver_node'
pkill -f '[s]afety_node'
# The odom chain too. Without these a re-run stacks a SECOND EKF and a second
# odom integrator on the first: /odom then has two publishers interleaving
# (its measured rate jumps), and two nodes fight over the odom->base_footprint
# TF. Observed live 2026-08-29 after re-running this script.
pkill -f '[b]ase_node_X3'
pkill -f '[i]mu_filter_madgwick_node'
pkill -f '[e]kf_node'
pkill -f '[r]obot_state_publisher'
pkill -f '[j]oint_state_publisher'
# The vendor driver is a single-writer on the base-board port -- rm_base
# cannot open it until this is gone. Bringup does not respawn it.

# Vendor driver, factory autostart, joy_ctrl and the kill-state directory --
# one shared copy, also used before ncp.launch.py.
bash "$(dirname "$0")/prep_robot.sh" || exit 1

echo "== lidar =="
setsid sh -c "ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  -p port:=$LIDAR -p baudrate:=512000 -p lidar_type:=0 -p device_type:=0 \
  -p sample_rate:=20 -p isSingleChannel:=false -p support_motor_dtr:=true \
  -p reversion:=true -p inverted:=true -p range_max:=64.0 -p range_min:=0.01 \
  -p frequency:=10.0 -p frame_id:=laser_link > /tmp/lidar.log 2>&1 &" < /dev/null

echo "== rm_base  mode:=$MODE  dry_run:=$DRY_RUN =="
setsid sh -c "ros2 launch rm_bringup base.launch.py \
  dry_run:=$DRY_RUN control_mode:=$MODE \
  calibration_path:=$CAL serial_port:=$BASE \
  > /tmp/rm_base.log 2>&1 & echo \$! > /tmp/rm_base.pid" < /dev/null

# The odom chain (vendor base_node_X3 + madgwick + EKF), fed by rm_base's
# /vel_raw. Without this /odom has NO publisher, and rm_safety's odom
# staleness watchdog fires at precedence 3 -- above the zone checks -- so
# every command is zeroed with reason "odom_stale" no matter what you send.
echo "== odom chain (base_node_X3 + madgwick + EKF) =="
setsid sh -c "ros2 launch rm_bringup odom.launch.py \
  > /tmp/odom.log 2>&1 &" < /dev/null
sleep 5

echo "== rm_safety  command_source:=$SOURCE  (starts KILLED) =="
setsid sh -c "ros2 launch rm_bringup safety.launch.py command_source:=$SOURCE \
  > /tmp/safety.log 2>&1 &" < /dev/null

sleep 10
echo
echo "================ CHECKS ================"
ros2 node list 2>/dev/null | grep -E 'rm_base|safety|ydlidar' || echo "!! NODES MISSING"
# `timeout` always exits 124 here, so test the captured text, not $?.
echo -n "/scan:    "
scan_hz="$(timeout 6 ros2 topic hz /scan 2>&1 | grep -o 'average rate: [0-9.]*' | head -1)"
echo "${scan_hz:-!! NO /scan}"
echo -n "/voltage: "; timeout 4 ros2 topic echo /voltage --once 2>&1 | grep data
echo -n "dry_run:  "; ros2 param get /rm_base_node dry_run 2>&1
echo -n "mode:     "; ros2 param get /rm_base_node control_mode 2>&1
echo -n "/cmd_vel publishers: "; ros2 topic info /cmd_vel 2>&1 | grep -i 'publisher count'
odom_hz="$(timeout 8 ros2 topic hz /odom 2>&1 | grep -o 'average rate: [0-9.]*' | head -1)"
echo "/odom:    ${odom_hz:-!! NO /odom -- rm_safety will fail closed on odom_stale}"
echo "--- /safety/status ---"; timeout 4 ros2 topic echo /safety/status --once 2>&1 | grep -E 'state|reason'
echo "======================================="
echo "state 3 = KILLED (expected). Nothing can move until /safety/reset."
