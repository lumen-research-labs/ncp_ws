#!/usr/bin/env bash
# rm_base wheels-up bench test: lidar + rm_base(motion) + rm_safety.
#
# Runs ON the robot (paths are bernard-1's). Copy over with deploy_to_robot.sh
# and run from an ssh shell. Scaffolding for the rm_base bringup / calibration
# sessions -- to be folded into a single committed bringup script once the
# per-session sequence stops changing (BUILD-PLAN "next session" item 7).
#
# Nothing moves until you run /safety/reset AND publish /policy/cmd_vel.
# Tear down with bench_down.sh (restores the vendor Mcnamu_driver_X3).
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /home/jetson/ncp_ws/install/setup.bash
export ROS_DOMAIN_ID=28

LIDAR=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
BASE=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
CAL=/home/jetson/ncp_ws/config/base_calibration.yaml

echo "== clearing stale processes =="
[ -f /tmp/rm_base.pid ] && kill "$(cat /tmp/rm_base.pid)" 2>/dev/null
pkill -f '[r]m_base_node'
pkill -f '[y]dlidar_ros2_driver_node'
pkill -f '[s]afety_node'
pgrep -f '[M]cnamu_driver_X3' | xargs -r kill
if fuser /dev/ttyUSB1 2>/dev/null | grep -q .; then
  pgrep -f '[r]osmaster_main.py' | xargs -r kill
fi
sleep 3

echo "== lidar =="
setsid sh -c "ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  -p port:=$LIDAR -p baudrate:=512000 -p lidar_type:=0 -p device_type:=0 \
  -p sample_rate:=20 -p isSingleChannel:=false -p support_motor_dtr:=true \
  -p reversion:=true -p inverted:=true -p range_max:=64.0 -p range_min:=0.01 \
  -p frequency:=10.0 -p frame_id:=laser_link > /tmp/lidar.log 2>&1 &" < /dev/null

echo "== rm_base  dry_run:=false =="
setsid sh -c "ros2 run rm_base rm_base_node --ros-args -p dry_run:=false \
  -p calibration_path:=$CAL -p serial_port:=$BASE \
  > /tmp/rm_base.log 2>&1 & echo \$! > /tmp/rm_base.pid" < /dev/null

echo "== rm_safety  command_source:=policy  (starts KILLED) =="
setsid sh -c "ros2 launch rm_bringup safety.launch.py command_source:=policy \
  > /tmp/safety.log 2>&1 &" < /dev/null

sleep 10
echo
echo "================ CHECKS ================"
ros2 node list 2>/dev/null | grep -E 'rm_base|safety|ydlidar' || echo "!! NODES MISSING"
echo -n "/scan:    "; timeout 6 ros2 topic hz /scan 2>&1 | grep -o 'average rate: [0-9.]*' | head -1 || echo "!! NO /scan"
echo -n "/voltage: "; timeout 4 ros2 topic echo /voltage --once 2>&1 | grep data
echo -n "dry_run:  "; ros2 param get /rm_base_node dry_run 2>&1
echo "--- /safety/status ---"; timeout 4 ros2 topic echo /safety/status --once 2>&1 | grep -E 'state|reason'
echo "======================================="
echo "state 3 = KILLED (expected). Next: /safety/reset, then publish /policy/cmd_vel."
