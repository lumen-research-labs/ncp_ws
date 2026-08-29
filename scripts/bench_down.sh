#!/usr/bin/env bash
# Tear down the rm_base bench test, restore the vendor base driver.
# Runs ON the robot (paths are bernard-1's). Pairs with bench_up.sh.
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=28

echo "== stopping test nodes =="
pkill -f 'topic pub .*policy/cmd_vel'
[ -f /tmp/rm_base.pid ] && kill "$(cat /tmp/rm_base.pid)" 2>/dev/null
pkill -f '[r]m_base_node'
pkill -f '[s]afety_node'
pkill -f '[y]dlidar_ros2_driver_node'
# odom chain started by odom.launch.py
pkill -f '[b]ase_node_X3'
pkill -f '[i]mu_filter_madgwick_node'
pkill -f '[e]kf_node'
pkill -f '[r]obot_state_publisher'
pkill -f '[j]oint_state_publisher'
sleep 3
fuser -v /dev/ttyUSB1 2>&1 | grep -q ttyUSB1 && echo "port still held" || echo "base board port free"

echo "== restoring vendor Mcnamu_driver_X3 =="
setsid sh -c "ros2 run yahboomcar_bringup Mcnamu_driver_X3 > /tmp/vendor_drv.log 2>&1 &" < /dev/null
sleep 5
timeout 4 ros2 topic echo /voltage --once 2>&1 | grep data
echo "done."
