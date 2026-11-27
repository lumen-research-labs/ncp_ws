#!/usr/bin/env bash
# The robot's own bring-up, run by ncp-stack.service at boot (also fine by
# hand). Sources ROS, waits for the serial devices and the clock, clears
# the factory autostart, launches the stack from the robot profile.
#
# Order and why:
#   1. the two serial by-id paths (lidar CP2102, base board CH340) must exist:
#      the lidar driver exits on a missing port and rm_base owns the base
#      port; on bernard-1 the lidar needs its second (power) cable, so a
#      missing path is reported by name.
#   2. the clock: systemd-timesyncd (pointed at the wall computer by
#      install.sh --time-server) is given TIME_SYNC_WAIT_S to report a sync,
#      so the stack starts on the right time instead of jumping under a
#      running stack later. The stack does not need the date; a wall
#      computer that is off costs only the wait.
#   3. prep_robot.sh: kills whatever still holds the ports (the factory
#      autostart, if install.sh did not disable it), creates the kill-state dir.
#   4. run_stack.sh with the zones on and the waypoint controller, today's
#      hand launch. exec, so systemd supervises the launch itself.
# Logs to ~/stack_boot.log (appended) with the launch output.
set -o pipefail
LOG="${HOME:-/home/jetson}/stack_boot.log"
exec >>"$LOG" 2>&1
echo "== boot_stack $(date -u +%Y-%m-%dT%H:%M:%SZ) uptime $(cut -d. -f1 /proc/uptime)s =="

source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source /home/jetson/ncp_ws/install/setup.bash
cd /home/jetson/ncp_ws || exit 1

LIDAR=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
BASE=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
DEVICE_WAIT_S=${DEVICE_WAIT_S:-60}
for _ in $(seq "$DEVICE_WAIT_S"); do
  [ -e "$LIDAR" ] && [ -e "$BASE" ] && break
  sleep 1
done
for dev in "$LIDAR" "$BASE"; do
  if [ ! -e "$dev" ]; then
    echo "boot_stack: REFUSING to launch: $dev is not present after ${DEVICE_WAIT_S}s (lidar: both cables? base board: USB?)"
    exit 1
  fi
done

TIME_SYNC_WAIT_S=${TIME_SYNC_WAIT_S:-30}
if command -v timedatectl >/dev/null 2>&1; then
  for _ in $(seq "$TIME_SYNC_WAIT_S"); do
    [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = yes ] && break
    sleep 1
  done
  echo "boot_stack: clock $(date -u +%Y-%m-%dT%H:%M:%SZ), NTPSynchronized=$(timedatectl show -p NTPSynchronized --value 2>/dev/null)"
fi

bash scripts/prep_robot.sh || { echo "boot_stack: prep_robot.sh NOT READY, not launching"; exit 1; }
exec bash scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true with_waypoint:=true
