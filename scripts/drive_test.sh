#!/usr/bin/env bash
# Timed, self-stopping velocity command for the wheels-down calibration runs.
#
# Runs ON the robot. OPERATOR-ONLY: this is the script that makes the robot
# move. There is no physical E-stop -- do not start it without a kill
# terminal open (see the banner it prints).
#
# Why this exists rather than a bare `ros2 topic pub`:
#   - `ros2 topic pub --once ... "{}"` DOES NOT STOP THE ROBOT. A single
#     message is routinely dropped or sent before the subscription matches.
#     Only continuous zeros stop it, so this script floods zeros on EVERY
#     exit path -- normal end, Ctrl-C, or error -- via a trap.
#   - a fixed duration makes the run measurable: commanded distance is
#     vx * seconds, which is exactly what you compare the tape measure to.
#
# It publishes to /policy/cmd_vel, an INPUT to rm_safety -- never to
# /cmd_vel, which rm_safety alone owns. So every clamp and e-stop arc still
# applies, and /safety/kill still stops it.
#
# Usage:
#   drive_test.sh straight <vx m/s> <seconds>     # e.g. straight 0.10 5
#   drive_test.sh spin     <wz rad/s> <seconds>   # e.g. spin 0.30 5
#
# Expected result printed at the end: compare it against a tape measure
# (straight) or a protractor / marked floor angle (spin).
# NOTE: no `set -u` -- ROS's setup.bash chain reads unbound variables.
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/jetson/ncp_ws/install/setup.bash
# The robot's domain, from its profile.
source "$(dirname "$0")/robot_env.sh"

RATE=20
TOPIC=/policy/cmd_vel

usage() { sed -n '25,29p' "$0"; exit 1; }
[ $# -eq 3 ] || usage

KIND="$1"; VAL="$2"; SECS="$3"

case "$KIND" in
  straight) VX="$VAL"; WZ=0.0 ;;
  spin)     VX=0.0;    WZ="$VAL" ;;
  *) usage ;;
esac

PUB_PID=""
stop_everything() {
  # Kill the command source FIRST, then flood zeros. Killing the publisher
  # alone is not enough: rm_base's watchdog would zero the PWM, but flooding
  # is the belt-and-braces path that does not depend on any watchdog.
  [ -n "$PUB_PID" ] && kill "$PUB_PID" 2>/dev/null
  wait "$PUB_PID" 2>/dev/null
  echo
  echo "== flooding zeros for 3 s =="
  timeout 3 ros2 topic pub -r "$RATE" "$TOPIC" geometry_msgs/msg/Twist "{}" \
    > /dev/null 2>&1
  echo "== stopped =="
}
trap stop_everything EXIT INT TERM

cat <<BANNER

  ###############################################################
  #  THE ROBOT IS ABOUT TO MOVE.                                #
  #  There is no physical E-stop. Have this ready in another    #
  #  terminal, typed, before you continue:                      #
  #                                                             #
  #     ros2 service call /safety/kill std_srvs/srv/Trigger     #
  #                                                             #
  ###############################################################

  ${KIND}:  vx=${VX}  wz=${WZ}   for ${SECS}s at ${RATE} Hz -> ${TOPIC}
BANNER
read -r -p "  Enter to go, Ctrl-C to abort: " _

echo "== driving =="
ros2 topic pub -r "$RATE" "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: $VX, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $WZ}}" \
  > /dev/null 2>&1 &
PUB_PID=$!

sleep "$SECS"

# Trap does the stopping; print the expectation for the tape measure.
echo
if [ "$KIND" = straight ]; then
  echo "COMMANDED: ${VX} m/s x ${SECS} s = $(python3 -c "print(f'{$VX*$SECS:.3f}')") m"
  echo "Measure the actual distance travelled and compare."
  echo "NOTE: rm_safety scales vx to zero in the slow/stop zone -- check"
  echo "      /safety/status reason was 'ok' for the whole run, or the"
  echo "      robot was braking for an obstacle, not under-driving."
else
  echo "COMMANDED: ${WZ} rad/s x ${SECS} s = $(python3 -c "import math; print(f'{math.degrees($WZ*$SECS):.1f}')") deg"
  echo "Measure the actual yaw and compare."
fi
