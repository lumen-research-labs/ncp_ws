#!/usr/bin/env bash
# Host-level cleanup that must happen before any rm_* node starts.
#
# Runs ON the robot. A launch file cannot do any of this -- it is killing
# processes and creating a root-owned directory -- so it is a separate step
# rather than something ncp.launch.py could absorb.
#
# Safe and idempotent: it starts nothing and only removes things that
# CONFLICT with our stack. Run it as often as you like.
#
# What it clears and why:
#   Mcnamu_driver_X3      the vendor base driver. The base-board serial port
#                         is single-writer, so rm_base cannot open it until
#                         this is gone. Bringup does not respawn it.
#   rosmaster_main.py     a Yahboom factory autostart that grabs the same
#                         port. RECURS EVERY BOOT, one to two minutes AFTER
#                         it. If it holds the port when bringup starts, the
#                         driver's serial thread crashes and /voltage,
#                         /imu/data_raw and /vel_raw all read ZERO while the
#                         topics keep publishing normally -- which looks like
#                         dead hardware, not a conflict. On a fresh boot this
#                         script WAITS for it to appear before killing it
#                         (AUTOSTART_WINDOW_S, default 180 s of uptime).
#   yahboom_joy_X3        publishes /cmd_vel DIRECTLY, which only rm_safety
#                         may do. Use teleop_joy.launch.py instead, which
#                         runs the same node remapped to /teleop/cmd_vel so
#                         the gamepad becomes an INPUT to rm_safety.
#   /var/lib/rm_safety    rm_safety persists its kill state here. Without it
#                         the node warns and degrades to memory-only on every
#                         kill/reset. Recurs on any reflash or new robot.
#
# Note every pattern is bracketed ('[M]cnamu') so it cannot match the shell
# running this script -- an unbracketed pkill -f over ssh kills your own
# session.
#
# Failure mode: reports what it could not clear and exits non-zero, rather
# than letting a later bringup fail with a confusing symptom.
set -o pipefail

# THE AUTOSTART RACE. rosmaster_main.py is started by Yahboom's desktop
# autostart ONE TO TWO MINUTES after boot (seen at 43 s and at 46 s, 2026-11).
# Prepping before it appears is worse than useless: it then starts AFTER
# rm_base has the port, takes the port, and every base value reads zero.
# So after a fresh boot, wait for it to show up and THEN kill it. The window
# is bounded: past AUTOSTART_WINDOW_S of uptime it is not coming.
AUTOSTART_WINDOW_S=${AUTOSTART_WINDOW_S:-180}
uptime_s() { cut -d. -f1 /proc/uptime; }
if ! pgrep -f '[r]osmaster_main.py' >/dev/null && [ "$(uptime_s)" -lt "$AUTOSTART_WINDOW_S" ]; then
  echo "== fresh boot ($(uptime_s) s up): waiting for the factory autostart to appear so it can be killed =="
  while ! pgrep -f '[r]osmaster_main.py' >/dev/null && [ "$(uptime_s)" -lt "$AUTOSTART_WINDOW_S" ]; do
    sleep 2
  done
  if pgrep -f '[r]osmaster_main.py' >/dev/null; then
    echo "   autostart appeared at $(uptime_s) s"
  else
    echo "   no autostart by $(uptime_s) s; it is not coming this boot"
  fi
fi

echo "== clearing conflicting processes =="
for pat in '[M]cnamu_driver_X3' '[r]osmaster_main.py' '[y]ahboom_joy_X3'; do
  pids="$(pgrep -f "$pat")"
  if [ -n "$pids" ]; then
    echo "   killing $pat -> $pids"
    # By PID, never `pkill -f`, so we kill exactly what we looked at.
    echo "$pids" | xargs -r kill
  else
    echo "   $pat: not running"
  fi
done
sleep 2

echo "== kill-state directory =="
if [ -d /var/lib/rm_safety ] && [ -w /var/lib/rm_safety ]; then
  echo "   /var/lib/rm_safety ok"
else
  sudo install -d -o "$(id -un)" -g "$(id -gn)" /var/lib/rm_safety \
    && echo "   created /var/lib/rm_safety"
fi

echo "== checks =="
rc=0
if fuser /dev/ttyUSB1 >/dev/null 2>&1; then
  echo "   !! base-board port STILL HELD by:"
  fuser -v /dev/ttyUSB1 2>&1 | sed 's/^/      /'
  rc=1
else
  echo "   base-board port free"
fi

# Ground truth is the chip, not the symlink: CP2102 (10c4:ea60) is the lidar,
# CH340 (1a86:7523) is the base board. /dev/rplidar points at whichever won
# the enumeration race this boot and has been wrong before.
for dev in usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
           usb-1a86_USB_Serial-if00-port0; do
  if [ -e "/dev/serial/by-id/$dev" ]; then
    echo "   found $(basename "$dev" | cut -c1-28)... -> $(readlink -f "/dev/serial/by-id/$dev")"
  else
    echo "   !! MISSING /dev/serial/by-id/$dev"
    rc=1
  fi
done

[ $rc -eq 0 ] && echo "== ready ==" || echo "== NOT READY, see above =="
exit $rc
