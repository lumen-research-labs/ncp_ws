#!/usr/bin/env bash
# Install the boot service ON THE ROBOT (run there, once, as jetson with sudo).
#
#   bash scripts/robot_boot/install.sh [--time-server <ip>] [--no-disable-autostart]
#
# 1. Installs and enables ncp-stack.service (system unit, user jetson).
# 2. Disables Yahboom's factory autostart (rosmaster_main.py), the one thing
#    that still races our stack for the base-board serial port: every file
#    under the XDG autostart dirs that starts it is renamed to .disabled
#    beside the original (never deleted; rename it back to re-enable), and a
#    systemd unit starting it is disabled. Anything else that starts it
#    (rc.local, a crontab) is NAMED and left for the operator.
# 3. With --time-server, points systemd-timesyncd at that address (the wall
#    computer running chrony) through a drop-in, so the robot takes the
#    computer's time at boot; boot_stack.sh waits for the sync.
# Nothing is started here: `sudo systemctl start ncp-stack` or a reboot does.
#
# Failure mode: every step reports; a step that cannot complete exits non-zero
# with the reason and the rest is not attempted.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIME_SERVER=""
DISABLE_AUTOSTART=yes
while [ $# -gt 0 ]; do
  case "$1" in
    --time-server) TIME_SERVER="$2"; shift 2 ;;
    --no-disable-autostart) DISABLE_AUTOSTART=no; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "install: unknown option $1" >&2; exit 1 ;;
  esac
done
[ "$(id -un)" = jetson ] || { echo "install: run as jetson on the robot" >&2; exit 1; }
sudo -n true 2>/dev/null || { echo "install: needs passwordless sudo for jetson" >&2; exit 1; }

echo "== ncp-stack.service =="
sudo install -m 0644 "$HERE/ncp-stack.service" /etc/systemd/system/ncp-stack.service
sudo systemctl daemon-reload
sudo systemctl enable ncp-stack.service
echo "   enabled (starts at the next boot; 'sudo systemctl start ncp-stack' starts it now)"

if [ "$DISABLE_AUTOSTART" = yes ]; then
  echo "== factory autostart (rosmaster_main.py) =="
  found=0
  for d in "$HOME/.config/autostart" /etc/xdg/autostart; do
    [ -d "$d" ] || continue
    while IFS= read -r -d '' f; do
      found=1
      if [ -w "$f" ]; then mv "$f" "$f.disabled"; else sudo mv "$f" "$f.disabled"; fi
      echo "   disabled $f -> $f.disabled"
    done < <(grep -lZ "rosmaster_main" "$d"/*.desktop 2>/dev/null || true)
  done
  for u in $(grep -ls "rosmaster_main" /etc/systemd/system/*.service /lib/systemd/system/*.service 2>/dev/null || true); do
    found=1; name="$(basename "$u")"
    sudo systemctl disable --now "$name" && echo "   disabled unit $name"
  done
  for other in /etc/rc.local "$HOME/.bashrc" "$HOME/.profile"; do
    if [ -f "$other" ] && grep -qs "rosmaster_main" "$other"; then
      echo "   NOTE: $other also starts rosmaster_main.py; edit it by hand"
    fi
  done
  if (crontab -l 2>/dev/null || true) | grep -qs rosmaster_main; then echo "   NOTE: jetson's crontab starts rosmaster_main.py; edit it by hand"; fi
  [ $found = 1 ] || echo "   nothing found that starts rosmaster_main.py (prep_robot.sh still kills it if it appears)"
fi

if [ -n "$TIME_SERVER" ]; then
  echo "== clock from $TIME_SERVER (systemd-timesyncd) =="
  sudo mkdir -p /etc/systemd/timesyncd.conf.d
  printf '# written by ncp_ws scripts/robot_boot/install.sh: the wall computer serves time\n[Time]\nNTP=%s\n' "$TIME_SERVER" \
    | sudo tee /etc/systemd/timesyncd.conf.d/robot-wall.conf >/dev/null
  sudo timedatectl set-ntp true
  sudo systemctl restart systemd-timesyncd
  echo "   timesyncd pointed at $TIME_SERVER; 'timedatectl' shows the sync state"
fi
echo "== done =="
