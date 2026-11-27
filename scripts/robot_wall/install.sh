#!/usr/bin/env bash
# Install the robot wall on THIS computer so it opens at login.
#
#   bash scripts/robot_wall/install.sh [--systemd] [name=url ...]
#
# Default robots: bernard-1=http://192.168.4.100:8080 bernard-3=http://192.168.4.102:8080
#
# Copies wall.html and robot_wall.sh to ~/.local/share/robot-wall, writes
# robots.js from the name=url pairs, and registers an XDG autostart entry
# (~/.config/autostart/robot-wall.desktop), which GNOME and COSMIC run at
# login. --systemd writes a user service on graphical-session.target instead,
# for a session that ignores XDG autostart. Re-running replaces the files.
#
# What it does NOT do: turn on automatic login (Settings > Users on Pop!_OS;
# needed for the wall to appear with no one typing a password), or put this
# computer on the robot network (docs/robot_wall.md, first section).
#
# Failure mode: refuses a pair that is not name=http(s)://host[:port], names
# it, and installs nothing.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="${XDG_DATA_HOME:-$HOME/.local/share}/robot-wall"
MODE=autostart
PAIRS=()
for a in "$@"; do
  case "$a" in
    --systemd) MODE=systemd ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) PAIRS+=("$a") ;;
  esac
done
[ ${#PAIRS[@]} -gt 0 ] || PAIRS=(bernard-1=http://192.168.4.100:8080 bernard-3=http://192.168.4.102:8080)

for p in "${PAIRS[@]}"; do
  if ! [[ "$p" =~ ^[A-Za-z0-9._-]+=https?://[^/[:space:]]+$ ]]; then
    echo "install: expected name=http://host:port, got '$p'" >&2; exit 1
  fi
done

mkdir -p "$DEST"
cp "$HERE/wall.html" "$HERE/robot_wall.sh" "$DEST/"
chmod +x "$DEST/robot_wall.sh"
{
  echo "// written by scripts/robot_wall/install.sh $(date -u +%Y-%m-%dT%H:%M:%SZ); re-run it to change"
  echo "window.ROBOTS = ["
  for p in "${PAIRS[@]}"; do
    echo "  { name: \"${p%%=*}\", url: \"${p#*=}\" },"
  done
  echo "];"
} > "$DEST/robots.js"
echo "wall page: $DEST/wall.html"
echo "robots:    ${PAIRS[*]}"

if [ "$MODE" = autostart ]; then
  mkdir -p "$HOME/.config/autostart"
  cat > "$HOME/.config/autostart/robot-wall.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=Robot wall
Comment=Both robot dashboards, always open (ncp_ws scripts/robot_wall)
Exec=$DEST/robot_wall.sh
Terminal=false
X-GNOME-Autostart-enabled=true
DESKTOP
  echo "autostart: $HOME/.config/autostart/robot-wall.desktop (runs at login on GNOME and COSMIC)"
else
  mkdir -p "$HOME/.config/systemd/user"
  cat > "$HOME/.config/systemd/user/robot-wall.service" <<UNIT
[Unit]
Description=Robot wall: both robot dashboards in a kiosk browser
PartOf=graphical-session.target
After=graphical-session.target

[Service]
ExecStart=$DEST/robot_wall.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=graphical-session.target
UNIT
  systemctl --user daemon-reload
  systemctl --user enable robot-wall.service
  echo "systemd: user service robot-wall.service enabled on graphical-session.target"
fi
echo
echo "Next: Settings > Users > Automatic Login ON, so the wall appears at boot."
echo "Try it now: $DEST/robot_wall.sh"
