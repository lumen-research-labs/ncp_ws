#!/usr/bin/env bash
# Open the robot wall in a kiosk browser. Started at login by the autostart
# entry install.sh writes; also fine to run by hand.
#
# Picks the first browser present: chromium, google-chrome, firefox (Pop!_OS
# ships firefox). Each gets its own kiosk flag. Logs to
# ~/.local/share/robot-wall/launch.log so a wall that did not appear can be
# explained after the fact; exits non-zero naming what it looked for if no
# browser exists, so the failure is loud rather than a blank screen.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PAGE="${ROBOT_WALL_PAGE:-$HERE/wall.html}"
LOGDIR="${XDG_DATA_HOME:-$HOME/.local/share}/robot-wall"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/launch.log"
log() { echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) $*" | tee -a "$LOG" >&2; }

[ -f "$PAGE" ] || { log "robot_wall: page not found: $PAGE"; exit 1; }
URL="file://$PAGE"

for b in chromium chromium-browser google-chrome google-chrome-stable; do
  if command -v "$b" >/dev/null 2>&1; then
    log "robot_wall: $b --kiosk $URL"
    exec "$b" --kiosk --noerrdialogs --disable-session-crashed-bubble --disable-infobars "$URL"
  fi
done
if command -v firefox >/dev/null 2>&1; then
  log "robot_wall: firefox --kiosk $URL"
  exec firefox --kiosk "$URL"
fi
if command -v flatpak >/dev/null 2>&1 && flatpak info org.mozilla.firefox >/dev/null 2>&1; then
  log "robot_wall: flatpak firefox --kiosk $URL"
  exec flatpak run org.mozilla.firefox --kiosk "$URL"
fi
log "robot_wall: no browser found (looked for chromium, chromium-browser, google-chrome, google-chrome-stable, firefox, flatpak org.mozilla.firefox)"
exit 1
