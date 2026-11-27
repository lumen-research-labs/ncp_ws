#!/usr/bin/env bash
# Open the robot wall in a kiosk browser. Started at login by the autostart
# entry install.sh writes; also fine to run by hand.
#
# Serves the page from a local web server (python3 -m http.server on
# 127.0.0.1:ROBOT_WALL_PORT, default 8099) and opens http://127.0.0.1:8099/
# wall.html: a page from localhost may fetch and frame the robots' http
# dashboards in every browser, where a file:// page is subject to each
# browser's own rules. Without python3 it falls back to file://.
#
# Picks the first browser present: chromium, google-chrome, firefox (Pop!_OS
# ships firefox). Each gets its own kiosk flag. Logs to
# ~/.local/share/robot-wall/launch.log so a wall that did not appear can be
# explained after the fact; exits non-zero naming what it looked for if no
# browser exists, so the failure is loud rather than a blank screen.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PAGE="${ROBOT_WALL_PAGE:-$HERE/wall.html}"
PORT="${ROBOT_WALL_PORT:-8099}"
LOGDIR="${XDG_DATA_HOME:-$HOME/.local/share}/robot-wall"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/launch.log"
log() { echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) $*" | tee -a "$LOG" >&2; }

[ -f "$PAGE" ] || { log "robot_wall: page not found: $PAGE"; exit 1; }
URL="file://$PAGE"
if command -v python3 >/dev/null 2>&1; then
  python3 -m http.server --bind 127.0.0.1 --directory "$(dirname "$PAGE")" "$PORT" >>"$LOG" 2>&1 &
  SERVER=$!
  trap 'kill $SERVER 2>/dev/null' EXIT
  sleep 0.5
  if kill -0 "$SERVER" 2>/dev/null; then
    URL="http://127.0.0.1:$PORT/$(basename "$PAGE")"
    log "robot_wall: serving $(dirname "$PAGE") on $URL"
  else
    log "robot_wall: local server failed to start (port $PORT busy?), falling back to $URL"
  fi
fi
# The browser runs in the foreground so the trap above stops the server when
# it exits; `exec` would drop the trap.
run() { log "robot_wall: $*"; "$@"; exit $?; }

for b in chromium chromium-browser google-chrome google-chrome-stable; do
  if command -v "$b" >/dev/null 2>&1; then
    run "$b" --kiosk --noerrdialogs --disable-session-crashed-bubble --disable-infobars "$URL"
  fi
done
if command -v firefox >/dev/null 2>&1; then
  run firefox --kiosk "$URL"
fi
if command -v flatpak >/dev/null 2>&1 && flatpak info org.mozilla.firefox >/dev/null 2>&1; then
  run flatpak run org.mozilla.firefox --kiosk "$URL"
fi
log "robot_wall: no browser found (looked for chromium, chromium-browser, google-chrome, google-chrome-stable, firefox, flatpak org.mozilla.firefox)"
exit 1
