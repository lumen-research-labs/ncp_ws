#!/usr/bin/env bash
# Open the robot wall in a kiosk browser. Started at login by the autostart
# entry install.sh writes; also fine to run by hand.
#
# Serves the page from a local web server (python3 -m http.server on
# 127.0.0.1:ROBOT_WALL_PORT, default 8099, started once and left running)
# and opens http://127.0.0.1:8099/wall.html: a page from localhost may fetch
# and frame the robots' http dashboards in every browser, where a file://
# page is subject to each browser's own rules. Without python3 it falls
# back to file://.
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
# The server outlives this script on purpose. `firefox URL` with Firefox
# already running hands the URL to that instance and EXITS AT ONCE; a server
# stopped on exit then vanished under the page it had just been asked for
# ("Problem loading page", 2026-11-26). One server per port, reused.
serving() { curl -s -o /dev/null --max-time 1 "http://127.0.0.1:$PORT/$(basename "$PAGE")" 2>/dev/null; }
if command -v python3 >/dev/null 2>&1; then
  if ! serving; then
    setsid nohup python3 -m http.server --bind 127.0.0.1 --directory "$(dirname "$PAGE")" "$PORT" >>"$LOG" 2>&1 < /dev/null &
    for _ in $(seq 50); do serving && break; sleep 0.1; done
  fi
  if serving; then
    URL="http://127.0.0.1:$PORT/$(basename "$PAGE")"
    log "robot_wall: serving $(dirname "$PAGE") on $URL"
  else
    log "robot_wall: local server did not answer on port $PORT (busy with something else?), falling back to $URL"
  fi
fi
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
