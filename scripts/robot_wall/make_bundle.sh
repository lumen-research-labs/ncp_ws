#!/usr/bin/env bash
# Build the robot-wall bundle for a USB stick, on THIS workstation:
#
#   bash scripts/robot_wall/make_bundle.sh <out dir>      e.g. /media/$USER/STICK/robot-wall
#
# Writes into <out dir>:
#   ncp_ws/          a git clone of this checkout's current branch
#   deb/chrony_*.deb the time server (apt-get download; deps are on a Pop!_OS desktop)
#   robotlab.psk     the RobotLab WiFi key, read from bernard-1 over SSH (mode 600;
#                    skipped with a note if the robot is unreachable: use --wired then)
#   setup.sh         the one command to run on the new computer
#   README.txt
# The new computer needs nothing else and no internet.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:?usage: make_bundle.sh <out dir>}"
mkdir -p "$OUT/deb"
BRANCH="$(git -C "$REPO" rev-parse --abbrev-ref HEAD)"
echo "== repository ($BRANCH) =="
rm -rf "$OUT/ncp_ws"
git clone -q --branch "$BRANCH" "$REPO" "$OUT/ncp_ws"
echo "   $(git -C "$OUT/ncp_ws" log --oneline -1)"
echo "== chrony .deb =="
( cd "$OUT/deb" && apt-get download chrony >/dev/null 2>&1 ) && ls "$OUT"/deb/chrony_*.deb | sed 's/^/   /' \
  || echo "   apt-get download chrony failed (no internet?); the new computer will need apt"
echo "== RobotLab WiFi key from bernard-1 =="
if PSK="$(ssh -o ConnectTimeout=6 -o BatchMode=yes bernard-1 'sudo -n nmcli -s -g 802-11-wireless-security.psk con show RobotLab' 2>/dev/null)" && [ -n "$PSK" ]; then
  ( umask 077; printf '%s' "$PSK" > "$OUT/robotlab.psk" ); echo "   written (robotlab.psk, mode 600)"
else
  echo "   bernard-1 unreachable or no key: not written; setup.sh will need --wired"
fi
cp "$HERE/setup_wall_computer.sh" "$OUT/setup.sh"; chmod +x "$OUT/setup.sh"
cat > "$OUT/README.txt" <<TXT
Robot wall: the second computer that always shows both robot dashboards.

On the new Pop!_OS computer, plug this stick in and run:

    bash /media/\$USER/<stick>/robot-wall/setup.sh

It joins the RobotLab WiFi at 192.168.4.251 (add --wired for a cable into
the RobotLab router, --address to pick another fixed address), installs
chrony as the robots' time server, installs the wall and its autostart
entry, and turns on automatic login where it can. Then reboot.

Docs: ncp_ws/docs/robot_wall.md and ncp_ws/docs/robot_boot.md.
Built $(date -u +%Y-%m-%dT%H:%M:%SZ) from $BRANCH @ $(git -C "$OUT/ncp_ws" rev-parse --short HEAD).
TXT
echo "== bundle at $OUT =="; du -sh "$OUT" | cut -f1 | sed 's/^/   /'
