#!/usr/bin/env bash
# ONE COMMAND on the new wall computer (Pop!_OS). Two ways to get it there:
#
#   from GitHub (the computer has internet at this point):
#     git clone -b feat/dynabarn-profile git@github.com:lumen-research-labs/ncp_ws.git ~/lumen/ncp_physical/ncp_ws
#     bash ~/lumen/ncp_physical/ncp_ws/scripts/robot_wall/setup_wall_computer.sh
#
#   from a USB stick written by make_bundle.sh (no internet needed):
#     bash /media/$USER/<stick>/robot-wall/setup.sh
#
# Options: --wired (a cable into the RobotLab router instead of its WiFi),
# --address <ip> (default 192.168.4.251), --psk <key> (the RobotLab WiFi key;
# otherwise the bundle's robotlab.psk, otherwise asked), --no-network.
#
# It does, in order, each step reporting:
#   1. the repository: from the stick, a clone to ~/lumen/ncp_physical/ncp_ws;
#      from a clone, nothing to do
#   2. chrony, the time server the robots take their clock from: from the
#      stick's .deb, or `apt install` while the internet is still there
#      (this is why it comes BEFORE the network step)
#   3. joins the robot network: a NetworkManager connection "RobotLab" with
#      a FIXED address; one WiFi radio means the internet goes away here,
#      which is right for a dedicated wall
#   4. installs the wall (scripts/robot_wall/install.sh): page, launcher,
#      autostart entry
#   5. turns on automatic login where it can (GDM); otherwise names the
#      Settings toggle; names the screen-blanking setting
# Then: reboot, and the wall is on the screen.
#
# Failure mode: a step that cannot complete says so and the script stops;
# every step is safe to re-run.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADDRESS="192.168.4.251"
WIRED=no
NETWORK=yes
PSK=""
while [ $# -gt 0 ]; do
  case "$1" in
    --wired) WIRED=yes; shift ;;
    --address) ADDRESS="$2"; shift 2 ;;
    --psk) PSK="$2"; shift 2 ;;
    --no-network) NETWORK=no; shift ;;
    -h|--help) sed -n '2,34p' "$0"; exit 0 ;;
    *) echo "setup: unknown option $1" >&2; exit 1 ;;
  esac
done
say() { echo; echo "== $* =="; }

say "1. the repository"
DEST="$HOME/lumen/ncp_physical/ncp_ws"
if [ -d "$HERE/ncp_ws/.git" ]; then                      # the stick
  if [ -d "$DEST/.git" ]; then
    git -C "$DEST" fetch -q "$HERE/ncp_ws" && git -C "$DEST" merge -q --ff-only FETCH_HEAD && echo "   $DEST fast-forwarded from the stick"
  else
    mkdir -p "$(dirname "$DEST")"; git clone -q "$HERE/ncp_ws" "$DEST"
    echo "   cloned from the stick to $DEST"
  fi
elif [ -d "$HERE/../../.git" ]; then                     # run from a clone
  DEST="$(cd "$HERE/../.." && pwd)"
  echo "   running from the clone at $DEST ($(git -C "$DEST" rev-parse --abbrev-ref HEAD) @ $(git -C "$DEST" rev-parse --short HEAD))"
else
  echo "   neither a bundle (ncp_ws/ beside this script) nor a clone; see the header" >&2; exit 1
fi

say "2. chrony: this computer serves its clock to the robots"
if ! dpkg -s chrony >/dev/null 2>&1; then
  DEB="$(ls "$HERE"/deb/chrony_*.deb 2>/dev/null | head -1)"
  if [ -n "$DEB" ]; then
    if dpkg -s systemd-timesyncd >/dev/null 2>&1; then
      sudo dpkg --remove systemd-timesyncd >/dev/null && echo "   removed systemd-timesyncd (chrony replaces it)"
    fi
    sudo dpkg -i "$DEB" >/dev/null && echo "   installed $(basename "$DEB") from the stick"
  else
    echo "   apt install chrony (needs the internet, which is why this comes before the network step)"
    sudo apt-get install -y chrony >/dev/null || { echo "   apt failed: no internet? run again while online, or use a stick bundle" >&2; exit 1; }
    echo "   installed chrony"
  fi
else
  echo "   chrony already installed"
fi
sudo mkdir -p /etc/chrony/conf.d
printf '# ncp_ws robot wall: serve this clock to the robots (no internet needed)\nallow 192.168.4.0/24\nlocal stratum 10\n' \
  | sudo tee /etc/chrony/conf.d/robot-wall.conf >/dev/null
sudo systemctl enable --now chrony >/dev/null 2>&1 || sudo systemctl enable --now chronyd >/dev/null
sudo systemctl restart chrony 2>/dev/null || sudo systemctl restart chronyd
echo "   chrony serving 192.168.4.0/24 (local stratum 10)"

if [ "$NETWORK" = yes ]; then
  say "3. the robot network ($ADDRESS/24 on RobotLab)"
  if [ "$WIRED" = yes ]; then
    IF="$(nmcli -t -f DEVICE,TYPE dev status | awk -F: '$2=="ethernet"{print $1; exit}')"
    [ -n "$IF" ] || { echo "   no Ethernet interface found" >&2; exit 1; }
    nmcli con delete RobotLab-wired >/dev/null 2>&1 || true
    nmcli con add type ethernet ifname "$IF" con-name RobotLab-wired \
      ipv4.method manual ipv4.addresses "$ADDRESS/24" ipv4.gateway 192.168.4.1 ipv4.dns 192.168.4.1 \
      connection.autoconnect yes connection.autoconnect-priority 100 >/dev/null
    nmcli con up RobotLab-wired >/dev/null && echo "   RobotLab-wired up on $IF at $ADDRESS"
  else
    [ -n "$PSK" ] || PSK="$(cat "$HERE/robotlab.psk" 2>/dev/null || true)"
    if [ -z "$PSK" ]; then
      read -r -s -p "   RobotLab WiFi password: " PSK; echo
      [ -n "$PSK" ] || { echo "   no key given; pass --psk <key> or use --wired" >&2; exit 1; }
    fi
    nmcli con delete RobotLab >/dev/null 2>&1 || true
    nmcli con add type wifi ifname "*" con-name RobotLab ssid RobotLab \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" \
      ipv4.method manual ipv4.addresses "$ADDRESS/24" ipv4.gateway 192.168.4.1 ipv4.dns 192.168.4.1 \
      connection.autoconnect yes connection.autoconnect-priority 100 >/dev/null
    nmcli con up RobotLab >/dev/null && echo "   RobotLab WiFi up at $ADDRESS"
  fi
  for r in 192.168.4.100 192.168.4.102; do
    printf "   %s: " "$r"; ping -c 1 -W 2 "$r" >/dev/null 2>&1 && echo "answers" || echo "no answer (off, or not on this network)"
  done
fi

say "4. the wall"
bash "$DEST/scripts/robot_wall/install.sh" | sed 's/^/   /'

say "5. boot straight into it"
if [ -f /etc/gdm3/custom.conf ]; then
  sudo sed -i -e '/^\[daemon\]/,/^\[/{/^AutomaticLogin/d}' /etc/gdm3/custom.conf
  sudo sed -i -e "s/^\[daemon\]/[daemon]\nAutomaticLoginEnable=true\nAutomaticLogin=$USER/" /etc/gdm3/custom.conf
  echo "   GDM automatic login enabled for $USER"
else
  echo "   not GDM: turn on Settings > Users > Automatic Login for $USER by hand"
fi
if command -v gsettings >/dev/null 2>&1 && gsettings writable org.gnome.desktop.session idle-delay >/dev/null 2>&1; then
  gsettings set org.gnome.desktop.session idle-delay 0
  gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null || true
  echo "   screen blanking off"
else
  echo "   set Settings > Power: screen off NEVER, suspend NEVER, by hand"
fi
echo
echo "Done. Reboot; the wall appears. To see it now: ~/.local/share/robot-wall/robot_wall.sh"
