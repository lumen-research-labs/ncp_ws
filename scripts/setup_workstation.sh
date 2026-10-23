#!/usr/bin/env bash
# Set up a fresh workstation for ncp_ws: the apt packages, the upstream
# NCP-on-Rosmaster checkout beside this repo, the test suite, the workstation
# build. Idempotent: run it again after a pull. Never touches the robot.
#
# Verified on Pop!_OS 24.04 with ROS 2 Jazzy (docs/workstation_setup.md). The
# robot stack itself runs on the Jetson; this machine only edits, tests,
# deploys and opens the dashboard.
#
#   bash scripts/setup_workstation.sh            # install, clone, test, build
#   bash scripts/setup_workstation.sh --check    # report only, change nothing
#
# Failure mode: stops at the first step that fails and names it; --check
# exits non-zero if anything is missing. ROS itself is not installed here
# (it is a distro-level choice): the script says what to install and stops.
set -eo pipefail   # no -u: ROS's setup.bash is not -u safe

CHECK=0
[[ "${1:-}" == "--check" ]] && CHECK=1
REPO="$(cd "$(dirname "$0")/.." && pwd)"
UPSTREAM="$(dirname "$REPO")/NCP-on-Rosmaster"
UPSTREAM_URL="git@github.com:lumen-research-labs/NCP-on-Rosmaster.git"
UPSTREAM_BRANCH="feat/720-beam-270fov"   # carries the trained weights
PKGS=(python3-pytest python3-numpy python3-yaml python3-colcon-common-extensions)
missing=0

say() { printf '==> %s\n' "$*"; }
need() { printf '   MISSING: %s\n' "$*"; missing=1; }

say "ROS 2"
ROS_SETUP=""
for d in jazzy humble; do
  [[ -f "/opt/ros/$d/setup.bash" ]] && { ROS_SETUP="/opt/ros/$d/setup.bash"; break; }
done
if [[ -z "$ROS_SETUP" ]]; then
  need "no ROS 2 under /opt/ros. Install ros-jazzy-ros-base (Ubuntu 24.04): https://docs.ros.org/en/jazzy/Installation.html"
else
  say "   found $ROS_SETUP"
fi

say "apt packages: ${PKGS[*]}"
absent=()
for p in "${PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || absent+=("$p"); done
if (( ${#absent[@]} )); then
  if (( CHECK )); then need "apt packages: ${absent[*]}"; else sudo apt-get install -y "${absent[@]}"; fi
else
  say "   all present"
fi

say "upstream checkout at $UPSTREAM ($UPSTREAM_BRANCH)"
if [[ ! -d "$UPSTREAM/.git" ]]; then
  if (( CHECK )); then need "$UPSTREAM (git clone $UPSTREAM_URL)"; else
    git clone "$UPSTREAM_URL" "$UPSTREAM" && git -C "$UPSTREAM" checkout "$UPSTREAM_BRANCH"
  fi
else
  say "   present, on $(git -C "$UPSTREAM" rev-parse --abbrev-ref HEAD)"
fi
[[ -f "$UPSTREAM/weights/actor.pt" ]] || need "$UPSTREAM/weights/actor.pt (checkout $UPSTREAM_BRANCH)"

say "robot access"
grep -q "^Host bernard-1" ~/.ssh/config 2>/dev/null || need "~/.ssh/config entry for bernard-1 (docs/workstation_setup.md, step 5)"
[[ -f ~/.ssh/lumen ]] || need "~/.ssh/lumen, the robot key (from Daniel)"

if (( missing )); then
  printf '\nSomething is missing (above). Fix it and run again.\n'
  exit 1
fi
(( CHECK )) && { say "everything is in place"; exit 0; }

say "tests"
( cd "$REPO" && python3 -m pytest tests/ -q -p no:cacheprovider | tail -1 )

say "workstation build (rm_interfaces)"
( cd "$REPO" && source "$ROS_SETUP" && colcon build --symlink-install --packages-select rm_interfaces 2>&1 | tail -1 )

say "done. Next: plug the lab cable, 'ssh bernard-1 echo up', then docs/operating_guide.md."
