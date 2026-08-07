#!/usr/bin/env bash
# Workstation -> robot: push, pull, build.
#
# Code is NEVER edited on the robot. This script is the only sanctioned path
# for getting changes onto it.
#
# Failure mode: exits non-zero on any step; never leaves a partial build
# silently in place.
set -euo pipefail

# TODO(operator): fill in once the robot's address is established (Phase 1
# task 1) and recorded in docs/deployment.md.
ROBOT_HOST="${ROBOT_HOST:-}"
ROBOT_USER="${ROBOT_USER:-}"
ROBOT_WS="${ROBOT_WS:-~/ncp_ws}"

ROBOT_PACKAGES="rm_interfaces rm_safety rm_policy rm_bringup"

if [[ -z "$ROBOT_HOST" || -z "$ROBOT_USER" ]]; then
  echo "ERROR: ROBOT_HOST/ROBOT_USER not set. See docs/deployment.md §1." >&2
  exit 1
fi

if [[ -n "$(git status --porcelain)" ]]; then
  echo "ERROR: working tree is dirty. Commit before deploying — the robot must" >&2
  echo "       run a state that exists in git, not a local-only one." >&2
  exit 1
fi

git push

ssh "${ROBOT_USER}@${ROBOT_HOST}" bash -s <<REMOTE
set -euo pipefail
cd ${ROBOT_WS}
git pull --ff-only
cd docker
docker compose exec -T robot colcon build --symlink-install \
  --packages-select ${ROBOT_PACKAGES}
REMOTE

echo "Deployed $(git rev-parse --short HEAD) to ${ROBOT_HOST}."
