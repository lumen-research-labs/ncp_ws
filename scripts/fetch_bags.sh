#!/usr/bin/env bash
# Robot -> workstation: rsync rosbags.
#
# Bags are recorded to the robot's local disk during a run and pulled
# afterward. Never stream bags over the network mid-run — the bandwidth
# competes with DDS traffic the safety layer depends on.
#
# Failure mode: exits non-zero; rsync is resumable, re-run it.
set -euo pipefail

# TODO(operator): fill in once the robot's address is established.
ROBOT_HOST="${ROBOT_HOST:-}"
ROBOT_USER="${ROBOT_USER:-}"
ROBOT_BAG_DIR="${ROBOT_BAG_DIR:-~/bags}"
LOCAL_BAG_DIR="${LOCAL_BAG_DIR:-$HOME/rosmaster_bags}"

if [[ -z "$ROBOT_HOST" || -z "$ROBOT_USER" ]]; then
  echo "ERROR: ROBOT_HOST/ROBOT_USER not set. See docs/deployment.md §1." >&2
  exit 1
fi

mkdir -p "$LOCAL_BAG_DIR"
rsync -avh --progress --partial \
  "${ROBOT_USER}@${ROBOT_HOST}:${ROBOT_BAG_DIR}/" "${LOCAL_BAG_DIR}/"

echo "Bags in ${LOCAL_BAG_DIR}."
