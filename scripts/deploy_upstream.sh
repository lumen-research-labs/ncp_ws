#!/usr/bin/env bash
# Workstation -> robot: push the NCP-on-Rosmaster checkout, INCLUDING the
# trained weights, over LAN SSH.
#
# Why this is separate from deploy_to_robot.sh: that script pushes THIS repo
# (ncp_ws) as a git branch. The upstream checkout is a different repository
# that this workspace only vendors from, and the robot's copy of it is not a
# git repo at all -- it is a plain directory (verified on bernard-1,
# 2026-08-29). Pushing a git branch into it is therefore not an option, and
# conflating the two would mean one command with two unrelated failure modes.
#
# Why it exists at all: bernard-1 has NO INTERNET (RobotLab has no WAN
# uplink), so `git pull` on the robot cannot reach GitHub. The workstation is
# the only route by which the 2 MB `weights/actor.pt` gets onto the robot.
#
# Mechanism: `tar | ssh tar -x`. Chosen over rsync because rsync's presence on
# a JetPack image is not guaranteed and a missing binary would fail late,
# after the operator has already committed to the step; tar and ssh are both
# certain. The transfer is small enough (~2 MB) that rsync's incrementality
# buys nothing.
#
# VERIFICATION IS THE POINT. A truncated or half-written checkpoint is not a
# loud failure -- torch would raise, but a stale-yet-valid FILE would not, and
# then the robot runs a different policy than the workstation believes. So
# every transferred file's sha256 is compared afterwards, and the script exits
# non-zero on any mismatch. The checkpoint hash printed here is the same one
# policy_node logs at startup and publishes on /ncp_debug, so all three can be
# eyeballed against each other.
#
# Transfers ONLY. Never builds, never launches, never touches a ROS node.
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Sibling-checkout convention, matching ncp.launch.py's upstream_path default
# and tests/unit/test_vendor_is_pristine.py.
DEFAULT_SRC="$(cd "${REPO_ROOT}/.." && pwd)/NCP-on-Rosmaster"

usage() {
  cat <<EOF
Usage: ${SCRIPT_NAME} [ROBOT] [--src PATH] [--remote-path PATH]

Push the NCP-on-Rosmaster checkout (code + trained weights) to a robot.

Arguments:
  ROBOT              ssh host alias. Default: \$ROBOT, or "bernard-1".

Options:
  --src PATH          Local upstream checkout.
                      Default: ${DEFAULT_SRC}
  --remote-path PATH  Path on the robot relative to its home.
                      Default: "NCP-on-Rosmaster"
  -h, --help          Show this help.

After it runs, launch with the checkpoint (this is the DEFAULT path, so no
weights argument is needed):

  ros2 launch rm_bringup ncp.launch.py command_source:=policy
EOF
}

ROBOT="${ROBOT:-bernard-1}"
SRC="$DEFAULT_SRC"
REMOTE_PATH="NCP-on-Rosmaster"
POSITIONAL_SET=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --src)
      [[ $# -ge 2 ]] || { echo "ERROR: --src requires a value." >&2; exit 1; }
      SRC="$2"; shift 2 ;;
    --remote-path)
      [[ $# -ge 2 ]] || { echo "ERROR: --remote-path requires a value." >&2; exit 1; }
      REMOTE_PATH="$2"; shift 2 ;;
    --*) echo "ERROR: unknown option: $1" >&2; usage >&2; exit 1 ;;
    *)
      [[ "$POSITIONAL_SET" -eq 0 ]] || { echo "ERROR: extra argument: $1" >&2; exit 1; }
      ROBOT="$1"; POSITIONAL_SET=1; shift ;;
  esac
done

if [[ "$REMOTE_PATH" == /* || "$REMOTE_PATH" == *".."* ]]; then
  echo "ERROR: --remote-path must be relative with no '..': '${REMOTE_PATH}'." >&2
  exit 1
fi

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8)

# The files that matter. Everything else in the upstream repo (its README, its
# tests) is carried along because it is small and because a partial copy is
# harder to reason about than a whole one -- but these three are the ones
# whose integrity is checked, because these three are what actually runs.
CRITICAL=(
  "rosmaster/policy.py"
  "rosmaster/node.py"
  "weights/actor.pt"
)

echo "==> Source: ${SRC}"
[[ -d "$SRC" ]] || { echo "ERROR: no upstream checkout at '${SRC}'." >&2; exit 1; }

for f in "${CRITICAL[@]}"; do
  if [[ ! -f "${SRC}/${f}" ]]; then
    echo "ERROR: missing '${f}' in ${SRC}." >&2
    if [[ "$f" == "weights/actor.pt" ]]; then
      echo "       The trained checkpoint is not in this checkout. Pull the" >&2
      echo "       upstream repo on the WORKSTATION first (it has internet);" >&2
      echo "       the robot cannot fetch it itself." >&2
    fi
    exit 1
  fi
done

# Warn (do not block) if the vendored copy has drifted from what is about to
# be shipped. They are allowed to differ transiently while re-vendoring; they
# must not differ when the robot runs, because policy_node loads the
# checkpoint into the VENDORED class and a shape mismatch is a startup crash.
VENDORED="${REPO_ROOT}/src/rm_policy/rm_policy/vendor/ncp_policy.py"
if [[ -f "$VENDORED" ]]; then
  if ! cmp -s "$VENDORED" "${SRC}/rosmaster/policy.py"; then
    echo "WARNING: the vendored ncp_policy.py DIFFERS from ${SRC}/rosmaster/policy.py." >&2
    echo "         policy_node loads actor.pt into the VENDORED class, so if the" >&2
    echo "         architectures disagree it will fail at startup. Re-vendor and" >&2
    echo "         update tests/unit/test_vendor_is_pristine.py before running." >&2
  fi
fi

echo "==> Checking ${ROBOT} is reachable..."
ssh "${SSH_OPTS[@]}" "$ROBOT" true || {
  echo "ERROR: could not reach '${ROBOT}' over SSH within 8s." >&2
  echo "       Check it is powered on, on the RobotLab network, and that the" >&2
  echo "       '${ROBOT}' entry in ~/.ssh/config still matches its address." >&2
  exit 1
}
echo "    reachable."

echo "==> Transferring to ${ROBOT}:~/${REMOTE_PATH}/ ..."
# --exclude the caches: shipping a workstation .pyc for a different Python
# than the robot runs is at best useless and at worst confusing in a listing.
tar -C "$SRC" \
    --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
    --exclude='.venv' \
    -cf - . \
  | ssh "${SSH_OPTS[@]}" "$ROBOT" \
      "mkdir -p ~/'${REMOTE_PATH}' && tar -C ~/'${REMOTE_PATH}' -xf -"

echo "==> Verifying checksums..."
FAILED=0
for f in "${CRITICAL[@]}"; do
  local_sum="$(sha256sum "${SRC}/${f}" | cut -d' ' -f1)"
  remote_sum="$(ssh "${SSH_OPTS[@]}" "$ROBOT" \
      "sha256sum ~/'${REMOTE_PATH}/${f}' 2>/dev/null | cut -d' ' -f1" || true)"
  if [[ "$local_sum" == "$remote_sum" && -n "$remote_sum" ]]; then
    printf '    OK   %-24s %s\n' "$f" "${local_sum:0:12}"
  else
    remote_show="${remote_sum:0:12}"
    [[ -n "$remote_sum" ]] || remote_show="<absent>"
    printf '    FAIL %-24s local %s remote %s\n' \
      "$f" "${local_sum:0:12}" "$remote_show"
    FAILED=1
  fi
done

if [[ "$FAILED" -ne 0 ]]; then
  echo "ERROR: transfer verification failed -- do NOT run the policy." >&2
  exit 1
fi

CKPT_SHA="$(sha256sum "${SRC}/weights/actor.pt" | cut -d' ' -f1)"
echo "==> Deployed and verified."
echo "    checkpoint sha256 = ${CKPT_SHA}"
echo "    policy_node logs this same hash at startup and publishes its first"
echo "    12 chars on /ncp_debug. If they ever disagree, the robot is running"
echo "    a different checkpoint than this workstation thinks."
echo "    Not built, not launched -- those remain operator-supervised steps."
