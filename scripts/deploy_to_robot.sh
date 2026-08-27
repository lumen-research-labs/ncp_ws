#!/usr/bin/env bash
# Workstation -> robot: push the repo directly over LAN SSH, no GitHub.
#
# Why this exists: the project has been blocked since 2026-08-14 waiting on
# a GitHub deploy key for bernard-2, and bernard-1 lives on an isolated lab
# network (RobotLab) with NO internet uplink at all -- GitHub is reachable
# from neither the robot's side. This script removes GitHub from the path
# entirely: the workstation is the only source of truth, reached directly
# over the LAN via the ssh host alias already in ~/.ssh/config.
#
# Mechanism: `git push` straight into a non-bare git repo at ~/<ws> on the
# robot, with `receive.denyCurrentBranch=updateInstead` set on that repo so
# the push updates the working tree in place (this is the standard
# "git push to deploy" pattern; see `git help push` NOTES on
# receive.denyCurrentBranch). Chosen over `git bundle` + scp because:
#   - It is incremental and idempotent for free: re-running only transfers
#     new objects and fast-forwards the remote tree. A bundle+scp path would
#     need its own bookkeeping to avoid re-sending the whole history and its
#     own logic to detect "already up to date".
#   - It gets the "refuse to clobber uncommitted work" requirement for free
#     too: git itself refuses an updateInstead push if the remote working
#     tree has modifications or untracked files that would conflict, and
#     exits non-zero. A bundle+scp path would have to hand-roll that check
#     (e.g. `git status --porcelain` over ssh, racy against the unbundle).
#   - The robot's wrong, non-NTP-syncable clock is a non-issue for this
#     mechanism: git objects are content-addressed and all commits are
#     authored on the workstation (correct clock); the robot never commits
#     locally, so there is no local timestamp for git to compare against or
#     warn about. A bundle is just a different serialization of the exact
#     same objects, so it has no clock advantage here -- it only adds a
#     manual unbundle/checkout step this approach skips.
#
# This script transfers ONLY. It never runs colcon build and never launches
# a ROS node on the robot -- those remain separate, operator-supervised
# steps (see docs/deployment.md and BUILD-PLAN.md phase gating).
#
# Failure mode: exits non-zero on any step (unreachable host, dirty remote
# tree, push rejection); never silently resets or force-pushes over robot
# state.
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

usage() {
  cat <<EOF
Usage: ${SCRIPT_NAME} [ROBOT] [--branch BRANCH] [--remote-path PATH]

Push the current repo state to a ROSMASTER robot over LAN SSH (no GitHub).

Arguments:
  ROBOT              ssh host alias for the robot (must exist in
                      ~/.ssh/config). Default: \$ROBOT env var, or
                      "bernard-1" if unset.

Options:
  --branch BRANCH     Local branch to deploy. Default: current branch.
  --remote-path PATH  Path on the robot, relative to the robot user's home,
                      to hold the working checkout. Default: "ncp_ws".
  -h, --help          Show this help and exit.

Examples:
  ${SCRIPT_NAME}                    # deploy current branch to bernard-1
  ${SCRIPT_NAME} bernard-2          # deploy to bernard-2 instead
  ROBOT=bernard-2 ${SCRIPT_NAME}    # same, via env var

First run on a robot initialises a git repo at ~/<remote-path> there; later
runs fast-forward it. Never builds or launches anything on the robot.
EOF
}

ROBOT="${ROBOT:-bernard-1}"
BRANCH=""
REMOTE_PATH="ncp_ws"

# --- argument parsing ---
POSITIONAL_SET=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --branch)
      [[ $# -ge 2 ]] || { echo "ERROR: --branch requires a value." >&2; exit 1; }
      BRANCH="$2"
      shift 2
      ;;
    --remote-path)
      [[ $# -ge 2 ]] || { echo "ERROR: --remote-path requires a value." >&2; exit 1; }
      REMOTE_PATH="$2"
      shift 2
      ;;
    --*)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
    *)
      if [[ "$POSITIONAL_SET" -eq 1 ]]; then
        echo "ERROR: unexpected extra argument: $1" >&2
        exit 1
      fi
      ROBOT="$1"
      POSITIONAL_SET=1
      shift
      ;;
  esac
done

if [[ -z "$BRANCH" ]]; then
  BRANCH="$(git rev-parse --abbrev-ref HEAD)"
fi
if [[ "$BRANCH" == "HEAD" ]]; then
  echo "ERROR: detached HEAD -- checkout a branch before deploying, or pass --branch." >&2
  exit 1
fi

# Reject anything that would make the remote path escape the robot's home
# directory or be interpreted oddly by the remote shell.
if [[ "$REMOTE_PATH" == /* || "$REMOTE_PATH" == *".."* ]]; then
  echo "ERROR: --remote-path must be a relative path with no '..': got '${REMOTE_PATH}'." >&2
  exit 1
fi

REMOTE_URL="${ROBOT}:${REMOTE_PATH}"
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8)

echo "==> Target: ${ROBOT} (~/${REMOTE_PATH}), branch '${BRANCH}'"

# --- preflight: is the robot even reachable? ---
echo "==> Checking ${ROBOT} is reachable..."
if ! ssh "${SSH_OPTS[@]}" "$ROBOT" true 2>/tmp/deploy_to_robot_ssh_err.$$; then
  echo "ERROR: could not reach '${ROBOT}' over SSH within 8s." >&2
  echo "       ssh said:" >&2
  sed 's/^/         /' /tmp/deploy_to_robot_ssh_err.$$ >&2 || true
  rm -f /tmp/deploy_to_robot_ssh_err.$$
  echo "       Check the robot is powered on, on the RobotLab network, and" >&2
  echo "       that the '${ROBOT}' entry in ~/.ssh/config still matches its" >&2
  echo "       current address." >&2
  exit 1
fi
rm -f /tmp/deploy_to_robot_ssh_err.$$
echo "    reachable."

# --- remote init (idempotent): ensure ~/<remote-path> is a git repo ready ---
# to receive a push-to-deploy. Safe to run every time; only initialises on
# the first run.
#
# Critically, HEAD must point at refs/heads/<branch> BEFORE the push
# happens: updateInstead only updates the working tree when the ref being
# pushed is the one HEAD currently points to. Pushing first and pointing
# HEAD afterward (tried during offline testing) creates the branch in
# .git/refs but leaves the working tree empty -- a silent-looking "deploy"
# that actually shipped nothing. Setting HEAD via symbolic-ref works even
# before the branch has any commits.
echo "==> Ensuring remote repo exists at ${ROBOT}:~/${REMOTE_PATH}..."
ssh "${SSH_OPTS[@]}" "$ROBOT" bash -s -- "$REMOTE_PATH" "$BRANCH" <<'REMOTE_INIT'
set -euo pipefail
remote_path="$1"
branch="$2"
target="${HOME}/${remote_path}"

mkdir -p "$target"
cd "$target"

if [[ ! -d .git ]]; then
  git init --quiet
  echo "    (initialised new git repo at ${target})"
fi

# Required every run, not just on init: cheap, and self-healing if someone
# ever hand-edits the remote config back to the git default (denyCurrentBranch
# refuse), which would otherwise turn future pushes into silent no-ops.
git config receive.denyCurrentBranch updateInstead

# Must happen before the push -- see comment above the ssh call.
git symbolic-ref HEAD "refs/heads/${branch}"
REMOTE_INIT
echo "    remote repo ready."

# --- push. git's own updateInstead safety net refuses this if the remote
# working tree has uncommitted/untracked changes that would be clobbered,
# so we do not need (and deliberately do not add) a --force anywhere here. ---
echo "==> Pushing ${BRANCH} to ${REMOTE_URL}..."
if ! git push "$REMOTE_URL" "refs/heads/${BRANCH}:refs/heads/${BRANCH}"; then
  echo "ERROR: push rejected. Most likely the robot's working tree at" >&2
  echo "       ~/${REMOTE_PATH} has uncommitted or untracked changes that" >&2
  echo "       would be overwritten -- refusing rather than clobbering them." >&2
  echo "       Investigate on the robot (git status inside ~/${REMOTE_PATH})" >&2
  echo "       before retrying. Code is never meant to be edited on the" >&2
  echo "       robot directly (see project CLAUDE.md); if that happened," >&2
  echo "       resolve it by hand, don't force-push over it." >&2
  exit 1
fi

DEPLOYED_REV="$(git rev-parse --short "$BRANCH")"
echo "==> Deployed ${DEPLOYED_REV} (${BRANCH}) to ${ROBOT}:~/${REMOTE_PATH}."
echo "    Not built, not launched -- colcon build and any ROS launch remain"
echo "    separate, operator-supervised steps."
