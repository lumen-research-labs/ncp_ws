#!/bin/bash
# SubagentStop hook: runs a fast build check when a subagent finishes, so the
# manager sees build status before deciding to accept the work.
# Keep this fast (build only, not the full test suite — that's rm-test-runner's job).

INPUT=$(cat)
AGENT_TYPE=$(echo "$INPUT" | jq -r '.agent_type // empty')

# Skip the read-only agents — nothing changed, nothing to build. If the payload
# carries no agent_type at all we run anyway rather than silently doing nothing,
# which is the bug the previous version had (it gated on "ros2-node-writer", an
# agent name that does not exist — the real one is rm-node-writer — so this
# hook never once ran).
case "$AGENT_TYPE" in
  rm-safety-reviewer|rm-test-runner) exit 0 ;;
esac

WORKSPACE_ROOT="${CLAUDE_PROJECT_DIR:-${NCP_WORKSPACE_ROOT:-$(pwd)}}"

if [ ! -d "$WORKSPACE_ROOT/src" ]; then
  echo "build-check: no src/ found at $WORKSPACE_ROOT, skipping" >&2
  exit 0
fi

# Nothing to compile yet — all rm_* packages are still skeletons. Ignore the
# ament boilerplate (setup.py, empty __init__.py) that ships with a bare
# package, or this never short-circuits.
if ! find "$WORKSPACE_ROOT/src" \( -name '*.py' -o -name '*.cpp' -o -name '*.msg' -o -name '*.srv' \) \
     -not -name '__init__.py' -not -name 'setup.py' -not -name 'conftest.py' \
     -print -quit | grep -q .; then
  exit 0
fi

# colcon lives in the workstation ROS2 container, not on the Arch host
# (docs/dev_env_setup.md). Absent it, report — never block on a tool that was
# never going to be here.
if ! command -v colcon >/dev/null 2>&1; then
  echo "build-check: colcon not on PATH (it lives in the workstation ROS2 container)." >&2
  echo "build-check: source changed but was NOT build-verified. Build in the container before accepting." >&2
  exit 0
fi

cd "$WORKSPACE_ROOT" || exit 0

BUILD_OUTPUT=$(colcon build --symlink-install 2>&1)
BUILD_STATUS=$?

if [ $BUILD_STATUS -ne 0 ]; then
  echo "build-check: colcon build FAILED after subagent changes." >&2
  echo "$BUILD_OUTPUT" | tail -30 >&2
  # exit 2 surfaces this as a blocking message back to Claude
  exit 2
fi

echo "build-check: build succeeded." >&2
exit 0
