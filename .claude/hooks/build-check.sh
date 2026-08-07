#!/bin/bash
# SubagentStop hook: runs a fast build check when a subagent finishes,
# so the manager sees build status before deciding to accept the work.
# Keep this fast (build only, not full test suite — that's test-runner's job).

INPUT=$(cat)
AGENT_TYPE=$(echo "$INPUT" | jq -r '.agent_type // empty')

# Only run after the writer subagent — no point building after a read-only reviewer
if [ "$AGENT_TYPE" != "ros2-node-writer" ]; then
  exit 0
fi

WORKSPACE_ROOT="${NCP_WORKSPACE_ROOT:-$(pwd)}"

if [ ! -d "$WORKSPACE_ROOT/src" ]; then
  echo "build-check: no src/ found at $WORKSPACE_ROOT, skipping" >&2
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
