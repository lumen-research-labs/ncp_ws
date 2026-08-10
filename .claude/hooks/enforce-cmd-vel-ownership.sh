#!/bin/bash
# PreToolUse hook on Write|Edit.
# Enforces BUILD_PLAN.md §0 rule 3: only rm_safety may publish to /cmd_vel.
# This is a hard rule, not a style preference, so this hook BLOCKS (exit 2).
#
# PreToolUse, not PostToolUse: the check runs against the *proposed* content,
# so a violating file is never written at all. The earlier PostToolUse version
# let the bad code land on disk and only complained afterwards.

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

[ -z "$FILE_PATH" ] && exit 0

case "$FILE_PATH" in
  *.py|*.cpp|*.hpp) ;;
  *) exit 0 ;;
esac

# rm_safety is the one package allowed to do this — nothing to check.
case "$FILE_PATH" in
  */rm_safety/*) exit 0 ;;
esac

# Only look at what this call is actually adding: Write.content, or the
# replacement half of an Edit. Checking old_string too would block edits that
# are *removing* a violation.
CONTENT=$(echo "$INPUT" | jq -r '.tool_input.content // .tool_input.new_string // empty')

[ -z "$CONTENT" ] && exit 0

# Matches "/cmd_vel" or "cmd_vel" as a whole topic string, but deliberately
# NOT "/policy/cmd_vel", "/teleop/cmd_vel" or any other namespaced input topic
# — those are the correct pattern and must keep working.
if echo "$CONTENT" | grep -qE "(create_publisher|advertise)[^,]*,\s*['\"]/?cmd_vel['\"]"; then
  cat >&2 <<EOF
BLOCKED: $FILE_PATH publishes to /cmd_vel directly.

Per BUILD_PLAN.md §0 rule 3, only rm_safety may publish /cmd_vel. Every other
command source is an *input* to the safety node, never a direct path to the
base driver.

Fix: publish to a namespaced input topic (e.g. /policy/cmd_vel,
/teleop/cmd_vel) and let rm_safety relay it after clamping.
EOF
  exit 2
fi

exit 0
