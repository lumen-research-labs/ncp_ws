#!/bin/bash
# PostToolUse hook: runs after Write/Edit anywhere in the rosmaster_rl workspace.
# Enforces BUILD_PLAN.md §0 rule 3: only rm_safety may publish to /cmd_vel.
# This is a hard rule, not a style preference, so this hook BLOCKS (exit 2)
# rather than just warning.

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

if [ -z "$FILE_PATH" ] || [ ! -f "$FILE_PATH" ]; then
  exit 0
fi

case "$FILE_PATH" in
  *.py|*.cpp|*.hpp) ;;
  *) exit 0 ;;
esac

# Find workspace root (has src/ and config/interface.yaml)
WORKSPACE_ROOT=$(dirname "$FILE_PATH")
while [ ! -f "$WORKSPACE_ROOT/config/interface.yaml" ] && [ "$WORKSPACE_ROOT" != "/" ]; do
  WORKSPACE_ROOT=$(dirname "$WORKSPACE_ROOT")
done

if [ ! -f "$WORKSPACE_ROOT/config/interface.yaml" ]; then
  exit 0
fi

# Does this file publish to /cmd_vel (not /policy/cmd_vel, /teleop/cmd_vel, etc.)?
if grep -qE "create_publisher\([^,]*,\s*['\"]\/cmd_vel['\"]" "$FILE_PATH" 2>/dev/null || \
   grep -qE "advertise[^,]*,\s*['\"]\/cmd_vel['\"]" "$FILE_PATH" 2>/dev/null; then

  # Allowed only if this file lives under a package named rm_safety
  if [[ "$FILE_PATH" != *"/rm_safety/"* ]]; then
    echo "BLOCKED: $FILE_PATH publishes to /cmd_vel directly." >&2
    echo "Per BUILD_PLAN.md §0 rule 3, only rm_safety may publish /cmd_vel." >&2
    echo "Publish to a namespaced input topic instead (e.g. /policy/cmd_vel) and let rm_safety relay it." >&2
    exit 2
  fi
fi

exit 0
