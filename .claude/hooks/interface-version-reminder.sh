#!/bin/bash
# PostToolUse hook: runs after Write/Edit to config/interface.yaml.
# BUILD_PLAN.md §Phase 3: interface_version must bump on ANY change to the
# frozen contract. This can't reliably detect "did the shape change" from a
# single file's post-edit state, so it WARNS (not blocks) and reminds the
# agent/reviewer to check explicitly.

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

if [ -z "$FILE_PATH" ]; then
  exit 0
fi

case "$FILE_PATH" in
  */config/interface.yaml) ;;
  *) exit 0 ;;
esac

echo "REMINDER: config/interface.yaml was just modified." >&2
echo "Per BUILD_PLAN.md §Phase 3, interface_version must bump on ANY change" >&2
echo "to this file. Confirm interface_version was updated, and that the" >&2
echo "schema loader (imported by both robot and workstation tooling) still" >&2
echo "matches this file — never hand-write the observation/action ordering" >&2
echo "a second time elsewhere." >&2

exit 0
