# robot_env.sh — which robot am I on? Sourced (not run) by run_stack.sh and
# stack_down.sh on the robot.
#
# Resolves this unit's profile (config/robots/<name>.yaml) by `hostname`, or
# by name when ROBOT=<name> is set, through rm_safety/robot_profile.py, and
# exports:
#   ROS_DOMAIN_ID      the profile's domain, so the stack and its stop button
#                      share one domain and two robots on one network do not
#   RM_ROBOT_PROFILE   the profile's path, passed to ncp.launch.py
#   RM_ROBOT_NAME
# An unknown hostname is a refusal that lists the known profiles; nothing
# that sources this file goes on to start or stop anything.

_RM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_RM_WHO="${ROBOT:-$(hostname)}"
_RM_OUT="$(python3 - "$_RM_ROOT" "$_RM_WHO" <<'PY'
import sys
root, who = sys.argv[1], sys.argv[2]
sys.path.insert(0, root + "/src/rm_safety")
from rm_safety.robot_profile import load_robot_profile, resolve_profile_path
try:
    path = resolve_profile_path(root + "/config/robots", who)
    p = load_robot_profile(path)
except ValueError as exc:
    print(f"robot_env: {exc}", file=sys.stderr)
    sys.exit(1)
print(path, p.ros_domain_id, p.name)
PY
)" || { echo "robot_env: cannot resolve a robot profile for '${_RM_WHO}'" >&2; return 1 2>/dev/null || exit 1; }
read -r RM_ROBOT_PROFILE ROS_DOMAIN_ID RM_ROBOT_NAME <<<"$_RM_OUT"
export RM_ROBOT_PROFILE ROS_DOMAIN_ID RM_ROBOT_NAME
echo "robot: ${RM_ROBOT_NAME} (domain ${ROS_DOMAIN_ID}, ${RM_ROBOT_PROFILE})"
