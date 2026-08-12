# SafetyStatus.msg
#
# Published by rm_safety on /safety/status at a fixed rate (safety_limits.yaml
# status_rate_hz). Carries the current state of the safety finite-state
# machine, the reason the last state transition was triggered, the minimum
# lidar range seen in the monitored arcs this cycle, and whether any velocity
# clamping was applied to the outgoing /cmd_vel command.
#
# Kill and reset are exposed as /safety/kill and /safety/reset services.
# Those services use std_srvs/srv/Trigger (bool success, string message),
# which is exactly the right shape for a no-argument, success+message pattern.
# rm_interfaces therefore ships NO custom srv — the dependency on std_srvs
# belongs to rm_safety, not to this package.
#
# State constants — match the FSM states in rm_safety/safety_logic.py.
uint8 OK      = 0   # nominal: commands are passed through (possibly clamped)
uint8 SLOW    = 1   # obstacle in slow-zone: velocity is capped to slow limits
uint8 STOPPED = 2   # obstacle in stop-zone or command timeout: zero velocity
uint8 KILLED  = 3   # hard kill active (startup default or /safety/kill call)

# ---- fields ---------------------------------------------------------------

# Stamp is set to the time the safety node evaluated this cycle.
# Frame is left empty — this is a scalar status, not a spatial measurement.
std_msgs/Header header

# Current FSM state. Use the named constants above (OK / SLOW / STOPPED / KILLED).
uint8 state

# Human-readable reason for the current state, e.g.:
#   "ok", "slow_zone", "stop_zone", "command_timeout",
#   "scan_stale", "odom_stale", "killed_on_startup", "kill_service"
string reason

# Minimum lidar range (metres) within the monitored forward arcs this cycle.
# Set to +inf (float('inf')) when no valid scan is available.
float32 min_range

# True if the safety node altered the outgoing /cmd_vel command via clamping
# (linear or angular velocity, or lateral zeroing) this cycle.
bool clamping_active
