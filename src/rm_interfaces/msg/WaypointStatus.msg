# Status of the go-to-waypoint controller (rm_waypoint/waypoint_node).
#
# Published every control tick, whether or not a waypoint is active, so the
# absence of motion is always explained by a value rather than by silence.
# The dashboard renders this verbatim; nothing here is a control input.

std_msgs/Header header

# --- state machine ---------------------------------------------------------
uint8 IDLE=0      # no waypoint; the node is publishing NOTHING
uint8 TURNING=1   # rotating in place to face the waypoint
uint8 DRIVING=2   # driving toward the waypoint
uint8 ARRIVED=3   # reached it; stopped and latched idle
uint8 ABORTED=4   # gave up; stopped and latched idle. See `reason`.
uint8 state

# Why the controller is in this state. Empty while simply driving. On ABORTED
# this is the whole explanation and is surfaced verbatim to the operator --
# "it stopped and I do not know why" is the failure this field exists to
# prevent.
string reason

# --- the target ------------------------------------------------------------
# The frame the waypoint is fixed in ("map" or "odom"). A waypoint is only
# meaningful with its frame: the same numbers mean different places in each,
# and in odom they drift.
string frame_id
float64 target_x
float64 target_y

# --- live geometry ---------------------------------------------------------
float64 range_m            # straight-line distance remaining
float64 heading_error_rad  # bearing to the target in the robot's own frame
float64 elapsed_s          # since the waypoint was accepted

# --- what is being asked for ----------------------------------------------
# The command this node is publishing on /teleop/cmd_vel. NOT what reaches the
# wheels: rm_safety clamps, scales and can zero it, and only forwards it at all
# when command_source is 'teleop' and the kill is cleared.
float64 commanded_vx
float64 commanded_wz

# True when the node is computing normally but deliberately publishing no
# Twist (dry_run). Reported rather than inferred, so a run that cannot move
# never looks like one that can.
bool dry_run
