# BaseDebug.msg
#
# Published by rm_base_node on /base/debug every control tick. Diagnostics
# only — nothing subscribes to this in the control path, and nothing may.
#
# This replaces an earlier geometry_msgs/Twist that packed four PWM values
# into linear.x/y/z + angular.x. That was unreadable in `ros2 topic echo`
# and, worse, it reported only the PWM actually written to the board, so in
# dry_run (the default) it showed four zeros and told the operator nothing
# about what the command WOULD have done. Every array here is reported in
# both dry_run and live mode; only `applied_pwm` is gated on dry_run.
#
# ---- wheel ordering -------------------------------------------------------
#
# Every fixed-size array below is indexed:
#
#   [0] front_left   (motor channel M1)
#   [1] back_left    (M2)
#   [2] front_right  (M3)  <- the failing motor on bernard-1
#   [3] back_right   (M4)
#
# This is the MEASURED motor->wheel map (2026-08-28), not the Yahboom sample
# order (M1,M2,M3,M4 = FL,FR,BL,BR). Do not "fix" it back.

uint8 FRONT_LEFT  = 0
uint8 BACK_LEFT   = 1
uint8 FRONT_RIGHT = 2
uint8 BACK_RIGHT  = 3

std_msgs/Header header

# ---- what was asked for ---------------------------------------------------

# The last /cmd_vel received, after rm_base's defensive body clamp. Reported
# even when it is not being acted on (dry_run, or stale), so that a silent
# robot can be told apart from an unheard command.
geometry_msgs/Twist commanded_twist

# "open_loop" or "closed_loop" — the control_mode parameter in force.
string control_mode

# True while only zero PWM is written to the board, whatever the rest of
# this message says.
bool dry_run

# True when no /cmd_vel arrived within command_timeout_ms; the driven twist
# is zero regardless of commanded_twist.
bool command_stale

# ---- per wheel ------------------------------------------------------------

# Wheel ground speed the mecanum inverse kinematics asked for, m/s, forward
# positive.
float32[4] target_speed_mps

# Wheel ground speed reconstructed from the sign-corrected encoder deltas.
# Zero and feedback_valid false when there was no fresh encoder report.
float32[4] measured_speed_mps

# Open-loop calibration term: what base_calibration.yaml alone would command.
float32[4] feedforward_pwm

# Accumulated PI integral, PWM units. Always 0.0 in open_loop mode.
float32[4] integral_pwm

# Controller output before slew limiting.
int16[4] target_pwm

# What was actually written with set_motor this tick. All zeros in dry_run.
int16[4] applied_pwm

# False when the encoder measurement for this wheel could not be trusted this
# tick (no fresh report, implausible jump, or dry_run). The loop degrades to
# feedforward when false.
bool[4] feedback_valid

# True when the controller output hit the wheel's max_pwm ceiling. A wheel
# saturating while the others do not is how the failing front-right motor
# shows up: closed loop will ask for everything it has and still fall short.
bool[4] saturated

# True when a wheel commanded to move reports no motion. The integrator is
# frozen for that wheel — this is the guard that stops a dead encoder from
# winding the output to the rail the way the STM32 firmware does.
bool[4] stalled

# True when the wheel is measurably turning AGAINST the commanded direction,
# i.e. the encoder polarity assumption is wrong. Latched, and forces that
# wheel's PWM to zero. If this ever goes true, STOP: the sign correction
# this whole package is built on no longer holds.
bool[4] sign_fault
