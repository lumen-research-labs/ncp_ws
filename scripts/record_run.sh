#!/usr/bin/env bash
# Record one NCP run for offline analysis.
#
# Records every topic needed to reconstruct what the stack saw and decided, so
# a proposed change can be replayed against REAL data before it is made. The
# replay tier (tests/replay/) feeds recorded scans through the actual
# safety_logic.evaluate(), which means a candidate e-stop geometry can be
# scored on a real run without touching config/safety_limits.yaml first.
#
# GROUND TRUTH IS THE OPERATOR'S JOB AND IT IS NOT OPTIONAL. This bag can tell
# you what /odom CLAIMED. It cannot tell you whether that was true --
# odometry cannot check itself, which is the lesson that cost this project the
# 2026-08-28 stop-the-line and the 2026-08-29 calibration refit. So the script
# prompts for tape-measured start/end marks and writes them next to the bag.
# Without them a drift question is unanswerable from the recording alone.
#
# Runs ALONGSIDE a live stack; it starts and stops nothing. Safe at any time.
#
# Failure mode: recording is additive and read-only with respect to the stack.
# If `ros2 bag record` dies the run is unaffected. The ground-truth notes are
# written whether or not they were filled in, so a bag is never silently
# missing its own provenance.
set -uo pipefail

NAME="${1:-run}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${HOME}/ncp_ws/bags/${NAME}_${STAMP}"

# The set is deliberate:
#   /scan            the full-resolution truth the safety layer acts on
#   /ncp/scan        the policy's beams, to confirm the resampler agrees
#   /ncp_debug/*     the observation, action, goal and pose each policy node
#                    used (one stream per config/topics.yaml policy row;
#                    /ncp_debug itself is the upstream consumer's)
#   /policy/*/cmd_vel  what each policy asked for; rm_safety forwarded the
#                    one named by command_source (/safety/status says which
#                    state it was in, the bag's /cmd_vel says what got through)
#   /cmd_vel         what was allowed -- the pair is the whole clamp story
#   /safety/status   state, reason and min_range per tick
#   /base/debug      per-wheel target/measured/pwm and the flags
#   /odom /vel_raw   what odometry claimed (to be checked against the tape)
#   /imu/data_raw    raw IMU, so a future SLAM/EKF comparison has its input
#   /tf /tf_static   frames, needed for any map-frame work later
TOPICS=(
  /scan /ncp/scan
  /ncp_debug /ncp_debug/m8v2 /ncp_debug/m4 /ncp_debug/m9
  /policy/cmd_vel /policy/m8v2/cmd_vel /policy/m4/cmd_vel /policy/m9/cmd_vel
  /teleop/cmd_vel /cmd_vel
  /safety/status /base/debug
  /odom /vel_raw /imu/data_raw /voltage
  /tf /tf_static
)

mkdir -p "$(dirname "$OUT")"

cat <<EOF
== recording: ${OUT}
   topics: ${#TOPICS[@]}

   BEFORE you start: mark the robot's start position on the floor (tape at
   two diagonal corners of the chassis, or a single mark plus a heading line).
   You will be asked for the measured result when you stop.

   Press Ctrl-C to stop recording.

EOF

ros2 bag record -o "$OUT" "${TOPICS[@]}"
STATUS=$?

echo
echo "== bag written: ${OUT} (ros2 bag record exit ${STATUS})"
echo

# --- ground truth, prompted after the fact so the operator measures the ---
# --- END state while the robot is still sitting where it stopped.       ---
NOTES="${OUT}/ground_truth.txt"
mkdir -p "$OUT" 2>/dev/null

echo "== ground truth (press Enter to skip any line, but a skipped line"
echo "   makes the drift question unanswerable from this bag) =="
read -r -p "  straight-line distance start -> end, tape, metres: " GT_DIST
read -r -p "  net heading change, protractor, degrees (+ = CCW/left): " GT_YAW
read -r -p "  lateral offset from the start heading line, metres:    " GT_LAT
read -r -p "  what were you testing / what happened:                 " GT_NOTE

{
  echo "bag:            ${OUT}"
  echo "recorded_utc:   $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "robot:          $(hostname)"
  echo "# TAPE-MEASURED, external to the robot. /odom in the bag is a CLAIM;"
  echo "# these are the measurement it gets checked against."
  echo "gt_distance_m:  ${GT_DIST:-UNMEASURED}"
  echo "gt_yaw_deg:     ${GT_YAW:-UNMEASURED}"
  echo "gt_lateral_m:   ${GT_LAT:-UNMEASURED}"
  echo "note:           ${GT_NOTE:-}"
} > "$NOTES"

echo
echo "== wrote ${NOTES}"
echo "   Pull it to the workstation with:  bash scripts/fetch_bags.sh"
