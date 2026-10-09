"""
slam_restart.py — the ROS-free half of the dashboard's RESET MAP control.

What it does:
    The slam_toolbox build on the robot (2.6.9, Humble) has no reset service,
    so a fresh map is a restart of the slam_toolbox PROCESS. slam.launch.py
    launches it with `respawn=True`, so the only act required is to
    terminate the running node: the launch system brings it back with the
    same parameters and an empty pose graph, and the robot's position at
    that moment becomes the new map origin. This module finds that one
    process and decides whether a reset is allowed. The ROS node
    (dashboard_node._reset_map) does the killing, the waiting and the goal
    re-fix, in that order, after stopping the robot.

    The process is found by scanning /proc for an argv[0] whose basename is
    the slam_toolbox executable. Exact basename, never a substring: a
    substring match is how `pkill -f` kills the shell running it (the trap in
    the robot-ssh skill), and here it would also match a `bash -c "..."`
    wrapper or the dashboard's own command line if it ever mentioned the name.

Subscribes to:  (nothing — pure module)
Publishes:      (nothing — pure module)

Failure mode:
    Refusal with a reason, never a guess. Zero processes means SLAM is not
    running and there is nothing to reset; more than one means a duplicate
    stack, which is the project's defining failure and must be torn down
    with stack_down.sh rather than reset from a web page.
"""

from __future__ import annotations

import os
from typing import List, Optional

SLAM_EXECUTABLE = "async_slam_toolbox_node"


def find_slam_pids(proc_root: str = "/proc", executable: str = SLAM_EXECUTABLE) -> List[int]:
    """PIDs whose argv[0] basename is `executable`, ascending."""
    pids: List[int] = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return pids
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join(proc_root, entry, "cmdline"), "rb") as fh:
                argv0 = fh.read().split(b"\0", 1)[0]
        except OSError:
            continue  # the process exited between listdir and open
        if os.path.basename(argv0.decode(errors="replace")) == executable:
            pids.append(int(entry))
    return sorted(pids)


def reset_refusal(pose_source: Optional[str], slam_pids: List[int]) -> Optional[str]:
    """Why a map reset must not proceed, or None when it may."""
    if pose_source != "map":
        return (
            f"refused: policy_node pose_source is {pose_source!r}, not 'map' — "
            "there is no SLAM pose to reset. Launch with with_slam:=true."
        )
    if not slam_pids:
        return (
            f"refused: no {SLAM_EXECUTABLE} process is running (with_slam:=true?). "
            "Nothing to restart."
        )
    if len(slam_pids) > 1:
        return (
            f"refused: {len(slam_pids)} {SLAM_EXECUTABLE} processes are running "
            f"(pids {slam_pids}) — a duplicate stack. Run scripts/stack_down.sh."
        )
    return None
