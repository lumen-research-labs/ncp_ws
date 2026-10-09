"""
slam_health.py — turns a stream of `map -> odom` corrections into the few
numbers that say whether SLAM is actually working. Pure, ROS-free, testable.

What it does:
    `map -> odom` IS the SLAM correction. It is the transform slam_toolbox
    publishes to say "the odometry thinks the robot is here; it is really over
    there". Everything worth knowing about the health of a running SLAM is in
    the behaviour of that one transform over time, and none of it is visible
    in a map picture:

      * ITS MAGNITUDE is the accumulated odometry error SLAM has had to undo.
        On this robot that number is expected to grow — /odom over-reads
        linear distance by ~5% (measured 2026-08-29) — and a correction that
        stays at exactly zero while the robot drives means scan matching is
        contributing nothing, i.e. SLAM is running but not working.

      * ITS STEPS are loop closures and relocalizations. Those are the hazard
        this pipeline cares about: `goal_local` is 2 of the 36 numbers the
        network reads, and policy_node resets the CfC's recurrent state on
        each one. Counting them tells an operator whether a run had four
        recurrent-state resets in it or none, which otherwise shows up only as
        unexplained steering.

      * ITS AGE says whether SLAM is alive at all. A dead slam_toolbox does
        not error — the transform simply stops being republished, the policy's
        pose lookup starts failing, and the robot fails closed with a
        `pose_*` reason several layers from the cause.

    A worked example of why the last one matters: on 2026-08-29 slam_toolbox
    silently rejected EVERY scan (the TG30's varying ray count against karto's
    latched expectation). No crash, no warning, no map, no `map -> odom`. It
    read exactly like SLAM failing to converge. This module's job is to make
    that state legible in one glance instead of one session.

    This module also holds `pack_grid` and `summarize_map`, which reduce the
    occupancy grid itself for display. They live here rather than in the node
    for one reason: `pack_grid`'s decimation rule is a correctness property —
    it must not delete walls — and a rule that can be silently wrong belongs
    somewhere a test can reach it without a ROS runtime.

Subscribes to:  (nothing — pure module; the node feeds it samples)
Publishes:      (nothing — pure module; the server reads a summary from it)

Failure mode:
    Total and non-raising, like the rest of the dashboard's pure layer. Every
    accessor returns a well-formed summary even when no correction has ever
    arrived (`available: False`, everything else None). Nothing here is a
    control input; it colours a web page. A jump it fails to notice costs a
    line on a diagnostic, not a safety decision — policy_node does its own
    jump detection, on its own samples, and is the authority.
"""

from __future__ import annotations

import base64
import math
import threading
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

# A correction step this large or larger counts as a loop closure. Well above
# the continuous creep of scan matching trimming a few millimetres per update,
# and comfortably below a real relocalization, which moves the robot by a
# noticeable fraction of a room. Diagnostic only — policy_node's own
# `detect_pose_jump` is what actually resets the recurrent state, and it works
# on implied speed rather than on a fixed step like this.
JUMP_TRANSLATION_M = 0.10
JUMP_YAW_RAD = 0.10

# How many recent jumps to keep for display. Enough to show a burst of loop
# closures without the payload growing without bound over a long session.
_JUMP_HISTORY = 12

# Correction samples retained for the trend readout.
_SAMPLE_HISTORY = 200

# Occupancy probability at or above which a grid cell counts as an obstacle.
# The ROS map convention: 0..100 is a probability, -1 is unknown. 65 is
# nav2's standard occupied threshold; used here for the coverage statistic and
# for choosing a pixel colour.
OCCUPIED_THRESHOLD = 65

# How the packed map bytes encode a cell, one byte each. Bit-packing would
# save two thirds of the payload at the cost of every reader having to agree
# on the packing, and the payload is already base64'd and fetched rarely.
CELL_UNKNOWN = 0
CELL_FREE = 1
CELL_OCCUPIED = 2


class SlamHealth:
    """Accumulates `map -> odom` samples and summarizes them.

    Thread-safe in the same shape as StateStore: the ROS thread writes, the
    HTTP thread reads a consistent snapshot.
    """

    def __init__(
        self,
        jump_translation_m: float = JUMP_TRANSLATION_M,
        jump_yaw_rad: float = JUMP_YAW_RAD,
    ) -> None:
        self._lock = threading.Lock()
        self._jump_translation_m = jump_translation_m
        self._jump_yaw_rad = jump_yaw_rad

        self._current: Optional[Tuple[float, float, float]] = None
        self._first_seen: Optional[float] = None
        self._last_seen: Optional[float] = None
        self._samples: Deque[Tuple[float, float]] = deque(maxlen=_SAMPLE_HISTORY)
        self._jumps: Deque[Dict[str, Any]] = deque(maxlen=_JUMP_HISTORY)
        self._jump_count = 0
        self._max_correction_m = 0.0

    def reset(self) -> None:
        """Forget every sample and every jump.

        For a DELIBERATE SLAM restart (the dashboard's RESET MAP): the
        correction history of the old map says nothing about the new one, and
        the first transform of the new map must not read as a loop closure
        against the last transform of the old.
        """
        with self._lock:
            self._current = None
            self._first_seen = None
            self._last_seen = None
            self._samples.clear()
            self._jumps.clear()
            self._jump_count = 0
            self._max_correction_m = 0.0

    def update(self, x: float, y: float, yaw: float, now: float) -> Optional[Dict[str, Any]]:
        """Record one `map -> odom` sample. Returns the jump it detected, if any.

        Non-finite samples are DISCARDED rather than stored. A NaN in the
        correction would poison the magnitude, the maximum and every
        subsequent delta — and a dashboard that reports NaN where it used to
        report a distance tells the operator nothing except that something is
        broken somewhere, which is the least useful true statement available.
        """
        if not all(math.isfinite(v) for v in (x, y, yaw, now)):
            return None

        with self._lock:
            previous = self._current
            self._current = (x, y, yaw)
            self._last_seen = now
            if self._first_seen is None:
                self._first_seen = now

            magnitude = math.hypot(x, y)
            self._samples.append((now, magnitude))
            self._max_correction_m = max(self._max_correction_m, magnitude)

            if previous is None:
                # The FIRST correction is not a jump. It is the transform
                # appearing, which is SLAM starting up — reporting it as a
                # loop closure would put a spurious event at the head of every
                # session's history.
                return None

            step_m = math.hypot(x - previous[0], y - previous[1])
            step_yaw = abs(_wrap(yaw - previous[2]))
            if step_m < self._jump_translation_m and step_yaw < self._jump_yaw_rad:
                return None

            self._jump_count += 1
            jump = {
                "at": now,
                "step_m": round(step_m, 4),
                "step_yaw_rad": round(step_yaw, 4),
            }
            self._jumps.append(jump)
            return jump

    def snapshot(self, now: float, max_age_s: float) -> Dict[str, Any]:
        """A JSON-shaped summary.

        `max_age_s` decides `stale`. It is passed in rather than stored so the
        caller can tie it to the transform's own expected rate; this module
        holds no opinion about how often SLAM should publish.
        """
        with self._lock:
            if self._current is None:
                return {
                    "available": False,
                    "x": None, "y": None, "yaw": None,
                    "correction_m": None,
                    "max_correction_m": None,
                    "age_s": None,
                    "stale": True,
                    "jump_count": 0,
                    "jumps": [],
                    "trend_m": None,
                    "samples": [],
                }

            x, y, yaw = self._current
            age = None if self._last_seen is None else max(0.0, now - self._last_seen)
            return {
                "available": True,
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(yaw, 4),
                "correction_m": round(math.hypot(x, y), 4),
                "max_correction_m": round(self._max_correction_m, 4),
                "age_s": None if age is None else round(age, 3),
                "stale": age is None or age > max_age_s,
                "jump_count": self._jump_count,
                "jumps": list(self._jumps),
                # How much the correction has grown over the retained window.
                # Positive means SLAM is having to undo more odometry error
                # than it was; near zero over a driven run means it is not
                # correcting at all.
                "trend_m": self._trend_locked(),
                # Down-sampled magnitude history for a sparkline. Capped so a
                # long session cannot inflate every snapshot.
                "samples": [round(m, 3) for _, m in list(self._samples)[-60:]],
            }

    def _trend_locked(self) -> Optional[float]:
        """Change in correction magnitude across the retained window.

        Caller must hold the lock. None with fewer than two samples: one
        sample is not a trend, and reporting 0.0 from one sample would be a
        claim that nothing is changing.
        """
        if len(self._samples) < 2:
            return None
        return round(self._samples[-1][1] - self._samples[0][1], 4)


def _wrap(angle: float) -> float:
    """Wrap to (-pi, pi], so a yaw crossing +-pi is not a 6.28 rad step.

    Without this every correction that happens to straddle the wrap point
    registers as an enormous loop closure, and the jump count becomes a
    function of which way the robot is pointing.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def summarize_map(
    width: int,
    height: int,
    resolution: float,
    known_cells: int,
    occupied_cells: int,
) -> Dict[str, Any]:
    """Reduce an occupancy grid to the numbers worth watching live.

    Deliberately NOT the grid itself — that is fetched separately, on demand,
    because a 1000x1000 map is a megabyte and this summary rides a 10 Hz
    stream. What an operator watches here is COVERAGE GROWING: a map whose
    known-cell count has stopped rising while the robot drives is a map that
    has stopped being built, which is the silent-rejection failure again.
    """
    cells = max(0, int(width) * int(height))
    area = cells * resolution * resolution if resolution > 0 else 0.0
    return {
        "width": int(width),
        "height": int(height),
        "resolution_m": round(float(resolution), 4),
        "cells": cells,
        "known_cells": int(known_cells),
        "occupied_cells": int(occupied_cells),
        "known_fraction": round(known_cells / cells, 4) if cells else 0.0,
        "known_area_m2": round(known_cells * resolution * resolution, 2)
        if resolution > 0 else 0.0,
        "extent_m2": round(area, 2),
    }


def pack_grid(
    data: Any, width: int, height: int, max_cells: int
) -> Dict[str, Any]:
    """Decimate an occupancy grid and base64 it, one byte per output cell.

    DECIMATION TAKES THE MOST OCCUPIED CELL IN EACH BLOCK, not the average or
    a sample. On a map at 0.05 m resolution a wall is one or two cells thick,
    so averaging or point-sampling deletes walls at any real decimation
    factor — and a map picture that has lost its walls looks like a room the
    robot can drive through. Erring toward "occupied" is also the right
    direction for a diagnostic a human is about to drive by.
    """
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        return {"width": 0, "height": 0, "step": 1, "data": ""}

    step = max(1, math.ceil(max(width, height) / max(1, max_cells)))
    out_w = math.ceil(width / step)
    out_h = math.ceil(height / step)

    packed = bytearray(out_w * out_h)
    for oy in range(out_h):
        y0 = oy * step
        y1 = min(y0 + step, height)
        row_base = oy * out_w
        for ox in range(out_w):
            x0 = ox * step
            x1 = min(x0 + step, width)
            best = -1
            for y in range(y0, y1):
                offset = y * width
                block = data[offset + x0:offset + x1]
                if block:
                    local = max(block)
                    if local > best:
                        best = local
                        if best >= OCCUPIED_THRESHOLD:
                            break
            if best < 0:
                packed[row_base + ox] = CELL_UNKNOWN
            elif best >= OCCUPIED_THRESHOLD:
                packed[row_base + ox] = CELL_OCCUPIED
            else:
                packed[row_base + ox] = CELL_FREE

    return {
        "width": out_w,
        "height": out_h,
        # Cells of the ORIGINAL grid per output cell. The browser needs it to
        # convert a click back into map metres, so it must travel with the
        # image rather than being assumed on either side.
        "step": step,
        "data": base64.b64encode(bytes(packed)).decode("ascii"),
    }
