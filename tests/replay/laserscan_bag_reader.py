"""
laserscan_bag_reader.py — Minimal, ROS-free reader for LaserScan messages in a
rosbag2 (sqlite3) bag.

What it does:
    Opens a rosbag2 `*.db3` file directly with the stdlib `sqlite3` module and
    hand-decodes the CDR payload of every `sensor_msgs/msg/LaserScan` message on
    a given topic.  This lets the replay tests feed REAL recorded scans through
    `rm_safety.safety_logic.evaluate()` on the workstation, where there is no ROS
    runtime (ROS lives only in a container here — see CLAUDE.md working
    conventions).

Subscribes to:  (nothing — reads a file)
Publishes:      (nothing — returns plain Python dicts)

Why hand-roll CDR instead of using rosbag2_py / rclpy:
    The workstation has no `rclpy`, no message packages, and building the ROS
    stack here is explicitly out of bounds.  A LaserScan is a fixed, simple
    layout, so a ~40-line little-endian CDR decoder is enough and keeps the
    replay suite in the "no ROS runtime" tier alongside the unit tests.

Scope / assumptions (guarded, not silent):
    * Little-endian CDR only.  The encapsulation header is asserted to be
      `00 01 00 00` (CDR_LE); any other value raises, rather than silently
      misparsing.
    * Only the fields safety_logic needs are returned (angles, range bounds,
      ranges).  `intensities` is skipped.
    * `read_scans()` asserts a caller-supplied expectation (point count, frame)
      on the first message so a format drift fails loudly at the source instead
      of producing subtly wrong ranges downstream.

Failure mode:
    Fail-loud.  A malformed blob, a wrong encapsulation header, or a mismatch
    against the expected point count raises `AssertionError`/`struct.error`.
    This module is test-only; there is no safe-default to fall back to — a
    silently mis-decoded scan would invalidate the safety conclusions the
    replay tests draw, so it must raise instead.
"""

from __future__ import annotations

import sqlite3
import struct
from typing import Dict, Iterator, List, Optional

# CDR encapsulation header for little-endian, plain CDR.
_CDR_LE_HEADER = b"\x00\x01\x00\x00"


def _decode_laserscan_cdr(data: bytes) -> Dict:
    """Decode one LaserScan CDR payload into a plain dict.

    Returns keys: frame_id, angle_min, angle_max, angle_increment, range_min,
    range_max, ranges (List[float]).  Intensities are parsed past but dropped.

    CDR alignment note: every primitive is aligned to its own size relative to
    the start of the payload (the byte after the 4-byte encapsulation header),
    which is what `_align` tracks via `base`.
    """
    if data[:4] != _CDR_LE_HEADER:
        raise AssertionError(
            f"unexpected CDR encapsulation header {data[:4].hex()!r}; "
            f"this reader only supports little-endian CDR ({_CDR_LE_HEADER.hex()})"
        )

    offset = 4          # first payload byte, after encapsulation header
    base = 4            # alignment origin

    def align(n: int) -> None:
        nonlocal offset
        rel = (offset - base) % n
        if rel:
            offset += n - rel

    def u32() -> int:
        nonlocal offset
        align(4)
        v = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        return v

    def i32() -> int:
        nonlocal offset
        align(4)
        v = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        return v

    def f32() -> float:
        nonlocal offset
        align(4)
        v = struct.unpack_from("<f", data, offset)[0]
        offset += 4
        return v

    def string() -> str:
        nonlocal offset
        n = u32()  # length includes the trailing NUL
        s = data[offset : offset + n - 1].decode("utf-8", "replace")
        offset += n
        return s

    # std_msgs/Header
    _sec = i32()
    _nanosec = u32()
    frame_id = string()

    # LaserScan scalar fields
    angle_min = f32()
    angle_max = f32()
    angle_increment = f32()
    _time_increment = f32()
    _scan_time = f32()
    range_min = f32()
    range_max = f32()

    # ranges (seq<float32>)
    n_ranges = u32()
    ranges: List[float] = list(struct.unpack_from(f"<{n_ranges}f", data, offset))
    offset += 4 * n_ranges

    return {
        "frame_id": frame_id,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
    }


def read_scans(
    bag_db3_path: str,
    topic: str = "/scan",
    expect_points: Optional[int] = None,
    expect_frame: Optional[str] = None,
) -> Iterator[Dict]:
    """Yield every LaserScan on `topic` from a rosbag2 sqlite3 file, in time order.

    Parameters
    ----------
    bag_db3_path:
        Path to the `*.db3` file inside the bag directory.
    topic:
        Topic name to read (default ``/scan``).
    expect_points:
        If given, the first decoded scan must have exactly this many range
        points, else AssertionError.  Guards against CDR-format drift.
    expect_frame:
        If given, the first decoded scan's ``frame_id`` must match, else
        AssertionError.

    Yields
    ------
    dict:
        One decoded LaserScan per message (see ``_decode_laserscan_cdr``).
    """
    con = sqlite3.connect(bag_db3_path)
    try:
        rows = con.execute(
            "SELECT m.data FROM messages m "
            "JOIN topics t ON m.topic_id = t.id "
            "WHERE t.name = ? ORDER BY m.timestamp",
            (topic,),
        )
        first = True
        for (blob,) in rows:
            scan = _decode_laserscan_cdr(blob)
            if first:
                if expect_points is not None:
                    assert len(scan["ranges"]) == expect_points, (
                        f"expected {expect_points} points, "
                        f"got {len(scan['ranges'])}"
                    )
                if expect_frame is not None:
                    assert scan["frame_id"] == expect_frame, (
                        f"expected frame {expect_frame!r}, "
                        f"got {scan['frame_id']!r}"
                    )
                first = False
            yield scan
    finally:
        con.close()
