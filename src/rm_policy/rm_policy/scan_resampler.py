"""
scan_resampler.py — Pure conversion of one raw LaserScan into a fixed-length
forward-arc range vector for a NCP/policy consumer.

What it does:
    The YDLIDAR TG30 publishes ~1982-2020 points over a full 360 degrees at a
    resolution the trained policy neither expects nor was trained on. This
    module bins the forward arc of one raw scan into a caller-specified number
    of equal-angle sectors, taking the MINIMUM range in each sector (never the
    mean — a narrow obstacle must never be averaged away), and returns the
    result in METERS, UNNORMALIZED. It is the only thing standing between raw
    scans and any policy/NCP inference code, and is deliberately parameterized
    (n_bins, arc_deg, range_min, range_max) rather than hardcoding either the
    36-bin rm_policy contract or the NCP repo's own bin count, since the two
    consumers may want different vector lengths from the same physical scan.

Subscribes to:  (nothing — pure module; the caller passes plain arrays/floats)
Publishes:      (nothing — pure module; the caller acts on the return value)

THE SAFETY-CRITICAL PART — no-return handling:
    The TG30 encodes a no-return as 0.0, NOT inf, regardless of the driver's
    `invalid_range_is_inf` parameter (measured live, 2026-08-12/2026-08-27:
    1982 points, 781 zeros (39.4%), 0 inf). An untreated 0.0 reads as an
    obstacle at zero distance, min-pools to 0, and would latch any downstream
    e-stop in every bin containing a no-return. This module maps ALL of the
    following to `range_max` before pooling: exactly 0.0, any value strictly
    less than `range_min`, negative values, +inf, -inf, and NaN. See
    `_is_no_return()`.

Contract ownership:
    This module is deliberately parameterized and has NO opinion on which
    `n_bins` / `arc_deg` / `range_min` / `range_max` are "the contract" — it
    cannot detect an interface_version violation. The CALLER (the future
    observation_builder) is responsible for sourcing those from the shared
    schema loader / `config/interface.yaml`. A reviewer checking contract
    conformance must look at the caller, not this file.

Failure mode:
    Fail-closed, following `rm_safety.safety_logic`'s discipline (though NOT
    its range_min boundary — see `_is_no_return`). Any
    unexpected input (empty/None ranges, non-finite or non-positive
    angle_increment, n_bins <= 0, range_max <= range_min, an arc wider than
    the scan physically covers, or an unhandled exception anywhere in
    `resample_forward_arc`) produces a `ScanResampleResult` with `fault` set
    to a non-None reason string and `bins` set to `None`. The caller MUST
    check `fault` before touching `bins` — there is no silent all-range_max
    fallback for a whole-scan failure, matching this module's
    `empty_bin_policy: fault` convention from `config/interface.yaml` (a bin
    with zero member rays is a sensor fault, not evidence of a clear path, and
    is therefore never silently filled).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ScanResampleResult:
    """Output of `resample_forward_arc`.

    Parameters
    ----------
    bins:
        `n_bins` range values in meters, unnormalized, ordered right -> left
        (ascending angle; bin 0 is the most negative/starboard angle). `None`
        whenever `fault` is set — the caller must check `fault` first and
        must not treat `None` as "zero range" or otherwise substitute a
        default; that would be exactly the fail-open behaviour this module
        exists to prevent.
    fault:
        `None` on success. Otherwise one of the reason strings documented on
        `resample_forward_arc`, identifying which precondition failed or
        which bin(s) received zero member rays.
    """

    bins: Optional[List[float]]
    fault: Optional[str]


# ---------------------------------------------------------------------------
# No-return detection
# ---------------------------------------------------------------------------


def _is_no_return(r: float, range_min: float) -> bool:
    """True when a raw range reading must be treated as "no return".

    Covers exactly 0.0, any value < range_min (including negatives), and any
    non-finite value (+inf, -inf, NaN). This is the TG30-specific safety fix:
    the sensor emits 0.0 for a no-return regardless of the driver's
    `invalid_range_is_inf` parameter, so `math.isfinite` alone is NOT
    sufficient here — see the module docstring.

    BOUNDARY, DELIBERATE (2026-08-27 review): the test here is `r < range_min`,
    so a ray reading EXACTLY `range_min` is kept as real data. This diverges
    from `rm_safety.safety_logic._sensor_valid`, which requires
    `r > scan_range_min` (strict) and therefore DISCARDS a boundary-exact ray.
    The divergence is intentional and the two are not being reconciled:
      * here, keeping it is the conservative direction — the policy sees a very
        close obstacle rather than being told the path is clear;
      * there, discarding it is the established, signed-off behaviour backed by
        the rm_safety unit + replay tiers, and is not changed casually.
    sensor_msgs/LaserScan treats [range_min, range_max] as inclusive, so this
    module follows the message spec and safety_logic is the stricter one.
    If either side's no-return logic is ever touched, re-check this boundary.
    """
    if not math.isfinite(r):
        return True
    if r < range_min:
        return True
    return False


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def resample_forward_arc(
    ranges: Optional[Sequence[float]],
    angle_min: float,
    angle_increment: float,
    *,
    n_bins: int,
    arc_deg: float,
    range_min: float,
    range_max: float,
) -> ScanResampleResult:
    """Bin one raw LaserScan's ranges into `n_bins` forward-arc sectors.

    Geometry: beam `i` is at angle `angle_min + i * angle_increment` radians.
    Forward is 0 rad (REP-103: +x forward, +y left). The forward arc of
    `arc_deg` degrees is centered on 0, i.e. beams with (wrapped) angle in
    `[-arc_deg/2, +arc_deg/2]` degrees are selected; beams outside the arc are
    discarded. `n_bins` equal-width sectors are then laid across that arc.
    Output ordering is right -> left, i.e. ascending angle: `bins[0]` covers
    the most negative (starboard) angles, `bins[n_bins - 1]` the most positive
    (port) angles. The input array is NOT assumed to start at forward — the
    TG30 has `angle_min == -pi` — so angles are wrapped to `(-pi, pi]` before
    the arc/bin comparison, matching `rm_safety.safety_logic.min_range_in_arcs`
    (the angle-wrap formula is shared; the range-validity boundary is not).

    No-return handling (safety-critical, see module docstring): any ray with
    range exactly 0.0, < range_min, negative, or non-finite (inf/-inf/NaN) is
    replaced with `range_max` before pooling. This must happen BEFORE the
    min-pool step, never after, or a single genuine no-return would still
    read as a zero-distance obstacle.

    Pooling: each bin takes the MINIMUM range among its member rays (never the
    mean), so a narrow obstacle is never averaged away.

    Empty-bin policy: `fault` (never `range_max`, per `config/interface.yaml`
    `empty_bin_policy: fault`). If any bin ends up with zero member rays after
    the arc selection — e.g. `n_bins` too fine for the sensor's angular
    resolution, or the scan is malformed — this is a sensor/config fault, not
    a "clear path" reading, and the whole call fails closed.

    Parameters
    ----------
    ranges:
        Raw range array from one LaserScan message (meters). May be `None` or
        empty; treated as a fault.
    angle_min:
        Angle of `ranges[0]`, radians.
    angle_increment:
        Angular step between consecutive elements, radians. Must be finite
        and strictly positive.
    n_bins:
        Number of output sectors. Must be a positive integer. Not hardcoded
        by this module — callers pass 36 (rm_policy's own contract) or
        whatever the NCP model's own input layer expects.
    arc_deg:
        Total forward arc width in degrees, centered on 0 rad. Must be > 0
        and <= 360, and must not exceed what the scan's `angle_min`/
        `angle_increment`/length can actually cover.
    range_min:
        Sensor minimum valid range, meters (LaserScan `range_min`, or the
        stricter interface-contract value — the caller decides).
    range_max:
        Sensor/contract maximum valid range, meters. No-return rays are
        mapped to this value. Must be > `range_min`.

    Returns
    -------
    ScanResampleResult:
        `bins` populated (meters, unnormalized) on success; `fault` set and
        `bins` `None` on any precondition failure, empty bin, or unexpected
        exception. Never raises.

    Fault reasons
    -------------
    "empty_ranges"          - `ranges` is `None` or has zero length.
    "invalid_angle_increment" - `angle_increment` is <= 0, non-finite, or
                                  `angle_min` is non-finite.
    "invalid_n_bins"        - `n_bins` <= 0.
    "invalid_arc_deg"       - `arc_deg` <= 0 or > 360.
    "invalid_range_bounds"  - `range_max` <= `range_min`, or either is
                                  non-finite.
    "arc_exceeds_scan_coverage" - the requested forward arc is wider than the
                                  angular span the scan array actually covers.
    "empty_bin:<index>"     - bin `<index>` (0-based) received zero member
                                  rays after arc selection.
    "internal_error"        - an unexpected exception was caught; fail closed
                                  exactly like `rm_safety.safety_logic.evaluate`.
    """
    try:
        return _resample_forward_arc_inner(
            ranges,
            angle_min,
            angle_increment,
            n_bins=n_bins,
            arc_deg=arc_deg,
            range_min=range_min,
            range_max=range_max,
        )
    except Exception:
        return ScanResampleResult(bins=None, fault="internal_error")


def _resample_forward_arc_inner(
    ranges: Optional[Sequence[float]],
    angle_min: float,
    angle_increment: float,
    *,
    n_bins: int,
    arc_deg: float,
    range_min: float,
    range_max: float,
) -> ScanResampleResult:
    """Inner implementation of `resample_forward_arc`; exceptions propagate."""

    if ranges is None or len(ranges) == 0:
        return ScanResampleResult(bins=None, fault="empty_ranges")

    if (
        not math.isfinite(angle_increment)
        or angle_increment <= 0.0
        or not math.isfinite(angle_min)
    ):
        return ScanResampleResult(bins=None, fault="invalid_angle_increment")

    if not isinstance(n_bins, int) or n_bins <= 0:
        return ScanResampleResult(bins=None, fault="invalid_n_bins")

    if not math.isfinite(arc_deg) or arc_deg <= 0.0 or arc_deg > 360.0:
        return ScanResampleResult(bins=None, fault="invalid_arc_deg")

    if (
        not math.isfinite(range_min)
        or not math.isfinite(range_max)
        or range_max <= range_min
    ):
        return ScanResampleResult(bins=None, fault="invalid_range_bounds")

    n_points = len(ranges)
    scan_span_rad = (n_points - 1) * angle_increment
    arc_rad = math.radians(arc_deg)
    if scan_span_rad < arc_rad:
        return ScanResampleResult(bins=None, fault="arc_exceeds_scan_coverage")

    half_arc = arc_rad / 2.0
    bin_width = arc_rad / n_bins

    # bin_sums / bin_counts hold the running min and member-ray count per bin.
    bin_min: List[float] = [math.inf] * n_bins
    bin_count: List[int] = [0] * n_bins

    for i in range(n_points):
        raw_angle = angle_min + i * angle_increment
        # Wrap to (-pi, pi], matching rm_safety.safety_logic.min_range_in_arcs.
        angle = (raw_angle + math.pi) % (2.0 * math.pi) - math.pi

        if angle < -half_arc or angle > half_arc:
            continue

        # Map angle in [-half_arc, +half_arc] to a bin index in [0, n_bins).
        # Ascending angle -> ascending bin index gives right(-)->left(+)
        # ordering, per REP-103 (+y left).
        idx = int((angle + half_arc) / bin_width)
        if idx >= n_bins:
            # Beam exactly at +half_arc lands one past the last bin; clamp.
            idx = n_bins - 1
        if idx < 0:
            idx = 0

        r = ranges[i]
        r_effective = range_max if _is_no_return(r, range_min) else r
        # Also clip any absurdly-large-but-finite return to range_max, so a
        # sensor value beyond spec never leaks past the contract ceiling.
        if r_effective > range_max:
            r_effective = range_max

        bin_count[idx] += 1
        if r_effective < bin_min[idx]:
            bin_min[idx] = r_effective

    for idx in range(n_bins):
        if bin_count[idx] == 0:
            return ScanResampleResult(bins=None, fault=f"empty_bin:{idx}")

    return ScanResampleResult(bins=bin_min, fault=None)
