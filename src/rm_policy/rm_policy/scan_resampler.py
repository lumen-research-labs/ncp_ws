"""
scan_resampler.py — Pure conversion of one raw LaserScan into a fixed-length
forward-arc range vector for a NCP/policy consumer.

What it does:
    The YDLIDAR TG30 publishes ~1982-2020 points over a full 360 degrees at a
    resolution the trained policy neither expects nor was trained on. This
    module bins the forward arc of one raw scan into a caller-specified number
    of equal-angle sectors and reduces each sector to ONE range: the real
    return nearest the sector centre (default, training-faithful) or the
    sector minimum (never the mean — a narrow obstacle must never be averaged
    away). It returns METERS, UNNORMALIZED. It is the only thing standing
    between raw scans and any policy/NCP inference code, and is deliberately
    parameterized (n_bins, arc_deg, range_min, range_max) rather than
    hardcoding any one contract's bin count, since consumers may want
    different vector lengths from the same physical scan.

    Two options added for interface 4.0.0 (720 bins over 270 deg, ~2 TG30
    rays per bin), both off by default so older callers are unchanged:
    `search_window_bins` lets a bin with no real return borrow the nearest
    real return from neighbouring bins, and `mask_zones_deg` blanks angular
    sectors where the robot sees its own chassis. See `resample_forward_arc`.

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

import bisect
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

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


# Sampling modes. See `resample_forward_arc`.
#
# TRAINING FIDELITY (governing principle, operator 2026-08-27): the policy was
# trained on a simulated lidar with exactly ONE ray per beam direction (32 over
# 180 deg for the M4 checkpoint, 720 over 270 deg since interface 4.0.0). The
# TG30 delivers more rays than that across the same arc (~31 per bin at 32
# bins, ~2 at 720), and they must be reduced somehow. The two ways differ in a
# way that matters:
#
#   SAMPLING_NEAREST_RAY (default) — take the single REAL RETURN closest to the
#       bin's CENTRE angle, discarding the rest. This reconstructs what a clean
#       one-ray-per-bin lidar pointed in that direction would have reported, so the
#       observation distribution matches training. A thin obstacle between
#       sampled rays is invisible to the POLICY — exactly as it was invisible
#       in simulation.
#
#       "REAL RETURN" is load-bearing. A naive nearest-ray that accepts the
#       centre ray whatever it is would inherit the TG30's ~39% dropout rate:
#       measured on tg30_open_scan.yaml, 5 of 32 bins (16%) have a DROPOUT as
#       their centre ray, and would therefore be reported as range_max
#       ("clear") while neighbouring rays in the same bin see a real obstacle
#       at 0.5-3 m. Simulation had NO dropout — every sim ray returns — so that
#       failure mode did not exist in training and must not be introduced here.
#       Skipping dropouts in favour of the nearest real return is therefore the
#       MORE training-faithful choice, not a safety compromise bolted on.
#       If a bin contains no real return at all, it falls back to range_max:
#       genuinely nothing was detected anywhere in that sector.
#
#   SAMPLING_MIN_POOL — take the nearest return anywhere in the bin's cone.
#       Strictly more conservative, and the right choice for a SAFETY input,
#       but it reports obstacles as closer and wider than the training lidar
#       would, which is a systematic shift away from what the network saw in
#       training.
#
# We can afford the faithful option because this module feeds the POLICY only.
# rm_safety independently computes its e-stop from the FULL-RESOLUTION scan
# (see safety_logic.min_range_in_arcs), so an obstacle the policy misses
# between rays is still caught by the safety layer. Never wire this module's
# output into a safety path; that is what the full scan is for.
SAMPLING_NEAREST_RAY = "nearest_ray"
SAMPLING_MIN_POOL = "min_pool"
_VALID_SAMPLING = (SAMPLING_NEAREST_RAY, SAMPLING_MIN_POOL)


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
    sampling: str = SAMPLING_NEAREST_RAY,
    search_window_bins: int = 0,
    mask_zones_deg: Sequence[Sequence[float]] = (),
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
    `empty_bin_policy: fault`). A bin with zero member rays after the arc
    selection is first offered the search window (2026-11-18): a sensor
    coarser than the bin grid (the RPLIDAR A1's 1 deg rays against 0.375 deg
    bins) leaves most bins without a member of their own, and the nearest
    ray within reach serves them exactly as it serves a bin whose members
    are all no-returns; a no-return within reach keeps the bin at
    range_max. Only a bin with NO ray within reach (an angular gap in the
    scan, a malformed scan, or strict mode with `search_window_bins` 0) is
    a sensor/config fault, not a "clear path" reading, and the whole call
    fails closed.

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
    search_window_bins:
        SAMPLING_NEAREST_RAY only. A bin that contains no real return may
        fall back to the real return nearest its centre within this many
        neighbouring bins on either side, i.e. any ray with
        |angle - centre| <= (search_window_bins + 0.5) * bin_width, including
        rays just outside the arc edge. 0 (default) disables the fallback and
        reproduces the strict per-bin behaviour exactly. Rays inside the bin
        always win over the fallback. Why it exists: at 720 bins over 270 deg
        the TG30 contributes ~2 rays per bin with ~30% dropout, so 5-8% of
        bins would otherwise read range_max ("clear") while a real return
        sits a fraction of a degree away; +/-2 bins cuts that to ~1.3% on the
        recorded fixtures. Simulation had no dropout at all. Ignored in
        SAMPLING_MIN_POOL mode, which already uses every ray in the bin.
    mask_zones_deg:
        Angular sectors `[lo_deg, hi_deg]` (wrapped angle, REP-103, inclusive)
        whose rays are treated as no-returns in BOTH sampling modes: they
        still count as member rays (so they never cause an empty_bin fault)
        but never contribute a range and never feed the search-window
        fallback. For chassis self-hits the training sensor could not see:
        its 0.1 m min_range put the chassis inside a dead zone, so those
        directions read "clear" in simulation. The caller sources the list
        from config/interface.yaml `self_occlusion_zones_deg`.

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
    "invalid_search_window" - `search_window_bins` is not a non-negative int.
    "invalid_mask_zone"     - a mask zone is not a `[lo, hi]` pair of finite
                                  degrees with lo <= hi.
    "empty_bin:<index>"     - bin `<index>` (0-based) received zero member
                                  rays after arc selection and, with a
                                  search window, no ray lies within reach.
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
            sampling=sampling,
            search_window_bins=search_window_bins,
            mask_zones_deg=mask_zones_deg,
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
    sampling: str = SAMPLING_NEAREST_RAY,
    search_window_bins: int = 0,
    mask_zones_deg: Sequence[Sequence[float]] = (),
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

    if sampling not in _VALID_SAMPLING:
        return ScanResampleResult(bins=None, fault="invalid_sampling_mode")

    if not isinstance(search_window_bins, int) or search_window_bins < 0:
        return ScanResampleResult(bins=None, fault="invalid_search_window")

    zones_rad: List[Tuple[float, float]] = []
    for zone in mask_zones_deg:
        try:
            if len(zone) != 2:
                raise ValueError
            lo, hi = float(zone[0]), float(zone[1])
        except (TypeError, ValueError):
            return ScanResampleResult(bins=None, fault="invalid_mask_zone")
        if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi:
            return ScanResampleResult(bins=None, fault="invalid_mask_zone")
        zones_rad.append((math.radians(lo), math.radians(hi)))

    n_points = len(ranges)
    scan_span_rad = (n_points - 1) * angle_increment
    arc_rad = math.radians(arc_deg)
    if scan_span_rad < arc_rad:
        return ScanResampleResult(bins=None, fault="arc_exceeds_scan_coverage")

    half_arc = arc_rad / 2.0
    bin_width = arc_rad / n_bins

    # The search-window fallback may borrow rays up to `window_rad` beyond the
    # arc edge, so candidate collection uses a slightly wider arc. Capped at
    # pi because angles are wrapped to (-pi, pi].
    window_rad = (
        search_window_bins * bin_width
        if sampling == SAMPLING_NEAREST_RAY and search_window_bins > 0
        else 0.0
    )
    ext_half = min(half_arc + window_rad, math.pi)

    # bin_min  : running minimum range (SAMPLING_MIN_POOL)
    # bin_pick  : range of the ray nearest the bin centre (SAMPLING_NEAREST_RAY)
    # bin_best  : |angle - bin_centre| of that ray, for comparison
    # bin_count : member-ray count, used to enforce empty_bin_policy: fault
    bin_min: List[float] = [math.inf] * n_bins
    bin_pick: List[float] = [math.inf] * n_bins
    bin_best: List[float] = [math.inf] * n_bins
    bin_count: List[int] = [0] * n_bins
    ray_angles: List[float] = []          # every in-reach ray, usable or not
    # (angle, range) of every real, unmasked ray in the extended arc — the
    # pool the search-window fallback draws from. Empty when the window is 0.
    candidates: List[Tuple[float, float]] = []

    for i in range(n_points):
        raw_angle = angle_min + i * angle_increment
        # Wrap to (-pi, pi], matching rm_safety.safety_logic.min_range_in_arcs.
        angle = (raw_angle + math.pi) % (2.0 * math.pi) - math.pi

        if angle < -ext_half or angle > ext_half:
            continue

        r = ranges[i]
        ray_angles.append(angle)
        # A masked ray is a no-return for every purpose below: the robot is
        # looking at itself, and the training sensor saw "clear" there.
        usable = not _is_no_return(r, range_min) and not any(
            lo <= angle <= hi for lo, hi in zones_rad
        )
        r_effective = r if usable else range_max
        # Also clip any absurdly-large-but-finite return to range_max, so a
        # sensor value beyond spec never leaks past the contract ceiling.
        if r_effective > range_max:
            r_effective = range_max

        if usable and window_rad > 0.0:
            candidates.append((angle, r_effective))

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

        bin_count[idx] += 1
        if r_effective < bin_min[idx]:
            bin_min[idx] = r_effective

        # Nearest-to-centre bookkeeping: which single ray a CLEAN lidar with
        # one beam per bin, pointed at this bin's centre, would have returned.
        # Dropouts and masked rays are skipped rather than accepted — see
        # SAMPLING_NEAREST_RAY above — so neither ever wins the centre slot.
        if usable:
            bin_centre = -half_arc + (idx + 0.5) * bin_width
            offset = abs(angle - bin_centre)
            if offset < bin_best[idx]:
                bin_best[idx] = offset
                bin_pick[idx] = r_effective

    # A bin without a member ray of its own: a fault in strict mode, and with
    # a search window only when no ray at all (no-returns included) lies
    # within reach. The window fallback below then fills it from the nearest
    # usable ray, or it reads range_max like any bin whose rays are all
    # no-returns. See the docstring's empty-bin policy.
    if window_rad > 0.0:
        ray_angles.sort()
        reach = window_rad + bin_width / 2.0
        for idx in range(n_bins):
            if bin_count[idx] != 0:
                continue
            bin_centre = -half_arc + (idx + 0.5) * bin_width
            j = bisect.bisect_left(ray_angles, bin_centre)
            if not any(0 <= k < len(ray_angles) and abs(ray_angles[k] - bin_centre) <= reach
                       for k in (j - 1, j)):
                return ScanResampleResult(bins=None, fault=f"empty_bin:{idx}")
    else:
        for idx in range(n_bins):
            if bin_count[idx] == 0:
                return ScanResampleResult(bins=None, fault=f"empty_bin:{idx}")

    if sampling == SAMPLING_MIN_POOL:
        return ScanResampleResult(bins=bin_min, fault=None)

    if window_rad > 0.0 and candidates:
        # Fallback for bins that have rays but no real, unmasked return: take
        # the candidate nearest the bin centre within reach. Candidates are
        # sorted by angle, so the nearest is one of the two around the
        # insertion point. Bins that already picked a ray are untouched.
        candidates.sort()
        cand_angles = [a for a, _ in candidates]
        reach = window_rad + bin_width / 2.0
        for idx in range(n_bins):
            if not math.isinf(bin_pick[idx]):
                continue
            bin_centre = -half_arc + (idx + 0.5) * bin_width
            j = bisect.bisect_left(cand_angles, bin_centre)
            best_offset = math.inf
            for k in (j - 1, j):
                if 0 <= k < len(candidates):
                    offset = abs(cand_angles[k] - bin_centre)
                    if offset <= reach and offset < best_offset:
                        best_offset = offset
                        bin_pick[idx] = candidates[k][1]

    # A bin with rays but no real return anywhere in reach: nothing was
    # detected in that sector, which is range_max. bin_pick is still inf
    # there because no candidate ever won the centre slot.
    selected = [range_max if math.isinf(v) else v for v in bin_pick]
    return ScanResampleResult(bins=selected, fault=None)
