---
name: pattern-slow-stop-arc-split
description: How safety_logic splits slow-zone vs stop-zone monitored arcs (2026-08-30) without weakening the stop zone — the subset-invariant argument and what to re-check on future edits to this area.
metadata:
  type: project
---

`src/rm_safety/rm_safety/safety_logic.py` computes two separate minimum
ranges per cycle: `min_r` (wide arcs — `forward_arc_deg`/`lateral_arc_deg`,
governs the stop zone, the latch, and `/safety/status.min_range`) and
`min_r_slow` (narrow arcs — optional `slow_forward_arc_deg`/
`slow_lateral_arc_deg`, governs velocity scaling only). This was added
because a single shared arc pair meant lateral returns (walls, furniture
beside the robot) throttled forward velocity with a clear path ahead.

**Why the subset invariant (`min_r_slow >= min_r`) holds by construction, not
just by config validation:** both calls to `min_range_in_arcs()` run over the
*same* cleaned_ranges list with the *same* self-occlusion exclusion (that
exclusion is config-driven, not parameterized per call), and both center the
forward/lateral windows on the same bearings (0°, ±90°). So the narrow-arc
match set is always a geometric subset of the wide-arc match set whenever the
narrow half-angles are <= the wide ones — `Limits.from_dict` additionally
enforces this at load time (fatal ValueError, not silently clamped) so a
misconfigured YAML can't invert it. A lateral arc of `0.0` is a special case
meaning "disabled", not a hairline arc that flakes on rays landing exactly at
±90° — checked via `lateral_enabled = lat_deg > 0.0`, not `half_lat == 0`.

**The critical wiring to check whenever this code is touched again:**
`estop_zone()`'s STOP test always uses the wide `min_range` argument (never
the slow one) — zone `"stop"` membership, and therefore the latch, is
unaffected by how narrow the slow cone is. In the velocity-scaling branch,
`slow_scale(min_r if zone == "stop" else min_r_slow, limits)` — this
conditional is what preserves the 2026-08-14 pre-latch-scales-to-zero
decision ([[pattern_angle_only_masking]] is a related but distinct masking
concept — that one is angle-only blind spots, this one is a wide/narrow arc
*pair* for two different zones, not a mask). Getting that ternary backwards
(using `min_r_slow` unconditionally) would let a stop-zone obstacle *outside*
the narrow cone emit full velocity for the entire pre-latch window — this is
the single most important line to re-check on any future edit near here.

**How to apply:** when reviewing a future change to arc geometry, (1) confirm
`min_range` reported to `/safety/status` still traces to the wide arc, (2)
confirm the stop/latch decision (`is_stop_zone`) is computed from the wide
arc, (3) re-derive or re-run the subset-invariant sweep test
(`tests/unit/test_safety_arc_split.py::test_slow_arc_minimum_is_never_below_the_wide_arc_minimum`)
rather than trusting it holds, (4) check `docs/safety_validation.md` — as of
2026-08-30 it was NOT updated alongside this change (same gap noted in
[[pattern_angle_only_masking]] for occlusion masks), so verify it's been
updated by the time this lands, and (5) the geometry is a fixed-angle cone
approximation of a rectangular corridor and is only sized correctly at one
radius (the slow radius) — if a review ever needs it right across a range,
the fix is a lateral-offset test (`|r*sin(theta)| < half_width`), not a
bigger angle.
