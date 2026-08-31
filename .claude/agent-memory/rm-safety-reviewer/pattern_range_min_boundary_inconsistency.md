---
name: pattern-range-min-boundary-inconsistency
description: rm_policy.scan_resampler and rm_safety.safety_logic disagree on whether a ray reading exactly range_min is valid data or a fault — check this boundary whenever either lidar-processing module changes.
metadata:
  type: project
---

`src/rm_safety/rm_safety/safety_logic.py`'s `_sensor_valid()` requires
`r > inputs.scan_range_min` (strict) — a ray reading exactly `range_min` is
treated as invalid and mapped to `inf` (excluded from the e-stop min-range
calc). `src/rm_policy/rm_policy/scan_resampler.py`'s `_is_no_return()` uses
`r < range_min` (strict) — a ray reading exactly `range_min` is treated as
**valid** real data and passed through unchanged into the observation.
Verified live 2026-08-27: a ray at exactly `range_min` produces a real
close-range bin value in scan_resampler, but would be excluded/ignored by
safety_logic's arc-min calculation.

**Why this matters:** both modules' docstrings claim to match each other's
conventions (scan_resampler explicitly cites `safety_logic.min_range_in_arcs`
for its angle-wrapping — that specific claim IS accurate, `(a + pi) % (2*pi)
- pi` is identical in both — but no equivalent claim is made, or holds, for
the range-validity boundary). It's not fail-open in scan_resampler itself
(treating range_min as a real reading is the more conservative direction:
the policy still sees the obstacle), so don't over-escalate it alone to
Critical — but it means the safety layer and the policy's observation can
disagree about whether a boundary-exact ray is "real" at all, and any future
module that assumes the two are interchangeable will be wrong.

**How to apply:** whenever either module's no-return/range-validity handling
changes, diff the boundary condition (`<` vs `<=` vs `>` vs `>=` against
range_min) explicitly against the other module, and check whether a unit
test exists for `r == range_min` exactly in both test suites — as of
2026-08-27 `tests/unit/test_scan_resampler.py` had no test for the exact
`range_min` boundary at all (only `range_min/2.0`, which is unambiguously
below it).
