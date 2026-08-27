# Vendored policy code — provenance and verification

`ncp_policy.py` is a **byte-exact, unmodified copy** of the trained policy's
ROS-free module. Operator requirement, 2026-08-27: *"the policy code should be
as similar to, if not an exact copy of, what's on the NCP-on-Rosmaster repo."*

| | |
|---|---|
| Source repo | `LumenLabs/NCP-on-Rosmaster` |
| Source path | `rosmaster/policy.py` |
| Commit | `0ad548c6236f2c52f8435cd58a9610604047f919` (branch `main`, clean tree) |
| Vendored | 2026-08-27 |

## Verifying it is still an exact copy

```
diff <upstream>/rosmaster/policy.py src/rm_policy/rm_policy/vendor/ncp_policy.py
```

`tests/unit/test_vendor_is_pristine.py` asserts this automatically wherever an
upstream checkout is present, and skips where it is not. **If that test fails,
someone has edited vendored code — revert it and fix the problem in our own
modules instead.**

## Why there are ZERO local divergences

An earlier revision carried one: `prepare_scan()` was patched to treat the
YDLIDAR TG30's `0.0` no-return sentinel as "no return", because the TG30 emits
`0.0` and never `inf` (measured on bernard-1: 1982 points, 781 zeros = 39.4%,
0 inf; the driver's `invalid_range_is_inf:=true` is inert). Untreated, a `0.0`
normalizes through `build_obs` to `-1.0` — "obstacle at zero distance" — across
~39% of the field of view.

That patch has been **removed**, and the protection moved upstream of the
policy into code we own: `rm_policy.scan_resampler` maps every no-return
(`0.0`, sub-`range_min`, negative, `inf`, `NaN`) to `range_max` *before* the
32-vector is built. Verified against real TG30 fixtures: the vector handed to
`build_obs` can never contain `0.0`. `prepare_scan` is therefore redundant on
our path and the vendored file needs no edits at all.

**Consequence to understand:** the vendored module is now only safe when fed by
`scan_resampler`. Calling `ncp_policy.prepare_scan()` directly on a raw TG30
scan would reproduce the original bug. That is upstream's behaviour, faithfully
preserved — it is not a defect in this copy, and it must not be "fixed" here.

## Re-syncing

Copy the file over and re-run the pristine test. Because there are no local
divergences, a re-sync is a plain copy with nothing to re-apply — which is the
entire point of keeping it exact.
