# Vendored policy code — provenance and verification

`ncp_policy.py` is a **byte-exact, unmodified copy** of the trained policy's
ROS-free module. Operator requirement, 2026-08-27: *"the policy code should be
as similar to, if not an exact copy of, what's on the NCP-on-Rosmaster repo."*

| | |
|---|---|
| Source repo | `LumenLabs/NCP-on-Rosmaster` (local checkout `~/lumen/ncp`) |
| Source path | `rosmaster/policy.py` |
| Commit | `bc1099d43831db3b228b2b7ea3baf4e267b9dc4c` (branch `feat/720-beam-270fov`, clean tree) |
| Vendored | 2026-10-08 (re-vendored; previously 2026-08-29 @ `69785c8`, first 2026-08-27 @ `0ad548c`) |

## Re-vendoring 2026-10-08 — interface 4.0.0, 720 beams over 270°

Only two constants changed: `BEAMS` 32 → 720 and `FOV_DEG` 180 → 270, the native BARN UST-10LX layout the M7 and M8 runs trained on. `OBS_DIM` follows (36 → 724). The `NcpActor` architecture is unchanged; its input layer width follows `OBS_DIM`. `weights/actor.pt` upstream is now the M8-v2 snapshot `s1436792` (63% held-out success, official BARN 0.219), sha256 `71cc50aeb734…`, exported on the training box by the generalized `weights/export_from_sb3.py` with max action difference against SB3 of 0.0.

A 3.0.0 checkpoint can no longer load: `load_state_dict` is strict and raises at node startup, never driving on a shape mismatch.

## Re-vendoring 2026-08-29 — the actor architecture CHANGED

The first vendoring took `0ad548c`, which predated the trained checkpoint. When
`weights/actor.pt` landed upstream (`a9af7af`), `NcpActor` changed shape:

| | `0ad548c` (old) | `69785c8` (now) |
|---|---|---|
| `head` | `nn.Linear(motor, 2)` | `Linear(128,64)-Tanh-Linear(64,64)-Tanh-Linear(64,2)` — the SB3 `mlp_extractor.policy_net` + `action_net`, tanh being SB3's default `net_arch` activation |
| obs normalization | none | `obs_mean` / `obs_var` buffers (the checkpoint's VecNormalize statistics), applied in `act()` and clipped to ±10 (`OBS_CLIP`, VecNormalize's `clip_obs` at training time) |

`actor.pt` is a `state_dict` for the NEW shape. The old copy could not have
loaded it: `PolicyRunner` calls `load_state_dict` at its default `strict=True`,
so the mismatch raises a `RuntimeError` during node `__init__` — before any
subscription or timer exists — rather than degrading silently. That is the
designed behaviour and it is why this mismatch was a startup failure and never
a wrong-numbers-on-the-wire failure.

Two consequences worth holding onto:

* **`obs_mean`/`obs_var` ride inside the `.pt`.** The normalization the policy
  was trained under is carried by the checkpoint, not by our config. A run with
  no weights file gets the default buffers (mean 0, var 1), which pass
  observations through untouched — so random-weight plumbing runs and trained
  runs differ in the normalization applied, by design.
* **The clip is ±10 in NORMALIZED units**, applied after the VecNormalize
  transform. It is unrelated to `laser_clip_m: 5.0` in `config/interface.yaml`,
  which is applied earlier, in metres, by `build_obs`. Two different clips at
  two different stages — do not reconcile them.

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
