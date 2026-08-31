---
name: pattern-angle-only-masking
description: Angle-only self-occlusion masks (interface.yaml and safety_limits.yaml estop) create real angular blind spots, not just standoff filters — flag as residual risk requiring safety_validation.md sign-off
metadata:
  type: project
---

ncp_ws has two independent self-occlusion mask concepts that look similar but
serve different code paths: `config/interface.yaml` (lidar *observation*
input to the policy, forward-180 only) and `config/safety_limits.yaml`
`estop.self_occlusion_zones_deg` (the safety layer's own monitored arcs,
which extend to ±120 deg, wider than the observation's ±90). They are
deliberately independent — do not treat a change to one as covering the
other, and don't be surprised to see both touched/untouched independently
across commits (e.g. 9adf846 zeroed the interface.yaml mask after confirming
the camera doesn't occlude the forward plane, unrelated to the 2026-08-14
estop-side standoff mask added afterward for a different angular sector).

**Both masks work by angle range alone, not angle+range.** That means a
masked zone is a genuine blind spot for *any* obstacle at that bearing, not
just a filter for the known chassis artifact (standoff/camera body) that
motivated it — a real obstacle at the same bearing as a masked chassis
standoff would also go undetected, whatever its actual range. This is
probably correct/necessary (the known artifact's own return means the arc
was already shadowed at that exact bearing regardless of masking, so no real
capability is lost for objects at that specific angle) but it should always
be recorded explicitly in `docs/safety_validation.md` as an accepted blind
sector, the same way the rear hemisphere blind spot is documented there — as
of 2026-08-14 the estop standoff mask was NOT added to that doc, only to
inline yaml/code comments. Check for this gap on future occlusion-mask
changes.

**How to apply:** when reviewing any change to either mask, (1) confirm which
of the two configs changed and that the other wasn't assumed to cover it,
(2) check the mask bounds against the *monitored arc* bounds it lives inside
(e.g. estop lateral arc is 60–120°/−60–−120°) rather than assuming the whole
180° or 360° sensor FOV is relevant, (3) check whether the margin between
measured artifact extent and configured mask bounds is generous relative to
stated measurement jitter — narrow margins fail toward *under*-masking
(intermittent false e-stop latches, safe-but-annoying) not over-masking
(true blind spot growth), so err toward flagging tight margins as a
Suggestion/Warning rather than Critical, and (4) confirm
`docs/safety_validation.md` was updated alongside the config, not just
code/yaml comments.
