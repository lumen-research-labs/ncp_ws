---
name: pattern-stop-zone-shrink-downstream
description: safety_limits.yaml clamp/zone changes (e.g. stop_zone_m) are re-verified for the node that changed them but not for other consumers of rm_safety that were designed around the old value — check every /teleop/cmd_vel or /policy/cmd_vel producer whenever a zone radius changes.
metadata:
  type: project
---

`stop_zone_m` 0.35 -> 0.15 m (commit 724be1c, operator decision, tuned/
verified against teleop and policy driving) landed in the same merge as
`rm_waypoint`, a brand-new autonomous consumer of `/teleop/cmd_vel` that has
**zero obstacle awareness by design** and depends entirely on rm_safety's
latch + its own 6 s progress watchdog to avoid a collision
(`waypoint_controller.py` module docstring). Nobody re-verified the new,
much tighter stop margin against this new unattended drive path — it was
tuned for the drivers that existed at tuning time.

**Why:** rm_safety's zone constants are shared infrastructure; a change
justified for one consumer (teleop, hand on the stick, reacts in real time)
silently changes the safety margin for every other consumer, including ones
added in the same commit that never got a chance to be considered during the
tuning session.

**How to apply:** whenever `stop_zone_m`, `slow_zone_m`, or the arc widths in
`safety_limits.yaml` change, grep for every publisher of `/teleop/cmd_vel` and
`/policy/cmd_vel` (not just the one being tuned against) and ask whether each
one's own assumptions (e.g. "rm_safety gives me at least N cm of margin")
still hold. `rm_waypoint`'s `arrive_tol_m` sitting at exactly the new
`stop_zone_m` value (both 0.15 m) is the kind of coincidence worth flagging
even without proof it's wrong.
