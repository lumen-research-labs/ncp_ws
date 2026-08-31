---
name: pattern-dashboard-no-auth-drive-path
description: rm_dashboard has no authentication; check every new dispatch command against the full set of pre-existing ones for a chained remote-drive path, not just in isolation.
metadata:
  type: project
---

`src/rm_dashboard` binds `0.0.0.0` with zero authentication (stdlib-only
constraint, no pip on the robot — documented tradeoff in `web_server.py`).
Its `_CONTROLS` dispatch table (`dashboard_node.py`) already exposed `kill`,
`reset`, and `source` (command_source is runtime-settable and a change
force-kills, `safety_node.py`). None of those alone can move the robot, so
the dashboard's own docstring claims "arming always remains a separate,
explicit act."

That claim broke the moment a command that can produce a nonzero Twist was
added (`rm_waypoint`'s `/waypoint/goto`, merged 166c8d1) — because the SAME
browser session can call `source` -> `reset` -> `waypoint` itself, no human
step in between, no confirmation enforced server-side. The new module's own
safety docstring (`dashboard_node.py` "THE WAYPOINT CONTROL...") asserted the
old claim was still true without checking it against the other controls
already in `_CONTROLS`.

**Why:** each PR reviews its own diff in isolation and reasons "this new
command alone can't move the robot" — true, but the dashboard's real attack
surface is the *union* of all `_CONTROLS` ever exposed, not the new one in
isolation.

**How to apply:** whenever a new dashboard `_CONTROLS` entry is added, re-walk
the full chain "what is the shortest sequence of already-exposed HTTP POSTs
that reaches a nonzero Twist" and check it against the docstring's safety
claim before trusting it. See [[pattern_slow_stop_arc_split]] for the same
"check the whole config, not just the diff" instinct applied to arcs.
