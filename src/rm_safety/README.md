# `rm_safety` — the safety layer

**Runs on: the robot.** This package is the reason the rest of the workspace is
shaped the way it is.

> **`rm_safety` is the ONLY publisher of `/cmd_vel` in the entire system.**
> Every other source of motion — the policy, the gamepad, the waypoint
> controller — publishes to an *input* topic and has no route to the base
> driver except through here.

There is **no physical E-stop on this robot.** `/safety/kill` is the only stop
mechanism short of pulling the battery. Every design decision below follows
from that one fact.

---

## What's in here

| File | What it is |
|---|---|
| `rm_safety/safety_logic.py` | The decision core. **Pure**: no ROS, no I/O, no clock reads, no logging. One function, `evaluate()`, takes a snapshot and returns a `Decision`. |
| `rm_safety/safety_node.py` | The ROS2 shell. Owns the timer, the subscriptions, the services, the kill-state file — and the sole `/cmd_vel` publisher. |

The split is deliberate and must not be collapsed. `safety_logic` has no clock
and no I/O, so every safety decision in the system is reachable from a unit
test with no ROS runtime and no robot. That is what makes the 81-test unit tier
and the replay tier possible.

---

## The pipeline

```
/policy/cmd_vel  ─┐
/teleop/cmd_vel  ─┤
/scan            ─┼──►  safety_node  ──►  safety_logic.evaluate()  ──►  /cmd_vel
/odom            ─┘                                    │
                                                       └──►  /safety/status
```

**Only ONE command input is live at a time**, selected by the `command_source`
parameter (`"teleop"` or `"policy"`). The other is ignored entirely — it is not
merged, arbitrated, or fallen back to.

### Topics and services

| | |
|---|---|
| Subscribes | `/policy/cmd_vel`, `/teleop/cmd_vel` (Twist) · `/scan` (LaserScan) · `/odom` (Odometry) |
| Publishes | **`/cmd_vel`** (Twist) @ 20 Hz — sole publisher · `/safety/status` (`rm_interfaces/SafetyStatus`) @ 10 Hz |
| Services | `/safety/kill` (`std_srvs/Trigger`) · `/safety/reset` (`std_srvs/Trigger`) |

`/odom` is used **only** as a liveness signal. `_odom_cb` stamps the staleness
watchdog and never feeds velocity into the decision. So odometry *accuracy*
cannot change a safety decision — but odometry *presence* gates every command
(see precedence 3 below).

---

## The decision, in precedence order

`evaluate()` runs these checks in order and returns on the first that fires.
Every one of them produces **zero velocity** except the last two.

| # | Check | State | Reason string |
|---|---|---|---|
| 1 | Kill switch active | `KILLED` | `killed` |
| 2 | `/scan` older than `scan_timeout_ms`, **or** no valid return anywhere in the monitored arcs | `STOPPED` | `scan_stale` |
| 3 | `/odom` older than `odom_timeout_ms` | `STOPPED` | `odom_stale` |
| 4 | Obstacle inside `stop_zone_m` for `consecutive_scans_to_trigger` scans — **latches** | `STOPPED` | `stop_zone` |
| 5 | Active command source silent for `command_timeout_ms` | `STOPPED` | `command_timeout` |
| 6 | Obstacle inside `slow_zone_m` | `SLOW` | `slow_zone` / `stop_zone_prelatch` |
| 7 | Nominal — clamp, acceleration-limit, pass through | `OK` | `ok` |
| — | Unhandled exception anywhere in `evaluate()` | `STOPPED` | `internal_error` |

Two subtleties worth knowing:

- **"No valid perception" is a stop, not a clear path.** An empty-but-fresh
  `/scan`, an all-NaN scan, or a scan with every return out of range all
  produce `min_range = inf`, which is treated as blindness. An early review
  round found this bypassing the e-stop — it was a real fail-OPEN bug.
- **A pre-latch stop-zone reading scales to zero, not through.** Check 6 covers
  `zone in ("slow", "stop")`, and `slow_scale()` returns 0.0 at or inside the
  stop zone. Before this fix (2026-08-14) a sustained stop-zone obstacle
  emitted *full commanded velocity* for the first N−1 ticks of the noise
  filter — ~400 ms at 5 Hz — which meant a closer obstacle braked *later*.

---

## Key design decisions

### Killed on startup, always

`safety_limits.yaml: startup.start_killed: true`. **Launching a node must never
be able to make the robot move.** There is no launch argument to override this
and none should be added. Motion always requires an explicit
`/safety/reset` call after the stack is up.

The kill state also **persists to disk** (`/var/lib/rm_safety/kill_state`), and
an unreadable state file is read as **KILLED**. If that directory does not
exist the node degrades to memory-only with a warning — `scripts/prep_robot.sh`
creates it, and that step must be re-run on any new robot or reflash.

### `command_source` is runtime-settable, and every change force-kills

Changing the source can therefore only ever *stop* the robot, never start it.
Arming stays a separate, explicit act. Every *other* parameter on this node is
read once at startup and a runtime `param set` is **rejected** with a reason —
not accepted and silently ignored, which is what it used to do.

### Two zones, two arcs

The slow zone and the stop zone have **separate arcs**, and this is not an
oversight:

| | radius | forward arc | lateral arc |
|---|---|---|---|
| slow | `slow_zone_m` 0.55 m | 30° cone | disabled (0.0) |
| stop | `stop_zone_m` 0.15 m | 90° | 60° each side |

The slow cone is deliberately too narrow to serve as a stop arc
(0.35·tan15° = 0.094 m is narrower than the robot), so `Limits.from_dict`
**refuses** a slow arc wider than the stop arc. Three constants are coupled and
must be changed together: `slow_zone_m`, `max_vx` (the band must cover braking
distance plus travel during the 3-scan filter), and `slow_forward_arc_deg` (the
cone must still subtend the robot's half-width at the slow radius). Both
couplings are pinned by `tests/unit/test_safety_arc_split.py`.

### The lateral clamps stay, even though the policy has no lateral action

Interface 3.0.0 has a 2-dim action `[vx, wz]` — no strafe. But `max_vy` and the
lateral e-stop arcs remain, because **`/teleop/cmd_vel` is also an input** and
an operator may legitimately strafe while repositioning. A non-zero `linear.y`
arriving on `/policy/cmd_vel` is treated as a **bug, not a command**: zeroed and
warned about, never silently clamped.

### The authority gap is intentional

The trained policy's own range is `vx ∈ [-1, 2] m/s`, `wz ∈ [-3.14, 3.14]
rad/s`. The clamps here are 0.25 / 0.8 — roughly 8× and 4× tighter. **Do not
raise the clamps to "let the policy through."** The safety layer is designed to
be the binding constraint. Note the current values are themselves *above* the
last signed-off values (0.15 / 0.4); revert for anything unattended.

### The zones can be switched off — deliberately, and loudly

`slow_zone_enabled` / `stop_zone_enabled` pause the lidar zones without
touching anything else. Both default **true** in `config/safety_limits.yaml`,
but `ncp.launch.py` passes **false** for both since 2026-08-31 (operator
decision), so the full-stack policy launch is the one entry point that comes
up without proximity protection. Added 2026-08-30 at operator
request: the slow zone's braking made it impossible to see what the *policy*
does about obstacles, because the layer beneath it always braked first.

**With the stop zone off there is no obstacle protection on this robot at
all.** The clamps bound speed, not proximity. `/safety/kill` becomes the only
stop.

Four properties make this something other than a hole in the safety layer:

- **Default on, from every direction.** A config omitting the keys gets the
  full e-stop; absence never reads as "disabled".
- **AND composition.** The effective value is `YAML AND parameter`. Either can
  disable, neither can re-enable what the other switched off — so whichever
  file you read, the protection it claims is at most what is running.
- **Startup-only.** A runtime `ros2 param set` is **rejected** with a reason.
  Disabling the e-stop is a bring-up decision, not something the
  unauthenticated dashboard on `0.0.0.0` gets to make mid-run.
- **Self-describing.** The nominal `reason` becomes `ok_estop_disabled` (or
  `ok_slow_zone_disabled` / `ok_stop_zone_disabled`), so `/safety/status`, the
  dashboard and every recorded bag carry the fact continuously — not just a
  startup log line someone scrolled past. The node also re-warns every 10 s.

Everything else survives untouched: all velocity clamps, all acceleration
limits, the `vy` violation check, kill and killed-on-startup, and all three
staleness watchdogs. `min_range` is still computed and published, so the
dashboard still shows proximity — the layer just does not act on it.

Two behaviours that follow, both intentional:

- **A disabled stop zone clears a latch it had already set.** Otherwise a
  robot that latched before the switch would stay `STOPPED` forever with no
  obstacle logic left running to release it, and the switch would appear to do
  nothing.
- **`no_perception` is gated on the zones; scan *staleness* is not.** "No
  valid returns in the monitored arcs" is a statement about arc geometry, and
  the arcs feed the zones — with both off, nothing consumes it, and on this
  hardware an open room genuinely can leave the forward arc empty (~31–39% of
  TG30 rays are no-returns). Message *age* is a different thing and is never
  gated: a frozen scan means `policy_node` is building its observation from
  stale data, which is a fault regardless.

### Deceleration bypasses the acceleration limiter

A fail-closed path zeroing the command is allowed to violate `max_ax`. Braking
hard is always safe; ramping down a stop is not.

---

## Configuration

`config/safety_limits.yaml` is the **only** source of numeric limits. No numeric
literal appears in a logic body. The node loads it at runtime via
`Limits.from_dict()` and **raises on init** if it cannot be read or parsed — it
never starts in an unknown-limits state.

## Failure mode

Fail-closed, everywhere, without exception. Any unexpected input (NaN, inf,
zero/negative dt, empty arc, malformed limits) produces zero velocity — never a
pass-through of the last command. An unhandled exception inside `evaluate()` is
caught at the top level and returns a fully zeroed `STOPPED` decision so the
caller always gets a safe value. On shutdown the node publishes a final zero
Twist best-effort.

## Tests

- `tests/unit/test_safety_logic.py` — 81 tests against `evaluate()` directly.
- `tests/unit/test_safety_arc_split.py` — pins the arc/radius couplings.
- `tests/replay/test_safety_replay.py` — real recorded TG30 scans through the
  real `evaluate()` with the real `safety_limits.yaml`, ROS-free.
- Hardware tier: `docs/safety_validation.md` §3. **Operator only** — never run
  by an agent.
