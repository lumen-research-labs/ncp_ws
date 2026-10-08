# `rm_dashboard` — the live web view

**Runs on: the robot.** Serves the whole stack as one page at
`http://<robot>:8080/`.

> **`rm_dashboard` publishes NO topic. Not one, ever.**
> It is a leaf. That is what makes it structurally incapable of becoming a
> second `/cmd_vel` publisher, and it is the property to protect above any
> feature.

---

## What's in here

| File | Kind | What it is |
|---|---|---|
| `state_store.py` | pure, thread-safe | Latest value per topic, plus arrival-rate, jitter and staleness. |
| `slam_health.py` | pure | Turns a stream of `map → odom` corrections into the few numbers that say whether SLAM works. Also `pack_grid` / `summarize_map`. |
| `web_server.py` | pure, ROS-free | `http.server` + SSE. One page, one event stream, a few command endpoints. |
| `dashboard_node.py` | ROS2 node | Subscribes to everything; runs the server on a background thread. |
| `static/index.html` | the page | |

---

## Why it looks the way it does

Nearly every integration failure on this project has been a failure of the
**joins**, not of the parts. Each stage was individually correct when:

- the scan bridge published BEST_EFFORT to a RELIABLE subscriber;
- a second EKF was started on top of the first;
- `command_source` accepted a value and ignored it;
- front-right was being dragged by the other three wheels.

In each case the missing view was the same one: **what does each stage think,
side by side, right now.** So the layout follows the signal:

```
/scan → /ncp/scan → [observation] → policy (v,w)
      → /policy/cmd_vel → rm_safety → /cmd_vel → rm_base → wheels
```

with **rate, jitter and staleness on every hop**, plus safety state/reason/
`min_range`, per-wheel base telemetry with its flags, and the SLAM panel.

Rates matter as much as values. Doubled rates (`/odom` 61 vs 30, `/cmd_vel` 41
vs 20) are the giveaway for a duplicate stack, which otherwise presents as an
impossible sensor or SLAM fault. That is why the page shows
expected-vs-actual rather than just a number.

---

## Key design decisions

### Stdlib only — a hard requirement, not a preference

bernard-1 has **no internet** (RobotLab has no WAN uplink), so anything needing
`pip install` cannot be deployed at all. That rules out flask, websockets and
rosbridge, and leaves SSE over plain HTTP — which fits anyway: the traffic is
one-way state at a fixed rate, and browsers reconnect dropped SSE streams on
their own.

### Served FROM the robot

It needs only TCP to reach a browser, so it works whenever SSH does. Running it
workstation-side would put the diagnostic behind DDS discovery across the lab
network — the single most repeatedly-broken thing on this project. **A
diagnostic must not share a failure mode with the thing it diagnoses.**

### The observation comes from `/ncp_debug`, not rebuilt here

`policy_node` publishes the actual 724-dim vector it consumed. A second
implementation of the frozen contract would render a *plausible* vector that is
not the one the network read.

### The map is on its own endpoint

`/api/map`, never the SSE stream. An occupancy grid is hundreds of KB and
changes every ~2 s against a 10 Hz stream; putting it on the stream would
multiply outbound traffic by two orders of magnitude to redraw an unchanged
picture. The browser refetches only when a `seq` in the snapshot moves.

**Grid decimation takes the MOST OCCUPIED cell per block, never an average.** A
wall is one or two cells thick at 0.05 m resolution, so averaging deletes walls
at any real factor — and a map that has lost its walls looks like a room the
robot can drive through. Pinned by a test.

`/map` is subscribed **RELIABLE + TRANSIENT_LOCAL**. The stack's usual sensor
QoS would have silently received nothing.

### `/scan` decimation is display-only

`scan_stride` (default 6) reduces ~2000 points for the plot. **Every statistic
— min range, dropout count — is computed on the full scan before decimation**,
so nothing derived is affected.

### A rate is `None` until the window holds two samples

A single message does not establish a rate, and printing one from a single
sample would be an invention. Similarly, an absent topic is represented
**explicitly** rather than omitted — a missing key renders as a blank panel,
which reads as "fine" instead of "not there".

### The SLAM panel is built on one insight

**`map → odom` IS the SLAM correction.** Everything worth knowing is in that
one transform's behaviour over time, and none of it is visible in a map
picture:

- **magnitude** = the odometry error SLAM has undone. It *should grow*; flat
  zero while driving means scan matching is contributing nothing.
- **steps** = loop closures. Each one resets the CfC's recurrent state, so
  counting them tells an operator whether a run had four recurrent-state resets
  in it or none — which otherwise shows up only as unexplained steering.
- **age** = whether SLAM is alive at all. A dead slam_toolbox does not error;
  the transform simply stops, and the robot fails closed with a `pose_*` reason
  several layers from the cause.

The panel also shows the **`/scan_sanitized` zero count, which must be 0** —
that is SLAM's actual input, and the TG30's `0.0` sentinel reads to karto as a
hit at the sensor.

> **Map EXTENT is not evidence of pose drift.** It measures what the lidar can
> *see*, not where the robot has been. A 12 m map from a stationary robot in a
> room is normal. Measure pose stability directly instead.

### Clicking the map does not drive

A click only fills in the waypoint numbers; motion needs a separate **DRIVE
THERE** press. The click↔marker transform round-trips exactly, so the robot
goes where the operator clicked.

---

## Controls, and an honest note about them

The page can send: `/safety/kill`, `/safety/reset`, parameter sets on
`safety_node` / `policy_node`, and the two `/waypoint` services (`goto`,
`cancel`, `jog`). It has **no publisher of any kind**.

An earlier version of this docstring claimed waypoints "require two prior
operator acts that this page cannot perform". **That was false**, and it is
corrected here rather than quietly deleted, because a false claim in safety
documentation is worse than no claim. `source` and `reset` are both exposed, so
a browser can set the source, clear the kill, and send a waypoint in three
POSTs with no physical operator involved.

That is a **deliberate operator decision** (2026-08-29: full control was chosen
over observe-and-kill), not an oversight. But be clear what it means: the
server binds `0.0.0.0` with **no authentication**, so the boundary protecting
the robot is the network itself. On an isolated lab network with no WAN uplink
that is a reasonable trade. It would not be on any other network, and
**`enable_controls:=false`** is the switch that takes it away — the command
endpoints refuse and the buttons render disabled.

`command_source` is **read back** from `safety_node` via
`/safety_node/get_parameters`, not remembered from what the page last asked
for. The page also states plainly, *before* the button is pressed, why a
waypoint would not move the robot (killed / wrong source / dry run).

### Keyboard driving lives in `rm_waypoint`, not here

The WASD jog was the "let me drive it from the browser" request, and it would
have needed a Twist publisher in this package. Instead the publisher went into
`rm_waypoint` and the dashboard reaches it by **service**. **If a future
feature seems to need a publisher here, that is the pattern** — a separate node
owns it. A test pins that `rm_dashboard` contains no `create_publisher` at all.

---

## Failure mode

Diagnostic-grade, and fail-quiet toward the robot / loud toward the browser.

- Every callback is wrapped: a malformed message degrades one panel and never
  takes the node down.
- The HTTP server runs on daemon threads and is never awaited by the ROS
  thread, so a wedged browser cannot stall the subscriptions.
- A slow client is dropped from the broadcast list rather than blocking.
- If the port is already bound the node logs the error and **continues
  headless** rather than dying — the subscriptions are harmless and the
  operator may be about to close the other tab.
- Every accessor returns a well-formed snapshot even when nothing has ever
  arrived. A dashboard that raises because a topic is missing is useless in
  exactly the situation you opened it for.
