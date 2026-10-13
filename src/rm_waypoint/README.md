# `rm_waypoint` — go-to-waypoint and manual jog

**Runs on: the robot.** Drives to an operator-chosen point, or under WASD from
the dashboard. **Not part of the NCP pipeline** — nothing in that pipeline
reads it.

> **`rm_waypoint` publishes `/teleop/cmd_vel` and NOTHING else.**
> The topic is a **module constant with an assert**, not a parameter, so no
> launch typo can point it at `/cmd_vel`.

---

## Why it exists

**Picking the robot up breaks SLAM completely.** slam_toolbox gates on
odometry-reported motion (`minimum_travel_distance` 0.2 m), so a lift-and-place
reports zero movement: no scan is processed, and the map and pose stay frozen
at the old spot. `mode: mapping` has **no relocalization** — this is the
kidnapped-robot problem, and no threshold tuning helps, because zero reported
motion never crosses any threshold. Recovery after a hand-move is a **stack
restart** with a fresh map.

So: reposition by **driving**. That is this package.

---

## Where it sits — the whole safety argument

```
/waypoint/goto ──► waypoint_node ──► /teleop/cmd_vel ──► rm_safety ──► /cmd_vel
```

It is an **input** to `rm_safety`, exactly like the gamepad, so it drives
behind the same clamps, the same slow and stop arcs, the same acceleration
limiter and the same kill as every other source.

**Three independent things must all be true before it can move the robot, and
this node grants none of them:**

1. an operator explicitly sends a waypoint (it idles otherwise);
2. `rm_safety`'s `command_source` is `teleop`;
3. `rm_safety` has been armed with `/safety/reset`.

`with_waypoint` defaults **false** in `ncp.launch.py` for the same reason: this
is the only autonomous motion path in the workspace that is not the policy, and
on a robot with no physical E-stop a new one should be something you ask for.

---

## What's in here

| File | Kind | What it is |
|---|---|---|
| `waypoint_controller.py` | pure, ROS-free | The control law and its give-up conditions. |
| `waypoint_node.py` | ROS2 node | The clock, the pose, the services, and the one publisher. |

### Interface

| | |
|---|---|
| Subscribes | `/odom` (`pose_source:=odom`) · `/tf`,`/tf_static` (`pose_source:=map`) · `/safety/status` |
| Publishes | `/teleop/cmd_vel` **while active only** · `/waypoint/status` (`rm_interfaces/WaypointStatus`) always |
| Services | `/waypoint/goto` (`SetWaypoint`) · `/waypoint/cancel` (Trigger) · `/waypoint/jog` (`Jog`) |

---

## The control law

```
far from heading   ->  turn in place
facing the target  ->  drive, steering proportionally
within tolerance   ->  stop, ARRIVED
```

### P-only, deliberately

**No integral term.** With `pose_source:=map` the pose is corrected by SLAM,
which means it **steps** at every loop closure. An integrator would wind up on
that step and then drive the windup back out as a real command. A P law forgets
the step in one tick — the same reasoning that makes `policy_node` reset its
recurrent state there.

### A command floor, also deliberately

This robot has a **measured rotational dead-band**: roughly the first
0.107 rad/s of a commanded spin is lost to mecanum roller scrub before the
chassis turns at all, and front-right needs a real push to break loose. A
tapering proportional law therefore does not converge here — it **asymptotes
into the dead-band**, sitting at 0.04 rad/s forever while reporting it is
nearly there. Below `min_vx`/`min_wz` this module commands exactly **zero**
instead. Either move properly or do not pretend to be moving.

**Do not "fix" either of these into a textbook PID.**

### No obstacle logic, and the progress watchdog is why

This controller has no idea what an obstacle is, and does not need one.
`rm_safety` owns the arcs and the stop zone; when they fire the robot stops and
the range stops falling — which the **progress watchdog** sees as "no progress"
and gives up on. It equally catches a stalled wheel and a frozen pose source.

Obstacle avoidance living in exactly one place, one layer down, is what this
whole workspace is built around. A second opinion up here could only ever
disagree with the layer that actually has authority.

**It never retries.** Unattended retry into a failure nobody has looked at is
how a robot grinds into a wall. An operator who wants another attempt presses
the button again.

**It never reverses** on an autonomous waypoint — it turns to face instead —
because `rm_safety`'s e-stop arcs are forward and lateral. **There is no rear
arc.**

---

## Manual jog — the TTL is the deadman

WASD / arrow keys on the dashboard, hold to drive.

A browser can close its tab, sleep its laptop, lose WiFi, or simply stop
sending mid-keypress, and **none of those deliver a "stop"**. So a jog does not
mean "drive until told otherwise" — it means **"drive for at most `ttl_s`
more"**:

- the page re-sends every 150 ms against a 400 ms TTL;
- **the node caps any TTL at 0.5 s** — the robot decides the maximum, not the
  caller;
- **releasing a key sends nothing.** The command simply expires;
- blur and tab-hide also stop it, since a keyup landing in another window would
  leave the page re-sending forever.

`rm_safety`'s `command_timeout` is the second, independent layer.

**Never "simplify" this into a start/stop pair.**

Unlike an autonomous waypoint, jog **allows reverse** — backing out of a corner
is exactly what repositioning needs, and a human is watching. A jog cancels an
active waypoint.

---

## Configuration — `config/waypoint.yaml`

**THE authority.** A **missing key is an error**, not a default — deliberately
avoiding the `closed_loop.guards` shadowing bug one package over. The node
refuses to start without the file.

Limits sit deliberately **under** the safety clamps (0.15 / 0.6 vs 0.25 / 0.8),
**pinned by a test**, so `rm_safety` is always the binding constraint.

> `arrive_tol_m` is 0.15 m and the stop zone is now also 0.15 m. It drives
> **blind** by design, leaning entirely on `rm_safety`. Keep waypoints short
> and watched.

---

## Failure mode

Fail-closed on every path, and **silent when idle**.

**IDLE MEANS SILENT — it publishes no Twist at all, not zeros.** The gamepad
shares `/teleop/cmd_vel`; a node helpfully publishing zeros forever would fight
the operator's stick for the whole session. On every transition *into* a
terminal state it floods zeros first, then goes quiet — so the robot is
definitely stopped before the topic is released.

Anything that goes wrong stops the robot and **latches idle with a reason**
on `/waypoint/status`: a missing, stale or non-finite pose, a timeout, a
stalled range, a kill, an unhandled exception. "It stopped and I do not know
why" is the failure that `reason` field exists to prevent.

A kill is **watched for** rather than assumed: if `rm_safety` reports KILLED
while a run is active, the run is abandoned immediately. That changes nothing
about what the wheels do — `rm_safety` would zero the command anyway — but the
operator is told the run is over instead of watching a controller command a
robot that is not listening.

A waypoint sent while `command_source` names a policy row is **refused up front**,
rather than accepted, never moving, and then aborting with a stall complaint.

A waypoint in the wrong frame is **refused, not converted** — silently
converting between a drifting frame and a corrected one changes where the robot
is being sent without saying so.

### `/teleop/cmd_vel` sharing

`teleop_joy.launch.py`'s remapped `joy_ctrl` publishes the same topic. That is
not a conflict the way two `/cmd_vel` publishers would be — `rm_safety` takes
the latest message either way — but while a waypoint run is **active**, moving
the gamepad stick fights it. Cancel the waypoint before driving manually.
