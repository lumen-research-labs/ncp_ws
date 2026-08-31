# `rm_interfaces` — custom messages and services

**Built on: both the robot and the workstation.** The only package both build
targets share.

It contains type definitions only — no nodes, no logic.

---

## Contents

| File | Published/served by | Consumed by |
|---|---|---|
| `msg/SafetyStatus.msg` | `rm_safety` on `/safety/status` @ 10 Hz | dashboard, `rm_waypoint`, operators |
| `msg/BaseDebug.msg` | `rm_base` on `/base/debug` @ 20 Hz | dashboard, diagnostics. **Nothing in the control path reads it, and nothing may.** |
| `msg/WaypointStatus.msg` | `rm_waypoint` on `/waypoint/status` every tick | dashboard, operators |
| `srv/SetWaypoint.srv` | `rm_waypoint` `/waypoint/goto` | dashboard |
| `srv/Jog.srv` | `rm_waypoint` `/waypoint/jog` | dashboard |

Each file carries its full rationale in comments; read the `.msg`/`.srv`
itself, not a summary.

---

## Design decisions

### Kill and reset use `std_srvs/Trigger`, not a custom srv

`bool success, string message` is exactly the right shape for a no-argument
success+message pattern. The dependency on `std_srvs` therefore belongs to
`rm_safety`, not to this package.

### `SetWaypoint` and `Jog` are **services**, not topics — on purpose

Two reasons, both load-bearing:

1. A waypoint is a discrete operator decision that must be **accepted or
   refused with a reason** — "the pose is not available", "that frame is not
   the one I am driving in", "the command source is policy". A topic would let
   every one of those fail silently.
2. A topic would give `rm_dashboard` a **publisher**, which it must never have.
   That is the property making it structurally incapable of becoming a second
   `/cmd_vel` source.

### `BaseDebug` array indexing

Every fixed-size array is indexed by the **measured** motor→wheel map:

```
[0] front_left (M1)   [1] back_left (M2)   [2] front_right (M3)   [3] back_right (M4)
```

Not the Yahboom sample order (`FL,FR,BL,BR`). Do not "fix" it back.

### `BaseDebug` replaced a Twist with PWM packed into `linear.x/y/z`

That was unreadable in `ros2 topic echo` and — worse — reported only the PWM
actually *written to the board*, so in `dry_run` (the default) it showed four
zeros and told the operator nothing about what the command **would** have done.
Every array here is populated in both modes; only `applied_pwm` is gated on
`dry_run`.

### Status messages are published unconditionally

`WaypointStatus` goes out every tick whether or not a waypoint is active, so
**the absence of motion is always explained by a value rather than by
silence**. The same principle governs `SafetyStatus`.

### `commanded_*` fields are requests, not outcomes

`WaypointStatus.commanded_vx/wz` is what the node is publishing on
`/teleop/cmd_vel` — **not** what reaches the wheels. `rm_safety` clamps, scales
and can zero it, and only forwards it at all when `command_source` is `teleop`
and the kill is cleared. Likewise `BaseDebug.commanded_twist` is reported even
when it is not being acted on, so a silent robot can be told apart from an
unheard command.

---

## Building

Changes here require a `colcon build` on the robot before any node using them
will run:

```bash
colcon build --symlink-install --packages-select rm_interfaces
```
