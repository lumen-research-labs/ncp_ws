# `rm_tools` — offline analysis (workstation only)

**Runs on: the dev workstation. Never the robot.**

> `rm_tools` is **never a runtime dependency of `rm_safety` or `rm_policy`.**
> That separation is a project invariant, not a convention: nothing that runs
> on the robot in the control path may import anything from here.

---

## Scope

Offline bag analysis, model export, and schema checking — work that happens
*after* a run, on recorded data, with no robot attached and no real-time
constraint.

The package currently ships no console scripts; the working offline tooling
lives in `scripts/` and `scripts/diagnostics/` (see
[`scripts/diagnostics/README.md`](../../scripts/diagnostics/README.md)). This
package is the home for that code as it stabilises into importable modules with
tests.

---

## Why the separation is enforced

Three reasons, in order of how badly each has bitten:

1. **Build targets are separate and must not be co-installed.** The robot
   builds `rm_interfaces rm_safety rm_policy rm_base rm_bringup rm_dashboard
   rm_waypoint`; the workstation builds `rm_interfaces rm_tools`. An accidental
   import would make a robot build depend on packages that were never installed
   there.
2. **The workstation has no torch, numpy or ncps**, and the robot has no dev
   tooling. The two tiers genuinely cannot run each other's code — the
   workstation test tier is smaller than the robot's for exactly this reason,
   and tests that need numpy/torch skip there.
3. **Analysis code is allowed to be wrong.** A bag reader that misparses a
   frame costs a re-run. The same defect inside `rm_safety` costs a collision.
   Keeping the two in separate packages keeps that asymmetry visible.

---

## Related tooling that already exists

| Where | What |
|---|---|
| `tests/replay/laserscan_bag_reader.py` | Decodes recorded `LaserScan` messages straight from a rosbag2 sqlite3 store with a self-checking little-endian CDR reader — **no rclpy needed**, which is what lets the replay tier run on the workstation at all. |
| `scripts/check_interface.py` | Schema check against `config/interface.yaml`. |
| `scripts/diagnostics/` | Hardware diagnostics; these run **on the robot** and are not part of this package. |
| `scripts/fetch_bags.sh` | Pulls recorded runs back to the workstation. |

---

## Building

```bash
colcon build --symlink-install --packages-select rm_interfaces rm_tools
```
