# Plan: the safety zones are clearances beyond the robot's `min_range_m`

> Status: BUILT and DEPLOYED to bernard-3 2026-11-18 (cdd27f8; 872 workstation tests; approved by Daniel: "shouldn't we just have min range m and then we set it to the length to bumper or min lidar length? And yes go ahead"). bernard-1 deployed the same evening (aaa9484): start line `slow 0.55 m / stop 0.15 m (min_range 0.07 + 0.48 / 0.08)`, unchanged by construction; both robots ran at once on domains 28 and 29 with no cross-talk. The previous plan (one profile per robot) is built and deployed to bernard-3; git history keeps it. Written to `~/lumen/optomotive/CODING_PROMPT.md` (2026-10-22).

## Context

bernard-3 drives into things with both lidar zones on. `safety_limits.yaml` gives the stop zone as 0.15 m and the slow zone as 0.55 m, both **from the lidar**. On bernard-1 that works: the TG30 sees down to 0.05 m and 0.15 m is 8 cm in front of the bumper. On bernard-3 the RPLIDAR A1 returns nothing inside 0.15 m, so the stop zone sits exactly on the blind radius and can never fire, and the slow zone fails open: an object scaling the speed toward zero crosses into the blind radius 7.5 cm in front of the bumper, vanishes from the arcs, and the speed scale jumps back up. Measured 2026-11-18: the lidar sits 0.0435 m ahead of base_link and the chassis mesh ends at 0.116 m, so the bumper is 0.075 m from the lidar on both units (stock mount).

## Design

One new robot fact, in the profile's lidar block: **`min_range_m`, the larger of the sensor's blind radius and the lidar-to-bumper distance.** Inside it the zones can see nothing, or it is already contact. bernard-1: 0.07 (the bumper; the TG30 sees closer). bernard-3: 0.15 (the A1's blind radius).

`safety_limits.yaml` loses `stop_zone_m` and `slow_zone_m` and gains **`stop_clearance_m` 0.08 and `slow_clearance_m` 0.48, clearances beyond `min_range_m`.** `Limits.from_dict(cfg, mount)` computes the lidar ranges the logic runs on: `stop_zone_m = min_range_m + stop_clearance_m`, likewise slow. The `EstopLimits` fields `stop_zone_m` and `slow_zone_m` keep their meaning (lidar ranges), so `evaluate`, `slow_scale`, the retreat and the dashboard are untouched. A limits file that still carries `stop_zone_m` or `slow_zone_m` is refused, naming the new keys; a mount without `min_range_m` is refused.

| robot | min_range_m | stop zone (lidar) | slow zone (lidar) | stop, in front of the bumper |
|---|---|---|---|---|
| bernard-1 | 0.07 | 0.15 (unchanged) | 0.55 (unchanged) | 8 cm |
| bernard-3 | 0.15 | 0.23 | 0.63 | 15.5 cm |

## Failure modes

| Failure | Surfaces as |
|---|---|
| `safety_limits.yaml` still carries a lidar-range zone key | `Limits.from_dict` refuses, naming the clearance keys |
| A profile without `min_range_m`, or a non-positive or absurd one | the profile loader refuses, naming the file and the key |
| Clearances not finite, not positive, or slow not above stop | `Limits.from_dict` refuses |
| A reader asking for the zones without a mount | gets the clearances as ranges (min_range 0): tests that mean a robot pass bernard-1's mount |

## Changes

`rm_safety/robot_profile.py` (field, parse, `mount`), `rm_safety/safety_logic.py` (stray keys, mount, computed zones, three new fields, validation), `safety_node.py` (the start line names min_range and the clearances), `config/safety_limits.yaml`, `config/robots/bernard-1.yaml`, `config/robots/bernard-3.yaml`, `scripts/diagnostics/velocity_accuracy.py` (the runway check builds `Limits` with the profile instead of reading the key), `ncp.launch.py` and `config/roam.yaml` comments, docs (`add_a_robot.md`, `rm_safety/README.md`, `rm_bringup/README.md`, `calibration.md`), tests (`test_robot_profile`, `test_safety_zone_switches` keys, `test_safety_arc_split`/`test_safety_logic`/`test_retreat` pass bernard-1's mount, new `test_zone_clearance.py`, `test_docs` needle).

## Verification

Suite green; bernard-1's `real_limits` still reads 0.15 / 0.55 and the replay tier is unchanged. Deploy to bernard-3, restart, and the safety start line reads `slow 0.63 m / stop 0.23 m (min_range 0.15 + clearances 0.48 / 0.08)`; a hand held in front of the bumper latches the stop zone before contact. bernard-1 when it is back: start line unchanged.

## Not in this plan

The roam and retreat distances (`retreat_release_range_m`, roam's `clearance_m`, `self_radius_m`) are still lidar ranges; the same treatment is a follow-up.
