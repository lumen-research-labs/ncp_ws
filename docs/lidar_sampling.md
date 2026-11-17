# Lidar sampling, why the 720-beam block is built the way it is

The lidar block of the contract ([`interface_contract.md`](interface_contract.md)) is 720 beams over 270°, reduced from the TG30's ~2000 rays by `src/rm_policy/rm_policy/scan_resampler.py`. The policy trained on a simulated sensor with one ray per beam, no dropout and no view of its chassis. Measured facts about the real sensor decide how the reduction reproduces that.

## About two rays per bin, and a third of them missing

The TG30 returns ~1514 rays across the 270° arc, about 2.1 per 0.375° bin, and drops 28 to 39% of rays as `0.0`. `nearest_valid_ray` takes the real return nearest each bin's centre. A bin whose two rays are both dropouts has no real return. On the fixtures in `tests/fixtures/`, strict per-bin sampling leaves 5.4 to 8.3% of the 720 bins in that state, each reading "clear at 5 m" with a real return a fraction of a degree away; simulation never produced that.

`nearest_ray_search_window_bins: 2` lets such a bin borrow the real return nearest its centre within ±2 bins, a reach of 0.9375°. On the same fixtures this leaves 1.2 to 1.4% of bins empty. Rays inside the bin always win, so the window is a fallback and never a smoothing. A bin with no ray of its own is served the same way (bernard-3's RPLIDAR A1 spaces rays 1° apart, so two bins in three have none); a bin with no ray within reach is a fault. Tests pin both.

## The 270° arc sees the chassis

The M4 contract's 180° arc stopped short of the chassis standoffs; the 270° arc includes them, at 0.11 to 0.25 m. The training sensor's 0.1 m `min_range` put the Jackal chassis in a dead zone, so those directions read "clear" in simulation.

`self_occlusion_zones_deg` lists `[92°, 123°]` and `[-135°, -97°]`, the same sectors `config/safety_limits.yaml` masks for the e-stop. Their rays count as bin members, contribute no range, and never feed the window. The two files hold the same numbers on purpose; the safety path stays independent.

## No-return is `0.0`

The TG30 encodes a missing return as `0.0`; the driver's `invalid_range_is_inf` flag does nothing here. `scan_resampler` maps `0.0`, negatives, values below `range_min`, `inf`, `NaN` and masked rays to `range_max` before any sampling, which `build_obs` then clips to the 5 m training distance. This mapping is the policy path's only defence. Measurements: [`interface_contract_3.0.0_superseded.md`](interface_contract_3.0.0_superseded.md).

## Why not min-pooling

Min-pooling reports obstacles closer and wider than a one-ray-per-bin sensor. Measured at 32 bins on real scans it shifts the normalized observation by 0.03 to 0.19 on average and up to 1.05 in single bins: a shift away from training. `min_pool` stays available in `scan_resampler` for a consumer that wants conservatism; `rm_safety` computes its e-stop from the full-resolution scan regardless.

## The body floor

Nothing in the world is nearer the lidar than the chassis, so a return under a measured floor is the robot. Measured 2026-11-04 (`tests/fixtures/tg30_scan_2026-11-04_standoffs.yaml`): the standoffs return 0.06 to 0.11 m from −138° to −94°, closer and wider than on 2026-08-14 when the zones were cut. The −97° to −94° band leaked past the mask and reached the policy as a 0.10 m obstacle; M9 answered with full left yaw. `body_floor_m: 0.12` in each contract's lidar block makes the resampler treat such a return as a no-return (`range_max`, clear); `rm_safety` skips the same returns before its arcs (`estop.body_floor_m`); `scan_sanitizer_node` rewrites returns under `estop.body_radius_m` (0.25 m, the corners) to `+inf` for slam_toolbox. The zones stay for anything above it. Live check: `/ncp_debug/<profile>` `obs` has no lidar entry below −0.9 with nothing near the robot. `tests/unit/test_body_floor.py` holds the measurement.
