# Lidar sampling, why the 720-beam block is built the way it is

The lidar block of the contract ([`interface_contract.md`](interface_contract.md)) is 720 beams over 270°, reduced from the TG30's ~2000 rays by `src/rm_policy/rm_policy/scan_resampler.py`. The policy trained on a simulated sensor with exactly one ray per beam, no dropout and no view of its own chassis. Three measured facts about the real sensor decide how the reduction reproduces that.

## About two rays per bin, and a third of them missing

The TG30 returns ~1514 rays across the 270° arc, about 2.1 per 0.375° bin, and drops 28 to 39% of rays as `0.0`. `nearest_valid_ray` takes the real return nearest each bin's centre. With two rays per bin, a bin whose rays are both dropouts has no real return at all. Measured on the fixtures in `tests/fixtures/`, strict per-bin sampling leaves 5.4 to 8.3% of the 720 bins in that state. Each reads "clear at 5 m" while a real return sits a fraction of a degree away, and simulation never produced that pattern.

`nearest_ray_search_window_bins: 2` lets such a bin borrow the real return nearest its centre within ±2 bins, a reach of 0.9375°. On the same fixtures this leaves 1.2 to 1.4% of bins empty. Rays inside the bin always win, so the window is a fallback and never a smoothing. The test `test_window_cuts_empty_bins_on_real_tg30_scans_at_720_bins` pins these numbers.

## The 270° arc sees the chassis

The 180° arc of the M4 contract stopped short of the chassis standoffs. The 270° arc includes them, at 0.11 to 0.25 m. The training sensor's 0.1 m `min_range` put the Jackal chassis in a dead zone, so those directions read "clear" in simulation.

`self_occlusion_zones_deg` lists `[92°, 123°]` and `[-135°, -97°]`, the same sectors `config/safety_limits.yaml` masks for the e-stop. Their rays count as bin members, contribute no range, and never feed the window. The two files hold the same numbers on purpose, and the safety path stays independent of this one.

## No-return is `0.0`

The TG30 encodes a missing return as `0.0`, and the driver's `invalid_range_is_inf` flag does nothing on this hardware. `scan_resampler` maps `0.0`, negatives, values below `range_min`, `inf`, `NaN` and masked rays to `range_max` before any sampling, which `build_obs` then clips to the 5 m training distance. This mapping is the only defence in the policy path. The measurements are in [`interface_contract_3.0.0_superseded.md`](interface_contract_3.0.0_superseded.md).

## Why not min-pooling

Min-pooling reports obstacles closer and wider than a one-ray-per-bin sensor would. Measured at 32 bins against real scans, it shifts the normalized observation by 0.03 to 0.19 on average and by up to 1.05 in single bins. That is a distribution shift away from training. `min_pool` stays available in `scan_resampler` for a consumer that wants conservatism, and `rm_safety` computes its e-stop from the full-resolution scan regardless.
