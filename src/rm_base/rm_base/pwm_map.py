"""
pwm_map.py — pure wheel-speed <-> open-loop PWM conversion for rm_base.

What it does:
    Maps a target wheel ground speed (m/s) to a Rosmaster `set_motor` value
    (dimensionless, [-100, 100]) and back, per wheel, using a linear model
    with a static-friction dead-band:

        |pwm| = dead_pwm + gain_pwm_per_mps * |v|      (v above v_min)
        pwm   = 0                                       (|v| <= v_min)

    Each wheel has its own dead_pwm/gain because bernard-1's front-right
    motor is weak (2026-08-28): it needs a far larger dead_pwm than the
    other three and stalls below it. The model is open-loop — there is no
    feedback term — so these constants are a CALIBRATION, not a controller.
    They must be tuned on hardware, wheels-down, against a tape measure.

Failure mode:
    Pure arithmetic, never raises. `to_pwm` always returns an int clamped to
    +/- max_pwm. `to_speed` returns 0.0 inside the dead-band (the inverse is
    genuinely ambiguous there).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WheelPwmModel:
    """Open-loop PWM calibration for one wheel."""

    dead_pwm: float          # PWM needed to overcome static friction
    gain_pwm_per_mps: float  # slope above the dead-band
    v_min_mps: float = 0.02  # below this, command 0 (can't creep open-loop)
    max_pwm: int = 60        # per-wheel ceiling; keep low until calibrated

    def to_pwm(self, v_mps: float) -> int:
        if abs(v_mps) <= self.v_min_mps:
            return 0
        sign = 1.0 if v_mps > 0.0 else -1.0
        mag = self.dead_pwm + self.gain_pwm_per_mps * abs(v_mps)
        pwm = int(round(sign * mag))
        if pwm > self.max_pwm:
            return self.max_pwm
        if pwm < -self.max_pwm:
            return -self.max_pwm
        return pwm

    def to_speed(self, pwm: float) -> float:
        """Inverse, for turning a commanded PWM back into an expected speed.

        Only meaningful for |pwm| > dead_pwm; inside the dead-band the wheel
        is not turning, so this returns 0.0.
        """
        if abs(pwm) <= self.dead_pwm or self.gain_pwm_per_mps <= 0.0:
            return 0.0
        sign = 1.0 if pwm > 0.0 else -1.0
        return sign * (abs(pwm) - self.dead_pwm) / self.gain_pwm_per_mps


def slew_limit(prev_pwm: int, target_pwm: int, max_step: int) -> int:
    """Clamp a PWM change to +/- max_step per tick (drivetrain / STM32 protection)."""
    if max_step <= 0:
        return target_pwm
    delta = target_pwm - prev_pwm
    if delta > max_step:
        return prev_pwm + max_step
    if delta < -max_step:
        return prev_pwm - max_step
    return target_pwm
