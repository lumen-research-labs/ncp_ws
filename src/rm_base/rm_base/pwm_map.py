"""
pwm_map.py — pure wheel-speed <-> open-loop PWM conversion for rm_base.

What it does:
    Maps a target wheel ground speed (m/s) to a Rosmaster `set_motor` value
    (dimensionless, [-100, 100]) and back, per wheel, using a linear model
    with a static-friction dead-band:

        |pwm| = dead_pwm + gain_pwm_per_mps * |v|      (v above v_min)
        pwm   = 0                                       (|v| <= v_min)

    Each wheel has its own dead_pwm/gain: the four differ measurably in how
    much push they need, and the model is open-loop — there is no feedback
    term — so these constants are a CALIBRATION, not a controller. They must
    be tuned on hardware, wheels-down, against a tape measure.

PER-DIRECTION CALIBRATION:
    A wheel does not necessarily behave the same forwards and backwards.
    Measured on bernard-1: back-left turns FORWARD at PWM 7 but is DEAD in
    reverse until about PWM 16, and on the floor BOTH left wheels delivered
    only 23-32% of their commanded speed while running in reverse during a
    spin, against 99-106% for the right pair going forward.

    A single symmetric dead_pwm/gain pair cannot express that. Setting it for
    reverse over-drives the wheel forwards; setting it for forwards means the
    wheel barely moves in reverse — and the closed loop cannot rescue either,
    because the integrator is deliberately reset whenever the target crosses
    zero (carrying forward-windup into reverse would have the wrong sign), so
    it never accumulates across a direction change.

    So dead_pwm_rev / gain_pwm_per_mps_rev may be set independently. Both
    default to None, meaning "same as forward" — an uncalibrated wheel keeps
    exactly its previous symmetric behaviour.

Failure mode:
    Pure arithmetic, never raises. `to_pwm` always returns an int clamped to
    +/- max_pwm. `to_speed` returns 0.0 inside the dead-band (the inverse is
    genuinely ambiguous there).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class WheelPwmModel:
    """Open-loop PWM calibration for one wheel."""

    dead_pwm: float          # PWM needed to overcome static friction
    gain_pwm_per_mps: float  # slope above the dead-band
    v_min_mps: float = 0.02  # below this, command 0 (can't creep open-loop)
    max_pwm: int = 60        # per-wheel ceiling; keep low until calibrated
    # Reverse-direction overrides. None means "same as forward", which keeps
    # an uncalibrated wheel exactly symmetric. See the module docstring.
    dead_pwm_rev: Optional[float] = None
    gain_pwm_per_mps_rev: Optional[float] = None
    # PWM needed to BREAK A STOPPED WHEEL LOOSE, which for a sticky motor is
    # much higher than the PWM needed to keep it turning. None (the default)
    # means this wheel has no stiction problem and needs no kick — see
    # wheel_control's breakaway kick. Measured wheels-up with the motor driven
    # directly (scripts/diagnostics/fr_threshold.py); it is NOT the dead_pwm,
    # and setting it as dead_pwm would over-drive the wheel at every speed.
    breakaway_pwm: Optional[float] = None
    breakaway_pwm_rev: Optional[float] = None

    def _params(self, reverse: bool) -> tuple[float, float]:
        """(dead_pwm, gain) for the requested direction."""
        if not reverse:
            return self.dead_pwm, self.gain_pwm_per_mps
        dead = self.dead_pwm if self.dead_pwm_rev is None else self.dead_pwm_rev
        gain = (
            self.gain_pwm_per_mps
            if self.gain_pwm_per_mps_rev is None
            else self.gain_pwm_per_mps_rev
        )
        return dead, gain

    def breakaway_for(self, v_mps: float) -> Optional[float]:
        """Breakaway PWM for this direction, or None if not configured.

        Falls back to the forward value when only that is set, so a wheel with
        a single measured breakaway still gets a kick in both directions.
        """
        if v_mps < 0.0 and self.breakaway_pwm_rev is not None:
            return self.breakaway_pwm_rev
        return self.breakaway_pwm

    def dead_pwm_for(self, v_mps: float) -> float:
        """The dead-band that applies when driving in v_mps's direction.

        Callers that need to know "is this PWM actually pushing the wheel"
        must use this rather than .dead_pwm, or they will test a reverse
        command against the forward dead-band.
        """
        return self._params(v_mps < 0.0)[0]

    def to_pwm(self, v_mps: float) -> int:
        if abs(v_mps) <= self.v_min_mps:
            return 0
        sign = 1.0 if v_mps > 0.0 else -1.0
        dead, gain = self._params(v_mps < 0.0)
        mag = dead + gain * abs(v_mps)
        pwm = int(round(sign * mag))
        if pwm > self.max_pwm:
            return self.max_pwm
        if pwm < -self.max_pwm:
            return -self.max_pwm
        return pwm

    def to_speed(self, pwm: float) -> float:
        """Inverse, for turning a commanded PWM back into an expected speed.

        Only meaningful for |pwm| > dead_pwm; inside the dead-band the wheel
        is not turning, so this returns 0.0. Uses the same direction's
        constants as to_pwm, so the two stay mutual inverses per direction.
        """
        dead, gain = self._params(pwm < 0.0)
        if abs(pwm) <= dead or gain <= 0.0:
            return 0.0
        sign = 1.0 if pwm > 0.0 else -1.0
        return sign * (abs(pwm) - dead) / gain


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
