"""
wheel_control.py — pure per-wheel closed-loop speed control for rm_base.

What it does:
    Implements one wheel's speed controller as feedforward + PI on the speed
    error, with the encoder feedback the STM32 firmware could not use:

        pwm = ff(target) + integral + kp * (target - measured)

    `ff` is the existing open-loop calibration (rm_base.pwm_map.WheelPwmModel),
    so the loop starts near the right PWM and the integrator only has to
    absorb per-wheel differences — load, tyre wear, and bernard-1's weak
    front-right motor. That is the whole point: open loop, every wheel gets
    the PWM its calibration row says, and they end up at different speeds on
    the ground; closed loop, each wheel is driven to the speed the kinematics
    asked for.

Why the firmware's own PID cannot be used (bernard-1, 2026-08-28):
    Every wheel reports its encoder with the WRONG SIGN relative to what the
    firmware PID expects, so `err = target - measured` never shrinks and the
    output integrates to the +/-3600 rail. This module consumes the
    SIGN-CORRECTED feedback computed in base_node (forward rotation makes
    `get_motor_encoder` count DOWN on this robot, so the delta is negated),
    which is exactly the term the firmware got backwards.

    Because that failure mode is a runaway with no physical E-stop to catch
    it, this controller carries three guards that the firmware's did not:

      * anti-windup — the integrator never grows in the direction that
        deepens an output saturation, and is hard-clamped to i_max_pwm;
      * stall detection — a wheel commanded to move that reports no motion
        for stall_ticks stops integrating and holds feedforward. Without
        this, bernard-1's failing front-right motor (whose encoder only
        reads once it is already turning fast) would wind the integrator to
        the ceiling on every start, reproducing the firmware bug in ROS;
      * sign guard — if the wheel is measurably moving AGAINST the commanded
        direction, the feedback polarity is wrong, so the loop is unsafe.
        Output is forced to zero and the integrator reset. This is the
        tripwire for the firmware sign bug reappearing or being "fixed"
        into the opposite polarity by a reflash.

Failure mode:
    Pure arithmetic, never raises, no I/O. Every guard fails toward LESS
    authority: an invalid or missing measurement degrades the loop to pure
    feedforward (i.e. to the previously validated open-loop behaviour), and
    a sign fault degrades it to zero PWM. State lives in the instance, so a
    controller is per-wheel and must not be shared.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from rm_base.pwm_map import WheelPwmModel


@dataclass(frozen=True)
class PiGains:
    """Closed-loop gains for one wheel, in PWM units per (m/s).

    kp/ki are small because feedforward already supplies the bulk of the
    PWM — these only trim the residual. i_max_pwm bounds the integrator's
    total authority so a bad measurement can never command more than
    feedforward plus that margin.
    """

    kp: float = 60.0
    ki: float = 80.0
    i_max_pwm: float = 25.0


@dataclass(frozen=True)
class Guards:
    """Thresholds for the three fail-toward-less-authority guards."""

    # A wheel commanded above this is expected to be turning.
    stall_target_mps: float = 0.03
    # Measured speed below this counts as "not turning".
    stall_measured_mps: float = 0.01
    # Consecutive stalled ticks before the integrator is frozen.
    stall_ticks: int = 4
    # Reverse motion must exceed this before the sign guard trips — below it
    # the reading is encoder quantisation noise, not real reverse rotation.
    sign_fault_mps: float = 0.05
    # Ticks after a commanded direction change during which the sign guard is
    # SUSPENDED entirely. Resetting the counter on a reversal (see update())
    # was not enough on hardware: front-right has the highest dead_pwm of the
    # four, so it takes the longest to coast down and start turning the other
    # way, and it still latched inside the post-reset window. Polarity is a
    # static property of the wiring — there is nothing to learn from a wheel
    # mid-reversal, so do not judge it then at all.
    sign_settle_ticks: int = 10
    # ---- breakaway kick ---------------------------------------------------
    # A sticky motor needs far more PWM to START than to keep going
    # (bernard-1's front-right: ~60 to break loose, ~30 + gain to sustain).
    # The feedforward is fitted to the RUNNING curve, so at low speeds it
    # never reaches breakaway and the wheel simply never starts.
    #
    # The integrator cannot solve this and must not be asked to: the stall
    # guard freezes it precisely so a stalled wheel cannot wind the output to
    # the rail, which is the firmware runaway this package exists to escape.
    # So the kick is a SEPARATE, BOUNDED, EXPLICIT action instead — a fixed
    # number of ticks at the measured breakaway PWM, then back to the normal
    # model, with a cooldown and a hard attempt cap.
    #
    # It only ever applies to a wheel with breakaway_pwm configured, and only
    # while that wheel is commanded and reading stalled.
    kick_ticks: int = 8           # 0.4 s at 20 Hz — enough for the slew
                                  # limiter to walk the PWM up to breakaway
    kick_cooldown_ticks: int = 20  # 1.0 s between attempts
    max_kick_attempts: int = 3     # then give up until the command clears —
                                   # a genuinely blocked motor must not be
                                   # driven at breakaway PWM indefinitely
    # Consecutive wrong-direction ticks before the sign guard trips.
    #
    # 3 (0.15 s at 20 Hz) was far too short and latched every wheel on the
    # first hardware run of the full stack (2026-08-29). When the commanded
    # direction REVERSES — which the debug policy does constantly, alternating
    # cruise (all wheels forward) with spin (left and right opposed) — the
    # wheel is still coasting the old way while the new target points the
    # other. That is momentum, not wrong polarity, and it lasts far longer
    # than 0.15 s.
    #
    # 12 ticks = 0.6 s. A REAL polarity inversion reports the wrong direction
    # continuously and forever, so it still trips in well under a second; a
    # spin-down through zero does not. Paired with the direction-change reset
    # in update(), which gives each reversal a fresh window.
    sign_fault_ticks: int = 12


@dataclass
class WheelCommand:
    """One tick of controller output, for /base/debug and for the caller."""

    pwm: int
    feedforward_pwm: float
    integral_pwm: float
    error_mps: float
    saturated: bool
    stalled: bool
    sign_fault: bool
    # True on the ticks where a breakaway kick is being applied, so the caller
    # can log it. A wheel that is kicking is also stalled by definition.
    kicking: bool = False
    # True once this wheel has used up max_kick_attempts without moving.
    # Distinguishes "sticky, being helped" from "not responding at all".
    kick_exhausted: bool = False


@dataclass
class WheelController:
    """Feedforward + PI speed controller for a single wheel."""

    model: WheelPwmModel
    gains: PiGains = field(default_factory=PiGains)
    guards: Guards = field(default_factory=Guards)

    _integral: float = 0.0
    _stall_ticks: int = 0
    _sign_ticks: int = 0
    _sign_latched: bool = False
    # Sign of last tick's target (+1/-1/0), to spot a commanded reversal.
    _prev_target_sign: int = 0
    # Ticks left in the post-reversal window where the sign guard is suspended.
    _settle_ticks: int = 0
    # Breakaway-kick state.
    _kick_ticks_left: int = 0
    _kick_cooldown_left: int = 0
    _kick_attempts: int = 0

    # ---- state ------------------------------------------------------------

    def reset(self) -> None:
        """Drop all accumulated state. Called on stop and on mode change."""
        self._integral = 0.0
        self._stall_ticks = 0
        self._sign_ticks = 0
        self._sign_latched = False
        self._prev_target_sign = 0
        self._settle_ticks = 0
        self._kick_ticks_left = 0
        self._kick_cooldown_left = 0
        self._kick_attempts = 0

    @property
    def sign_fault_latched(self) -> bool:
        """True once the feedback polarity has been judged wrong.

        Latched deliberately: a polarity fault is a wiring/firmware fact, not
        a transient, and silently recovering from it would hide the exact
        failure this package exists to work around. Cleared only by reset().
        """
        return self._sign_latched

    # ---- control ----------------------------------------------------------

    def update(
        self,
        target_mps: float,
        measured_mps: float,
        dt: float,
        feedback_valid: bool,
        applied_pwm: float | None = None,
    ) -> WheelCommand:
        """Advance the loop one tick and return the PWM to apply.

        feedback_valid is False whenever `measured_mps` cannot be trusted for
        this tick — no fresh encoder report, an implausible jump, or dry_run
        (the wheels are off the ground / not being driven, so a zero reading
        is not evidence of anything). In that case the loop degrades to pure
        feedforward and the integrator is held, never grown.
        """
        feedforward = float(self.model.to_pwm(target_mps))

        # A latched polarity fault outranks everything: zero output.
        if self._sign_latched:
            return WheelCommand(0, feedforward, 0.0, 0.0, False, False, True)

        # Commanding a stop stops the loop too — never integrate toward a
        # standstill, and never carry windup across a stop into the next move.
        if self.model.to_pwm(target_mps) == 0:
            self.reset()
            return WheelCommand(0, 0.0, 0.0, 0.0, False, False, False)

        if not feedback_valid or dt <= 0.0:
            pwm = self._clamp(feedforward + self._integral)
            return WheelCommand(
                pwm, feedforward, self._integral, 0.0,
                abs(pwm) >= self.model.max_pwm, False, False,
            )

        error = target_mps - measured_mps

        # A commanded REVERSAL restarts the sign guard's window. The wheel
        # cannot obey instantly: it has to coast down through zero first, and
        # during that time it is genuinely moving against the new target
        # without anything being wrong. Judging polarity across that boundary
        # is what latched all four wheels on the first full-stack run.
        target_sign = (target_mps > 0.0) - (target_mps < 0.0)
        # Only a genuine REVERSAL (+1 -> -1 or -1 -> +1) opens a settle
        # window. Starting from rest (previous sign 0) is not a reversal:
        # there is no momentum to coast off, so the guard applies immediately
        # and a truly inverted wheel is caught on its very first move.
        reversed_direction = (
            target_sign != 0
            and self._prev_target_sign != 0
            and target_sign != self._prev_target_sign
        )
        if reversed_direction:
            self._sign_ticks = 0
            self._settle_ticks = self.guards.sign_settle_ticks
        if target_sign != 0:
            self._prev_target_sign = target_sign

        # Are we ACTUALLY pushing the wheel the way we asked it to go?
        #
        # base_node slew-limits the PWM (max_pwm_step_per_tick) AFTER this
        # returns, so on a reversal the applied PWM crawls from the old
        # direction, through zero, to the new one — measured at ~8.5 ticks
        # coming out of cruise. For all of that time the wheel is still being
        # driven the OLD way while the target points the new way, so of course
        # it keeps moving the old way. Judging polarity there is judging the
        # slew limiter, not the wiring. `applied_pwm` is last tick's actual
        # output; None means the caller cannot say, and the guard falls back
        # to the time-based window alone.
        if applied_pwm is None:
            driving_as_commanded = True
        else:
            # dead_pwm_for, not .dead_pwm: a wheel with a separate reverse
            # calibration must be tested against the dead-band for the
            # direction it is being asked to go.
            driving_as_commanded = (
                (applied_pwm > 0.0) == (target_mps > 0.0)
                and abs(applied_pwm) > self.model.dead_pwm_for(target_mps)
            )

        if not driving_as_commanded:
            # Not yet pushing it that way: nothing to conclude, and re-arm the
            # settle window so the wheel also gets time to physically turn
            # around once the PWM does arrive.
            self._settle_ticks = self.guards.sign_settle_ticks
            self._sign_ticks = 0
            moving_backwards = False
        elif self._settle_ticks > 0:
            # Driving correctly now, but the wheel is still coasting the old
            # way. Momentum, not polarity.
            self._settle_ticks -= 1
            moving_backwards = False
        else:
            moving_backwards = (
                abs(measured_mps) > self.guards.sign_fault_mps
                and (measured_mps > 0.0) != (target_mps > 0.0)
            )
        self._sign_ticks = self._sign_ticks + 1 if moving_backwards else 0
        if self._sign_ticks >= self.guards.sign_fault_ticks:
            self._sign_latched = True
            self._integral = 0.0
            return WheelCommand(0, feedforward, 0.0, error, False, False, True)

        stalling = (
            abs(target_mps) > self.guards.stall_target_mps
            and abs(measured_mps) < self.guards.stall_measured_mps
        )
        self._stall_ticks = self._stall_ticks + 1 if stalling else 0
        stalled = self._stall_ticks >= self.guards.stall_ticks

        # Provisional output decides whether integrating further is allowed.
        provisional = feedforward + self._integral + self.gains.kp * error
        saturated_high = provisional >= self.model.max_pwm
        saturated_low = provisional <= -self.model.max_pwm
        winding_up = (error > 0.0 and saturated_high) or (error < 0.0 and saturated_low)

        if not stalled and not winding_up:
            self._integral += self.gains.ki * error * dt
            self._integral = _clamp_abs(self._integral, self.gains.i_max_pwm)

        pwm = self._clamp(feedforward + self._integral + self.gains.kp * error)

        kicking, exhausted = self._breakaway_kick(target_mps, stalled)
        if kicking:
            # Deliberately REPLACES the loop output rather than adding to it:
            # the kick is an open-loop nudge of known size, not extra
            # controller authority. The integrator is already frozen by the
            # stall guard, so nothing accumulates during a kick.
            breakaway = self.model.breakaway_for(target_mps)
            sign = 1.0 if target_mps > 0.0 else -1.0
            kicked = self._clamp(sign * abs(breakaway))
            # Never let the kick REDUCE the output — if the loop already asks
            # for more than breakaway, the wheel is not short of push.
            if abs(kicked) > abs(pwm):
                pwm = kicked
            else:
                kicking = False

        return WheelCommand(
            pwm, feedforward, self._integral, error,
            abs(pwm) >= self.model.max_pwm, stalled, False,
            kicking, exhausted,
        )

    def _breakaway_kick(self, target_mps: float, stalled: bool) -> tuple[bool, bool]:
        """Run the kick state machine. Returns (kicking, exhausted).

        Only ever fires for a wheel with a measured breakaway_pwm, and only
        while that wheel is commanded and reading stalled. Bounded three ways:
        each attempt lasts kick_ticks, attempts are separated by
        kick_cooldown_ticks, and there are at most max_kick_attempts before it
        gives up — because a wheel that will not move after three kicks is
        blocked or broken, and holding a blocked motor at breakaway PWM is how
        it burns out.
        """
        if self.model.breakaway_for(target_mps) is None:
            return False, False

        if not stalled:
            # It is turning: clear the attempt budget so the next genuine
            # standstill starts fresh.
            self._kick_ticks_left = 0
            self._kick_cooldown_left = 0
            self._kick_attempts = 0
            return False, False

        if self._kick_ticks_left > 0:
            self._kick_ticks_left -= 1
            if self._kick_ticks_left == 0:
                self._kick_cooldown_left = self.guards.kick_cooldown_ticks
            return True, False

        if self._kick_cooldown_left > 0:
            self._kick_cooldown_left -= 1
            return False, False

        if self._kick_attempts >= self.guards.max_kick_attempts:
            return False, True

        self._kick_attempts += 1
        # kick_ticks - 1: THIS tick is the first of the attempt. Setting the
        # full count here would spend kick_ticks + 1 ticks per attempt and
        # overrun the cap that bounds how long a blocked motor is driven.
        self._kick_ticks_left = max(self.guards.kick_ticks - 1, 0)
        if self._kick_ticks_left == 0:
            self._kick_cooldown_left = self.guards.kick_cooldown_ticks
        return True, False

    def _clamp(self, value: float) -> int:
        return int(round(_clamp_abs(value, float(self.model.max_pwm))))


def _clamp_abs(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


@dataclass
class EncoderTracker:
    """Turns raw `get_motor_encoder` samples into per-wheel ground speeds.

    Two robot-specific facts are handled here rather than in the controller:

      * SIGN — on bernard-1 a forward-turning wheel makes the count DECREASE
        (measured 2026-08-28), so the delta is negated. This is the term the
        firmware PID gets backwards.
      * REPORT RATE — `get_motor_encoder` reads a value cached by
        Rosmaster_Lib's auto-report thread, not the wire. Polling it faster
        than the board reports yields runs of identical samples; dividing an
        unchanged count by the control period would fabricate a zero speed
        and, worse, feed that zero to the loop as if it were a measurement.
        So a sample identical to the previous one is reported as NOT fresh,
        and dt accumulates until the count actually moves.

    Failure mode:
        Pure arithmetic. An implausible jump (faster than max_wheel_mps) is
        discarded and reported not-fresh, so the loop degrades to
        feedforward rather than reacting to a serial glitch.
    """

    counts_per_rev: float
    wheel_circumference_m: float
    max_wheel_mps: float = 2.0

    _prev: list[int] | None = None
    _elapsed: float = 0.0
    _last: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    def reset(self) -> None:
        self._prev = None
        self._elapsed = 0.0
        self._last = (0.0, 0.0, 0.0, 0.0)

    def update(
        self, counts: list[int], dt: float
    ) -> tuple[tuple[float, float, float, float], bool, float]:
        """Return (speeds_mps, fresh, dt_used) in (FL, BL, FR, BR) order."""
        self._elapsed += max(dt, 0.0)
        if self._prev is None:
            self._prev = list(counts)
            self._elapsed = 0.0
            return self._last, False, 0.0
        if list(counts) == self._prev or self._elapsed <= 0.0:
            return self._last, False, 0.0

        window = self._elapsed
        metres_per_count = self.wheel_circumference_m / self.counts_per_rev
        speeds = tuple(
            -(counts[i] - self._prev[i]) * metres_per_count / window for i in range(4)
        )
        self._prev = list(counts)
        self._elapsed = 0.0
        if any(abs(s) > self.max_wheel_mps for s in speeds):
            return self._last, False, 0.0

        self._last = speeds  # type: ignore[assignment]
        return self._last, True, window
