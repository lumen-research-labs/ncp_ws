"""
base_node.py — mecanum base driver for the ROSMASTER X3.

What it does:
    Replaces the Yahboom `Mcnamu_driver_X3` node. Owns the base-board serial
    link (Rosmaster_Lib) and:
      * subscribes /cmd_vel, converts the body twist to four wheel speeds
        (rm_base.mecanum), and drives each wheel with raw `set_motor` PWM,
        bypassing the STM32's wheel-speed PID entirely. Two control modes:

          open_loop   (default) PWM straight from the per-wheel calibration
                      in base_calibration.yaml. No feedback. Validated path.
          closed_loop that same calibration as FEEDFORWARD, plus a per-wheel
                      PI trim on the SIGN-CORRECTED encoder speed
                      (rm_base.wheel_control). This is the loop the firmware
                      should have been running; it exists because open loop
                      gives every wheel the PWM its calibration row says and
                      they still end up at different speeds under real load.
      * publishes /imu/data_raw, /imu/mag, /voltage, /edition unchanged from
        the vendor driver (so imu_filter_madgwick + the EKF are unaffected);
      * publishes /vel_raw reconstructed from the wheel encoders with the
        firmware sign bug corrected.

Why this exists (bernard-1, 2026-08-28):
    The STM32 firmware's closed-loop speed control is unusable on this robot.
    Measured per-motor: every working wheel reports its encoder with the
    WRONG SIGN relative to what the firmware's PID expects, so any nonzero
    `set_car_motion` command drives the PID straight to 100% PWM and the
    wheel runs flat out regardless of the commanded speed. Note what that
    does and does not condemn: the firmware's LOOP was fine, its FEEDBACK
    POLARITY was not. Correcting the sign in ROS (below) makes the same
    feedback usable, which is why closed_loop mode is possible at all. The front-right
    motor (M3) is also failing — it needs ~5x the PWM of the others to move
    and its encoder only reads at speed. `set_motor` (raw PWM) was verified
    clean and roughly proportional on the three good wheels, so this node
    drives that path and does the kinematics itself.

    Motor channel -> wheel, MEASURED, not the Yahboom sample order:
        M1 = front-left   M2 = back-left   M3 = front-right   M4 = back-right
    For every wheel, a POSITIVE `set_motor` value drives it FORWARD, and a
    forward-turning wheel makes `get_motor_encoder` count DOWN — hence the
    negation in the odometry path.

Subscribes:
    /cmd_vel      [geometry_msgs/Twist]   the safety-clamped command. This
                  node is a pure CONSUMER of /cmd_vel; rm_safety remains its
                  sole publisher.

Publishes:
    /imu/data_raw [sensor_msgs/Imu]            @ sensor_rate_hz
    /imu/mag      [sensor_msgs/MagneticField]  @ sensor_rate_hz
    /vel_raw      [geometry_msgs/Twist]        @ sensor_rate_hz, no header
    /voltage      [std_msgs/Float32]           @ sensor_rate_hz
    /edition      [std_msgs/Float32]           @ sensor_rate_hz
    /base/debug   [rm_interfaces/BaseDebug]     per-wheel target/measured
                  speed, feedforward, integral, target and applied PWM, plus
                  the feedback_valid / saturated / stalled / sign_fault flags
                  — diagnostics only, nothing in the control path reads it.
                  Populated in dry_run too: only `applied_pwm` is gated on it.

Parameters:
    serial_port         (str)  by-id CH340 path; default is bernard-1's.
    calibration_path     (str) absolute path to config/base_calibration.yaml.
    control_mode        (str, "open_loop") "open_loop" | "closed_loop".
                  Defaults to the validated open-loop path; closed_loop must
                  be asked for explicitly and proven wheels-up first.
    dry_run             (bool, TRUE) compute and publish everything but send
                  only zero PWM. Motion needs BOTH dry_run:=false AND an
                  armed rm_safety — two explicit acts, because there is no
                  physical E-stop.
    command_timeout_ms  (int, 300) no /cmd_vel within this -> zero PWM.
    control_rate_hz     (float, 20.0) PWM update / watchdog rate.
    sensor_rate_hz      (float, 30.0) IMU / voltage / vel_raw publish rate.
    imu_link            (str, "imu_link") IMU + mag frame_id.

Failure mode (fail-closed everywhere — there is NO physical E-stop):
    * Rosmaster_Lib cannot open the port -> raise in __init__, node does not
      start.
    * No /cmd_vel within command_timeout_ms -> zero PWM every control tick.
      (The vendor driver had NO watchdog; that is the whole "the robot does
      not stop on its own" hazard in the robot-ssh skill. This node fixes it.)
    * Any exception in the control tick -> set_motor(0,0,0,0), log ERROR,
      never leave the last PWM applied.
    * closed_loop with no fresh/plausible encoder report -> that wheel
      degrades to feedforward, i.e. to open-loop behaviour. Feedback is never
      assumed; a missing measurement is never read as "speed is zero".
    * closed_loop with a wheel measurably turning the WRONG WAY -> latched
      sign_fault, that wheel is held at zero PWM and the node logs ERROR.
      That is the firmware runaway's signature and must never be ridden out.
    * dry_run true (default) -> never a nonzero PWM.
    * On shutdown / SIGINT -> flood set_motor(0,0,0,0) before closing serial.
    * A serial read that throws is skipped for that tick; stale sensor data
      is never republished with a fresh stamp beyond one tick.
"""
from __future__ import annotations

import time

import rclpy
import rclpy.node
import yaml
from geometry_msgs.msg import Twist
from rm_interfaces.msg import BaseDebug
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

from rm_base import mecanum, pwm_map, wheel_control

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:  # workstation / CI — the lib is robot-only
    Rosmaster = None

# Firmware constants (Rosmaster board, bsp_encoder.h): 30 gear * 11 PPR * 2
# edges * 2 channels. Wheel circumference is the "mecanum mini" value from
# bsp_motion.h; both are overridable from the calibration file.
_COUNTS_PER_REV = 1320.0
_WHEEL_CIRCUMFERENCE_M = 0.204

_DEFAULT_SERIAL_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
)

# Index order for every per-wheel array in this module and in BaseDebug.msg.
# MEASURED motor->wheel map (2026-08-28): M1=FL, M2=BL, M3=FR, M4=BR. This is
# NOT the Yahboom sample order; do not "fix" it back.
_WHEELS = ("front_left", "back_left", "front_right", "back_right")

_OPEN_LOOP = "open_loop"
_CLOSED_LOOP = "closed_loop"


class BaseNode(rclpy.node.Node):
    """Mecanum driver + sensor bridge for the X3 base board.

    Drives raw `set_motor` PWM in either control mode; the firmware speed PID
    is never used. See the module docstring for why.
    """

    def __init__(self) -> None:
        super().__init__("rm_base_node")

        self.declare_parameter("serial_port", _DEFAULT_SERIAL_PORT)
        self.declare_parameter("calibration_path", "")
        self.declare_parameter("control_mode", _OPEN_LOOP)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_timeout_ms", 300)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("sensor_rate_hz", 30.0)
        self.declare_parameter("imu_link", "imu_link")

        self._mode = str(self.get_parameter("control_mode").value)
        if self._mode not in (_OPEN_LOOP, _CLOSED_LOOP):
            raise RuntimeError(
                f"control_mode must be '{_OPEN_LOOP}' or '{_CLOSED_LOOP}', "
                f"got {self._mode!r}"
            )
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._cmd_timeout_s = int(self.get_parameter("command_timeout_ms").value) / 1000.0
        self._imu_link = str(self.get_parameter("imu_link").value)
        control_hz = float(self.get_parameter("control_rate_hz").value)
        sensor_hz = float(self.get_parameter("sensor_rate_hz").value)

        cal = self._load_calibration(str(self.get_parameter("calibration_path").value))
        self._lever_arm_m = float(cal["lever_arm_m"])
        self._counts_per_rev = float(cal.get("counts_per_rev", _COUNTS_PER_REV))
        self._wheel_circ_m = float(cal.get("wheel_circumference_m", _WHEEL_CIRCUMFERENCE_M))
        self._max_slew = int(cal.get("max_pwm_step_per_tick", 8))
        self._max_body = cal["max_body"]  # defensive clamp; rm_safety is primary
        self._wheel_models = self._build_wheel_models(cal["wheels"])
        self._controllers = self._build_controllers(cal)
        self._encoders = wheel_control.EncoderTracker(
            counts_per_rev=self._counts_per_rev,
            wheel_circumference_m=self._wheel_circ_m,
            max_wheel_mps=float(cal.get("max_plausible_wheel_mps", 2.0)),
        )

        if Rosmaster is None:
            raise RuntimeError("Rosmaster_Lib not importable — this node is robot-only")
        port = str(self.get_parameter("serial_port").value)
        self._car = Rosmaster(com=port, delay=0.002, debug=False)
        self._car.set_car_type(1)  # X3
        self._car.create_receive_threading()
        self._car.set_auto_report_state(True, forever=False)
        time.sleep(0.5)
        self._flood_stop(0.4)

        self._last_twist = mecanum.BodyTwist(0.0, 0.0, 0.0)
        self._last_cmd_mono = 0.0
        self._applied_pwm = [0, 0, 0, 0]
        self._measured = (0.0, 0.0, 0.0, 0.0)
        self._control_prev_mono = time.monotonic()

        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self._mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)
        self._vel_pub = self.create_publisher(Twist, "/vel_raw", 10)
        self._volt_pub = self.create_publisher(Float32, "/voltage", 10)
        self._edition_pub = self.create_publisher(Float32, "/edition", 10)
        self._debug_pub = self.create_publisher(BaseDebug, "/base/debug", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(1.0 / control_hz, self._control_tick)
        self.create_timer(1.0 / sensor_hz, self._sensor_tick)

        self.get_logger().warn(
            f"rm_base_node up on {port}. mode={self._mode} "
            f"dry_run={self._dry_run} "
            f"(motion needs dry_run:=false AND an armed rm_safety). "
            f"control={control_hz:.0f}Hz sensor={sensor_hz:.0f}Hz"
        )

    # ---- config -----------------------------------------------------------

    def _load_calibration(self, path: str) -> dict:
        if not path:
            raise RuntimeError(
                "calibration_path is required — pass config/base_calibration.yaml"
            )
        with open(path) as fh:
            cal = yaml.safe_load(fh)
        for key in ("lever_arm_m", "wheels", "max_body"):
            if key not in cal:
                raise RuntimeError(f"base_calibration.yaml missing '{key}'")
        for wheel in ("front_left", "back_left", "front_right", "back_right"):
            if wheel not in cal["wheels"]:
                raise RuntimeError(f"base_calibration.yaml wheels missing '{wheel}'")
        return cal

    @staticmethod
    def _build_wheel_models(wheels: dict) -> dict[str, pwm_map.WheelPwmModel]:
        out = {}
        for name, p in wheels.items():
            out[name] = pwm_map.WheelPwmModel(
                dead_pwm=float(p["dead_pwm"]),
                gain_pwm_per_mps=float(p["gain_pwm_per_mps"]),
                v_min_mps=float(p.get("v_min_mps", 0.02)),
                max_pwm=int(p.get("max_pwm", 60)),
            )
        return out

    def _build_controllers(self, cal: dict) -> list[wheel_control.WheelController]:
        """One controller per wheel, in _WHEELS order.

        Built in both modes so that /base/debug reports the same fields
        either way; in open_loop the controllers are simply never stepped.
        Gains come from the calibration file's optional `closed_loop` block,
        with a per-wheel `gains` override — the failing front-right motor
        needs more integral authority than the three healthy wheels.
        """
        block = cal.get("closed_loop", {}) or {}
        defaults = block.get("gains", {}) or {}
        guards = wheel_control.Guards(**(block.get("guards", {}) or {}))
        out = []
        for name in _WHEELS:
            over = {**defaults, **((cal["wheels"][name].get("gains") or {}))}
            out.append(
                wheel_control.WheelController(
                    model=self._wheel_models[name],
                    gains=wheel_control.PiGains(**over),
                    guards=guards,
                )
            )
        return out

    # ---- command path ---------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_twist = mecanum.BodyTwist(
            vx=_clamp(msg.linear.x, self._max_body["vx"]),
            vy=_clamp(msg.linear.y, self._max_body["vy"]),
            wz=_clamp(msg.angular.z, self._max_body["wz"]),
        )
        self._last_cmd_mono = time.monotonic()

    def _control_tick(self) -> None:
        """Read feedback, compute PWM, write it, publish diagnostics.

        The encoders are read HERE and not in the sensor tick so that the
        control loop's measurement and its dt come from the same clock as its
        output. The sensor tick only publishes /vel_raw from whatever this
        tick last measured.
        """
        try:
            now_mono = time.monotonic()
            dt = now_mono - self._control_prev_mono
            self._control_prev_mono = now_mono

            stale = (now_mono - self._last_cmd_mono) > self._cmd_timeout_s
            if stale and not self._dry_run and any(self._applied_pwm):
                self.get_logger().warn(
                    "no /cmd_vel within timeout — holding zero PWM",
                    throttle_duration_sec=2.0,
                )

            # dry_run does NOT zero the command: the operator needs to see
            # what the command WOULD have produced. It zeroes only the PWM
            # written to the board, at the very end of this method.
            driven = mecanum.BodyTwist(0.0, 0.0, 0.0) if stale else self._last_twist
            targets_mps = mecanum.inverse(driven, self._lever_arm_m).as_tuple()

            measured, fresh = self._read_feedback(dt)
            # In dry_run the wheels are off the ground or not being driven, so
            # a zero reading is not evidence about anything — never close the
            # loop on it.
            usable = fresh and not self._dry_run and self._mode == _CLOSED_LOOP

            commands = self._compute(targets_mps, measured, dt, usable)
            target_pwm = [c.pwm for c in commands]

            if self._dry_run:
                applied = [0, 0, 0, 0]
            else:
                applied = [
                    pwm_map.slew_limit(
                        self._applied_pwm[i], target_pwm[i], self._max_slew
                    )
                    for i in range(4)
                ]
            self._car.set_motor(applied[0], applied[1], applied[2], applied[3])
            self._applied_pwm = applied

            self._warn_on_faults(commands)
            self._publish_debug(targets_mps, measured, commands,
                                target_pwm, applied, usable, stale)
        except Exception as exc:  # noqa: BLE001 — fail-closed is the point
            self.get_logger().error(f"control tick failed ({exc!r}) — zeroing motors")
            self._safe_zero()

    def _compute(
        self,
        targets_mps: tuple[float, ...],
        measured: tuple[float, ...],
        dt: float,
        usable: bool,
    ) -> list[wheel_control.WheelCommand]:
        """Per-wheel PWM. open_loop bypasses the controllers entirely."""
        if self._mode == _OPEN_LOOP:
            out = []
            for i, name in enumerate(_WHEELS):
                pwm = self._wheel_models[name].to_pwm(targets_mps[i])
                out.append(
                    wheel_control.WheelCommand(
                        pwm=pwm, feedforward_pwm=float(pwm), integral_pwm=0.0,
                        error_mps=0.0,
                        saturated=abs(pwm) >= self._wheel_models[name].max_pwm,
                        stalled=False, sign_fault=False,
                    )
                )
            return out
        # self._applied_pwm is LAST tick's post-slew output. The sign guard
        # needs it to tell "this wheel is wired backwards" apart from "the
        # slew limiter has not finished reversing this wheel yet".
        return [
            self._controllers[i].update(
                targets_mps[i], measured[i], dt, usable,
                applied_pwm=self._applied_pwm[i],
            )
            for i in range(4)
        ]

    def _read_feedback(self, dt: float) -> tuple[tuple[float, ...], bool]:
        """Sign-corrected per-wheel speed from the encoders, plus freshness.

        A failed serial read is not a zero measurement — it degrades the loop
        to feedforward for this tick and nothing more.
        """
        try:
            counts = list(self._car.get_motor_encoder())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"encoder read failed ({exc!r}) — feedforward only this tick",
                throttle_duration_sec=2.0,
            )
            return self._measured, False
        speeds, fresh, _window = self._encoders.update(counts, dt)
        self._measured = speeds
        return speeds, fresh

    def _warn_on_faults(self, commands: list[wheel_control.WheelCommand]) -> None:
        for i, cmd in enumerate(commands):
            if cmd.sign_fault:
                self.get_logger().error(
                    f"{_WHEELS[i]}: encoder polarity fault — wheel turning "
                    "against the command. Holding this wheel at zero PWM. "
                    "This is the firmware runaway signature; STOP and "
                    "re-measure the encoder sign before driving.",
                    throttle_duration_sec=5.0,
                )
            elif cmd.stalled:
                self.get_logger().warn(
                    f"{_WHEELS[i]}: commanded but not turning — integrator "
                    "frozen (feedforward only).",
                    throttle_duration_sec=5.0,
                )

    def _publish_debug(
        self,
        targets_mps: tuple[float, ...],
        measured: tuple[float, ...],
        commands: list[wheel_control.WheelCommand],
        target_pwm: list[int],
        applied: list[int],
        usable: bool,
        stale: bool,
    ) -> None:
        msg = BaseDebug()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.commanded_twist.linear.x = self._last_twist.vx
        msg.commanded_twist.linear.y = self._last_twist.vy
        msg.commanded_twist.angular.z = self._last_twist.wz
        msg.control_mode = self._mode
        msg.dry_run = self._dry_run
        msg.command_stale = stale
        msg.target_speed_mps = [float(v) for v in targets_mps]
        msg.measured_speed_mps = [float(v) for v in measured]
        msg.feedforward_pwm = [float(c.feedforward_pwm) for c in commands]
        msg.integral_pwm = [float(c.integral_pwm) for c in commands]
        msg.target_pwm = [int(v) for v in target_pwm]
        msg.applied_pwm = [int(v) for v in applied]
        msg.feedback_valid = [usable] * 4
        msg.saturated = [c.saturated for c in commands]
        msg.stalled = [c.stalled for c in commands]
        msg.sign_fault = [c.sign_fault for c in commands]
        self._debug_pub.publish(msg)

    # ---- sensor path --------------------------------------------------

    def _sensor_tick(self) -> None:
        now = self.get_clock().now().to_msg()
        try:
            ax, ay, az = self._car.get_accelerometer_data()
            gx, gy, gz = self._car.get_gyroscope_data()
            mx, my, mz = self._car.get_magnetometer_data()
            volts = self._car.get_battery_voltage()
            edition = self._car.get_version()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"sensor read failed ({exc!r}) — skipping this tick",
                throttle_duration_sec=2.0,
            )
            return

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self._imu_link
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0
        self._imu_pub.publish(imu)

        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self._imu_link
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = my * 1.0
        mag.magnetic_field.z = mz * 1.0
        self._mag_pub.publish(mag)

        self._volt_pub.publish(Float32(data=volts * 1.0))
        self._edition_pub.publish(Float32(data=edition * 1.0))
        self._publish_vel_raw()

    def _publish_vel_raw(self) -> None:
        """Body twist from the wheel speeds the CONTROL tick last measured.

        The encoders are deliberately not read here: two readers with two
        different periods would each see a fraction of the board's
        auto-report updates and both would compute nonsense deltas. The
        control tick owns the measurement; this just republishes it in the
        vendor driver's shape (bare Twist, no header).
        """
        speeds = mecanum.WheelSpeeds(*self._measured)
        body = mecanum.forward(speeds, self._lever_arm_m)
        out = Twist()
        out.linear.x = body.vx
        out.linear.y = body.vy
        out.angular.z = body.wz
        self._vel_pub.publish(out)

    # ---- stop helpers -------------------------------------------------

    def _safe_zero(self) -> None:
        try:
            self._car.set_motor(0, 0, 0, 0)
        except Exception:  # noqa: BLE001
            pass
        self._applied_pwm = [0, 0, 0, 0]
        # Drop every integrator too: windup accumulated before a fault must
        # never be carried into the first tick after recovery.
        for ctrl in self._controllers:
            ctrl.reset()

    def _flood_stop(self, secs: float) -> None:
        end = time.time() + secs
        while time.time() < end:
            self._safe_zero()
            time.sleep(0.03)

    def shutdown(self) -> None:
        self._flood_stop(1.0)


def _clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BaseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
