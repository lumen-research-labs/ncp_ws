"""
base_node.py — open-loop mecanum base driver for the ROSMASTER X3.

What it does:
    Replaces the Yahboom `Mcnamu_driver_X3` node. Owns the base-board serial
    link (Rosmaster_Lib) and:
      * subscribes /cmd_vel, converts the body twist to four wheel speeds
        (rm_base.mecanum), maps each to a raw motor PWM (rm_base.pwm_map),
        and drives them with `set_motor` — OPEN LOOP, bypassing the STM32's
        wheel-speed PID entirely;
      * publishes /imu/data_raw, /imu/mag, /voltage, /edition unchanged from
        the vendor driver (so imu_filter_madgwick + the EKF are unaffected);
      * publishes /vel_raw reconstructed from the wheel encoders with the
        firmware sign bug corrected.

Why this exists (bernard-1, 2026-08-28):
    The STM32 firmware's closed-loop speed control is unusable on this robot.
    Measured per-motor: every working wheel reports its encoder with the
    WRONG SIGN relative to what the firmware's PID expects, so any nonzero
    `set_car_motion` command drives the PID straight to 100% PWM and the
    wheel runs flat out regardless of the commanded speed. The front-right
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
    /base/debug   [geometry_msgs/Twist]        intended per-wheel PWM
                  (linear.x/y/z = FL/BL/FR, angular.x = BR) — diagnostics only

Parameters:
    serial_port         (str)  by-id CH340 path; default is bernard-1's.
    calibration_path     (str) absolute path to config/base_calibration.yaml.
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
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

from rm_base import mecanum, pwm_map

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


class BaseNode(rclpy.node.Node):
    """Open-loop mecanum driver + sensor bridge for the X3 base board."""

    def __init__(self) -> None:
        super().__init__("rm_base_node")

        self.declare_parameter("serial_port", _DEFAULT_SERIAL_PORT)
        self.declare_parameter("calibration_path", "")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_timeout_ms", 300)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("sensor_rate_hz", 30.0)
        self.declare_parameter("imu_link", "imu_link")

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
        self._enc_prev: list[int] | None = None
        self._enc_prev_mono = time.monotonic()

        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self._mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)
        self._vel_pub = self.create_publisher(Twist, "/vel_raw", 10)
        self._volt_pub = self.create_publisher(Float32, "/voltage", 10)
        self._edition_pub = self.create_publisher(Float32, "/edition", 10)
        self._debug_pub = self.create_publisher(Twist, "/base/debug", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(1.0 / control_hz, self._control_tick)
        self.create_timer(1.0 / sensor_hz, self._sensor_tick)

        self.get_logger().warn(
            f"rm_base_node up on {port}. dry_run={self._dry_run} "
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

    # ---- command path ---------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_twist = mecanum.BodyTwist(
            vx=_clamp(msg.linear.x, self._max_body["vx"]),
            vy=_clamp(msg.linear.y, self._max_body["vy"]),
            wz=_clamp(msg.angular.z, self._max_body["wz"]),
        )
        self._last_cmd_mono = time.monotonic()

    def _control_tick(self) -> None:
        try:
            stale = (time.monotonic() - self._last_cmd_mono) > self._cmd_timeout_s
            if self._dry_run or stale:
                twist = mecanum.BodyTwist(0.0, 0.0, 0.0)
                if stale and not self._dry_run:
                    self.get_logger().warn(
                        "no /cmd_vel within timeout — holding zero PWM",
                        throttle_duration_sec=2.0,
                    )
            else:
                twist = self._last_twist

            wheels = mecanum.inverse(twist, self._lever_arm_m)
            targets = [
                self._wheel_models["front_left"].to_pwm(wheels.front_left),
                self._wheel_models["back_left"].to_pwm(wheels.back_left),
                self._wheel_models["front_right"].to_pwm(wheels.front_right),
                self._wheel_models["back_right"].to_pwm(wheels.back_right),
            ]
            applied = [
                pwm_map.slew_limit(self._applied_pwm[i], targets[i], self._max_slew)
                for i in range(4)
            ]
            self._car.set_motor(applied[0], applied[1], applied[2], applied[3])
            self._applied_pwm = applied

            dbg = Twist()
            dbg.linear.x, dbg.linear.y, dbg.linear.z = (
                float(applied[0]), float(applied[1]), float(applied[2]),
            )
            dbg.angular.x = float(applied[3])
            self._debug_pub.publish(dbg)
        except Exception as exc:  # noqa: BLE001 — fail-closed is the point
            self.get_logger().error(f"control tick failed ({exc!r}) — zeroing motors")
            self._safe_zero()

    # ---- sensor path --------------------------------------------------

    def _sensor_tick(self) -> None:
        now = self.get_clock().now().to_msg()
        try:
            ax, ay, az = self._car.get_accelerometer_data()
            gx, gy, gz = self._car.get_gyroscope_data()
            mx, my, mz = self._car.get_magnetometer_data()
            volts = self._car.get_battery_voltage()
            edition = self._car.get_version()
            enc = list(self._car.get_motor_encoder())
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
        self._publish_vel_raw(enc)

    def _publish_vel_raw(self, enc: list[int]) -> None:
        mono = time.monotonic()
        dt = mono - self._enc_prev_mono
        self._enc_prev_mono = mono
        if self._enc_prev is None or dt <= 0.0:
            self._enc_prev = enc
            return

        # Firmware reports forward rotation as a DECREASING count on this
        # robot (measured 2026-08-28) — negate so forward is positive.
        per_s = [-(enc[i] - self._enc_prev[i]) / dt for i in range(4)]
        self._enc_prev = enc
        rev_per_s = [c / self._counts_per_rev for c in per_s]
        speeds = mecanum.WheelSpeeds(
            front_left=rev_per_s[0] * self._wheel_circ_m,
            back_left=rev_per_s[1] * self._wheel_circ_m,
            front_right=rev_per_s[2] * self._wheel_circ_m,
            back_right=rev_per_s[3] * self._wheel_circ_m,
        )
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
