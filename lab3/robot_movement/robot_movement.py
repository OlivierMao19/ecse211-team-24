import math
from time import sleep, time
from typing import Optional

from gyro_sensor.gyro_sensor import GyroSensor
from utils.brick import Motor


class RobotMovement:
    left_motor: Motor
    right_motor: Motor
    gyro_sensor: Optional[GyroSensor]

    BASE_R_POWER: int = 20
    BASE_L_POWER: int = 10
    WHEEL_DIAMETER_CM: float = 5.6
    TURN_SLOWDOWN_DEG: float = 20.0
    MIN_TURN_POWER: float = 7.0
    HEADING_GAIN: float = 1.2
    MAX_HEADING_CORRECTION: float = 6.0

    def __init__(
        self,
        left_motor: Motor,
        right_motor: Motor,
        gyro_sensor: Optional[GyroSensor] = None,
        left_motor_sign: int = 1,
        right_motor_sign: int = 1,
        straight_left_trim: float = 0.0,
        straight_right_trim: float = 0.0,
    ):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyro_sensor = gyro_sensor
        self.left_motor_sign = left_motor_sign
        self.right_motor_sign = right_motor_sign
        self.straight_left_trim = straight_left_trim
        self.straight_right_trim = straight_right_trim
        self._left_reference = 0.0
        self._right_reference = 0.0

        self.right_motor.reset_encoder()
        self.right_motor.set_limits(50)
        self.left_motor.reset_encoder()
        self.left_motor.set_limits(50)
        sleep(1)
        self.reset_drive_reference()

    def set_limits(self, power: int = 0, dps: int = 0):
        self.left_motor.set_limits(power, dps)
        self.right_motor.set_limits(power, dps)

    def move_straight(self, power: int):
        left_power, right_power = self._get_trimmed_straight_powers(power)
        self.left_motor.set_power(left_power * self.left_motor_sign)
        self.right_motor.set_power(right_power * self.right_motor_sign)

    def stop_move(self):
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)

    def wait_for_gyro_settle(
        self, timeout_s: float = 2.0, tolerance_deg: float = 2.0
    ) -> bool:
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for gyro-based motion")

        self.stop_move()
        self.gyro_sensor.set_reference()
        readings = []
        deadline = time() + timeout_s

        while time() < deadline:
            angle = self.gyro_sensor.get_angle()
            readings.append(angle)
            print("Gyro settle reading: %.2f deg" % angle)
            sleep(0.2)

        if not readings:
            return False

        drift = max(readings) - min(readings)
        print("Gyro settle drift: %.2f deg" % drift)
        self.gyro_sensor.set_reference()
        return drift <= tolerance_deg

    def reset_drive_reference(self):
        self._left_reference = self.left_motor.get_encoder() * self.left_motor_sign
        self._right_reference = self.right_motor.get_encoder() * self.right_motor_sign

    def get_average_encoder(self) -> float:
        left_delta = (self.left_motor.get_encoder() * self.left_motor_sign) - self._left_reference
        right_delta = (self.right_motor.get_encoder() * self.right_motor_sign) - self._right_reference
        return (left_delta + right_delta) / 2

    def get_turn_encoder_progress(self) -> float:
        left_delta = abs((self.left_motor.get_encoder() * self.left_motor_sign) - self._left_reference)
        right_delta = abs((self.right_motor.get_encoder() * self.right_motor_sign) - self._right_reference)
        return (left_delta + right_delta) / 2

    def get_distance_travelled_cm(self) -> float:
        wheel_circumference = math.pi * self.WHEEL_DIAMETER_CM
        return (self.get_average_encoder() / 360.0) * wheel_circumference

    def drive_distance_cm(self, distance_cm: float, power: float = 20):
        direction = 1 if distance_cm >= 0 else -1
        target_distance = abs(distance_cm)
        self.reset_drive_reference()
        left_power, right_power = self._get_trimmed_straight_powers(
            direction * abs(power)
        )
        self.adjust_speed(left_power, right_power)

        while abs(self.get_distance_travelled_cm()) < target_distance:
            sleep(0.01)

        self.stop_move()

    def drive_motor_degrees(self, motor_degrees: float, power: float = 20):
        direction = 1 if motor_degrees >= 0 else -1
        target_degrees = abs(motor_degrees)
        self.reset_drive_reference()
        left_power, right_power = self._get_trimmed_straight_powers(
            direction * abs(power)
        )
        self.adjust_speed(left_power, right_power)

        while abs(self.get_average_encoder()) < target_degrees:
            sleep(0.01)

        self.stop_move()

    def drive_motor_degrees_heading(self, motor_degrees: float, power: float = 20):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for heading-hold driving")

        direction = 1 if motor_degrees >= 0 else -1
        target_degrees = abs(motor_degrees)
        self.reset_drive_reference()
        self.start_heading_hold()

        while abs(self.get_average_encoder()) < target_degrees:
            self.adjust_heading_hold(direction * abs(power))
            sleep(0.01)

        self.stop_move()

    def start_heading_hold(self):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for heading-hold driving")
        self.gyro_sensor.set_reference()

    def adjust_heading_hold(self, base_power: float) -> float:
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for heading-hold driving")

        heading_error = self.gyro_sensor.get_angle()
        correction = self._clamp(
            heading_error * self.HEADING_GAIN,
            -self.MAX_HEADING_CORRECTION,
            self.MAX_HEADING_CORRECTION,
        )
        left_power, right_power = self._get_trimmed_straight_powers(base_power)
        self.adjust_speed(left_power - correction, right_power + correction)

        return correction

    def pivot_turn_motor_degrees(self, motor_degrees: float, power: float = 16):
        direction = 1 if motor_degrees >= 0 else -1
        target_degrees = abs(motor_degrees)
        self.reset_drive_reference()
        self.adjust_speed(direction * abs(power), -direction * abs(power))

        while self.get_turn_encoder_progress() < target_degrees:
            sleep(0.01)

        self.stop_move()

    def pivot_turn_gyro(
        self,
        angle_deg: float,
        power: float = 16,
        slow_power: Optional[float] = None,
        slowdown_deg: Optional[float] = None,
    ):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for gyro-based turns")

        direction = 1 if angle_deg >= 0 else -1
        target_angle = abs(angle_deg)
        slowdown = (
            self.TURN_SLOWDOWN_DEG if slowdown_deg is None else abs(slowdown_deg)
        )
        slow_turn_power = (
            self.MIN_TURN_POWER if slow_power is None else abs(slow_power)
        )

        self.gyro_sensor.set_reference()
        self.adjust_speed(direction * abs(power), -direction * abs(power))

        while abs(self.gyro_sensor.get_angle()) < max(0.0, target_angle - slowdown):
            sleep(0.01)

        self.adjust_speed(direction * slow_turn_power, -direction * slow_turn_power)

        while abs(self.gyro_sensor.get_angle()) < target_angle:
            sleep(0.01)

        self.stop_move()

    def pivot_turn(self, angle_deg: float, power: float = 16):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for gyro-based turns")
        direction = 1 if angle_deg >= 0 else -1
        target_angle = abs(angle_deg)
        self.gyro_sensor.set_reference()
        self.adjust_speed(direction * abs(power), -direction * abs(power))

        while abs(self.gyro_sensor.get_angle()) < max(
            0.0, target_angle - self.TURN_SLOWDOWN_DEG
        ):
            sleep(0.01)

        slow_power = min(abs(power), self.MIN_TURN_POWER)
        self.adjust_speed(direction * slow_power, -direction * slow_power)

        while abs(self.gyro_sensor.get_angle()) < target_angle:
            sleep(0.01)

        self.stop_move()

    def intersection_turn_right(self, deg: int):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for gyro-based turns")
        self.gyro_sensor.set_reference()
        self.adjust_speed(30, -8)
        while self.gyro_sensor.get_angle() < deg:
            sleep(0.01)
        self.adjust_speed(0, 0)

    def turn_with_angle(self, angle: float, base_power: float = 10):
        self.pivot_turn(angle, base_power)

    def turn_specific_with_angle(
        self, angle: float, left_power: float = 10, right_power: float = 10
    ):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for gyro-based turns")
        self.gyro_sensor.set_reference()
        self.adjust_speed(left_power, right_power)

        while abs(self.gyro_sensor.get_angle()) < abs(angle):
            sleep(0.01)
        self.adjust_speed(0, 0)

    def turn_specific_with_angle_without_refs(
        self, angle: float, left_power: float = 10, right_power: float = 10
    ):
        if self.gyro_sensor is None:
            raise ValueError("Gyro sensor is required for gyro-based turns")
        self.adjust_speed(left_power, right_power)

        while abs(self.gyro_sensor.get_angle()) < abs(angle):
            sleep(0.01)
        self.adjust_speed(0, 0)

    def adjust_left_speed(self, left_power: float):
        self.left_motor.set_power(left_power * self.left_motor_sign)

    def adjust_speed(self, left_power: float, right_power: float):
        self.left_motor.set_power(left_power * self.left_motor_sign)
        self.right_motor.set_power(right_power * self.right_motor_sign)

    def _get_trimmed_straight_powers(self, base_power: float):
        direction = 1 if base_power >= 0 else -1
        magnitude = abs(base_power)
        left_power = direction * (magnitude + self.straight_left_trim)
        right_power = direction * (magnitude + self.straight_right_trim)
        return left_power, right_power

    def _clamp(self, value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def change_relative_angle(self, angleLeft: float, angleRight: float):
        if angleLeft != 0:
            self.left_motor.set_position_relative(angleLeft * self.left_motor_sign)
        if angleRight != 0:
            self.right_motor.set_position_relative(angleRight * self.right_motor_sign)

    def is_robot_motor_moving(self) -> bool:
        return (self.left_motor.is_moving() or False) or (
            self.right_motor.is_moving() or False
        )

    def a_bit_forward(self):
        self.turn_specific_with_angle(30, -20, 20)
        sleep(0.1)
        self.turn_specific_with_angle(30, 20, 10)
        sleep(0.1)
