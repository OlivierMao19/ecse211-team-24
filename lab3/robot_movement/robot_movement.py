import math
from time import sleep

from gyro_sensor.gyro_sensor import GyroSensor
from utils.brick import Motor


class RobotMovement:
    left_motor: Motor
    right_motor: Motor
    gyro_sensor: GyroSensor

    BASE_R_POWER: int = 20
    BASE_L_POWER: int = 10
    WHEEL_DIAMETER_CM: float = 5.6

    def __init__(self, left_motor: Motor, right_motor: Motor, gyro_sensor: GyroSensor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyro_sensor = gyro_sensor
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
        self.left_motor.set_power(power)
        self.right_motor.set_power(power)

    def stop_move(self):
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)

    def reset_drive_reference(self):
        self._left_reference = self.left_motor.get_encoder()
        self._right_reference = self.right_motor.get_encoder()

    def get_average_encoder(self) -> float:
        left_delta = self.left_motor.get_encoder() - self._left_reference
        right_delta = self.right_motor.get_encoder() - self._right_reference
        return (left_delta + right_delta) / 2

    def get_distance_travelled_cm(self) -> float:
        wheel_circumference = math.pi * self.WHEEL_DIAMETER_CM
        return (self.get_average_encoder() / 360.0) * wheel_circumference

    def drive_distance_cm(self, distance_cm: float, power: float = 20):
        direction = 1 if distance_cm >= 0 else -1
        target_distance = abs(distance_cm)
        self.reset_drive_reference()
        self.adjust_speed(direction * abs(power), direction * abs(power))

        while abs(self.get_distance_travelled_cm()) < target_distance:
            sleep(0.01)

        self.stop_move()

    def pivot_turn(self, angle_deg: float, power: float = 16):
        direction = 1 if angle_deg >= 0 else -1
        self.gyro_sensor.set_reference()
        self.adjust_speed(direction * abs(power), -direction * abs(power))

        while abs(self.gyro_sensor.get_angle()) < abs(angle_deg):
            sleep(0.01)

        self.stop_move()

    def intersection_turn_right(self, deg: int):
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
        self.gyro_sensor.set_reference()
        self.adjust_speed(left_power, right_power)

        while abs(self.gyro_sensor.get_angle()) < abs(angle):
            sleep(0.01)
        self.adjust_speed(0, 0)

    def turn_specific_with_angle_without_refs(
        self, angle: float, left_power: float = 10, right_power: float = 10
    ):
        self.adjust_speed(left_power, right_power)

        while abs(self.gyro_sensor.get_angle()) < abs(angle):
            sleep(0.01)
        self.adjust_speed(0, 0)

    def adjust_left_speed(self, left_power: float):
        self.left_motor.set_power(left_power)

    def adjust_speed(self, left_power: float, right_power: float):
        self.left_motor.set_power(left_power)
        self.right_motor.set_power(right_power)

    def change_relative_angle(self, angleLeft: float, angleRight: float):
        if angleLeft != 0:
            self.left_motor.set_position_relative(angleLeft)
        if angleRight != 0:
            self.right_motor.set_position_relative(angleRight)

    def is_robot_motor_moving(self) -> bool:
        return (self.left_motor.is_moving() or False) or (
            self.right_motor.is_moving() or False
        )

    def a_bit_forward(self):
        self.turn_specific_with_angle(30, -20, 20)
        sleep(0.1)
        self.turn_specific_with_angle(30, 20, 10)
        sleep(0.1)
