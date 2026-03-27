from time import sleep

from utils.brick import Motor


class PickupController:
    def __init__(
        self,
        left_motor: Motor,
        right_motor: Motor,
        left_sign: int = 1,
        right_sign: int = 1,
        power_limit: int = 60,
        dps_limit: int = 400,
    ):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_sign = left_sign
        self.right_sign = right_sign

        self.left_motor.set_limits(power=power_limit, dps=dps_limit)
        self.right_motor.set_limits(power=power_limit, dps=dps_limit)

    def rotate_relative(self, degrees: float):
        if degrees == 0:
            return

        self.left_motor.set_position_relative(degrees * self.left_sign)
        self.right_motor.set_position_relative(degrees * self.right_sign)

        while self.left_motor.is_moving() or self.right_motor.is_moving():
            sleep(0.01)

    def rotate_left_relative(self, degrees: float):
        if degrees == 0:
            return

        self.left_motor.set_position_relative(degrees * self.left_sign)
        while self.left_motor.is_moving():
            sleep(0.01)

    def rotate_right_relative(self, degrees: float):
        if degrees == 0:
            return

        self.right_motor.set_position_relative(degrees * self.right_sign)
        while self.right_motor.is_moving():
            sleep(0.01)

    def stop(self):
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)
