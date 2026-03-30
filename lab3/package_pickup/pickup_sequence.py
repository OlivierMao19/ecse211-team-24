#!/usr/bin/env python3

from pathlib import Path
import sys
from time import sleep
from typing import Optional, Tuple


ROOT_DIR = Path(__file__).resolve().parents[3]
LAB3_DIR = ROOT_DIR / "lab3"
PROJECT_DIR = ROOT_DIR / "lab2-starter-code-team-24" / "project"

for path in (str(LAB3_DIR), str(PROJECT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from gyro_sensor.gyro_sensor import GyroSensor
from package_pickup.pickup_controller import PickupController
from robot_movement.robot_movement import RobotMovement
from stop_button.stop_button import StopButton
from utils.brick import (
    EV3GyroSensor,
    Motor,
    TouchSensor,
    reset_brick,
    wait_ready_sensors,
)


LEFT_DRIVE_MOTOR_PORT = "B"
RIGHT_DRIVE_MOTOR_PORT = "C"
LEFT_DRIVE_SIGN = -1
RIGHT_DRIVE_SIGN = -1

LEFT_PICKUP_MOTOR_PORT = "A"
RIGHT_PICKUP_MOTOR_PORT = "D"
LEFT_PICKUP_SIGN = 1
RIGHT_PICKUP_SIGN = 1
PICKUP_POWER_LIMIT = 45
PICKUP_DPS_LIMIT = 260

GYRO_SENSOR_PORT = 3
STOP_SENSOR_PORT: Optional[int] = 1
DEBUG_SENSOR_INIT = True

DRIVE_POWER = 30
TURN_POWER = 14
STRAIGHT_LEFT_TRIM = 0.0
STRAIGHT_RIGHT_TRIM = 3.0
HEADING_GAIN = 1.4
MAX_HEADING_CORRECTION = 6.0
TURN_SLOW_POWER = 10.0
TURN_SLOWDOWN_DEG = 16.0
GYRO_SETTLE_SECONDS = 2.0
GYRO_SETTLE_TOLERANCE_DEG = 2.0

FORWARD_DEGREES = 630
BACKWARD_DEGREES = -400
PICKUP_ROTATE_DEGREES = 135 #Change this if we want the cube to be stuck more
PICKUP_SETTLE_SECONDS = 0.6
EXIT_TURN_DEG = -89


def build_system() -> Tuple[RobotMovement, PickupController, Optional[StopButton]]:
    left_drive_motor = Motor(LEFT_DRIVE_MOTOR_PORT)
    right_drive_motor = Motor(RIGHT_DRIVE_MOTOR_PORT)
    left_pickup_motor = Motor(LEFT_PICKUP_MOTOR_PORT)
    right_pickup_motor = Motor(RIGHT_PICKUP_MOTOR_PORT)
    base_gyro_sensor = EV3GyroSensor(GYRO_SENSOR_PORT)
    stop_button_sensor = TouchSensor(STOP_SENSOR_PORT) if STOP_SENSOR_PORT else None

    wait_ready_sensors(DEBUG_SENSOR_INIT)

    gyro_sensor = GyroSensor(base_gyro_sensor)
    robot_movement = RobotMovement(
        left_drive_motor,
        right_drive_motor,
        gyro_sensor=gyro_sensor,
        left_motor_sign=LEFT_DRIVE_SIGN,
        right_motor_sign=RIGHT_DRIVE_SIGN,
        straight_left_trim=STRAIGHT_LEFT_TRIM,
        straight_right_trim=STRAIGHT_RIGHT_TRIM,
    )
    robot_movement.HEADING_GAIN = HEADING_GAIN
    robot_movement.MAX_HEADING_CORRECTION = MAX_HEADING_CORRECTION
    robot_movement.TURN_SLOWDOWN_DEG = TURN_SLOWDOWN_DEG
    robot_movement.MIN_TURN_POWER = TURN_SLOW_POWER

    pickup_controller = PickupController(
        left_pickup_motor,
        right_pickup_motor,
        left_sign=LEFT_PICKUP_SIGN,
        right_sign=RIGHT_PICKUP_SIGN,
        power_limit=PICKUP_POWER_LIMIT,
        dps_limit=PICKUP_DPS_LIMIT,
    )

    stop_button = (
        StopButton(stop_button_sensor, on_press=robot_movement.stop_move)
        if stop_button_sensor
        else None
    )
    return robot_movement, pickup_controller, stop_button


def run_pickup_sequence(
    robot_movement: RobotMovement,
    pickup_controller: PickupController,
):
    print("Pickup step 1: drive forward to cubes")
    robot_movement.drive_motor_degrees_heading(FORWARD_DEGREES, DRIVE_POWER)

    print("Pickup step 2: close scoops")
    pickup_controller.rotate_relative(PICKUP_ROTATE_DEGREES)
    print(
        "Pickup step 2b: waiting %.1f s for pickup to settle"
        % PICKUP_SETTLE_SECONDS
    )
    sleep(PICKUP_SETTLE_SECONDS)
    
    print("Pickup step 3: back away from cubes")
    robot_movement.drive_motor_degrees_heading(BACKWARD_DEGREES, DRIVE_POWER)

    print("Pickup step 4: turn left to navigation start heading")
    robot_movement.pivot_turn_gyro(EXIT_TURN_DEG, TURN_POWER)

    print("Pickup sequence finished")


def main():
    robot_movement = None
    pickup_controller = None
    stop_button = None

    try:
        robot_movement, pickup_controller, stop_button = build_system()
        print("Keep the robot still while the gyro settles")
        if not robot_movement.wait_for_gyro_settle(
            GYRO_SETTLE_SECONDS,
            GYRO_SETTLE_TOLERANCE_DEG,
        ):
            raise RuntimeError("Gyro did not settle. Restart with robot kept still.")

        run_pickup_sequence(robot_movement, pickup_controller)
    except KeyboardInterrupt:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        print("Pickup sequence stopped")
    except Exception as exc:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        print("Pickup sequence failed:", exc)
    finally:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        if stop_button is not None:
            stop_button.dispose()
        reset_brick()


if __name__ == "__main__":
    main()
