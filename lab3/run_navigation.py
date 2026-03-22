#!/usr/bin/env python3

from pathlib import Path
import sys
from typing import List, Optional, Tuple


ROOT_DIR = Path(__file__).resolve().parents[1]
LAB3_DIR = ROOT_DIR / "lab3"
PROJECT_DIR = ROOT_DIR / "lab2-starter-code-team-24" / "project"

for path in (str(LAB3_DIR), str(PROJECT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from gyro_sensor.gyro_sensor import GyroSensor
from robot_movement.robot_movement import RobotMovement
from stop_button.stop_button import StopButton
from utils.brick import (
    EV3GyroSensor,
    Motor,
    TouchSensor,
    reset_brick,
    wait_ready_sensors,
)


LEFT_MOTOR_PORT = "D"
RIGHT_MOTOR_PORT = "C"
LEFT_MOTOR_SIGN = -1
RIGHT_MOTOR_SIGN = -1
GYRO_SENSOR_PORT = 1
STOP_SENSOR_PORT: Optional[int] = 3
DEBUG_SENSOR_INIT = True

DRIVE_POWER = 22
TURN_POWER = 18
STRAIGHT_LEFT_TRIM = 0.0
STRAIGHT_RIGHT_TRIM = 0.0
DEGREE_UNIT = 720
HEADING_GAIN = 1.2
MAX_HEADING_CORRECTION = 6.0
TURN_SLOW_POWER = 7.0
TURN_SLOWDOWN_DEG = 20.0
GYRO_SETTLE_SECONDS = 2.0
GYRO_SETTLE_TOLERANCE_DEG = 2.0

# "drive" uses DEGREE_UNIT * value.
# Positive = forward, negative = backward.
# "turn" uses TURN_90_MOTOR_DEGREES * value.
# Positive = right, negative = left.
MISSION_STEPS: List[Tuple[str, float]] = [
    ("drive", 3),
    ("drive", -1),
    ("turn", 1),
    ("drive", 1.85),
    ("turn", -1),
    ("drive", 1),
    ("drive", -1),
    ("turn", 1),
    ("drive", 0.9),
    ("turn", 1),
    ("drive", 1),
    ("drive", -1),
    ("turn", 1),
    ("drive", 2),
    ("turn", -1),
    ("drive", 1),
]


def build_robot() -> Tuple[RobotMovement, Optional[StopButton]]:
    left_motor = Motor(LEFT_MOTOR_PORT)
    right_motor = Motor(RIGHT_MOTOR_PORT)
    base_gyro_sensor = EV3GyroSensor(GYRO_SENSOR_PORT)
    stop_button_sensor = TouchSensor(STOP_SENSOR_PORT) if STOP_SENSOR_PORT else None

    wait_ready_sensors(DEBUG_SENSOR_INIT)

    gyro_sensor = GyroSensor(base_gyro_sensor)
    robot_movement = RobotMovement(
        left_motor,
        right_motor,
        gyro_sensor=gyro_sensor,
        left_motor_sign=LEFT_MOTOR_SIGN,
        right_motor_sign=RIGHT_MOTOR_SIGN,
        straight_left_trim=STRAIGHT_LEFT_TRIM,
        straight_right_trim=STRAIGHT_RIGHT_TRIM,
    )
    robot_movement.HEADING_GAIN = HEADING_GAIN
    robot_movement.MAX_HEADING_CORRECTION = MAX_HEADING_CORRECTION
    robot_movement.TURN_SLOWDOWN_DEG = TURN_SLOWDOWN_DEG
    robot_movement.MIN_TURN_POWER = TURN_SLOW_POWER
    stop_button = (
        StopButton(stop_button_sensor, on_press=robot_movement.stop_move)
        if stop_button_sensor
        else None
    )
    return robot_movement, stop_button


def run_mission(robot_movement: RobotMovement):
    for index, (action, value) in enumerate(MISSION_STEPS, start=1):
        if action == "drive":
            motor_degrees = DEGREE_UNIT * value
            direction = "forward" if value > 0 else "backward"
            print(
                "Step %d: drive %s for %.0f motor degrees"
                % (index, direction, abs(motor_degrees))
            )
            robot_movement.drive_motor_degrees_heading(motor_degrees, DRIVE_POWER)
            continue

        if action == "turn":
            direction = "right" if value > 0 else "left"
            print(
                "Step %d: turn %s for %.0f degrees"
                % (index, direction, abs(90 * value))
            )
            robot_movement.pivot_turn_gyro(90 * value, TURN_POWER)
            continue

        raise ValueError("Unsupported action: %s" % action)


def main():
    robot_movement = None
    stop_button = None

    try:
        robot_movement, stop_button = build_robot()
        print("Keep the robot still while the gyro settles")
        if not robot_movement.wait_for_gyro_settle(
            GYRO_SETTLE_SECONDS,
            GYRO_SETTLE_TOLERANCE_DEG,
        ):
            raise RuntimeError("Gyro did not settle. Restart with robot kept still.")
        print("Starting hard-coded mission")
        run_mission(robot_movement)
        print("Mission finished")
    except KeyboardInterrupt:
        if robot_movement is not None:
            robot_movement.stop_move()
        print("Navigation stopped")
    except Exception as exc:
        if robot_movement is not None:
            robot_movement.stop_move()
        print("Navigation failed:", exc)
    finally:
        if robot_movement is not None:
            robot_movement.stop_move()
        if stop_button is not None:
            stop_button.dispose()
        reset_brick()


if __name__ == "__main__":
    main()
