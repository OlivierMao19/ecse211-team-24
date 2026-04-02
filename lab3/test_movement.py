#!/usr/bin/env python3

from time import sleep

from run_navigation import (
    DRIVE_POWER,
    GYRO_SETTLE_SECONDS,
    GYRO_SETTLE_TOLERANCE_DEG,
    TURN_POWER,
    build_robot,
)
from utils.brick import reset_brick


TEST_DRIVE_DEGREES = 700
TEST_TURN_DEG = 88
TEST_SECOND_DRIVE_DEGREES = 700
STEP_PAUSE_S = 0.8


def main():
    robot_movement = None
    color_sensor = None
    pickup_controller = None
    stop_button = None

    try:
        robot_movement, color_sensor, pickup_controller, stop_button = build_robot()
        print("Keep the robot still while the gyro settles")
        if not robot_movement.wait_for_gyro_settle(
            GYRO_SETTLE_SECONDS,
            GYRO_SETTLE_TOLERANCE_DEG,
        ):
            raise RuntimeError("Gyro did not settle. Restart with robot kept still.")

        print("Test step 1: drive forward %.0f motor degrees" % TEST_DRIVE_DEGREES)
        robot_movement.drive_motor_degrees_heading(TEST_DRIVE_DEGREES, DRIVE_POWER)
        sleep(STEP_PAUSE_S)

        print("Test step 2: turn %.0f degrees" % TEST_TURN_DEG)
        robot_movement.pivot_turn_gyro(TEST_TURN_DEG, TURN_POWER)
        sleep(STEP_PAUSE_S)

        print(
            "Test step 3: drive forward %.0f motor degrees"
            % TEST_SECOND_DRIVE_DEGREES
        )
        robot_movement.drive_motor_degrees_heading(
            TEST_SECOND_DRIVE_DEGREES,
            DRIVE_POWER,
        )
        print("Movement test finished")
    except KeyboardInterrupt:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        print("Movement test stopped")
    except Exception as exc:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        print("Movement test failed:", exc)
    finally:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        if color_sensor is not None:
            color_sensor.dispose()
        if stop_button is not None:
            stop_button.dispose()
        reset_brick()


if __name__ == "__main__":
    main()
