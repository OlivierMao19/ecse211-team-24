#!/usr/bin/env python3

from run_navigation import (
    DEGREE_UNIT,
    GYRO_SETTLE_SECONDS,
    GYRO_SETTLE_TOLERANCE_DEG,
    ROOM_APPROACH_POWER,
    ROOM_COLOR_CONFIRM_SAMPLES,
    ROOM_DROPOFF_APPROACH_DEGREES,
    ROOM_DROPOFF_DETECT_PAUSE_S,
    ROOM_DROPOFF_LEFT_ROTATE_DEGREES,
    ROOM_DROPOFF_PAUSE_S,
    ROOM_DROPOFF_RIGHT_ROTATE_DEGREES,
    ROOM_DROPOFF_SHIFT_DEGREES,
    ROOM_DROPOFF_SHIFT_INNER_POWER,
    ROOM_DROPOFF_SHIFT_OUTER_POWER,
    ROOM_SCAN_MAX_MULTIPLIER,
    ROOM_SCAN_POWER,
    ROOM_SCAN_PRINT_EVERY_DEGREES,
    ROOM_HEADING_EXTRA_CORRECTION_DEG,
    ROOM_HEADING_TOLERANCE_DEG,
    ROOM_HEADING_TURN_POWER,
    ROOM_REALIGN_TO_HEADING,
    ROOM_SWEEP_INNER_POWER,
    ROOM_SWEEP_LEFT_ARC_DEGREES,
    ROOM_SWEEP_LEFT_RETURN_SCALE,
    ROOM_SWEEP_LEFT_TRIM,
    ROOM_SWEEP_OUTER_POWER,
    ROOM_SWEEP_RIGHT_ARC_DEGREES,
    ROOM_SWEEP_RIGHT_RETURN_SCALE,
    ROOM_SWEEP_RIGHT_TRIM,
    ROOM_SWEEP_STEP_DEGREES,
    ROOM_SWEEP_STEP_PAUSE_S,
    RobotSound,
    build_robot,
)
from room_scan.room_scan import RoomScanner
from utils.brick import reset_brick


ROOM_TEST_ENTRY_MULTIPLIER = 1.0


def build_room_scanner():
    robot_movement = None
    color_sensor = None
    pickup_controller = None
    stop_button = None
    robot_sound = None

    try:
        robot_movement, color_sensor, pickup_controller, stop_button = build_robot()
        robot_sound = RobotSound()
        room_scanner = RoomScanner(
            robot_movement=robot_movement,
            color_sensor=color_sensor,
            room_approach_power=ROOM_APPROACH_POWER,
            room_scan_power=ROOM_SCAN_POWER,
            room_color_confirm_samples=ROOM_COLOR_CONFIRM_SAMPLES,
            room_scan_print_every_degrees=ROOM_SCAN_PRINT_EVERY_DEGREES,
            room_scan_max_degrees=DEGREE_UNIT * ROOM_SCAN_MAX_MULTIPLIER,
            sweep_left_arc_degrees=ROOM_SWEEP_LEFT_ARC_DEGREES,
            sweep_right_arc_degrees=ROOM_SWEEP_RIGHT_ARC_DEGREES,
            sweep_step_degrees=ROOM_SWEEP_STEP_DEGREES,
            sweep_outer_power=ROOM_SWEEP_OUTER_POWER,
            sweep_inner_power=ROOM_SWEEP_INNER_POWER,
            sweep_left_trim=ROOM_SWEEP_LEFT_TRIM,
            sweep_right_trim=ROOM_SWEEP_RIGHT_TRIM,
            sweep_left_return_scale=ROOM_SWEEP_LEFT_RETURN_SCALE,
            sweep_right_return_scale=ROOM_SWEEP_RIGHT_RETURN_SCALE,
            step_pause_s=ROOM_SWEEP_STEP_PAUSE_S,
            realign_to_room_heading=ROOM_REALIGN_TO_HEADING,
            room_heading_extra_correction_deg=ROOM_HEADING_EXTRA_CORRECTION_DEG,
            room_heading_tolerance_deg=ROOM_HEADING_TOLERANCE_DEG,
            room_heading_turn_power=ROOM_HEADING_TURN_POWER,
            robot_sound=robot_sound,
            pickup_controller=pickup_controller,
            dropoff_left_rotate_degrees=ROOM_DROPOFF_LEFT_ROTATE_DEGREES,
            dropoff_right_rotate_degrees=ROOM_DROPOFF_RIGHT_ROTATE_DEGREES,
            dropoff_detect_pause_s=ROOM_DROPOFF_DETECT_PAUSE_S,
            dropoff_approach_degrees=ROOM_DROPOFF_APPROACH_DEGREES,
            dropoff_shift_degrees=ROOM_DROPOFF_SHIFT_DEGREES,
            dropoff_shift_outer_power=ROOM_DROPOFF_SHIFT_OUTER_POWER,
            dropoff_shift_inner_power=ROOM_DROPOFF_SHIFT_INNER_POWER,
            dropoff_pause_s=ROOM_DROPOFF_PAUSE_S,
        )
        return (
            robot_movement,
            color_sensor,
            pickup_controller,
            stop_button,
            room_scanner,
        )
    except Exception:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        if color_sensor is not None:
            color_sensor.dispose()
        if stop_button is not None:
            stop_button.dispose()
        reset_brick()
        raise


def main():
    robot_movement = None
    color_sensor = None
    pickup_controller = None
    stop_button = None

    try:
        (
            robot_movement,
            color_sensor,
            pickup_controller,
            stop_button,
            room_scanner,
        ) = build_room_scanner()
        print("Keep the robot still while the gyro settles")
        if not robot_movement.wait_for_gyro_settle(
            GYRO_SETTLE_SECONDS,
            GYRO_SETTLE_TOLERANCE_DEG,
        ):
            raise RuntimeError("Gyro did not settle. Restart with robot kept still.")

        print(
            "Starting room-only scan test with room entry limit %.0f motor degrees"
            % (DEGREE_UNIT * ROOM_TEST_ENTRY_MULTIPLIER)
        )
        room_scanner.scan_room(DEGREE_UNIT * ROOM_TEST_ENTRY_MULTIPLIER)
        print("Room-only scan test finished")
    except KeyboardInterrupt:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        print("Room-only scan test stopped")
    except Exception as exc:
        if robot_movement is not None:
            robot_movement.stop_move()
        if pickup_controller is not None:
            pickup_controller.stop()
        print("Room-only scan test failed:", exc)
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
