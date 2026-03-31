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

from color_sensor.color_sensor import ColorSensor
from gyro_sensor.gyro_sensor import GyroSensor
from package_pickup.pickup_controller import PickupController
from package_pickup.pickup_sequence import run_pickup_sequence
from room_scan.room_scan import RoomScanner
from robot_movement.robot_movement import RobotMovement
from sound.robot_sound import RobotSound
from stop_button.stop_button import StopButton
from utils.brick import (
    EV3ColorSensor,
    EV3GyroSensor,
    Motor,
    TouchSensor,
    reset_brick,
    wait_ready_sensors,
)


LEFT_MOTOR_PORT = "B"
RIGHT_MOTOR_PORT = "C"
LEFT_MOTOR_SIGN = -1
RIGHT_MOTOR_SIGN = -1
LEFT_PICKUP_MOTOR_PORT = "A"
RIGHT_PICKUP_MOTOR_PORT = "D"
LEFT_PICKUP_SIGN = 1
RIGHT_PICKUP_SIGN = 1
PICKUP_POWER_LIMIT = 45
PICKUP_DPS_LIMIT = 260
GYRO_SENSOR_PORT = 3
COLOR_SENSOR_PORT = 2
STOP_SENSOR_PORT: Optional[int] = 1
DEBUG_SENSOR_INIT = True

DRIVE_POWER = 30
TURN_POWER = 14
ROOM_APPROACH_POWER = DRIVE_POWER
ROOM_SCAN_POWER = DRIVE_POWER
STRAIGHT_LEFT_TRIM = 0.0
STRAIGHT_RIGHT_TRIM = 0.0
DEGREE_UNIT = 700
HEADING_GAIN = 1.4
MAX_HEADING_CORRECTION = 6.0
TURN_SLOW_POWER = 10.0
TURN_SLOWDOWN_DEG = 16.0
GYRO_SETTLE_SECONDS = 2.0
GYRO_SETTLE_TOLERANCE_DEG = 2.0
ROOM_SCAN_PRINT_EVERY_DEGREES = 140
ROOM_COLOR_CONFIRM_SAMPLES = 2
ROOM_SCAN_MAX_MULTIPLIER = 1.5
ROOM_SWEEP_LEFT_ARC_DEGREES = 70
ROOM_SWEEP_RIGHT_ARC_DEGREES = 70
ROOM_SWEEP_STEP_DEGREES = 180
ROOM_SWEEP_OUTER_POWER = 24
ROOM_SWEEP_INNER_POWER = 18
ROOM_SWEEP_LEFT_TRIM = 0.0
ROOM_SWEEP_RIGHT_TRIM = 0.0
ROOM_SWEEP_LEFT_RETURN_SCALE = 1.0
ROOM_SWEEP_RIGHT_RETURN_SCALE = 1.0
ROOM_ENTRY_PAUSE_S = 0.8
ROOM_SWEEP_STEP_PAUSE_S = 0.8
ROOM_REALIGN_TO_HEADING = True
ROOM_HEADING_EXTRA_CORRECTION_DEG = 0.0
ROOM_HEADING_TOLERANCE_DEG = 1.0
ROOM_HEADING_TURN_POWER = 8.0
ROOM_EXIT_EXTRA_DEGREES = 100
ROOM_DROPOFF_LEFT_ROTATE_DEGREES = -180
ROOM_DROPOFF_RIGHT_ROTATE_DEGREES = -180
ROOM_DROPOFF_DETECT_PAUSE_S = 0.8
ROOM_DROPOFF_APPROACH_DEGREES = 180
ROOM_DROPOFF_SHIFT_DEGREES = 180
ROOM_DROPOFF_SHIFT_OUTER_POWER = 24
ROOM_DROPOFF_SHIFT_INNER_POWER = 12
ROOM_DROPOFF_PAUSE_S = 3.0
EXTRA_ROOM_LINK_MULTIPLIER = 360.0 / DEGREE_UNIT
FINAL_RETURN_FORWARD_MULTIPLIER = 1.3
FINAL_RETURN_MAIN_MULTIPLIER = 2.0 + EXTRA_ROOM_LINK_MULTIPLIER

# "drive" uses DEGREE_UNIT * value.
# Positive = forward, negative = backward.
# "turn" uses 90 degrees * value.
# Positive = right, negative = left.
# "room" drives in until yellow, scans until green/red, then backs up from yellow.
MISSION_STEPS: List[Tuple[str, float]] = [
    ("room", 3),
    ("turn", 1),
    ("drive", 1.9),
    ("turn", -1),
    ("room", 1),
    ("turn", 1),
    ("drive", 0.9),
    ("turn", 1),
    ("room", 1),
    ("turn", -1),
    ("drive", EXTRA_ROOM_LINK_MULTIPLIER),
    ("turn", 1),
    ("room", 1),
    ("turn", 1),
    ("drive", FINAL_RETURN_MAIN_MULTIPLIER),
    ("turn", -1),
    ("drive", FINAL_RETURN_FORWARD_MULTIPLIER),
]


def build_robot() -> Tuple[
    RobotMovement,
    ColorSensor,
    PickupController,
    Optional[StopButton],
]:
    left_motor = Motor(LEFT_MOTOR_PORT)
    right_motor = Motor(RIGHT_MOTOR_PORT)
    left_pickup_motor = Motor(LEFT_PICKUP_MOTOR_PORT)
    right_pickup_motor = Motor(RIGHT_PICKUP_MOTOR_PORT)
    base_color_sensor = EV3ColorSensor(COLOR_SENSOR_PORT)
    base_gyro_sensor = EV3GyroSensor(GYRO_SENSOR_PORT)
    stop_button_sensor = TouchSensor(STOP_SENSOR_PORT) if STOP_SENSOR_PORT else None

    wait_ready_sensors(DEBUG_SENSOR_INIT)

    color_sensor = ColorSensor(base_color_sensor)
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
    return robot_movement, color_sensor, pickup_controller, stop_button


def run_mission(robot_movement: RobotMovement, room_scanner: RoomScanner):
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

        if action == "room":
            print("Step %d: enter room and scan for bed color" % index)
            room_scanner.scan_room(DEGREE_UNIT * value)
            continue

        if action == "turn":
            direction = "right" if value > 0 else "left"
            print(
                "Step %d: turn %s for %.0f degrees"
                % (index, direction, abs(90 * value))
            )
            robot_movement.pivot_turn_gyro(85 * value, TURN_POWER)
            continue

        raise ValueError("Unsupported action: %s" % action)


def main():
    robot_movement = None
    color_sensor = None
    pickup_controller = None
    room_scanner = None
    robot_sound = None
    stop_button = None

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
            room_entry_pause_s=ROOM_ENTRY_PAUSE_S,
            step_pause_s=ROOM_SWEEP_STEP_PAUSE_S,
            realign_to_room_heading=ROOM_REALIGN_TO_HEADING,
            room_heading_extra_correction_deg=ROOM_HEADING_EXTRA_CORRECTION_DEG,
            room_heading_tolerance_deg=ROOM_HEADING_TOLERANCE_DEG,
            room_heading_turn_power=ROOM_HEADING_TURN_POWER,
            room_exit_extra_degrees=ROOM_EXIT_EXTRA_DEGREES,
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
        print("Keep the robot still while the gyro settles")
        if not robot_movement.wait_for_gyro_settle(
            GYRO_SETTLE_SECONDS,
            GYRO_SETTLE_TOLERANCE_DEG,
        ):
            raise RuntimeError("Gyro did not settle. Restart with robot kept still.")
        print("Starting pickup sequence")
        run_pickup_sequence(robot_movement, pickup_controller)
        print("Starting hard-coded mission")
        run_mission(robot_movement, room_scanner)
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
        if pickup_controller is not None:
            pickup_controller.stop()
        if color_sensor is not None:
            color_sensor.dispose()
        if stop_button is not None:
            stop_button.dispose()
        reset_brick()


if __name__ == "__main__":
    main()
