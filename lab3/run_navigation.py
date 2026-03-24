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
from robot_movement.robot_movement import RobotMovement
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
GYRO_SENSOR_PORT = 1
COLOR_SENSOR_PORT = 4
STOP_SENSOR_PORT: Optional[int] = 3
DEBUG_SENSOR_INIT = True

DRIVE_POWER = 30
TURN_POWER = 18
ROOM_APPROACH_POWER = 18
ROOM_SCAN_POWER = 14
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

# "drive" uses DEGREE_UNIT * value.
# Positive = forward, negative = backward.
# "turn" uses 90 degrees * value.
# Positive = right, negative = left.
# "room" drives in until yellow, scans until green/red, then backs up from yellow.
MISSION_STEPS: List[Tuple[str, float]] = [
    ("room", 3),
    ("turn", 1),
    ("drive", 1.85),
    ("turn", -1),
    ("room", 1),
    ("turn", 1),
    ("drive", 0.9),
    ("turn", 1),
    ("room", 1),
    ("turn", 1),
    ("drive", 2),
    ("turn", -1),
    ("drive", 1.3),
]


def build_robot() -> Tuple[RobotMovement, ColorSensor, Optional[StopButton]]:
    left_motor = Motor(LEFT_MOTOR_PORT)
    right_motor = Motor(RIGHT_MOTOR_PORT)
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
    stop_button = (
        StopButton(stop_button_sensor, on_press=robot_movement.stop_move)
        if stop_button_sensor
        else None
    )
    return robot_movement, color_sensor, stop_button


def wait_for_color(
    color_sensor: ColorSensor,
    target_colors: Tuple[str, ...],
    consecutive_samples: int,
    current_streak_color: Optional[str],
    current_streak_count: int,
) -> Tuple[bool, Optional[str], int]:
    current_color = color_sensor.get_current_color()
    if current_color == current_streak_color:
        current_streak_count += 1
    else:
        current_streak_color = current_color
        current_streak_count = 1

    matched = (
        current_streak_color in target_colors
        and current_streak_count >= consecutive_samples
    )
    return matched, current_streak_color, current_streak_count


def print_color_measurement(prefix: str, color_sensor: ColorSensor):
    rgb = color_sensor.get_current_rgb()
    color = color_sensor.get_current_color()
    print(
        "%s rgb=(%.1f, %.1f, %.1f) detected=%s"
        % (prefix, rgb[0], rgb[1], rgb[2], color)
    )


def scan_room(
    robot_movement: RobotMovement,
    color_sensor: ColorSensor,
    value: float,
):
    max_room_entry_degrees = DEGREE_UNIT * value
    print(
        "Room approach: driving until yellow for up to %.0f motor degrees"
        % max_room_entry_degrees
    )

    robot_movement.reset_drive_reference()
    robot_movement.start_heading_hold()

    phase = "search_yellow"
    streak_color = None
    streak_count = 0
    detected_bed_color = None
    last_reported_bucket = -1
    max_scan_degrees = DEGREE_UNIT * ROOM_SCAN_MAX_MULTIPLIER

    while True:
        if phase == "search_yellow":
            if abs(robot_movement.get_average_encoder()) >= max_room_entry_degrees:
                break

            robot_movement.adjust_heading_hold(ROOM_APPROACH_POWER)
            print_color_measurement("Room approach:", color_sensor)
            found_yellow, streak_color, streak_count = wait_for_color(
                color_sensor,
                ("YELLOW",),
                ROOM_COLOR_CONFIRM_SAMPLES,
                streak_color,
                streak_count,
            )
            if found_yellow:
                yellow_position = abs(robot_movement.get_average_encoder())
                print(
                    "Room approach: detected YELLOW at %.0f motor degrees"
                    % yellow_position
                )
                robot_movement.reset_drive_reference()
                robot_movement.start_heading_hold()
                streak_color = None
                streak_count = 0
                phase = "search_bed"
            continue

        if abs(robot_movement.get_average_encoder()) >= max_scan_degrees:
            break

        robot_movement.adjust_heading_hold(ROOM_SCAN_POWER)
        travelled_since_yellow = abs(robot_movement.get_average_encoder())
        print_color_measurement("Room scan:", color_sensor)
        report_bucket = int(travelled_since_yellow / ROOM_SCAN_PRINT_EVERY_DEGREES)
        if report_bucket > last_reported_bucket:
            print(
                "Room scan: %.0f degrees since yellow, color=%s"
                % (travelled_since_yellow, color_sensor.get_current_color())
            )
            last_reported_bucket = report_bucket

        found_bed_color, streak_color, streak_count = wait_for_color(
            color_sensor,
            ("GREEN", "RED"),
            ROOM_COLOR_CONFIRM_SAMPLES,
            streak_color,
            streak_count,
        )
        if found_bed_color:
            detected_bed_color = streak_color
            break

    robot_movement.stop_move()

    if phase == "search_yellow":
        print("Room approach: yellow not detected, continuing mission")
        return

    travelled_since_yellow = abs(robot_movement.get_average_encoder())

    if detected_bed_color is None:
        print(
            "Room scan: no GREEN or RED found within %.0f motor degrees"
            % max_scan_degrees
        )
    else:
        print(
            "Room scan: detected %s after %.0f motor degrees"
            % (detected_bed_color, travelled_since_yellow)
        )

    if travelled_since_yellow > 0:
        print(
            "Room scan: backing up %.0f motor degrees to return from yellow scan"
            % travelled_since_yellow
        )
        robot_movement.drive_motor_degrees_heading(
            -travelled_since_yellow,
            ROOM_APPROACH_POWER,
        )


def run_mission(robot_movement: RobotMovement, color_sensor: ColorSensor):
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
            scan_room(robot_movement, color_sensor, value)
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
    color_sensor = None
    stop_button = None

    try:
        robot_movement, color_sensor, stop_button = build_robot()
        print("Keep the robot still while the gyro settles")
        if not robot_movement.wait_for_gyro_settle(
            GYRO_SETTLE_SECONDS,
            GYRO_SETTLE_TOLERANCE_DEG,
        ):
            raise RuntimeError("Gyro did not settle. Restart with robot kept still.")
        print("Starting hard-coded mission")
        run_mission(robot_movement, color_sensor)
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
        if color_sensor is not None:
            color_sensor.dispose()
        if stop_button is not None:
            stop_button.dispose()
        reset_brick()


if __name__ == "__main__":
    main()
