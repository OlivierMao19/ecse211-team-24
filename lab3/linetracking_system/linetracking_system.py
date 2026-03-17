from __future__ import annotations

from dataclasses import dataclass
from time import monotonic, sleep
from typing import Any, Callable, Optional

from color_sensor.color_sensor import ColorSensor
from gyro_sensor.gyro_sensor import GyroSensor
from robot_movement.robot_movement import RobotMovement
from stop_button.stop_button import StopButton


@dataclass(slots=True)
class RouteStep:
    name: str
    stop_color: Optional[str] = None
    min_distance_cm: float = 0.0
    branch_turn_deg: float = 0.0
    room_side: Optional[str] = None
    on_arrival: Optional[Callable[[], None]] = None
    approach_distance_cm: float = 5.0
    room_entry_distance_cm: float = 16.0
    room_exit_distance_cm: float = 16.0
    pause_seconds: float = 0.5


class LineTracker:
    BASE_SPEED: float = 20.0
    MAX_CORRECTION: float = 10.0
    CORRECTION_GAIN: float = 28.0
    EDGE_TARGET: float = 0.45
    STEERING_SIGN: float = 1.0

    TURN_POWER: float = 16.0
    ROOM_DRIVE_POWER: float = 18.0
    REACQUIRE_POWER: float = 12.0

    MARKER_CONFIRM_SAMPLES: int = 3
    LINE_REACQUIRE_TIMEOUT_S: float = 1.75
    LINE_FOLLOW_SLEEP_S: float = 0.01

    PHARMACY_EXIT_DISTANCE_CM: float = 18.0
    PHARMACY_DOCK_DISTANCE_CM: float = 12.0
    REACQUIRE_FORWARD_DISTANCE_CM: float = 5.0

    stop_button: Optional[StopButton]
    color_sensor: ColorSensor
    robot_movement: RobotMovement
    gyro: GyroSensor

    def __init__(
        self,
        robot_movement: RobotMovement,
        color_sensor: ColorSensor,
        gyro: GyroSensor,
        zone_detection: Optional[Any] = None,
        stop_button: Optional[StopButton] = None,
        sound_engine: Optional[Any] = None,
    ):
        self.stop_button = stop_button
        self.robot_movement = robot_movement
        self.color_sensor = color_sensor
        self.gyro = gyro
        self.zone_detection = zone_detection
        self.sound_engine = sound_engine
        self.route_steps = self.build_default_route()

    def build_default_route(self) -> list[RouteStep]:
        # Positive angles are right pivots, negative angles are left pivots.
        return [
            RouteStep(
                name="ROOM_1_DOOR",
                stop_color="ORANGE",
                min_distance_cm=24.0,
                branch_turn_deg=-90.0,
                room_side="left",
            ),
            RouteStep(
                name="ROOM_2_DOOR",
                stop_color="ORANGE",
                min_distance_cm=26.0,
                branch_turn_deg=-90.0,
                room_side="left",
            ),
            RouteStep(
                name="ROOM_3_DOOR",
                stop_color="ORANGE",
                min_distance_cm=42.0,
                branch_turn_deg=90.0,
                room_side="right",
                room_entry_distance_cm=20.0,
                room_exit_distance_cm=20.0,
            ),
            RouteStep(
                name="PHARMACY_RETURN",
                stop_color="BLUE",
                min_distance_cm=30.0,
            ),
        ]

    def follow_line(self):
        self.run_route(self.route_steps)

    def run_route(self, route: list[RouteStep]):
        self.leave_pharmacy()

        for step in route:
            self.follow_line_until(step)
            if step.name == "PHARMACY_RETURN":
                self.dock_at_pharmacy()
                return

            self.enter_room(step)
            self.exit_room(step)

    def leave_pharmacy(self):
        self.robot_movement.drive_distance_cm(
            self.PHARMACY_EXIT_DISTANCE_CM, power=self.ROOM_DRIVE_POWER
        )
        self.acquire_line(preferred_turn_deg=0.0)

    def follow_line_until(self, step: RouteStep):
        marker_count = 0
        self.robot_movement.reset_drive_reference()

        while True:
            color = self.detect_route_marker()
            rgb = self.color_sensor.get_current_rgb()
            distance_cm = abs(self.robot_movement.get_distance_travelled_cm())

            if (
                step.stop_color is not None
                and distance_cm >= step.min_distance_cm
                and color == step.stop_color
            ):
                marker_count += 1
                if marker_count >= self.MARKER_CONFIRM_SAMPLES:
                    self.robot_movement.stop_move()
                    return
            else:
                marker_count = 0

            if not self.color_sensor.sees_line(rgb) and not self.color_sensor.is_route_marker(
                color
            ):
                self.recover_line(step.branch_turn_deg)
                continue

            correction = self.compute_correction(rgb)
            left_power = self.BASE_SPEED + correction
            right_power = self.BASE_SPEED - correction
            self.robot_movement.adjust_speed(left_power, right_power)
            sleep(self.LINE_FOLLOW_SLEEP_S)

    def detect_route_marker(self) -> str:
        return self.color_sensor.get_current_color()

    def enter_room(self, step: RouteStep):
        self.robot_movement.stop_move()
        self.robot_movement.drive_distance_cm(
            step.approach_distance_cm, power=self.ROOM_DRIVE_POWER
        )

        turn_angle = self.resolve_turn_angle(step)
        if turn_angle != 0:
            self.robot_movement.pivot_turn(turn_angle, power=self.TURN_POWER)

        self.robot_movement.drive_distance_cm(
            step.room_entry_distance_cm, power=self.ROOM_DRIVE_POWER
        )

        callback = step.on_arrival
        if callback is None and self.zone_detection is not None:
            callback = getattr(self.zone_detection, "detect_zone", None)

        if callable(callback):
            callback()
        else:
            sleep(step.pause_seconds)

    def exit_room(self, step: RouteStep):
        self.robot_movement.drive_distance_cm(
            -step.room_exit_distance_cm, power=self.ROOM_DRIVE_POWER
        )

        turn_angle = self.resolve_turn_angle(step)
        if turn_angle != 0:
            self.robot_movement.pivot_turn(-turn_angle, power=self.TURN_POWER)

        self.robot_movement.drive_distance_cm(
            self.REACQUIRE_FORWARD_DISTANCE_CM, power=self.REACQUIRE_POWER
        )
        self.acquire_line(preferred_turn_deg=turn_angle)

    def dock_at_pharmacy(self):
        self.robot_movement.drive_distance_cm(
            self.PHARMACY_DOCK_DISTANCE_CM, power=self.ROOM_DRIVE_POWER
        )
        self.robot_movement.stop_move()
        self.play_effect("FINISH")

    def acquire_line(self, preferred_turn_deg: float = 0.0) -> bool:
        direction = 1 if preferred_turn_deg >= 0 else -1
        started_at = monotonic()

        while monotonic() - started_at < self.LINE_REACQUIRE_TIMEOUT_S:
            if self.color_sensor.sees_line():
                self.robot_movement.stop_move()
                return True

            left_power = self.REACQUIRE_POWER + (4.0 * direction)
            right_power = self.REACQUIRE_POWER - (4.0 * direction)
            self.robot_movement.adjust_speed(left_power, right_power)
            sleep(self.LINE_FOLLOW_SLEEP_S)

        self.robot_movement.stop_move()
        return False

    def recover_line(self, preferred_turn_deg: float):
        direction = 1 if preferred_turn_deg >= 0 else -1
        self.robot_movement.adjust_speed(8.0 * direction, -8.0 * direction)
        sleep(0.08)
        self.robot_movement.stop_move()

    def resolve_turn_angle(self, step: RouteStep) -> float:
        if step.branch_turn_deg != 0:
            return step.branch_turn_deg
        if step.room_side == "left":
            return -90.0
        if step.room_side == "right":
            return 90.0
        return 0.0

    def compute_correction(self, rgb: tuple[float, float, float]) -> float:
        error = self.color_sensor.get_line_error(rgb, self.EDGE_TARGET)
        correction = self.STEERING_SIGN * self.CORRECTION_GAIN * error
        return max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, correction))

    def play_effect(self, effect_name: str):
        if self.sound_engine is None:
            return
        play_effect = getattr(self.sound_engine, "play_effect", None)
        if callable(play_effect):
            play_effect(effect_name)
