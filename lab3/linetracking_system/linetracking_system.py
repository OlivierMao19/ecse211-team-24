from __future__ import annotations

from dataclasses import dataclass
from time import sleep
from typing import Any, Callable, List, Optional, Tuple

from color_sensor.color_sensor import ColorSensor
from gyro_sensor.gyro_sensor import GyroSensor
from robot_movement.robot_movement import RobotMovement
from stop_button.stop_button import StopButton


@dataclass
class RouteStep:
    name: str
    intersections_to_pass: int = 0
    entry_turn_deg: float = 0.0
    entry_marker_color: Optional[str] = None
    entry_motor_degrees: float = 0.0
    exit_motor_degrees: float = 0.0
    room_depth_cm: float = 0.0
    exit_detect_intersection_after_cm: float = 0.0
    exit_intersection_count: int = 1
    room_exit_turn_deg: float = 0.0
    stop_after_exit: bool = False
    on_arrival: Optional[Callable[[], None]] = None
    pause_seconds: float = 0.5


class LineTracker:
    LEFT_TURN_DEG: float = -90.0
    RIGHT_TURN_DEG: float = 90.0

    BASE_SPEED: float = 20.0
    REVERSE_BASE_SPEED: float = 16.0
    LEFT_SPEED_TRIM: float = 0.0
    RIGHT_SPEED_TRIM: float = 2.0
    MAX_CORRECTION: float = 10.0
    CORRECTION_GAIN: float = 20.0
    # The color sensor is mounted slightly left of center, so we track the left edge.
    EDGE_TARGET: float = 0.55
    STEERING_SIGN: float = -1.0
    REVERSE_STEERING_SIGN: float = 1.0

    TURN_POWER: float = 16.0
    ROOM_DRIVE_POWER: float = 18.0
    ROOM_SEARCH_POWER: float = 18.0
    TURN_MOTOR_DEGREES_90: float = 260.0
    REACQUIRE_POWER: float = 12.0
    INTERSECTION_CONFIRM_SAMPLES: int = 4
    LINE_FOLLOW_SLEEP_S: float = 0.01
    INTERSECTION_CENTERING_CM: float = 8.0
    STRAIGHT_INTERSECTION_PASS_CM: float = 6.0
    REACQUIRE_TIMEOUT_S: float = 1.75

    PHARMACY_EXIT_DISTANCE_CM: float = 18.0
    ROOM_1_DEPTH_CM: float = 15.0
    ROOM_2_ENTRY_DEPTH_CM: float = 15.0
    ROOM_3_ENTRY_DEPTH_CM: float = 15.0

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
        self.last_correction_sign = -1.0
        self.last_reverse_correction_sign = -1.0
        self.route_steps = self.build_default_route()

    def build_default_route(self) -> List[RouteStep]:
        return [
            RouteStep(
                name="ROOM_1",
                intersections_to_pass=2,
                entry_marker_color="YELLOW",
                entry_motor_degrees=720.0,
                exit_motor_degrees=720.0,
                exit_intersection_count=0,
                room_depth_cm=0.0,
                room_exit_turn_deg=self.RIGHT_TURN_DEG,
            ),
            RouteStep(
                name="ROOM_2",
                intersections_to_pass=1,
                entry_turn_deg=self.LEFT_TURN_DEG,
                room_depth_cm=self.ROOM_2_ENTRY_DEPTH_CM,
                room_exit_turn_deg=self.RIGHT_TURN_DEG,
            ),
            RouteStep(
                name="ROOM_3",
                intersections_to_pass=0,
                entry_turn_deg=self.RIGHT_TURN_DEG,
                room_depth_cm=self.ROOM_3_ENTRY_DEPTH_CM,
                room_exit_turn_deg=0.0,
                stop_after_exit=True,
            ),
        ]

    def follow_line(self):
        self.run_route(self.route_steps)

    def run_route(self, route: List[RouteStep]):
        self.leave_pharmacy()

        for step in route:
            self.follow_route_to_room(step)
            self.enter_room(step)
            self.exit_room(step)
            if step.stop_after_exit:
                self.robot_movement.stop_move()
                return

    def leave_pharmacy(self):
        self.robot_movement.drive_distance_cm(
            self.PHARMACY_EXIT_DISTANCE_CM, power=self.ROOM_DRIVE_POWER
        )
        self.require_line(preferred_turn_deg=0.0, reason="pharmacy exit")

    def run_room_callback(self, step: RouteStep):
        callback = step.on_arrival
        if callback is None and self.zone_detection is not None:
            callback = getattr(self.zone_detection, "detect_zone", None)

        if callable(callback):
            callback()
            return

        sleep(step.pause_seconds)

    def follow_route_to_room(self, step: RouteStep):
        print("[route] heading to", step.name)
        self.pass_intersections(step.intersections_to_pass)

        if step.entry_turn_deg != 0:
            turn_name = "left" if step.entry_turn_deg < 0 else "right"
            print("[route]", step.name, "turning", turn_name, "at next intersection")
            self.turn_at_next_intersection(step.entry_turn_deg)
            return

        if step.entry_marker_color is not None:
            print("[route]", step.name, "searching for", step.entry_marker_color)
            self.drive_straight_until_color(step.entry_marker_color)
            return

        self.robot_movement.stop_move()

    def drive_straight_until_color(self, target_color: str):
        print("[route] driving straight until color", target_color)
        while self.color_sensor.get_current_color() != target_color:
            left_power = self.ROOM_SEARCH_POWER + self.LEFT_SPEED_TRIM
            right_power = self.ROOM_SEARCH_POWER + self.RIGHT_SPEED_TRIM
            self.robot_movement.adjust_speed(left_power, right_power)
            sleep(self.LINE_FOLLOW_SLEEP_S)

        self.robot_movement.stop_move()
        print("[route] detected color", target_color)
        print("[route] going forward for 720 degrees more")

    def follow_line_step(self):
        rgb = self.color_sensor.get_current_rgb()
        color = self.color_sensor.get_current_color()

        if not self.color_sensor.sees_line(rgb, color):
            self.recover_line()
            return

        correction = self.compute_correction(rgb)
        self.last_correction_sign = 1.0 if correction >= 0 else -1.0
        left_power = self.BASE_SPEED + self.LEFT_SPEED_TRIM + correction
        right_power = self.BASE_SPEED + self.RIGHT_SPEED_TRIM - correction
        self.robot_movement.adjust_speed(left_power, right_power)
        sleep(self.LINE_FOLLOW_SLEEP_S)

    def follow_line_backward_step(self):
        rgb = self.color_sensor.get_current_rgb()
        color = self.color_sensor.get_current_color()

        if not self.color_sensor.sees_line(rgb, color):
            self.recover_line_backward()
            return

        correction = self.compute_reverse_correction(rgb)
        self.last_reverse_correction_sign = 1.0 if correction >= 0 else -1.0
        left_power = -self.REVERSE_BASE_SPEED + self.LEFT_SPEED_TRIM + correction
        right_power = -self.REVERSE_BASE_SPEED + self.RIGHT_SPEED_TRIM - correction
        self.robot_movement.adjust_speed(left_power, right_power)
        sleep(self.LINE_FOLLOW_SLEEP_S)

    def turn_at_next_intersection(self, turn_deg: float):
        turn_name = "left" if turn_deg < 0 else "right"
        print("[route] waiting for next intersection to turn", turn_name)
        self.wait_for_intersection_forward(1)
        self.robot_movement.drive_distance_cm(
            self.INTERSECTION_CENTERING_CM, power=self.ROOM_DRIVE_POWER
        )

        if turn_deg != 0:
            print("[route] turning", turn_name)
            self.execute_turn(turn_deg)

    def pass_intersections(self, intersections_to_pass: int):
        if intersections_to_pass <= 0:
            return

        print("[route] passing", intersections_to_pass, "intersection(s)")
        intersections_passed = 0

        while intersections_passed < intersections_to_pass:
            self.wait_for_intersection_forward(1)
            self.pass_straight_through_intersection()
            intersections_passed += 1
            print("[route] passed intersection", intersections_passed, "/", intersections_to_pass)

        self.require_line(preferred_turn_deg=0.0, reason="passing intersections")

    def enter_room(self, step: RouteStep):
        if step.entry_motor_degrees > 0:
            print("[room] entering", step.name, "for", step.entry_motor_degrees, "motor degrees")
            self.robot_movement.drive_motor_degrees(
                step.entry_motor_degrees, power=self.ROOM_DRIVE_POWER
            )
            self.run_room_callback(step)
            return

        if step.room_depth_cm <= 0:
            return

        print("[room] entering", step.name)
        self.drive_into_room(step.room_depth_cm)
        self.run_room_callback(step)

    def exit_room(self, step: RouteStep):
        if step.exit_motor_degrees > 0:
            print("[room] backing out of", step.name, "for", step.exit_motor_degrees, "motor degrees")
            self.robot_movement.drive_motor_degrees(
                -step.exit_motor_degrees, power=self.ROOM_DRIVE_POWER
            )
            self.reverse_until_intersection(step)
        elif step.room_depth_cm > 0:
            print("[room] backing out of", step.name)
            self.back_out_of_room(step.room_depth_cm)
            self.reverse_until_intersection(step)
        else:
            return

        if step.room_exit_turn_deg != 0:
            turn_name = "left" if step.room_exit_turn_deg < 0 else "right"
            print("[room]", step.name, "turning", turn_name, "after exit")
            self.execute_turn(step.room_exit_turn_deg)
            self.require_line(
                preferred_turn_deg=step.room_exit_turn_deg,
                reason="room exit turn at " + step.name,
            )

    def acquire_line(self, preferred_turn_deg: float = 0.0, timeout_s: float = None) -> bool:
        if timeout_s is None:
            timeout_s = self.REACQUIRE_TIMEOUT_S
        direction = 1 if preferred_turn_deg >= 0 else -1
        elapsed = 0.0

        while elapsed < timeout_s:
            rgb = self.color_sensor.get_current_rgb()
            color = self.color_sensor.get_current_color()
            if self.color_sensor.sees_line(rgb, color):
                self.robot_movement.stop_move()
                return True

            left_power = self.REACQUIRE_POWER + (4.0 * direction)
            right_power = self.REACQUIRE_POWER - (4.0 * direction)
            self.robot_movement.adjust_speed(left_power, right_power)
            sleep(self.LINE_FOLLOW_SLEEP_S)
            elapsed += self.LINE_FOLLOW_SLEEP_S

        self.robot_movement.stop_move()
        return False

    def require_line(self, preferred_turn_deg: float, reason: str):
        if self.acquire_line(preferred_turn_deg=preferred_turn_deg):
            return

        self.robot_movement.stop_move()
        raise RuntimeError("Could not reacquire line after " + reason)

    def recover_line(self):
        direction = self.last_correction_sign
        self.robot_movement.adjust_speed(8.0 * direction, -8.0 * direction)
        sleep(0.08)
        self.robot_movement.stop_move()

    def pass_straight_through_intersection(self):
        self.robot_movement.drive_distance_cm(
            self.STRAIGHT_INTERSECTION_PASS_CM, power=self.ROOM_DRIVE_POWER
        )

    def recover_line_backward(self):
        direction = self.last_reverse_correction_sign
        self.robot_movement.adjust_speed(-8.0 * direction, 8.0 * direction)
        sleep(0.08)
        self.robot_movement.stop_move()

    def drive_into_room(self, distance_cm: float):
        self.robot_movement.drive_distance_cm(
            distance_cm, power=self.ROOM_DRIVE_POWER
        )

    def back_out_of_room(self, distance_cm: float):
        self.robot_movement.drive_distance_cm(
            -distance_cm, power=self.ROOM_DRIVE_POWER
        )

    def reverse_until_intersection(self, step: RouteStep):
        if step.exit_intersection_count <= 0:
            return

        print("[room] reversing until previous intersection for", step.name)
        intersections_seen = 0
        intersection_samples = 0
        waiting_to_clear = False
        detect_after = step.exit_detect_intersection_after_cm
        self.robot_movement.reset_drive_reference()

        while intersections_seen < step.exit_intersection_count:
            self.follow_line_backward_step()

            rgb = self.color_sensor.get_current_rgb()
            color = self.color_sensor.get_current_color()
            distance_cm = abs(self.robot_movement.get_distance_travelled_cm())
            ready_to_detect = distance_cm >= detect_after

            if not ready_to_detect:
                continue

            if self.color_sensor.is_intersection_candidate(rgb, color):
                intersection_samples += 1
                if (
                    not waiting_to_clear
                    and intersection_samples >= self.INTERSECTION_CONFIRM_SAMPLES
                ):
                    intersections_seen += 1
                    print("[room] reverse intersection", intersections_seen, "/", step.exit_intersection_count)
                    waiting_to_clear = True
                    intersection_samples = 0
                    self.robot_movement.stop_move()
                    sleep(0.05)
            else:
                intersection_samples = 0
                if waiting_to_clear:
                    waiting_to_clear = False

        self.robot_movement.stop_move()
        sleep(0.05)

    def wait_for_intersection_forward(self, intersection_count: int):
        if intersection_count <= 0:
            return

        print("[route] waiting for", intersection_count, "forward intersection(s)")
        intersections_seen = 0
        intersection_samples = 0
        waiting_to_clear = False

        while intersections_seen < intersection_count:
            self.follow_line_step()

            rgb = self.color_sensor.get_current_rgb()
            color = self.color_sensor.get_current_color()

            if self.color_sensor.is_intersection_candidate(rgb, color):
                intersection_samples += 1
                if (
                    not waiting_to_clear
                    and intersection_samples >= self.INTERSECTION_CONFIRM_SAMPLES
                ):
                    intersections_seen += 1
                    print("[route] detected forward intersection", intersections_seen, "/", intersection_count)
                    waiting_to_clear = True
                    intersection_samples = 0
                    self.robot_movement.stop_move()
                    sleep(0.05)
            else:
                intersection_samples = 0
                if waiting_to_clear:
                    waiting_to_clear = False

    def turn_left(self):
        self.execute_turn(self.LEFT_TURN_DEG)

    def turn_right(self):
        self.execute_turn(self.RIGHT_TURN_DEG)

    def execute_turn(self, turn_deg: float):
        motor_turn_degrees = self.TURN_MOTOR_DEGREES_90
        if turn_deg == self.LEFT_TURN_DEG:
            self.robot_movement.pivot_turn_motor_degrees(
                -motor_turn_degrees, power=self.TURN_POWER
            )
            return
        if turn_deg == self.RIGHT_TURN_DEG:
            self.robot_movement.pivot_turn_motor_degrees(
                motor_turn_degrees, power=self.TURN_POWER
            )
            return

        direction = 1.0 if turn_deg > 0 else -1.0
        self.robot_movement.pivot_turn_motor_degrees(
            direction * motor_turn_degrees, power=self.TURN_POWER
        )

    def compute_correction(self, rgb: Tuple[float, float, float]) -> float:
        error = self.color_sensor.get_line_error(rgb, self.EDGE_TARGET)
        correction = self.STEERING_SIGN * self.CORRECTION_GAIN * error
        return max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, correction))

    def compute_reverse_correction(self, rgb: Tuple[float, float, float]) -> float:
        error = self.color_sensor.get_line_error(rgb, self.EDGE_TARGET)
        correction = self.REVERSE_STEERING_SIGN * self.CORRECTION_GAIN * error
        return max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, correction))

    def play_effect(self, effect_name: str):
        if self.sound_engine is None:
            return
        play_effect = getattr(self.sound_engine, "play_effect", None)
        if callable(play_effect):
            play_effect(effect_name)
