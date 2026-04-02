from time import sleep
from typing import Optional, Tuple

from color_sensor.color_sensor import ColorSensor
from package_pickup.pickup_controller import PickupController
from robot_movement.robot_movement import RobotMovement
from sound.robot_sound import RobotSound


class RoomScanner:
    def __init__(
        self,
        robot_movement: RobotMovement,
        color_sensor: ColorSensor,
        room_approach_power: float,
        room_scan_power: float,
        room_color_confirm_samples: int,
        room_scan_print_every_degrees: float,
        room_scan_max_degrees: float,
        sweep_left_arc_degrees: float,
        sweep_right_arc_degrees: float,
        sweep_step_degrees: float,
        sweep_outer_power: float,
        sweep_inner_power: float,
        sweep_left_trim: float = 0.0,
        sweep_right_trim: float = 0.0,
        sweep_left_return_scale: float = 1.0,
        sweep_right_return_scale: float = 1.0,
        room_entry_pause_s: float = 0.0,
        step_pause_s: float = 0.0,
        realign_to_room_heading: bool = True,
        room_heading_extra_correction_deg: float = 0.0,
        room_heading_tolerance_deg: float = 2.0,
        room_heading_turn_power: float = 8.0,
        room_exit_extra_degrees: float = 0.0,
        robot_sound: Optional[RobotSound] = None,
        pickup_controller: Optional[PickupController] = None,
        dropoff_left_rotate_degrees: float = -180.0,
        dropoff_right_rotate_degrees: float = 180.0,
        dropoff_detect_pause_s: float = 0.8,
        dropoff_shift_degrees: float = 180.0,
        dropoff_opposite_shift_degrees: Optional[float] = None,
        dropoff_approach_degrees: float = 180.0,
        dropoff_shift_outer_power: float = 24.0,
        dropoff_shift_inner_power: float = 12.0,
        dropoff_pause_s: float = 0.4,
    ):
        self.robot_movement = robot_movement
        self.color_sensor = color_sensor
        self.room_approach_power = room_approach_power
        self.room_scan_power = room_scan_power
        self.room_color_confirm_samples = room_color_confirm_samples
        self.room_scan_print_every_degrees = room_scan_print_every_degrees
        self.room_scan_max_degrees = room_scan_max_degrees
        self.sweep_left_arc_degrees = sweep_left_arc_degrees
        self.sweep_right_arc_degrees = sweep_right_arc_degrees
        self.sweep_step_degrees = sweep_step_degrees
        self.sweep_outer_power = sweep_outer_power
        self.sweep_inner_power = sweep_inner_power
        self.sweep_left_trim = sweep_left_trim
        self.sweep_right_trim = sweep_right_trim
        self.sweep_left_return_scale = sweep_left_return_scale
        self.sweep_right_return_scale = sweep_right_return_scale
        self.room_entry_pause_s = room_entry_pause_s
        self.step_pause_s = step_pause_s
        self.realign_to_room_heading = realign_to_room_heading
        self.room_heading_extra_correction_deg = room_heading_extra_correction_deg
        self.room_heading_tolerance_deg = room_heading_tolerance_deg
        self.room_heading_turn_power = room_heading_turn_power
        self.room_exit_extra_degrees = room_exit_extra_degrees
        self.robot_sound = robot_sound
        self.pickup_controller = pickup_controller
        self.dropoff_left_rotate_degrees = dropoff_left_rotate_degrees
        self.dropoff_right_rotate_degrees = dropoff_right_rotate_degrees
        self.dropoff_detect_pause_s = dropoff_detect_pause_s
        self.dropoff_shift_degrees = dropoff_shift_degrees
        self.dropoff_opposite_shift_degrees = (
            dropoff_shift_degrees
            if dropoff_opposite_shift_degrees is None
            else dropoff_opposite_shift_degrees
        )
        self.dropoff_approach_degrees = dropoff_approach_degrees
        self.dropoff_shift_outer_power = dropoff_shift_outer_power
        self.dropoff_shift_inner_power = dropoff_shift_inner_power
        self.dropoff_pause_s = dropoff_pause_s
        self.completed_dropoffs = 0
        self.room_heading_reference = None

    def has_completed_all_dropoffs(self) -> bool:
        return self.completed_dropoffs >= 2

    def scan_room(self, max_room_entry_degrees: float):
        print(
            "Room approach: driving until yellow for up to %.0f motor degrees"
            % max_room_entry_degrees
        )

        self.robot_movement.reset_drive_reference()
        self.robot_movement.start_heading_hold()

        streak_color = None
        streak_count = 0

        while abs(self.robot_movement.get_average_encoder()) < max_room_entry_degrees:
            self.robot_movement.adjust_heading_hold(self.room_approach_power)
            self._print_color_measurement("Room approach:")
            found_yellow, streak_color, streak_count = self._wait_for_color(
                ("YELLOW",),
                streak_color,
                streak_count,
            )
            if found_yellow:
                yellow_position = abs(self.robot_movement.get_average_encoder())
                print(
                    "Room approach: detected YELLOW at %.0f motor degrees"
                    % yellow_position
                )
                self.robot_movement.stop_move()
                self._mark_room_heading()
                if self.room_entry_pause_s > 0:
                    sleep(self.room_entry_pause_s)
                return self._sweep_for_bed()

            sleep(0.01)

        self.robot_movement.stop_move()
        print("Room approach: yellow not detected, continuing mission")
        return None

    def _sweep_for_bed(self):
        forward_progress = 0.0

        initial_step_degrees = min(self.sweep_step_degrees, self.room_scan_max_degrees)
        if initial_step_degrees > 0:
            detected_color, travelled = self._scan_straight_step(initial_step_degrees)
            forward_progress += travelled
            if detected_color is not None:
                self._play_detected_color_sound(detected_color)
                self._back_to_yellow(forward_progress)
                return detected_color
            self._pause_between_steps()

        while forward_progress < self.room_scan_max_degrees:
            detected_color, travelled = self._scan_arc("left")
            if detected_color is not None:
                self._handle_arc_detection("left", detected_color, travelled)
                self._pause_between_steps()
                self._back_to_yellow(forward_progress)
                return detected_color
            self._return_from_arc("left", travelled)
            self._pause_between_steps()

            detected_color, travelled = self._scan_arc("right")
            if detected_color is not None:
                self._handle_arc_detection("right", detected_color, travelled)
                self._pause_between_steps()
                self._back_to_yellow(forward_progress)
                return detected_color
            self._return_from_arc("right", travelled)
            self._pause_between_steps()

            remaining_progress = self.room_scan_max_degrees - forward_progress
            step_degrees = min(self.sweep_step_degrees, remaining_progress)
            detected_color, travelled = self._scan_straight_step(step_degrees)
            forward_progress += travelled
            if detected_color is not None:
                if detected_color == "GREEN":
                    next_dropoff_side = self._get_next_dropoff_side()
                    if next_dropoff_side is not None:
                        virtual_side = "right" if next_dropoff_side == "left" else "left"
                        self._handle_green_detection(virtual_side, 0.0)
                else:
                    self._play_detected_color_sound(detected_color)
                self._back_to_yellow(forward_progress)
                return detected_color
            self._pause_between_steps()

        print(
            "Room scan: no GREEN or RED found within %.0f motor degrees"
            % self.room_scan_max_degrees
        )
        if forward_progress > 0:
            self._back_to_yellow(forward_progress)
        return None

    def _scan_arc(self, side: str) -> Tuple[Optional[str], float]:
        label = "Room scan: pivot %s" % side
        left_power, right_power = self._get_arc_powers(side)
        if side == "left":
            target_degrees = self.sweep_left_arc_degrees
        else:
            target_degrees = self.sweep_right_arc_degrees

        return self._run_segment_with_detection(
            left_power,
            right_power,
            target_degrees,
            label,
            use_turn_progress=True,
        )

    def _scan_straight_step(self, motor_degrees: float) -> Tuple[Optional[str], float]:
        print("Room scan: advancing forward %.0f motor degrees" % motor_degrees)
        self.robot_movement.reset_drive_reference()
        self.robot_movement.start_heading_hold()
        streak_color = None
        streak_count = 0
        last_reported_bucket = -1

        while abs(self.robot_movement.get_average_encoder()) < motor_degrees:
            self.robot_movement.adjust_heading_hold(self.room_scan_power)
            travelled = abs(self.robot_movement.get_average_encoder())
            self._print_color_measurement("Room scan:")
            report_bucket = int(
                travelled / self.room_scan_print_every_degrees
            )
            if report_bucket > last_reported_bucket:
                print(
                    "Room scan: %.0f degrees into forward step, color=%s"
                    % (travelled, self.color_sensor.get_current_color())
                )
                last_reported_bucket = report_bucket

            found_bed_color, streak_color, streak_count = self._wait_for_color(
                ("GREEN", "RED"),
                streak_color,
                streak_count,
            )
            if found_bed_color:
                self.robot_movement.stop_move()
                print(
                    "Room scan: detected %s after %.0f motor degrees"
                    % (streak_color, travelled)
                )
                return streak_color, travelled

            sleep(0.01)

        self.robot_movement.stop_move()
        return None, abs(self.robot_movement.get_average_encoder())

    def _run_segment_with_detection(
        self,
        left_power: float,
        right_power: float,
        target_degrees: float,
        label: str,
        use_turn_progress: bool = False,
    ) -> Tuple[Optional[str], float]:
        self.robot_movement.reset_drive_reference()
        streak_color = None
        streak_count = 0
        detected_color = None

        while self._get_segment_progress(use_turn_progress) < target_degrees:
            trimmed_left, trimmed_right = self._get_sweep_trimmed_powers(
                left_power,
                right_power,
            )
            self.robot_movement.adjust_speed(trimmed_left, trimmed_right)
            travelled = self._get_segment_progress(use_turn_progress)
            self._print_color_measurement(label)
            found_bed_color, streak_color, streak_count = self._wait_for_color(
                ("GREEN", "RED"),
                streak_color,
                streak_count,
            )
            if found_bed_color:
                self.robot_movement.stop_move()
                print(
                    "%s detected %s after %.0f motor degrees"
                    % (label, streak_color, travelled)
                )
                return streak_color, travelled

            sleep(0.01)

        self.robot_movement.stop_move()
        return detected_color, self._get_segment_progress(use_turn_progress)

    def _run_segment_without_detection(
        self,
        left_power: float,
        right_power: float,
        target_degrees: float,
        label: str,
        use_turn_progress: bool = False,
    ):
        if target_degrees <= 0:
            return

        print("%s for %.0f motor degrees" % (label, target_degrees))
        self.robot_movement.reset_drive_reference()
        while self._get_segment_progress(use_turn_progress) < target_degrees:
            trimmed_left, trimmed_right = self._get_sweep_trimmed_powers(
                left_power,
                right_power,
            )
            self.robot_movement.adjust_speed(trimmed_left, trimmed_right)
            sleep(0.01)
        self.robot_movement.stop_move()

    def _back_to_yellow(self, travelled_since_yellow: float):
        total_backout = travelled_since_yellow + self.room_exit_extra_degrees
        if total_backout <= 0:
            return

        self._realign_to_room_heading()
        print(
            "Room scan: backing up %.0f motor degrees to return from yellow scan"
            % total_backout
        )
        self.robot_movement.drive_motor_degrees_heading(
            -total_backout,
            self.room_approach_power,
        )

    def _handle_arc_detection(
        self,
        side: str,
        detected_color: str,
        travelled: float,
    ):
        if detected_color == "GREEN":
            self._handle_green_detection(side, travelled)
            return

        if self.dropoff_detect_pause_s > 0:
            sleep(self.dropoff_detect_pause_s)
        self._return_from_arc(side, travelled)

    def _return_from_arc(self, side: str, travelled: float):
        if travelled <= 0:
            return

        return_left_power, return_right_power = self._get_return_powers(side)
        self._run_segment_without_detection(
            return_left_power,
            return_right_power,
            self._get_return_degrees(side, travelled),
            "Room scan: backing out of curved %s sweep" % side,
            use_turn_progress=True,
        )

    def _wait_for_color(
        self,
        target_colors: Tuple[str, ...],
        current_streak_color: Optional[str],
        current_streak_count: int,
    ) -> Tuple[bool, Optional[str], int]:
        current_color = self.color_sensor.get_current_color()
        if current_color == current_streak_color:
            current_streak_count += 1
        else:
            current_streak_color = current_color
            current_streak_count = 1

        matched = (
            current_streak_color in target_colors
            and current_streak_count >= self.room_color_confirm_samples
        )
        return matched, current_streak_color, current_streak_count

    def _print_color_measurement(self, prefix: str):
        rgb = self.color_sensor.get_current_rgb()
        color = self.color_sensor.get_current_color()
        print(
            "%s rgb=(%.1f, %.1f, %.1f) detected=%s"
            % (prefix, rgb[0], rgb[1], rgb[2], color)
        )

    def _play_detected_color_sound(self, detected_color: str):
        if detected_color != "GREEN" or self.robot_sound is None:
            return
        print("Room scan: GREEN detected, playing beep")
        self.robot_sound.play_green_detected()

    def _run_dropoff(self, side: str):
        if self.pickup_controller is None:
            return

        rotate_degrees = self.dropoff_left_rotate_degrees
        if side != "left":
            rotate_degrees = self.dropoff_right_rotate_degrees

        print(
            "Room scan: releasing %s cube with %.0f motor degrees"
            % (side, abs(rotate_degrees))
        )
        if side == "left":
            self.pickup_controller.rotate_left_relative(rotate_degrees)
            return
        self.pickup_controller.rotate_right_relative(rotate_degrees)

    def _handle_green_detection(self, side: str, travelled: float):
        dropoff_side = self._get_next_dropoff_side()
        if dropoff_side is None:
            print("Room scan: GREEN detected but no cubes remain for dropoff")
            self._return_from_arc(side, travelled)
            self._play_detected_color_sound("GREEN")
            sleep(self.dropoff_pause_s)
            return

        if self.dropoff_detect_pause_s > 0:
            sleep(self.dropoff_detect_pause_s)

        offset_degrees = self.dropoff_shift_degrees
        return_degrees = self._get_return_degrees(side, travelled)
        if dropoff_side == side:
            # The matching scoop is closer after a small move toward the middle.
            offset_left_power, offset_right_power = self._get_return_powers(side)
            offset_degrees = min(offset_degrees, return_degrees)
            remaining_return = max(return_degrees - offset_degrees, 0.0)
            offset_label = "Room scan: offset %s toward middle for dropoff" % side
        else:
            # The opposite scoop needs a little more travel in the current sweep direction.
            offset_degrees = self.dropoff_opposite_shift_degrees
            offset_left_power, offset_right_power = self._get_arc_powers(side)
            remaining_return = return_degrees + offset_degrees
            offset_label = "Room scan: offset %s outward for opposite dropoff" % side

        self._run_segment_without_detection(
            offset_left_power,
            offset_right_power,
            offset_degrees,
            offset_label,
            use_turn_progress=True,
        )
        print("Room scan: opening %s pickup motor on GREEN" % dropoff_side)
        self._run_dropoff(dropoff_side)
        self.completed_dropoffs += 1
        self._play_detected_color_sound("GREEN")
        sleep(self.dropoff_pause_s)
        self._run_segment_without_detection(
            *self._get_return_powers(side),
            remaining_return,
            "Room scan: backing out of curved %s sweep" % side,
            use_turn_progress=True,
        )

    def _pause_between_steps(self):
        if self.step_pause_s <= 0:
            return
        sleep(self.step_pause_s)

    def _get_sweep_trimmed_powers(
        self,
        left_power: float,
        right_power: float,
    ) -> Tuple[float, float]:
        trimmed_left = left_power + self._signed_trim(left_power, self.sweep_left_trim)
        trimmed_right = right_power + self._signed_trim(
            right_power,
            self.sweep_right_trim,
        )
        return trimmed_left, trimmed_right

    def _signed_trim(self, power: float, trim: float) -> float:
        if power == 0 or trim == 0:
            return 0.0
        return trim if power > 0 else -trim

    def _get_return_powers(self, side: str) -> Tuple[float, float]:
        if side == "left":
            return self.sweep_inner_power, -self.sweep_outer_power
        return -self.sweep_outer_power, self.sweep_inner_power

    def _get_arc_powers(self, side: str) -> Tuple[float, float]:
        if side == "left":
            return -self.sweep_inner_power, self.sweep_outer_power
        return self.sweep_outer_power, -self.sweep_inner_power

    def _get_return_degrees(self, side: str, travelled: float) -> float:
        if side == "left":
            return travelled * self.sweep_left_return_scale
        return travelled * self.sweep_right_return_scale

    def _get_segment_progress(self, use_turn_progress: bool) -> float:
        if use_turn_progress:
            return self.robot_movement.get_turn_encoder_progress()
        return abs(self.robot_movement.get_average_encoder())

    def _get_next_dropoff_side(self) -> Optional[str]:
        if self.completed_dropoffs == 0:
            return "left"
        if self.completed_dropoffs == 1:
            return "right"
        return None

    def _mark_room_heading(self):
        if self.robot_movement.gyro_sensor is None:
            return
        self.robot_movement.gyro_sensor.set_reference()
        self.room_heading_reference = self.robot_movement.gyro_sensor.get_reference()
        print("Room scan: stored room heading reference")

    def _realign_to_room_heading(self):
        if not self.realign_to_room_heading or self.robot_movement.gyro_sensor is None:
            return
        if self.room_heading_reference is None:
            return

        self.robot_movement.gyro_sensor.set_reference(self.room_heading_reference)
        current_angle = self.robot_movement.gyro_sensor.get_angle()
        target_angle = 0.0
        if current_angle > 0:
            target_angle = -self.room_heading_extra_correction_deg
        elif current_angle < 0:
            target_angle = self.room_heading_extra_correction_deg

        heading_error = current_angle - target_angle
        if abs(heading_error) <= self.room_heading_tolerance_deg:
            return

        print(
            "Room scan: realigning from %.1f deg toward %.1f deg"
            % (current_angle, target_angle)
        )
        slow_turn_power = min(
            self.room_heading_turn_power,
            self.robot_movement.MIN_TURN_POWER,
        )
        settle_deadline = 120

        while settle_deadline > 0:
            current_angle = self.robot_movement.gyro_sensor.get_angle()
            heading_error = current_angle - target_angle
            if abs(heading_error) <= self.room_heading_tolerance_deg:
                break

            direction = -1 if heading_error > 0 else 1
            turn_power = self.room_heading_turn_power
            if abs(heading_error) <= 8.0:
                turn_power = slow_turn_power
            self.robot_movement.adjust_speed(
                direction * turn_power,
                -direction * turn_power,
            )
            sleep(0.01)
            settle_deadline -= 1

        self.robot_movement.stop_move()
        sleep(0.05)
        self.robot_movement.gyro_sensor.set_reference(self.room_heading_reference)
        final_error = (
            self.robot_movement.gyro_sensor.get_angle() - target_angle
        )
        print("Room scan: room heading error after realign %.1f deg" % final_error)
