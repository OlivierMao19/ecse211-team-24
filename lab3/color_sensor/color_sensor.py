import math
from pathlib import Path
from threading import Thread
from time import sleep
from typing import Dict, Optional, Tuple, Union

from utils.brick import EV3ColorSensor


class ColorSensor:
    sensor: EV3ColorSensor
    current_color: str
    cache: Dict[str, Tuple[float, float, float]] = {}
    thread: Thread
    thread_run: bool = True

    DEFAULT_COLOR_REFERENCES = {
        "RED": (0.8525, 0.0863, 0.0612),
        "GREEN": (0.3988, 0.5420, 0.0593),
        "BLUE": (0.1800, 0.2500, 0.5700),
        "YELLOW": (0.5678, 0.3906, 0.0416),
        "ORANGE": (0.6700, 0.2900, 0.0400),
        "WHITE": (0.3333, 0.3333, 0.3333),
    }
    MARKER_DISTANCE_THRESHOLD = 0.18
    BED_DISTANCE_THRESHOLD = 0.38
    YELLOW_DISTANCE_THRESHOLD = 0.14
    GREEN_DISTANCE_THRESHOLD = 0.12
    BLACK_BRIGHTNESS = 10.0
    DARK_SUM_THRESHOLD = 45.0
    WHITE_BRIGHTNESS = 260.0
    INTERSECTION_RATIO_THRESHOLD = 0.18

    def __init__(self, sensor: EV3ColorSensor):
        print("initializing color sensor")
        self.sensor = sensor
        self.current_color = "UNKNOWN"
        self.current_rgb: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.current_normalized_rgb: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.init_cache()

        self.thread = Thread(target=self.main, args=[])
        self.thread.start()

    def init_cache(self):
        self.cache = dict(self.DEFAULT_COLOR_REFERENCES)
        self._load_reference_files()
        print("cache initialized, ", self.cache)

    def _load_reference_files(self):
        calibration_dir = Path(__file__).resolve().parent / "calibration"
        if not calibration_dir.exists():
            return

        for file_path in calibration_dir.glob("*.txt"):
            color_name = file_path.stem.upper()
            values = self._read_reference_file(file_path)
            if values is not None:
                self.cache[color_name] = self.__normalize_rgb(values)

    def _read_reference_file(self, file_path: Path) -> Optional[Tuple[float, float, float]]:
        rows = [row.strip() for row in file_path.read_text().splitlines() if row.strip()]
        if not rows:
            return None

        r_sum, g_sum, b_sum = 0.0, 0.0, 0.0
        for row in rows:
            r, g, b = [float(value.strip()) for value in row.split(",")]
            r_sum += r
            g_sum += g
            b_sum += b

        n_rows = len(rows)
        return (r_sum / n_rows, g_sum / n_rows, b_sum / n_rows)

    def main(self):
        while self.thread_run:
            _ = self.__detect_color()
            sleep(0.01)

    def get_rgb(self) -> Tuple[float, float, float]:
        r, g, b = self.sensor.get_rgb()
        return r, g, b

    def __set_rgb_color(self, rgb: Tuple[float, float, float], color: str):
        self.current_rgb = rgb
        self.current_normalized_rgb = self.__normalize_rgb(rgb)
        self.current_color = color

    def __normalize_rgb(
        self, rgb: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        total = sum(rgb)
        if total <= 0:
            return (0.0, 0.0, 0.0)
        return rgb[0] / total, rgb[1] / total, rgb[2] / total

    def filter_data(
        self, r: Union[float, None], g: Union[float, None], b: Union[float, None]
    ):
        if r is not None and g is not None and b is not None:
            if r >= 0 and g >= 0 and b >= 0:
                return True
        return False

    def __handle_threshold(self, color: str):
        return color

    def get_brightness(self, rgb: Optional[Tuple[float, float, float]] = None) -> float:
        if rgb is None:
            rgb = self.current_rgb
        return sum(rgb) / 3.0

    def get_distance(self, rgb: Tuple[float, float, float], target_color: str) -> float:
        if target_color not in self.cache:
            return -1.0
        rr, gg, bb = self.cache[target_color]
        nr, ng, nb = self.__normalize_rgb(rgb)
        dist = math.sqrt((nr - rr) ** 2 + (ng - gg) ** 2 + (nb - bb) ** 2)
        return dist

    def classify_color(self, rgb: Tuple[float, float, float]) -> str:
        brightness = self.get_brightness(rgb)
        if sum(rgb) <= self.DARK_SUM_THRESHOLD or brightness <= self.BLACK_BRIGHTNESS:
            return "BLACK"
        if brightness >= self.WHITE_BRIGHTNESS:
            return "WHITE"

        color_found = "UNKNOWN"
        closest_dist = math.inf
        for name, reference in self.cache.items():
            if name == "WHITE":
                continue
            dist = self.get_distance(rgb, name)
            if dist < closest_dist:
                closest_dist = dist
                color_found = name

        if color_found == "YELLOW" and closest_dist <= self.YELLOW_DISTANCE_THRESHOLD:
            return color_found

        if color_found == "GREEN" and closest_dist <= self.GREEN_DISTANCE_THRESHOLD:
            return color_found

        if color_found == "RED" and closest_dist <= self.BED_DISTANCE_THRESHOLD:
            return color_found

        if closest_dist > self.MARKER_DISTANCE_THRESHOLD:
            return "UNKNOWN"
        return color_found

    def __detect_color(self):
        r, g, b = self.get_rgb()
        if not self.filter_data(r, g, b):
            self.__set_rgb_color((0.0, 0.0, 0.0), "UNKNOWN")
            return "UNKNOWN"
        color_found = self.classify_color((r, g, b))
        color_found = self.__handle_threshold(color_found)

        self.__set_rgb_color((r, g, b), color_found)
        return color_found

    def get_current_color(self) -> str:
        return self.current_color

    def get_current_rgb(self) -> Tuple[float, float, float]:
        return self.current_rgb

    def get_current_normalized_rgb(self) -> Tuple[float, float, float]:
        return self.current_normalized_rgb

    def get_line_ratio(self, rgb: Optional[Tuple[float, float, float]] = None) -> float:
        brightness = self.get_brightness(rgb)
        span = self.WHITE_BRIGHTNESS - self.BLACK_BRIGHTNESS
        if span <= 0:
            return 0.5

        ratio = (brightness - self.BLACK_BRIGHTNESS) / span
        return max(0.0, min(1.0, ratio))

    def get_line_error(
        self, rgb: Optional[Tuple[float, float, float]] = None, target_ratio: float = 0.45
    ) -> float:
        return target_ratio - self.get_line_ratio(rgb)

    def sees_line(
        self,
        rgb: Optional[Tuple[float, float, float]] = None,
        color: Optional[str] = None,
    ) -> bool:
        if color is None:
            color = self.current_color
        return self.get_line_ratio(rgb) < 0.8 and not self.is_route_marker(color)

    def is_route_marker(self, color: Optional[str] = None) -> bool:
        if color is None:
            color = self.current_color
        return color in {"BLUE", "YELLOW", "ORANGE"}

    def is_intersection_candidate(
        self,
        rgb: Optional[Tuple[float, float, float]] = None,
        color: Optional[str] = None,
    ) -> bool:
        if color is None:
            color = self.current_color
        return color == "BLACK" or self.get_line_ratio(rgb) <= self.INTERSECTION_RATIO_THRESHOLD

    def get_ratio(
        self, rgb: Tuple[float, float, float], target1: str, target2: str
    ) -> float:
        if target1 == "BLACK" and target2 == "WHITE":
            return self.get_line_ratio(rgb)

        dist_diff = self.get_distance(self.cache[target1], target2)
        diff = self.get_distance(rgb, target2)
        return diff / dist_diff if dist_diff > 0 else 0.0

    def dispose(self):
        print("disposing color sensor")
        self.thread_run = False
        self.thread.join()
