import math
from utils.brick import EV3ColorSensor


class ColorClassifier:
    def __init__(self, sensor: EV3ColorSensor):
        self.sensor = sensor

        # Missing blue average value - To measure tomorrow!
        self.reference_colors = {
            "Red":    (0.8525, 0.0863, 0.0612),
            "Green":  (0.1667, 0.6631, 0.1703),
            #"Blue":   (0.0, 0.0, 1.0),
            "Yellow": (0.6077, 0.3580, 0.0343),
        }

    # ------------------------
    # Utility Functions
    # ------------------------

    def normalize(self, rgb):
        """
        normalizes the rgb input - returns none if black
        """
        r, g, b = rgb
        total = r + g + b
        if total <= 0:
            return None
        return (r / total, g / total, b / total)

    def distance(self, a, b):
        """
        a: normalized RGB tuple (r, g, b) (ex: current measurement)
        b: normalized RGB tuple (r, g, b) (ex: reference color)
        returns: Euclidean distance between the two points in RGB space.
                 Smaller distance = more similar color.
        """
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    # ------------------------
    # Main classification
    # ------------------------

    def classify(self, threshold=0.18):
        """
        threshold: max allowed distance to accept a color match.
                   If the closest reference is still farther than threshold => return "Unknown".

        returns: one of {"Red","Green","Blue","Yellow"} or "Unknown"

        Behavior:
        - reads one RGB measurement from the sensor
        - normalizes it
        - finds closest reference color by Euclidean distance
        - applies threshold check
        """
        
        # Get rgb value
        r,g,b = self.sensor.get_rgb()

        # Normalize value
        rgb = self.normalize((r,g,b))
        if rgb is None:
            return "Unknown"

        best_color = "Unknown"
        best_dist = math.inf

        # Check closes color euclidean distance
        for name, ref in self.reference_colors.items():
            d = self.distance(rgb, ref)
            if d < best_dist:
                best_dist = d
                best_color = name

        # Threshold gate
        if best_dist > threshold:
            return "Unknown"

        return best_color