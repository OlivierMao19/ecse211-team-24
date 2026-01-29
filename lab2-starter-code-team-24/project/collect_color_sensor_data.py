#!/usr/bin/env python3

"""
This test is used to collect data from the color sensor.
It must be run on the robot.
"""

# Add your imports here, if any
from utils.brick import EV3ColorSensor, wait_ready_sensors, TouchSensor

COLOR_SENSOR_DATA_FILE = "../data_analysis/color_sensor-red.csv"

# complete this based on your hardware setup
COLOR_SENSOR = EV3ColorSensor(2)
TOUCH_SENSOR = TouchSensor(1)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.


def collect_color_sensor_data():
    "Collect color sensor data."
    try:
        print("Press touch sensor to record one RGB sample.")
        while True:
            if TOUCH_SENSOR.is_pressed():
                rgb = COLOR_SENSOR.get_rgb()
                if rgb is not None:
                    r, g, b = rgb
                    print(f"{r},{g},{b}")
                    
                    with open(COLOR_SENSOR_DATA_FILE, "a") as f:
                        f.write(f"{r},{g},{b}\n")
                # wait for release to avoid multiple samples per press
                while TOUCH_SENSOR.is_pressed():
                    pass
                
    except BaseException:
        exit()


if __name__ == "__main__":
    collect_color_sensor_data()
