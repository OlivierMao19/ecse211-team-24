"""
Drumming system: 
- Press button to start drumming - motor moving back and forth.
- While drumming, pressing button again captures color and plays a note (1 note = 1 color).
- Emergency stop button stops everything.
"""

import threading
from utils import sound
from utils.brick import TouchSensor, EV3ColorSensor, Motor, wait_ready_sensors
from color_classifier import ColorClassifier
import time

# Setup
DRUM_MOTOR = Motor("A")  # TO CHANGE to actual value..
TOUCH_SENSOR = TouchSensor(1)
COLOR_SENSOR = EV3ColorSensor(2)
EMERGENCY_SENSOR = TouchSensor(3) # idk if that exists on the brickpi.. to verify

DRUM_MOTOR.set_limits(power=60, dps=600) # Speed limit

NOTES = {
    "Red": "C4",
    "Green": "D4",
    "Purple": "E4",
    "Yellow": "F4"
}

# Instantiate color classifier
COLOR_CLASSIFIER = ColorClassifier(COLOR_SENSOR)


def drum_loop(stop_event):
    """Continuously move drum back and forth until stopped."""
    while not stop_event.is_set():
        DRUM_MOTOR.set_position_relative(45)
        DRUM_MOTOR.wait_is_stopped()
        if stop_event.is_set():
            break
def detect_color_and_play_note():
    # Match a color from sensor measurement
    color = COLOR_CLASSIFIER.classify()
    print(color)
    note = NOTES.get(color, None)
    if note:
        s = sound.Sound(duration=0.3, pitch=note, volume=80)
        s.play()
        s.wait_done()
    else:
        return

def main():
    wait_ready_sensors()
    drumming = False
    stop_event = threading.Event()

    print("Press button to start drumminggg")

    while True:
        # Emergency stop if clicked
        if EMERGENCY_SENSOR.is_pressed():
            print("EMERGENCY STOP ACTIVATED")
            stop_event.set()
            break

        # Main button logic 
        if TOUCH_SENSOR.is_pressed():

            if not drumming:
                print("Drumming started. Press button again to detect color and play note.")
                drumming = True
                stop_event.clear()
                # Start the drumming in a background thread (btw - daemon = closes with main program)
                threading.Thread(target=drum_loop, args=(stop_event,), daemon=True).start()
                time.sleep(0.5) # debounce
            else:
                print("Detecting color and playing note.")
                detect_color_and_play_note()
                time.sleep(0.5) # debounce
                
        time.sleep(0.1)

if __name__ == "__main__":
    main()