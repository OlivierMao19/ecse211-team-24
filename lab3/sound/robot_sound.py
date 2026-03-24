from utils.sound import Sound


class RobotSound:
    def __init__(self):
        self.green_detected_sound = Sound(duration=0.2, pitch="A4", volume=70)

    def play_green_detected(self):
        self.green_detected_sound.play()
        self.green_detected_sound.wait_done()
