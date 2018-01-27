from hardware.sensors import BallSensor
from threading import Thread
import logging
import time

class BallSensorReader(Thread):
    def __init__(self):
        self.ball_sensor = BallSensor('in4')
        self.detected = False
        self.running = True
        Thread.__init__(self)

    def ball_detected(self):
        return self.detected

    def run(self):
        logging.debug("Started ballsensor thread")
        while self.running:
            self.detected = self.ball_sensor.ball_detected()
            time.sleep(0.1)

    def stop(self):
        logging.debug("Stopping thread")
        self.running = False