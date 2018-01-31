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

    @property
    def distance(self):
        return self.ball_sensor.avg_distance()

    def run(self):
        logging.debug("Started ballsensor thread")
        while self.running:
            self.detected = self.ball_sensor.ball_detected()
            time.sleep(0.2)

    def stop(self):
        self.running = False
        time.sleep(0.2)
        logging.debug("Stopped thread")