from .simple_device import InfraredSensor
import time
from collections import deque


class BallSensor(InfraredSensor):
    def __init__(self, port):
        self.threshold = 6

        frequency = 5  # Hz
        self.interval = 1/frequency
        self.next_reading_time = time.time()+self.interval

        self.num_readings = 1  # For averaging over
        self.readings = deque([100] * self.num_readings, maxlen=self.num_readings)

        InfraredSensor.__init__(self, port)
        self.mode = self.MODE_IR_PROX

    @ property
    def most_recent_value(self):
        return self.readings[-1]

    def avg_distance(self):
        return sum(self.readings) / self.num_readings

    def ball_detected(self):
        now = time.time() 
        if now > self.next_reading_time:
            distance = self.proximity
            if distance < 100:
                self.readings.append(distance)
                self.next_reading_time = now + self.interval

        if self.avg_distance() < self.threshold:  # True if a ball is close enough to the sensor.
            self.readings.extend([10]*self.num_readings)
            return True
        else:
            return False


class RemoteControl():
    """Configures IR Sensor as IR Receiver and reads IR button status"""

    # Ordered list of possible button presses
    button_list = ['NONE','LEFT_UP','LEFT_DOWN','RIGHT_UP','RIGHT_DOWN',
                'BOTH_UP','LEFT_UP_RIGHT_DOWN','LEFT_DOWN_RIGHT_UP','BOTH_DOWN',
                'BEACON', 'BOTH_LEFT','BOTH_RIGHT']

    def __init__(self, port):
        """Configure IR sensor in remote mode"""
        self.sensor = InfraredSensor(port)
        self.sensor.mode = self.sensor.MODE_IR_REMOTE        

    @property
    def button(self):
        """Return name (string) of button currently pressed"""
        return self.button_list[self.sensor.value]

    def is_pressed(self, button):
        """Check if specified button is currently pressed"""
        return self.button_list.index(button) == self.sensor.value
