#!/usr/bin/env python3
from hardware.motors import DriveBase, Picker
from hardware.sensors import BallSensor
import time

from os import devnull
from subprocess import Popen

def beep():
    with open('/dev/null', 'w') as file:
        return Popen('/usr/bin/beep', stdout=file)

# Configure the devices
eyes = BallSensor('in4')
base = DriveBase(left=('outC', DriveBase.POLARITY_INVERSED),
                 right=('outB', DriveBase.POLARITY_INVERSED),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False) 
picker = Picker('outA')

# Make a sound when we detect a ball and print readings
while True:
    if eyes.ball_detected():
        beep()
    print(eyes.most_recent_value)
    time.sleep(0.1)
        