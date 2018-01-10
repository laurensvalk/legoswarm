#!/usr/bin/env python3
from hardware import Picker, DriveBase, IRRemoteControl, eprint, BallSensor
from collections import defaultdict
import time

IR_REMOTE = False

# Configure the devices
if IR_REMOTE:
    remote = IRRemoteControl('in4')
    # Dictionary of action tuples associated with remote button
    actions = {
        'NONE': (0, 0, None),
        'LEFT_UP': (0, 40, None),
        'RIGHT_UP': (0, -40, None),
        'BOTH_UP': (6, 0, None),
        'BOTH_DOWN': (-6, 0, None),
        'LEFT_DOWN': (None, None, Picker.target_store),
        'RIGHT_DOWN': (None, None, Picker.target_open),
        'BEACON': (None, None, Picker.target_purge)
    }
    # Make unused buttons the same as no buttons case
    actions = defaultdict(lambda: actions['NONE'], actions)
else:
    ball_sensor = BallSensor()

base = DriveBase(left='outB', right='outC', wheel_diameter=0.043, wheel_span = 0.12)
picker = Picker('outA')

# Main Loop: Drive around and control picker based on remote
while True:
    if IR_REMOTE:
        speed_now, steering_now, target_now = actions[remote.Button()]

        if target_now is not None:
            picker.go_to(target_now)
        if speed_now is not None and steering_now is not None:
            base.drive_and_turn(speed_now, steering_now)
    else:
        ball_sensor.check_ball()
        time.sleep(0.04)