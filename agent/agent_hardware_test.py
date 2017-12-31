#!/usr/bin/env python3
from hardware import Picker, DriveBase, RemoteControl, eprint
from collections import defaultdict
import time

# Configure the devices
remote = RemoteControl('in4')
base = DriveBase(left='outB',right='outC',wheel_diameter=0.043,wheel_span = 0.12)
picker = Picker('outA')

# Dictionary of action tuples associated with remote button
actions = {
    'NONE' : (0,0,None),
    'LEFT_UP' : (0,40,None),
    'RIGHT_UP' : (0,-40,None),
    'BOTH_UP' : (6,0,None),
    'BOTH_DOWN' : (-6,0,None),
    'LEFT_DOWN' : (None, None, Picker.target_store),
    'RIGHT_DOWN' : (None, None, Picker.target_open),
    'BEACON' : (None, None, Picker.target_purge)    
}
# Make unused buttons the same as no buttons case
actions = defaultdict(lambda: actions['NONE'], actions)

# Main Loop: Drive around and control picker based on remote
while True:
    speed_now, steering_now, target_now = actions[remote.Button()]

    if target_now is not None:
        picker.Goto(target_now)
    if speed_now is not None and steering_now is not None:
        base.DriveAndTurn(speed_now,steering_now)
    time.sleep(0.1)
