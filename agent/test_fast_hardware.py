#!/usr/bin/env micropython
from hardware.simple_device import Motor, Sensor, eprint
import time
from hardware.motors import DriveBase, Picker

base = DriveBase(left=('outC', Motor.POLARITY_INVERSED),
                 right=('outB', Motor.POLARITY_INVERSED),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False)
picker = Picker('outA')                 

base.drive_and_turn(speed_cm_sec=5, turnrate_deg_sec=30)
time.sleep(1)
base.stop() 
time.sleep(1)                
picker.go_to_target(Picker.STORE, blocking=True)
picker.go_to_target(Picker.OPEN, blocking=True)