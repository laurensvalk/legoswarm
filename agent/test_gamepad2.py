#!/usr/bin/env python3

__author__ = 'anton'

from hardware.gamepad import PS3GamePad
from hardware.motors import DriveBase, Picker
from hardware.sensors import BallSensor
import time

turn_rate = 0
turn_speed = 0
fwd_speed = 0
triangle_pressed_time = 0
picker = Picker('outA')
gamepad_settings = {'left_stick_x': {'min_value': 3, 'scale': (40, -40) },
                    'left_stick_y': {'min_value': 4, 'scale': (-60, 60) }}
gamepad = PS3GamePad(gamepad_settings)
base = DriveBase(left=('outC', 'inversed'),
                 right=('outB', 'inversed'),
                 wheel_diameter=4.3,
                 wheel_span=12,
                 counter_clockwise_is_positive=False)
ballsensor = BallSensor('in4')

print("Engines running!")
while True:
    # Autopicker
    if ballsensor.check_ball() and not picker.is_running:
        picker.go_to_target(picker.STORE)

    # Close after storing
    if picker.is_at_store:
        picker.go_to_target(picker.OPEN)

    # Gamepad
    fwd_speed = gamepad.left_stick_y
    print(fwd_speed)
    turn_rate = gamepad.left_stick_x
    if gamepad.cross_btn:
        picker.go_to_target(picker.STORE)
    if gamepad.square_btn:
        break
    picker.run()
    base.drive_and_turn(fwd_speed, turn_rate)

    # Give the Ev3 some time to handle other threads.
    time.sleep(0.04)

base.stop()
picker.stop()
