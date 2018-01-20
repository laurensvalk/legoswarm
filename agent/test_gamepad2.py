#!/usr/bin/env python3

# TODO: REVISE USING NEW HARDWARE CLASSES

__author__ = 'anton'

import hardware.old_anton_version as hardware
import time

turn_rate = 0
turn_speed = 0
fwd_speed = 0
triangle_pressed_time = 0
picker = hardware.Picker()
gamepad_settings = {'left_stick_x': {'min_value': 3, 'scale': (40, -40) },
                    'left_stick_y': {'min_value': 4, 'scale': (-60, 60) }}
gamepad = hardware.PS3GamePad(gamepad_settings)
base = hardware.DriveBase()
ballsensor = hardware.BallSensor()

print("Engines running!")
while True:
    # Autopicker
    if picker.target == picker.OPEN and picker.is_at_target:
        if ballsensor.check_ball():
            picker.target = picker.CLOSED
    elif picker.target == picker.STORE and picker.is_at_target:
        picker.target = picker.OPEN

    # Gamepad
    fwd_speed = gamepad.left_stick_y
    print(fwd_speed)
    turn_rate = gamepad.left_stick_x
    if gamepad.cross_btn:
        picker.target = picker.STORE
    if gamepad.square_btn:
        break
    picker.run()
    base.drive_and_turn(fwd_speed, turn_rate)

    # Give the Ev3 some time to handle other threads.
    time.sleep(0.04)

base.stop()
picker.stop()
