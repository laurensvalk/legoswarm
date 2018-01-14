#!/usr/bin/env python3

__author__ = 'anton'

import hardware
import time

MAX_SPEED = 40  # cm per s
MIN_SPEED = 3
MAX_TURNRATE = 80  # deg per s
MIN_TURNRATE = 4


turn_rate = 0
turn_speed = 0
fwd_speed = 0
triangle_pressed_time = 0
picker = hardware.Picker()
gamepad_settings = {'left_stick_x': {'min_value': 3, 'scale': (-40, 40) },
                    'left_stick_y': {'min_value': 4, 'scale': (-80, 80) }}
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
    turn_rate = gamepad.left_stick_x
    if gamepad.cross:
        picker.target = picker.STORE

    picker.run()
    base.drive_and_turn(fwd_speed, turn_rate)

    # Give the Ev3 some time to handle other threads.
    time.sleep(0.04)

base.stop()
picker.stop()