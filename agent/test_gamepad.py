#!/usr/bin/env python3

# TODO: REVISE USING NEW HARDWARE CLASSES
__author__ = 'anton'

import evdev
import hardware.old_anton_version as hardware
import threading
import time

MAX_SPEED = 40  # cm per s
MIN_SPEED = 3
MAX_TURNRATE = 80  # deg per s
MIN_TURNRATE = 4


def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.

    val: float or int
    src: tuple
    dst: tuple

    example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

print("Finding ps3 controller...")
devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
for device in devices:
    if device.name == 'PLAYSTATION(R)3 Controller':
        ps3dev = device.fn

try:
    gamepad = evdev.InputDevice(ps3dev)
except:
    gamepad = hardware.GamePadStub()

turn_rate = 0
turn_speed = 0
fwd_speed = 0
triangle_pressed_time = 0
running = True
picker = hardware.Picker()


class MotorThread(threading.Thread):
    def __init__(self):
        self.base = hardware.DriveBase()
        self.ballsensor = hardware.BallSensor()
        threading.Thread.__init__(self)

    def run(self):
        global gripper
        print("Engines running!")
        while running:
            # Autopicker
            if picker.target == picker.OPEN and picker.is_at_target:
                if self.ballsensor.check_ball():
                    gripper.target = picker.CLOSED
            elif picker.target == picker.STORE and picker.is_at_target:
                picker.target = picker.OPEN

            picker.run()
            self.base.drive_and_turn(fwd_speed, -turn_rate)

            # Give the Ev3 some time to handle other threads.
            time.sleep(0.04)
        self.base.stop()
        picker.stop()


if __name__ == "__main__":
    motor_thread = MotorThread()
    motor_thread.start()

    for event in gamepad.read_loop(): #this loops infinitely
        if event.type == 3: #A stick is moved

            if event.code == 2: #X axis on right stick
                turn_rate = scale(event.value, (255, 0), (-MAX_TURNRATE, MAX_TURNRATE))
                if -MIN_TURNRATE < turn_rate < MIN_TURNRATE: turn_rate = 0


            if event.code == 5: #Y axis on right stick
                fwd_speed = scale(event.value, (255, 0), (-MAX_SPEED, MAX_SPEED))
                if -MIN_SPEED < fwd_speed < MIN_SPEED: fwd_speed = 0

        if event.type == 1:
            if event.code == 300:
                if event.value == 1:
                    triangle_pressed_time = time.time()
                if event.value == 0 and time.time() > triangle_pressed_time + 1:
                    print("Triangle button is pressed. Break.")
                    running = False
                    time.sleep(0.5) # Wait for the motor thread to finish
                    break
            elif event.code == 302:
                if event.value == 1:
                    print("X button is pressed. Eating.")
                    picker.target = picker.STORE
                if event.value == 0:
                    picker.target = picker.OPEN
            elif event.code == 301:
                if event.value == 1:
                    print("O button is pressed. Purging.")
                    picker.target = picker.PURGE
                if event.value == 0:
                    picker.target = picker.OPEN
