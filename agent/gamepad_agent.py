#!/usr/bin/env python3

__author__ = 'anton'

import evdev
import hardware
import threading
import time

MAX_SPEED = -40  # cm per s
MAX_TURNRATE = -60  # deg per s

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


gamepad = evdev.InputDevice(ps3dev)



turn_rate = 0
turn_speed = 0
fwd_speed = 0
triangle_pressed_time = 0
eat = False
running = True

class MotorThread(threading.Thread):
    def __init__(self):
        self.base = hardware.DriveBase()
        self.picker = hardware.Picker()
        self.picker.open()
        threading.Thread.__init__(self)

    def run(self):
        print("Engines running!")
        while running:
            self.base.drive_and_turn(fwd_speed, turn_rate)
            if eat:
                self.picker.store()
            else:
                self.picker.open()
            time.sleep(0.03)

        self.base.stop()


if __name__ == "__main__":
    motor_thread = MotorThread()
    motor_thread.setDaemon(True)
    motor_thread.start()

    for event in gamepad.read_loop(): #this loops infinitely
        if event.type == 3: #A stick is moved

            if event.code == 2: #X axis on right stick
                turn_rate = scale(event.value,(0,255), (-MAX_TURNRATE, MAX_TURNRATE))

            if event.code == 5: #Y axis on right stick
                fwd_speed = scale(event.value, (0,255), (-MAX_SPEED, MAX_SPEED))

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
                    eat = True
                if event.value == 0:
                    eat = False

        # Flush
        while gamepad.read_one() != None:
            pass
