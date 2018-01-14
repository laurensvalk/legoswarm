import sys
import time
import platform
import evdev
import ev3dev.auto as ev3
from collections import deque
from threading import Thread

# Check if code is running on the ev3
def running_on_ev3():
    # Return True if 'ev3' occurs in the platform string
    return 'ev3' in platform.platform()


# Debug print
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def limit(speed, max_speed=650):
    return max(min(max_speed, speed), -max_speed)


def is_within_tolerance(value, target, tolerance=0):
    return target - abs(tolerance) < value < target + abs(tolerance)


def run_to_abs_pos(motor, target, max_speed=650, tolerance=3, speed_p=3, hold=0):
    not_there_yet = True
    while not_there_yet:
        error = target-motor.position
        speed = limit(error * speed_p, max_speed)
        motor.run_forever(speed_sp=speed)
        if -tolerance <= error <= tolerance: not_there_yet = False
        time.sleep(0.04)
    end_time = time.time() + hold
    while time.time() < end_time:
        # Hold the position a while longer
        error = target - motor.position
        motor.run_forever(speed_sp=error)
        time.sleep(0.04)
    motor.stop()

class EZMotor(ev3.Motor):
    """
    Extending/subclassing the ev3dev motor class for ease of use.
    """
    # degree per second. Errors are raised somewhere above this number.
    max_speed_sp = 650

    def __init__(self, port):
        # self.motor = ev3.Motor(port)
        self.port = port
        if running_on_ev3():
            self.running_on_ev3 = True
            ev3.Motor.__init__(self, port)
        else:
            #LATER: virtualize the complete motor in case it is not running on an ev3, using logging.
            ev3.Motor.__init__(self)

    def set_speed(self, speed):
        self.speed_sp = limit(speed, self.max_speed_sp)
        self.run_forever()

    def get_speed(self):
        return self.speed
    
    def get_position(self):
        return self.position

    def go_to(self, reference, speed, tolerance):
        # if not (reference - tolerance <= self.position <= reference + tolerance):
        #     self.position_sp = reference
        #     self.speed_sp = abs(limit(speed))
        #     self.run_to_abs_pos()
        if self.running_on_ev3:
            if not self.is_running:
                motor_thread = Thread(target=run_to_abs_pos(self, reference, speed, tolerance))
                motor_thread.start()
        else:
            print("Running {0} to: {1}".format(self, reference))

    def wait_for_completion(self):
        self.wait_while('running')

    def __del__(self):
        self.stop()


class BallSensor:
    def __init__(self, port=ev3.INPUT_4):
        self.irsensor = ev3.InfraredSensor(port)
        self.threshold = 7

        frequency = 5  # Hz
        self.interval = 1/frequency
        self.next_reading_time = time.time()+self.interval

        self.num_readings = 4  # For averaging over
        self.readings = deque([100] * self.num_readings, maxlen=self.num_readings)

    def check_ball(self):
        if time.time() > self.next_reading_time:
            prox = self.irsensor.proximity
            if prox < 100:
                self.readings.append(prox)
                self.next_reading_time = time.time() + self.interval
        avg_proximity = sum(self.readings) / self.num_readings
        # print(self.readings, avg_proximity)
        if avg_proximity < self.threshold:  # True if a ball is close enough to the sensor.
           self.readings.extend([10]*4)
           return True
        else:
           return False


class Picker:
    """Steer the picker mechanism to the desired target"""

    # Amount of degrees the motor must turn to rotate the gripper by one degree
    motor_deg_per_picker_deg = -3

    # Target positions for the gripper (degrees). 0 corresponds to the gripper all the way open
    OPEN = 40 * motor_deg_per_picker_deg
    CLOSED = OPEN + 90 * motor_deg_per_picker_deg
    STORE = CLOSED + 135 * motor_deg_per_picker_deg
    PURGE = STORE + 45 * motor_deg_per_picker_deg

    # Speed and tolerance parameters
    tolerance = 4 * abs(motor_deg_per_picker_deg)

    def __init__(self, port=ev3.OUTPUT_A, p=2.8):

        # Check if we're running on the EV3
        self.running_on_ev3 = running_on_ev3()

        self.target = self.OPEN
        self.p = p

        # If running on the EV3, perform a reset routine
        if self.running_on_ev3:
            self.pickermotor = EZMotor(port)
            self.pick_at_rate(-40)
            self.pickermotor.wait_until('stalled')
            self.pickermotor.stop()
            self.pickermotor.position = 0
            self.error = 0

    def pick_at_rate(self, rate):
        """Set the picker reference speed"""
        self.pickermotor.set_speed(rate * self.motor_deg_per_picker_deg)

    def get_picking_rate(self):
        """Get the current picker speed"""
        return self.pickermotor.get_speed() / self.motor_deg_per_picker_deg

    @property
    def position(self):
        return self.pickermotor.position

    @property
    def is_at_target(self):
        return is_within_tolerance(self.error, 0, self.tolerance)

    def run(self):
        self.error = self.target - self.pickermotor.position
        speed = self.error * self.p
        self.pickermotor.set_speed(speed)

    def stop(self):
        """Stop the picker motor"""
        self.pickermotor.stop()

    def go_to(self, reference):
        """Steer Picker mechanism to desired target"""
        # If running on the EV3, steer picker to the desired target
        self.pickermotor.go_to(reference,  # Reference position
                               500,  # Speed to get there
                               self.tolerance)# Allowed tolerance

    def open(self):
        self.go_to(self.OPEN)

    def close(self):
        self.go_to(self.CLOSED)

    def store(self):
        self.go_to(self.STORE)

    def purge(self):
        self.go_to(self.PURGE)


class DriveBase:
    """Easily control two large motors to drive a skid steering robot using specified forward speed and turnrate"""
    def __init__(self, left=ev3.OUTPUT_B, right=ev3.OUTPUT_C, wheel_diameter=0.043, wheel_span=0.12):
        """Set up two Large motors"""   
        # Math constants
        deg_per_rad = 180/3.1416

        # Forward speed conversions
        wheel_radius = wheel_diameter/2
        self.wheel_cm_sec_per_deg_s = wheel_radius * 100 / deg_per_rad # cm of forward travel for 1 deg/s wheel rotation

        # Turnrate conversions
        wheel_base_radius = wheel_span/2
        self.wheel_cm_sec_per_base_deg_sec = wheel_base_radius * 100 / deg_per_rad

        # Check if we're running on the EV3
        self.running_on_ev3 = running_on_ev3()

        # Initialize motors if running on the EV3
        if self.running_on_ev3:
            self.left = EZMotor(left)
            self.right = EZMotor(right)

    def drive_and_turn(self, speed_cm_sec, turnrate_deg_sec):
        """Set speed of two motors to attain desired forward speed and turnrate"""
        nett_speed = speed_cm_sec / self.wheel_cm_sec_per_deg_s
        difference = turnrate_deg_sec * self.wheel_cm_sec_per_base_deg_sec / self.wheel_cm_sec_per_deg_s
        if self.running_on_ev3:
            self.left.set_speed(nett_speed - difference)
            self.right.set_speed(nett_speed + difference)
        else:
            print("Setting Drivebase to Speed cm/s: " + str(speed_cm_sec) + ", Turnrate deg/s: " + str(turnrate_deg_sec))

    def stop(self):
        """Stop the robot by setting both motors to zero speed"""
        self.drive_and_turn(0, 0)


class IRRemoteControl:
    """Configures IR Sensor as IR Receiver and reads IR button status"""

    # Ordered list of possible button presses
    button_list = ['NONE','LEFT_UP','LEFT_DOWN','RIGHT_UP','RIGHT_DOWN',
                'BOTH_UP','LEFT_UP_RIGHT_DOWN','LEFT_DOWN_RIGHT_UP','BOTH_DOWN',
                'BEACON', 'BOTH_LEFT','BOTH_RIGHT']

    def __init__(self,port):
        """Configure IR sensor in remote mode"""
        # Check whether we run this on an EV3
        self.running_on_ev3 = running_on_ev3()
        if self.running_on_ev3:
            # If so, configure the sensor and its mode
            self.remote = ev3.InfraredSensor(port)
            self.remote.mode = self.remote.MODE_IR_REMOTE

    def Button(self):
        """Return name (string) of button currently pressed"""
        if self.running_on_ev3:
            # If runnning on the EV3, return name of button currently pressed
            return self.button_list[self.remote.value()]
        else:
            # Always return that no buttons are pressed if not running on the EV3
            return 'NONE'

    def IsPressed(self, button):
        """Check if specified button is currently pressed"""
        if self.running_on_ev3:
            # If runnning on the EV3, return true if specified button currently pressed
            return self.button_list.index(button) == self.remote.value()
        else:
            # Always return that the button is not pressed if not running on the EV3
            return False


## Gamepad ##

class GamePadStub:
    @staticmethod
    def read_loop():
        time.sleep(0.2)
        return []

class PS3GamePad(Thread):
    """
    PS3 Game pad class.
    Optionallyinitialize with a settings dictionary like this one:

    mysettings ={'right_stick_x': {'min_value': 5, 'scale': (-100,100) },
     'right_stick_y': {'min_value': 5, 'scale': (-100, 100) },
     'left_stick_x': {'min_value': 5, 'scale': (-100, 100) },
     'left_stick_y': {'min_value': 5, 'scale': (-100, 100) }
    }

    Usage:
    my_gamepad = PS3GamePad(my_settings)
    print(my_gamepad.right_stick_x)
    """

    UP = 0
    DOWN = 1
    PRESSED = 2
    RELEASED = 3
    BUTTONS = 1
    STICKS = 3

    def __init__(self, settings={}):
        Thread.__init__(self)
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        for device in devices:
            if device.name == 'PLAYSTATION(R)3 Controller':
                ps3dev = device.fn

        try:
            self.gamepad = evdev.InputDevice(ps3dev)
            print("Found ps3 controller")
        except:
            self.gamepad = GamePadStub()
            print("No PS3 gamepad connected")

        self.states = {0:{}, self.BUTTONS:{}, 2:{}, self.STICKS:{}, 4:{}}
        self.settings = settings

        self.running = True
        self.start()
        print("done initializing gamepad")

    def get_button_state(self, code):
        try:
            # Try to look up the keycode in our dictionary of key states
            state = self.states[self.BUTTONS][code]
            if state == self.PRESSED:
                self.states[self.BUTTONS][code] = self.DOWN
            elif state == self.RELEASED:
                self.states[self.BUTTONS][code] = self.UP
        except KeyError:
            # The button doesn't exist or has never been pressed
            state = 0
        # print("Called for state of {0} with value {1}".format(code, state))
        return state

    def get_stick_value(self, code, stick_name):
        try:
            # Try to look up the stick code in our dictionary of  states
            value = self.states[self.STICKS][code]
        except KeyError:
            # The stick doesn't exist or has never been used
            value = 127

        try:
            scale = self.settings[stick_name]['scale']
        except KeyError:
            scale = (-100, 100)
            self.settings[stick_name]['scale'] = scale

        try:
            deadzone = self.settings[stick_name]['deadzone']
        except KeyError:
            deadzone = 5

            self.settings[stick_name]['deadzone'] = deadzone

        print("Returned stick value {0}, {1}, {2}".format(scale, value, deadzone))
        scaled_value = self.scale(value, (255, 0), scale)
        if -deadzone < scaled_value < deadzone:
            scaled_value = 0
        return scaled_value

    def run(self):
        for event in self.gamepad.read_loop():  # this loops infinitely
            # print("Event: event type {0}, even code {1}, event value {2}".format(event.type, event.code, event.value))
            if event.type == self.BUTTONS:
                if event.value == self.DOWN:
                    result = self.PRESSED
                elif event.value == self.UP:
                    result = self.RELEASED
            else:
                result = event.value
            try:
                self.states[event.type][event.code] = result
            except KeyError:
                print("Keyerror: event type {0}, even code {1}, event value {2}".format(event.type,event.code,result))
            if not self.running:
                break

    def __del__(self):
        self.running = False

    @property
    def back_btn(self):
        return self.get_button_state(288)

    @property
    def start_btn(self):
        return self.get_button_state(291)

    @property
    def up_btn(self):
        return self.get_button_state(292)

    @property
    def right_btn(self):
        return self.get_button_state(293)

    @property
    def down_btn(self):
        return self.get_button_state(294)

    @property
    def left_btn(self):
        return self.get_button_state(295)

    @property
    def l2_btn(self):
        return self.get_button_state(296)

    @property
    def r2_btn(self):
        return self.get_button_state(297)

    @property
    def l1_btn(self):
        return self.get_button_state(298)

    @property
    def r1_btn(self):
        return self.get_button_state(299)

    @property
    def triangle_btn(self):
        return self.get_button_state(300)

    @property
    def circle_btn(self):
        return self.get_button_state(301)

    @property
    def cross_btn(self):
        return self.get_button_state(302)

    @property
    def square_btn(self):
        return self.get_button_state(303)

    @property
    def right_stick_x(self):
        return self.get_stick_value(2, "right_stick_x")

    @property
    def right_stick_y(self):
        return self.get_stick_value(5, "right_stick_y")

    @property
    def left_stick_x(self):
        return self.get_stick_value(0, "left_stick_x")

    @property
    def left_stick_y(self):
        return self.get_stick_value(1, "left_stick_y")

    @staticmethod
    def scale(val, src, dst):
        """
        Scale the given value from the scale of src to the scale of dst.

        val: float or int
        src: tuple
        dst: tuple

        example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
        """
        return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]
