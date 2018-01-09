import sys
import time
import platform
import ev3dev.auto as ev3
from collections import deque
from threading import Thread

# Check if code is running on the ev3
def running_on_ev3():
    # Return True if 'ev3' occurs in the platform string
    return 'ev3' in platform.platform()

# Import ev3dev only if we're running on the EV3
# if running_on_ev3():
#     import ev3dev.ev3 as ev3


# Debug print
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def limit(speed, max_speed=650):
    return max(min(max_speed, speed), -max_speed)

def is_within_tolerance(value, target, tolerance=0):
    return target - tolerance < value < target + tolerance


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

    # def Stop(self):
    #     self.SetSpeed(0)

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
        # self.last_reading_t = time.time()
        # self.last_prox = self.last_good_prox = 100
        # self.MAX_RATE = 10 # IR % increase per second.
        self.readings = deque([100]*5)

    def check_ball(self):
        # elapsed = time.time() - self.last_reading_t
        prox = self.irsensor.proximity
        self.readings.append(prox)
        # rate = (prox-self.last_prox)/elapsed
        # self.last_prox = prox
        avg_prox = sum(self.readings)/5
        print(prox, avg_prox)
        # if abs(rate) < self.MAX_RATE:
        #     self.last_good_prox = prox
        #     self.last_reading_t = time.time()
        #     return prox < self.threshold
        # else:
        #     return self.last_good_prox < self.threshold
        return avg_prox < self.threshold

class Picker:
    """Steer the picker mechanism to the desired target"""

    # Amount of degrees the motor must turn to rotate the gripper by one degree
    motor_deg_per_picker_deg = -3

    # Target positions for the gripper (degrees). 0 corresponds to the gripper all the way open
    target_open = 40 * motor_deg_per_picker_deg
    target_closed = target_open + 90 * motor_deg_per_picker_deg
    target_store = target_closed + 145 * motor_deg_per_picker_deg
    target_purge = target_store + 45 * motor_deg_per_picker_deg

    # Speed and tolerance parameters
    abs_speed = 400
    tolerance = 4 * abs(motor_deg_per_picker_deg)

    def __init__(self, port=ev3.OUTPUT_A, p=2):

        # Check if we're running on the EV3
        self.running_on_ev3 = running_on_ev3()
        

        self.target = self.target_open
        self.p = p

        # If running on the EV3, perform a reset routine
        if self.running_on_ev3:
            self.pickermotor = EZMotor(port)
            self.pick_at_rate(-40)
            # time.sleep(0.5)
            #
            # while self.get_picking_rate() < -10:
            #     time.sleep(0.02)
            self.pickermotor.wait_until('stalled')
            self.pickermotor.stop()
            self.pickermotor.position = 0

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
    def state(self):
        pos = self.pickermotor.position

        if is_within_tolerance(pos, self.target_open, self.tolerance):
            return 'open'
        elif is_within_tolerance(pos, self.target_purge, self.tolerance):
            return 'purge'
        elif is_within_tolerance(pos, self.target_store, self.tolerance):
            return 'store'
        else:
            return 'running'

    def run(self):
        error = self.target - self.pickermotor.position
        speed = error * self.p
        self.pickermotor.set_speed(speed)

    def stop(self):
        """Stop the picker motor"""
        self.pickermotor.stop()

    def go_to(self, reference):
        """Steer Picker mechanism to desired target"""
        # If running on the EV3, steer picker to the desired target
        self.pickermotor.go_to(reference,  # Reference position
                                   500,  # Speed to get there
                                   abs(self.tolerance))# Allowed tolerance

    def open(self):
        self.go_to(self.target_open)

    def close(self):
        self.go_to(self.target_closed)

    def store(self):
        self.go_to(self.target_store)

    def purge(self):
        self.go_to(self.target_purge)


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
