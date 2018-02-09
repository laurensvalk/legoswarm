import time
from .simple_device import Motor, eprint

class Picker(Motor):
    """Steer the picker mechanism to the desired target"""
    # Target positions for the gripper (degrees). 0 corresponds to the gripper all the way open
    OPEN = 90
    CLOSED = 130
    STORE = 270
    PURGE = 287

    # Speed and tolerance parameters
    abs_speed = 150
    tolerance = 4

    # # Amount of degrees the motor must turn to rotate the gripper by one degree
    motor_deg_per_picker_deg = 3
    store_count = 0

    def __init__(self, port):   
        # Initialize motor
        Motor.__init__(self, port)
        self.polarity = self.POLARITY_INVERSED

        # Do a reset routine
        self.pick_rate = -40
        time.sleep(0.5)
        start_time = time.time()
        while self.pick_rate < -10 and time.time() - start_time < 10:
            time.sleep(0.1)
        self.stop()
        self.reset()
        self.polarity = self.POLARITY_INVERSED
        self.go_to_target(self.OPEN, blocking=True)

    @property
    def beak_position(self):
        return self.position/self.motor_deg_per_picker_deg

    def is_within_tolerance(self, target):
        return self.at_target(target*self.motor_deg_per_picker_deg, self.tolerance*self.motor_deg_per_picker_deg)

    @property
    def is_open(self):
        return self.is_within_tolerance(self.OPEN)

    @property
    def is_at_store(self):
        return self.is_within_tolerance(self.STORE)

    @property
    def pick_rate(self):
        """Get the current picker speed"""
        return self.speed/self.motor_deg_per_picker_deg

    @pick_rate.setter
    def pick_rate(self, rate):
        """Set the picker reference speed"""
        self.run_forever_at_speed(rate * self.motor_deg_per_picker_deg)

    def store(self, blocking=True):
        self.go_to_target(self.STORE, blocking)
        self.store_count += 1

    def purge(self, blocking=True):
        self.go_to_target(self.PURGE, blocking)
        self.store_count = 0

    def open(self, blocking=False):
        self.go_to_target(self.OPEN, blocking)

    def go_to_target(self, target, blocking=False):
        """Steer Picker mechanism to desired target"""
        self.go_to(target*self.motor_deg_per_picker_deg,             # Reference position
                   self.abs_speed*self.motor_deg_per_picker_deg,     # Speed to get there
                   self.tolerance*self.motor_deg_per_picker_deg,# Allowed tolerance
                   blocking)

class DriveBase:
    """Easily control two large motors to drive a skid steering robot using specified forward speed and turnrate"""

    POLARITY_INVERSED = Motor.POLARITY_INVERSED
    POLARITY_NORMAL = Motor.POLARITY_INVERSED

    def __init__(self, left, right, wheel_diameter, wheel_span, counter_clockwise_is_positive=True, max_speed=5):
        """Set up two Large motors and predefine conversion constants"""   
        self.max_speed = max_speed

        # Store which is the positive direction
        self.counter_clockwise_is_positive = counter_clockwise_is_positive

        # Math constants
        deg_per_rad = 180/3.1416

        #Compute radii
        wheel_radius = wheel_diameter/2
        wheel_base_radius = wheel_span/2

        self.wheel_span = wheel_span
        self.wheel_diameter = wheel_diameter

        # cm of forward travel for 1 deg/s wheel rotation
        self.wheel_cm_sec_per_deg_s = wheel_radius / deg_per_rad 
        # wheel speed for a given rotation of the base
        self.wheel_cm_sec_per_base_deg_sec =  wheel_base_radius / deg_per_rad

        # Initialize left motor
        left_name, left_polarity = left      
        self.leftmotor = Motor(left_name)
        self.leftmotor.polarity = left_polarity
        
        # Initialize right motor
        right_name, right_polarity = right
        self.rightmotor = Motor(right_name)
        self.rightmotor.polarity = right_polarity

    def drive_and_turn(self, speed_cm_sec, turnrate_deg_sec):
        """Set speed of two motors to attain desired forward speed and turnrate"""
        # Wheel speed for given forward rate
        nett_speed = min(self.max_speed, speed_cm_sec) / self.wheel_cm_sec_per_deg_s

        # Wheel speed for given turnrate
        difference = turnrate_deg_sec * self.wheel_cm_sec_per_base_deg_sec / self.wheel_cm_sec_per_deg_s

        # Depending on sign of turnrate, go left or right
        if self.counter_clockwise_is_positive:
            leftspeed = nett_speed - difference
            rightspeed = nett_speed + difference
        else:
            leftspeed = nett_speed + difference
            rightspeed = nett_speed - difference            

        # Apply the calculated speeds to the motor
        self.leftmotor.run_forever_at_speed(leftspeed)
        self.rightmotor.run_forever_at_speed(rightspeed)

    def turn_degrees(self, degrees, turnrate=200 blocking=True):
        self.stop()
        wheel_degrees = int(degrees * self.wheel_span / self.wheel_diameter)
        if self.counter_clockwise_is_positive:
            self.leftmotor.go_to(self.leftmotor.position + wheel_degrees, turnrate, 2, blocking=False)
            self.rightmotor.go_to(self.rightmotor.position - wheel_degrees, turnrate, 2, blocking)
        else:
            self.leftmotor.go_to(self.leftmotor.position - wheel_degrees, turnrate, 2, blocking=False)
            self.rightmotor.go_to(self.rightmotor.position + wheel_degrees, turnrate, 2, blocking)

    def drive_cm(self, cm, speed=200, blocking=True):
        wheel_degrees = int(cm * 360 / (3.1415 * self.wheel_diameter))
        self.stop()
        self.leftmotor.go_to(self.leftmotor.position + wheel_degrees, speed, 2, blocking=False)
        self.rightmotor.go_to(self.rightmotor.position + wheel_degrees, speed, 2, blocking)

    def stop(self):
        """Stop the robot"""
        # Stop robot by stopping motors
        self.leftmotor.stop()
        self.rightmotor.stop()