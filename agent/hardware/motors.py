import time
from .compat_device import CompatMotor


class Motor(CompatMotor):
    """Add a few convenience functions to standard ev3dev motor class"""
    def run_forever_at_speed(self, speed):
        self.speed_sp = self.limit(speed)
        self.run_forever() 

    def limit(self, speed):
        # Ensure we bind the speed to the known maximum for this motor
        return max(min(self.max_speed, speed), -self.max_speed)

    def go_to(self, reference, speed, tolerance):
        if not self.is_running and not (reference - tolerance <= self.position <= reference + tolerance):
            self.position_sp = reference
            self.speed_sp = abs(self.limit(speed))
            self.run_to_abs_pos()

class Picker(Motor):
    """Steer the picker mechanism to the desired target"""
    # Target positions for the gripper (degrees). 0 corresponds to the gripper all the way open
    OPEN = 40
    CLOSED = 130
    STORE = 255
    PURGE = 287

    # Speed and tolerance parameters
    abs_speed = 120
    tolerance = 4

    # # Amount of degrees the motor must turn to rotate the gripper by one degree
    motor_deg_per_picker_deg = -3 

    def __init__(self, port='outA'):
        # Initialize motor
        Motor.__init__(self, port)

        # Do a reset routine
        self.pick_rate = -40
        time.sleep(0.5)
        start_time = time.time()
        while self.pick_rate < -10 and time.time() - start_time < 10:
            time.sleep(0.1)
        self.stop()
        self.reset()
        self.target = self.OPEN

    @property
    def pick_rate(self):
        """Get the current picker speed"""
        return self.speed/self.motor_deg_per_picker_deg

    @pick_rate.setter
    def pick_rate(self, rate):
        """Set the picker reference speed"""
        self.run_forever_at_speed(rate * self.motor_deg_per_picker_deg)

    def go_to_target(self, target):
        """Steer Picker mechanism to desired target"""
        self.go_to(target*self.motor_deg_per_picker_deg,             # Reference position
                   self.abs_speed*self.motor_deg_per_picker_deg,     # Speed to get there
                   abs(self.tolerance*self.motor_deg_per_picker_deg))# Allowed tolerance
              
class DriveBase:
    """Easily control two large motors to drive a skid steering robot using specified forward speed and turnrate"""
    def __init__(self, left, right, wheel_diameter, wheel_span, reverse_motors = True, counter_clockwise_is_positive=True):
        """Set up two Large motors and predefine conversion constants"""   
        
        # Store which is the positive direction
        self.counter_clockwise_is_positive = counter_clockwise_is_positive

        # Math constants
        deg_per_rad = 180/3.1416

        #Compute radii
        wheel_radius = wheel_diameter/2
        wheel_base_radius = wheel_span/2
        
        # cm of forward travel for 1 deg/s wheel rotation
        self.wheel_cm_sec_per_deg_s = wheel_radius / deg_per_rad 
        # wheel speed for a given rotation of the base
        self.wheel_cm_sec_per_base_deg_sec =  wheel_base_radius / deg_per_rad

        # Initialize left motor
        self.leftmotor = Motor(left)
        
        # Initialize right motor
        self.rightmotor = Motor(right)

        if reverse_motors:
            self.leftmotor.polarity = 'inversed'
            self.rightmotor.polarity = 'inversed'

    def drive_and_turn(self, speed_cm_sec, turnrate_deg_sec):
        """Set speed of two motors to attain desired forward speed and turnrate"""
        # Wheel speed for given forward rate
        nett_speed = speed_cm_sec / self.wheel_cm_sec_per_deg_s

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
        
    def stop(self):
        """Stop the robot"""
        # Stop robot by stopping motors
        self.leftmotor.stop()
        self.rightmotor.stop()
