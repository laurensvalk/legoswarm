import sys
import time
import platform

# Check if code is running on the ev3
def running_on_ev3():
    # Return True if 'ev3' occurs in the platform string
    return 'ev3' in platform.platform()

# Import ev3dev only if we're running on the EV3
if running_on_ev3():
    import ev3dev.ev3 as ev3

# Debug print
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# TODO: This class (and whole module) needs better documentation
class Motor:

    max_speed_sp = 500 # degree per second

    def __init__(self, port):
        self.motor = ev3.Motor(port)

    def SetSpeed(self, speed):
        self.motor.speed_sp = self.limit(speed)
        self.motor.run_forever()

    def GetSpeed(self):
        return self.motor.speed
    
    def GetPosition(self):
        return self.motor.position

    def Stop(self):
        self.SetSpeed(0)    

    def limit(self, speed):
        return max(min(self.max_speed_sp, speed), -self.max_speed_sp)       

    def Goto(self, reference, speed, tolerance):
        if not self.Running() and not (reference - tolerance <= self.GetPosition() <= reference + tolerance):
            self.motor.position_sp = reference
            self.motor.speed_sp = abs(self.limit(speed))
            self.motor.run_to_abs_pos()

    def Running(self):
        return 'running' in self.motor.state

    def WaitForCompletion(self):
        while self.Running():
            time.sleep(0.01)

    def __del__(self):
        self.Stop()    

class Picker:
    """Steer the picker mechanism to the desired target"""

    # Target positions for the gripper (degrees). 0 corresponds to the gripper all the way open
    target_open = 40
    target_closed = 130
    target_store = 255
    target_purge = 295

    # Speed and tolerance parameters
    abs_speed = 120
    tolerance = 4

    def __init__(self,port):   

        # Check if we're running on the EV3
        self.running_on_ev3 = running_on_ev3()
        
        # Amount of degrees the motor must turn to rotate the gripper by one degree
        self.motor_deg_per_picker_deg = -3

        # If running on the EV3, perform a reset routine
        if self.running_on_ev3:
            self.pickermotor = Motor(port)
            self.__SetPickRate(-40)
            time.sleep(0.5)

            while self.__GetPickRate() < -10:
                pass
            self.__Stop()
            self.pickermotor.motor.reset()

    def __SetPickRate(self,rate):
        """Set the picker reference speed"""
        self.pickermotor.SetSpeed(rate * self.motor_deg_per_picker_deg)

    def __GetPickRate(self):
        """Get the current picker speed"""
        return self.pickermotor.GetSpeed()/self.motor_deg_per_picker_deg

    def __Stop(self):
        """Stop the picker motor"""
        self.__SetPickRate(0)

    def Goto(self, reference):
        """Steer Picker mechanism to desired target"""
        # If running on the EV3, steer picker to the desired target
        if self.running_on_ev3:
            self.pickermotor.Goto(reference*self.motor_deg_per_picker_deg,          # Reference position
                                  self.abs_speed*self.motor_deg_per_picker_deg,     # Speed to get there
                                  abs(self.tolerance*self.motor_deg_per_picker_deg))# Allowed tolerance
        else:
            print("Turning the gripper to: " + str(reference))
        

class DriveBase:
    """Easily control two large motors to drive a skid steering robot using specified forward speed and turnrate"""
    def __init__(self,left,right,wheel_diameter,wheel_span):   
        """Set up two Large motors"""   
        # Math constants
        deg_per_rad = 180/3.1416

        # Forward speed conversions
        wheel_radius = wheel_diameter/2
        self.wheel_cm_sec_per_deg_s = wheel_radius * 100 / deg_per_rad # cm of forward travel for 1 deg/s wheel rotation

        # Turnrate conversions
        wheel_base_radius = wheel_span/2
        self.wheel_cm_sec_per_base_deg_sec =  wheel_base_radius * 100 / deg_per_rad

        # Check if we're running on the EV3
        self.running_on_ev3 = running_on_ev3()

        # Initialize motors if running on the EV3
        if self.running_on_ev3:
            self.left = Motor(left)
            self.right = Motor(right)

    def DriveAndTurn(self, speed_cm_sec, turnrate_deg_sec):
        """Set speed of two motors to attain desired forward speed and turnrate"""
        nett_speed = speed_cm_sec / self.wheel_cm_sec_per_deg_s
        difference = turnrate_deg_sec * self.wheel_cm_sec_per_base_deg_sec / self.wheel_cm_sec_per_deg_s
        if self.running_on_ev3:
            self.left.SetSpeed(nett_speed - difference)
            self.right.SetSpeed(nett_speed + difference)
        else:
            print("Setting Drivebase to Speed cm/s: " + str(speed_cm_sec) + ", Turnrate deg/s: " + str(turnrate_deg_sec))

    def Stop(self):
        """Stop the robot by setting both motors to zero speed"""
        self.DriveAndTurn(0,0)

class RemoteControl:
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
