import ev3dev.ev3 as ev3
import sys
import time

# Debug print
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

class Motor:
    def __init__(self,port):
        self.motor = ev3.Motor(port)

    def SetSpeed(self,speed):
        self.motor.speed_sp = speed
        self.motor.run_forever()

    def GetSpeed(self):
        return self.motor.speed
    
    def GetPosition(self):
        return self.motor.position

    def Stop(self):
        self.SetSpeed(0)       

    def Goto(self, reference, speed, tolerance):
        if not self.Running() and not (reference - tolerance <= self.GetPosition() <= reference + tolerance):
            self.motor.position_sp = reference
            self.motor.speed_sp = abs(speed)
            self.motor.run_to_abs_pos()

    def Running(self):
        return 'running' in self.motor.state

    def WaitForCompletion(self):
        while self.Running():
            time.sleep(0.01)

    def __del__(self):
        self.Stop()    

class Picker:
    target_open   = 40
    target_closed = target_open + 90
    target_store  = target_closed + 120
    target_purge = target_store + 45    

    def __init__(self,port):   

        self.pickermotor = Motor(port)
        self.motor_deg_per_picker_deg = -3

        self.SetPickRate(-40)
        time.sleep(0.5)

        while self.GetPickRate() < -10:
            pass
        self.SetPickRate(0)
        self.pickermotor.motor.reset()

    def Goto(self, reference):
        abs_speed = 120
        tolerance = 4
        self.pickermotor.Goto(reference*self.motor_deg_per_picker_deg, abs_speed*self.motor_deg_per_picker_deg, abs(tolerance*self.motor_deg_per_picker_deg))

    def SetPickRate(self,rate):
        self.pickermotor.SetSpeed(rate * self.motor_deg_per_picker_deg)

    def GetPickRate(self):
        return self.pickermotor.GetSpeed()/self.motor_deg_per_picker_deg

    def Stop(self):
        SetPickRate(self,0)


class DriveBase:
    def __init__(self,left,right,wheel_diameter,wheel_span):
        
        # Math constants
        deg_per_rad = 180/3.1416

        # Forward speed conversions
        wheel_radius = wheel_diameter/2
        self.wheel_cm_sec_per_deg_s = wheel_radius * 100 / deg_per_rad # cm of forward travel for 1 deg/s wheel rotation

        # Turnrate conversions
        wheel_base_radius = wheel_span/2
        self.wheel_cm_sec_per_base_deg_sec =  wheel_base_radius * 100 / deg_per_rad

        self.left = Motor(left)
        self.right = Motor(right)

    def DriveAndTurn(self, speed_cm_sec, turnrate_deg_sec):
        nett = speed_cm_sec / self.wheel_cm_sec_per_deg_s
        diff = turnrate_deg_sec * self.wheel_cm_sec_per_base_deg_sec / self.wheel_cm_sec_per_deg_s
        self.left.SetSpeed(nett - diff)
        self.right.SetSpeed(nett + diff)

    def Stop(self):
        self.DriveAndTurn(0,0)

class RemoteControl:

    button_list = ['NONE','LEFT_UP','LEFT_DOWN','RIGHT_UP','RIGHT_DOWN',
                'BOTH_UP','LEFT_UP_RIGHT_DOWN','LEFT_DOWN_RIGHT_UP','BOTH_DOWN',
                'BEACON', 'BOTH_LEFT','BOTH_RIGHT']

    def __init__(self,port):
        self.remote = ev3.InfraredSensor(port)
        self.remote.mode = self.remote.MODE_IR_REMOTE

    def Button(self):
        return self.button_list[self.remote.value()]

    def IsPressed(self,button):
        return self.button_list.index(button) == self.remote.value()