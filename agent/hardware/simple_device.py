from os import listdir
from sys import stderr
import time


def eprint(*args, **kwargs):
    print(*args, file=stderr, **kwargs)

def read_int(infile):
    infile.seek(0)    
    return(int(infile.read().decode().strip()))

def write_int(outfile, value):
    outfile.write(str(int(value)))
    outfile.flush()    

def read_str(infile):
    infile.seek(0)    
    return(infile.read().decode().strip())

def write_str(outfile, value):
    outfile.write(value)
    outfile.flush()      

def get_device_path(parent_folder, port_name):
    """Get a path to a device based on port name. For example:
    
    get_device_path('/sys/class/tacho-motor', 'outA')

    This could return: /sys/class/tacho-motor/motor2
    
    """
    
    try:
        # Iterate through list of numbered device folders (['motor0', 'motor1', 'motor2'] etc, or ['sensor1'] etc)
        for device_dir in listdir(parent_folder):
            # In each folder, open the address file
            with open(parent_folder + '/' + device_dir + '/address', 'r') as address_file:
                # Read the port string (e.g. 'outB')
                port_name_found = address_file.read().strip('\n')
                # If the port name matches, we are done searching and we know the full path
                if port_name in port_name_found:
                    # Make the full path
                    return parent_folder + '/' + device_dir
        # Raise an error if the specified device is not attached
        raise IOError('Device not attached!')
    except:
        # If we don't have the device folders, we are running on a PC. We return a path containing dummy data
        return 'hardware/pcdevice'


import fcntl, array
class Buttons():
    BUTTONS_FILENAME = '/dev/input/by-path/platform-gpio-keys.0-event'
    _buttons = {
        'up': {'name': BUTTONS_FILENAME, 'value': 103},
        'down': {'name': BUTTONS_FILENAME, 'value': 108},
        'left': {'name': BUTTONS_FILENAME, 'value': 105},
        'right': {'name': BUTTONS_FILENAME, 'value': 106},
        'enter': {'name': BUTTONS_FILENAME, 'value': 28},
        'backspace': {'name': BUTTONS_FILENAME, 'value': 14},
    }
    KEY_MAX = 0x2FF
    KEY_BUF_LEN = int((KEY_MAX + 7) / 8)
    EVIOCGKEY = (2 << (14 + 8 + 8) | KEY_BUF_LEN << (8 + 8) | ord('E') << 8 | 0x18)

    def __init__(self):
        self._file_cache = {}
        self._buffer_cache = {}

        for b in self._buttons:
            name = self._buttons[b]['name']

            if name not in self._file_cache:
                self._file_cache[name] = open(name, 'rb', 0)
                self._buffer_cache[name] = array.array('B', [0] * self.KEY_BUF_LEN)

    def _button_file(self, name):
        return self._file_cache[name]

    def _button_buffer(self, name):
        return self._buffer_cache[name]

    @property
    def buttons_pressed(self):
        """
        Returns list of names of pressed buttons.
        """
        for b in self._buffer_cache:
            fcntl.ioctl(self._button_file(b), self.EVIOCGKEY, self._buffer_cache[b])

        pressed = []
        for k, v in self._buttons.items():
            buf = self._buffer_cache[v['name']]
            bit = v['value']

            if bool(buf[int(bit / 8)] & 1 << bit % 8):
                pressed.append(k)

        return pressed


class Motor():

    MAX_SPEED = 1000
    COMMAND_RUN_FOREVER = 'run-forever'
    COMMAND_STOP = 'stop'
    COMMAND_RESET = 'reset'
    COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'
    COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'
    POLARITY_NORMAL = 'normal'
    POLARITY_INVERSED = 'inversed'

    def __init__(self, port):
        self.port = port
        self.path = get_device_path('/sys/class/tacho-motor', self.port)
        self.position_file = open(self.path + '/position', 'rb')
        self.speed_file = open(self.path + '/speed', 'rb')
        self.speed_sp_file = open(self.path + '/speed_sp', 'w')
        self.position_sp_file = open(self.path + '/position_sp', 'w')
        self.command_file = open(self.path + '/command', 'w')
        self.polarity_file = open(self.path + '/polarity', 'w')
        self.state_file = open(self.path + '/state', 'rb')

    @property
    def position(self):
        return read_int(self.position_file)

    @property
    def speed(self):
        return read_int(self.speed_file)    

    def limit(self, speed):
        # Ensure we bind the speed to the known maximum for this motor
        return max(min(self.MAX_SPEED, speed), -self.MAX_SPEED)    

    def run_forever_at_speed(self, speed):
        limited_speed = self.limit(speed)
        write_int(self.speed_sp_file, limited_speed)
        write_str(self.command_file, self.COMMAND_RUN_FOREVER) 

    def stop(self):
        write_str(self.command_file, self.COMMAND_STOP) 

    def reset(self):
        write_str(self.command_file, self.COMMAND_RESET)         

    @property
    def polarity(self):
        return read_str(self.polarity_file)

    @polarity.setter
    def polarity(self, polarity_string):
        write_str(self.polarity_file, polarity_string)    

    @property
    def state(self):
        return read_str(self.state_file)

    @property
    def is_running(self):
        return 'running' in self.state

    def at_target(self, target, tolerance):
        """Return True when position is near the target with the specified tolerance"""
        return target - tolerance <= self.position <= target + tolerance

    def go_to(self, reference, speed, tolerance, blocking=False):
        # When blocking is false, we do not want to wait for completion, so we use the ev3dev run_to_abs method
        if not blocking and not self.is_running and not self.at_target(reference, tolerance):
            write_int(self.position_sp_file, reference) # Write speed setpoint
            write_int(self.speed_sp_file, abs(int(self.limit(speed)))) # Write target
            write_str(self.command_file, self.COMMAND_RUN_TO_ABS_POS) # Write command

        # When we allow blocking, we can use normal speed control to ensure we actually get there    
        if blocking:
            # Check if we aren't already at the target
            if not self.at_target(reference, tolerance):
                # Otherwise, check if we should go backwards or forwards to reach the target
                if reference > self.position:
                    # If the reference is ahead of us, we go forwards
                    self.run_forever_at_speed(speed)
                    # Wait until we're there                    
                    while self.position <= reference - tolerance:
                        time.sleep(0.01)
                else:
                    # If the reference is behind us, we go backwards
                    self.run_forever_at_speed(-speed)
                    # Wait until we're there                                    
                    while self.position >= reference + tolerance:
                        time.sleep(0.01)
            # Stop the motor when at target                        
            self.stop()                        

    def turn_degrees(self, degrees, speed, tolerance):
        self.go_to(self.position+degrees, speed, tolerance)

class InfraredSensor():

    MODE_IR_REMOTE = 'IR-REMOTE'
    MODE_IR_PROX = 'IR-PROX'

    def __init__(self, port):
        self.port = port
        self.path = get_device_path('/sys/class/lego-sensor', self.port)
        self.mode_file = open(self.path + '/mode', 'w')
        self.value_file = open(self.path + '/value0', 'rb')

    @property
    def mode(self):
        return read_str(self.mode_file)    

    @mode.setter
    def mode(self, set_mode):
        return write_str(self.mode_file, set_mode)    

    @property
    def value(self):
        return read_int(self.value_file)    

    @property
    def proximity(self):
        return read_int(self.value_file)       


class PowerSupply():
    def __init__(self):
        try:
            # Open real voltage file if running on EV3
            try:
                # Open real voltage file if running on EV3 stretch
                self.voltage_file = open('/sys/class/power_supply/lego-ev3-battery/voltage_now', 'rb')
            except:
                # Open real voltage file if running on EV3 jessie
                self.voltage_file = open('/sys/class/power_supply/legoev3-battery/voltage_now', 'rb')
        except:
            # Otherwise, open the dummy file
            self.voltage_file = open('hardware/pcdevice/voltage_now', 'rb')
                  
    @property
    def voltage(self):
        return read_int(self.voltage_file) / 1e6


if __name__ == '__main__':
    """
    Test all devices in this module
    """
    p = PowerSupply()
    b = Buttons()
    try:
        ir = InfraredSensor('in4')
        found_ir = True
    except:
        found_ir = False

    while True:
        volts = p.voltage
        pressed = b.buttons_pressed
        if found_ir:
            prox = ir.proximity
        else:
            prox = ""
        print("Voltage: {0}, Buttons: {1}, Ir: {2}".format(volts, pressed,prox))
        time.sleep(0.5)

