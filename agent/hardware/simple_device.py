from os import listdir
from sys import stderr


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

class Motor():

    MAX_SPEED = 1000
    COMMAND_RUN_FOREVER = 'run-forever'
    COMMAND_STOP = 'stop'
    COMMAND_RESET = 'reset'
    COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'
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

    def go_to(self, reference, speed, tolerance):
        if not self.is_running and not (reference - tolerance <= self.position <= reference + tolerance):
            write_int(self.position_sp_file, reference)
            absolute_limited_speed = abs(self.limit(speed))
            write_int(self.speed_sp_file, absolute_limited_speed)
            write_str(self.command_file, self.COMMAND_RUN_TO_ABS_POS) 

# TODO            
class Sensor():
    def __init__(self, port):
        self.path = get_device_path('/sys/class/lego-sensor', port)


class PowerSupply():
    def __init__(self):
        try:
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
