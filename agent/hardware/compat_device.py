import ev3dev.ev3 as ev3
import platform

# MOTORS ALREADY PHASED OUT

# SENSORS WILL BE NEXT...

class CompatInfraredSensor(ev3.InfraredSensor):
    """Identical to ev3.InfraredSensor, but returns dummy values when running on PC"""    
    def __init__(self, *args, **kwargs):
        super(CompatInfraredSensor, self).__init__(*args, **kwargs)
        self.running_on_ev3 = 'ev3' in platform.platform()

    def _get_attribute(self, attribute, name):
        if self.running_on_ev3:
            return ev3.InfraredSensor._get_attribute(self, attribute, name)
        else:
            return attribute, 0
    def _set_attribute(self, attribute, name, value):
        if self.running_on_ev3:
            return ev3.InfraredSensor._set_attribute(self, attribute, name, value)
        else:
            return attribute

class CompatPowerSupply(ev3.PowerSupply):
    """Identical to ev3.InfraredSensor, but returns dummy values when running on PC"""
    def __init__(self, *args, **kwargs):
        super(CompatPowerSupply, self).__init__(*args, **kwargs)
        self.running_on_ev3 = 'ev3' in platform.platform()

    def _get_attribute(self, attribute, name):
        if self.running_on_ev3:
            return ev3.PowerSupply._get_attribute(self, attribute, name)
        else:
            return attribute, 0
    def _set_attribute(self, attribute, name, value):
        if self.running_on_ev3:
            return ev3.PowerSupply._set_attribute(self, attribute, name, value)
        else:
            return attribute