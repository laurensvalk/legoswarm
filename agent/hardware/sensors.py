from .compat_device import CompatInfraredSensor

class BallSensor(CompatInfraredSensor):
    pass


class RemoteControl(CompatInfraredSensor):
    """Configures IR Sensor as IR Receiver and reads IR button status"""

    # Ordered list of possible button presses
    button_list = ['NONE','LEFT_UP','LEFT_DOWN','RIGHT_UP','RIGHT_DOWN',
                'BOTH_UP','LEFT_UP_RIGHT_DOWN','LEFT_DOWN_RIGHT_UP','BOTH_DOWN',
                'BEACON', 'BOTH_LEFT','BOTH_RIGHT']

    def __init__(self, port):
        """Configure IR sensor in remote mode"""
        CompatInfraredSensor.__init__(self,port)
        self.mode = self.MODE_IR_REMOTE        

    @property
    def button(self):
        """Return name (string) of button currently pressed"""
        return self.button_list[self.value()]

    def is_pressed(self, button):
        """Check if specified button is currently pressed"""
        return self.button_list.index(button) == self.value()
