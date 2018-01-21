import time, evdev
from threading import Thread

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

        # print("Returned stick value {0}, {1}, {2}".format(scale, value, deadzone))
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
