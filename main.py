"""
GPIOs
2  - encoder A chip select
3  - encoder B chip select
4  - encoder C chip select
17 - encoder D chip select
10 - MOSI
9  - MISO
11 - SCI CLK
6  - magnet A -> MOSFET
13 - magnet B -> MOSFET
19 - magnet C -> MOSFET
26 - LEDs -> MOSFET
14 - switch B - can switch lamp be resistor?
15 - switch C - can switch lamp be resistor?
18 - switch D - can switch lamp be resistor?
23 - lamp B -> relay or MOSFET
24 - lamp C -> relay or MOSFET
25 - lamp D -> relay or MOSFET

"""

import amt203s
import math
import queue
import threading

# signal generation

    # how to create accurate signals?

        # output audio?

        # FPGA?

        # some kind of DAC chips in stock?

    # how to ensure phase alignment?
"""
bit-banging GPIOs 
    requires some trick to get around the OS scheduler
multichannel audio 
    requires signal generator and multichannel interface
FPGA solution 
    requires resurrecting old Xilinx IDE
    having hirose connector
    resurrecting old python interface code
outboard IC on carry board?
"""
class Signals():
    def __init__(self):
        pass

    def set_frequency(self,name,value):
        print(name,value)

class Lamp():
    def __init__(
            self,
            pin
        ):
        self.pin = pin
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    def set_state(self, state):
        GPIO.output(self.pin, GPIO.HIGH if state else GPIO.LOW)

class Lamps():
    def __init__(
            self, 
            GPIOs
        ):
        self.lamps = {}
        for name, pin in GPIOs:
            self.lamps[name] = Lamp(pin)

    def set_state(self, name, state):
        self.lamps[name].set_state(state)

class Pushbutton():
    def __init__(
            self,
            pin
        ):
        self.pin = pin
        self.last_state = False
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def get_state(self, state):
        state = GPIO.output(self.pin)
        change = self.last_state != state
        self.last_state = state
        return (state, change)

class Pushbuttons(threading.Thread):
    def __init__(
            self,
            event_receiver,
            GPIOs
        ):
        threading.Thread.__init__(self)
        self.event_receiver = event_receiver
        self.buttons = {}
        for name, pin in GPIOs:
            self.buttons[name] = Pushbutton(pin)
        self.start()

    def get_states(self):
        return {
            "b":self.buttons["b"].get_state()[0],
            "c":self.buttons["c"].get_state()[0],
            "d":self.buttons["d"].get_state()[0],
        }

    def run(self):
        while True: 
            time.sleep(0.2)
            for name in ["b", "c", "d"]:
                change = self.buttons[name].get_state()
                if change == True:
                    self.event_receiver("pushbutton_event", name, state)


class Main(threading.Thread):
    PITCH_NAMES = [
        "c3","db3","d3","eb3","e3","f3","gb3","g3","ab3","a3","bb3","b3",
        "c4","db4","d4","eb4","e4","f4","gb4","g4","ab4","a4","bb4","b4",
        "c5","db5","d5","eb5","e5","f5","gb5","g5","ab5","a5","bb5","b5",
        "c6","db6","d6","eb6","e6","f6","gb6","g6","ab6","a6","bb6","b6",
        "c7","db7","d7","eb7","e7","f7","gb7","g7","ab7","a7","bb7","b7",
    ]
    PITCH_FREQUENCIES = [
        130.81, 138.59,  146.83,  155.56,  164.81,  174.61,  185,     196,     207.65,  220,  233.08,  246.94,
        261.63, 277.18,  293.66,  311.13,  329.63,  349.23,  369.99,  392,     415.3,   440,  466.16,  493.88,
        523.25, 554.37,  587.33,  622.25,  659.25,  698.46,  739.99,  783.99,  830.61,  880,  932.33,  987.77,
        1046.5, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53,
        2093,   2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520, 3729.31, 3951.07,
    ]

    def __init__(
            self,
        ):
        threading.Thread.__init__(self)
        self.queue = queue.Queue()

        self.encoders = amt203s.AMT203s(
                self.add_to_queue,
                {
                    "a":2,
                    "b":3,
                    "c":4,
                    "d":17
                }
            )

        self.lamps = Lamps(
                {
                    "a":23,
                    "b":24,
                    "c":25,
                    "d":8
                }
            )

        self.pushbuttons = Pushbuttons(
                {
                    "b":14,
                    "c":15,
                    "d":18
                }
            )

        self.signals = Signals()
        self.start()
        self.encoders.send_positions()

    def set_encoder_to_zero(self, encoder_name):
        """
        this is called only from interactive python mode
        """
        self.encoders.set_encoder_to_zero(encoder_name)

    def convert_position_to_frequency(self, position):
        pitch_range_int = int(position/60)
        pitch_range_name = self.PITCH_NAMES[pitch_range_int]
        frequency_range_center = self.PITCH_FREQUENCIES[pitch_range_int]
        local_position_offset = position-(60*pitch_range_int)
        if local_position_offset < 29:
            cents = -(local_position_offset/29*50)
            frequency = frequency_range_center*(math.pow(2,(cents/1200)))
            in_center_range = False
        elif local_position_offset > 39:
            cents = (local_position_offset-39)/29*50
            frequency = frequency_range_center*(math.pow(2,(cents/1200)))
            in_center_range = False
        else:
            frequency = frequency_range_center
            in_center_range = True
        return (pitch_range_name, frequency,in_center_range)

    def handle_start_conditions(self):
        button_states = self.pushbuttons.get_states()
        button_states["a"] = True
        for button_name, button_value in button_states.items():
            self.handle_pushbutton_event(button_name, button_value)

    def handle_pushbutton_event(self, button_name, button_value):
        self.pushbuttons.set_value[button_name] = button_value
        if button_value == True:
            position = self.encoders.get_position(button_name)
            pitch_range_name, frequency,in_center_range = convert_position_to_frequency(encoder_value)
            self.lamps.set_state(button_name, in_center_range)
            self.signals.set_frequency(button_name, frequency)
        else:
            self.lamps.set_state(button_name, 0)
            self.signals.set_frequency(button_name, 0)

    def handle_encoder_event(self, encoder_name, encoder_value):
        if encoder_name == "a" or encoder_name in self.pushbuttons.get_on():
            pitch_range_name, frequency,in_center_range = convert_position_to_frequency(encoder_value)
            self.lamps.set_state(encoder_name, in_center_range)
            self.signals.set_frequency(encoder_name, frequency)

    def add_to_queue(self, topic, device, value):
        self.queue.put((topic, device, value))

    def run(self):
        self.handle_start_conditions()
        while True:
            topic, device, value = self.queue.get()
            match topic:
                case "pushbutton_event":
                    self.handle_pushbutton_event(device, value)

                case "encoder_event":
                    self.handle_encoder_event(device, value)

                case "encoder_exception":
                    pass

