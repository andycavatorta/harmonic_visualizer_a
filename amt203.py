"""
usage:

def receptor(message):
    print(message)

encoders = AMT203(
    receptor,
    {
        "a":3,
        "b":5,
        "c":7,
        "d":11,
    }
)
"""
import spidev
import RPi.GPIO as GPIO
import time

# command bytes
BYTEORDER = "big"
NO_OP = 0x00
READ_POS = 0x10
SET_ZERO = 0x70
ACK_ZERO = 0x80
WAIT = 0xA5

class AMT203(threading.Thread):
    def __init__(
            self, 
            data_receiver,
            encoder_names_and_chip_select_gpio, # not optional
            bus_number=0, 
            device_number=0, 
            speed_hz=1953125,
            polling_interval=40,
            delay=40, 
            mode = 0b00
        ):
        threading.Thread.__init__(self)
        # locals
        self.data_receiver = data_receiver
        self.delay_sec = delay / 1E3
        self.encoders = {}

        GPIO.setmode(GPIO.BCM)
        self.open()

        for name,pin in encoder_names_and_chip_select_gpio:
            self.encoders[name] = self.AMT203(
                bus_number,
                device_number,
                pin,
                speed_hz,
                self.delay_sec
            )

    def reopen(self):
        pass

    def open(self):
        self.spi = spidev.SpiDev()
        self.spi.open(bus_number, device_number)
        self.spi_speed = speed_hz
        self.spi.mode = 0b00
        self.spi.no_cs = True 

    def run(self):
        while True:
            for encoder_name in self.encoders:
                print(encoder_name)
                time.sleep(1)

class AMT203():
    def __init__(
        self,
        bus_number, 
        device_number, 
        gpio_for_chip_select,
        speed_hz,
        delay):
        self.gpio_for_chip_select = gpio_for_chip_select
        self.speed_hz = speed_hz
        self.delay = delay
        GPIO.setup(gpio_for_chip_select, GPIO.OUT)
        GPIO.output(gpio_for_chip_select, GPIO.HIGH)
        self.last_position = 0

    def read(self):
        try:
            GPIO.output(self.gpio_for_chip_select, GPIO.LOW)
            # specific operation
            status = True
        except Exception as e: # todo: add specific exceptions?
            status = e
        finally:
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
        return (device_ref, status)
        
    def write(self):
        try:
            GPIO.output(self.gpio_for_chip_select, GPIO.LOW)
            # specific operation
            status = True
        except Exception as e: # todo: add specific exceptions?
            status = e
        finally:
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
        return (device_ref, status)

        
    def get_position_and_difference(self):
        attempts = 0
        first_result = self._write_read_([0x10])   

        while True:
          first_result = self._write_read_([0x00]) 
          attempts = attempts + 1
          if attempts > 100: 
            print(" yuk")
          
          if first_result[ 0 ] != 0x10:
            break;

        attempts = 0
        while True:
          tmp = self._write_read_( [0x00])
          if tmp[ 0 ] == 0x10:
            msb_result = self._write_read_( [0x00])
            break;
          attempts = attempts + 1
          if attempts > 100: 
            print(" yuk2 ", pin)
            continue

        lsb_result = self._write_read_([0x00]) 
        final_result = (msb_result[0]<<8 | lsb_result[0])
        while True:
            first_result = self._write_read_([0x00]) 
            if first_result[ 0 ] == 165:
                break;

        print("final_result",final_result)

        
    def _write_read_(self, msg_out):
        try:
            GPIO.output(self.gpio_for_chip_select, GPIO.LOW)
            time.sleep(.05)
            msg_in = spi.xfer(msg_out, self.speed, self.delay)
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
            status = True
        except Exception as e: # todo: add specific exceptions?
            status = e
            msg_in = ""
        finally:
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
        return (msg_in, status)
        
    def set_zero(self):
        try:
            GPIO.output(self.gpio_for_chip_select, GPIO.LOW)
            # specific operation
            status = True
        except Exception as e: # todo: add specific exceptions?
            status = e
        finally:
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
        return (device_ref, status)

    def get_presence(self):
        try:
            GPIO.output(self.gpio_for_chip_select, GPIO.LOW)
            # specific operation
            status = True
        except Exception as e: # todo: add specific exceptions?
            status = e
        finally:
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
        return (device_ref, status)


#def init():
























