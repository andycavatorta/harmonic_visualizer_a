import spidev
import time
import RPi.GPIO as GPIO


class AMT203():
  BYTEORDER = "big"
    
  NO_OP = 0x00
  READ_POS = 0x10
  SET_ZERO = 0x70
  ACK_ZERO = 0x80
  WAIT = 0xA5
    
  def __init__(self, 
               bus_number=0, 
               device_number=0, 
               gpios_for_chip_select=[8], 
               speed_hz=1953125,
               delay=40):       # cs=16
    self.speed_hz = speed_hz
    self.gpios_for_chip_select = gpios_for_chip_select
    self.delay_usec = delay     # microseconds
    self.delay_sec = delay / 1E3

    GPIO.setmode(GPIO.BCM)
    self.spi = spidev.SpiDev()
    self.spi.open(bus_number, device_number)
    self.spi_speed = speed_hz
    self.spi.mode = 0b00
    self.spi.no_cs = True 

    for pin in gpios_for_chip_select:
      GPIO.setup(pin, GPIO.OUT)
      GPIO.output(pin, GPIO.HIGH)

  def close(self):
    self.spi.close()

  def from_bytes(self, value: bytes) -> int:
    return int.from_bytes(value, self.BYTEORDER)

  def spi_write_read(self, chip_select_pin, output_bytes) -> bytes:
    GPIO.output(chip_select_pin, GPIO.LOW)
    time.sleep(self.delay_sec)
    received_bytes = self.spi.xfer(output_bytes, self.speed_hz, self.delay_usec)
    GPIO.output(chip_select_pin, GPIO.HIGH)
    return received_bytes

  def spi_clean_buffer(self, chip_select_pin):
    first_result = self.spi_write_read(chip_select_pin, [self.NO_OP])
    while first_result[0] != self.WAIT:
      first_result = self.spi_write_read(chip_select_pin, [self.NO_OP])

  def get_position(self, chip_select_pin) -> int:
    request = self.spi_write_read(chip_select_pin, [self.READ_POS])
    counter = 0
    while request[0] != self.READ_POS:
      request = self.spi_write_read(chip_select_pin, [self.NO_OP])
      counter += 1
      if counter == 100:
          return -1
    position_bytes = self.spi_write_read(chip_select_pin, [self.NO_OP])
    position_bytes += self.spi_write_read(chip_select_pin, [self.NO_OP])
    return self.from_bytes(position_bytes)
  
  def get_presence(self, chip_select_pin) -> bool:
    return self.get_position(chip_select_pin) > -1

  def set_zero(self, chip_select_pin) -> bool:
    """ Must power-cycle to start using new zero point """
    request = self.spi_write_read(chip_select_pin, [self.SET_ZERO])
    counter = 0
    while request[0] != self.ACK_ZERO:
      request = self.spi_write_read(chip_select_pin, [self.NO_OP])
      counter += 1
      if counter == 100:
        return False
    return True

  def get_positions(self) -> list:
    positions = []
    for gpio_for_chip_select in self.gpios_for_chip_select:
      positions.append(self.get_position(gpio_for_chip_select))
    return positions

  def get_presences(self) -> list:
    presences = []
    for gpio_for_chip_select in self.gpios_for_chip_select:
      presences.append(self.get_presence(gpio_for_chip_select))
    return presences

amt203 = AMT203(
    gpios_for_chip_select=[3,5,7,17]
)
amt203.get_presences()
amt203.get_positions()


"""
usage:

def receptor(message):
    print(message)

encoders = AMT203s(
    receptor,
    {
        "a":3,
        "b":5,
        "c":7,
        "d":11,
    }
)

"""

"""
import spidev
import RPi.GPIO as GPIO
import threading
import time

# command bytes
BYTEORDER = "big"
NO_OP = 0x00
READ_POS = 0x10
SET_ZERO = 0x70
ACK_ZERO = 0x80
WAIT = 0xA5

class AMT203s(threading.Thread):
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
        self.delay_sec = delay #/ 1E3
        self.encoders = {}
        self.bus_number = bus_number
        self.device_number = device_number
        self.speed_hz = speed_hz
        self.polling_interval = polling_interval
        self.delay = delay

        GPIO.setmode(GPIO.BCM)
        self.open()

        for name,pin in encoder_names_and_chip_select_gpio.items():
            self.encoders[name] = AMT203(
                self.spi,
                pin,
                self.speed_hz,
                self.delay_sec
            )
        self.start()

    def reopen(self):
        pass

    def open(self):
        self.spi = spidev.SpiDev()
        self.spi.open(self.bus_number, self.device_number)
        self.spi_speed = self.speed_hz
        self.spi.mode = 0b00
        self.spi.no_cs = True 

    def run(self):
        while True:
            for encoder_name in self.encoders:
                print(encoder_name, self.encoders[encoder_name].get_position_and_difference())
                time.sleep(1)

class AMT203():
    def __init__(
        self,
        spi,
        gpio_for_chip_select,
        speed,
        delay):
        self.spi = spi
        self.gpio_for_chip_select = gpio_for_chip_select
        self.speed_hz = speed
        self.delay = delay
        self.delay_sec = delay / 1E3
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
        request = self._write_read_([READ_POS])
        counter = 0
        while request[0] != READ_POS:
          request = self._write_read_([NO_OP])
          counter += 1
          if counter == 100:
              return -1
        position_bytes = self._write_read_([NO_OP])
        position_bytes += self._write_read_([NO_OP])
        print("position_bytes=",position_bytes)
        return self.from_bytes(int.from_bytes(position_bytes, self.BYTEORDER))
        
    def _write_read_(self, output_bytes) -> bytes:
        try:
            GPIO.output(self.gpio_for_chip_select, GPIO.LOW)
            time.sleep(0.01)
            received_bytes = self.spi.xfer(output_bytes, self.speed_hz, self.delay)
            status = True
        except Exception as e: # todo: add specific exceptions?
            status = e
            received_bytes = ""
        finally:
            GPIO.output(self.gpio_for_chip_select, GPIO.HIGH)
        #print(received_bytes, status)
        return (received_bytes, status)
        
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

def receptor(message):
    print(message)

def test():
    encoders = AMT203s(
        receptor,
        {
            "a":2,
            "b":3,
            "c":4,
            "d":17,
        }
    )
"""

