import AMT203
import spidev
import threading
import time
import RPi.GPIO as GPIO


class AMT203s():
    BYTEORDER = "big"
    NO_OP = 0x00
    READ_POS = 0x10
    SET_ZERO = 0x70
    ACK_ZERO = 0x80
    WAIT = 0xA5

    def __init__(
            self,
            name, 
            pin, 
            spi,
            delay_usec,
            delay_sec
        ):

        self.name = name
        self.pin = pin
        self.spi = spi
        self.delay_usec = delay_usec
        self.delay_sec = delay_sec

        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)

        self.last_position = -1

    def from_bytes(self, value: bytes) -> int:
        return int.from_bytes(value, self.BYTEORDER)

    def spi_write_read(self, output_bytes) -> bytes:
        GPIO.output(self.pin, GPIO.LOW)
        time.sleep(self.delay_sec)
        received_bytes = self.spi.xfer(output_bytes, self.speed_hz, self.delay_usec)
        GPIO.output(self.pin, GPIO.HIGH)
        return received_bytes

    def get_position(self) -> int:
        request = self.spi_write_read(self.pin, [self.READ_POS])
        counter = 0
        while request[0] != self.READ_POS:
            request = self.spi_write_read(self.pin, [self.NO_OP])
            counter += 1
            if counter == 100:
                return -1
        position_bytes = self.spi_write_read(self.pin, [self.NO_OP])
        position_bytes += self.spi_write_read(self.pin, [self.NO_OP])
        position_int = self.from_bytes(position_bytes)
        change_int = position_int - self.last_position
        self.last_position = position_int
        return (position_int, change_int)

    def set_zero(self, self.pin) -> bool:
        """ Must power-cycle to start using new zero point """
        request = self.spi_write_read(self.pin, [self.SET_ZERO])
        counter = 0
        while request[0] != self.ACK_ZERO:
            request = self.spi_write_read(self.pin, [self.NO_OP])
            counter += 1
            if counter == 100:
                return False
        return True


class AMT203s(threading.thread):
    
    def __init__(
            self,
            event_receiver,
            names_gpios,               
            bus_number=0, 
            device_number=0, 
            speed_hz=1953125,
            delay=40,
            polling_period = 0.1
        ):
        threading.thread.__init__(self)
        delay_sec = delay / 1E3
        self.polling_period = polling_period
        self.event_receiver = event_receiver

        self.spi = spidev.SpiDev()
        self.spi.open(bus_number, device_number)
        self.spi_speed = speed_hz
        self.spi.mode = 0b00
        self.spi.no_cs = True 

        GPIO.setmode(GPIO.BCM)

        self.encoders = {}
        for name, pin in names_gpios.items():
            self.encoders[name] = AMT203(
                name, 
                pin, 
                self.spi,
                delay_usec,
                delay_sec,
                event_receiver
            )

        self.start()

    def close(self):
        self.spi.close()

    def get_positions(self):
        positions = {}
        for name,encoder in self.encoders.items():
            positions[name] = encoder.get_position()[0]
        return positions

    def get_presences(self):
        return map(lambda x: x >-1, self.get_positions())

    def set_zero(self,name):
        return self.encoders[name].set_zero()

    def run(self):
        while True:
            time.sleep(self.polling_period)
            for name,encoder in self.encoders.items():
                position, change = encoder.get_position()
                if change != 0:
                    self.event_receiver("encoder_event", name, position)
