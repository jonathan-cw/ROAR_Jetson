import socket
from serial import Serial
import struct
import sys
import logging
from typing import Optional

MOTOR_MAX = 1750;
MOTOR_MIN = 800;
MOTOR_NEUTRAL = 1500;
THETA_MAX = 3000;
THETA_MIN = 0;

COMMAND_THROTTLE = 0
COMMAND_STEERING = 1
UDP_PORT = 7788


class ArduinoReceiver:
    def __init__(self, client_ip: str, serial: Optional[Serial] = None):
        self.serial = serial if serial is not None else self._create_serial()
        self.fl = 0
        self.fr = 0
        self.bl = 0
        self.br = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_ip = client_ip
        self.logger = logging.getLogger("Arduino Receiver")
        self.logger.debug("Receiver Initialized")

    @staticmethod
    def _create_serial():
        if 'win' in sys.platform:
            serial = Serial(port='COM5', baudrate=115200, timeout=1, writeTimeout=1)
        else:
            serial = Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1, writeTimeout=1)
        return serial

    def update(self):
        while True:
            self.run_threaded()

    def run_threaded(self, **args):
        try:
            vel_wheel = self.serial.readline().decode().rstrip()
            self.fl, self.fr, self.bl, self.br = vel_wheel.split(" , ")
            print(self.fl, self.fr, self.bl, self.br)
        except Exception as e:
            print("Error:", e)


if __name__ == '__main__':
    import time

    serial_connection = Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1, writeTimeout=1)
    arduino_cmd_receiver = ArduinoReceiver(client_ip="localhost", serial=serial_connection)
    while True:
        arduino_cmd_receiver.run_threaded()
        print(arduino_cmd_receiver.fl, arduino_cmd_receiver.fr, arduino_cmd_receiver.bl, arduino_cmd_receiver.br)
