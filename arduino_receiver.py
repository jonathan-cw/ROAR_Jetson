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
            try:
                vel_wheel = self.serial.readline().decode().rstrip()
                rpm_throttle, rpm_steering = vel_wheel.split(",")
                # self.logger.debug(f"rpm_throttle = {rpm_throttle} | rpm_steering = {rpm_steering}")
            except Exception as e:
                pass
                # self.logger.error(e)
            # vel_wheel = vel_wheel[2:][:-5]
            # vel_wheel = vel_wheel.split()
            # try:
            #     throttle, steering = vel_wheel
            #     throttle = float(throttle)
            #     steering = float(steering)
            # except:
            #     continue
            # # TODO @ Yuri, you might want to re-do this part to match the jetson_cmd_sender
            # # https://github.com/augcog/ROAR_Jetson/blob/revamp/jetson_cmd_sender.py#L126
            # if self.new_throttle >= MOTOR_NEUTRAL:
            #     self.new_throttle = float(throttle - MOTOR_NEUTRAL) / (MOTOR_MAX - MOTOR_NEUTRAL)
            # else:
            #     self.new_throttle = float(throttle - MOTOR_NEUTRAL) / (MOTOR_NEUTRAL - MOTOR_MIN)
            # self.new_throttle = max(-1, self.new_throttle)
            # self.new_throttle = min(1, self.new_throttle)
            # self.new_steering = float(steering - THETA_MIN) / (THETA_MAX - THETA_MIN) * 2 - 1
            # self.new_steering = max(-1, self.new_steering)
            # self.new_steering = min(1, self.new_steering)

    def run_threaded(self, **args):
        try:
            vel_wheel = self.serial.readline().decode().rstrip()
            self.fl, self.fr, self.bl, self.br = vel_wheel.split(" , ")
        except Exception as e:
            print(e)


if __name__ == '__main__':
    import time

    serial_connection = Serial("COM5", baudrate=115200, timeout=0.5, writeTimeout=0.5)
    arduino_cmd_receiver = ArduinoReceiver(client_ip="localhost", serial=serial_connection)
    while True:
        arduino_cmd_receiver.run_threaded()
        print(arduino_cmd_receiver.fl, arduino_cmd_receiver.fr, arduino_cmd_receiver.bl, arduino_cmd_receiver.br)
