import serial
from enum import Enum, auto
from typing import Optional
import numpy as np


class HubertStatus(Enum):
    IDLE = auto()           # No connection to the Hubert robot made
    MOVING = auto()         # Connection established and waiting for instructions
    NOT_CONNECTED = auto()  # Performing motion


class HubertCommand(Enum):
    SET_POSITION = ord('m')     # Set a new position (m for move)
    GET_POSITION = ord('g')     # Get the current position (g for get)
    GET_STATUS = ord('s')       # Get the current status of Hubert (s for status)


class Servo:

    def __init__(self, angles: list[float], pulses: list[int]) -> None:
        self.angles = angles
        self.pulses = pulses

    def angle_to_pulse(self, angle: float) -> int:
        """
        Convert an angle to a pulse by interpolating the angle
        """
        pulse_len = np.interp(angle, self.angles, self.pulses)
        return int(np.round(pulse_len))


class Hubert:

    def __init__(self, port: str, baudrate: int, servos: list[Servo], timeout: Optional[float] = None) -> None:
        self.port = port
        self.baudrate = baudrate
        self.servos = servos
        self.timeout = timeout

        self.arduino: Optional[serial.Serial] = None

    def connect(self):
        if self.status != HubertStatus.NOT_CONNECTED:
            raise RuntimeError("Hubert has already established a connection")
        
        self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)

    @property
    def status(self):
        """
        Return the status of Hubert'

        NOT_CONNECTED: No connection to the Hubert robot made
        IDLE: Connection established and waiting for instructions
        MOVING: Performing motion
        """
        if self.arduino is None:
            return HubertStatus.NOT_CONNECTED
        return HubertStatus.IDLE
    
    def set_position(self, joint_angles: list[float]):
        """
        Send a new position to Hubert
        """
        pulse_len = self._convert_angle_to_pulse(joint_angles)
        joint_args = [j.to_bytes(2, 'big') for j in pulse_len]

        self._send(HubertCommand.SET_POSITION, *joint_args)

    def _send(self, cmd: HubertCommand, *args: bytes):
        """
        Send bytes to Hubert
        """
        if self.status == HubertStatus.NOT_CONNECTED:
            raise RuntimeError("Hubert is not connected")
        
        msg = bytearray([cmd.value])
        for arg in args:
            msg.extend(arg)

        self.arduino.write(msg)

    def _convert_angle_to_pulse(self, joint_angles: list[float]) -> list[int]:
        return [servo.angle_to_pulse(angle) for servo, angle in zip(self.servos, joint_angles)]
