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
    
    def pulse_to_angle(self, pulse: int) -> float:
        """
        Convert a pulse to an angle by interpolating the pulse
        """
        angle = np.interp(pulse, self.pulses, self.angles)
        return float(angle)
    
    def servo_range(self):
        return np.min(self.angles), np.max(self.angles)


class Hubert:

    def __init__(self, port: str, baudrate: int, servos: list[Servo], timeout: Optional[float] = None) -> None:
        self.port = port
        self.baudrate = baudrate
        self.servos = servos
        self.timeout = timeout

        self.arduino: Optional[serial.Serial] = None

        self.joint_angles = {f'j{i+1}': 0.0 for i in range(len(self.servos))}

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
    
    def set_position(self, **joints: float):
        """
        Send a new position to Hubert

        """
        # Update joint angles
        for j, v in joints.items():
            if j not in self.joint_angles:
                raise KeyError(f"The joint {j} is not a valid joint")
            self.joint_angles[j] = v

        joint_angles = [self.joint_angles[f'j{i+1}'] for i in range(len(self.joint_angles))]

        assert len(joint_angles) == len(self.servos)
        pulse_len = self._convert_angle_to_pulse(joint_angles)
        joint_args = [j.to_bytes(2, 'big') for j in pulse_len]

        self._send(HubertCommand.SET_POSITION, *joint_args)

    def get_position(self) -> list[float]:
        """
        Get the current position of Hubert
        """
        self._send(HubertCommand.GET_POSITION)
        angles = []
        for servo in self.servos:
            bytes = self.arduino.read(size=2)
            pulse_len = int.from_bytes(bytes, byteorder='big')
            angle = servo.pulse_to_angle(pulse_len)
            angles.append(angle)
        return angles
    
    def get_status(self) -> str:
        """
        Get the status of Hubert
        """
        self._send(HubertCommand.GET_STATUS)
        return self.arduino.readline().decode('utf-8')

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
