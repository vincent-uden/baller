import serial
from enum import Enum, auto
from typing import Optional
import numpy as np
from threading import Lock
import time


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

        self.arduino_lock = Lock()

    def connect(self):
        if self.status != HubertStatus.NOT_CONNECTED:
            raise RuntimeError("Hubert has already established a connection")
        
        self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        
        # Wait for the serial connection to be ready
        time.sleep(1.0)

        # Clear all buffers
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()
        

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

        with self.arduino_lock:
            self._send(HubertCommand.SET_POSITION, *joint_args)

    def get_position(self) -> list[float]:
        """
        Get the current position of Hubert
        """
        angles = []

        with self.arduino_lock:
            self._send(HubertCommand.GET_POSITION)
            bs = self._read(2 * len(self.servos))

        for i in range(len(self.servos)):
            b = bs[2*i:2*i+2]
            pulse_len = int.from_bytes(b, byteorder='big')
            angle = self.servos[i].pulse_to_angle(pulse_len)
            angles.append(angle)
        return angles
    
    def get_status(self) -> str:
        """
        Get the status of Hubert
        """
        with self.arduino_lock:
            self._send(HubertCommand.GET_STATUS)
            return self._readline()

    def _send(self, cmd: HubertCommand, *args: bytes):
        """
        Send bytes to Hubert
        """
        assert self.arduino_lock.locked(), "You must lock the arduino before comunicaiton"

        if self.status == HubertStatus.NOT_CONNECTED:
            raise RuntimeError("Hubert is not connected")
        
        msg = bytearray([cmd.value])
        for arg in args:
            msg.extend(arg)

        self.arduino.write(msg)

    def _read(self, n: int) -> bytes:
        """
        Read n bytes from Hubert
        """
        assert self.arduino_lock.locked(), "You must lock the arduino before comunicaiton"
        bs = self.arduino.read(size=n)
        return bs
    
    def _readline(self) -> str:
        assert self.arduino_lock.locked(), "You must lock the arduino before comunicaiton"
        bs = self.arduino.readline()
        return bs.decode('utf-8')

    def _convert_angle_to_pulse(self, joint_angles: list[float]) -> list[int]:
        return [servo.angle_to_pulse(angle) for servo, angle in zip(self.servos, joint_angles)]
