import serial
from enum import Enum, auto, IntFlag
from typing import Optional, Literal
import numpy as np
from threading import Lock
import time

from baller.model.hubert import HubertModel


class HubertStatus(IntFlag):
    LAUNCHING = auto()      # Currently in the process of launching a projectile
    MOVING = auto()         # Performing motion
    IDLE = auto()           # Connection established and waiting for instructions
    NOT_CONNECTED = auto()  # No connection to the Hubert robot made


class HubertCommand(Enum):
    SET_POSITION = ord('m')     # Set a new position (m for move)
    GET_POSITION = ord('g')     # Get the current position (g for get)
    GET_STATUS = ord('s')       # Get the current status of Hubert (s for status)
    LAUNCH = ord('l')           # Start the launch of a projectile


class Servo:

    def __init__(self, angles: list[float], pulses: list[int], units: Literal['rad', 'deg'] = 'deg') -> None:
        assert len(angles) == len(pulses), "Angles and pulses must be the same length"
        assert len(angles) > 1, "You must provide more than one value"
        assert np.all(np.diff(angles) > 0), "Angles must be increasing"

        self.angles = [a if units == 'rad' else np.deg2rad(a) for a in angles]
        self.pulses = pulses

        pulses_decresing = pulses[1] < pulses[0]   # Check if pulses is decreasing, then they must be reversed
        self.rev_angles = list(reversed(self.angles)) if pulses_decresing else self.angles
        self.rev_pulses = list(reversed(self.pulses)) if pulses_decresing else self.pulses

        assert np.all(np.diff(self.rev_pulses) > 0), "Pulses could not be converted to an increasing list"

    def angle_to_pulse(self, angle: float, units: Literal['rad', 'deg'] = 'deg') -> int:
        """
        Convert an angle to a pulse by interpolating the angle
        """
        angle = angle if units == 'rad' else np.deg2rad(angle)
        pulse_len = np.interp(angle, self.angles, self.pulses)
        return int(np.round(pulse_len))
    
    def pulse_to_angle(self, pulse: int, units: Literal['rad', 'deg'] = 'deg') -> float:
        """
        Convert a pulse to an angle by interpolating the pulse
        """
        angle = np.interp(pulse, self.rev_pulses, self.rev_angles)
        angle = angle if units == 'rad' else np.rad2deg(angle)
        return float(angle)
    
    def servo_range(self, units: Literal['rad', 'deg'] = 'deg') -> tuple[float, float]:
        min_a = np.min(self.angles)
        max_a = np.max(self.angles)
        if units == 'deg':
            min_a = np.rad2deg(min_a)
            max_a = np.rad2deg(max_a)
        return min_a, max_a


class Hubert(HubertModel):

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
    def status(self) -> HubertStatus:
        """
        Return the status of Hubert
        """
        if self.arduino is None:
            return HubertStatus.NOT_CONNECTED
        # Ask arduino for status
        with self.arduino_lock:
            self._send(HubertCommand.GET_STATUS)
            bs = self._read(1)
        status_flag = HubertStatus(int.from_bytes(bs, byteorder='big'))
        
        # If any status message was in the status flag return the flag
        if status_flag:
            return status_flag
        # Oterwise return the idel flag
        return HubertStatus.IDLE
    
    def wait_unitl_idle(self) -> None:
        self.wait_for(HubertStatus.IDLE)

    def wait_for(self, status_flag: HubertStatus) -> None:
        time.sleep(0.1)
        while status_flag not in self.status:
            time.sleep(1)
    
    def set_pose(self, units: Literal['rad', 'deg'] = 'rad', **joints: float):
        """
        Send a new position to Hubert
        """
        # Update joint angles
        for j, v in joints.items():
            if j not in self.joint_angles:
                raise KeyError(f"The joint {j} is not a valid joint")
            self.joint_angles[j] = v if units == 'rad' else np.deg2rad(v)

        joint_angles = [self.joint_angles[f'j{i+1}'] for i in range(len(self.joint_angles))]

        assert len(joint_angles) == len(self.servos)
        pulse_len = self._convert_angle_to_pulse(joint_angles)
        joint_args = [j.to_bytes(2, 'big') for j in pulse_len]

        with self.arduino_lock:
            self._send(HubertCommand.SET_POSITION, *joint_args)

    def get_pose(self, units: Literal['rad', 'deg'] = 'rad') -> dict[str, float]:
        """
        Get the current position of Hubert
        """
        with self.arduino_lock:
            self._send(HubertCommand.GET_POSITION)
            bs = self._read(2 * len(self.servos))

        joint_angles = {}

        for i in range(len(self.servos)):
            b = bs[2*i:2*i+2]
            pulse_len = int.from_bytes(b, byteorder='big')
            angle = self.servos[i].pulse_to_angle(pulse_len, units=units)
            joint_angles[f'j{i+1}'] = angle
        return joint_angles
        
    def launch(self) -> None:
        """
        Launch a projectile
        """
        with self.arduino_lock:
            self._send(HubertCommand.LAUNCH)

    def _send(self, cmd: HubertCommand, *args: bytes):
        """
        Send bytes to Hubert
        """
        assert self.arduino_lock.locked(), "You must lock the arduino before comunicaiton"

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

    def _convert_angle_to_pulse(self, joint_angles: list[float]) -> list[int]:
        return [servo.angle_to_pulse(angle, units='rad') for servo, angle in zip(self.servos, joint_angles)]


if __name__ == '__main__':
    servos = [
        Servo([-90, 0, 90], [700, 1600, 2070]),
        Servo([0, 180], [2200, 1350]),
        Servo([-90, 0, 90], [420, 1410, 2400]),
        Servo([-90, 90], [600, 1500]),
        Servo([-90, 90], [1170, 2100]),
    ]
    com = Hubert("COM3", baudrate=57600, servos=servos, timeout=0.1)
    print(com.status.name)
    com.connect()
    print(com.status.name)
    com.set_pose(j1=90)
    time.sleep(0.1)
    print(com.status.name)
    com.launch()
    time.sleep(0.1)
    print(com.status.name)
    time.sleep(5)
    print(com.status.name)
    