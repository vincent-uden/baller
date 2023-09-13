import serial
from enum import Enum, auto
from typing import Optional


class HubertStatus(Enum):
    IDLE = auto()           # No connection to the Hubert robot made
    MOVING = auto()         # Connection established and waiting for instructions
    NOT_CONNECTED = auto()  # Performing motion


class Hubert:

    def __init__(self, port: str, baudrate: int, timeout: Optional[float] = None) -> None:
        self.port = port
        self.baudrate = baudrate
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