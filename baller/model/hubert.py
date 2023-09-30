from abc import ABC, abstractmethod
from typing import Literal


class HubertModel(ABC):

    @abstractmethod
    def set_pose(self, units: Literal['rad', 'deg'] = 'rad', **joints: float):
        """
        Set the pose of hubert
        """

    @abstractmethod
    def get_pose(self, units: Literal['rad', 'deg'] = 'rad') -> dict[str, float]:
        """
        Get Huberts pose
        """

    def wait_unitl_idle(self) -> None:
        """
        Wait until Hubert is Idle
        """
        pass