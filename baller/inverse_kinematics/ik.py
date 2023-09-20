import numpy as np
from scipy.optimize import fsolve

from baller.utils.hubert.constants import *


LAUNCH_PLANE_OFFSET = L4 - L5


def calculate_yaw_angle(x: float, y: float) -> float:
    """
    Given the target positions x- and y-coordinate, return the required yaw angle of Hubert (The body angle)

    Parameters:
    - x (float): The targets x-coordinate in world coordinates in meters
    - y (float): The targets y-coordinate in world coordinates in meters

    Returns:
    - yaw (float):  The yaw angle of Hubert in radians
    """
    assert y > LAUNCH_PLANE_OFFSET, "This function assumes the target plane is far away. The given y-coordinate does not satisfy this condition"

    # Remove the special case of a vertical line
    if np.isclose(x, LAUNCH_PLANE_OFFSET):
        return 0.0

    def func(yaw):
        # Check if (x, y) is in the launc plane
        beta = np.pi / 2 - yaw
        k = np.tan(beta)
        m = -LAUNCH_PLANE_OFFSET * np.sin(yaw) - k * LAUNCH_PLANE_OFFSET * np.cos(yaw)
        return k * x - y + m
    
    # Discontinuity at yaw = 0
    # To avoid problems yaw0 must be on the correct side from start
    yaw0 = -0.1 if x < LAUNCH_PLANE_OFFSET else 0.1
    yaw, _, flag, msg = fsolve(func, yaw0, full_output=True, maxfev=1000)

    if flag == 1:
        return yaw

    raise RuntimeError(f"Could not find a valid yaw angle. Failed with message:\n{msg}")


def target_pos_to_joint_angles(x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Given a target position return the corresponding joint angles

    Parameters:
    - x (float):        The x position of the target in the absolute coordinate system
    - y (float):        The y position of the target in the absolute coordinate system
    - z (float):        The z position of the target in the absolute coordinate system

    Returns:
    - body rotation (float):        The rotation of the body
    - shoulder rotation (float):    The rotation of the sholder joint
    - elbow rotation (float):       The rotation of the elbow joint
    """
    yaw = calculate_yaw_angle(x, y)
