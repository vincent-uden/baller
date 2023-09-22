import numpy as np
from scipy.optimize import fsolve, minimize

from baller.utils.hubert.constants import *
from baller.utils.hubert.forward_kinematics import launcher_pos
from baller.trajectory_solver.trajectory_solver import trajectory_solver


LAUNCH_PLANE_OFFSET = L4 - L5
PITCH_OFFSET = 0


def calculate_yaw_angle(x: float, y: float) -> float:
    """
    Given the target positions x- and y-coordinate, return the required yaw angle of Hubert (The body angle)

    Parameters:
    - x (float): The targets x-coordinate in world coordinates in meters
    - y (float): The targets y-coordinate in world coordinates in meters

    Returns:
    - yaw (float):  The yaw angle of Hubert in radians
    """
    assert x > LAUNCH_PLANE_OFFSET, "This function assumes the target plane is far away. The given x-coordinate does not satisfy this condition"

    r_sq = x**2 + y**2
    sina = (LAUNCH_PLANE_OFFSET*x + y * np.sqrt(r_sq - LAUNCH_PLANE_OFFSET**2)) / r_sq
    return np.arcsin(sina)


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

    def func(js):
        xl, yl, zl = launcher_pos(yaw, js[0], js[1])
        pitch = js[0] + js[1] + PITCH_OFFSET
        xt, _, zt = trajectory_solver(xl, yl, zl, pitch, yaw, target_plane=y)
        return (xt - x)**2 + (zt - z)**2
    
    res = minimize(func, [0.0, 0.0])

    dist = np.sqrt(func(res.x))

    if dist < 0.01:
        sholder, elbow = res.x
        return yaw, sholder, elbow
    
    raise RuntimeError(f"Best solution misses with {dist*100} cm")
