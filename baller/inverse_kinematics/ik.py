import numpy as np
from scipy.optimize import minimize
from typing import Optional

from baller.utils.hubert.constants import LAUNCH_PLANE_OFFSET
from baller.trajectory_solver.trajectory_solver import trajectory_solver_from_joints, launcher_pitch


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


def target_pos_to_joint_angles(
        x: float,
        y: float,
        z: float,
        j1: Optional[float] = None,
        j2: Optional[float] = None,
        j3: Optional[float] = None,
    ) -> tuple[float, float, float, float]:
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
        _, yt, zt = trajectory_solver_from_joints(yaw, js[0], js[1], target_plane=x)
        return (yt - y)**2 + (zt - z)**2
    
    if j2 is None:
        j2 = 0.0
    if j3 is None:
        j3 = 0.0

    # Constrain j2 and j3 to give a pitch that is in the range (-90, 90)
    constraints = [
        {
            'type': 'ineq',
            'fun': lambda js: np.pi / 2 - 0.1 + launcher_pitch(js[0], js[1]),
        },
        {
            'type': 'ineq',
            'fun': lambda js:  np.pi / 2 - 0.1 - launcher_pitch(js[0], js[1]),
        }
    ]
    res = minimize(func, [j2, j3], constraints=constraints)

    dist = np.sqrt(func(res.x))

    sholder, elbow = res.x
    return yaw, sholder, elbow, dist


if __name__ == '__main__':
    print(target_pos_to_joint_angles(1.0, 0.0, 0.2))
