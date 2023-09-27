import numpy as np

from baller.utils.hubert.forward_kinematics import launcher_pos


V0 = 3.0    # m/s
g = 9.82    # m/s^2

PITCH_OFFSET = np.pi / 2


def trajectory_solver_from_launcher_pos(x: float, y: float, z: float, pitch: float, yaw: float, target_plane: float) -> tuple[float, float, float]:
    """
    Solve for the trajectory of a projectile launched from position (x, y, z) with given pitch and yaw angles.
    Return the coordinates of the projectile when it passes the target_plane.

    Parameters:
    - x (float):            The x-coordinate of the launch position given in world coordinates (measured in meters)
    - y (float):            The y-coordinate of the launch position given in world coordinates (measured in meters)
    - z (float):            The z-coordinate of the launch position given in world coordinates (measured in meters)
    - pitch (float):        The pitch of the launch (up-down) (measured in radians)
    - yaw: (float):         The yaw of the launch (left-right) (measured in radians)
    - target_plane (float): The target is assumed to be located at x=target_plane

    Returns:
    - xp (float):           The x-coordinate of the projectile in the target plane (always equal to target_plane)
    - yp (float):           The y-coordinate of the projectile in the target plane
    - zp (float):           The z-coordinate of the projectile in the target plane
    """
    assert target_plane > x, "This algorithm assumes that the target plane is further away than the launch position"

    vx = V0 * np.cos(yaw) * np.cos(pitch)
    vy = V0 * np.sin(yaw) * np.cos(pitch)
    vz = V0 * np.sin(pitch)

    assert vx > 0, "Projectile will never hit target"

    # Calculate time of flight
    dx = target_plane - x
    t = dx / vx

    yf = y + vy * t
    zf = z + vz * t - g * t**2 / 2

    return target_plane, yf, zf


def launcher_pitch(j2: float, j3: float) -> float:
    """
    Get the launcher position from the robot pose
    """
    return j2 + j3 + PITCH_OFFSET - np.pi / 2


def trajectory_solver_from_joints(j1: float, j2: float, j3: float, target_plane: float) -> tuple[float, float, float]:
    xl, yl, zl = launcher_pos(j1, j2, j3)
    pitch = launcher_pitch(j2, j3)
    return trajectory_solver_from_launcher_pos(xl, yl, zl, pitch, j1, target_plane=target_plane)
