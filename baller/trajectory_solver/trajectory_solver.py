import numpy as np


V0 = 1.0    # m/s
g = 9.82    # m/s^2


def trajectory_solver(x: float, y: float, z: float, pitch: float, yaw: float, target_plane: float) -> tuple[float, float, float]:
    """
    Solve for the trajectory of a projectile launched from position (x, y, z) with given pitch and yaw angles.
    Return the coordinates of the projectile when it passes the target_plane.

    Parameters:
    - x (float):            The x-coordinate of the launch position given in world coordinates (measured in meters)
    - y (float):            The y-coordinate of the launch position given in world coordinates (measured in meters)
    - z (float):            The z-coordinate of the launch position given in world coordinates (measured in meters)
    - pitch (float):        The pitch of the launch (up-down) (measured in radians)
    - yaw: (float):         The yaw of the launch (left-right) (measured in radians)
    - target_plane (float): The target is assumed to be located at y=target_plane

    Returns:
    - xp (float):           The x-coordinate of the projectile in the target plane
    - yp (float):           The y-coordinate of the projectile in the target plane (always equal to target_plane)
    - zp (float):           The z-coordinate of the projectile in the target plane
    """
    assert target_plane > y, "This algorithm assumes that the target plane is further away than the launch position"

    vx = V0 * np.sin(yaw) * np.cos(pitch)
    vy = V0 * np.cos(yaw) * np.cos(pitch)
    vz = V0 * np.sin(pitch)

    assert vy > 0, "Projectile will never hit target"

    # Calculate time of flight
    dy = target_plane - y
    t = dy / vy

    xf = x + vx * t
    zf = z + vz * t - g * t**2 / 2

    return xf, target_plane, zf
