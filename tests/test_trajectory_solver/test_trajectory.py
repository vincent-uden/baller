import pytest
import numpy as np

from baller.trajectory_solver.trajectory_solver import trajectory_solver
import baller.trajectory_solver.trajectory_solver as ts


# Set velocity and gravity to easy to use constants
ts.V0 = 1.0
ts.g = 10.0


@pytest.mark.parametrize(
    ("x1", "y1", "z1", "pitch", "yaw", "target_plane", "y2", "z2"),
    (
        (0, 0, 0, 0, 0, 1, 0, -5),
        (0, 1, 0, 0, 0, 1, 1, -5),
        (0, 1, 5, 0, 0, 1, 1, 0),
        (1, 1, 5, 0, 0, 2, 1, 0),
    )
)
def test_trajectory_solver(x1, y1, z1, pitch, yaw, target_plane, y2, z2):
    assert np.all(np.isclose(trajectory_solver(x1, y1, z1, pitch, yaw, target_plane), (target_plane, y2, z2)))