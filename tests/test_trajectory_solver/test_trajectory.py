import pytest
import numpy as np

from baller.trajectory_solver.trajectory_solver import trajectory_solver_from_launcher_pos, trajectory_solver_from_joints
import baller.trajectory_solver.trajectory_solver as ts
from baller.utils.hubert.constants import LAUNCH_PLANE_OFFSET, L2, L3, L8, L9


Z_REST = L2 + L3 - L8 - L9


@pytest.mark.parametrize(
    ("x1", "y1", "z1", "pitch", "yaw", "target_plane", "y2", "z2"),
    (
        (0, 0, 0, 0, 0, 1, 0, -5),
        (0, 1, 0, 0, 0, 1, 1, -5),
        (0, 1, 5, 0, 0, 1, 1, 0),
        (1, 1, 5, 0, 0, 2, 1, 0),
    )
)
def test_trajectory_solver_from_launcher_pos(x1, y1, z1, pitch, yaw, target_plane, y2, z2):
    # Set velocity and gravity to easy to use constants
    ts.V0 = 1.0
    ts.g = 10.0
    assert np.all(np.isclose(trajectory_solver_from_launcher_pos(x1, y1, z1, pitch, yaw, target_plane), (target_plane, y2, z2)))


@pytest.mark.parametrize(
    ("j1", "j2", "j3", "target_plane", "y2", "z2"),
    (
        (0, 0, 0, 1.0, -LAUNCH_PLANE_OFFSET, Z_REST),
    )
)
def test_trajectory_solver_from_joints(j1, j2, j3, target_plane, y2, z2):
    # Set velocity, gravity and pitch offset to easy to use constants
    ts.V0 = 1.0
    ts.g = 0.0
    ts.PITCH_OFFSET = np.pi / 2
    assert np.all(np.isclose(trajectory_solver_from_joints(j1, j2, j3, target_plane=target_plane), (target_plane, y2, z2)))