import pytest
import numpy as np

from baller.utils.hubert.forward_kinematics import joint3pos
from baller.utils.hubert.constants import L2, L3, L4, L5, L6, L7, L8, L9


x0 = L6 + L7
y0 = -L4 + L5
z0 = L2 + L3 - L8 - L9

DEG90 = np.pi / 2
DEG180 = np.pi


@pytest.mark.parametrize(
    ("j1", "j2", "j3", "x", "y", "z"),
    (
        (0, 0, 0, x0, y0, z0),
        (DEG90, 0, 0, -y0, x0, z0),
        (DEG180, 0, 0, -x0, -y0, z0),
        (0, DEG90, 0, L6 + L8 + L9, y0, L2 + L3 + L7),
        (0, DEG180, 0, L6 - L7, y0, L2 + L3 + L8 + L9),
        (0, 0, DEG90, L6 + L7 + L9, y0, L2 + L3 - L8),
        (0, 0, DEG180, x0, y0, L2 + L3 - L8 + L9),
    )
)
def test_joint_3_pos(j1, j2, j3, x, y, z):
    assert np.all(np.isclose(joint3pos(j1, j2, j3), [x, y, z]))
