import pytest
import numpy as np

from baller.inverse_kinematics.ik import calculate_yaw_angle, LAUNCH_PLANE_OFFSET


@pytest.mark.parametrize(
        ("x", "y", "expected_yaw"),
        (
            (LAUNCH_PLANE_OFFSET, 1.0, 0.0),
            (0.0, np.sqrt(2) * LAUNCH_PLANE_OFFSET, np.pi / 4),
            (1e10, 1.0, -np.pi / 2),
            (-1e10, 1.0, np.pi / 2),
        )
)
def test_yaw_angle(x, y, expected_yaw):
    yaw = calculate_yaw_angle(x, y)
    assert np.isclose(yaw, expected_yaw)
