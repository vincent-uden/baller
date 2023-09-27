import pytest
import numpy as np

from baller.communication.hubert import Servo


@pytest.mark.parametrize(
        ("angles", "pulses", "angle"),
        (
            ([0, 100], [0, 100], 50),
            ([0, 100], [0, 1000], 50),
            ([0, 100], [-100, 100], 50),
            ([0, 100], [100, -100], 50),
            ([0, 10, 100], [0, 90, 100], 5),
            ([0, 10, 100], [100, 70, 0], 5),
        )
)
def test_servo_identety(angles, pulses, angle):
    # Be very carefull with rounding in this test as angle to pulse always gives an int
    s = Servo(angles, pulses)
    assert np.isclose(s.pulse_to_angle(s.angle_to_pulse(angle)), angle)