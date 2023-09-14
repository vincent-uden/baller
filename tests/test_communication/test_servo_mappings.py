import pytest

from baller.communication.hubert import Servo


@pytest.mark.parametrize(
        ("angle", "pulse_width"),
        (
            (0.0, 0),
            (1.0, 100),
            (-1.0, 0),
            (1.1, 100),
            (0.5, 50),
        ),
)
def test_angle_to_pulse_conversion_linear(angle, pulse_width):
    """
    Test the mappings of a servo
    """
    angles = [0.0, 1.0]
    pulses = [0, 100]
    servo = Servo(angles, pulses)

    assert servo.angle_to_pulse(angle) == pulse_width


@pytest.mark.parametrize(
        ("angle", "pulse_width"),
        (
            (0.0, 0),
            (1.0, 100),
            (-1.0, 0),
            (1.1, 100),
            (0.5, 60),
            (0.25, 30),
            (0.75, 80),
        ),
)
def test_angle_to_pulse_conversion_piecewise(angle, pulse_width):
    """
    Test the mappings of a servo
    """
    angles = [0.0, 0.5, 1.0]
    pulses = [0, 60, 100]
    servo = Servo(angles, pulses)

    assert servo.angle_to_pulse(angle) == pulse_width