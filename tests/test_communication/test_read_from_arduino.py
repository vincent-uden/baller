import pytest

from baller.communication.hubert import Servo, Hubert, HubertCommand


@pytest.mark.parametrize(
        ('joints', 'joint_encoding'),
        (
            (
                [0, 0, 0], 
                [b'\x00\x00', b'\x00\x00', b'\x00\x00']
            ),
            (
                [255, 255, 255], 
                [b'\x00\xff', b'\x00\xff', b'\x00\xff']
            ),
            (
                [256, 256, 256], 
                [b'\x01\x00', b'\x01\x00', b'\x01\x00']
            ),
            (
                [1000, 258, 17], 
                [b'\x03\xe8', b'\x01\x02', b'\x00\x11']
            ),
        )
)
def test_set_position(mocker, joints, joint_encoding):
    """
    Assert that a position is correctly sent to the arduino
    """
    mock_arduino = mocker.Mock()
    # Set read to return two new bytes each time it is called
    mock_arduino.read.side_effect = joint_encoding
    servos = [
        Servo([0, 1000], [0, 1000]),
        Servo([0, 1000], [0, 1000]),
        Servo([0, 1000], [0, 1000]),
    ]
    hubert = Hubert("test", 9600, servos)

    # Fake a connection to the arduino on Hubert
    hubert.arduino = mock_arduino

    # Try to send a position
    pos = hubert.get_position()

    # Assert that we call the arduino with the correct command
    expected = bytearray(chr(HubertCommand.GET_POSITION.value).encode('utf-8'))
    mock_arduino.write.assert_called_with(expected)

    # Assert that we read the correct values
    assert pos == joints
