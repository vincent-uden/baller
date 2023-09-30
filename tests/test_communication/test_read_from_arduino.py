import pytest
import numpy as np

from baller.communication.hubert import Servo, Hubert, HubertCommand


@pytest.mark.parametrize(
        ('joints', 'joint_encoding'),
        (
            (
                {'j1':0, 'j2':0, 'j3': 0}, 
                b'\x00\x00\x00\x00\x00\x00',
            ),
            (
                {'j1':255, 'j2':255, 'j3': 255}, 
                b'\x00\xff\x00\xff\x00\xff',
            ),
            (
                {'j1':256, 'j2':256, 'j3': 256}, 
                b'\x01\x00\x01\x00\x01\x00',
            ),
            (
                {'j1':1000, 'j2':258, 'j3': 17}, 
                b'\x03\xe8\x01\x02\x00\x11',
            ),
        )
)
def test_get_position(mocker, joints, joint_encoding):
    """
    Assert that a position is correctly sent to the arduino
    """
    mock_arduino = mocker.Mock()
    # Set read to return two new bytes each time it is called
    mock_arduino.read.return_value = joint_encoding
    servos = [
        Servo([0, 1000], [0, 1000]),
        Servo([0, 1000], [0, 1000]),
        Servo([0, 1000], [0, 1000]),
    ]
    hubert = Hubert("test", 9600, servos)

    # Fake a connection to the arduino on Hubert
    hubert.arduino = mock_arduino

    # Try to send a position
    pos = hubert.get_pose(units='deg')

    # Assert that we call the arduino with the correct command
    expected = bytearray(chr(HubertCommand.GET_POSITION.value).encode('utf-8'))
    mock_arduino.write.assert_called_with(expected)

    # Assert that we read the correct values
    assert all([np.isclose(pos[j], joints[j]) for j in joints])
