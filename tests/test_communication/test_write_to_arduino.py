import pytest

from baller.communication.hubert import Servo, Hubert, HubertCommand


@pytest.mark.parametrize(
        "cmd", [e for e in HubertCommand]
)
def test_send(mocker, cmd):
    """
    Assert that commands can be sent to the arduino
    """
    mock_arduino = mocker.Mock()
    hubert = Hubert("test", 9600, [])

    # Fake a connection to the arduino on Hubert
    hubert.arduino = mock_arduino

    # Try to send a message
    hubert._send(cmd)

    # Assert that we tried to write to the arduino
    expected = bytearray(chr(cmd.value).encode('utf-8'))
    mock_arduino.write.assert_called_with(expected)


@pytest.mark.parametrize(
        ('joints', 'joint_encoding'),
        (
            (
                [0, 0, 0], 
                bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            ),
            (
                [255, 255, 255], 
                bytearray([0x00, 0xff, 0x00, 0xff, 0x00, 0xff])
            ),
            (
                [256, 256, 256], 
                bytearray([0x01, 0x00, 0x01, 0x00, 0x01, 0x00])
            ),
            (
                [1000, 258, 17], 
                bytearray([0x03, 0xe8, 0x01, 0x02, 0x00, 0x11])
            ),
        )
)
def test_set_position(mocker, joints, joint_encoding):
    """
    Assert that a position is correctly sent to the arduino
    """
    mock_arduino = mocker.Mock()
    servos = [
        Servo([0, 1000], [0, 1000]),
        Servo([0, 1000], [0, 1000]),
        Servo([0, 1000], [0, 1000]),
    ]
    hubert = Hubert("test", 9600, servos)

    # Fake a connection to the arduino on Hubert
    hubert.arduino = mock_arduino

    # Try to send a position
    hubert.set_position(joints)

    # Assert that we tried to write to the arduino
    cmd_byte = bytearray(chr(HubertCommand.SET_POSITION.value).encode('utf-8'))
    expected = cmd_byte + joint_encoding
    mock_arduino.write.assert_called_with(expected)
