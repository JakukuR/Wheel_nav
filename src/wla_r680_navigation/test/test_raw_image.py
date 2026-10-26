"""Header fields parsed from serialized Image bytes match full deserialization."""
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from wla_r680_navigation_py.raw_image import RawFrame, as_frame, frame_message, frame_meta, image_header


def image(frame_id, height, width, sec=1793000000, nanosec=123456789):
    msg = Image()
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    msg.header.frame_id = frame_id
    msg.height, msg.width, msg.encoding = height, width, '16UC1'
    msg.step = width * 2
    msg.data = bytes(range(256)) * (height * width * 2 // 256) + bytes(height * width * 2 % 256)
    return msg


@pytest.mark.parametrize('frame_id', ['', 'a', 'ab', 'abc', 'abcd', 'abcde', 'd455_color_optical_frame', '相机'])
def test_header_matches_deserialization_for_every_string_alignment(frame_id):
    msg = image(frame_id, 48, 64)
    raw = serialize_message(msg)
    assert image_header(raw) == (1793000000123456789, frame_id, 48, 64)
    frame = as_frame(raw)
    assert isinstance(frame, RawFrame)
    assert frame_meta(frame) == frame_meta(msg)
    back = frame_message(frame)
    assert (back.header.frame_id, back.height, back.width, back.step, back.encoding) == (frame_id, 48, 64, 128, '16UC1')
    assert bytes(back.data) == bytes(msg.data)


def test_image_objects_pass_through_unchanged():
    msg = image('d455_color_optical_frame', 2, 3)
    assert as_frame(msg) is msg and frame_message(msg) is msg
    assert frame_meta(msg) == (1793000000123456789, 'd455_color_optical_frame', 2, 3)


def test_unknown_encoding_or_truncated_bytes_fall_back_to_none():
    raw = bytearray(serialize_message(image('x', 2, 2)))
    assert image_header(bytes(raw[:20])) is None
    raw[1] = 0  # big-endian marker: not parsed by hand
    assert image_header(bytes(raw)) is None
