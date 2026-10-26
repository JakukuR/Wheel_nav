"""sensor_msgs/Image header fields straight from serialized CDR bytes.

Callbacks subscribed with raw=True skip rclpy deserialization of the pixel
buffer (0.6-0.9 MB per D455 frame); they read stamp, frame and size here and
deserialize only the frames they actually use. Image objects pass through
unchanged, so callers and tests may still hand in deserialized messages.
"""
import struct

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


class RawFrame:
    """A serialized Image with its header fields; pixels decoded on demand."""
    __slots__ = ('stamp_ns', 'frame_id', 'height', 'width', 'raw')

    def __init__(self, raw, stamp_ns, frame_id, height, width):
        self.raw, self.stamp_ns, self.frame_id, self.height, self.width = raw, stamp_ns, frame_id, height, width

    def message(self):
        return deserialize_message(self.raw, Image)


def image_header(raw):
    """(stamp_ns, frame_id, height, width) from little-endian CDR, or None."""
    # 4-byte encapsulation header 00 01 00 00 = plain CDR, little endian. Field
    # alignment is relative to the payload, which starts after it at offset 4.
    if len(raw) < 24 or raw[0] != 0 or raw[1] != 1:
        return None
    sec, nanosec, length = struct.unpack_from('<iII', raw, 4)
    if length == 0 or 16 + length > len(raw):
        return None
    frame_id = bytes(raw[16:15 + length]).decode('utf-8', 'replace')
    offset = 4 + ((12 + length + 3) & ~3)
    if offset + 8 > len(raw):
        return None
    height, width = struct.unpack_from('<II', raw, offset)
    return sec * 1_000_000_000 + nanosec, frame_id, height, width


def as_frame(data):
    """Wrap raw bytes as a RawFrame; return an Image unchanged."""
    if isinstance(data, Image):
        return data
    header = image_header(data)
    if header is None:
        return deserialize_message(data, Image)
    return RawFrame(data, *header)


def frame_meta(frame):
    """(stamp_ns, frame_id, height, width) of a RawFrame or an Image."""
    if isinstance(frame, RawFrame):
        return frame.stamp_ns, frame.frame_id, frame.height, frame.width
    stamp = frame.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec, frame.header.frame_id, frame.height, frame.width


def frame_message(frame):
    """The deserialized Image for a RawFrame or an Image."""
    return frame.message() if isinstance(frame, RawFrame) else frame
