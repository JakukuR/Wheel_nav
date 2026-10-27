"""Local IPC boundaries; no DDS initialization or hardware endpoints."""
import json
import struct
import pytest
from wla_person_follow.hybrid_worker import encode_packet, decode_packet, SLOT_BYTES


def test_shared_packet_roundtrip_and_owned_bytes():
    source = bytearray(b'\x00\x01\xff')
    packed = encode_packet(dict(generation=3), dict(depth=source, info=b'abc'))
    metadata, blobs = decode_packet(packed)
    source[:] = b'zzz'
    assert metadata['generation'] == 3
    assert blobs == dict(depth=b'\x00\x01\xff', info=b'abc')


@pytest.mark.parametrize('offset,size', [(-1, 1), (0, -1), (4, 1), (1, 4), (True, 1), (0, '1')])
def test_corrupt_blob_range_is_rejected(offset, size):
    metadata = json.dumps(dict(blobs=dict(depth=dict(offset=offset, size=size)))).encode()
    with pytest.raises(ValueError):
        decode_packet(struct.pack('<I', len(metadata)) + metadata + b'abc')


@pytest.mark.parametrize('data', [b'', b'abc', struct.pack('<I', 100), b'\xff'*4])
def test_truncated_metadata_is_rejected(data):
    with pytest.raises(ValueError):
        decode_packet(data)


def test_shared_slot_has_fixed_capacity():
    with pytest.raises(ValueError):
        encode_packet({}, dict(depth=b'\0'*SLOT_BYTES))
