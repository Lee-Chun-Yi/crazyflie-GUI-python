import socket
import struct
import time
import pytest

from cfmarslab.vicon import ViconUDP51001
from cfmarslab.ui import decode_vicon_be


def test_vicon_udp_receives_packet(tmp_path):
    # obtain a free UDP port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()

    recv = ViconUDP51001(port=port)
    recv.start()
    time.sleep(0.05)  # allow thread to start

    vals = (1.1, 2.2, 3.3, 4.4, 5.5, 6.6)
    data = struct.pack("<6f", *vals)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(data, ("127.0.0.1", port))
    sock.close()

    # wait for the receiver to process the packet
    time.sleep(0.1)
    result = recv.get_last()
    recv.stop()

    assert len(result) == 6
    for r, v in zip(result, vals):
        assert abs(r - v) < 1e-5


def test_decode_vicon_be():
    vals = (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    data = struct.pack(">6d", *vals)
    x, y, z, rx, ry, rz, fmt = decode_vicon_be(data)
    assert fmt == ">6d"
    for r, v in zip((x, y, z, rx, ry, rz), vals):
        assert abs(r - v) < 1e-9
    data_f = struct.pack(">6f", *vals)
    x, y, z, rx, ry, rz, fmt = decode_vicon_be(data_f)
    assert fmt == ">6f"
    for r, v in zip((x, y, z, rx, ry, rz), vals):
        assert abs(r - v) < 1e-5
    with pytest.raises(ValueError):
        decode_vicon_be(b"\x00" * 10)
