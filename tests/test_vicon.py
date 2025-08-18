import socket
import struct
import time
import pytest

from cfmarslab.vicon import ViconUDP51001
from cfmarslab.ui import decode_vicon_data_matlab


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


def test_decode_vicon_data_matlab():
    vals = (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    data = struct.pack(">6f", *vals)
    res = decode_vicon_data_matlab(data)
    for r, v in zip(res, vals):
        assert abs(r - v) < 1e-6
    res = decode_vicon_data_matlab(data + b"extra")
    for r, v in zip(res, vals):
        assert abs(r - v) < 1e-6
    with pytest.raises(ValueError):
        decode_vicon_data_matlab(b"\x00" * 10)
