"""Vicon UDP listener for position/orientation data.

This module provides :class:`ViconUDP51001`, a small helper that listens on a
UDP port (default ``51001``) for six little-endian ``float32`` values
representing ``(x, y, z, rx, ry, rz)``.  The most recent packet is stored and
can be retrieved using :meth:`get_last`.

The implementation mirrors the lightweight UDP helpers used elsewhere in the
GUI, such as :class:`UDPInput` and :class:`PWMUDPReceiver`, so it uses a daemon
thread and non-blocking socket to keep the GUI responsive.
"""

from __future__ import annotations

import socket
import struct
from threading import Event, Thread
from typing import Tuple, Optional


class ViconUDP51001:
    """Background UDP receiver for Vicon position/orientation data.

    Each packet is expected to contain six little-endian ``float32`` values
    ``(x, y, z, rx, ry, rz)`` totalling 24 bytes.  The last successfully parsed
    set is stored and returned by :meth:`get_last`.
    """

    def __init__(self, port: int = 51001):
        self.port = port
        self._running = Event()
        self._thread: Optional[Thread] = None
        nan = float("nan")
        self._last: Tuple[float, float, float, float, float, float] = (
            nan, nan, nan, nan, nan, nan
        )

    # --- public API -----------------------------------------------------
    def start(self) -> None:
        """Start the background receiver thread."""
        if self._thread and self._thread.is_alive():
            return
        self._running.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Request the background thread to stop."""
        self._running.clear()

    def get_last(self) -> Tuple[float, float, float, float, float, float]:
        """Return the most recently received values."""
        return self._last

    # --- worker ---------------------------------------------------------
    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", self.port))
            sock.settimeout(0.2)
            while self._running.is_set():
                try:
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    continue
                except Exception:
                    continue
                if len(data) != 24:
                    continue
                try:
                    vals = struct.unpack("<6f", data)
                except Exception:
                    continue
                self._last = tuple(float(v) for v in vals)
        finally:
            try:
                sock.close()
            except Exception:
                pass
