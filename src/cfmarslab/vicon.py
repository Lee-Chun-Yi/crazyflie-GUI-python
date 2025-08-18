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
import errno
from threading import Event, Thread
from typing import Tuple, Optional, Callable


def decode_vicon_data(data: bytes) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Decode a Vicon UDP packet containing 6 float32 values.

    Returns a tuple ``(x, y, z, rx, ry, rz)`` if the packet is valid, otherwise
    ``None``."""
    if len(data) != 24:
        return None
    try:
        return struct.unpack("<6f", data)
    except Exception:
        return None


class ViconUDP51001:
    """Background UDP receiver for Vicon position/orientation data.

    Each packet is expected to contain six little-endian ``float32`` values
    ``(x, y, z, rx, ry, rz)`` totalling 24 bytes.  The last successfully parsed
    set is stored and returned by :meth:`get_last`.
    """

    def __init__(self, port: int = 51001, logger: Optional[Callable[[str], None]] = None):
        self.port = port
        self._logger = logger
        self._running = Event()
        self._thread: Optional[Thread] = None
        nan = float("nan")
        self._last: Tuple[float, float, float, float, float, float] = (
            nan, nan, nan, nan, nan, nan
        )

    def _log(self, msg: str) -> None:
        if self._logger:
            try:
                self._logger(msg)
            except Exception:
                pass
        else:
            print(msg)

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
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", self.port))
        except OSError as e:
            if getattr(e, "errno", None) == errno.EADDRINUSE or getattr(e, "winerror", None) == 10048:
                self._log(
                    f"[Vicon] Port {self.port} already in use. Another instance or process is bound. Receiver not started."
                )
            else:
                self._log(f"[Vicon] Socket error: {e}")
            self._running.clear()
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            return
        except Exception as e:
            self._log(f"[Vicon] Init error: {e}")
            self._running.clear()
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            return

        try:
            sock.settimeout(0.2)
            recv_warned = False
            while self._running.is_set():
                try:
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    continue
                except Exception as e:
                    if not recv_warned:
                        self._log(f"[Vicon] recv error: {e}")
                        recv_warned = True
                    continue
                vals = decode_vicon_data(data)
                if vals is None:
                    continue
                self._last = tuple(float(v) for v in vals)
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
