from __future__ import annotations

"""Utility helpers for cfmarslab."""

import os, sys, time, subprocess
from typing import Optional

try:  # psutil is optional
    import psutil  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    psutil = None  # type: ignore


def set_realtime_priority(thread_id: Optional[int]) -> None:
    """Attempt to raise the scheduling priority of ``thread_id``.

    Elevated priority can reduce control-loop jitter but may require
    administrator/root privileges.  Unsupported platforms or failures are
    silently ignored.
    """

    if thread_id is None:
        return

    try:
        if os.name == "posix":
            # Try POSIX real-time FIFO scheduling
            try:
                param = os.sched_param(10)
                os.sched_setscheduler(thread_id, os.SCHED_FIFO, param)
                return
            except Exception:
                pass

        if psutil is None:
            return

        proc = psutil.Process()
        if os.name == "nt":
            # Windows: raise whole process priority class
            try:
                proc.nice(psutil.REALTIME_PRIORITY_CLASS)
            except Exception:
                pass
        else:
            # Fallback: lower nice value for the process
            try:
                proc.nice(-10)
            except Exception:
                pass
    except Exception:
        # Any failure is swallowed: this is best-effort only
        pass


def clear_udp_ports_windows(ports: list[int]) -> None:
    """Best-effort clearing of stuck UDP ports on Windows."""
    if not sys.platform.startswith("win"):
        return
    for port in ports:
        try:
            out = subprocess.check_output(
                f'netstat -ano | findstr :{port}',
                shell=True,
                text=True,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            continue
        for line in out.splitlines():
            parts = line.split()
            if parts and parts[-1].isdigit():
                pid = parts[-1]
                try:
                    subprocess.run(
                        f'taskkill /PID {pid} /F',
                        shell=True,
                        check=False,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                except Exception:
                    pass

