from __future__ import annotations

"""Utility helpers for cfmarslab."""

import os
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

