from __future__ import annotations

"""Cross platform helpers for high precision timing and scheduling.

The :class:`Realtime` class exposes a small API that abstracts the underlying
operating system.  The implementation aims to be dependency free and any
failures in applying real‑time features are ignored gracefully.
"""

import os
import platform
import time
import ctypes
from ctypes import byref, Structure, c_int, c_long, c_longlong, c_ulong, c_void_p

__all__ = ["Realtime"]


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


_warned: set[str] = set()

def _warn_once(msg: str) -> None:
    if msg in _warned:
        return
    _warned.add(msg)
    try:
        print(msg)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# realtime implementation
# ---------------------------------------------------------------------------


class Realtime:
    """Platform abstraction for deterministic control loop timing."""

    def __init__(
        self,
        *,
        cpu: int | None = None,
        fifo_priority: int = 50,
        spin_ns: int = 150_000,
    ) -> None:
        self._cpu = cpu
        self._prio = int(fifo_priority)
        self._spin_ns = int(spin_ns)
        self._spin = self._spin_ns / 1_000_000_000.0
        self._dt = 0.01
        self._started = False

        self._is_linux = os.name == "posix" and platform.system() == "Linux"
        self._is_windows = os.name == "nt"

        self._setup_platform()

    # ------------------------------------------------------------------
    def _setup_platform(self) -> None:
        if self._is_linux:
            # decide which clock to use
            self._clock_id = getattr(time, "CLOCK_MONOTONIC", 1)
            self._clock_id = getattr(time, "CLOCK_MONOTONIC_RAW", self._clock_id)
            try:
                self._libc = ctypes.CDLL("libc.so.6", use_errno=True)
            except Exception:  # pragma: no cover
                self._libc = None
                _warn_once("Realtime: unable to load libc")
                return

            class timespec(Structure):
                _fields_ = [("tv_sec", c_long), ("tv_nsec", c_long)]

            self._timespec = timespec
            try:
                self._clock_gettime = self._libc.clock_gettime
                self._clock_gettime.argtypes = [c_int, ctypes.POINTER(timespec)]
            except Exception:
                self._clock_gettime = None
            try:
                self._clock_nanosleep = self._libc.clock_nanosleep
                self._clock_nanosleep.argtypes = [
                    c_int,
                    c_int,
                    ctypes.POINTER(timespec),
                    c_void_p,
                ]
            except Exception:
                self._clock_nanosleep = None
        elif self._is_windows:
            self._kernel32 = ctypes.windll.kernel32
            self._winmm = ctypes.windll.winmm
            self._QueryPerformanceCounter = self._kernel32.QueryPerformanceCounter
            self._QueryPerformanceFrequency = self._kernel32.QueryPerformanceFrequency
            freq = c_longlong()
            self._QueryPerformanceFrequency(byref(freq))
            self._qpc_freq = float(freq.value)
        else:
            self._clock_gettime = None

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._started:
            return
        self._started = True
        try:
            if self._is_linux and self._libc is not None:
                # priority via SCHED_FIFO
                try:
                    class sched_param(Structure):
                        _fields_ = [("sched_priority", c_int)]

                    param = sched_param(self._prio)
                    self._libc.sched_setscheduler(0, 1, byref(param))  # 1=SCHED_FIFO
                except Exception:
                    _warn_once("Realtime: failed to set SCHED_FIFO (permission?)")
                # CPU affinity
                if self._cpu is not None:
                    try:
                        os.sched_setaffinity(0, {int(self._cpu)})
                    except Exception:
                        _warn_once("Realtime: failed to set CPU affinity")
                # memory locking
                try:
                    self._libc.mlockall(3)  # MCL_CURRENT|MCL_FUTURE
                except Exception:
                    _warn_once("Realtime: mlockall failed")
            elif self._is_windows:
                try:
                    self._winmm.timeBeginPeriod(1)
                except Exception:
                    _warn_once("Realtime: timeBeginPeriod failed")
                try:
                    thread = self._kernel32.GetCurrentThread()
                    self._kernel32.SetThreadPriority(thread, 2)  # THREAD_PRIORITY_HIGHEST
                except Exception:
                    _warn_once("Realtime: SetThreadPriority failed")
                if self._cpu is not None:
                    try:
                        mask = 1 << int(self._cpu)
                        self._kernel32.SetThreadAffinityMask(thread, c_ulong(mask))
                    except Exception:
                        _warn_once("Realtime: affinity mask failed")
        except Exception:
            pass

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        try:
            if self._is_linux and self._libc is not None:
                try:
                    self._libc.munlockall()
                except Exception:
                    pass
            elif self._is_windows:
                try:
                    self._winmm.timeEndPeriod(1)
                except Exception:
                    pass
        except Exception:
            pass

    # ------------------------------------------------------------------
    # timing helpers
    # ------------------------------------------------------------------
    def now(self) -> float:
        if self._is_linux and self._libc is not None and self._clock_gettime:
            ts = self._timespec()
            self._clock_gettime(self._clock_id, byref(ts))
            return ts.tv_sec + ts.tv_nsec * 1e-9
        elif self._is_windows:
            counter = c_longlong()
            self._QueryPerformanceCounter(byref(counter))
            return counter.value / self._qpc_freq
        return time.monotonic()

    def sleep_until(self, t_abs: float) -> None:
        spin = self._spin
        pre = t_abs - spin
        while True:
            now = self.now()
            if now >= pre:
                break
            remaining = pre - now
            if (
                self._is_linux
                and self._libc is not None
                and self._clock_nanosleep is not None
            ):
                ts = self._timespec(int(pre), int((pre - int(pre)) * 1e9))
                self._clock_nanosleep(self._clock_id, 1, byref(ts), None)  # TIMER_ABSTIME
            else:
                time.sleep(remaining)
        # busy wait for final window
        while self.now() < t_abs:
            pass

    def set_rate(self, hz: int) -> None:
        hz = max(1, int(hz))
        self._dt = 1.0 / float(hz)

    def next_tick(self, t_prev: float) -> float:
        t_target = t_prev + self._dt
        self.sleep_until(t_target)
        return t_target

