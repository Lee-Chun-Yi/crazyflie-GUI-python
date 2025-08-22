import socket, struct, threading, time, queue, logging, sys, traceback
from time import perf_counter
import tkinter as tk
from tkinter import ttk, scrolledtext
from collections import deque
import math

from .models import SharedState
from .config import load_config, save_config, Rates
from .control import UDPInput, SetpointLoop, PWMSetpointLoop, PWMUDPReceiver
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .link import LinkManager

def decode_vicon_be(payload: bytes):
    """Decode big-endian Vicon packets.
    - 48 bytes -> '>6d' (x,y,z,rx,ry,rz) as doubles
    - 24 bytes -> '>6f' (x,y,z,rx,ry,rz) as float32
    Raises ValueError for other sizes.
    """
    n = len(payload)
    if n >= 48:
        x, y, z, rx, ry, rz = struct.unpack(">6d", payload[:48])
        return float(x), float(y), float(z), float(rx), float(ry), float(rz), ">6d"
    if n == 24:
        x, y, z, rx, ry, rz = struct.unpack(">6f", payload[:24])
        return float(x), float(y), float(z), float(rx), float(ry), float(rz), ">6f"
    raise ValueError(f"unexpected packet size: {n} bytes")

def decode_pose_be_doubles(payload: bytes):
    """Backward compatible wrapper returning only pose."""
    x, y, z, rx, ry, rz, _ = decode_vicon_be(payload)
    return x, y, z, rx, ry, rz

UDP_COORD_PORT = 51002
RADIO_BITRATES = ("2M", "1M", "250K")
UI_TICK_MS = int(1000 / Rates.LOG_UI_HZ)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Crazyflie GUI — VBAT: 0.00 V")
        try: self.state("zoomed")
        except Exception: pass
        try: ttk.Style().theme_use("clam")
        except Exception: pass

        self.state_model = SharedState()
        self.cfg = load_config()
        self.state_model.throttle_offset = self.cfg.throttle_offset
        self.link: "LinkManager|None" = None
        self.cf = None

        self._coords_running = False
        self._coords_thread: threading.Thread|None = None

        self._log_q = queue.Queue()
        self.enqueue_log = lambda s: self._log_q.put(s if isinstance(s, str) else str(s))

        def _thread_excepthook(args):
            lines = traceback.format_exception(args.exc_type, args.exc_value, args.exc_traceback)
            self.enqueue_log("\n".join(lines).rstrip())

        threading.excepthook = _thread_excepthook

        class _StdWriter:
            def __init__(self, q):
                self.q = q
                self.buf = ""

            def write(self, s):
                self.buf += s
                while "\n" in self.buf:
                    line, self.buf = self.buf.split("\n", 1)
                    if line:
                        self.q.put(line)
                    else:
                        self.q.put("")

            def flush(self):
                pass

        sys.stdout = _StdWriter(self._log_q)
        sys.stderr = _StdWriter(self._log_q)

        root = ttk.Frame(self, padding=8)
        root.pack(fill=tk.BOTH, expand=True)

        # top bar with connection controls and telemetry
        topbar = ttk.Frame(root)
        topbar.pack(fill=tk.X)
        self.topbar_left = ttk.Frame(topbar)
        self.topbar_left.pack(side=tk.LEFT, padx=4, pady=4)
        self.telemetry_frame = ttk.Frame(topbar)
        self.telemetry_frame.pack(side=tk.RIGHT, padx=6, pady=4)

        # two-column body: left controls, right plot/console
        body = ttk.Frame(root)
        body.pack(fill=tk.BOTH, expand=True)
        self.left_pane = ttk.Frame(body)
        self.left_pane.pack(side=tk.LEFT, fill=tk.Y, padx=(4,2), pady=(0,4))

        self.right_box = ttk.Frame(body, padding=6)
        self.right_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(2,4), pady=(0,4))

        # --- Connection bar in top-left ---
        ttk.Label(self.topbar_left, text="Interface:").pack(side=tk.LEFT)
        self.iface_var = tk.StringVar(value="radio")
        self.iface_combo = ttk.Combobox(self.topbar_left, textvariable=self.iface_var, width=16, values=["radio"], state="readonly")
        self.iface_combo.pack(side=tk.LEFT, padx=6)
        self.iface_combo.bind("<<ComboboxSelected>>", lambda *_: self._rebuild_uri())

        self.btn_conn = ttk.Button(self.topbar_left, text="Connect", command=self.on_connect)
        self.btn_disc = ttk.Button(self.topbar_left, text="Disconnect", command=self.on_disconnect, state=tk.DISABLED)
        self.btn_scan = ttk.Button(self.topbar_left, text="Scan", command=self._on_scan)
        self.btn_conn.pack(side=tk.LEFT, padx=(6,0)); self.btn_disc.pack(side=tk.LEFT, padx=(6,0)); self.btn_scan.pack(side=tk.LEFT, padx=(6,0))

        ttk.Label(self.topbar_left, text="Channel:").pack(side=tk.LEFT, padx=(12,0))
        self.chan_var = tk.IntVar(value=99)
        ttk.Spinbox(self.topbar_left, from_=0, to=125, width=5, textvariable=self.chan_var, command=self._rebuild_uri).pack(side=tk.LEFT, padx=(4,12))
        ttk.Label(self.topbar_left, text="Bitrate:").pack(side=tk.LEFT)
        self.rate_var = tk.StringVar(value="2M")
        self.rate_combo = ttk.Combobox(self.topbar_left, textvariable=self.rate_var, width=6, values=list(RADIO_BITRATES), state="readonly")
        self.rate_combo.pack(side=tk.LEFT, padx=(4,12))
        self.rate_combo.bind("<<ComboboxSelected>>", lambda *_: self._rebuild_uri())
        ttk.Label(self.topbar_left, text="Address:").pack(side=tk.LEFT)
        self.addr_var = tk.StringVar(value="0xE7E7E7E7E7")
        addr_e = ttk.Entry(self.topbar_left, width=14, textvariable=self.addr_var); addr_e.pack(side=tk.LEFT, padx=(4,12))
        addr_e.bind("<KeyRelease>", lambda *_: self._rebuild_uri())

        # MRU list
        self.uri_var = tk.StringVar()
        self.mru_var = tk.StringVar()
        self.mru_combo = ttk.Combobox(self.topbar_left, textvariable=self.mru_var, width=40, values=self.cfg.recent_uris, state="readonly")
        self.mru_combo.pack(side=tk.LEFT)
        self.mru_combo.bind("<<ComboboxSelected>>", self._on_select_mru)
        self._rebuild_uri()

        # Telemetry labels on top-right
        style = ttk.Style()
        style.configure("TelemetryBold.TLabel", foreground="blue", font=("TkDefaultFont", 10, "bold"))
        self.lbl_latency = ttk.Label(self.telemetry_frame, text="Latency: -- ms", style="TelemetryBold.TLabel")
        self.lbl_latency.grid(row=0, column=0, padx=(0,8))
        self.lbl_rssi = ttk.Label(self.telemetry_frame, text="RSSI: --", style="TelemetryBold.TLabel")
        self.lbl_rssi.grid(row=0, column=1, padx=(0,8))
        self.lbl_vbat = ttk.Label(self.telemetry_frame, text="VBAT: -- V", style="TelemetryBold.TLabel")
        self.lbl_vbat.grid(row=0, column=2, padx=(0,8))
        self.lbl_p95 = ttk.Label(self.telemetry_frame, text="P95: -- ms", style="TelemetryBold.TLabel")
        self.lbl_p95.grid(row=1, column=0, padx=(0,8))
        self.lbl_p99 = ttk.Label(self.telemetry_frame, text="P99: -- ms", style="TelemetryBold.TLabel")
        self.lbl_p99.grid(row=1, column=1, padx=(0,8))
        self.lbl_miss = ttk.Label(self.telemetry_frame, text="Miss: -- %", style="TelemetryBold.TLabel")
        self.lbl_miss.grid(row=1, column=2)

        # --- Main area ---
        self.notebook = ttk.Notebook(self.left_pane)
        tab_controls = ttk.Frame(self.notebook)
        tab_logparam = ttk.Frame(self.notebook)
        self.notebook.add(tab_controls, text="Controls")
        self.notebook.add(tab_logparam, text="Log Parameter")
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        # Vicon plot state
        self.trail_buf = deque(maxlen=20000)   # (t, x, y, z)
        self.trail_secs_var = tk.IntVar(value=5)    # Show trail (seconds)
        self.decimate_var   = tk.IntVar(value=1)    # 1 = no decimation
        self._quiver_artist = None
        self._trail_artist  = None
        self._point_artist  = None
        self._last_draw_ts = 0.0
        self._last_kpi_update = 0.0
        self.vx_var = tk.StringVar(value="--")
        self.vy_var = tk.StringVar(value="--")
        self.vz_var = tk.StringVar(value="--")
        self.vrx_var = tk.StringVar(value="--")
        self.vry_var = tk.StringVar(value="--")
        self.vrz_var = tk.StringVar(value="--")
        self.vrx_rate_var = tk.StringVar(value="20")
        self._vrx_running = False
        self._vrx_thread: threading.Thread | None = None
        self._vrx_sock: socket.socket | None = None
        self._vrx_lock = threading.Lock()
        self._last_vicon: tuple[float, float, float, float, float, float] | None = None
        # latest Vicon sample (position only) and timestamp
        self._vicon_xyz: tuple[float, float, float] | None = None
        self._vicon_rpy: tuple[float, float, float] | None = None
        self._vicon_ts: float = 0.0

        # Build tabs
        self._build_controls_tab(tab_controls)
        self._build_log_param_tab(tab_logparam)

        # Right column contents: 3D plot and console
        self.plot_container = ttk.Frame(self.right_box)
        self.plot_container.pack(fill=tk.BOTH, expand=True)
        self.plot_container.grid_rowconfigure(0, weight=1)
        self.plot_container.grid_columnconfigure(0, weight=1)

        self.azim_var = tk.DoubleVar(value=-60.0)
        self.elev_var = tk.DoubleVar(value=20.0)
        self.scale_elev = ttk.Scale(
            self.plot_container, from_=90.0, to=-90.0,
            variable=self.elev_var, orient=tk.VERTICAL,
            command=self._update_view,
        )
        self.scale_elev.grid(row=0, column=1, sticky="ns")
        self.scale_azim = ttk.Scale(
            self.plot_container, from_=-180.0, to=180.0,
            variable=self.azim_var, orient=tk.HORIZONTAL,
            command=self._update_view,
        )
        self.scale_azim.grid(row=1, column=0, sticky="ew")

        self.console_frame = ttk.Labelframe(self.right_box, text="Console", padding=6, height=120)
        self.console_frame.pack(fill=tk.X, side=tk.BOTTOM)
        self.console_frame.propagate(False)
        self.console = scrolledtext.ScrolledText(self.console_frame, height=8, state="disabled")
        self.console.pack(fill=tk.BOTH, expand=True)

        self._fig = None
        self.ax3d = None
        self.canvas3d = None
        self._update_view()

        # timers
        self.after(UI_TICK_MS, self._ui_tick)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # background UDP reader
        self.udp = UDPInput(self.state_model)
        self.udp.start()

        self.setpoints: SetpointLoop|None = None
        self.pwm_loop: PWMSetpointLoop|None = None
        self.pwm_udp: PWMUDPReceiver|None = None

    # ---- helpers ----
    def log(self, s: str):
        self.console.configure(state='normal'); self.console.insert('end', s + '\n')
        self.console.configure(state='disabled'); self.console.see('end')

    def _log(self, s: str):
        """Thread-safe logger used by background threads."""
        try:
            self.enqueue_log(s)
        except Exception:
            pass

    def _is_4pid_mode_active(self) -> bool:
        """Return True iff the 4-PID Controls tab is selected."""
        try:
            return self.ctrl_nb.tab(self.ctrl_nb.select(), "text") == "4-PID Controls"
        except Exception:
            return False

    # ---- Controls tab ----
    def _build_controls_tab(self, parent):
        # XYZ → MATLAB
        xyz = ttk.Labelframe(parent, text="XYZ → MATLAB (UDP 51002)", padding=8); xyz.pack(fill=tk.X)
        row = ttk.Frame(xyz); row.pack(fill=tk.X, pady=(0,6))
        ttk.Label(row, text="X").grid(row=0, column=0, padx=(0,4))
        ttk.Label(row, text="Y").grid(row=0, column=2, padx=(12,4))
        ttk.Label(row, text="Z").grid(row=0, column=4, padx=(12,4))
        self.x_var = tk.StringVar(value="0.0"); self.y_var = tk.StringVar(value="0.0"); self.z_var = tk.StringVar(value="0.0")
        ttk.Entry(row, width=10, textvariable=self.x_var).grid(row=0, column=1)
        ttk.Entry(row, width=10, textvariable=self.y_var).grid(row=0, column=3)
        ttk.Entry(row, width=10, textvariable=self.z_var).grid(row=0, column=5)

        row2 = ttk.Frame(xyz); row2.pack(fill=tk.X)
        ttk.Label(row2, text="Rate (Hz)").pack(side=tk.LEFT)
        self.xyz_hz_var = tk.IntVar(value=20)
        ttk.Spinbox(row2, from_=1, to=500, width=6, textvariable=self.xyz_hz_var).pack(side=tk.LEFT, padx=(4,12))
        self.btn_xyz_start = ttk.Button(row2, text="Start setpoints → MATLAB", command=self.start_coords)
        self.btn_xyz_stop  = ttk.Button(row2, text="Stop setpoints", state=tk.DISABLED, command=self.stop_coords)
        self.btn_xyz_start.pack(side=tk.LEFT)
        self.btn_xyz_stop.pack(side=tk.LEFT, padx=6)
        self.btn_use_vicon = ttk.Button(
            row2, text="Current XYZ", command=self._on_use_vicon, state=tk.DISABLED
        )
        self.btn_use_vicon.pack(side=tk.LEFT, padx=6)

        # Vicon (UDP 8889)
        vgp = ttk.Labelframe(parent, text="Vicon (UDP 8889)", padding=8)
        vgp.pack(fill=tk.BOTH, pady=(8,0))

        row_top = ttk.Frame(vgp); row_top.pack(fill=tk.X, pady=(0,6))
        ttk.Label(row_top, text="X").pack(side=tk.LEFT); ttk.Entry(row_top, width=10, textvariable=self.vx_var, state="readonly").pack(side=tk.LEFT, padx=(0,8))
        ttk.Label(row_top, text="Y").pack(side=tk.LEFT); ttk.Entry(row_top, width=10, textvariable=self.vy_var, state="readonly").pack(side=tk.LEFT, padx=(0,8))
        ttk.Label(row_top, text="Z").pack(side=tk.LEFT); ttk.Entry(row_top, width=10, textvariable=self.vz_var, state="readonly").pack(side=tk.LEFT, padx=(0,12))
        ttk.Label(row_top, text="Rate (Hz)").pack(side=tk.LEFT, padx=(0,4))
        vcmd = (self.register(lambda P: P.isdigit() or P == ""), "%P")
        self.vrx_rate_entry = ttk.Spinbox(
            row_top,
            from_=1,
            to=500,
            width=6,
            textvariable=self.vrx_rate_var,
            validate="key",
            validatecommand=vcmd,
        )
        self.vrx_rate_entry.pack(side=tk.LEFT, padx=(0,12))
        self.btn_vrx_start = ttk.Button(row_top, text="Start", command=self._vrx_start)
        self.btn_vrx_stop  = ttk.Button(row_top, text="Stop",  command=self._vrx_stop, state=tk.DISABLED)
        self.btn_vrx_start.pack(side=tk.LEFT); self.btn_vrx_stop.pack(side=tk.LEFT, padx=(6,0))

        row_rot = ttk.Frame(vgp); row_rot.pack(fill=tk.X, pady=(0,6))
        ttk.Label(row_rot, text="Rx").pack(side=tk.LEFT); ttk.Entry(row_rot, width=10, textvariable=self.vrx_var, state="readonly").pack(side=tk.LEFT, padx=(0,8))
        ttk.Label(row_rot, text="Ry").pack(side=tk.LEFT); ttk.Entry(row_rot, width=10, textvariable=self.vry_var, state="readonly").pack(side=tk.LEFT, padx=(0,8))
        ttk.Label(row_rot, text="Rz").pack(side=tk.LEFT); ttk.Entry(row_rot, width=10, textvariable=self.vrz_var, state="readonly").pack(side=tk.LEFT, padx=(0,12))

        brow = ttk.Frame(vgp); brow.pack(fill=tk.X)
        ttk.Label(brow, text="X").grid(row=0, column=0, padx=(0,4))
        self.bx0 = tk.StringVar(value="-1000"); self.bx1 = tk.StringVar(value="1000")
        ttk.Entry(brow, width=8, textvariable=self.bx0).grid(row=0, column=1)
        ttk.Entry(brow, width=8, textvariable=self.bx1).grid(row=0, column=2)
        ttk.Label(brow, text="Y").grid(row=0, column=3, padx=(12,4))
        self.by0 = tk.StringVar(value="-1000"); self.by1 = tk.StringVar(value="1000")
        ttk.Entry(brow, width=8, textvariable=self.by0).grid(row=0, column=4)
        ttk.Entry(brow, width=8, textvariable=self.by1).grid(row=0, column=5)
        ttk.Label(brow, text="Z").grid(row=0, column=6, padx=(12,4))
        self.bz0 = tk.StringVar(value="0"); self.bz1 = tk.StringVar(value="1500")
        ttk.Entry(brow, width=8, textvariable=self.bz0).grid(row=0, column=7)
        ttk.Entry(brow, width=8, textvariable=self.bz1).grid(row=0, column=8)
        ttk.Button(brow, text="Apply", command=self._on_apply_bounds).grid(row=0, column=9, padx=(12,0))

        optrow = ttk.Frame(vgp); optrow.pack(fill=tk.X, pady=(4,0))
        ttk.Label(optrow, text="Show trail (s)").pack(side=tk.LEFT, padx=(0,4))
        ttk.Spinbox(optrow, from_=0, to=120, width=4, textvariable=self.trail_secs_var).pack(side=tk.LEFT)
        ttk.Label(optrow, text="Decimate").pack(side=tk.LEFT, padx=(12,4))
        ttk.Combobox(optrow, width=4, state="readonly", values=["1","2","5","10"], textvariable=self.decimate_var).pack(side=tk.LEFT)

        # Arm toggle, restart and state labels
        armrow = ttk.Frame(parent)
        self.btn_arm = ttk.Button(armrow, text="Arm", command=self._on_arm_toggle, state=tk.DISABLED)
        self.btn_arm.pack(side=tk.LEFT)
        self.lbl_arm_state = ttk.Label(armrow, text="—", foreground="gray")
        self.lbl_arm_state.pack(side=tk.LEFT, padx=(6,0))
        self.btn_restart = ttk.Button(armrow, text="Restart", command=self._on_restart)
        self.btn_restart.pack(side=tk.LEFT, padx=(6,0))
        self.lbl_restart = ttk.Label(armrow, text="—", foreground="gray")
        self.lbl_restart.pack(side=tk.LEFT, padx=(6,0))
        armrow.pack(anchor=tk.W, pady=(8,0))

        # 控制模式分頁（2-PID / 4-PID）
        self.ctrl_nb = ttk.Notebook(parent)
        tab2 = ttk.Frame(self.ctrl_nb)   # 2-PID Controls
        tab4 = ttk.Frame(self.ctrl_nb)   # 4-PID Controls
        self.ctrl_nb.add(tab2, text="2-PID Controls")
        self.ctrl_nb.add(tab4, text="4-PID Controls")
        self.ctrl_nb.pack(fill=tk.X, pady=(8,0))
        self.ctrl_nb.bind("<<NotebookTabChanged>>", self._on_ctrl_tab_change)

        # --- 2-PID tab (RPYT setpoints) ---
        fc = ttk.Labelframe(tab2, text="Flight Control", padding=8)
        fc.pack(fill=tk.X)
        self.btn_sp_start = ttk.Button(fc, text="Start", command=self._sp_start)
        self.btn_sp_stop  = ttk.Button(fc, text="Stop",  command=self._sp_stop, state=tk.DISABLED)
        self.btn_sp_start.pack(side=tk.LEFT)
        self.btn_sp_stop.pack(side=tk.LEFT, padx=(6,12))
        self.sp_hz_var = tk.IntVar(value=100)
        ttk.Label(fc, text="Rate (Hz)").pack(side=tk.LEFT)
        ttk.Spinbox(fc, from_=1, to=1000, width=6, textvariable=self.sp_hz_var).pack(side=tk.LEFT, padx=(4,0))
        self.lbl_sp_actual = ttk.Label(fc, text="Actual: -- Hz")
        self.lbl_sp_actual.pack(side=tk.LEFT, padx=(12,0))

        self.lbl_rpyt = ttk.Label(fc, text="R: 0.00°  P: 0.00°  Y: 0.00°/s  T: 0")
        self.lbl_rpyt.pack(side=tk.LEFT, padx=(12,0))
        ttk.Label(fc, text="Throttle offset").pack(side=tk.LEFT, padx=(12,0))
        self.offset_var = tk.StringVar(value=str(self.cfg.throttle_offset))
        ttk.Spinbox(fc, from_=0, to=65535, width=8,
                    textvariable=self.offset_var).pack(side=tk.LEFT, padx=(4,0))
        self.offset_var.trace_add("write", self._on_offset_change)

        # --- 4-PID tab (PWM direct) ---
        pwmf = ttk.Labelframe(tab4, text="PWM Control", padding=8)
        pwmf.pack(fill=tk.X)
        rowm = ttk.Frame(pwmf); rowm.pack(fill=tk.X)
        self.pwm_mode_var = tk.StringVar(value="manual")
        ttk.Radiobutton(rowm, text="Manual entry", variable=self.pwm_mode_var,
                        value="manual", command=self._on_pwm_mode_change).pack(side=tk.LEFT)
        ttk.Radiobutton(rowm, text="UDP 8888", variable=self.pwm_mode_var,
                        value="udp", command=self._on_pwm_mode_change).pack(side=tk.LEFT, padx=(12,0))
        rowp = ttk.Frame(pwmf); rowp.pack(fill=tk.X, pady=(4,0))
        self.pwm_vars = [tk.StringVar(value="0") for _ in range(4)]
        self.pwm_entries: list[ttk.Entry] = []
        for i, var in enumerate(self.pwm_vars):
            ttk.Label(rowp, text=f"m{i+1}").grid(row=0, column=2*i, padx=(0,4))
            e = ttk.Entry(rowp, width=8, textvariable=var)
            e.grid(row=0, column=2*i+1, padx=(0,8))
            self.pwm_entries.append(e)
        self._on_pwm_mode_change()
        rowb = ttk.Frame(pwmf); rowb.pack(fill=tk.X, pady=(8,0))
        self.btn_pwm_start = ttk.Button(rowb, text="Start", command=self._pwm_start)
        self.btn_pwm_stop  = ttk.Button(rowb, text="Stop", command=self._pwm_stop, state=tk.DISABLED)
        self.btn_pwm_start.pack(side=tk.LEFT)
        self.btn_pwm_stop.pack(side=tk.LEFT, padx=(6,12))
        self.pwm_hz_var = tk.IntVar(value=200)
        ttk.Label(rowb, text="Rate (Hz)").pack(side=tk.LEFT)
        ttk.Spinbox(rowb, from_=1, to=1000, width=6, textvariable=self.pwm_hz_var).pack(side=tk.LEFT, padx=(4,0))
        self.lbl_pwm_actual = ttk.Label(rowb, text="Actual: -- Hz")
        self.lbl_pwm_actual.pack(side=tk.LEFT, padx=(12,0))

        # Safety（共用）
        safe = ttk.Labelframe(parent, text="Safety", padding=8); safe.pack(fill=tk.X, pady=(8,0))
        ttk.Button(safe, text="Emergency stop", command=self.emergency_stop).pack(side=tk.LEFT)
        ttk.Button(safe, text="Land (ramp down)", command=self.land).pack(side=tk.LEFT, padx=8)

    def _apply_axes_bounds(self):
        if not self.ax3d:
            return
        try:
            bx0 = float(self.bx0.get()); bx1 = float(self.bx1.get())
            by0 = float(self.by0.get()); by1 = float(self.by1.get())
            bz0 = float(self.bz0.get()); bz1 = float(self.bz1.get())
            if bx0 == bx1: bx1 = bx0 + 1.0
            if by0 == by1: by1 = by0 + 1.0
            if bz0 == bz1: bz1 = bz0 + 1.0
            self.ax3d.set_xlim(min(bx0,bx1), max(bx0,bx1))
            self.ax3d.set_ylim(min(by0,by1), max(by0,by1))
            self.ax3d.set_zlim(min(bz0,bz1), max(bz0,bz1))
        except Exception:
            pass

    def _on_apply_bounds(self):
        self._apply_axes_bounds()
        if self.canvas3d:
            self.canvas3d.draw_idle()

    def _update_view(self, *_):
        if not self.ax3d:
            return
        az = float(self.azim_var.get())
        el = float(self.elev_var.get())
        self.ax3d.view_init(elev=el, azim=az)
        if self.canvas3d:
            self.canvas3d.draw_idle()

    def _draw_quiver(self, x, y, z, roll, pitch, yaw):
        if not self.ax3d:
            return
        if self._quiver_artist is not None:
            try: self._quiver_artist.remove()
            except Exception: pass
            self._quiver_artist = None
        cx, sx = math.cos(roll),  math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw),   math.sin(yaw)
        dx =  cy*cz
        dy =  cy*sz
        dz = -sy
        length = 0.2 * max(1.0, abs(self.ax3d.get_zlim()[1]-self.ax3d.get_zlim()[0]))/5.0
        try:
            self._quiver_artist = self.ax3d.quiver(x, y, z, dx, dy, dz, length=length, normalize=True)
        except Exception:
            self._quiver_artist = None

    def _vicon_clear(self):
        self.trail_buf.clear()
        if self._trail_artist is not None:
            try: self._trail_artist.remove()
            except Exception: pass
            self._trail_artist = None
        if self._point_artist is not None:
            try: self._point_artist.remove()
            except Exception: pass
            self._point_artist = None
        if self._quiver_artist is not None:
            try: self._quiver_artist.remove()
            except Exception: pass
            self._quiver_artist = None
        if self.canvas3d:
            self.canvas3d.draw_idle()

    def _ensure_3d_canvas(self):
        if getattr(self, "canvas3d", None):
            return
        try:
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as e:
            if not getattr(self, "_matplotlib_error_label", None):
                self._matplotlib_error_label = ttk.Label(self.plot_container, text=f"3D view unavailable: {e}")
                self._matplotlib_error_label.grid(row=0, column=0, sticky="nsew")
            return
        self._fig = Figure(figsize=(5, 4), dpi=100)
        self.ax3d = self._fig.add_subplot(111, projection="3d")
        self._fig.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
        self.ax3d.set_xlim(-1000, 1000)
        self.ax3d.set_ylim(-1000, 1000)
        self.ax3d.set_zlim(0, 1500)
        self.canvas3d = FigureCanvasTkAgg(self._fig, master=self.plot_container)
        self.canvas3d.draw_idle()
        self.canvas3d.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        self._last_draw_ts = 0.0
        self._update_view()

    def _on_tab_changed(self, evt):
        tab_text = self.notebook.tab(self.notebook.select(), "text") or ""
        if "Vicon" in tab_text or "3D" in tab_text:
            self._ensure_3d_canvas()

    def _safe_rate_hz(self, default=30):
        try:
            v = self.vrx_rate_var.get()
            return max(1, int(float(v)))
        except Exception:
            return default

    def _now(self) -> float:
        return perf_counter()

    def _sleep(self, t: float) -> None:
        time.sleep(t)

    def _vrx_start(self):
        sock = None
        try:
            self._ensure_3d_canvas()
            if self._vrx_running:
                return
            hz = self._safe_rate_hz(30)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 8889))
            sock.setblocking(False)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            except Exception:
                pass
            self._vrx_sock = sock
            self._vrx_running = True
            self._vrx_thread = threading.Thread(target=self._vrx_loop, args=(hz,), daemon=True)
            self._vrx_thread.start()
            self.btn_vrx_start.configure(state=tk.DISABLED)
            self.btn_vrx_stop.configure(state=tk.NORMAL)
            self.vrx_rate_entry.configure(state=tk.DISABLED)
            self._log("Vicon UDP receive started")
        except Exception as e:
            try:
                if sock:
                    sock.close()
            except Exception:
                pass
            self._vrx_running = False
            self.vrx_rate_entry.configure(state=tk.NORMAL)
            self._log(f"Vicon receive start failed: {e}")

    def _vrx_stop(self):
        try:
            if not self._vrx_running:
                return
            self._vrx_running = False
            sock = self._vrx_sock
            if sock:
                try:
                    sock.close()
                except OSError as e:
                    if getattr(e, "winerror", None) != 10038:
                        raise
                except Exception:
                    pass
            self._vrx_sock = None
            t = self._vrx_thread
            if t and t.is_alive():
                t.join(timeout=0.5)
            self._vrx_thread = None
            self.btn_vrx_start.configure(state=tk.NORMAL)
            self.btn_vrx_stop.configure(state=tk.DISABLED)
            self.vrx_rate_entry.configure(state=tk.NORMAL)
            self._log("Vicon UDP receive stopped")
        except Exception as e:
            self._log(f"Vicon receive stop failed: {e}")

    def _vrx_loop(self, hz: int):
        sock = self._vrx_sock
        period = 1.0 / max(1, hz)
        last_err = 0.0
        fmt_once = None
        self._log("[Vicon] 8889 listener started")
        while self._vrx_running and sock:
            try:
                while True:
                    data, _ = sock.recvfrom(2048)
                    x, y, z, rx, ry, rz, fmt = decode_vicon_be(data)
                    if fmt_once is None:
                        fmt_once = fmt
                        self._log(f"[Vicon] 8889 first packet len={len(data)} bytes, fmt={fmt_once}")
                    with self._vrx_lock:
                        self._last_vicon = (x, y, z, rx, ry, rz)
                        self.trail_buf.append((time.time(), x, y, z))
                        self._vicon_xyz = (x, y, z)
                        self._vicon_rpy = (rx, ry, rz)
                        self._vicon_ts = self._now()
            except BlockingIOError:
                pass
            except OSError:
                break
            except Exception as e:
                now = time.time()
                if now - last_err >= 1.0:
                    self._log(f"[Vicon] decode error: {e}")
                    last_err = now
            self._sleep(period)

        try:
            if sock:
                sock.close()
        except Exception:
            pass
        self._log("[Vicon] 8889 listener stopped")

    def _get_latest_vicon_xyz(self, max_age: float = 1.0):
        """Return (x, y, z) tuple if last Vicon sample is fresh; else None."""
        with self._vrx_lock:
            xyz = self._vicon_xyz
            ts = self._vicon_ts
        if xyz and (self._now() - ts) <= max_age:
            return xyz
        return None

    def _on_use_vicon(self):
        vals = self._get_latest_vicon_xyz()
        if not vals:
            return
        x, y, z = vals
        try:
            self.x_var.set(f"{x:.3f}")
            self.y_var.set(f"{y:.3f}")
            self.z_var.set(f"{z:.3f}")
        except Exception:
            pass

    # ---- Log Parameter tab (alias used in __init__) ----
    def _build_log_param_tab(self, parent):
        """Compatibility alias so __init__ can call either name."""
        return self._build_logs_tab(parent)

    # ---- Log Parameter tab (implementation) ----
    def _build_logs_tab(self, parent):
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(0,6))
        self.btn_clear_console = ttk.Button(toolbar, text="Clear", command=self._on_clear_console)
        self.btn_clear_console.pack(side="left")

        # 建立打勾圖案
        check_font = ("Segoe UI Symbol", 12)
        self.check_symbol = tk.Label(parent, text="✔", font=check_font, fg="green")
        self.empty_symbol = tk.Label(parent, text="  ", font=check_font)

        # 嘗試從 ttk.Style 取得背景色，若取不到則用系統預設顏色
        style = ttk.Style()
        try:
            bg_color = style.lookup("TFrame", "background")
            if not bg_color:
                bg_color = self.cget("background")
        except Exception:
            bg_color = self.cget("background")

        # Mapping of internal log keys -> checkbox labels displayed in the UI
        # Keys are stored in lowercase for easy lookup later when samples are
        # appended.  Using a tuple (key, label) keeps display text unchanged
        # while normalising access.
        log_items = [
            ("x", "X"), ("y", "Y"), ("z", "Z"),
            ("rot_x", "Rot_X"), ("rot_y", "Rot_Y"), ("rot_z", "Rot_Z"),
            ("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw"),
            ("thrust", "Thrust"),
        ]
        self.log_opts = {}

        for key, label in log_items:
            var = tk.BooleanVar(value=False)
            self.log_opts[key] = var
            cb = tk.Checkbutton(
                parent,
                text=label,
                variable=var,
                font=("Segoe UI", 10),
                selectimage=None,
                indicatoron=True,
                bg=bg_color,                # 勾選框背景
                activebackground=bg_color   # 滑鼠移上去背景
            )
            cb.pack(anchor="w", pady=1)

    def _on_clear_console(self):
        try:
            self.console.configure(state="normal")
            self.console.delete("1.0", "end")
            self.console.configure(state="disabled")
        except Exception:
            pass

    # ---- connection ----
    def on_connect(self):
        """Blocking connect (keep console lines)."""
        uri = (self.uri_var.get() or "").strip()
        if not uri:
            self.log("No URI"); return
        self.log("Connecting...")
        try:
            from .link import LinkManager
            self.link = LinkManager(self.state_model, uri); self.link.connect()
            self.cf = self.link.cf
            self.link.detect_platform_and_arm_param()
            plat = "Bolt" if self.link.is_bolt else "Unknown"
            self.log(f"Platform: {plat}; arming param: {self.link.arm_param or 'not found'}")
            if not self.link.arm_param:
                self.log("[ARM] no arming param")
            self.btn_conn.configure(state=tk.DISABLED); self.btn_disc.configure(state=tk.NORMAL)
            self.btn_arm.configure(state=tk.DISABLED)
            self.lbl_arm_state.configure(text="—")
            uris = [u for u in [uri] + self.cfg.recent_uris if u and u != uri]
            self.cfg.recent_uris = [uri] + uris[:7]; save_config(self.cfg)
            self.mru_combo.configure(values=self.cfg.recent_uris)
            self.log(f"Connected: {uri}")
        except Exception as e:
            self.log(f"Connect failed: {e}")

    def on_disconnect(self):
        try:
            if self.setpoints: self.setpoints.stop(); self.setpoints=None
            if self.pwm_loop: self.pwm_loop.stop(); self.pwm_loop=None
            if self.pwm_udp: self.pwm_udp.stop(); self.pwm_udp=None
            if getattr(self, "_vrx_running", False):
                try: self._vrx_stop()
                except Exception: pass
            if self.link: self.link.disconnect(); self.link=None
            self.cf = None
            self.btn_conn.configure(state=tk.NORMAL); self.btn_disc.configure(state=tk.DISABLED)
            self.btn_arm.configure(state=tk.DISABLED)
            self.lbl_arm_state.configure(text="—")
            if hasattr(self, "lbl_status"):
                self.lbl_status.configure(text=" | Disconnected")
            self.log("Disconnected")
        except Exception as e:
            self.log(f"Disconnect error: {e}")

    def _refresh_control_states(self) -> None:
        if self.link:
            self.btn_conn.config(state=tk.DISABLED)
            self.btn_disc.config(state=tk.NORMAL)
            self.btn_arm.config(state=tk.NORMAL if self.link.arm_param else tk.DISABLED)
        else:
            self.btn_conn.config(state=tk.NORMAL)
            self.btn_disc.config(state=tk.DISABLED)
            self.btn_arm.config(state=tk.DISABLED)
        self.btn_restart.config(state=tk.NORMAL)
        self.btn_scan.config(state=tk.NORMAL)
        self.btn_sp_start.config(state=tk.NORMAL)
        self.btn_sp_stop.config(state=tk.DISABLED)
        self.btn_pwm_start.config(state=tk.NORMAL)
        self.btn_pwm_stop.config(state=tk.DISABLED)
        self.btn_xyz_start.config(state=tk.NORMAL)
        self.btn_xyz_stop.config(state=tk.DISABLED)

    def _set_controls_enabled(self, enabled: bool) -> None:
        widgets = [
            self.btn_arm,
            self.btn_restart,
            self.btn_conn,
            self.btn_disc,
            self.btn_scan,
            self.btn_sp_start,
            self.btn_sp_stop,
            self.btn_pwm_start,
            self.btn_pwm_stop,
            self.btn_xyz_start,
            self.btn_xyz_stop,
        ]
        if not enabled:
            for w in widgets:
                try: w.config(state=tk.DISABLED)
                except Exception: pass
        else:
            self._refresh_control_states()

    # ---- XYZ → MATLAB ----
    def start_coords(self):
        if self._coords_running: return
        self._coords_running = True
        self.btn_xyz_start.configure(state=tk.DISABLED); self.btn_xyz_stop.configure(state=tk.NORMAL)
        self.log("Start sending XYZ to MATLAB")
        t = threading.Thread(target=self._coords_loop, daemon=True); t.start(); self._coords_thread = t

    def stop_coords(self):
        if not self._coords_running: return
        self._coords_running = False
        self.btn_xyz_start.configure(state=tk.NORMAL); self.btn_xyz_stop.configure(state=tk.DISABLED)
        self.log("Stop sending XYZ")

    def _coords_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            while self._coords_running:
                try:
                    x = float(self.x_var.get() or 0.0); y = float(self.y_var.get() or 0.0); z = float(self.z_var.get() or 0.0)
                    sock.sendto(struct.pack("<3f", x, y, z), ("127.0.0.1", UDP_COORD_PORT))
                except Exception:
                    pass
                hz = max(1, int(self.xyz_hz_var.get() or 20))
                time.sleep(1.0/float(hz))
        finally:
            sock.close()

    # ---- Setpoint loop (CF) ----
    def _sp_start(self):
        if self.pwm_loop and self.pwm_loop.is_running():
            self._pwm_stop()
        if not self.link:
            self.log("Not connected"); return
        if not self.setpoints:
            self.setpoints = SetpointLoop(self.state_model, self.link, rate_hz=int(self.sp_hz_var.get()))
        self.setpoints.set_rate(int(self.sp_hz_var.get()))
        self.setpoints.start()
        self.btn_sp_start.configure(state=tk.DISABLED); self.btn_sp_stop.configure(state=tk.NORMAL)
        self.log(f"Setpoint loop started @ {self.setpoints.get_rate()} Hz")

    def _sp_stop(self):
        if self.setpoints: self.setpoints.stop()
        self.btn_sp_start.configure(state=tk.NORMAL); self.btn_sp_stop.configure(state=tk.DISABLED)
        self.log("Setpoint loop stopped")


    # ---- PWM loop ----
    def _pwm_start(self):
        if self.setpoints and self.setpoints.is_running():
            self._sp_stop()
        if not self.link:
            self.log("Not connected"); return
        if not self.pwm_loop:
            self.pwm_loop = PWMSetpointLoop(self.link)
        mode = self.pwm_mode_var.get()
        if mode == "manual":
            pwm = []
            for var in self.pwm_vars:
                try:
                    v = int(var.get())
                except Exception:
                    v = 0
                pwm.append(max(0, min(65535, v)))
            self.pwm_loop.set_mode("manual")
            self.pwm_loop.set_manual_pwm(pwm)
        else:
            if not self.pwm_udp:
                self.pwm_udp = PWMUDPReceiver(port=8888)
            self.pwm_udp.start()
            self.pwm_loop.attach_udp(self.pwm_udp)
            self.pwm_loop.set_mode("udp")
        self.pwm_loop.set_rate(int(self.pwm_hz_var.get()))
        self.pwm_loop.start()
        self.btn_pwm_start.configure(state=tk.DISABLED); self.btn_pwm_stop.configure(state=tk.NORMAL)
        self.log("4PID loop started")
        self._on_pwm_mode_change()

    def _pwm_stop(self):
        if self.pwm_loop: self.pwm_loop.stop()
        if self.pwm_mode_var.get() == "udp" and self.pwm_udp:
            self.pwm_udp.stop()
        self.btn_pwm_start.configure(state=tk.NORMAL); self.btn_pwm_stop.configure(state=tk.DISABLED)
        self.log("4PID loop stopped")
        self._on_pwm_mode_change()
    def _on_pwm_mode_change(self, *_):
        """Switch m1~m4 Entry state based on selected PWM mode."""
        mode = (self.pwm_mode_var.get() or "manual").lower()
        state = "normal" if mode == "manual" else "readonly"
        for e in getattr(self, "pwm_entries", []):
            try:
                e.configure(state=state)
            except Exception:
                pass

    def _on_offset_change(self, *_):
        try:
            val = int(self.offset_var.get())
        except Exception:
            return
        val = max(0, min(65535, val))
        with self.state_model.lock:
            self.state_model.throttle_offset = val
        self.cfg.throttle_offset = val
        save_config(self.cfg)

    def _on_ctrl_tab_change(self, *_):
        if not hasattr(self, 'ctrl_nb'):
            return
        tab = self.ctrl_nb.tab(self.ctrl_nb.select(), "text")
        if tab == "2-PID Controls":
            if self.pwm_loop and self.pwm_loop.is_running():
                self._pwm_stop()
        elif tab == "4-PID Controls":
            if self.setpoints and self.setpoints.is_running():
                self._sp_stop()

    def _on_arm_toggle(self):
        if not (self.link and self.link.arm_param):
            self.log("[ARM] no arming param")
            return
        name = self.link.arm_param
        try:
            cur = self.link.get_bool_param(name, False)
            target = not cur
            ok = self.link.set_bool_param(name, target)
            armed = self.link.get_bool_param(name, False)
            if ok:
                self.log(f"[ARM] -> {'arm' if armed else 'disarm'}")
            else:
                self.log("[ARM] write failed")
            # update UI immediately
            if armed:
                self.btn_arm.config(text="Disarm")
                self.lbl_arm_state.config(text="arm")
            else:
                self.btn_arm.config(text="Arm")
                self.lbl_arm_state.config(text="disarm")
        except Exception as e:
            self.log(f"[ARM] error: {e}")

    # ---- Safety ----
    def emergency_stop(self):
        if self._is_4pid_mode_active():
            try:
                if self.pwm_loop and self.pwm_loop.is_running():
                    self._pwm_stop()
                if not self.cf:
                    self.log("Emergency stop: Not connected")
                    return
                for i in range(4):
                    try:
                        self.cf.param.set_value(f"motorPowerSet.m{i+1}", "0")
                    except Exception as e:
                        self.log(f"Emergency stop m{i+1} error: {e}")
                if getattr(self, "pwm_vars", None):
                    for var in self.pwm_vars:
                        try:
                            var.set("0")
                        except Exception:
                            pass
                self.log("Emergency stop (4-PID): motors set to 0")
            except Exception as e:
                self.log(f"Emergency stop failed: {e}")
        else:
            try:
                if self.link: self.link.send_setpoint(0.0, 0.0, 0.0, 0)
                self.log("Emergency stop (RPYT=0,0,0,0) sent")
            except Exception as e:
                self.log(f"Emergency stop failed: {e}")

    def land(self):
        def _ramp_2pid():
            try:
                if not self.link: self.log("Not connected"); return
                steps, T = 20, 2.5; thrust = 48000
                for i in range(steps):
                    level = int(thrust * (1 - (i+1)/steps))
                    self.link.send_setpoint(0.0, 0.0, 0.0, max(level, 0))
                    time.sleep(T/steps)
                self.link.send_setpoint(0.0, 0.0, 0.0, 0)
                self.log("Land complete")
            except Exception as e:
                self.log(f"Land error: {e}")

        def _ramp_4pid():
            try:
                if not self.cf:
                    self.log("Not connected"); return
                if self.pwm_loop and self.pwm_loop.is_running():
                    self._pwm_stop()
                vals = []
                if self.pwm_loop and getattr(self.pwm_loop, "last_pwm", None):
                    vals = self.pwm_loop.last_pwm
                else:
                    for var in getattr(self, "pwm_vars", []):
                        try:
                            vals.append(int(var.get()))
                        except Exception:
                            vals.append(0)
                if len(vals) < 4:
                    vals.extend([0]*(4-len(vals)))
                start = int(sum(vals)/4)
                steps, T = 20, 2.5
                for i in range(steps):
                    level = max(int(start * (1 - (i+1)/steps)), 0)
                    for j in range(4):
                        try:
                            self.cf.param.set_value(f"motorPowerSet.m{j+1}", str(level))
                        except Exception as e:
                            self.log(f"Land m{j+1} set error: {e}")
                    time.sleep(T/steps)
                for j in range(4):
                    try:
                        self.cf.param.set_value(f"motorPowerSet.m{j+1}", "0")
                    except Exception as e:
                        self.log(f"Land m{j+1} final error: {e}")
                if getattr(self, "pwm_vars", None):
                    for var in self.pwm_vars:
                        try:
                            var.set("0")
                        except Exception:
                            pass
                self.log("Land complete (4-PID)")
            except Exception as e:
                self.log(f"Land error: {e}")

        threading.Thread(target=_ramp_4pid if self._is_4pid_mode_active() else _ramp_2pid, daemon=True).start()

    def _on_restart(self):
        def worker():
            self.lbl_restart.config(text="Restarting…")
            self._set_controls_enabled(False)
            try:
                try:
                    if (self.setpoints and self.setpoints.is_running()) or (
                        self.pwm_loop and self.pwm_loop.is_running()
                    ):
                        self.lbl_restart.config(text="Landing…")
                        self.land()
                        time.sleep(3)
                except Exception as e:
                    self.log(f"Restart land error: {e}")

                try:
                    if self.link:
                        self.lbl_restart.config(text="Disconnecting…")
                        self.on_disconnect()
                        self._set_controls_enabled(False)
                except Exception as e:
                    self.log(f"Restart disconnect error: {e}")

                uri = (self.uri_var.get() or "").strip()
                try:
                    if uri:
                        self.lbl_restart.config(text="Powering down…")
                        from cflib.utils.power_switch import PowerSwitch

                        ps = PowerSwitch(uri)
                        ps.stm_power_down()
                        time.sleep(3)
                        self.lbl_restart.config(text="Powering up…")
                        ps.stm_power_up()
                        time.sleep(3)
                except Exception as e:
                    self.log(f"Power cycle error: {e}")

                try:
                    self.lbl_restart.config(text="Clearing ports…")
                    from .utils import clear_udp_ports_windows

                    clear_udp_ports_windows([8888, 8889])
                except Exception:
                    pass

                if self.cfg.auto_reconnect:
                    try:
                        self.lbl_restart.config(text="Reconnecting…")
                        self.on_connect()
                        self._set_controls_enabled(False)
                        self.lbl_restart.config(text="Restart complete")
                    except Exception as e:
                        self.log(f"Reconnect error: {e}")
                        self.lbl_restart.config(text="Restart done (not connected)")
                else:
                    self.lbl_restart.config(text="Restart done")
            finally:
                self._set_controls_enabled(True)
                self.after(2000, lambda: self.lbl_restart.config(text="—"))

        threading.Thread(target=worker, daemon=True).start()

    # ---- logging of selected params ----
    def _append_log_params_sample(self):
        """Append one sample to state_model.log_buf only for checked items."""
        sample = {}
        # positions from GUI
        try:
            if self.log_opts["x"].get():
                sample["x"] = float(self.x_var.get() or 0.0)
            if self.log_opts["y"].get():
                sample["y"] = float(self.y_var.get() or 0.0)
            if self.log_opts["z"].get():
                sample["z"] = float(self.z_var.get() or 0.0)
        except Exception:
            pass
        # rotations / setpoints from SharedState.rpyth (roll/pitch/yaw/thrust)
        try:
            with self.state_model.lock:
                rpyth = self.state_model.rpyth
                if self.log_opts["rot_x"].get():
                    sample["rot_x"] = float(rpyth.get("roll", 0.0))
                if self.log_opts["rot_y"].get():
                    sample["rot_y"] = float(rpyth.get("pitch", 0.0))
                if self.log_opts["rot_z"].get():
                    sample["rot_z"] = float(rpyth.get("yaw", 0.0))
                if self.log_opts.get("roll") and self.log_opts["roll"].get():
                    sample["roll"] = float(rpyth.get("roll", 0.0))
                if self.log_opts.get("pitch") and self.log_opts["pitch"].get():
                    sample["pitch"] = float(rpyth.get("pitch", 0.0))
                if self.log_opts.get("yaw") and self.log_opts["yaw"].get():
                    sample["yaw"] = float(rpyth.get("yaw", 0.0))
                if self.log_opts.get("thrust") and self.log_opts["thrust"].get():
                    sample["thrust"] = float(rpyth.get("thrust", 0.0))
        except Exception:
            pass
        if sample:
            try:
                ts = time.time()
                with self.state_model.lock:
                    self.state_model.log_buf.append((ts, sample))
            except Exception:
                pass

    # ---- UI tick ----
    def _ui_tick(self):
        while not self._log_q.empty():
            try:
                self.log(self._log_q.get())
            except Exception:
                pass
        with self._vrx_lock:
            vals = self._last_vicon
        if vals:
            try:
                x, y, z, rx, ry, rz = vals
                self.vx_var.set(f"{x:.3f}")
                self.vy_var.set(f"{y:.3f}")
                self.vz_var.set(f"{z:.3f}")
                self.vrx_var.set(f"{rx:.3f}")
                self.vry_var.set(f"{ry:.3f}")
                self.vrz_var.set(f"{rz:.3f}")
            except Exception:
                pass

        try:
            fresh = self._vrx_running and (self._now() - self._vicon_ts) <= 1.0
            state = "normal" if fresh else "disabled"
            self.btn_use_vicon.configure(state=state)
        except Exception:
            pass

        # telemetry heads-up
        with self.state_model.lock:
            v = getattr(self.state_model, 'vbat', float('nan'))
            rssi_dbm = getattr(self.state_model, 'rssi', float('nan'))
            lat_ms = getattr(self.state_model, 'latency_ms', float('nan'))
        if v == v:
            self.title(f"Crazyflie GUI — VBAT: {v:.2f} V")
            self.lbl_vbat.config(text=f"VBAT: {v:.2f} V")
        else:
            self.title("Crazyflie GUI — VBAT: -- V")
            self.lbl_vbat.config(text="VBAT: -- V")
        self.lbl_latency.config(text=f"Latency: {lat_ms:.1f} ms" if lat_ms==lat_ms else "Latency: -- ms")
        self.lbl_rssi.config(text=f"RSSI: {rssi_dbm}" if rssi_dbm==rssi_dbm else "RSSI: --")

        try:
            with self.state_model.lock:
                rpyth = dict(self.state_model.rpyth)
                offset = int(getattr(self.state_model, "throttle_offset", 40000))
            self.lbl_rpyt.config(
                text=(
                    f"R: {rpyth['roll']:.2f}°  P: {rpyth['pitch']:.2f}°  "
                    f"Y: {rpyth['yaw']:.2f}/s  T: {int(rpyth['thrust'])}"
                )
            )
            if self.offset_var.get() != str(offset):
                self.offset_var.set(str(offset))
        except Exception:
            pass

        # arm/disarm toggle state
        try:
            if self.link and self.link.arm_param:
                armed = self.link.get_bool_param(self.link.arm_param, False)
                self.btn_arm.config(state=tk.NORMAL, text="Disarm" if armed else "Arm")
                self.lbl_arm_state.config(text="arm" if armed else "disarm")
            else:
                self.btn_arm.config(state=tk.DISABLED, text="Arm")
                self.lbl_arm_state.config(text="—")
        except Exception:
            self.btn_arm.config(state=tk.DISABLED, text="Arm")
            self.lbl_arm_state.config(text="—")

        # control timing KPIs and actual rates (update <=1Hz)
        now = perf_counter()
        if now - getattr(self, "_last_kpi_update", 0.0) >= 1.0:
            try:
                loop = None
                if self.setpoints and self.setpoints.is_running():
                    loop = self.setpoints
                elif self.pwm_loop and self.pwm_loop.is_running():
                    loop = self.pwm_loop
                if loop:
                    p95, p99, miss_pct = loop.get_cached_stats(now)
                    self.lbl_p95.config(text=f"P95: {p95:.1f} ms")
                    self.lbl_p99.config(text=f"P99: {p99:.1f} ms")
                    self.lbl_miss.config(text=f"Miss: {miss_pct:.1f} %")
                else:
                    self.lbl_p95.config(text="P95: -- ms")
                    self.lbl_p99.config(text="P99: -- ms")
                    self.lbl_miss.config(text="Miss: -- %")
            except Exception:
                self.lbl_p95.config(text="P95: -- ms")
                self.lbl_p99.config(text="P99: -- ms")
                self.lbl_miss.config(text="Miss: -- %")

            try:
                if self.setpoints and self.setpoints.is_running():
                    ar = self.setpoints.get_actual_rate()
                    self.lbl_sp_actual.configure(text=f"Actual: {ar:.1f} Hz")
                else:
                    self.lbl_sp_actual.configure(text="Actual: -- Hz")
            except Exception:
                pass

            try:
                if self.pwm_loop and self.pwm_loop.is_running():
                    ar = self.pwm_loop.get_actual_rate()
                    self.lbl_pwm_actual.configure(text=f"Actual: {ar:.1f} Hz")
                else:
                    self.lbl_pwm_actual.configure(text="Actual: -- Hz")
            except Exception:
                pass
            self._last_kpi_update = now

        # update PWM display fields
        try:
            if self.pwm_mode_var.get() == "udp" and self.pwm_udp:
                vals = self.pwm_udp.get_last()
                for i, var in enumerate(self.pwm_vars):
                    var.set(str(int(vals[i])))
        except Exception:
            pass

        # record only selected parameters
        self._append_log_params_sample()
        # 3D Vicon plot
        if self.canvas3d is None and self.right_box.winfo_ismapped():
            self._ensure_3d_canvas()
        if self.canvas3d is None:
            self.after(UI_TICK_MS, self._ui_tick)
            return
        with self._vrx_lock:
            last = self._last_vicon
            trail = list(self.trail_buf)
        if last:
            x, y, z, rx, ry, rz = last
            now = time.time()
            self._apply_axes_bounds()
            if self._point_artist is None:
                self._point_artist = self.ax3d.scatter([x], [y], [z], s=12)
            else:
                self._point_artist._offsets3d = ([x], [y], [z])
            self._draw_quiver(x, y, z, roll=rx, pitch=ry, yaw=rz)
            secs = max(0, int(self.trail_secs_var.get() or 0))
            k = max(1, int(self.decimate_var.get() or 1))
            if secs > 0:
                pts = [(tx, xx, yy, zz) for (tx, xx, yy, zz) in trail if now - tx <= secs]
                if len(pts) >= 2:
                    xs = [p[1] for p in pts][::k]
                    ys = [p[2] for p in pts][::k]
                    zs = [p[3] for p in pts][::k]
                    if self._trail_artist is None:
                        self._trail_artist, = self.ax3d.plot(xs, ys, zs, linewidth=1)
                    else:
                        self._trail_artist.set_data(xs, ys)
                        self._trail_artist.set_3d_properties(zs)
                elif self._trail_artist is not None:
                    try: self._trail_artist.remove()
                    except Exception: pass
                    self._trail_artist = None
            elif self._trail_artist is not None:
                try: self._trail_artist.remove()
                except Exception: pass
                self._trail_artist = None
            if now - self._last_draw_ts >= 0.1:
                self.canvas3d.draw_idle()
                self._last_draw_ts = now

        self.after(UI_TICK_MS, self._ui_tick)

    def on_close(self):
        try:
            self._coords_running = False
            if self.setpoints: self.setpoints.stop()
            if self.pwm_loop: self.pwm_loop.stop()
            if self.pwm_udp: self.pwm_udp.stop()
            if getattr(self, "_vrx_running", False):
                try: self._vrx_stop()
                except Exception: pass
            if self.link: self.link.disconnect()
        finally:
            self.destroy()

    # ---- URI helpers / scan ----
    def _normalize_addr(self, s: str) -> str:
        s = (s or "").strip(); s = s[2:] if s.lower().startswith("0x") else s
        return s.upper()

    def _rebuild_uri(self):
        chan = int(self.chan_var.get() or 99)
        rate = self.rate_var.get() or "2M"
        addr = self._normalize_addr(self.addr_var.get() or "E7E7E7E7E7")
        self.uri_var.set(f"radio://0/{chan}/{rate}/{addr}")

    def _on_scan(self):
        """Keep console messages as requested."""
        try:
            self.log("Scanning for Crazyradio/Crazyflie...")
            import cflib.crtp
            from . import link as _link
            if not _link._cflib_inited:
                cflib.crtp.init_drivers()
                _link._cflib_inited = True
            found = cflib.crtp.scan_interfaces()
            uris = [u for (u, _d) in (found or [])]
            self.log(f"Scan complete: found {len(uris)} device(s)")
            if uris:
                self.cfg.recent_uris = uris + [u for u in self.cfg.recent_uris if u not in uris]
                save_config(self.cfg)
                self.mru_combo.configure(values=self.cfg.recent_uris)
                self.mru_var.set(uris[0]); self._on_select_mru()
        except Exception as e:
            self.log(f"Scan failed: {e}")

    def _on_select_mru(self, *_):
        uri = (self.mru_var.get() or "").strip()
        if not uri: return
        self.uri_var.set(uri)
        try:
            if uri.startswith("radio://"):
                parts = uri.split("/")
                self.chan_var.set(int(parts[3])); self.rate_var.set(parts[4]); self.addr_var.set("0x" + parts[5])
        except Exception:
            pass


if __name__ == "__main__":
    App().mainloop()
