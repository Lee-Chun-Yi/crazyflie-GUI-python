import socket, struct, threading, time, queue, logging, sys, traceback
from time import perf_counter
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from collections import deque
import math

from .models import SharedState, get_last_stream_xyz
from .config import load_config, save_config, Rates, PathCfg, PreviewCfg
from .control import UDPInput
from . import control as controller
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

def power_cycle_crazyflie(uri: str) -> None:
    """Power cycle a Crazyflie via the PowerSwitch.

    Prints status messages and toggles the STM32 power.
    """
    from cflib.utils.power_switch import PowerSwitch

    print("Power cycling Crazyflie…")
    ps = PowerSwitch(uri)
    ps.stm_power_down()
    time.sleep(1)
    ps.stm_power_up()
    time.sleep(1)
    print("Power cycle complete")

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

        self.path_loop = None
        self._last_apply_ts = 0.0

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
        # Preview artists for XYZ path apply
        self._preview_target = None
        self._preview_path = None
        self._preview_extra = None  # center or vertices
        self._cur_target_scatter = None
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
        self.after(50, self._cur_target_tick)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # background UDP reader
        self.udp = UDPInput(self.state_model)
        self.udp.start()

        self.setpoints = None
        self.pwm_loop = None

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
        self.x_var = tk.StringVar(); self.y_var = tk.StringVar(); self.z_var = tk.StringVar()
        x_entry = ttk.Entry(row, width=10, textvariable=self.x_var); x_entry.grid(row=0, column=1); x_entry.insert(0, "0")
        y_entry = ttk.Entry(row, width=10, textvariable=self.y_var); y_entry.grid(row=0, column=3); y_entry.insert(0, "0")
        z_entry = ttk.Entry(row, width=10, textvariable=self.z_var); z_entry.grid(row=0, column=5); z_entry.insert(0, "500")

        row2 = ttk.Frame(xyz); row2.pack(fill=tk.X)
        ttk.Label(row2, text="Rate (Hz)").pack(side=tk.LEFT)
        self.xyz_hz_var = tk.IntVar(value=PathCfg.DEFAULT_HZ)
        ttk.Spinbox(row2, from_=PathCfg.MIN_HZ, to=PathCfg.MAX_HZ, width=6,
                    textvariable=self.xyz_hz_var).pack(side=tk.LEFT, padx=(4,12))
        self.btn_xyz_start = ttk.Button(row2, text="Start setpoints → MATLAB", command=self.start_coords)
        self.btn_xyz_stop  = ttk.Button(row2, text="Stop setpoints", state=tk.DISABLED, command=self.stop_coords)
        self.btn_xyz_start.pack(side=tk.LEFT)
        self.btn_xyz_stop.pack(side=tk.LEFT, padx=6)
        self.btn_use_vicon = ttk.Button(
            row2, text="Current XYZ", command=self._on_use_vicon, state=tk.DISABLED
        )
        self.btn_use_vicon.pack(side=tk.LEFT, padx=6)

        # Path selector
        path_row = ttk.Frame(xyz); path_row.pack(fill=tk.X, pady=(6,0))
        ttk.Label(path_row, text="Path").pack(side=tk.LEFT)
        self.path_type_var = tk.StringVar(value="none")
        ttk.Radiobutton(path_row, text="None", variable=self.path_type_var, value="none",
                        command=self._on_path_type).pack(side=tk.LEFT, padx=(6,0))
        ttk.Radiobutton(path_row, text="Circle", variable=self.path_type_var, value="circle",
                        command=self._on_path_type).pack(side=tk.LEFT, padx=(6,0))
        ttk.Radiobutton(path_row, text="Square", variable=self.path_type_var, value="square",
                        command=self._on_path_type).pack(side=tk.LEFT, padx=(6,0))
        ttk.Button(path_row, text="Apply", command=self._on_apply_path).pack(side=tk.LEFT, padx=(12,0))

        # Parameter container
        self.path_params_frame = ttk.Frame(xyz)
        self.path_params_frame.pack(fill=tk.X, pady=(4,0))

        # Circle params
        self.circle_frame = ttk.Frame(self.path_params_frame)
        ttk.Label(self.circle_frame, text="Center X").grid(row=0, column=0, padx=(0,4))
        ttk.Label(self.circle_frame, text="Center Y").grid(row=0, column=2, padx=(12,4))
        self.c_cx_var = tk.StringVar(value="0.0")
        self.c_cy_var = tk.StringVar(value="0.0")
        ttk.Entry(self.circle_frame, width=10, textvariable=self.c_cx_var).grid(row=0, column=1)
        ttk.Entry(self.circle_frame, width=10, textvariable=self.c_cy_var).grid(row=0, column=3)
        ttk.Label(self.circle_frame, text="Radius").grid(row=1, column=0, padx=(0,4))
        self.c_radius_var = tk.StringVar(value="500")
        ttk.Entry(self.circle_frame, width=10, textvariable=self.c_radius_var).grid(row=1, column=1)
        ttk.Label(self.circle_frame, text="Speed").grid(row=1, column=2, padx=(12,4))
        self.c_speed_var = tk.StringVar(value="300")
        ttk.Entry(self.circle_frame, width=10, textvariable=self.c_speed_var).grid(row=1, column=3)
        self.c_cw_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.circle_frame, text="Clockwise", variable=self.c_cw_var).grid(row=2, column=0, columnspan=2, sticky=tk.W)
        self.c_holdz_var = tk.BooleanVar(value=True)
        chk = ttk.Checkbutton(self.circle_frame, text="Hold Z", variable=self.c_holdz_var,
                              command=self._on_circle_holdz)
        chk.grid(row=2, column=2, columnspan=2, sticky=tk.W)
        ttk.Label(self.circle_frame, text="Z amp").grid(row=3, column=0, padx=(0,4))
        self.c_zamp_var = tk.StringVar(value="0.0")
        self.c_zamp_entry = ttk.Entry(self.circle_frame, width=10, textvariable=self.c_zamp_var)
        self.c_zamp_entry.grid(row=3, column=1)
        ttk.Label(self.circle_frame, text="Z period").grid(row=3, column=2, padx=(12,4))
        self.c_zper_var = tk.StringVar(value="1.0")
        self.c_zper_entry = ttk.Entry(self.circle_frame, width=10, textvariable=self.c_zper_var)
        self.c_zper_entry.grid(row=3, column=3)

        # Square params
        self.square_frame = ttk.Frame(self.path_params_frame)
        ttk.Label(self.square_frame, text="Center X").grid(row=0, column=0, padx=(0,4))
        ttk.Label(self.square_frame, text="Center Y").grid(row=0, column=2, padx=(12,4))
        self.s_cx_var = tk.StringVar(value="0.0")
        self.s_cy_var = tk.StringVar(value="0.0")
        ttk.Entry(self.square_frame, width=10, textvariable=self.s_cx_var).grid(row=0, column=1)
        ttk.Entry(self.square_frame, width=10, textvariable=self.s_cy_var).grid(row=0, column=3)
        ttk.Label(self.square_frame, text="Side length").grid(row=1, column=0, padx=(0,4))
        self.s_side_var = tk.StringVar(value="500")
        ttk.Entry(self.square_frame, width=10, textvariable=self.s_side_var).grid(row=1, column=1)
        ttk.Label(self.square_frame, text="Speed").grid(row=1, column=2, padx=(12,4))
        self.s_speed_var = tk.StringVar(value="300")
        ttk.Entry(self.square_frame, width=10, textvariable=self.s_speed_var).grid(row=1, column=3)
        self.s_cw_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.square_frame, text="Clockwise", variable=self.s_cw_var).grid(row=2, column=0, columnspan=2, sticky=tk.W)
        ttk.Label(self.square_frame, text="Corner dwell (s)").grid(row=2, column=2, padx=(12,4))
        self.s_dwell_var = tk.StringVar(value="0.0")
        ttk.Entry(self.square_frame, width=10, textvariable=self.s_dwell_var).grid(row=2, column=3)
        self.s_holdz_var = tk.BooleanVar(value=True)
        chk2 = ttk.Checkbutton(self.square_frame, text="Hold Z", variable=self.s_holdz_var,
                               command=self._on_square_holdz)
        chk2.grid(row=3, column=0, columnspan=2, sticky=tk.W)
        ttk.Label(self.square_frame, text="Z amp").grid(row=4, column=0, padx=(0,4))
        self.s_zamp_var = tk.StringVar(value="0.0")
        self.s_zamp_entry = ttk.Entry(self.square_frame, width=10, textvariable=self.s_zamp_var)
        self.s_zamp_entry.grid(row=4, column=1)
        ttk.Label(self.square_frame, text="Z period").grid(row=4, column=2, padx=(12,4))
        self.s_zper_var = tk.StringVar(value="1.0")
        self.s_zper_entry = ttk.Entry(self.square_frame, width=10, textvariable=self.s_zper_var)
        self.s_zper_entry.grid(row=4, column=3)

        # Status and error line
        status_row = ttk.Frame(xyz); status_row.pack(fill=tk.X, pady=(4,0))
        self.lbl_xyz_error = ttk.Label(status_row, text="", foreground="red")
        self.lbl_xyz_error.pack(side=tk.LEFT)
        self.lbl_xyz_status = ttk.Label(status_row, text="Ready: None")
        self.lbl_xyz_status.pack(side=tk.RIGHT)

        self._on_circle_holdz(); self._on_square_holdz(); self._on_path_type()

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

        # Restart and state label
        armrow = ttk.Frame(parent)
        self.btn_restart = ttk.Button(armrow, text="Restart", command=self._on_restart)
        self.btn_restart.pack(side=tk.LEFT)
        self.lbl_restart = ttk.Label(armrow, text="—")
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
        row1 = ttk.Frame(fc)
        row1.pack(fill=tk.X)
        self.btn_sp_start = ttk.Button(row1, text="Take off", command=self._sp_start)
        self.btn_sp_start.pack(side=tk.LEFT)
        self.btn_sp_start.tooltip = "Connect, send neutral frame, auto-arm, and start the control loop."
        self.btn_sp_stop  = ttk.Button(row1, text="Land",  command=self._sp_stop, state=tk.DISABLED)
        self.btn_sp_stop.pack(side=tk.LEFT, padx=(6,12))
        self.btn_sp_stop.tooltip = "Smoothly ramp down and send final zero. Stop the control loop."
        self.sp_hz_var = tk.IntVar(value=100)
        ttk.Label(row1, text="Rate (Hz)").pack(side=tk.LEFT)
        ttk.Spinbox(row1, from_=1, to=1000, width=6, textvariable=self.sp_hz_var).pack(side=tk.LEFT, padx=(4,0))
        self.lbl_sp_actual = ttk.Label(row1, text="Actual: -- Hz")
        self.lbl_sp_actual.pack(side=tk.LEFT, padx=(12,0))

        row2 = ttk.Frame(fc)
        row2.pack(fill=tk.X, pady=(4,0))
        self.lbl_rpyt = ttk.Label(row2, text="R: 0.00°  P: 0.00°  Y: 0.00°/s  T: 0", font=("TkFixedFont",))
        self.lbl_rpyt.pack(side=tk.LEFT)
        ttk.Label(row2, text="Throttle offset").pack(side=tk.LEFT, padx=(12,0))
        self.offset_var = tk.StringVar(value=str(self.cfg.throttle_offset))
        self.offset_spin = ttk.Spinbox(row2, from_=0, to=65535, width=8,
                                       textvariable=self.offset_var)
        self.offset_spin.pack(side=tk.LEFT, padx=(4,0))
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
        self.btn_pwm_start = ttk.Button(rowb, text="Take off", command=self._pwm_start)
        self.btn_pwm_start.pack(side=tk.LEFT)
        self.btn_pwm_start.tooltip = "Connect, send neutral frame, auto-arm, and start the control loop."
        self.btn_pwm_stop  = ttk.Button(rowb, text="Land", command=self._pwm_stop, state=tk.DISABLED)
        self.btn_pwm_stop.pack(side=tk.LEFT, padx=(6,12))
        self.btn_pwm_stop.tooltip = "Smoothly ramp down and send final zero. Stop the control loop."
        self.pwm_hz_var = tk.IntVar(value=200)
        ttk.Label(rowb, text="Rate (Hz)").pack(side=tk.LEFT)
        ttk.Spinbox(rowb, from_=1, to=1000, width=6, textvariable=self.pwm_hz_var).pack(side=tk.LEFT, padx=(4,0))
        self.lbl_pwm_actual = ttk.Label(rowb, text="Actual: -- Hz")
        self.lbl_pwm_actual.pack(side=tk.LEFT, padx=(12,0))

        # Safety（共用）
        safe = ttk.Labelframe(parent, text="Safety", padding=8); safe.pack(fill=tk.X, pady=(8,0))
        ttk.Button(safe, text="Emergency stop", command=self.emergency_stop).pack(side=tk.LEFT)
        ttk.Button(safe, text="Land", command=self.land).pack(side=tk.LEFT, padx=8)

    # ---- Flight Path tab ----
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
        self._cur_target_scatter = self.ax3d.scatter([], [], [], s=40, c="green", depthshade=False)
        self.canvas3d = FigureCanvasTkAgg(self._fig, master=self.plot_container)
        self.canvas3d.draw_idle()
        self.canvas3d.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        self._last_draw_ts = 0.0
        self._update_view()

    def _on_tab_changed(self, evt):
        tab_text = self.notebook.tab(self.notebook.select(), "text") or ""
        if "Vicon" in tab_text or "3D" in tab_text:
            self._ensure_3d_canvas()

    def _cur_target_tick(self):
        last = get_last_stream_xyz()
        if last and self._cur_target_scatter is not None:
            x, y, z = last
            self._cur_target_scatter._offsets3d = ([x], [y], [z])
        elif self._cur_target_scatter is not None:
            self._cur_target_scatter._offsets3d = ([], [], [])
        if getattr(self, "canvas3d", None):
            self.canvas3d.draw_idle()
        self.after(50, self._cur_target_tick)

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

    def _set_frame_state(self, frame, state):
        for child in frame.winfo_children():
            try:
                child.configure(state=state)
            except Exception:
                pass

    def _on_path_type(self):
        t = self.path_type_var.get()
        for f in (self.circle_frame, self.square_frame):
            try:
                f.pack_forget()
            except Exception:
                pass
        if t == "circle":
            self._set_frame_state(self.circle_frame, tk.NORMAL)
            self._set_frame_state(self.square_frame, tk.DISABLED)
            try:
                self.c_cx_var.set(self.x_var.get())
                self.c_cy_var.set(self.y_var.get())
            except Exception:
                pass
            self.circle_frame.pack(fill=tk.X)
        elif t == "square":
            self._set_frame_state(self.square_frame, tk.NORMAL)
            self._set_frame_state(self.circle_frame, tk.DISABLED)
            try:
                self.s_cx_var.set(self.x_var.get())
                self.s_cy_var.set(self.y_var.get())
            except Exception:
                pass
            self.square_frame.pack(fill=tk.X)
        else:
            self._set_frame_state(self.circle_frame, tk.DISABLED)
            self._set_frame_state(self.square_frame, tk.DISABLED)

    def _on_circle_holdz(self):
        state = tk.DISABLED if self.c_holdz_var.get() else tk.NORMAL
        self.c_zamp_entry.configure(state=state)
        self.c_zper_entry.configure(state=state)

    def _on_square_holdz(self):
        state = tk.DISABLED if self.s_holdz_var.get() else tk.NORMAL
        self.s_zamp_entry.configure(state=state)
        self.s_zper_entry.configure(state=state)

    def _clear_preview(self):
        for attr in ("_preview_target", "_preview_path", "_preview_extra"):
            art = getattr(self, attr, None)
            if art is not None:
                try:
                    art.remove()
                except Exception:
                    pass
                setattr(self, attr, None)

    def _redraw_preview(self, ptype: str, x: float, y: float, z: float, params: dict):
        self._ensure_3d_canvas()
        if not self.ax3d:
            return
        path_pts: list[tuple[float, float, float]] = []
        extra_pts: list[tuple[float, float, float]] = []
        if ptype == "circle":
            path_pts = controller.preview_points_circle(
                params.get("center_x", x),
                params.get("center_y", y),
                z,
                params.get("radius", 0.0),
                params.get("clockwise", True),
                params.get("hold_z", True),
                params.get("z_amp", 0.0),
                params.get("z_period", 1.0),
            )
            extra_pts = [(params.get("center_x", x), params.get("center_y", y), z)]
        elif ptype == "square":
            path_pts = controller.preview_points_square(
                params.get("center_x", x),
                params.get("center_y", y),
                z,
                params.get("side", params.get("side_length", 0.0)),
                params.get("clockwise", True),
                params.get("hold_z", True),
                params.get("z_amp", 0.0),
                params.get("z_period", 1.0),
            )
            half = params.get("side", params.get("side_length", 0.0)) / 2.0
            cx = params.get("center_x", x)
            cy = params.get("center_y", y)
            extra_pts = [
                (cx + half, cy + half, z),
                (cx + half, cy - half, z),
                (cx - half, cy - half, z),
                (cx - half, cy + half, z),
            ]

        # target
        if self._preview_target is None:
            self._preview_target = self.ax3d.scatter(
                [x], [y], [z], color=PreviewCfg.TARGET_COLOR, s=PreviewCfg.TARGET_SIZE
            )
        else:
            self._preview_target._offsets3d = ([x], [y], [z])

        # path line
        if path_pts:
            xs, ys, zs = zip(*path_pts)
            if self._preview_path is None:
                (self._preview_path,) = self.ax3d.plot(xs, ys, zs, color=PreviewCfg.PATH_COLOR)
            else:
                self._preview_path.set_data(xs, ys)
                self._preview_path.set_3d_properties(zs)
        else:
            if self._preview_path is not None:
                self._preview_path.set_data([], [])
                self._preview_path.set_3d_properties([])

        # extra markers
        if extra_pts:
            xs = [p[0] for p in extra_pts]
            ys = [p[1] for p in extra_pts]
            zs = [p[2] for p in extra_pts]
            if self._preview_extra is None:
                marker = "x" if len(extra_pts) == 1 else None
                self._preview_extra = self.ax3d.scatter(
                    xs, ys, zs, color=PreviewCfg.CENTER_COLOR, marker=marker
                )
            else:
                self._preview_extra._offsets3d = (xs, ys, zs)
        else:
            if self._preview_extra is not None:
                self._preview_extra._offsets3d = ([], [], [])

        all_pts = path_pts + [(x, y, z)] + extra_pts
        xs = [p[0] for p in all_pts] or [0.0]
        ys = [p[1] for p in all_pts] or [0.0]
        zs = [p[2] for p in all_pts] or [0.0]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        zmin, zmax = min(zs), max(zs)
        rng = max(xmax - xmin, ymax - ymin, zmax - zmin)
        if rng <= 0:
            rng = 1.0
        half = rng / 2.0 * (1 + PreviewCfg.AXIS_MARGIN_FRAC)
        mx = (xmax + xmin) / 2.0
        my = (ymax + ymin) / 2.0
        mz = (zmax + zmin) / 2.0
        self.ax3d.set_xlim(mx - half, mx + half)
        self.ax3d.set_ylim(my - half, my + half)
        self.ax3d.set_zlim(mz - half, mz + half)
        if self.canvas3d:
            self.canvas3d.draw_idle()


    def _on_apply_path(self):
        now = self._now()
        if now - getattr(self, "_last_apply_ts", 0.0) < 0.1:
            return
        self._last_apply_ts = now
        self._on_path_type()

        vals = self._get_latest_vicon_xyz()
        if vals:
            try:
                self.x_var.set(f"{vals[0]:.3f}")
                self.y_var.set(f"{vals[1]:.3f}")
                self.z_var.set(f"{vals[2]:.3f}")
            except Exception:
                pass

        try:
            x = float(self.x_var.get())
            y = float(self.y_var.get())
            z = float(self.z_var.get())
            rate = int(float(self.xyz_hz_var.get()))
        except Exception:
            self.lbl_xyz_error.configure(text="Invalid XYZ or rate")
            return

        ptype = self.path_type_var.get()
        params: dict[str, float] = {}
        try:
            if ptype == "circle":
                params = {
                    "center_x": float(self.c_cx_var.get() or x),
                    "center_y": float(self.c_cy_var.get() or y),
                    "radius": float(self.c_radius_var.get()),
                    "speed": float(self.c_speed_var.get()),
                    "clockwise": bool(self.c_cw_var.get()),
                    "hold_z": bool(self.c_holdz_var.get()),
                }
                if params["radius"] <= 0 or params["speed"] <= 0:
                    raise ValueError
                if not params["hold_z"]:
                    params["z_amp"] = float(self.c_zamp_var.get() or 0.0)
                    params["z_period"] = float(self.c_zper_var.get() or 1.0)
                    if params["z_amp"] < 0 or params["z_period"] <= 0:
                        raise ValueError
            elif ptype == "square":
                params = {
                    "center_x": float(self.s_cx_var.get() or x),
                    "center_y": float(self.s_cy_var.get() or y),
                    "side": float(self.s_side_var.get()),
                    "speed": float(self.s_speed_var.get()),
                    "clockwise": bool(self.s_cw_var.get()),
                    "dwell": float(self.s_dwell_var.get() or 0.0),
                    "hold_z": bool(self.s_holdz_var.get()),
                }
                if params["side"] <= 0 or params["speed"] <= 0 or params["dwell"] < 0:
                    raise ValueError
                if not params["hold_z"]:
                    params["z_amp"] = float(self.s_zamp_var.get() or 0.0)
                    params["z_period"] = float(self.s_zper_var.get() or 1.0)
                    if params["z_amp"] < 0 or params["z_period"] <= 0:
                        raise ValueError
            else:
                ptype = "none"
        except Exception:
            self.lbl_xyz_error.configure(text="Invalid input")
            return

        if rate < PathCfg.MIN_HZ or rate > PathCfg.MAX_HZ:
            self.lbl_xyz_error.configure(text="Rate out of range")
            return

        with self.state_model.lock:
            self.state_model.xyz_target = (x, y, z)
            self.state_model.rate_hz = rate
            self.state_model.path_type = ptype
            self.state_model.path_params = params
            running = bool(self.state_model.stream_running)

        self.lbl_xyz_error.configure(text="")
        self._redraw_preview(ptype, x, y, z, params)

        label_ptype = ptype.capitalize() if ptype != "none" else "None"
        self.lbl_xyz_status.configure(
            text=f"Ready: {label_ptype} | XYZ=({x:.2f}, {y:.2f}, {z:.2f})"
        )

        if running:
            controller.hot_update({
                "xyz": (x, y, z),
                "rate_hz": rate,
                "path_type": ptype,
                "path_params": params,
            })


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
            if getattr(self, "_vrx_running", False):
                try: self._vrx_stop()
                except Exception: pass
            if self.link: self.link.disconnect(); self.link=None
            self.cf = None
            self.btn_conn.configure(state=tk.NORMAL); self.btn_disc.configure(state=tk.DISABLED)
            if hasattr(self, "lbl_status"):
                self.lbl_status.configure(text=" | Disconnected")
            self.log("Disconnected")
        except Exception as e:
            self.log(f"Disconnect error: {e}")

    def _refresh_control_states(self) -> None:
        if self.link:
            self.btn_conn.config(state=tk.DISABLED)
            self.btn_disc.config(state=tk.NORMAL)
        else:
            self.btn_conn.config(state=tk.NORMAL)
            self.btn_disc.config(state=tk.DISABLED)
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
        if self.path_loop and self.path_loop.is_running():
            controller.stop_path(self.state_model)
            self.path_loop = None
        loop = controller.start_path(self.state_model)
        if loop:
            self.path_loop = loop
            self.btn_xyz_start.configure(state=tk.DISABLED)
            self.btn_xyz_stop.configure(state=tk.NORMAL)
            self.log("Start sending XYZ to MATLAB")

    def stop_coords(self):
        if self.path_loop:
            controller.stop_path(self.state_model)
            self.path_loop = None
        controller.clear_udp_8888()
        self.btn_xyz_start.configure(state=tk.NORMAL)
        self.btn_xyz_stop.configure(state=tk.DISABLED)
        self.lbl_xyz_status.configure(text="Idle")
        self.log("Stop sending XYZ")

    # ---- Setpoint loop (CF) ----
    def _sp_start(self):
        if self.pwm_loop and getattr(self.pwm_loop, "is_running", lambda: False)():
            self._pwm_stop()
        loop = controller.start_mode("rpyt", self.state_model, self.link, rate_hz=int(self.sp_hz_var.get()))
        if loop:
            self.setpoints = loop
            self.btn_sp_start.configure(state=tk.DISABLED)
            self.btn_sp_stop.configure(state=tk.NORMAL)
            self.log(f"Setpoint loop started @ {loop.get_rate()} Hz")
        else:
            self.log("Setpoint loop failed to start")

    def _sp_stop(self):
        controller.land("rpyt", self.state_model, self.link)
        controller.clear_udp_8888()
        self.setpoints = None
        self.btn_sp_start.configure(state=tk.NORMAL)
        self.btn_sp_stop.configure(state=tk.DISABLED)
        self.log("Landing complete, RPYT=0 sent, port 8888 cleared.")


    # ---- PWM loop ----
    def _pwm_start(self):
        if self.setpoints and getattr(self.setpoints, "is_running", lambda: False)():
            self._sp_stop()
        mode = self.pwm_mode_var.get()
        pwm_vals = []
        if mode == "manual":
            for var in self.pwm_vars:
                try:
                    pwm_vals.append(int(var.get()))
                except Exception:
                    pwm_vals.append(0)
        loop = controller.start_mode("pwm", self.state_model, self.link,
                                     rate_hz=int(self.pwm_hz_var.get()),
                                     pwm_mode=mode, manual_pwm=pwm_vals)
        if loop:
            self.pwm_loop = loop
            self.btn_pwm_start.configure(state=tk.DISABLED)
            self.btn_pwm_stop.configure(state=tk.NORMAL)
            self.log("4PID loop started")
            self._on_pwm_mode_change()
        else:
            self.log("4PID loop failed to start")

    def _pwm_stop(self):
        controller.land("pwm", self.state_model, self.link)
        controller.clear_udp_8888()
        self.pwm_loop = None
        self.btn_pwm_start.configure(state=tk.NORMAL)
        self.btn_pwm_stop.configure(state=tk.DISABLED)
        self.log("Landing complete (PWM), port 8888 cleared.")
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
        if self._is_4pid_mode_active():
            self._pwm_stop()
        else:
            self._sp_stop()

    def _on_restart(self):
        uri = str(self._current_uri())
        threading.Thread(target=power_cycle_crazyflie, args=(uri,), daemon=True).start()

    def _current_uri(self) -> str:
        if getattr(self, "uri_var", None) and self.uri_var.get():
            return self.uri_var.get()
        return self._rebuild_uri()

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
            if self.pwm_loop and self.pwm_mode_var.get() == "udp":
                vals = getattr(self.pwm_loop, "last_pwm", [0, 0, 0, 0])
                for i, var in enumerate(self.pwm_vars):
                    var.set(str(int(vals[i])))
        except Exception:
            pass


        # flight path status (~2 Hz)
        now = perf_counter()
        try:
            if self.path_loop and self.path_loop.is_running():
                if now - getattr(self, "_last_path_status", 0.0) >= 0.5:
                    ar = self.path_loop.get_actual_rate()
                    with self.state_model.lock:
                        t = float(self.state_model.path_elapsed)
                        x, y, z = self.state_model.path_last_xyz
                        ptype = self.state_model.path_type
                    self.lbl_xyz_status.configure(
                        text=f"Streaming {ptype.title()} | t={t:.1f} s | rate={ar:.1f} Hz | XYZ=({x:.2f}, {y:.2f}, {z:.2f})",
                    )
                    self._last_path_status = now
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
            if self.setpoints: self.setpoints.stop()
            if self.pwm_loop: self.pwm_loop.stop()
            if self.path_loop:
                controller.stop_path(self.state_model)
                self.path_loop = None
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
