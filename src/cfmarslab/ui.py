import socket, struct, threading, time
import tkinter as tk
from tkinter import ttk, scrolledtext
from collections import deque
import math
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from .models import SharedState
from .config import load_config, save_config
from .link import LinkManager
from .control import UDPInput, SetpointLoop, PWMSetpointLoop, PWMUDPReceiver
import cflib.crtp

UDP_COORD_PORT = 51002
RADIO_BITRATES = ("2M", "1M", "250K")

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
        self.link: LinkManager|None = None
        self.cf = None

        self._coords_running = False
        self._coords_thread: threading.Thread|None = None

        root = ttk.Frame(self, padding=8); root.pack(fill=tk.BOTH, expand=True)

        # --- Connection bar ---
        top = ttk.Frame(root); top.pack(fill=tk.X)
        ttk.Label(top, text="Interface:").pack(side=tk.LEFT)
        self.iface_var = tk.StringVar(value="radio")
        self.iface_combo = ttk.Combobox(top, textvariable=self.iface_var, width=16, values=["radio"], state="readonly")
        self.iface_combo.pack(side=tk.LEFT, padx=6)
        self.iface_combo.bind("<<ComboboxSelected>>", lambda *_: self._rebuild_uri())

        self.btn_conn = ttk.Button(top, text="Connect", command=self.on_connect)
        self.btn_disc = ttk.Button(top, text="Disconnect", command=self.on_disconnect, state=tk.DISABLED)
        self.btn_scan = ttk.Button(top, text="Scan", command=self._on_scan)
        self.btn_conn.pack(side=tk.LEFT, padx=(6,0)); self.btn_disc.pack(side=tk.LEFT, padx=(6,0)); self.btn_scan.pack(side=tk.LEFT, padx=(6,0))

        ttk.Label(top, text="Channel:").pack(side=tk.LEFT, padx=(12,0))
        self.chan_var = tk.IntVar(value=99)
        ttk.Spinbox(top, from_=0, to=125, width=5, textvariable=self.chan_var, command=self._rebuild_uri).pack(side=tk.LEFT, padx=(4,12))
        ttk.Label(top, text="Bitrate:").pack(side=tk.LEFT)
        self.rate_var = tk.StringVar(value="2M")
        self.rate_combo = ttk.Combobox(top, textvariable=self.rate_var, width=6, values=list(RADIO_BITRATES), state="readonly")
        self.rate_combo.pack(side=tk.LEFT, padx=(4,12))
        self.rate_combo.bind("<<ComboboxSelected>>", lambda *_: self._rebuild_uri())
        ttk.Label(top, text="Address:").pack(side=tk.LEFT)
        self.addr_var = tk.StringVar(value="0xE7E7E7E7E7")
        addr_e = ttk.Entry(top, width=14, textvariable=self.addr_var); addr_e.pack(side=tk.LEFT, padx=(4,12))
        addr_e.bind("<KeyRelease>", lambda *_: self._rebuild_uri())

        # MRU
        self.uri_var = tk.StringVar()
        self.mru_var = tk.StringVar()
        self.mru_combo = ttk.Combobox(top, textvariable=self.mru_var, width=40, values=self.cfg.recent_uris, state="readonly")
        self.mru_combo.pack(side=tk.LEFT)
        self.mru_combo.bind("<<ComboboxSelected>>", self._on_select_mru)
        self._rebuild_uri()

        # Telemetry top-right
        style = ttk.Style(self)
        style.configure("Telemetry.TLabel", font=("Segoe UI", 12, "bold"), foreground="#0044cc")
        self.lbl_vbat = ttk.Label(top, text="VBAT: -- V", style="Telemetry.TLabel")
        self.lbl_vbat.pack(side=tk.RIGHT, padx=(12,0))
        self.lbl_rssi = ttk.Label(top, text="RSSI: --", style="Telemetry.TLabel")
        self.lbl_rssi.pack(side=tk.RIGHT, padx=(12,0))
        self.lbl_latency = ttk.Label(top, text="Latency: -- ms", style="Telemetry.TLabel")
        self.lbl_latency.pack(side=tk.RIGHT, padx=(12,0))

        # --- Main split ---
        split = ttk.Panedwindow(root, orient=tk.HORIZONTAL); split.pack(fill=tk.BOTH, expand=True, pady=(8,0))

        # Left: Notebook with two tabs (Controls, Log Parameter)
        left_frame = ttk.Frame(split)
        self.nb = ttk.Notebook(left_frame)
        tab_controls = ttk.Frame(self.nb)
        tab_logparam = ttk.Frame(self.nb)
        self.nb.add(tab_controls, text="Controls")
        self.nb.add(tab_logparam, text="Log Parameter")
        self.nb.pack(fill=tk.BOTH, expand=True)
        split.add(left_frame, weight=0)

        # Vicon plot state
        self.trail_buf = deque(maxlen=20000)   # (t, x, y, z)
        self.trail_secs_var = tk.IntVar(value=5)    # Show trail (seconds)
        self.decimate_var   = tk.IntVar(value=1)    # 1 = no decimation
        self._quiver_artist = None
        self._trail_artist  = None
        self._point_artist  = None
        self._last_plot_ts  = 0.0

        # Build tabs
        self._build_controls_tab(tab_controls)
        self._build_log_param_tab(tab_logparam)

        # Right: Console
        right = ttk.Labelframe(split, text="Console", padding=6)
        self.console = scrolledtext.ScrolledText(right, height=18, state="disabled")
        self.console.pack(fill=tk.BOTH, expand=True)
        split.add(right, weight=1)

        # timers
        self.after(250, self._ui_tick)
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
        self.btn_xyz_start.pack(side=tk.LEFT); self.btn_xyz_stop.pack(side=tk.LEFT, padx=6)

        # Arm 按鈕（放在 Controls 區塊、Flight Control 上方）
        self.btn_arm = ttk.Button(parent, text="Arm", command=self._arm_cf, state=tk.DISABLED)
        self.btn_arm.pack(anchor=tk.W, pady=(8,0))

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

        # --- 4-PID tab (PWM direct) ---
        pwmf = ttk.Labelframe(tab4, text="PWM Control", padding=8)
        pwmf.pack(fill=tk.X)
        rowm = ttk.Frame(pwmf); rowm.pack(fill=tk.X)
        self.pwm_mode_var = tk.StringVar(value="manual")
        ttk.Radiobutton(rowm, text="Manual entry", variable=self.pwm_mode_var,
                        value="manual", command=self._on_pwm_mode_change).pack(side=tk.LEFT)
        ttk.Radiobutton(rowm, text="UDP @ 8888", variable=self.pwm_mode_var,
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

        # Vicon (UDP 51001) 3D plot
        vicon = ttk.Labelframe(parent, text="Vicon (UDP 51001)", padding=8)
        vicon.pack(fill=tk.BOTH, pady=(8,0))

        brow = ttk.Frame(vicon); brow.pack(fill=tk.X)
        ttk.Label(brow, text="X").grid(row=0, column=0, padx=(0,4))
        self.bx0 = tk.StringVar(value="-1.0"); self.bx1 = tk.StringVar(value="1.0")
        ttk.Entry(brow, width=8, textvariable=self.bx0).grid(row=0, column=1)
        ttk.Entry(brow, width=8, textvariable=self.bx1).grid(row=0, column=2)
        ttk.Label(brow, text="Y").grid(row=0, column=3, padx=(12,4))
        self.by0 = tk.StringVar(value="-1.0"); self.by1 = tk.StringVar(value="1.0")
        ttk.Entry(brow, width=8, textvariable=self.by0).grid(row=0, column=4)
        ttk.Entry(brow, width=8, textvariable=self.by1).grid(row=0, column=5)
        ttk.Label(brow, text="Z").grid(row=0, column=6, padx=(12,4))
        self.bz0 = tk.StringVar(value="0.0"); self.bz1 = tk.StringVar(value="1.0")
        ttk.Entry(brow, width=8, textvariable=self.bz0).grid(row=0, column=7)
        ttk.Entry(brow, width=8, textvariable=self.bz1).grid(row=0, column=8)
        ttk.Button(brow, text="Apply", command=lambda: (self._apply_axes_bounds(), self.canvas3d.draw_idle())).grid(row=0, column=9, padx=(12,0))

        optrow = ttk.Frame(vicon); optrow.pack(fill=tk.X, pady=(4,0))
        ttk.Label(optrow, text="Show trail (s)").pack(side=tk.LEFT, padx=(0,4))
        ttk.Spinbox(optrow, from_=0, to=120, width=4, textvariable=self.trail_secs_var).pack(side=tk.LEFT)
        ttk.Label(optrow, text="Decimate").pack(side=tk.LEFT, padx=(12,4))
        ttk.Combobox(optrow, width=4, state="readonly", values=["1","2","5","10"], textvariable=self.decimate_var).pack(side=tk.LEFT)

        self.fig3d = Figure(figsize=(4,3))
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        self.ax3d.set_xlabel('X'); self.ax3d.set_ylabel('Y'); self.ax3d.set_zlabel('Z')
        self.canvas3d = FigureCanvasTkAgg(self.fig3d, master=vicon)
        self.canvas3d.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(8,0))
        self._apply_axes_bounds()

        # Safety（共用）
        safe = ttk.Labelframe(parent, text="Safety", padding=8); safe.pack(fill=tk.X, pady=(8,0))
        ttk.Button(safe, text="Emergency stop (RPYT=0,0,0,0)", command=self.emergency_stop).pack(side=tk.LEFT)
        ttk.Button(safe, text="Land (ramp down)", command=self.land).pack(side=tk.LEFT, padx=8)

    def _apply_axes_bounds(self):
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

    def _draw_quiver(self, x, y, z, roll, pitch, yaw):
        # Remove previous
        if self._quiver_artist is not None:
            try: self._quiver_artist.remove()
            except Exception: pass
            self._quiver_artist = None
        # Compute direction from RPY (Z-Y-X; yaw,pitch,roll)
        # Body x-axis projected to world:
        cx, sx = math.cos(roll),  math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw),   math.sin(yaw)
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
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
        if hasattr(self, "canvas3d"):
            self.canvas3d.draw_idle()

    # ---- Log Parameter tab (alias used in __init__) ----
    def _build_log_param_tab(self, parent):
        """Compatibility alias so __init__ can call either name."""
        return self._build_logs_tab(parent)

    # ---- Log Parameter tab (implementation) ----
    def _build_logs_tab(self, parent):
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




    # ---- connection ----
    def on_connect(self):
        """Blocking connect (keep console lines)."""
        uri = (self.uri_var.get() or "").strip()
        if not uri:
            self.log("No URI"); return
        self.log("Connecting...")
        try:
            self.link = LinkManager(self.state_model, uri); self.link.connect()
            self.cf = self.link.cf
            self.btn_conn.configure(state=tk.DISABLED); self.btn_disc.configure(state=tk.NORMAL)
            self.btn_arm.configure(state=tk.NORMAL)
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
            if self.link: self.link.disconnect(); self.link=None
            self.cf = None
            self.btn_conn.configure(state=tk.NORMAL); self.btn_disc.configure(state=tk.DISABLED)
            self.btn_arm.configure(state=tk.DISABLED)
            if hasattr(self, "lbl_status"):
                self.lbl_status.configure(text=" | Disconnected")
            self.log("Disconnected")
        except Exception as e:
            self.log(f"Disconnect error: {e}")

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

    def _arm_cf(self):
        try:
            if self.cf:
                self.cf.platform.send_arming_request(True)
                self.log("Arming request sent")
        except Exception as e:
            self.log(f"Arming request failed: {e}")

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
        # telemetry heads-up
        with self.state_model.lock:
            v = float(self.state_model.vbat or 0.0)
            rssi = getattr(self.state_model, 'rssi', float('nan'))
            lat = getattr(self.state_model, 'latency_ms', float('nan'))
        self.title(f"Crazyflie GUI — VBAT: {v:.2f} V"); self.lbl_vbat.configure(text=f"VBAT: {v:.2f} V")
        self.lbl_rssi.configure(text=f"RSSI: {rssi:.0f} dBm" if rssi==rssi else "RSSI: --")
        self.lbl_latency.configure(text=f"Latency: {lat:.1f} ms" if lat==lat else "Latency: -- ms")

        # setpoint loop actual rate label (if available)
        try:
            if self.setpoints and self.setpoints.is_running():
                ar = self.setpoints.get_actual_rate()
                self.lbl_sp_actual.configure(text=f"Actual: {ar:.1f} Hz")
            else:
                self.lbl_sp_actual.configure(text="Actual: -- Hz")
        except Exception:
            pass

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
        last = None
        try:
            if hasattr(self, "vicon") and self.vicon:
                last = self.vicon.get_last()
        except Exception:
            last = None
        if last:
            x, y, z, rx, ry, rz = last
            now = time.time()
            self.trail_buf.append((now, x, y, z))
            self._apply_axes_bounds()
            if self._point_artist is None:
                self._point_artist = self.ax3d.scatter([x], [y], [z], s=12)
            else:
                self._point_artist._offsets3d = ([x], [y], [z])
            self._draw_quiver(x, y, z, roll=rx, pitch=ry, yaw=rz)
            secs = max(0, int(self.trail_secs_var.get() or 0))
            k = max(1, int(self.decimate_var.get() or 1))
            if secs > 0:
                pts = [(tx, xx, yy, zz) for (tx, xx, yy, zz) in self.trail_buf if now - tx <= secs]
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
            if now - self._last_plot_ts >= 0.1:
                self.canvas3d.draw_idle()
                self._last_plot_ts = now

        self.after(250, self._ui_tick)

    def on_close(self):
        try:
            self._coords_running = False
            if self.setpoints: self.setpoints.stop()
            if self.pwm_loop: self.pwm_loop.stop()
            if self.pwm_udp: self.pwm_udp.stop()
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
            cflib.crtp.init_drivers(enable_debug_driver=False)
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
