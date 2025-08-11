# ===== src/cfmarslab/ui.py (Controls + Log Parameter tab; checkbox-gated logging) =====
import socket, struct, threading, time
import tkinter as tk
from tkinter import ttk, scrolledtext

from .models import SharedState
from .config import load_config, save_config
from .link import LinkManager
from .control import UDPInput, SetpointLoop
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

    # ---- helpers ----
    def log(self, s: str):
        self.console.configure(state='normal'); self.console.insert('end', s + '\n')
        self.console.configure(state='disabled'); self.console.see('end')

    # ---- Controls tab ----
    def _build_controls_tab(self, parent):
        # XYZ → MATLAB
        xyz = ttk.Labelframe(parent, text="XYZ → MATLAB (UDP 51002)", padding=8); xyz.pack(fill=tk.X)
        row = ttk.Frame(xyz); row.pack(fill=tk.X, pady=(0,6))
        ttk.Label(row, text="X").grid(row=0, column=0, padx=(0,4)); ttk.Label(row, text="Y").grid(row=0, column=2, padx=(12,4)); ttk.Label(row, text="Z").grid(row=0, column=4, padx=(12,4))
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

        # Flight Control
        fc = ttk.Labelframe(parent, text="Flight Control", padding=8)
        fc.pack(fill=tk.X, pady=(8,0))
        self.btn_sp_start = ttk.Button(fc, text="Start", command=self._sp_start)
        self.btn_sp_stop  = ttk.Button(fc, text="Stop",  command=self._sp_stop, state=tk.DISABLED)
        self.btn_sp_start.pack(side=tk.LEFT)
        self.btn_sp_stop.pack(side=tk.LEFT, padx=(6,12))
        self.sp_hz_var = tk.IntVar(value=100)
        ttk.Label(fc, text="Rate (Hz)").pack(side=tk.LEFT)
        ttk.Spinbox(fc, from_=1, to=1000, width=6, textvariable=self.sp_hz_var).pack(side=tk.LEFT, padx=(4,0))
        self.lbl_sp_actual = ttk.Label(fc, text="Actual: -- Hz")
        self.lbl_sp_actual.pack(side=tk.LEFT, padx=(12,0))

        # Safety
        safe = ttk.Labelframe(parent, text="Safety", padding=8); safe.pack(fill=tk.X, pady=(8,0))
        ttk.Button(safe, text="Emergency stop (RPYT=0,0,0,0)", command=self.emergency_stop).pack(side=tk.LEFT)
        ttk.Button(safe, text="Land (ramp down)", command=self.land).pack(side=tk.LEFT, padx=8)

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

        log_items = ["X", "Y", "Z", "Rot_X", "Rot_Y", "Rot_Z", "Roll", "Pitch", "Yaw", "Thrust"]
        self.log_opts = {}

        for name in log_items:
            var = tk.BooleanVar(value=False)
            self.log_opts[name] = var
            cb = tk.Checkbutton(
                parent,
                text=name,
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
            if self.link: self.link.disconnect(); self.link=None
            self.btn_conn.configure(state=tk.NORMAL); self.btn_disc.configure(state=tk.DISABLED)
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

    # ---- Safety ----
    def emergency_stop(self):
        try:
            if self.link: self.link.send_setpoint(0.0, 0.0, 0.0, 0)
            self.log("Emergency stop (RPYT=0,0,0,0) sent")
        except Exception as e:
            self.log(f"Emergency stop failed: {e}")

    def land(self):
        def _ramp():
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
        threading.Thread(target=_ramp, daemon=True).start()

    # ---- logging of selected params ----
    def _append_log_params_sample(self):
        """Append one sample to state_model.log_buf only for checked items."""
        sample = {}
        # positions from GUI
        try:
            if self.log_params["x"].get(): sample["x"] = float(self.x_var.get() or 0.0)
            if self.log_params["y"].get(): sample["y"] = float(self.y_var.get() or 0.0)
            if self.log_params["z"].get(): sample["z"] = float(self.z_var.get() or 0.0)
        except Exception:
            pass
        # rotations from SharedState.rpyth (roll/pitch/yaw)
        try:
            with self.state_model.lock:
                if self.log_params["rot_x"].get(): sample["rot_x"] = float(self.state_model.rpyth.get("roll", 0.0))
                if self.log_params["rot_y"].get(): sample["rot_y"] = float(self.state_model.rpyth.get("pitch", 0.0))
                if self.log_params["rot_z"].get(): sample["rot_z"] = float(self.state_model.rpyth.get("yaw", 0.0))
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

        # record only selected parameters
        self._append_log_params_sample()

        self.after(250, self._ui_tick)

    def on_close(self):
        try:
            self._coords_running = False
            if self.setpoints: self.setpoints.stop()
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
