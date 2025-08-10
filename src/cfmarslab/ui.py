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
        self.title("Crazyflie GUI")
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

        # Connection bar
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

        self.uri_var = tk.StringVar()
        self.mru_var = tk.StringVar()
        self.mru_combo = ttk.Combobox(top, textvariable=self.mru_var, width=40, values=self.cfg.recent_uris, state="readonly")
        self.mru_combo.pack(side=tk.LEFT)
        self.mru_combo.bind("<<ComboboxSelected>>", self._on_select_mru)
        self._rebuild_uri()

        # Telemetry display top-right
        style = ttk.Style(self)
        style.configure("Telemetry.TLabel", font=("Segoe UI", 12, "bold"), foreground="#0044cc")
        self.lbl_vbat = ttk.Label(top, text="VBAT: -- V", style="Telemetry.TLabel")
        self.lbl_vbat.pack(side=tk.RIGHT, padx=(12,0))
        self.lbl_rssi = ttk.Label(top, text="RSSI: --", style="Telemetry.TLabel")
        self.lbl_rssi.pack(side=tk.RIGHT, padx=(12,0))
        self.lbl_latency = ttk.Label(top, text="Latency: -- ms", style="Telemetry.TLabel")
        self.lbl_latency.pack(side=tk.RIGHT, padx=(12,0))

        # Main split
        split = ttk.Panedwindow(root, orient=tk.HORIZONTAL); split.pack(fill=tk.BOTH, expand=True, pady=(8,0))

        # Left side - Notebook with Controls and Log Filters
        nb_frame = ttk.Frame(split)
        nb = ttk.Notebook(nb_frame)
        controls_tab = ttk.Frame(nb)
        logs_tab = ttk.Frame(nb)
        nb.add(controls_tab, text="Controls")
        nb.add(logs_tab, text="Log Filters")
        nb.pack(fill=tk.BOTH, expand=True)

        # Controls tab content
        self._build_controls_tab(controls_tab)
        # Log Filters tab content
        self._build_logs_tab(logs_tab)

        split.add(nb_frame, weight=0)

        # Right side - Console
        right = ttk.Labelframe(split, text="Console", padding=6)
        self.console = scrolledtext.ScrolledText(right, height=18, state="disabled")
        self.console.pack(fill=tk.BOTH, expand=True)
        split.add(right, weight=1)

        self.after(250, self._ui_tick)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.udp = UDPInput(self.state_model)
        self.udp.start()
        self.setpoints: SetpointLoop|None = None

    def _build_controls_tab(self, parent):
        # XYZ to MATLAB
        xyz = ttk.Labelframe(parent, text="XYZ → MATLAB", padding=8); xyz.pack(fill=tk.X)
        self.x_var = tk.StringVar(value="0.0"); self.y_var = tk.StringVar(value="0.0"); self.z_var = tk.StringVar(value="0.0")
        for lbl, var in zip(["X","Y","Z"],[self.x_var,self.y_var,self.z_var]):
            f = ttk.Frame(xyz); f.pack(fill=tk.X)
            ttk.Label(f, text=lbl).pack(side=tk.LEFT)
            ttk.Entry(f, width=10, textvariable=var).pack(side=tk.LEFT)
        row2 = ttk.Frame(xyz); row2.pack(fill=tk.X)
        ttk.Label(row2, text="Rate (Hz)").pack(side=tk.LEFT)
        self.xyz_hz_var = tk.IntVar(value=20)
        ttk.Spinbox(row2, from_=1, to=500, width=6, textvariable=self.xyz_hz_var).pack(side=tk.LEFT, padx=(4,12))
        self.btn_xyz_start = ttk.Button(row2, text="Start", command=self.start_coords)
        self.btn_xyz_stop  = ttk.Button(row2, text="Stop", state=tk.DISABLED, command=self.stop_coords)
        self.btn_xyz_start.pack(side=tk.LEFT); self.btn_xyz_stop.pack(side=tk.LEFT, padx=6)

        # Flight control
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

    def log(self, s: str, category: str = "Other"):
 
        # 檢查是否為受控類別
        if category in self.log_opts:
            if not self.log_opts[category].get():
                return

        # 寫入 Console
        self.console.configure(state='normal')
        self.console.insert('end', s + '\n')
        self.console.configure(state='disabled')
        self.console.see('end')


    def on_connect(self):
        uri = self.uri_var.get().strip()
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
            self.log("Disconnected")
        except Exception as e:
            self.log(f"Disconnect error: {e}")

    def start_coords(self):
        if self._coords_running: return
        self._coords_running = True
        self.btn_xyz_start.configure(state=tk.DISABLED); self.btn_xyz_stop.configure(state=tk.NORMAL)
        t = threading.Thread(target=self._coords_loop, daemon=True); t.start(); self._coords_thread = t

    def stop_coords(self):
        if not self._coords_running: return
        self._coords_running = False
        self.btn_xyz_start.configure(state=tk.NORMAL); self.btn_xyz_stop.configure(state=tk.DISABLED)

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

    def _sp_start(self):
        if not self.link:
            self.log("Not connected"); return
        if not self.setpoints:
            self.setpoints = SetpointLoop(self.state_model, self.link, rate_hz=int(self.sp_hz_var.get()))
        self.setpoints.set_rate(int(self.sp_hz_var.get()))
        self.setpoints.start()
        self.btn_sp_start.configure(state=tk.DISABLED); self.btn_sp_stop.configure(state=tk.NORMAL)

    def _sp_stop(self):
        if self.setpoints: self.setpoints.stop()
        self.btn_sp_start.configure(state=tk.NORMAL); self.btn_sp_stop.configure(state=tk.DISABLED)

    def _ui_tick(self):
        with self.state_model.lock:
            v = float(self.state_model.vbat or 0.0)
            rssi = getattr(self.state_model, 'rssi', float('nan'))
            lat = getattr(self.state_model, 'latency_ms', float('nan'))
        self.lbl_vbat.configure(text=f"VBAT: {v:.2f} V")
        self.lbl_rssi.configure(text=f"RSSI: {rssi:.0f} dBm" if rssi==rssi else "RSSI: --")
        self.lbl_latency.configure(text=f"Latency: {lat:.1f} ms" if lat==lat else "Latency: -- ms")
        try:
            if self.setpoints and self.setpoints.is_running():
                ar, avgj, maxj, miss = self.setpoints.get_jitter_stats()
                self.lbl_sp_actual.configure(text=f"Actual: {ar:.1f} Hz  Jitter: {avgj:.1f}/{maxj:.1f} ms Missed: {miss}")
            else:
                self.lbl_sp_actual.configure(text="Actual: -- Hz")
        except Exception:
            pass
        self.after(250, self._ui_tick)

    def on_close(self):
        try:
            self._coords_running = False
            if self.setpoints: self.setpoints.stop()
            if self.link: self.link.disconnect()
        finally:
            self.destroy()

    def _normalize_addr(self, s: str) -> str:
        s = (s or "").strip(); s = s[2:] if s.lower().startswith("0x") else s
        return s.upper()

    def _rebuild_uri(self):
        chan = int(self.chan_var.get() or 99)
        rate = self.rate_var.get() or "2M"
        addr = self._normalize_addr(self.addr_var.get() or "E7E7E7E7E7")
        self.uri_var.set(f"radio://0/{chan}/{rate}/{addr}")

    def _on_scan(self):
        try:
            cflib.crtp.init_drivers(enable_debug_driver=False)
            found = cflib.crtp.scan_interfaces()
            uris = [u for (u, _d) in found] or []

            # 顯示找到的數量（不受 Log Filters 限制）
            self.log(f"Scan complete: found {len(uris)} device(s)", category="Other")

            if uris:
                self.cfg.recent_uris = uris + [u for u in self.cfg.recent_uris if u not in uris]
                save_config(self.cfg)
                self.mru_combo.configure(values=self.cfg.recent_uris)
                self.mru_var.set(uris[0])
                self._on_select_mru()
            else:
                self.log("No Crazyradio devices found", category="Other")
        except Exception as e:
            self.log(f"Scan failed: {e}", category="Other")

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