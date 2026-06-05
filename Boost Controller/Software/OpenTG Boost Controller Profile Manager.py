import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
from serial.tools import list_ports

# ---------------- Protocol ----------------
MAGIC = bytes([0xAA, 0x55])

MSG_UPLOAD_PROFILE    = 0x01
MSG_READ_PROFILE_REQ  = 0x02
MSG_READ_PROFILE_RESP = 0x03
MSG_ERROR             = 0x7F

# Arduino profile data:
# 3 bytes limits + 8*uint16 ratios (16 bytes) + 49 bytes map
PROFILE_DATA_LEN = 68
UPLOAD_PAYLOAD_LEN = 1 + PROFILE_DATA_LEN  # profile index + data = 69

FIXED_RPM_POINTS = [2000, 3000, 4000, 5000, 6000, 7000, 8000]
RPM_TO_INDEX = {rpm: (i + 2) for i, rpm in enumerate(FIXED_RPM_POINTS)}  # 2000->2 ... 8000->8
INDEX_TO_RPM = {v: k for k, v in RPM_TO_INDEX.items()}

RATIO_SCALE = 1000  # 4.056 -> 4056

MAX_GEAR_LIMIT = 7
MAX_BOOST_LIMIT = 44
MAX_RPM_LIMIT = 8000
FILL_UNUSED_U8 = 0xFF
FILL_UNUSED_U16 = 0xFFFF


# ---------------- Theme ----------------
def apply_dark_theme(root: tk.Tk):
    BG      = "#1e2b36"
    PANEL   = "#243645"
    FIELD   = "#1b2630"
    BORDER  = "#2d475a"
    BORDER_SOFT = "#3b5566"
    TEXT    = "#e8f1f8"
    MUTED   = "#9db0be"
    ACCENT  = "#2db14a"

    root.configure(bg=BG)

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass

    style.configure(".", background=BG, foreground=TEXT)
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=TEXT)
    style.configure("Muted.TLabel", background=BG, foreground=MUTED)

    style.configure("Panel.TFrame", background=PANEL)
    style.configure("Panel.TLabel", background=PANEL, foreground=TEXT)
    style.configure("PanelMuted.TLabel", background=PANEL, foreground=MUTED)
    style.configure("PanelBold.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 10, "bold"))

    style.configure("TButton", background="#2a5673", foreground=TEXT, padding=(10, 6), bordercolor=BORDER_SOFT)
    style.map("TButton",
              background=[("active", "#32709a"), ("disabled", "#2a3b46")],
              foreground=[("disabled", "#6e7f8c")])

    # softer entry outlines
    style.configure("TEntry",
                    fieldbackground=FIELD,
                    foreground=TEXT,
                    bordercolor=BORDER_SOFT,
                    lightcolor=BORDER_SOFT,
                    darkcolor=BORDER_SOFT)
    style.map("TEntry",
              bordercolor=[("focus", ACCENT), ("!focus", BORDER_SOFT)],
              lightcolor=[("focus", ACCENT), ("!focus", BORDER_SOFT)],
              darkcolor=[("focus", ACCENT), ("!focus", BORDER_SOFT)])

    style.configure("TCombobox",
                    fieldbackground=FIELD,
                    foreground=TEXT,
                    background=FIELD,
                    arrowcolor=TEXT,
                    bordercolor=BORDER_SOFT,
                    lightcolor=BORDER_SOFT,
                    darkcolor=BORDER_SOFT)
    style.map("TCombobox",
              fieldbackground=[("readonly", FIELD)],
              bordercolor=[("focus", ACCENT), ("!focus", BORDER_SOFT)],
              lightcolor=[("focus", ACCENT), ("!focus", BORDER_SOFT)],
              darkcolor=[("focus", ACCENT), ("!focus", BORDER_SOFT)])

    style.configure("TNotebook", background=BG, borderwidth=0)
    style.configure("TNotebook.Tab", background=PANEL, foreground=TEXT, padding=(12, 8))
    style.map("TNotebook.Tab",
              background=[("selected", ACCENT), ("active", "#2b4556")],
              foreground=[("selected", "#ffffff"), ("active", TEXT)])

    return {"BG": BG, "PANEL": PANEL, "FIELD": FIELD, "BORDER": BORDER, "BORDER_SOFT": BORDER_SOFT,
            "TEXT": TEXT, "MUTED": MUTED, "ACCENT": ACCENT}


# ---------------- Rounded groups ----------------
class RoundedGroup(ttk.Frame):
    """
    Rounded-corner panel with a title, using Canvas.
    Put widgets inside .body (ttk.Frame with Panel.TFrame style).
    mode:
      - "auto": shrink-wrap to content height
    """
    def __init__(self, parent, title: str, colors: dict, radius: int = 12, pad: int = 12, mode: str = "auto"):
        super().__init__(parent)
        self.colors = colors
        self.radius = radius
        self.pad = pad
        self.mode = mode

        self.canvas = tk.Canvas(self, bg=colors["BG"], highlightthickness=0, bd=0)
        self.canvas.pack(fill="both", expand=True)

        self.title_id = self.canvas.create_text(
            16, 10, anchor="w",
            text=title,
            fill=colors["TEXT"],
            font=("Segoe UI", 10, "bold")
        )

        self.body = ttk.Frame(self, style="Panel.TFrame")
        self.body_id = self.canvas.create_window(0, 0, anchor="nw", window=self.body)

        self.bind("<Configure>", self._redraw)
        self.body.bind("<Configure>", self._redraw)
        self.after(0, self._redraw)

    def _rounded_rect(self, x1, y1, x2, y2, r, **kwargs):
        points = [
            x1+r, y1, x2-r, y1, x2, y1, x2, y1+r,
            x2, y2-r, x2, y2, x2-r, y2, x1+r, y2,
            x1, y2, x1, y2-r, x1, y1+r, x1, y1
        ]
        return self.canvas.create_polygon(points, smooth=True, splinesteps=36, **kwargs)

    def _redraw(self, _evt=None):
        w = self.winfo_width()
        if w < 200:
            return

        title_h = 20
        pad = self.pad
        border_pad = 4

        body_req_h = self.body.winfo_reqheight()
        body_h = max(20, body_req_h)
        total_h = title_h + pad + body_h + pad + border_pad

        self.configure(height=total_h)
        self.canvas.configure(height=total_h)

        h = total_h
        self.canvas.delete("bg")

        x1, y1 = border_pad, title_h
        x2, y2 = w - border_pad, h - border_pad

        self._rounded_rect(
            x1, y1, x2, y2, self.radius,
            fill=self.colors["PANEL"],
            outline=self.colors["BORDER"],
            width=2,
            tags="bg"
        )

        self.canvas.coords(self.title_id, 16, 12)

        body_x = x1 + pad
        body_y = y1 + pad
        body_w = (x2 - x1) - 2*pad

        self.canvas.coords(self.body_id, body_x, body_y)
        self.canvas.itemconfigure(self.body_id, width=body_w)


def make_toggle_radio(parent, text, value, variable, colors, command=None):
    rb = tk.Radiobutton(
        parent,
        text=text,
        value=value,
        variable=variable,
        command=command,
        indicatoron=0,
        padx=12,
        pady=6,
        bd=0,
        relief="flat",
        fg=colors["TEXT"],
        bg=colors["FIELD"],
        activeforeground=colors["TEXT"],
        activebackground=colors["FIELD"],
        selectcolor=colors["ACCENT"],
        highlightthickness=1,
        highlightbackground=colors["BORDER_SOFT"],
        highlightcolor=colors["BORDER_SOFT"],
        font=("Segoe UI", 9, "bold"),
    )
    return rb


# ---------------- Serial helpers ----------------
def lrc8(data: bytes) -> int:
    s = sum(data) & 0xFF
    return (-s) & 0xFF

def build_packet(msg_type: int, payload: bytes) -> bytes:
    ln = len(payload)
    header = MAGIC + bytes([msg_type, ln & 0xFF, (ln >> 8) & 0xFF])
    return header + payload + bytes([lrc8(payload)])

def read_exact(ser: serial.Serial, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError("Serial timeout while reading")
        buf.extend(chunk)
    return bytes(buf)

def read_packet(ser: serial.Serial) -> tuple[int, bytes]:
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for packet")
        if b[0] == MAGIC[0]:
            b2 = read_exact(ser, 1)
            if b2[0] == MAGIC[1]:
                break

    msg_type = read_exact(ser, 1)[0]
    ln_bytes = read_exact(ser, 2)
    ln = ln_bytes[0] | (ln_bytes[1] << 8)

    payload = read_exact(ser, ln)
    rx_lrc = read_exact(ser, 1)[0]

    calc = lrc8(payload)
    if rx_lrc != calc:
        raise ValueError("Bad LRC (checksum mismatch)")
    return msg_type, payload


# ---------------- Packing / parsing ----------------
def pack_u16_le(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])

def unpack_u16_le(b0: int, b1: int) -> int:
    return b0 | (b1 << 8)

def ratio_to_u16(text: str) -> int:
    try:
        f = float(text.strip())
    except Exception:
        raise ValueError("Ratio must be a number like 4.056")
    if f < 0:
        raise ValueError("Ratio cannot be negative")
    v = int(round(f * RATIO_SCALE))
    if not (0 <= v <= 65535):
        raise ValueError("Ratio out of uint16 range after scaling")
    return v

def u16_to_ratio_str(v: int) -> str:
    return f"{v / RATIO_SCALE:.3f}"

def parse_int_or_default(s: str, default: int) -> int:
    s = s.strip()
    if s == "":
        return default
    return int(s)

def clamp_int(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


# ---------------- Profile Editor ----------------
class ProfileEditor(ttk.Frame):
    def __init__(self, parent, profile_index_1based: int, colors: dict):
        super().__init__(parent)
        self.profile_index = profile_index_1based
        self.colors = colors

        self.columnconfigure(0, weight=1)

        # --- Profile Type ---
        self.mode_group = RoundedGroup(self, "Profile Type", colors)
        self.mode_group.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        self.mode_group.body.columnconfigure(2, weight=1)

        self.mode_var = tk.StringVar(value="gear")  # "gear" or "rpm"
        mode_row = tk.Frame(self.mode_group.body, bg=colors["PANEL"])
        mode_row.grid(row=0, column=0, sticky="w")

        make_toggle_radio(mode_row, "Boost by Gear", "gear", self.mode_var, colors, command=self.on_mode_change)\
            .pack(side="left", padx=(0, 8))
        make_toggle_radio(mode_row, "Boost by RPM (single row)", "rpm", self.mode_var, colors, command=self.on_mode_change)\
            .pack(side="left")

        ttk.Label(
            self.mode_group.body,
            text="RPM mode sends MaxGear=0 and fills hidden rows with 0xFF.",
            style="PanelMuted.TLabel"
        ).grid(row=0, column=2, sticky="e")

        # --- Limits ---
        self.limits_group = RoundedGroup(self, "Limits", colors)
        self.limits_group.grid(row=1, column=0, sticky="ew", padx=10, pady=6)

        ttk.Label(self.limits_group.body, text="Max Gear (0..7)", style="Panel.TLabel")\
            .grid(row=0, column=0, padx=6, pady=8, sticky="e")
        self.gear_max = ttk.Entry(self.limits_group.body, width=6, justify="center")
        self.gear_max.grid(row=0, column=1, padx=(0, 14), pady=8, sticky="w")

        ttk.Label(self.limits_group.body, text="Max RPM (2000..8000)", style="Panel.TLabel")\
            .grid(row=0, column=2, padx=6, pady=8, sticky="e")
        self.rpm_max_real = ttk.Entry(self.limits_group.body, width=8, justify="center")
        self.rpm_max_real.grid(row=0, column=3, padx=(0, 14), pady=8, sticky="w")

        ttk.Label(self.limits_group.body, text="Max Boost (0..44 psi)", style="Panel.TLabel")\
            .grid(row=0, column=4, padx=6, pady=8, sticky="e")
        self.boost_max = ttk.Entry(self.limits_group.body, width=6, justify="center")
        self.boost_max.grid(row=0, column=5, padx=(0, 14), pady=8, sticky="w")

        ttk.Label(
            self.limits_group.body,
            text="Table values clamp to Max Boost. Max RPM must be 2000..8000 (step 1000).",
            style="PanelMuted.TLabel"
        ).grid(row=1, column=0, columnspan=8, sticky="w", padx=6, pady=(0, 8))

        self.gear_max.bind("<FocusOut>", lambda _e: self.enforce_limits_and_layout())
        self.gear_max.bind("<KeyRelease>", lambda _e: self.enforce_limits_and_layout(soft=True))
        self.rpm_max_real.bind("<FocusOut>", lambda _e: self.enforce_limits_and_layout())
        self.boost_max.bind("<FocusOut>", lambda _e: self.enforce_limits_and_layout())

        # --- Ratios ---
        self.ratios_group = RoundedGroup(self, "Gear Ratios & Final Drive (ratio → uint16 x1000)", colors)
        self.ratios_group.grid(row=2, column=0, sticky="ew", padx=10, pady=6)

        # keep label+entry widgets to hide per-gear
        self.ratio_labels: list[ttk.Label] = []
        self.ratio_entries: list[ttk.Entry] = []

        for i in range(7):
            lbl = ttk.Label(self.ratios_group.body, text=f"Gear{i+1}", style="Panel.TLabel")
            ent = ttk.Entry(self.ratios_group.body, width=7, justify="center")
            lbl.grid(row=0, column=i*2, padx=(8 if i == 0 else 4, 2), pady=8, sticky="e")
            ent.grid(row=0, column=i*2+1, padx=(0, 8), pady=8, sticky="w")
            self.ratio_labels.append(lbl)
            self.ratio_entries.append(ent)

        # Final drive always visible (index 7 in payload)
        self.fd_label = ttk.Label(self.ratios_group.body, text="Final Drive", style="Panel.TLabel")
        self.fd_entry = ttk.Entry(self.ratios_group.body, width=7, justify="center")
        self.fd_label.grid(row=0, column=14, padx=(4, 2), pady=8, sticky="e")
        self.fd_entry.grid(row=0, column=15, padx=(0, 8), pady=8, sticky="w")

        # --- Boost Map ---
        self.map_group = RoundedGroup(self, "Boost Target Map - Gears × Fixed RPM", colors)
        self.map_group.grid(row=3, column=0, sticky="ew", padx=10, pady=(6, 10))
        self.map_group.body.columnconfigure(tuple(range(8)), weight=1)

        ttk.Label(self.map_group.body, text="Gear \\ RPM", style="PanelBold.TLabel").grid(row=0, column=0, padx=6, pady=6)
        for c, rpm in enumerate(FIXED_RPM_POINTS):
            ttk.Label(self.map_group.body, text=str(rpm), style="PanelBold.TLabel").grid(row=0, column=c+1, padx=6, pady=6)

        self.row_labels: list[ttk.Label] = []
        self.table_entries: list[list[ttk.Entry]] = []
        for r in range(7):
            lbl = ttk.Label(self.map_group.body, text=f"Gear {r+1}", style="PanelBold.TLabel")
            lbl.grid(row=r+1, column=0, padx=6, pady=4, sticky="e")
            self.row_labels.append(lbl)

            row_entries = []
            for c in range(7):
                e = ttk.Entry(self.map_group.body, width=4, justify="center")
                e.grid(row=r+1, column=c+1, padx=3, pady=3, sticky="nsew")
                e.bind("<FocusOut>", lambda _ev, rr=r, cc=c: self.clamp_cell(rr, cc))
                row_entries.append(e)
            self.table_entries.append(row_entries)

        actions = ttk.Frame(self.map_group.body, style="Panel.TFrame")
        actions.grid(row=8, column=0, columnspan=8, sticky="w", padx=6, pady=(10, 2))
        ttk.Button(actions, text="Fill Map With Max Boost", command=self.fill_map_with_max).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Reset To Defaults", command=self.set_defaults).pack(side="left")

        self._last_gear_nonzero = 7
        self.set_defaults()
        self.on_mode_change()

    # -------- Mode / visibility --------
    def on_mode_change(self):
        if self.mode_var.get() == "rpm":
            # remember prior gear
            try:
                cur = parse_int_or_default(self.gear_max.get(), self._last_gear_nonzero)
                if cur > 0:
                    self._last_gear_nonzero = clamp_int(cur, 1, MAX_GEAR_LIMIT)
            except Exception:
                pass

            self.gear_max.delete(0, tk.END)
            self.gear_max.insert(0, "0")
            self.gear_max.state(["disabled"])

            self.ratios_group.grid_remove()
        else:
            self.gear_max.state(["!disabled"])
            try:
                cur = parse_int_or_default(self.gear_max.get(), 0)
            except Exception:
                cur = 0
            if cur == 0:
                self.gear_max.delete(0, tk.END)
                self.gear_max.insert(0, str(self._last_gear_nonzero))

            self.ratios_group.grid()

        self.enforce_limits_and_layout()

    def visible_rows(self) -> int:
        if self.mode_var.get() == "rpm":
            return 1
        try:
            g = parse_int_or_default(self.gear_max.get(), 1)
        except Exception:
            g = 1
        return clamp_int(g, 1, MAX_GEAR_LIMIT)

    def update_map_row_visibility(self):
        vis = self.visible_rows()
        for r in range(7):
            if r < vis:
                self.row_labels[r].grid()
                for c in range(7):
                    self.table_entries[r][c].grid()
            else:
                self.row_labels[r].grid_remove()
                for c in range(7):
                    self.table_entries[r][c].grid_remove()

    def update_ratio_visibility(self):
        """
        In gear mode: show Gear1..GearN ratios, hide the rest.
        In rpm mode: ratios group is hidden anyway.
        """
        if self.mode_var.get() != "gear":
            return

        vis = self.visible_rows()  # equals max gear in gear mode
        for i in range(7):
            if i < vis:
                self.ratio_labels[i].grid()
                self.ratio_entries[i].grid()
            else:
                self.ratio_labels[i].grid_remove()
                self.ratio_entries[i].grid_remove()

    # -------- Limits + clamping --------
    def max_boost_value(self) -> int:
        try:
            b = parse_int_or_default(self.boost_max.get(), 0)
        except Exception:
            b = 0
        return clamp_int(b, 0, MAX_BOOST_LIMIT)

    def clamp_cell(self, r: int, c: int):
        if r >= self.visible_rows():
            return
        e = self.table_entries[r][c]
        try:
            v = parse_int_or_default(e.get(), 0)
        except Exception:
            v = 0
        v = clamp_int(v, 0, self.max_boost_value())
        e.delete(0, tk.END)
        e.insert(0, str(v))

    def clamp_visible_cells_to_maxboost(self):
        vis = self.visible_rows()
        bmax = self.max_boost_value()
        for r in range(vis):
            for c in range(7):
                e = self.table_entries[r][c]
                try:
                    v = parse_int_or_default(e.get(), 0)
                except Exception:
                    v = 0
                v = clamp_int(v, 0, bmax)
                e.delete(0, tk.END)
                e.insert(0, str(v))

    def enforce_limits_and_layout(self, soft: bool = False):
        # Max boost clamp
        try:
            b = parse_int_or_default(self.boost_max.get(), 0)
        except Exception:
            b = 0
        b2 = clamp_int(b, 0, MAX_BOOST_LIMIT)
        if str(b2) != self.boost_max.get().strip():
            self.boost_max.delete(0, tk.END)
            self.boost_max.insert(0, str(b2))

        # Max RPM clamp
        try:
            r = parse_int_or_default(self.rpm_max_real.get(), 8000)
        except Exception:
            r = 8000
        r2 = clamp_int(r, 0, MAX_RPM_LIMIT)
        if str(r2) != self.rpm_max_real.get().strip():
            self.rpm_max_real.delete(0, tk.END)
            self.rpm_max_real.insert(0, str(r2))

        # In gear mode, clamp gear to 1..7
        if self.mode_var.get() == "gear":
            try:
                g = parse_int_or_default(self.gear_max.get(), self._last_gear_nonzero)
            except Exception:
                g = self._last_gear_nonzero
            g2 = clamp_int(g, 1, MAX_GEAR_LIMIT)
            self._last_gear_nonzero = g2
            if str(g2) != self.gear_max.get().strip():
                self.gear_max.delete(0, tk.END)
                self.gear_max.insert(0, str(g2))

        self.update_map_row_visibility()
        self.update_ratio_visibility()
        self.clamp_visible_cells_to_maxboost()

    # -------- Defaults/actions --------
    def set_defaults(self):
        self.mode_var.set("gear")
        self.gear_max.state(["!disabled"])
        self.gear_max.delete(0, tk.END); self.gear_max.insert(0, "7")
        self._last_gear_nonzero = 7

        self.rpm_max_real.delete(0, tk.END); self.rpm_max_real.insert(0, "8000")
        self.boost_max.delete(0, tk.END); self.boost_max.insert(0, "44")

        defaults = ["4.056", "2.438", "1.548", "1.121", "0.971", "0.815", "0.700"]
        for e, v in zip(self.ratio_entries, defaults):
            e.delete(0, tk.END); e.insert(0, v)
        self.fd_entry.delete(0, tk.END); self.fd_entry.insert(0, "3.900")

        for r in range(7):
            for c in range(7):
                self.table_entries[r][c].delete(0, tk.END)
                self.table_entries[r][c].insert(0, "10")

        self.on_mode_change()

    def fill_map_with_max(self):
        bmax = self.max_boost_value()
        vis = self.visible_rows()
        for r in range(vis):
            for c in range(7):
                self.table_entries[r][c].delete(0, tk.END)
                self.table_entries[r][c].insert(0, str(bmax))

    # -------- Serialization --------
    def to_profile_data(self) -> bytes:
        self.enforce_limits_and_layout()

        mode = self.mode_var.get()

        # Max gear
        if mode == "rpm":
            gear = 0
        else:
            gear = self.visible_rows()  # already clamped 1..7

        # Max RPM -> index 2..8
        try:
            rpm_real = parse_int_or_default(self.rpm_max_real.get(), 8000)
        except Exception:
            rpm_real = 8000
        rpm_real = clamp_int(rpm_real, 0, MAX_RPM_LIMIT)
        if rpm_real not in RPM_TO_INDEX:
            raise ValueError(f"Max RPM must be one of {FIXED_RPM_POINTS}")
        rpm_index = RPM_TO_INDEX[rpm_real]  # 2..8

        # Max boost
        boost = self.max_boost_value()

        # Ratios: always send 8*uint16 (7 gears + final drive)
        ratios_u16: list[int] = []
        if mode == "rpm":
            ratios_u16 = [FILL_UNUSED_U16] * 8
        else:
            # Gear ratios: unused gears beyond max gear -> 0xFFFF
            for i in range(7):
                if i < gear:
                    ratios_u16.append(ratio_to_u16(self.ratio_entries[i].get()))
                else:
                    ratios_u16.append(FILL_UNUSED_U16)
            # Final drive always used
            ratios_u16.append(ratio_to_u16(self.fd_entry.get()))

        # Table: visible rows from UI, rest 0xFF
        vis_rows = 1 if mode == "rpm" else gear
        table_bytes: list[int] = []
        for r in range(7):
            for c in range(7):
                if r < vis_rows:
                    try:
                        v = parse_int_or_default(self.table_entries[r][c].get(), 0)
                    except Exception:
                        v = 0
                    v = clamp_int(v, 0, boost)
                    table_bytes.append(v)
                else:
                    table_bytes.append(FILL_UNUSED_U8)

        out = bytearray()
        out += bytes([gear, rpm_index, boost])
        for v in ratios_u16:
            out += pack_u16_le(v)
        out += bytes(table_bytes)

        if len(out) != PROFILE_DATA_LEN:
            raise RuntimeError(f"Profile packing error (got {len(out)}, expected {PROFILE_DATA_LEN})")
        return bytes(out)

    def load_from_profile_data(self, data: bytes):
        if len(data) != PROFILE_DATA_LEN:
            raise ValueError("Bad profile length")

        gear = data[0]
        rpm_index = data[1]
        boost = data[2]

        self.mode_var.set("rpm" if gear == 0 else "gear")

        if gear == 0:
            self.gear_max.state(["disabled"])
            self.gear_max.delete(0, tk.END); self.gear_max.insert(0, "0")
        else:
            g = clamp_int(int(gear), 1, MAX_GEAR_LIMIT)
            self._last_gear_nonzero = g
            self.gear_max.state(["!disabled"])
            self.gear_max.delete(0, tk.END); self.gear_max.insert(0, str(g))

        rpm_real = INDEX_TO_RPM.get(rpm_index, 8000)
        rpm_real = clamp_int(rpm_real, 0, MAX_RPM_LIMIT)
        self.rpm_max_real.delete(0, tk.END); self.rpm_max_real.insert(0, str(rpm_real))

        b = clamp_int(int(boost), 0, MAX_BOOST_LIMIT)
        self.boost_max.delete(0, tk.END); self.boost_max.insert(0, str(b))

        ratios_bytes = data[3:19]
        ratios_u16 = [unpack_u16_le(ratios_bytes[i*2], ratios_bytes[i*2+1]) for i in range(8)]

        if self.mode_var.get() == "gear":
            # 7 gear ratios
            for i in range(7):
                self.ratio_entries[i].delete(0, tk.END)
                if ratios_u16[i] == FILL_UNUSED_U16:
                    self.ratio_entries[i].insert(0, "")
                else:
                    self.ratio_entries[i].insert(0, u16_to_ratio_str(ratios_u16[i]))
            # final drive
            self.fd_entry.delete(0, tk.END)
            self.fd_entry.insert(0, "" if ratios_u16[7] == FILL_UNUSED_U16 else u16_to_ratio_str(ratios_u16[7]))

        table = data[19:]
        k = 0
        for r in range(7):
            for c in range(7):
                val = int(table[k])
                self.table_entries[r][c].delete(0, tk.END)
                self.table_entries[r][c].insert(0, "0" if val == 0xFF else str(val))
                k += 1

        self.on_mode_change()
        self.enforce_limits_and_layout()


# ---------------- App ----------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arduino Boost Profile Manager (Profiles 1–4)")

        # Fixed size to avoid resize weirdness with canvas + grid
        self.geometry("1100x820")
        self.resizable(False, False)

        self.colors = apply_dark_theme(self)

        self.ser: serial.Serial | None = None
        self.serial_lock = threading.Lock()

        # Top bar
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port:", font=("Segoe UI", 10, "bold")).pack(side="left")
        self.port_var = tk.StringVar(value="")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.pack(side="left", padx=(8, 6))

        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left", padx=4)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=(10, 6))

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Label(top, text="Baud:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Label(top, text="115200").pack(side="left", padx=(6, 0))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var, style="Muted.TLabel").pack(side="right")

        # Notebook (profiles)
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        self.editors: list[ProfileEditor] = []
        for i in range(1, 5):
            ed = ProfileEditor(self.nb, i, self.colors)
            self.nb.add(ed, text=f"Profile {i}")
            self.editors.append(ed)

        # Actions
        actions = ttk.Frame(self, padding=(10, 0, 10, 10))
        actions.pack(fill="x")

        ttk.Button(actions, text="Upload Selected Profile → Arduino", command=self.upload_selected).pack(side="left", padx=4)
        ttk.Button(actions, text="Read Selected Profile ← Arduino", command=self.read_selected).pack(side="left", padx=4)

        ttk.Separator(actions, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Button(actions, text="Copy Selected Profile → Others", command=self.copy_selected_to_others).pack(side="left", padx=4)

        # Activity log
        self.log_group = RoundedGroup(self, "Activity Log", self.colors, radius=12, pad=12)
        self.log_group.pack(fill="x", padx=10, pady=(0, 10))

        self.log = tk.Text(
            self.log_group.body,
            height=6,
            wrap="word",
            bg=self.colors["FIELD"],
            fg=self.colors["TEXT"],
            insertbackground=self.colors["TEXT"],
            relief="flat",
            highlightthickness=1,
            highlightbackground=self.colors["BORDER_SOFT"],
            highlightcolor=self.colors["BORDER_SOFT"],
        )
        self.log.pack(fill="both", expand=True)

        self.refresh_ports()

    def log_msg(self, s: str):
        self.log.insert(tk.END, s + "\n")
        self.log.see(tk.END)

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and (not self.port_var.get() or self.port_var.get() not in ports):
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.connect_btn.config(text="Connect")
            self.status_var.set("Disconnected")
            self.log_msg("Disconnected.")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return

        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.connect_btn.config(text="Disconnect")
            self.status_var.set(f"Connected: {port}")
            self.after(200, lambda: self.log_msg(f"Connected to {port} @ 115200."))
        except Exception as e:
            self.ser = None
            messagebox.showerror("Connect failed", str(e))

    def selected_profile_index_1based(self) -> int:
        return self.nb.index(self.nb.select()) + 1

    def get_selected_editor(self) -> ProfileEditor:
        return self.editors[self.selected_profile_index_1based() - 1]

    def ensure_connected(self):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Not connected to a serial port")

    def serial_request(self, pkt: bytes) -> tuple[int, bytes]:
        self.ensure_connected()
        with self.serial_lock:
            self.ser.reset_input_buffer()
            self.ser.write(pkt)
            self.ser.flush()
            return read_packet(self.ser)

    def upload_selected(self):
        ed = self.get_selected_editor()
        prof_idx = self.selected_profile_index_1based()

        try:
            data68 = ed.to_profile_data()
            payload = bytes([prof_idx]) + data68
            pkt = build_packet(MSG_UPLOAD_PROFILE, payload)
        except Exception as e:
            messagebox.showerror("Upload error", str(e))
            return

        self.log_msg(f"Uploading Profile {prof_idx}...")
        threading.Thread(target=self._upload_worker, args=(pkt,), daemon=True).start()

    def _upload_worker(self, pkt: bytes):
        try:
            msg_type, payload = self.serial_request(pkt)
            if msg_type == MSG_READ_PROFILE_RESP and len(payload) == UPLOAD_PAYLOAD_LEN:
                self.log_msg(f"Upload OK (Arduino echoed Profile {payload[0]}).")
            elif msg_type == MSG_ERROR and payload:
                self.log_msg(f"Arduino ERROR: 0x{payload[0]:02X}")
            else:
                self.log_msg(f"Unexpected response 0x{msg_type:02X} len={len(payload)}")
        except Exception as e:
            self.log_msg(f"Upload failed: {e}")

    def read_selected(self):
        prof_idx = self.selected_profile_index_1based()
        ed = self.get_selected_editor()

        pkt = build_packet(MSG_READ_PROFILE_REQ, bytes([prof_idx]))
        self.log_msg(f"Reading Profile {prof_idx} from Arduino...")
        threading.Thread(target=self._read_worker, args=(pkt, ed), daemon=True).start()

    def _read_worker(self, pkt: bytes, ed: ProfileEditor):
        try:
            msg_type, payload = self.serial_request(pkt)
            if msg_type == MSG_READ_PROFILE_RESP:
                if len(payload) != UPLOAD_PAYLOAD_LEN:
                    self.log_msg(f"Bad READ_PROFILE_RESP length: {len(payload)}")
                    return
                idx1 = payload[0]
                data68 = payload[1:]
                self.after(0, lambda: ed.load_from_profile_data(data68))
                self.log_msg(f"Loaded Profile {idx1} into UI.")
            elif msg_type == MSG_ERROR and payload:
                self.log_msg(f"Arduino ERROR: 0x{payload[0]:02X}")
            else:
                self.log_msg(f"Unexpected response 0x{msg_type:02X}")
        except Exception as e:
            self.log_msg(f"Read failed: {e}")

    def copy_selected_to_others(self):
        src_idx = self.selected_profile_index_1based()
        src_editor = self.get_selected_editor()

        try:
            data68 = src_editor.to_profile_data()
        except Exception as e:
            messagebox.showerror("Copy error", str(e))
            return

        for i, ed in enumerate(self.editors, start=1):
            if i == src_idx:
                continue
            ed.load_from_profile_data(data68)

        self.log_msg(f"Copied Profile {src_idx} into the other profiles (UI only).")


if __name__ == "__main__":
    # pip install pyserial
    App().mainloop()
