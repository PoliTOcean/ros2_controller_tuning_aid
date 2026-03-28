#!/usr/bin/env python3

import subprocess
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Tuple

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


# ── Palette ───────────────────────────────────────────────────────────────────
BG_DEEP    = "#0a0e1a"
BG_PANEL   = "#111827"
BG_ENTRY   = "#1c2436"
BG_HEADER  = "#0d1525"
BG_ROW_ALT = "#131c2e"
BORDER     = "#1e3a5f"
ACCENT     = "#00c8ff"
ACCENT2    = "#0077aa"
TEXT_MAIN  = "#e0eaf8"
TEXT_DIM   = "#6b8cae"
TEXT_LABEL = "#a0c4e0"
SUCCESS    = "#00e676"
WARNING    = "#ffb300"
ERROR      = "#ff5252"

FONT_TITLE  = ("Courier New", 10, "bold")
FONT_LABEL  = ("Courier New", 9)
FONT_ENTRY  = ("Courier New", 9)
FONT_BUTTON = ("Courier New", 9, "bold")
FONT_STATUS = ("Courier New", 9)
FONT_HEAD   = ("Courier New", 8, "bold")


class PidTunerGui(Node):
    def __init__(self) -> None:
        super().__init__("nereo_pid_tuner_gui")

        self.target_node = self.declare_parameter(
            "target_node", "/nereo_controller_node"
        ).value
        self.get_parameters_client = self.create_client(
            GetParameters, f"{self.target_node}/get_parameters"
        )
        self.set_parameters_client = self.create_client(
            SetParameters, f"{self.target_node}/set_parameters"
        )

        self.root = tk.Tk()
        self.root.title("NEREO — PID Controller Tuner")
        self.root.geometry("900x700")
        self.root.minsize(580, 400)
        self.root.configure(bg=BG_DEEP)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.resizable(True, True)

        self.is_running          = True
        self.connected           = False
        self.plotjuggler_process = None
        self.status_var          = tk.StringVar(value="⬡  Attendo connessione ai servizi parametri…")
        self.panel_selector_var  = tk.StringVar(value="All")
        self.section_expanded    = {
            "CONTROL MODE": True,
            "PID GAINS": True,
            "MANUAL SETPOINTS": True,
            "CS CONTROLLER": True,
        }

        self.control_mode_var = tk.StringVar(value="0")
        self.pid_vars: Dict[str, List[tk.StringVar]] = {
            "kp": [tk.StringVar(value="0.0") for _ in range(4)],
            "ki": [tk.StringVar(value="0.0") for _ in range(4)],
            "kd": [tk.StringVar(value="0.0") for _ in range(4)],
        }
        self.manual_vars: Dict[str, tk.BooleanVar] = {
            "manual_setpoint_depth": tk.BooleanVar(value=False),
            "manual_setpoint_roll":  tk.BooleanVar(value=False),
            "manual_setpoint_pitch": tk.BooleanVar(value=False),
            "manual_setpoint_yaw":   tk.BooleanVar(value=False),
        }
        self.setpoint_vars: Dict[str, tk.StringVar] = {
            "setpoint_depth": tk.StringVar(value="0.0"),
            "setpoint_roll":  tk.StringVar(value="0.0"),
            "setpoint_pitch": tk.StringVar(value="0.0"),
            "setpoint_yaw":   tk.StringVar(value="0.0"),
        }
        self.cs_array_vars: Dict[str, List[tk.StringVar]] = {
            "cs_kx0": [tk.StringVar(value="0.0"), tk.StringVar(value="0.0")],
            "cs_kx1": [tk.StringVar(value="0.0"), tk.StringVar(value="0.0")],
            "cs_kx2": [tk.StringVar(value="0.0"), tk.StringVar(value="0.0")],
        }
        self.cs_scalar_vars: Dict[str, tk.StringVar] = {
            "cs_ki0":       tk.StringVar(value="0.0"),
            "cs_ki1":       tk.StringVar(value="0.0"),
            "cs_ki2":       tk.StringVar(value="0.0"),
            "cs_heave_min": tk.StringVar(value="0.0"),
            "cs_heave_max": tk.StringVar(value="0.0"),
            "cs_angle_min": tk.StringVar(value="0.0"),
            "cs_angle_max": tk.StringVar(value="0.0"),
        }
        self.parameter_order = [
            "control_mode",
            "kp", "ki", "kd",
            "manual_setpoint_depth", "manual_setpoint_roll",
            "manual_setpoint_pitch", "manual_setpoint_yaw",
            "setpoint_depth", "setpoint_roll", "setpoint_pitch", "setpoint_yaw",
            "cs_kx0", "cs_kx1", "cs_kx2",
            "cs_ki0", "cs_ki1", "cs_ki2",
            "cs_heave_min", "cs_heave_max", "cs_angle_min", "cs_angle_max",
        ]

        self._build_ui()

    # ── widget helpers ────────────────────────────────────────────────────────

    def _label(self, parent, text, font=FONT_LABEL, fg=TEXT_MAIN, **kw):
        bg = parent.cget("bg") if hasattr(parent, "cget") else BG_PANEL
        return tk.Label(parent, text=text, font=font, fg=fg, bg=bg, **kw)

    def _entry(self, parent, textvariable, width=12):
        e = tk.Entry(parent, textvariable=textvariable, width=width,
                     font=FONT_ENTRY, fg=TEXT_MAIN, bg=BG_ENTRY,
                     insertbackground=ACCENT, relief="flat",
                     highlightthickness=1,
                     highlightcolor=ACCENT, highlightbackground=BORDER)
        e.bind("<FocusIn>",  lambda _: e.config(highlightbackground=ACCENT))
        e.bind("<FocusOut>", lambda _: e.config(highlightbackground=BORDER))
        return e

    def _option_menu(self, parent, variable, *values, fg=TEXT_MAIN,
                     command=None, width=6):
        kw = {"command": command} if command else {}
        m = tk.OptionMenu(parent, variable, *values, **kw)
        m.config(font=FONT_ENTRY, bg=BG_ENTRY, fg=fg,
                 activebackground=ACCENT2, activeforeground=TEXT_MAIN,
                 relief="flat", highlightthickness=1,
                 highlightbackground=BORDER, width=width, cursor="hand2")
        m["menu"].config(bg=BG_ENTRY, fg=TEXT_MAIN,
                         activebackground=ACCENT2, activeforeground=TEXT_MAIN,
                         font=FONT_ENTRY, relief="flat", bd=0)
        return m

    def _button(self, parent, text, command, primary=False):
        bg  = ACCENT  if primary else BG_ENTRY
        fg  = BG_DEEP if primary else ACCENT
        hbg = "#00e0ff" if primary else BORDER
        b = tk.Button(parent, text=text, command=command,
                      font=FONT_BUTTON, fg=fg, bg=bg,
                      activebackground=hbg, activeforeground=BG_DEEP,
                      relief="flat", cursor="hand2", padx=14, pady=5,
                      bd=0, highlightthickness=0)
        b.bind("<Enter>", lambda _: b.config(bg=hbg))
        b.bind("<Leave>", lambda _: b.config(bg=bg))
        return b

    def _section_frame(self, parent, title):
        outer = tk.Frame(parent, bg=BG_PANEL,
                         highlightthickness=1, highlightbackground=BORDER)
        outer.columnconfigure(0, weight=1)
        
        # Clickable header
        header = tk.Frame(outer, bg=BG_HEADER, cursor="hand2")
        header.grid(row=0, column=0, sticky="ew")
        
        header_label = tk.Label(header, text=f"  ▾  {title}", font=FONT_TITLE,
                                fg=ACCENT, bg=BG_HEADER, pady=6, cursor="hand2")
        header_label.pack(side="left")
        
        inner = tk.Frame(outer, bg=BG_PANEL, padx=12, pady=8)
        inner.grid(row=1, column=0, sticky="nsew")
        outer.rowconfigure(1, weight=1)
        
        # Store references for toggle
        outer._header_label = header_label
        outer._inner_frame = inner
        outer._title = title
        
        def toggle_section():
            expanded = self.section_expanded.get(title, True)
            self.section_expanded[title] = not expanded
            new_expanded = self.section_expanded[title]
            
            if new_expanded:
                inner.grid(row=1, column=0, sticky="nsew")
                header_label.config(text=f"  ▾  {title}")
            else:
                inner.grid_remove()
                header_label.config(text=f"  ▸  {title}")
        
        header.bind("<Button-1>", lambda _: toggle_section())
        header_label.bind("<Button-1>", lambda _: toggle_section())
        
        return outer, inner

    def _col_header(self, parent, text, col):
        tk.Label(parent, text=text, font=FONT_HEAD,
                 fg=TEXT_LABEL, bg=BG_PANEL).grid(
            row=0, column=col, sticky="w", padx=6, pady=(0, 4))

    # ── main UI ───────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        # root grid: col 0 fills width, row 1 is scrollable
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        # ── Fixed top bar ─────────────────────────────────────────────────────
        topbar = tk.Frame(self.root, bg=BG_HEADER,
                          highlightthickness=1, highlightbackground=BORDER)
        topbar.grid(row=0, column=0, sticky="ew")
        topbar.columnconfigure(1, weight=1)

        tk.Label(topbar, text="  ◈  NEREO",
                 font=("Courier New", 13, "bold"),
                 fg=ACCENT, bg=BG_HEADER, pady=10).grid(
            row=0, column=0, sticky="w")
        tk.Label(topbar, text="PID CONTROLLER TUNER",
                 font=("Courier New", 9), fg=TEXT_DIM, bg=BG_HEADER).grid(
            row=0, column=1, sticky="w", padx=(6, 0))

        btn_f = tk.Frame(topbar, bg=BG_HEADER)
        btn_f.grid(row=0, column=2, sticky="e", padx=12)
        self._button(btn_f, "⟳  Carica da nodo",
                     self.load_parameters).pack(side="left", padx=4)
        self._button(btn_f, "✔  Applica tutto",
                     self.apply_parameters, primary=True).pack(side="left", padx=4)
        self._button(btn_f, "📈  PlotJuggler",
                     self.launch_plotjuggler).pack(side="left", padx=4)

        # ── Scrollable canvas (no visible scrollbar, mousewheel only) ─────────
        canvas = tk.Canvas(self.root, bg=BG_DEEP, highlightthickness=0)
        canvas.grid(row=1, column=0, sticky="nsew")

        content = tk.Frame(canvas, bg=BG_DEEP)
        cw_id = canvas.create_window((0, 0), window=content, anchor="nw")

        # content tracks canvas width → responsive
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfig(cw_id, width=e.width))
        content.bind("<Configure>",
                     lambda e: canvas.configure(
                         scrollregion=canvas.bbox("all")))

        # mouse wheel scroll (Linux Button-4/5, Windows/Mac MouseWheel)
        def _scroll(e):
            if e.num == 4:
                canvas.yview_scroll(-1, "units")
            elif e.num == 5:
                canvas.yview_scroll(1, "units")
            else:
                canvas.yview_scroll(int(-e.delta / 120), "units")
        for seq in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            self.root.bind_all(seq, _scroll)

        content.columnconfigure(0, weight=1)

        # ── Status strip ──────────────────────────────────────────────────────
        status_f = tk.Frame(content, bg=BG_DEEP)
        status_f.grid(row=0, column=0, sticky="ew", padx=14, pady=(10, 4))
        self._status_dot = tk.Label(status_f, text="●",
                                    font=("Courier New", 9),
                                    fg=WARNING, bg=BG_DEEP)
        self._status_dot.pack(side="left")
        tk.Label(status_f, textvariable=self.status_var,
                 font=FONT_STATUS, fg=TEXT_DIM, bg=BG_DEEP).pack(
            side="left", padx=(4, 0))

        # ── Control Mode ──────────────────────────────────────────────────────
        outer, inner = self._section_frame(content, "CONTROL MODE")
        outer.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 8))
        inner.columnconfigure(2, weight=1)

        # row 0: mode selector + description
        self._label(inner, "Mode", fg=TEXT_LABEL).grid(
            row=0, column=0, sticky="w", padx=(0, 6))
        self._option_menu(inner, self.control_mode_var,
                          "0", "1", "2", "3", width=4).grid(
            row=0, column=1, sticky="w", padx=(0, 12))
        self._label(inner,
                    "0: passthrough  │  1: PID  │  2: PID anti-windup  │  3: CS",
                    fg=TEXT_DIM).grid(row=0, column=2, sticky="w")

        # separator
        tk.Frame(inner, bg=BORDER, height=1).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(8, 6))

        # row 2: panel filter as inline button group
        self._label(inner, "Vista:", fg=TEXT_LABEL).grid(
            row=2, column=0, sticky="w", padx=(0, 8))
        btn_row = tk.Frame(inner, bg=BG_PANEL)
        btn_row.grid(row=2, column=1, columnspan=2, sticky="w")

        self._panel_btns: Dict[str, tk.Button] = {}
        for p in ["All", "PID", "Setpoint", "CS"]:
            active = (p == "All")
            b = tk.Button(
                btn_row, text=p, font=FONT_BUTTON,
                fg=BG_DEEP if active else ACCENT,
                bg=ACCENT if active else BG_ENTRY,
                activebackground="#00e0ff", activeforeground=BG_DEEP,
                relief="flat", cursor="hand2",
                padx=12, pady=3, bd=0, highlightthickness=0,
                command=lambda p=p: self._select_panel(p),
            )
            b.pack(side="left", padx=(0, 4))
            self._panel_btns[p] = b

        # ── PID Gains ─────────────────────────────────────────────────────────
        self._pid_outer, pid_inner = self._section_frame(
            content, "PID GAINS  —  depth · roll · pitch · yaw")
        self._pid_outer.grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 8))
        pid_inner.columnconfigure(0, weight=1, minsize=70)
        for c in range(1, 4):
            pid_inner.columnconfigure(c, weight=3)

        self._col_header(pid_inner, "AXIS", 0)
        self._col_header(pid_inner, "Kp",   1)
        self._col_header(pid_inner, "Ki",   2)
        self._col_header(pid_inner, "Kd",   3)

        axis_names = ["Depth", "Roll", "Pitch", "Yaw"]
        axis_icons = ["⬇", "↺", "↕", "↻"]
        for i, (axis, icon) in enumerate(zip(axis_names, axis_icons)):
            r = i + 1
            bg_r = BG_PANEL if i % 2 == 0 else BG_ROW_ALT
            for col in range(4):
                tk.Frame(pid_inner, bg=bg_r, height=32).grid(
                    row=r, column=col, sticky="nsew")
            self._label(pid_inner, f"{icon}  {axis}", fg=TEXT_MAIN).grid(
                row=r, column=0, sticky="w", padx=6, pady=3)
            for ci, key in enumerate(["kp", "ki", "kd"]):
                self._entry(pid_inner, self.pid_vars[key][i]).grid(
                    row=r, column=ci + 1, sticky="ew", padx=6, pady=3)

        # ── Manual Setpoints ──────────────────────────────────────────────────
        self._sp_outer, sp_inner = self._section_frame(
            content, "MANUAL SETPOINTS")
        self._sp_outer.grid(row=3, column=0, sticky="ew", padx=14, pady=(0, 8))
        sp_inner.columnconfigure(0, weight=1, minsize=70)
        sp_inner.columnconfigure(1, weight=0, minsize=60)
        sp_inner.columnconfigure(2, weight=3)

        self._col_header(sp_inner, "AXIS",     0)
        self._col_header(sp_inner, "MANUAL",   1)
        self._col_header(sp_inner, "SETPOINT", 2)

        sp_keys  = ["depth", "roll", "pitch", "yaw"]
        sp_icons = ["⬇", "↺", "↕", "↻"]
        for i, (key, icon) in enumerate(zip(sp_keys, sp_icons)):
            r = i + 1
            bg_r = BG_PANEL if i % 2 == 0 else BG_ROW_ALT
            for col in range(3):
                tk.Frame(sp_inner, bg=bg_r, height=32).grid(
                    row=r, column=col, sticky="nsew")
            self._label(sp_inner, f"{icon}  {key.capitalize()}",
                        fg=TEXT_MAIN).grid(
                row=r, column=0, sticky="w", padx=6, pady=3)
            tk.Checkbutton(
                sp_inner,
                variable=self.manual_vars[f"manual_setpoint_{key}"],
                bg=bg_r, fg=ACCENT,
                activebackground=bg_r, activeforeground=ACCENT,
                selectcolor=BG_ENTRY, relief="flat",
                highlightthickness=0, cursor="hand2",
            ).grid(row=r, column=1, pady=3)
            self._entry(sp_inner,
                        self.setpoint_vars[f"setpoint_{key}"],
                        width=18).grid(
                row=r, column=2, sticky="ew", padx=6, pady=3)

        # ── CS Controller ─────────────────────────────────────────────────────
        self._cs_outer, cs_inner = self._section_frame(
            content, "CS CONTROLLER")
        self._cs_outer.grid(row=4, column=0, sticky="ew", padx=14, pady=(0, 8))
        cs_inner.columnconfigure(0, weight=2, minsize=110)
        cs_inner.columnconfigure(1, weight=3)
        cs_inner.columnconfigure(2, weight=3)

        self._col_header(cs_inner, "PARAM",   0)
        self._col_header(cs_inner, "VALUE 1", 1)
        self._col_header(cs_inner, "VALUE 2", 2)

        array_rows = [
            ("cs_kx0", "cs_kx0  (heave)"),
            ("cs_kx1", "cs_kx1  (roll)"),
            ("cs_kx2", "cs_kx2  (pitch)"),
        ]
        for i, (key, lbl) in enumerate(array_rows):
            r = i + 1
            bg_r = BG_PANEL if i % 2 == 0 else BG_ROW_ALT
            for col in range(3):
                tk.Frame(cs_inner, bg=bg_r, height=30).grid(
                    row=r, column=col, sticky="nsew")
            self._label(cs_inner, lbl, fg=TEXT_MAIN).grid(
                row=r, column=0, sticky="w", padx=6, pady=3)
            self._entry(cs_inner, self.cs_array_vars[key][0]).grid(
                row=r, column=1, sticky="ew", padx=6, pady=3)
            self._entry(cs_inner, self.cs_array_vars[key][1]).grid(
                row=r, column=2, sticky="ew", padx=6, pady=3)

        scalar_rows = [
            ("cs_ki0",       "cs_ki0"),
            ("cs_ki1",       "cs_ki1"),
            ("cs_ki2",       "cs_ki2"),
            ("cs_heave_min", "cs_heave_min"),
            ("cs_heave_max", "cs_heave_max"),
            ("cs_angle_min", "cs_angle_min"),
            ("cs_angle_max", "cs_angle_max"),
        ]
        base = len(array_rows) + 1
        for i, (key, lbl) in enumerate(scalar_rows):
            r = base + i
            bg_r = BG_PANEL if (len(array_rows) + i) % 2 == 0 else BG_ROW_ALT
            for col in range(3):
                tk.Frame(cs_inner, bg=bg_r, height=30).grid(
                    row=r, column=col, sticky="nsew")
            self._label(cs_inner, lbl, fg=TEXT_MAIN).grid(
                row=r, column=0, sticky="w", padx=6, pady=3)
            self._entry(cs_inner, self.cs_scalar_vars[key]).grid(
                row=r, column=1, sticky="ew", padx=6, pady=3)

        # ── Footer ────────────────────────────────────────────────────────────
        tk.Label(content,
                 text="  GUI → ROS2 parameter server  ·  NEREO ROV Controller Tuning Aid",
                 font=("Courier New", 8), fg=TEXT_DIM, bg=BG_DEEP).grid(
            row=5, column=0, sticky="w", padx=14, pady=(2, 12))

        self._apply_panel_visibility()

    # ── Panel selector ────────────────────────────────────────────────────────

    def _select_panel(self, panel: str) -> None:
        self.panel_selector_var.set(panel)
        for p, b in self._panel_btns.items():
            if p == panel:
                b.config(bg=ACCENT, fg=BG_DEEP)
            else:
                b.config(bg=BG_ENTRY, fg=ACCENT)
        self._apply_panel_visibility()

    def _apply_panel_visibility(self) -> None:
        sel = self.panel_selector_var.get()
        for frame, panels in [
            (self._pid_outer, ("All", "PID")),
            (self._sp_outer,  ("All", "Setpoint")),
            (self._cs_outer,  ("All", "CS")),
        ]:
            if sel in panels:
                frame.grid()
            else:
                frame.grid_remove()

    # ── Status ────────────────────────────────────────────────────────────────

    def _set_status(self, msg: str, color: str = TEXT_DIM) -> None:
        self.status_var.set(msg)
        self._status_dot.config(fg=color)

    # ── ROS2 logic ────────────────────────────────────────────────────────────

    def update_connection_status(self) -> None:
        is_connected = self._services_ready()
        if is_connected != self.connected:
            self.connected = is_connected
            if self.connected:
                self._set_status(
                    f"Connesso a  {self.target_node}  — carico parametri…", SUCCESS)
                self.load_parameters()
            else:
                self._set_status(
                    f"In attesa servizi parametri di  {self.target_node}…", WARNING)

    def _services_ready(self) -> bool:
        return (self.get_parameters_client.wait_for_service(timeout_sec=0.0) and
                self.set_parameters_client.wait_for_service(timeout_sec=0.0))

    def load_parameters(self) -> None:
        if not self._services_ready():
            self._set_status("Servizi parametri non disponibili", ERROR)
            return
        request = GetParameters.Request()
        request.names = self.parameter_order
        future = self.get_parameters_client.call_async(request)
        future.add_done_callback(self._on_load_done)
        self._set_status("Lettura parametri in corso…", WARNING)

    def _on_load_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._set_status(f"Errore lettura parametri: {exc}", ERROR)
            return
        for name, value in zip(self.parameter_order, result.values):
            decoded = self._decode_parameter_value(value)
            if decoded is None:
                continue
            self._set_ui_value(name, decoded)
        self._set_status("Parametri caricati dal nodo", SUCCESS)

    def apply_parameters(self) -> None:
        if not self._services_ready():
            self._set_status("Servizi parametri non disponibili", ERROR)
            return
        try:
            parameters = [
                Parameter(name="control_mode",
                          value=int(self.control_mode_var.get())),
                Parameter(name="kp",
                          value=self._parse_array(self.pid_vars["kp"])),
                Parameter(name="ki",
                          value=self._parse_array(self.pid_vars["ki"])),
                Parameter(name="kd",
                          value=self._parse_array(self.pid_vars["kd"])),
                Parameter(name="manual_setpoint_depth",
                          value=bool(self.manual_vars["manual_setpoint_depth"].get())),
                Parameter(name="manual_setpoint_roll",
                          value=bool(self.manual_vars["manual_setpoint_roll"].get())),
                Parameter(name="manual_setpoint_pitch",
                          value=bool(self.manual_vars["manual_setpoint_pitch"].get())),
                Parameter(name="manual_setpoint_yaw",
                          value=bool(self.manual_vars["manual_setpoint_yaw"].get())),
                Parameter(name="setpoint_depth",
                          value=self._parse_float_var(self.setpoint_vars["setpoint_depth"])),
                Parameter(name="setpoint_roll",
                          value=self._parse_float_var(self.setpoint_vars["setpoint_roll"])),
                Parameter(name="setpoint_pitch",
                          value=self._parse_float_var(self.setpoint_vars["setpoint_pitch"])),
                Parameter(name="setpoint_yaw",
                          value=self._parse_float_var(self.setpoint_vars["setpoint_yaw"])),
                Parameter(name="cs_kx0",
                          value=self._parse_array(self.cs_array_vars["cs_kx0"])),
                Parameter(name="cs_kx1",
                          value=self._parse_array(self.cs_array_vars["cs_kx1"])),
                Parameter(name="cs_kx2",
                          value=self._parse_array(self.cs_array_vars["cs_kx2"])),
                Parameter(name="cs_ki0",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_ki0"])),
                Parameter(name="cs_ki1",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_ki1"])),
                Parameter(name="cs_ki2",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_ki2"])),
                Parameter(name="cs_heave_min",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_heave_min"])),
                Parameter(name="cs_heave_max",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_heave_max"])),
                Parameter(name="cs_angle_min",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_angle_min"])),
                Parameter(name="cs_angle_max",
                          value=self._parse_float_var(self.cs_scalar_vars["cs_angle_max"])),
            ]
        except ValueError as exc:
            self._set_status(f"Valore non valido: {exc}", ERROR)
            return

        request = SetParameters.Request()
        request.parameters = [item.to_parameter_msg() for item in parameters]
        future = self.set_parameters_client.call_async(request)
        future.add_done_callback(self._on_apply_done)
        self._set_status("Scrittura parametri in corso…", WARNING)

    def _on_apply_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._set_status(f"Errore scrittura parametri: {exc}", ERROR)
            return
        failed = [res.reason for res in response.results if not res.successful]
        if failed:
            self._set_status(f"Parametri rifiutati: {'; '.join(failed)}", ERROR)
        else:
            self._set_status("Parametri applicati correttamente ✔", SUCCESS)

    def _parse_float_var(self, var: tk.StringVar) -> float:
        normalized = var.get().strip().replace(",", ".")
        var.set(normalized)
        return float(normalized)

    def _parse_array(self, vars_list: List[tk.StringVar]) -> List[float]:
        return [self._parse_float_var(item) for item in vars_list]

    def _decode_parameter_value(self, param_value):
        ptype = param_value.type
        if ptype == ParameterType.PARAMETER_BOOL:
            return param_value.bool_value
        if ptype == ParameterType.PARAMETER_INTEGER:
            return param_value.integer_value
        if ptype == ParameterType.PARAMETER_DOUBLE:
            return param_value.double_value
        if ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(param_value.double_array_value)
        return None

    def _set_ui_value(self, name: str, value) -> None:
        if name == "control_mode":
            self.control_mode_var.set(str(value))
            return
        if name in ("kp", "ki", "kd") and isinstance(value, list) and len(value) == 4:
            for i in range(4):
                self.pid_vars[name][i].set(f"{float(value[i]):.6g}")
            return
        if name in self.manual_vars and isinstance(value, bool):
            self.manual_vars[name].set(value)
            return
        if name in self.setpoint_vars:
            self.setpoint_vars[name].set(f"{float(value):.6g}")
            return
        if name in self.cs_array_vars and isinstance(value, list) and len(value) == 2:
            self.cs_array_vars[name][0].set(f"{float(value[0]):.6g}")
            self.cs_array_vars[name][1].set(f"{float(value[1]):.6g}")
            return
        if name in self.cs_scalar_vars:
            self.cs_scalar_vars[name].set(f"{float(value):.6g}")

    def launch_plotjuggler(self) -> None:
        if self.plotjuggler_process is not None and self.plotjuggler_process.poll() is None:
            self._set_status("PlotJuggler è già in esecuzione", WARNING)
            return
        try:
            self.plotjuggler_process = subprocess.Popen(
                ["ros2", "run", "plotjuggler", "plotjuggler"])
            self._set_status("PlotJuggler avviato", SUCCESS)
        except FileNotFoundError:
            self._set_status("Comando 'ros2 run plotjuggler plotjuggler' non trovato", ERROR)
        except Exception as exc:
            self._set_status(f"Errore avvio PlotJuggler: {exc}", ERROR)

    def _on_close(self) -> None:
        if self.plotjuggler_process is not None and self.plotjuggler_process.poll() is None:
            self.plotjuggler_process.terminate()
        self.is_running = False
        self.root.quit()

    def spin_in_tk(self) -> None:
        if not self.is_running:
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self.update_connection_status()
        self.root.after(50, self.spin_in_tk)


def main() -> None:
    rclpy.init()
    node = PidTunerGui()
    node.spin_in_tk()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()