import csv
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox


# ================= 路径配置 =================
# 默认使用 windblade.py 所在工程内的脚本目录；也可通过环境变量覆盖。
APP_FOLDER = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_FOLDER = os.path.abspath(
    os.path.expanduser(
        os.path.expandvars(
            os.environ.get(
                "WINDBLADE_SCRIPTS_FOLDER",
                os.path.join(APP_FOLDER, "shfiles", "wind"),
            )
        )
    )
)


# ================= 视觉系统 =================
COLORS = {
    "background": "#080D14",
    "sidebar": "#0C131D",
    "panel": "#101923",
    "panel_alt": "#131F2B",
    "card": "#142230",
    "card_hover": "#192B3B",
    "border": "#223446",
    "border_soft": "#192838",
    "accent": "#35D3A1",
    "accent_dark": "#123F38",
    "blue": "#55A7FF",
    "blue_dark": "#153656",
    "text": "#EAF2F8",
    "muted": "#8295A8",
    "dim": "#53677A",
    "danger": "#FF647C",
    "danger_dark": "#4B202B",
    "warning": "#F6C85F",
    "log": "#09111A",
}

FONT_FAMILY = "Noto Sans CJK SC"
FONT_MONO = "DejaVu Sans Mono"
WINDBLADE_LAUNCH_ID_ENV = "WINDBLADE_LAUNCH_ID"
WINDBLADE_SCRIPT_NAME_ENV = "WINDBLADE_SCRIPT_NAME"
WINDBLADE_PARENT_PID_ENV = "WINDBLADE_PARENT_PID"
TERMINAL_SERVER_NAMES = {
    "gnome-terminal",
    "gnome-terminal-",
    "gnome-terminal-server",
}
DEFAULT_POSE_TOPIC = "/iris_0/mavros/vision_odom/odom"

SCRIPT_DETAILS = (
    (("清理", "cleanup"), ("维护工具", "清理 Gazebo 与 ROS 残留进程", COLORS["danger"])),
    (("点云匹配", "mapping"), ("建图任务", "启动点云匹配与风机建图", COLORS["blue"])),
    (("slam", "fastlio"), ("定位建图", "启动 FAST-LIO 定位与建图", COLORS["blue"])),
    (("真值", "mavros", "ground"), ("位姿链路", "连接 MAVROS 并发布位姿真值", COLORS["accent"])),
    (("仿真", "gazebo", "simulation"), ("仿真环境", "启动风机巡检仿真环境", COLORS["warning"])),
    (("指定高度", "飞到", "80m", "向上飞", "上升", "climb", "px4ctrl持续"), ("飞行控制", "输入高度后 px4ctrl 飞到目标高度悬停", COLORS["accent"])),
    (("2m", "悬停", "hover", "quick_takeoff"), ("飞行控制", "快速起飞并在 2m 处悬停", COLORS["accent"])),
    (("keyboard", "起飞", "takeoff"), ("飞行控制", "启动键盘控制与起飞流程", COLORS["accent"])),
    (("停机", "停桨", "停止", "stop"), ("风机控制", "瞬时停止风机桨叶旋转", COLORS["danger"])),
)


def rounded_rectangle(canvas, x1, y1, x2, y2, radius=14, **kwargs):
    """在 Canvas 上绘制平滑圆角矩形。"""
    radius = max(2, min(radius, (x2 - x1) / 2, (y2 - y1) / 2))
    points = [
        x1 + radius, y1,
        x2 - radius, y1,
        x2, y1,
        x2, y1 + radius,
        x2, y2 - radius,
        x2, y2,
        x2 - radius, y2,
        x1 + radius, y2,
        x1, y2,
        x1, y2 - radius,
        x1, y1 + radius,
        x1, y1,
    ]
    return canvas.create_polygon(points, smooth=True, splinesteps=20, **kwargs)


def script_metadata(filename):
    stem = os.path.splitext(filename)[0]
    display_parts = stem.split(".")
    while len(display_parts) > 1 and display_parts[0].isdigit():
        display_parts.pop(0)
    display_name = ".".join(display_parts).replace("_", " ")

    lowered = filename.lower()
    for keywords, details in SCRIPT_DETAILS:
        if any(keyword in lowered for keyword in keywords):
            return display_name, details[0], details[1], details[2]
    return display_name, "任务脚本", "执行已配置的 Shell 任务流程", COLORS["blue"]


def configured_pose_topic():
    return os.environ.get("WINDBLADE_POSE_TOPIC", DEFAULT_POSE_TOPIC).strip() or DEFAULT_POSE_TOPIC


def quaternion_to_euler_degrees(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        return None

    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class ScriptCard(tk.Canvas):
    """脚本启动卡片，整张卡片都可点击。"""

    def __init__(self, parent, index, filename, command):
        super().__init__(
            parent,
            height=112,
            bg=COLORS["panel"],
            highlightthickness=0,
            borderwidth=0,
            cursor="hand2",
        )
        self.index = index
        self.filename = filename
        self.command = command
        self.hovered = False
        self.title, self.category, self.description, self.accent = script_metadata(filename)

        self.bind("<Configure>", self._draw)
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_click)

    def _draw(self, _event=None):
        self.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        if width < 80 or height < 40:
            return

        fill = COLORS["card_hover"] if self.hovered else COLORS["card"]
        outline = self.accent if self.hovered else COLORS["border"]
        rounded_rectangle(
            self,
            2,
            2,
            width - 2,
            height - 2,
            radius=15,
            fill=fill,
            outline=outline,
            width=1,
        )
        self.create_rectangle(3, 23, 7, height - 23, fill=self.accent, outline="")

        self.create_oval(20, 21, 54, 55, fill=COLORS["panel_alt"], outline=self.accent, width=1)
        self.create_text(
            37,
            38,
            text=f"{self.index:02d}",
            fill=self.accent,
            font=(FONT_MONO, 9, "bold"),
        )
        self.create_text(
            68,
            19,
            text=self.category.upper(),
            anchor="nw",
            fill=self.accent,
            font=(FONT_FAMILY, 8, "bold"),
        )
        self.create_text(
            68,
            41,
            text=self.title,
            anchor="nw",
            fill=COLORS["text"],
            font=(FONT_FAMILY, 12, "bold"),
            width=max(120, width - 170),
        )
        self.create_text(
            68,
            74,
            text=self.description,
            anchor="nw",
            fill=COLORS["muted"],
            font=(FONT_FAMILY, 9),
            width=max(140, width - 105),
        )
        self.create_text(
            width - 22,
            40,
            text="启动  ›",
            anchor="e",
            fill=self.accent if self.hovered else COLORS["muted"],
            font=(FONT_FAMILY, 9, "bold"),
        )

    def _on_enter(self, _event):
        self.hovered = True
        self._draw()

    def _on_leave(self, _event):
        self.hovered = False
        self._draw()

    def _on_click(self, _event):
        if self.command:
            self.command()


class WindbladeLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("WINDBLADE · 风机巡检任务控制台")
        self.geometry("1360x820")
        self.minsize(1080, 680)
        self.configure(bg=COLORS["background"])
        self.option_add("*Font", (FONT_FAMILY, 10))

        # 保存每次点击脚本后的启动记录。注意：这里不能在状态刷新时移除已退出的
        # Popen 对象，因为部分 .sh 会拉起 ROS/终端进程后自己退出；重启时仍要
        # 按启动时记录的 PID/PGID 尝试清理它们留下的子进程组。
        self.running_processes = []
        self.all_scripts = []
        self.current_columns = 0
        self._layout_after_id = None
        self.pose_topic = configured_pose_topic()
        self.pose_queue = queue.Queue()
        self.pose_monitor_stop = threading.Event()
        self.pose_monitor_thread = None
        self.pose_process = None
        self.pose_last_update = None
        self.pose_values = {}

        self._set_window_icon()
        self._build_layout()
        self._configure_shortcuts()
        self.protocol("WM_DELETE_WINDOW", self.safe_exit)

        self.log("风机巡检任务控制台已就绪", "success")
        self.load_buttons()
        self._update_clock()
        self._update_process_status()
        self._start_pose_monitor()
        self._poll_pose_queue()

    # ---------- 界面构建 ----------
    def _set_window_icon(self):
        candidates = (
            os.path.join(APP_FOLDER, "unnamed_safe.png"),
            "/home/byz/启动文件/unnamed_safe.png",
        )
        for icon_path in candidates:
            if os.path.exists(icon_path):
                try:
                    self._icon_image = tk.PhotoImage(file=icon_path)
                    self.iconphoto(False, self._icon_image)
                    return
                except tk.TclError:
                    pass

    def _build_layout(self):
        self.sidebar = tk.Frame(self, bg=COLORS["sidebar"], width=244)
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)
        self._build_sidebar()

        self.workspace = tk.Frame(self, bg=COLORS["background"])
        self.workspace.pack(side="left", fill="both", expand=True)
        self._build_header()
        self._build_content()
        self._build_status_bar()

    def _build_sidebar(self):
        brand = tk.Frame(self.sidebar, bg=COLORS["sidebar"])
        brand.pack(fill="x", padx=24, pady=(28, 20))

        logo = tk.Canvas(brand, width=44, height=44, bg=COLORS["sidebar"], highlightthickness=0)
        logo.pack(side="left", padx=(0, 12))
        logo.create_oval(2, 2, 42, 42, fill=COLORS["accent_dark"], outline=COLORS["accent"], width=1)
        logo.create_text(22, 22, text="WB", fill=COLORS["accent"], font=(FONT_MONO, 10, "bold"))

        brand_text = tk.Frame(brand, bg=COLORS["sidebar"])
        brand_text.pack(side="left", fill="x")
        tk.Label(
            brand_text,
            text="WINDBLADE",
            bg=COLORS["sidebar"],
            fg=COLORS["text"],
            font=(FONT_FAMILY, 13, "bold"),
        ).pack(anchor="w")
        tk.Label(
            brand_text,
            text="MISSION CONTROL",
            bg=COLORS["sidebar"],
            fg=COLORS["muted"],
            font=(FONT_MONO, 7),
        ).pack(anchor="w")

        tk.Frame(self.sidebar, bg=COLORS["border_soft"], height=1).pack(fill="x", padx=20)

        status_box = tk.Frame(self.sidebar, bg=COLORS["panel"], padx=16, pady=14)
        status_box.pack(fill="x", padx=18, pady=(20, 12))
        tk.Label(
            status_box,
            text="●  系统就绪",
            bg=COLORS["panel"],
            fg=COLORS["accent"],
            font=(FONT_FAMILY, 10, "bold"),
        ).pack(anchor="w")
        self.clock_label = tk.Label(
            status_box,
            text="--:--:--",
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=(FONT_MONO, 18, "bold"),
        )
        self.clock_label.pack(anchor="w", pady=(8, 0))
        self.date_label = tk.Label(
            status_box,
            text="",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 8),
        )
        self.date_label.pack(anchor="w")

        tk.Label(
            self.sidebar,
            text="任务概览",
            bg=COLORS["sidebar"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 9, "bold"),
        ).pack(anchor="w", padx=24, pady=(10, 8))

        stats = tk.Frame(self.sidebar, bg=COLORS["sidebar"])
        stats.pack(fill="x", padx=18)
        stats.grid_columnconfigure((0, 1), weight=1)
        self.script_stat_value = self._stat_card(stats, 0, "可用脚本", "0", COLORS["blue"])
        self.process_stat_value = self._stat_card(stats, 1, "运行任务", "0", COLORS["accent"])

        tk.Label(
            self.sidebar,
            text="脚本目录",
            bg=COLORS["sidebar"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 9, "bold"),
        ).pack(anchor="w", padx=24, pady=(24, 7))
        tk.Label(
            self.sidebar,
            text=SCRIPTS_FOLDER,
            bg=COLORS["sidebar"],
            fg=COLORS["dim"],
            justify="left",
            wraplength=194,
            font=(FONT_MONO, 7),
        ).pack(anchor="w", padx=24)

        shortcuts = tk.Frame(self.sidebar, bg=COLORS["sidebar"])
        shortcuts.pack(side="bottom", fill="x", padx=24, pady=22)
        tk.Label(
            shortcuts,
            text="快捷键",
            bg=COLORS["sidebar"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 9, "bold"),
        ).pack(anchor="w", pady=(0, 8))
        for key, action in (("Ctrl F", "搜索"), ("F5", "刷新"), ("Ctrl L", "清空日志")):
            row = tk.Frame(shortcuts, bg=COLORS["sidebar"])
            row.pack(fill="x", pady=2)
            tk.Label(
                row,
                text=key,
                bg=COLORS["panel_alt"],
                fg=COLORS["muted"],
                padx=6,
                pady=2,
                font=(FONT_MONO, 7),
            ).pack(side="left")
            tk.Label(
                row,
                text=action,
                bg=COLORS["sidebar"],
                fg=COLORS["dim"],
                font=(FONT_FAMILY, 8),
            ).pack(side="right")

    def _stat_card(self, parent, column, label, value, color):
        card = tk.Frame(parent, bg=COLORS["panel"], padx=12, pady=12)
        card.grid(row=0, column=column, sticky="nsew", padx=(0, 5) if column == 0 else (5, 0))
        value_label = tk.Label(
            card,
            text=value,
            bg=COLORS["panel"],
            fg=color,
            font=(FONT_MONO, 19, "bold"),
        )
        value_label.pack(anchor="w")
        tk.Label(
            card,
            text=label,
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 8),
        ).pack(anchor="w")
        return value_label

    def _build_header(self):
        header = tk.Frame(self.workspace, bg=COLORS["background"], height=122)
        header.pack(fill="x", padx=28, pady=(22, 0))
        header.pack_propagate(False)

        heading = tk.Frame(header, bg=COLORS["background"])
        heading.pack(side="left", fill="y")
        tk.Label(
            heading,
            text="任务启动器",
            bg=COLORS["background"],
            fg=COLORS["text"],
            font=(FONT_FAMILY, 24, "bold"),
        ).pack(anchor="w")
        tk.Label(
            heading,
            text="选择任务流程，启动风机巡检与建图模块",
            bg=COLORS["background"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 10),
        ).pack(anchor="w", pady=(3, 0))

        actions = tk.Frame(header, bg=COLORS["background"])
        actions.pack(side="right", anchor="n", pady=2)
        self._toolbar_button(actions, "刷新", lambda: self.load_buttons(announce=True)).pack(side="left", padx=4)
        self._toolbar_button(actions, "重启", self.restart_app).pack(side="left", padx=4)
        self._toolbar_button(actions, "退出", self.safe_exit, danger=True).pack(side="left", padx=(4, 0))

        search_shell = tk.Frame(header, bg=COLORS["panel_alt"], height=40)
        search_shell.place(x=0, y=76, relwidth=1, width=-2)
        search_shell.pack_propagate(False)
        tk.Label(
            search_shell,
            text="搜索",
            bg=COLORS["panel_alt"],
            fg=COLORS["accent"],
            font=(FONT_FAMILY, 9, "bold"),
        ).pack(side="left", padx=(14, 10))
        tk.Frame(search_shell, bg=COLORS["border"], width=1).pack(side="left", fill="y", pady=9)
        self.search_var = tk.StringVar()
        self.search_var.trace_add("write", self._on_search_changed)
        self.search_entry = tk.Entry(
            search_shell,
            textvariable=self.search_var,
            bg=COLORS["panel_alt"],
            fg=COLORS["text"],
            insertbackground=COLORS["accent"],
            selectbackground=COLORS["blue_dark"],
            relief="flat",
            borderwidth=0,
            font=(FONT_FAMILY, 10),
        )
        self.search_entry.pack(side="left", fill="both", expand=True, padx=12)
        self.search_hint = tk.Label(
            search_shell,
            text="输入脚本名称或任务关键词",
            bg=COLORS["panel_alt"],
            fg=COLORS["dim"],
            font=(FONT_FAMILY, 8),
        )
        self.search_hint.pack(side="right", padx=14)

    def _toolbar_button(self, parent, text, command, danger=False):
        normal_bg = COLORS["danger_dark"] if danger else COLORS["panel_alt"]
        hover_bg = COLORS["danger"] if danger else COLORS["blue_dark"]
        normal_fg = COLORS["danger"] if danger else COLORS["text"]
        hover_fg = COLORS["text"]
        button = tk.Button(
            parent,
            text=text,
            command=command,
            bg=normal_bg,
            fg=normal_fg,
            activebackground=hover_bg,
            activeforeground=hover_fg,
            relief="flat",
            borderwidth=0,
            padx=17,
            pady=8,
            cursor="hand2",
            font=(FONT_FAMILY, 9, "bold"),
        )
        button.bind("<Enter>", lambda _e: button.configure(bg=hover_bg, fg=hover_fg))
        button.bind("<Leave>", lambda _e: button.configure(bg=normal_bg, fg=normal_fg))
        return button

    def _build_content(self):
        content = tk.Frame(self.workspace, bg=COLORS["background"])
        content.pack(fill="both", expand=True, padx=28, pady=(15, 14))

        scripts_panel = tk.Frame(content, bg=COLORS["panel"])
        scripts_panel.pack(side="left", fill="both", expand=True, padx=(0, 14))

        scripts_header = tk.Frame(scripts_panel, bg=COLORS["panel"], height=58)
        scripts_header.pack(fill="x", padx=18, pady=(8, 0))
        scripts_header.pack_propagate(False)
        heading = tk.Frame(scripts_header, bg=COLORS["panel"])
        heading.pack(side="left", anchor="w", pady=8)
        tk.Label(
            heading,
            text="可用任务",
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=(FONT_FAMILY, 12, "bold"),
        ).pack(anchor="w")
        self.script_count_label = tk.Label(
            heading,
            text="正在扫描脚本目录…",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 8),
        )
        self.script_count_label.pack(anchor="w")

        self.canvas = tk.Canvas(
            scripts_panel,
            bg=COLORS["panel"],
            highlightthickness=0,
            borderwidth=0,
        )
        self.scrollbar = tk.Scrollbar(
            scripts_panel,
            orient="vertical",
            command=self.canvas.yview,
            bg=COLORS["panel_alt"],
            troughcolor=COLORS["panel"],
            activebackground=COLORS["accent"],
            relief="flat",
            borderwidth=0,
        )
        self.scroll_content = tk.Frame(self.canvas, bg=COLORS["panel"])
        self.canvas_window = self.canvas.create_window(
            (0, 0), window=self.scroll_content, anchor="nw", tags="content"
        )
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True, padx=(8, 0), pady=(0, 10))
        self.scrollbar.pack(side="right", fill="y", pady=(0, 10))

        self.scroll_content.bind("<Configure>", self._update_scroll_region)
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.canvas.bind("<Enter>", lambda _e: self._set_scroll_active(True))
        self.canvas.bind("<Leave>", lambda _e: self._set_scroll_active(False))

        self._build_log_panel(content)

    def _build_log_panel(self, parent):
        log_panel = tk.Frame(parent, bg=COLORS["panel"], width=372)
        log_panel.pack(side="right", fill="both")
        log_panel.pack_propagate(False)

        self._build_pose_panel(log_panel)

        log_header = tk.Frame(log_panel, bg=COLORS["panel"], height=66)
        log_header.pack(fill="x", padx=17)
        log_header.pack_propagate(False)
        title_box = tk.Frame(log_header, bg=COLORS["panel"])
        title_box.pack(side="left", pady=12)
        tk.Label(
            title_box,
            text="运行日志",
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=(FONT_FAMILY, 12, "bold"),
        ).pack(anchor="w")
        self.log_status_label = tk.Label(
            title_box,
            text="实时记录任务状态",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=(FONT_FAMILY, 8),
        )
        self.log_status_label.pack(anchor="w")
        clear_button = tk.Button(
            log_header,
            text="清空",
            command=self.clear_log,
            bg=COLORS["panel_alt"],
            fg=COLORS["muted"],
            activebackground=COLORS["blue_dark"],
            activeforeground=COLORS["text"],
            relief="flat",
            borderwidth=0,
            padx=11,
            pady=5,
            cursor="hand2",
            font=(FONT_FAMILY, 8),
        )
        clear_button.pack(side="right")

        log_body = tk.Frame(log_panel, bg=COLORS["log"])
        log_body.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        log_scroll = tk.Scrollbar(
            log_body,
            bg=COLORS["panel_alt"],
            troughcolor=COLORS["log"],
            relief="flat",
            borderwidth=0,
        )
        log_scroll.pack(side="right", fill="y")
        self.log_text = tk.Text(
            log_body,
            bg=COLORS["log"],
            fg=COLORS["muted"],
            insertbackground=COLORS["accent"],
            relief="flat",
            borderwidth=0,
            padx=13,
            pady=13,
            wrap="word",
            state="disabled",
            yscrollcommand=log_scroll.set,
            font=(FONT_MONO, 8),
            spacing1=2,
            spacing3=4,
        )
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll.configure(command=self.log_text.yview)

        self.log_text.tag_configure("timestamp", foreground=COLORS["dim"])
        self.log_text.tag_configure("info", foreground=COLORS["blue"])
        self.log_text.tag_configure("success", foreground=COLORS["accent"])
        self.log_text.tag_configure("warning", foreground=COLORS["warning"])
        self.log_text.tag_configure("error", foreground=COLORS["danger"])

    def _build_pose_panel(self, parent):
        pose_panel = tk.Frame(parent, bg=COLORS["panel_alt"], padx=14, pady=12)
        pose_panel.pack(fill="x", padx=12, pady=(12, 4))

        header = tk.Frame(pose_panel, bg=COLORS["panel_alt"])
        header.pack(fill="x")
        tk.Label(
            header,
            text="无人机实时位姿",
            bg=COLORS["panel_alt"],
            fg=COLORS["text"],
            font=(FONT_FAMILY, 11, "bold"),
        ).pack(side="left")
        self.pose_state_label = tk.Label(
            header,
            text="● 等待定位",
            bg=COLORS["panel_alt"],
            fg=COLORS["warning"],
            font=(FONT_FAMILY, 8, "bold"),
        )
        self.pose_state_label.pack(side="right")

        self.pose_topic_label = tk.Label(
            pose_panel,
            text=self.pose_topic,
            bg=COLORS["panel_alt"],
            fg=COLORS["dim"],
            anchor="w",
            justify="left",
            wraplength=320,
            font=(FONT_MONO, 7),
        )
        self.pose_topic_label.pack(fill="x", pady=(5, 8))

        grid = tk.Frame(pose_panel, bg=COLORS["panel_alt"])
        grid.pack(fill="x")
        for column in range(3):
            grid.grid_columnconfigure(column, weight=1, uniform="pose")

        metrics = (
            ("x", "X", "m"),
            ("y", "Y", "m"),
            ("z", "Z", "m"),
            ("roll", "Roll", "°"),
            ("pitch", "Pitch", "°"),
            ("yaw", "Yaw", "°"),
        )
        for index, (key, label, unit) in enumerate(metrics):
            row = index // 3
            column = index % 3
            self.pose_values[key] = self._pose_metric(grid, row, column, label, unit)

        footer = tk.Frame(pose_panel, bg=COLORS["panel_alt"])
        footer.pack(fill="x", pady=(8, 0))
        self.pose_frame_label = tk.Label(
            footer,
            text="Frame: --",
            bg=COLORS["panel_alt"],
            fg=COLORS["dim"],
            font=(FONT_MONO, 7),
        )
        self.pose_frame_label.pack(side="left")
        self.pose_age_label = tk.Label(
            footer,
            text="未收到数据",
            bg=COLORS["panel_alt"],
            fg=COLORS["dim"],
            font=(FONT_FAMILY, 8),
        )
        self.pose_age_label.pack(side="right")

    def _pose_metric(self, parent, row, column, label, unit):
        cell = tk.Frame(parent, bg=COLORS["panel"], padx=8, pady=7)
        cell.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        tk.Label(
            cell,
            text=f"{label} / {unit}",
            bg=COLORS["panel"],
            fg=COLORS["dim"],
            font=(FONT_FAMILY, 7),
        ).pack(anchor="w")
        value_label = tk.Label(
            cell,
            text="--",
            bg=COLORS["panel"],
            fg=COLORS["accent"],
            font=(FONT_MONO, 12, "bold"),
        )
        value_label.pack(anchor="w", pady=(2, 0))
        return value_label

    def _build_status_bar(self):
        bar = tk.Frame(self.workspace, bg=COLORS["sidebar"], height=30)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)
        self.footer_status = tk.Label(
            bar,
            text="●  READY",
            bg=COLORS["sidebar"],
            fg=COLORS["accent"],
            font=(FONT_MONO, 7, "bold"),
        )
        self.footer_status.pack(side="left", padx=28)
        tk.Label(
            bar,
            text="WINDBLADE CONTROL  /  ROS MISSION LAUNCHER",
            bg=COLORS["sidebar"],
            fg=COLORS["dim"],
            font=(FONT_MONO, 7),
        ).pack(side="right", padx=28)

    # ---------- 脚本列表 ----------
    def load_buttons(self, announce=False):
        if not os.path.isdir(SCRIPTS_FOLDER):
            self.all_scripts = []
            self.script_stat_value.configure(text="0")
            self.script_count_label.configure(text="脚本目录不可用")
            self._render_empty_state("无法找到脚本目录", SCRIPTS_FOLDER, error=True)
            self.log(f"脚本目录不存在：{SCRIPTS_FOLDER}", "error")
            return

        try:
            self.all_scripts = sorted(
                filename
                for filename in os.listdir(SCRIPTS_FOLDER)
                if filename.lower().endswith(".sh")
            )
        except OSError as exc:
            self.all_scripts = []
            self._render_empty_state("无法读取脚本目录", str(exc), error=True)
            self.log(f"读取脚本目录失败：{exc}", "error")
            return

        self.script_stat_value.configure(text=str(len(self.all_scripts)))
        self.search_var.set("")
        self._render_scripts(self.all_scripts)
        if announce:
            self.log(f"脚本列表已刷新，共发现 {len(self.all_scripts)} 个任务", "info")

    def filter_buttons(self):
        query = self.search_var.get().strip().lower()
        if not query:
            filtered = self.all_scripts
        else:
            filtered = []
            for filename in self.all_scripts:
                title, category, description, _accent = script_metadata(filename)
                haystack = " ".join((filename, title, category, description)).lower()
                if query in haystack:
                    filtered.append(filename)
        self._render_scripts(filtered, query=query)

    def _render_scripts(self, files, query=""):
        for widget in self.scroll_content.winfo_children():
            widget.destroy()

        if not files:
            if query:
                self.script_count_label.configure(text="没有匹配的任务")
                self._render_empty_state("未找到匹配项", f"换一个关键词试试：{query}")
            else:
                self.script_count_label.configure(text="目录中没有 Shell 脚本")
                self._render_empty_state("暂无可用任务", "请将 .sh 文件放入脚本目录")
            return

        total = len(self.all_scripts)
        if query:
            self.script_count_label.configure(text=f"找到 {len(files)} / {total} 个任务")
        else:
            self.script_count_label.configure(text=f"共 {total} 个任务 · 点击卡片即可启动")

        columns = self._desired_columns()
        self.current_columns = columns
        for column in range(columns):
            self.scroll_content.grid_columnconfigure(column, weight=1, uniform="script")

        for index, filename in enumerate(files, start=1):
            row = (index - 1) // columns
            column = (index - 1) % columns
            card = ScriptCard(
                self.scroll_content,
                index=index,
                filename=filename,
                command=lambda selected=filename: self.run_script(selected),
            )
            card.grid(
                row=row,
                column=column,
                sticky="nsew",
                padx=(10, 6) if column == 0 else (6, 10),
                pady=6,
            )
        self.after_idle(self._update_scroll_region)

    def _render_empty_state(self, title, detail, error=False):
        for widget in self.scroll_content.winfo_children():
            widget.destroy()
        box = tk.Frame(self.scroll_content, bg=COLORS["panel"])
        box.pack(fill="both", expand=True, pady=100)
        color = COLORS["danger"] if error else COLORS["muted"]
        tk.Label(
            box,
            text="!" if error else "—",
            bg=COLORS["panel_alt"],
            fg=color,
            width=3,
            pady=8,
            font=(FONT_MONO, 14, "bold"),
        ).pack()
        tk.Label(
            box,
            text=title,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=(FONT_FAMILY, 12, "bold"),
        ).pack(pady=(14, 4))
        tk.Label(
            box,
            text=detail,
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            wraplength=420,
            justify="center",
            font=(FONT_FAMILY, 9),
        ).pack()

    def _desired_columns(self):
        return 2 if self.canvas.winfo_width() >= 560 else 1

    def _on_canvas_resize(self, event):
        self.canvas.itemconfigure(self.canvas_window, width=max(1, event.width - 2))
        desired = 2 if event.width >= 560 else 1
        if self.current_columns and desired != self.current_columns:
            if self._layout_after_id:
                self.after_cancel(self._layout_after_id)
            self._layout_after_id = self.after(100, self.filter_buttons)

    def _update_scroll_region(self, _event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _set_scroll_active(self, active):
        if active:
            self.bind_all("<MouseWheel>", self._on_mousewheel)
            self.bind_all("<Button-4>", self._on_mousewheel)
            self.bind_all("<Button-5>", self._on_mousewheel)
        else:
            self.unbind_all("<MouseWheel>")
            self.unbind_all("<Button-4>")
            self.unbind_all("<Button-5>")

    def _on_mousewheel(self, event):
        if getattr(event, "num", None) == 5 or getattr(event, "delta", 0) < 0:
            self.canvas.yview_scroll(2, "units")
        elif getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0:
            self.canvas.yview_scroll(-2, "units")

    def _on_search_changed(self, *_args):
        if not hasattr(self, "scroll_content"):
            return
        query = self.search_var.get().strip()
        self.search_hint.configure(text="ESC 清除" if query else "输入脚本名称或任务关键词")
        self.filter_buttons()

    # ---------- 日志与状态 ----------
    def log(self, message, level="info"):
        timestamp = time.strftime("%H:%M:%S")
        level = level if level in {"info", "success", "warning", "error"} else "info"
        print(f"[{timestamp}] {message}", flush=True)
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{timestamp}  ", "timestamp")
        self.log_text.insert("end", f"{message}\n", level)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log("日志已清空", "info")

    # ---------- 实时位姿监控 ----------
    def _start_pose_monitor(self):
        if self.pose_monitor_thread and self.pose_monitor_thread.is_alive():
            return
        self.pose_monitor_stop.clear()
        self.pose_monitor_thread = threading.Thread(
            target=self._pose_monitor_loop,
            name="windblade-pose-monitor",
            daemon=True,
        )
        self.pose_monitor_thread.start()

    def _stop_pose_monitor(self):
        stop_event = getattr(self, "pose_monitor_stop", None)
        if stop_event:
            stop_event.set()

        self._terminate_pose_process(signal.SIGTERM)
        process = getattr(self, "pose_process", None)
        if process and process.poll() is None:
            try:
                process.wait(timeout=0.8)
            except subprocess.TimeoutExpired:
                self._terminate_pose_process(signal.SIGKILL)

    def _terminate_pose_process(self, sig):
        process = getattr(self, "pose_process", None)
        if not process or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                if sig == signal.SIGKILL:
                    process.kill()
                else:
                    process.terminate()
            except OSError:
                pass

    def _pose_monitor_loop(self):
        command = ["rostopic", "echo", "-p", self.pose_topic]
        while not self.pose_monitor_stop.is_set():
            header = None
            self.pose_queue.put(
                {
                    "type": "status",
                    "message": f"等待定位话题：{self.pose_topic}",
                    "color": COLORS["warning"],
                }
            )
            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
            except FileNotFoundError:
                self.pose_queue.put(
                    {
                        "type": "status",
                        "message": "未找到 rostopic，请先 source ROS 环境",
                        "color": COLORS["danger"],
                    }
                )
                self.pose_monitor_stop.wait(3.0)
                continue
            except OSError as exc:
                self.pose_queue.put(
                    {
                        "type": "status",
                        "message": f"位姿监听启动失败：{exc}",
                        "color": COLORS["danger"],
                    }
                )
                self.pose_monitor_stop.wait(3.0)
                continue

            self.pose_process = process
            try:
                for raw_line in process.stdout:
                    if self.pose_monitor_stop.is_set():
                        break
                    line = raw_line.strip()
                    if not line:
                        continue
                    if line.startswith("%time"):
                        header = self._csv_fields(line)
                        self.pose_queue.put(
                            {
                                "type": "status",
                                "message": "已连接定位数据",
                                "color": COLORS["accent"],
                            }
                        )
                        continue
                    if header is None:
                        if "ERROR" in line or "WARNING" in line or "unable" in line.lower():
                            self.pose_queue.put(
                                {
                                    "type": "status",
                                    "message": line[:90],
                                    "color": COLORS["warning"],
                                }
                            )
                        continue

                    values = self._csv_fields(line)
                    pose = self._parse_pose_row(header, values)
                    if pose:
                        self.pose_queue.put({"type": "pose", "pose": pose})
            finally:
                if process.poll() is None:
                    self._terminate_pose_process(signal.SIGTERM)
                if self.pose_process is process:
                    self.pose_process = None

            if not self.pose_monitor_stop.is_set():
                self.pose_queue.put(
                    {
                        "type": "status",
                        "message": "定位数据断开，正在重连…",
                        "color": COLORS["warning"],
                    }
                )
                self.pose_monitor_stop.wait(2.0)

    def _csv_fields(self, line):
        try:
            return next(csv.reader([line]))
        except (csv.Error, StopIteration):
            return []

    def _parse_pose_row(self, header, values):
        if len(values) < len(header):
            return None
        data = dict(zip(header, values))

        def clean_text(value):
            return (value or "").strip().strip('"')

        def get_float(*names):
            for name in names:
                value = data.get(name)
                if value not in (None, ""):
                    return float(value)
            raise KeyError(names[0] if names else "")

        try:
            x = get_float("field.pose.pose.position.x", "field.pose.position.x")
            y = get_float("field.pose.pose.position.y", "field.pose.position.y")
            z = get_float("field.pose.pose.position.z", "field.pose.position.z")
            qx = get_float("field.pose.pose.orientation.x", "field.pose.orientation.x")
            qy = get_float("field.pose.pose.orientation.y", "field.pose.orientation.y")
            qz = get_float("field.pose.pose.orientation.z", "field.pose.orientation.z")
            qw = get_float("field.pose.pose.orientation.w", "field.pose.orientation.w")
        except (KeyError, ValueError):
            return None

        euler = quaternion_to_euler_degrees(qx, qy, qz, qw)
        if euler is None:
            roll = pitch = yaw = None
        else:
            roll, pitch, yaw = euler

        return {
            "x": x,
            "y": y,
            "z": z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "frame": clean_text(data.get("field.header.frame_id")) or "--",
            "child_frame": clean_text(data.get("field.child_frame_id")) or "--",
            "received_at": time.time(),
        }

    def _poll_pose_queue(self):
        try:
            while True:
                item = self.pose_queue.get_nowait()
                if item.get("type") == "pose":
                    self._update_pose_display(item["pose"])
                elif item.get("type") == "status":
                    self._set_pose_status(item.get("message", "等待定位数据"), item.get("color", COLORS["warning"]))
        except queue.Empty:
            pass

        self._update_pose_age()
        self.after(250, self._poll_pose_queue)

    def _set_pose_status(self, message, color):
        if not hasattr(self, "pose_state_label"):
            return
        self.pose_state_label.configure(text=f"● {message}", fg=color)

    def _update_pose_display(self, pose):
        self.pose_last_update = pose.get("received_at", time.time())

        for key in ("x", "y", "z"):
            self.pose_values[key].configure(text=f"{pose[key]:.3f}")
        for key in ("roll", "pitch", "yaw"):
            value = pose.get(key)
            self.pose_values[key].configure(text="--" if value is None else f"{value:.1f}")

        self.pose_state_label.configure(text="● LIVE", fg=COLORS["accent"])
        self.pose_frame_label.configure(text=f"Frame: {pose.get('frame', '--')} → {pose.get('child_frame', '--')}")
        self.pose_age_label.configure(text="刚刚更新", fg=COLORS["accent"])

    def _update_pose_age(self):
        if not self.pose_last_update or not hasattr(self, "pose_age_label"):
            return
        age = max(0.0, time.time() - self.pose_last_update)
        if age < 1.0:
            text = "刚刚更新"
            color = COLORS["accent"]
        elif age < 5.0:
            text = f"{age:.1f}s 前"
            color = COLORS["accent"]
        elif age < 12.0:
            text = f"{age:.0f}s 未更新"
            color = COLORS["warning"]
            self.pose_state_label.configure(text="● 数据延迟", fg=COLORS["warning"])
        else:
            text = f"{age:.0f}s 未更新"
            color = COLORS["danger"]
            self.pose_state_label.configure(text="● 等待更新", fg=COLORS["danger"])
        self.pose_age_label.configure(text=text, fg=color)

    def _update_clock(self):
        now = time.localtime()
        self.clock_label.configure(text=time.strftime("%H:%M:%S", now))
        self.date_label.configure(text=time.strftime("%Y年%m月%d日  %A", now))
        self.after(1000, self._update_clock)

    def _pid_exists(self, pid):
        if not pid or pid == os.getpid():
            return False
        try:
            os.kill(pid, 0)
            return True
        except (ProcessLookupError, PermissionError, OSError):
            return False

    def _process_name(self, pid):
        try:
            with open(f"/proc/{pid}/comm", "r", encoding="utf-8", errors="ignore") as handle:
                return handle.read().strip()
        except OSError:
            return ""

    def _should_signal_pid(self, pid):
        name = self._process_name(pid)
        # gnome-terminal 的窗口通常由单独的 server 进程托管。不要直接杀 server，
        # 否则可能误关用户在本界面之外打开的终端；杀掉终端内带 token 的 bash/
        # roslaunch/python 后，窗口会随着前台进程组退出而关闭。
        if name in TERMINAL_SERVER_NAMES:
            return False
        return True

    def _read_ppid(self, pid):
        try:
            with open(f"/proc/{pid}/stat", "r", encoding="utf-8", errors="ignore") as handle:
                stat = handle.read()
            end = stat.rfind(")")
            if end == -1:
                return None
            fields = stat[end + 2 :].split()
            return int(fields[1]) if len(fields) > 1 else None
        except (OSError, ValueError):
            return None

    def _descendant_pids(self, root_pid):
        if not root_pid:
            return set()

        children_by_parent = {}
        try:
            proc_entries = [entry for entry in os.listdir("/proc") if entry.isdigit()]
        except OSError:
            return set()

        for entry in proc_entries:
            pid = int(entry)
            ppid = self._read_ppid(pid)
            if ppid is not None:
                children_by_parent.setdefault(ppid, set()).add(pid)

        descendants = set()
        stack = list(children_by_parent.get(root_pid, ()))
        while stack:
            pid = stack.pop()
            if pid in descendants:
                continue
            descendants.add(pid)
            stack.extend(children_by_parent.get(pid, ()))
        return descendants

    def _find_pids_by_env(self, env_name, env_value):
        if not env_value:
            return set()

        needle = f"{env_name}={env_value}".encode("utf-8")
        matches = set()
        try:
            proc_entries = [entry for entry in os.listdir("/proc") if entry.isdigit()]
        except OSError:
            return matches

        own_pid = os.getpid()
        for entry in proc_entries:
            pid = int(entry)
            if pid == own_pid:
                continue
            try:
                with open(f"/proc/{pid}/environ", "rb") as handle:
                    environ = handle.read()
            except OSError:
                continue
            if needle in environ:
                matches.add(pid)
        return matches

    def _refresh_process_record(self, record, announce=False):
        if not record:
            return set()

        pids = set()
        pid = record.get("pid")
        if self._pid_exists(pid):
            pids.add(pid)
        pids.update(self._descendant_pids(pid))

        launch_id = record.get("launch_id")
        pids.update(self._find_pids_by_env(WINDBLADE_LAUNCH_ID_ENV, launch_id))
        pids = {candidate for candidate in pids if self._pid_exists(candidate)}

        tracked_pids = record.setdefault("tracked_pids", set())
        tracked_pgids = record.setdefault("tracked_pgids", set())
        new_pids = pids - tracked_pids
        tracked_pids.update(pids)

        for candidate in pids:
            try:
                pgid = os.getpgid(candidate)
            except (ProcessLookupError, PermissionError, OSError):
                continue
            if pgid and pgid != os.getpgrp():
                tracked_pgids.add(pgid)

        if announce and new_pids:
            preview = ", ".join(str(item) for item in sorted(new_pids)[:8])
            suffix = "…" if len(new_pids) > 8 else ""
            self.log(f"已登记终端/子进程：{record.get('filename', '未知脚本')}  ·  {preview}{suffix}", "info")
        return pids

    def _refresh_process_record_by_launch_id(self, launch_id, announce=False):
        for record in self.running_processes:
            if record.get("launch_id") == launch_id:
                self._refresh_process_record(record, announce=announce)
                self.process_stat_value.configure(text=str(self._active_processes()))
                return

    def _is_process_record_active(self, record):
        if self._refresh_process_record(record):
            return True

        process = record.get("process")
        if process and process.poll() is None:
            return True

        pid = record.get("pid")
        if pid:
            try:
                os.kill(pid, 0)
                return True
            except (ProcessLookupError, PermissionError, OSError):
                pass

        return False

    def _active_processes(self):
        return sum(1 for record in self.running_processes if self._is_process_record_active(record))

    def _update_process_status(self):
        active = self._active_processes()
        self.process_stat_value.configure(text=str(active))
        if active:
            self.log_status_label.configure(text=f"{active} 个启动进程正在运行", fg=COLORS["accent"])
            self.footer_status.configure(text=f"●  RUNNING  /  {active} PROCESS", fg=COLORS["accent"])
        else:
            self.log_status_label.configure(text="实时记录任务状态", fg=COLORS["muted"])
            self.footer_status.configure(text="●  READY", fg=COLORS["accent"])
        self.after(1200, self._update_process_status)

    # ---------- 任务与窗口操作 ----------
    def run_script(self, filename):
        path = os.path.join(SCRIPTS_FOLDER, filename)
        if not os.path.isfile(path):
            self.log(f"脚本不存在：{filename}", "error")
            return

        self.log(f"正在启动：{filename}", "info")
        try:
            os.chmod(path, 0o755)
            launch_id = f"windblade-{os.getpid()}-{int(time.time() * 1000)}-{len(self.running_processes) + 1}"
            launch_env = os.environ.copy()
            launch_env[WINDBLADE_LAUNCH_ID_ENV] = launch_id
            launch_env[WINDBLADE_SCRIPT_NAME_ENV] = filename
            launch_env[WINDBLADE_PARENT_PID_ENV] = str(os.getpid())
            process = subprocess.Popen(
                [path],
                cwd=SCRIPTS_FOLDER,
                start_new_session=True,
                env=launch_env,
            )
            try:
                pgid = os.getpgid(process.pid)
            except (ProcessLookupError, PermissionError, OSError):
                pgid = None

            record = {
                "filename": filename,
                "pid": process.pid,
                "pgid": pgid,
                "process": process,
                "started_at": time.strftime("%H:%M:%S"),
                "launch_id": launch_id,
                "tracked_pids": {process.pid},
                "tracked_pgids": {pgid} if pgid else set(),
            }
            self.running_processes.append(record)
            self.after(800, lambda launch_id=launch_id: self._refresh_process_record_by_launch_id(launch_id, announce=True))
            self.after(2200, lambda launch_id=launch_id: self._refresh_process_record_by_launch_id(launch_id, announce=True))
            self.after(5000, lambda launch_id=launch_id: self._refresh_process_record_by_launch_id(launch_id, announce=False))
            self.process_stat_value.configure(text=str(self._active_processes()))
            if pgid:
                self.log(f"启动成功：{filename}  ·  PID {process.pid}  ·  PGID {pgid}  ·  已启用终端追踪", "success")
            else:
                self.log(f"启动成功：{filename}  ·  PID {process.pid}  ·  已启用终端追踪", "success")
        except Exception as exc:
            self.log(f"启动失败：{filename}  ·  {exc}", "error")

    def restart_app(self):
        if not messagebox.askyesno("重启控制台", "将停止当前启动的任务并重新打开控制台，是否继续？"):
            return
        self._cleanup_processes()
        python = sys.executable
        args = [python] + sys.argv
        self.destroy()
        os.execv(python, args)

    def safe_exit(self):
        active = self._active_processes()
        prompt = "退出控制台？"
        if active:
            prompt = f"当前有 {active} 个启动进程，退出将尝试停止这些任务。是否继续？"
        if not messagebox.askyesno("退出 WINDBLADE", prompt):
            return
        self._cleanup_processes()
        self.destroy()

    def destroy(self):
        self._stop_pose_monitor()
        super().destroy()

    def _cleanup_processes(self):
        records = list(self.running_processes)
        if records:
            self.log(f"正在停止 {len(records)} 条脚本启动记录…", "warning")

        for record in records:
            self._refresh_process_record(record, announce=True)
            filename = record.get("filename", "未知脚本")
            pid = record.get("pid")
            signal_count = self._send_signal_to_process_record(record, signal.SIGTERM)
            if signal_count:
                self.log(f"已发送停止信号：{filename}  ·  PID {pid}  ·  目标 {signal_count} 个", "warning")

        if records:
            self.update_idletasks()
            time.sleep(0.6)
            for record in records:
                remaining = self._refresh_process_record(record)
                if not remaining:
                    continue
                kill_count = self._send_signal_to_process_record(record, signal.SIGKILL)
                if kill_count:
                    self.log(
                        f"强制关闭残留进程：{record.get('filename', '未知脚本')}  ·  目标 {kill_count} 个",
                        "warning",
                    )
            self.update_idletasks()
        self.running_processes.clear()

    def _send_signal_to_process_record(self, record, sig):
        self._refresh_process_record(record)

        target_pids = {
            pid
            for pid in record.get("tracked_pids", set())
            if self._pid_exists(pid) and self._should_signal_pid(pid)
        }
        pid = record.get("pid")
        if self._pid_exists(pid) and self._should_signal_pid(pid):
            target_pids.add(pid)

        target_pgids = set()
        pgid = record.get("pgid")
        if pgid and self._pid_exists(record.get("pid")):
            try:
                if os.getpgid(record.get("pid")) == pgid:
                    target_pgids.add(pgid)
            except (ProcessLookupError, PermissionError, OSError):
                pass

        own_pgid = os.getpgrp()
        for target_pid in list(target_pids):
            try:
                target_pgid = os.getpgid(target_pid)
            except (ProcessLookupError, PermissionError, OSError):
                continue
            if target_pgid and target_pgid != own_pgid:
                target_pgids.add(target_pgid)

        signaled = 0
        for target_pgid in sorted(target_pgids):
            if not target_pgid or target_pgid == own_pgid:
                continue
            try:
                os.killpg(target_pgid, sig)
                signaled += 1
            except (ProcessLookupError, PermissionError, OSError):
                pass

        for target_pid in sorted(target_pids):
            try:
                target_pgid = os.getpgid(target_pid)
            except (ProcessLookupError, PermissionError, OSError):
                target_pgid = None
            if target_pgid in target_pgids:
                continue
            try:
                os.kill(target_pid, sig)
                signaled += 1
            except (ProcessLookupError, PermissionError, OSError):
                pass

        return signaled

    def _configure_shortcuts(self):
        self.bind("<F5>", lambda _event: self.load_buttons(announce=True))
        self.bind("<Control-f>", lambda _event: self.search_entry.focus_set())
        self.bind("<Control-l>", lambda _event: self.clear_log())
        self.bind("<Escape>", lambda _event: self.search_var.set(""))


# 保留旧类名，兼容可能引用 TitanLauncher 的启动方式。
TitanLauncher = WindbladeLauncher


if __name__ == "__main__":
    app = WindbladeLauncher()
    app.mainloop()
