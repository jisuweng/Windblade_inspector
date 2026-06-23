import os
import signal
import subprocess
import sys
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

SCRIPT_DETAILS = (
    (("清理", "cleanup"), ("维护工具", "清理 Gazebo 与 ROS 残留进程", COLORS["danger"])),
    (("点云匹配", "mapping"), ("建图任务", "启动点云匹配与风机建图", COLORS["blue"])),
    (("slam", "fastlio"), ("定位建图", "启动 FAST-LIO 定位与建图", COLORS["blue"])),
    (("真值", "mavros", "ground"), ("位姿链路", "连接 MAVROS 并发布位姿真值", COLORS["accent"])),
    (("仿真", "gazebo", "simulation"), ("仿真环境", "启动风机巡检仿真环境", COLORS["warning"])),
    (("keyboard", "起飞", "takeoff"), ("飞行控制", "启动键盘控制与起飞流程", COLORS["accent"])),
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
    display_name = stem.replace("_", " ")
    if "." in display_name and display_name.split(".", 1)[0].isdigit():
        display_name = display_name.split(".", 1)[1]

    lowered = filename.lower()
    for keywords, details in SCRIPT_DETAILS:
        if any(keyword in lowered for keyword in keywords):
            return display_name, details[0], details[1], details[2]
    return display_name, "任务脚本", "执行已配置的 Shell 任务流程", COLORS["blue"]


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

        self._set_window_icon()
        self._build_layout()
        self._configure_shortcuts()
        self.protocol("WM_DELETE_WINDOW", self.safe_exit)

        self.log("风机巡检任务控制台已就绪", "success")
        self.load_buttons()
        self._update_clock()
        self._update_process_status()

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

    def _update_clock(self):
        now = time.localtime()
        self.clock_label.configure(text=time.strftime("%H:%M:%S", now))
        self.date_label.configure(text=time.strftime("%Y年%m月%d日  %A", now))
        self.after(1000, self._update_clock)

    def _is_process_record_active(self, record):
        process = record.get("process")
        if process and process.poll() is None:
            return True

        pgid = record.get("pgid")
        if pgid:
            try:
                os.killpg(pgid, 0)
                return True
            except (ProcessLookupError, PermissionError, OSError):
                pass

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
            process = subprocess.Popen(
                [path],
                cwd=SCRIPTS_FOLDER,
                start_new_session=True,
            )
            try:
                pgid = os.getpgid(process.pid)
            except (ProcessLookupError, PermissionError, OSError):
                pgid = None

            self.running_processes.append(
                {
                    "filename": filename,
                    "pid": process.pid,
                    "pgid": pgid,
                    "process": process,
                    "started_at": time.strftime("%H:%M:%S"),
                }
            )
            self.process_stat_value.configure(text=str(self._active_processes()))
            if pgid:
                self.log(f"启动成功：{filename}  ·  PID {process.pid}  ·  PGID {pgid}", "success")
            else:
                self.log(f"启动成功：{filename}  ·  PID {process.pid}", "success")
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

    def _cleanup_processes(self):
        records = list(self.running_processes)
        if records:
            self.log(f"正在停止 {len(records)} 条脚本启动记录…", "warning")

        for record in records:
            filename = record.get("filename", "未知脚本")
            pid = record.get("pid")
            if self._send_signal_to_process_record(record, signal.SIGTERM):
                self.log(f"已发送停止信号：{filename}  ·  PID {pid}", "warning")

        if records:
            self.after(150, self.update_idletasks)
        self.running_processes.clear()

    def _send_signal_to_process_record(self, record, sig):
        pgid = record.get("pgid")
        if pgid:
            try:
                os.killpg(pgid, sig)
                return True
            except (ProcessLookupError, PermissionError, OSError):
                pass

        pid = record.get("pid")
        if pid:
            try:
                os.kill(pid, sig)
                return True
            except (ProcessLookupError, PermissionError, OSError):
                pass

        return False

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
