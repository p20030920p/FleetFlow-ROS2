"""车间生产调度看板（Andon / MES board）：把车队与产线状态渲染成一张生产看板。

设计语言
--------
参照国内棉纺厂车间墙上挂的**生产调度看板 / Andon 板**，而不是深色科幻 HUD：

* **纸白底 + 细墨线分格**：地面色 `#f4f2ed`，面板用 1px 细线分隔，直角、不发光；
* **中文为主、英文小字辅助**（`梳棉车间` / `CARDING`），一眼就是国内厂里的系统；
* **数字一律等宽字体**并按列对齐，像报表而不是海报；
* **颜色只用于语义**：机台绿 / 警告琥珀 / 报警红 / 钢蓝 / 工业藏青，绝不做装饰渐变；
* 版面顺序照抄车间看板：班次抬头 → 关键指标带 → 物料流转条 → 平面图 + 设备/机台 →
  车辆台账 + 任务节拍 → 页脚数据源。

运行方式
--------
无头运行（Agg 后端），三种出图路径都保留：

* `every_s > 0`：每 N 秒把当前态势写成 `out_dir/control_center_XXXX.png`；
* `once:=true`：等待 6 秒收数据后出一次图并退出；
* `--demo`：用一组可信的合成数据直接出一张 PNG（离线评审设计用，不启动 ROS 图）。

命令行新增参数只有 `--demo` / `--out` / `--size`，ROS 参数（`out_dir` / `every_s` /
`once` / `num_robots`）与原来完全一致。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.patheffects as path_effects
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon, Rectangle

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fleetflow_interfaces.msg import MachineState, RobotStatus, TransportTask

from . import layout

# ---------------------------------------------------------------- 版面尺寸
FIG_W_IN, FIG_H_IN = 16.0, 10.0        # 1600 × 1000 px @ 100 dpi
DPI = 100

# ---------------------------------------------------------------- 油墨配色
PAPER = "#f4f2ed"        # 看板底色：工艺卡纸
PANEL = "#fbfaf7"        # 面板纸
INK = "#1c1c1a"          # 正文墨色
NAVY = "#1f3a5f"         # 工业藏青：抬头、表头
STEEL = "#3d5a80"        # 钢蓝：运输中
GREEN = "#2e7d4f"        # 机台绿：运行
AMBER = "#e8a33d"        # 警告琥珀：待料 / 装卸
RED = "#c0392b"          # 报警红：阻塞 / 异常
GREY = "#8d9298"         # 灰色：停机 / 待命
OCHRE = "#a97b1f"        # 暗黄：等待泊位
RULE = "#c4bfb2"         # 面板外框
HAIR = "#ded9cc"         # 面板内细线
GRIDC = "#e9e5da"        # 纸面暗格
MUTED = "#66635a"        # 次级文字
FAINT = "#98948a"        # 三级文字

CAN_RGB = dict(empty=(0.62, 0.66, 0.70), green=(0.22, 0.68, 0.38),
               yellow=(0.92, 0.74, 0.22), red=(0.80, 0.26, 0.28))

# ---------------------------------------------------------------- 字体
_CJK_CANDIDATES = [
    "Noto Sans CJK SC", "Noto Sans CJK JP", "Noto Sans SC", "Source Han Sans SC",
    "WenQuanYi Zen Hei", "WenQuanYi Micro Hei", "Droid Sans Fallback",
    "Microsoft YaHei", "SimHei", "PingFang SC", "Heiti SC", "AR PL UMing CN",
]
_CJK_FONT_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
]


def _pick_cjk_font():
    """挑一个可用的中文字体族；找不到就退回 DejaVu（中文会掉字，但不崩）。"""
    from matplotlib import font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for name in _CJK_CANDIDATES:
        if name in have:
            return name
    for path in _CJK_FONT_PATHS:
        if os.path.exists(path):
            try:
                fm.fontManager.addfont(path)
                return fm.FontProperties(fname=path).get_name()
            except Exception:
                continue
    return None


_CJK = _pick_cjk_font()
FONT_SANS = ([_CJK] if _CJK else []) + ["DejaVu Sans"]
FONT_MONO = ["DejaVu Sans Mono"] + ([_CJK] if _CJK else [])
plt.rcParams["font.family"] = FONT_SANS
plt.rcParams["axes.unicode_minus"] = False

# ---------------------------------------------------------------- 语义字典
STAGE_CN = {"carding": "梳棉", "drawing": "并条", "roving": "粗纱"}
STAGE_EN = {"carding": "CARDING", "drawing": "DRAWING", "roving": "ROVING"}
MAT_CN = {"empty": "空筒", "green": "生条", "yellow": "熟条", "red": "粗纱"}
MAT_EN = {"empty": "EMPTY CAN", "green": "CARDED SLIVER",
          "yellow": "DRAWN SLIVER", "red": "ROVING PACKAGE"}
FLOW_KEY = ["empty", "green", "yellow", "red"]

ROBOT_STYLE = {
    "idle":          ("待命",     "IDLE",       GREY),
    "to_pickup":     ("取货中",   "TO PICKUP",  STEEL),
    "to_dropoff":    ("送货中",   "TO DROPOFF", NAVY),
    "loading":       ("装货中",   "LOADING",    AMBER),
    "unloading":     ("卸货中",   "UNLOADING",  AMBER),
    "departing":     ("驶离中",   "DEPARTING",  STEEL),
    "waiting_lease": ("等待泊位", "WAIT DOCK",  OCHRE),
    "to_charger":    ("前往充电", "TO CHARGE",  GREEN),
    "charging":      ("充电中",   "CHARGING",   GREEN),
}
MACH_STYLE = {
    "processing": ("加工中", "RUNNING", GREEN),
    "idle":       ("待料",   "IDLE",    GREY),
    "waiting":    ("等料",   "WAITING", AMBER),
    "blocked":    ("阻塞",   "BLOCKED", RED),
}
TASK_STYLE = {
    "pending": ("待分配", "PENDING", AMBER),
    "assigned": ("已派发", "ASSIGNED", STEEL),
    "running": ("执行中", "RUNNING", NAVY),
    "done": ("已完成", "DONE", GREEN),
    "failed": ("失败", "FAILED", RED),
}
SHIFTS = [("丙班", "SHIFT C", 0, 8), ("甲班", "SHIFT A", 8, 16), ("乙班", "SHIFT B", 16, 24)]
WEEKDAY_CN = ["星期一", "星期二", "星期三", "星期四", "星期五", "星期六", "星期日"]


def _hex(rgb) -> str:
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(round(c * 255)))) for c in rgb[:3])


def _tint(rgb, f: float) -> str:
    """把颜色按比例混向白纸色，用于区域底色。"""
    base = (0.965, 0.958, 0.941)
    return _hex(tuple(c * (1 - f) + b * f for c, b in zip(rgb[:3], base)))


def _stage_rgb(stage: str):
    """工序配色：机器占地用的真实工序色（layout.MACHINE_SPEC）。"""
    return layout.MACHINE_SPEC.get(stage, {}).get("rgb", (0.40, 0.40, 0.40))


def _station_cn(name: str) -> str:
    """工位内部名 → 车间里叫得出口的中文名。"""
    if not name:
        return "—"
    if name.startswith("storage_"):
        parts = name.split("_")
        zone = {"empty": "空筒库", "red": "成品库"}.get(parts[1], parts[1])
        tail = parts[-1]
        return f"{zone}取放位{int(tail) + 1}" if tail.isdigit() else zone
    if name.startswith("charger_"):
        tail = name.split("_")[-1]
        return f"充电桩{int(tail) + 1}" if tail.isdigit() else "充电桩"
    parts = name.split("_")
    if len(parts) == 3 and parts[0] in STAGE_CN and parts[2].isdigit():
        kind = "待加工位" if parts[1] == "waiting" else "完工位"
        return f"{STAGE_CN[parts[0]]}{int(parts[2]) + 1}{kind}"
    return name


def _battery_of(robot):
    """电量百分比：RobotStatus 里没有 battery 字段时返回 None，版面上显示 '—'。"""
    for attr in ("battery", "battery_pct", "soc", "power"):
        val = getattr(robot, attr, None)
        if isinstance(val, (int, float)):
            return float(val)
    return None


def _odom_of(robot):
    """累计行驶里程：优先用消息里的 odom_total，没有就返回 None（由看板自行累加）。"""
    for attr in ("odom_total", "odometer"):
        val = getattr(robot, attr, None)
        if isinstance(val, (int, float)):
            return float(val)
    return None


def _set_if_field(msg, name, value):
    """只在消息确实声明了该字段时赋值（避免上游改 msg 时演示路径崩掉）。"""
    try:
        if name in msg.get_fields_and_field_types():
            setattr(msg, name, value)
    except Exception:
        pass


def _battery_color(pct):
    if pct is None:
        return MUTED
    if pct < 20:
        return RED
    if pct < 40:
        return AMBER
    return INK


class Board:
    """纸面排版的绘制助手：图幅坐标定位 + 文本实测宽度（保证列对齐）。"""

    def __init__(self, fig):
        self.fig = fig
        self.r = fig.canvas.get_renderer()
        self.W = float(fig.bbox.width)
        self.H = float(fig.bbox.height)

    # ---- 基本图元
    def rect(self, x, y, w, h, **kw):
        kw.setdefault("facecolor", "none")
        kw.setdefault("edgecolor", "none")
        kw.setdefault("lw", 0.8)
        kw.setdefault("zorder", 2)
        r = Rectangle((x, y), w, h, transform=self.fig.transFigure, **kw)
        self.fig.add_artist(r)
        return r

    def hline(self, x0, x1, y, color=HAIR, lw=0.7, ls="-", zorder=5):
        self.fig.add_artist(Line2D([x0, x1], [y, y], color=color, lw=lw, linestyle=ls,
                                   zorder=zorder, transform=self.fig.transFigure))

    def vline(self, x, y0, y1, color=HAIR, lw=0.7, ls="-", zorder=5):
        self.fig.add_artist(Line2D([x, x], [y0, y1], color=color, lw=lw, linestyle=ls,
                                   zorder=zorder, transform=self.fig.transFigure))

    def text(self, x, y, s, size=8.0, color=INK, weight="normal", family=None,
             ha="left", va="center", rotation=0.0, zorder=8, **kw):
        t = self.fig.text(x, y, s, fontsize=size, color=color, fontweight=weight,
                          family=FONT_SANS if family is None else family,
                          ha=ha, va=va, rotation=rotation, rotation_mode="anchor",
                          zorder=zorder, **kw)
        return t

    # ---- 实测宽度（图幅比例），用于“标签 + 小字”紧跟排版
    def tw(self, t) -> float:
        return t.get_window_extent(renderer=self.r).width / self.W

    def th(self, t) -> float:
        return t.get_window_extent(renderer=self.r).height / self.H

    def seq(self, x, y, items, gap=0.005):
        """横排若干 (文本, 样式) —— 按实测宽度依次摆放，返回结束 x。"""
        for s, st in items:
            t = self.text(x, y, s, **st)
            x += self.tw(t) + gap
        return x

    def panel(self, x, y, w, h, cn, en, note=None, note_color=MUTED, accent=NAVY):
        """画一个直角面板（细线框 + 中文标题 + 英文小字 + 右侧备注），返回内容区。"""
        self.rect(x, y, w, h, facecolor=PANEL, edgecolor=RULE, lw=0.8, zorder=1)
        self.rect(x + 0.0062, y + h - 0.0200, 0.0055, 0.0110, facecolor=accent, zorder=3)
        self.text(x + 0.0160, y + h - 0.0145, cn, size=10.0, weight="bold", color=INK)
        self.text(x + 0.0160, y + h - 0.0272, en, size=5.9, color=FAINT)
        if note:
            self.text(x + w - 0.008, y + h - 0.0180, note, size=6.9, color=note_color,
                      family=FONT_MONO, ha="right")
        self.hline(x + 0.0062, x + w - 0.0062, y + h - 0.0352, color=HAIR, lw=0.7)
        pad = 0.0062
        return (x + pad, y + pad, w - 2 * pad, h - 0.0400 - pad)


class Dashboard(Node):
    def __init__(self):
        super().__init__("fleet_dashboard")
        self.declare_parameter("out_dir", "/tmp/fleetflow_frames")
        self.declare_parameter("every_s", 0.0)     # >0 则定时出图
        self.declare_parameter("once", False)

        self.out_dir = self.get_parameter("out_dir").value
        self.robots: dict[int, RobotStatus] = {}
        self.machines: list[MachineState] = []
        self.summary = dict(materials=0, completed=0, by_color={}, pending=0,
                            running=0, done=0, tasks_created=0)
        self.tasks: dict[int, TransportTask] = {}
        self.odo: dict[int, float] = {}            # 本次会话累计行驶距离（由 distance_done 累加）
        self.hist: list[tuple[float, int]] = []    # (时刻, 累计完成单数) —— 节拍曲线
        self.t0 = None
        self.frames = 0
        self.every_s = float(self.get_parameter("every_s").value)

        self.create_subscription(RobotStatus, "/fleet/robots", self.on_robot, 30)
        self.create_subscription(MachineState, "/factory/machines", self.on_machine, 30)
        self.create_subscription(String, "/factory/summary", self.on_summary, 10)
        self.create_subscription(TransportTask, "/factory/task_status", self.on_task, 30)

        if _CJK is None:
            self.get_logger().warn("未找到中文字体（Noto Sans CJK / 文泉驿 / Droid Sans Fallback），"
                                   "看板中文可能显示为方框")

        every = self.every_s
        if every > 0:
            self.create_timer(every, lambda: self.render(auto=True))
        elif bool(self.get_parameter("once").value):
            self.create_timer(6.0, lambda: (self.render(auto=True), rclpy.shutdown()))
        self.get_logger().info(f"dashboard up · out_dir={self.out_dir}")

    # ---------- 订阅回调 ----------
    def on_robot(self, m: RobotStatus):
        prev = self.robots.get(m.robot_id)
        if prev is not None:
            step = float(m.distance_done) - float(prev.distance_done)
            if 0.0 < step < 5.0 and m.task_id == prev.task_id:      # 同一任务内前进
                self.odo[m.robot_id] = self.odo.get(m.robot_id, 0.0) + step
        self.robots[m.robot_id] = m

    def on_machine(self, m: MachineState):
        self.machines = [x for x in self.machines if x.name != m.name] + [m]

    def on_task(self, m: TransportTask):
        self.tasks[m.task_id] = m

    def on_summary(self, m: String):
        try:
            self.summary = json.loads(m.data)
        except Exception:
            return
        self.hist.append((time.time(), int(self.summary.get("completed", 0) or 0)))
        if len(self.hist) > 4000:
            del self.hist[:2000]

    # ---------- 演示数据（只影响 --demo 路径） ----------
    def load_demo(self):
        """填一组可信的合成状态：8 台 AGV、8 台机器、若干任务与节拍历史。"""
        import random
        rng = random.Random(20240612)
        now = time.time()

        def robot(rid, state, x, y, yaw, task_id, done, total, batt, odom):
            m = RobotStatus()
            m.robot_id, m.state = rid, state
            m.x, m.y, m.yaw = float(x), float(y), float(yaw)
            m.task_id, m.distance_done, m.distance_total = task_id, float(done), float(total)
            _set_if_field(m, "battery", float(batt))        # 上游消息有这两个字段时才写
            _set_if_field(m, "odom_total", float(odom))
            return m

        parked = layout.park_poses(8)
        c0 = layout.CHARGERS["charger_0"]
        demo_robots = [
            robot(1, "idle", parked[0][0], parked[0][1], 0.0, -1, 0.0, 0.0, 92.0, 412.6),
            robot(2, "to_pickup", 4.35, 4.10, -2.05, 120, 3.8, 9.4, 74.0, 286.3),
            robot(3, "idle", parked[2][0], parked[2][1], 0.0, -1, 0.0, 0.0, 88.0, 508.9),
            robot(4, "to_pickup", 11.20, 8.60, 1.42, 121, 6.1, 13.7, 61.0, 331.2),
            robot(5, "charging", c0["x"], 1.55, 0.0, -1, 0.0, 0.0, 34.0, 194.7),
            robot(6, "to_dropoff", 15.05, 11.35, 0.24, 122, 9.6, 12.3, 57.0, 447.5),
            robot(7, "waiting_lease", 21.30, 8.00, 0.0, 123, 12.4, 14.9, 46.0, 262.8),
            robot(8, "unloading", 15.70, 6.20, 1.57, 124, 10.8, 10.8, 18.0, 389.4),
        ]
        self.robots = {m.robot_id: m for m in demo_robots}
        self.odo = {1: 412.6, 2: 286.3, 3: 508.9, 4: 331.2,
                    5: 194.7, 6: 447.5, 7: 262.8, 8: 389.4}

        def task(tid, mat, src, dst, status, rid, sx, sy, dx, dy, prio=1):
            t = TransportTask()
            t.task_id, t.priority, t.material_type = tid, prio, mat
            t.source_name, t.dest_name = src, dst
            t.source_x, t.source_y, t.dest_x, t.dest_y = float(sx), float(sy), float(dx), float(dy)
            t.status, t.robot_id = status, rid
            return t

        demo_tasks = [
            task(120, "empty", "storage_empty_slot_0", "carding_waiting_2", "running", 2, 3.4, 3.0, 5.55, 9.0),
            task(121, "green", "carding_finished_1", "drawing_waiting_0", "assigned", 4, 11.55, 6.0, 12.75, 4.5),
            task(122, "yellow", "drawing_finished_0", "roving_waiting_1", "running", 6, 17.2, 4.5, 17.85, 10.5),
            task(123, "red", "roving_finished_1", "storage_red_slot_1", "assigned", 7, 22.3, 10.5, 22.95, 8.0),
            task(124, "green", "carding_finished_2", "drawing_waiting_1", "running", 8, 11.55, 9.0, 12.75, 10.5),
            task(119, "empty", "storage_empty_slot_3", "carding_waiting_0", "done", 1, 3.4, 12.0, 5.55, 3.0),
            task(118, "red", "roving_finished_0", "storage_red_slot_0", "done", 3, 22.3, 4.5, 22.95, 5.5),
            task(117, "yellow", "drawing_finished_1", "roving_waiting_0", "done", 5, 17.2, 10.5, 17.85, 4.5),
            task(126, "empty", "storage_empty_slot_1", "carding_waiting_3", "pending", -1, 3.4, 6.0, 5.55, 12.0),
            task(127, "green", "carding_finished_0", "drawing_waiting_0", "pending", -1, 11.55, 3.0, 12.75, 4.5),
        ]
        self.tasks = {t.task_id: t for t in demo_tasks}

        self.machines = []
        busy = {("carding", 0): 0.82, ("carding", 1): 0.74, ("carding", 2): 0.91, ("carding", 3): 0.63,
                ("drawing", 0): 0.68, ("drawing", 1): 0.55, ("roving", 0): 0.79, ("roving", 1): 0.46}
        states = {("carding", 0): "processing", ("carding", 1): "processing",
                  ("carding", 2): "processing", ("carding", 3): "idle",
                  ("drawing", 0): "processing", ("drawing", 1): "idle",
                  ("roving", 0): "processing", ("roving", 1): "waiting"}
        for stage in layout.STAGES:
            for lane in range(len(stage["lanes"])):
                m = MachineState()
                m.name = layout.machine_name(stage["name"], lane)
                m.stage = stage["name"]
                m.state = states.get((stage["name"], lane), "idle")
                rect = layout.machine_rect(stage["name"], lane)
                m.x, m.y = rect[0], (rect[1] + rect[3]) / 2.0
                m.input_count = 1 if m.state in ("processing", "waiting") else rng.choice([0, 1])
                m.output_count = 18 + 7 * lane + rng.randint(0, 9)
                m.busy_ratio = busy.get((stage["name"], lane), 0.5)
                self.machines.append(m)

        by_color = {"empty": 12, "green": 16, "yellow": 11, "red": 7}
        self.summary = dict(materials=sum(by_color.values()), completed=128, by_color=by_color,
                            pending=2, running=4, done=117, tasks_created=247)

        # 节拍历史：近 3 分钟，每 5 秒一个采样点（约 0.8 单/5s ≈ 10 单/分钟）
        self.hist = []
        acc = 128 - rng.randint(24, 30)
        for k in range(37):
            acc += rng.choice([0, 0, 1, 1, 1, 2])
            self.hist.append((now - 180.0 + k * 5.0, acc))
        self.summary["completed"] = max(self.summary["completed"], acc)

    # ---------- 出图 ----------
    def render(self, auto=False, path=None, size=None):
        os.makedirs(self.out_dir, exist_ok=True)
        w_px, h_px = size if size else (FIG_W_IN * DPI, FIG_H_IN * DPI)
        # 物理尺寸固定 16 in 宽、dpi 随像素尺寸走：换分辨率不改变版面比例与字号观感
        scale_dpi = max(20.0, w_px / FIG_W_IN)
        fig = plt.figure(figsize=(w_px / scale_dpi, h_px / scale_dpi), dpi=scale_dpi,
                         facecolor=PAPER)
        b = Board(fig)
        self._paper_grid(b)
        self._header(b)
        self._kpi_band(b)
        self._flow_strip(b)
        self._map_panel(b)
        self._util_panel(b)
        self._machine_panel(b)
        self._roster_panel(b)
        self._throughput_panel(b)
        self._footer(b)
        self.frames += 1
        if path is None:
            path = os.path.join(self.out_dir, f"control_center_{self.frames:04d}.png")
        fig.savefig(path, facecolor=PAPER)
        plt.close(fig)
        self.get_logger().info(f"frame -> {path}")
        return path

    # ---------- 纸面暗格 ----------
    def _paper_grid(self, b):
        step = 0.025
        n = int(1 / step)
        for i in range(1, n):
            b.vline(i * step, 0.0, 1.0, color=GRIDC, lw=0.5, zorder=0)
            b.hline(0.0, 1.0, i * step, color=GRIDC, lw=0.5, zorder=0)

    # ---------- 抬头 ----------
    def _header(self, b):
        now = time.time()
        lt = time.localtime(now)
        shift_cn, shift_en, h0, h1 = SHIFTS[(lt.tm_hour // 8) % 3]
        self._shift_cn, self._shift_en = shift_cn, shift_en

        b.rect(0.013, 0.9245, 0.0052, 0.0555, facecolor=NAVY, zorder=3)
        b.text(0.0265, 0.9625, "棉纺一车间 · 生产调度看板", size=19.5, weight="bold", color=INK)
        t = b.text(0.0272, 0.9335, "SPINNING MILL No.1 · PRODUCTION & DISPATCH ANDON BOARD",
                   size=7.2, color=MUTED)
        b.text(0.0272 + b.tw(t) + 0.010, 0.9335, f"线体 LINE：清花 → 梳棉 → 并条 → 粗纱",
               size=7.2, color=FAINT)

        # 右上：班次 / 日期 / 时钟
        b.text(0.987, 0.9335, f"{time.strftime('%Y-%m-%d', lt)}  {WEEKDAY_CN[lt.tm_wday]}",
               size=8.2, color=MUTED, ha="right")
        b.text(0.987, 0.9335 - 0.0215,
               f"班次 {shift_cn} {shift_en} · 班次时段 {h0:02d}:00 – {h1:02d}:00",
               size=8.2, color=MUTED, ha="right")
        clock = b.text(0.987, 0.9645, time.strftime("%H:%M:%S", lt), size=17.5,
                       family=FONT_MONO, weight="bold", color=NAVY, ha="right")
        tx = 0.987 - b.tw(clock) - 0.012          # "系统在线" 的右边界
        stat = b.text(tx, 0.9595, "系统在线", size=9.0, weight="bold",
                      color=GREEN, ha="right")
        # 状态灯放在文字**左侧**，间隙按实测文字宽度算，否则色块会压住"在线"
        lx = tx - b.tw(stat) - 0.006
        b.rect(lx, 0.9555, 0.0075, 0.0085, facecolor=GREEN, zorder=4)
        b.text(lx + 0.00375, 0.9455, "LINK OK", size=5.6,
               color=FAINT, ha="center")
        b.hline(0.013, 0.987, 0.9175, color=INK, lw=1.3)

    # ---------- 关键指标带 ----------
    def _kpi_band(self, b):
        s = self.summary
        bc = s.get("by_color", {}) or {}
        n_mach = len(self.machines)
        avg = sum(float(m.busy_ratio) for m in self.machines) / n_mach if n_mach else None
        n_move = sum(1 for r in self.robots.values()
                     if r.state in ("to_pickup", "to_dropoff", "departing"))
        n_chg = sum(1 for r in self.robots.values() if r.state in ("charging", "to_charger"))
        delta = self._completed_delta(60.0)

        x0, y0, w, h = 0.013, 0.8310, 0.974, 0.0780
        b.rect(x0, y0, w, h, facecolor=PANEL, edgecolor=RULE, lw=0.8, zorder=1)
        cells = [
            ("在制条筒", "WIP CANS", f"{int(s.get('materials', 0) or 0):,}", "只", NAVY,
             "空筒 {} · 生条 {}".format(bc.get("empty", 0), bc.get("green", 0))),
            ("已完成搬运", "COMPLETED", f"{int(s.get('completed', 0) or 0):,}", "单", GREEN,
             ("近 1 分钟 +%d 单" % delta) if delta is not None else "统计中"),
            ("在途任务", "IN TRANSIT", f"{int(s.get('running', 0) or 0):,}", "单", STEEL,
             f"车辆 {n_move} 台在途"),
            ("待分配任务", "QUEUED", f"{int(s.get('pending', 0) or 0):,}", "单", AMBER,
             f"累计派发 {int(s.get('tasks_created', 0) or 0):,} 单"),
            ("设备综合利用率", "AVG UTILISATION", "—" if avg is None else f"{avg * 100:.1f}", "%",
             GREEN if (avg or 0) >= 0.7 else (NAVY if (avg or 0) >= 0.5 else AMBER),
             f"{n_mach} 台样本"),
            ("在线车辆", "AGV ONLINE", f"{len(self.robots):,}", "台", INK,
             f"充电 {n_chg} 台"),
        ]
        cw = w / len(cells)
        for i, (cn, en, val, unit, color, note) in enumerate(cells):
            cx = x0 + i * cw
            if i:
                b.vline(cx, y0 + 0.008, y0 + h - 0.008, color=HAIR, lw=0.7)
            b.rect(cx + 0.0105, y0 + h - 0.0175, 0.0048, 0.0075, facecolor=color, zorder=3)
            b.text(cx + 0.0205, y0 + h - 0.0140, cn, size=9.2, weight="bold", color=INK)
            b.text(cx + 0.0205, y0 + h - 0.0272, en, size=5.8, color=FAINT)
            t = b.text(cx + 0.0105, y0 + 0.0265, val, size=21.0, family=FONT_MONO,
                       weight="bold", color=color)
            b.text(cx + 0.0105 + b.tw(t) + 0.0055, y0 + 0.0195, unit, size=8.0, color=MUTED)
            b.text(cx + 0.0105, y0 + 0.0088, note, size=6.2, color=FAINT)

    def _completed_delta(self, window_s):
        if len(self.hist) < 2:
            return None
        t_end, c_end = self.hist[-1]
        for t, c in reversed(self.hist):
            if t_end - t >= window_s:
                return max(0, c_end - c)
        return None

    # ---------- 物料流转条 ----------
    def _flow_strip(self, b):
        s = self.summary
        bc = s.get("by_color", {}) or {}
        x0, y0, w, h = 0.013, 0.7405, 0.974, 0.0725
        b.rect(x0, y0, w, h, facecolor=PANEL, edgecolor=RULE, lw=0.8, zorder=1)
        b.rect(x0 + 0.0062, y0 + h - 0.0200, 0.0055, 0.0110, facecolor=NAVY, zorder=3)
        b.text(x0 + 0.0160, y0 + h - 0.0145, "物料流转", size=10.0, weight="bold", color=INK)
        b.text(x0 + 0.0160, y0 + h - 0.0272, "WIP FLOW", size=5.9, color=FAINT)
        total = sum(int(bc.get(k, 0) or 0) for k in FLOW_KEY)
        b.text(x0 + 0.1080, y0 + h - 0.0185, f"在制合计", size=7.4, color=MUTED)
        t = b.text(x0 + 0.1080, y0 + h - 0.0320, f"{total:,}", size=13.0, family=FONT_MONO,
                   weight="bold", color=INK)
        b.text(x0 + 0.1080 + b.tw(t) + 0.0035, y0 + h - 0.0345, "只", size=7.0, color=MUTED)

        bx0, bx1 = x0 + 0.1520, x0 + w - 0.0062
        gap = 0.0195
        bw = (bx1 - bx0 - gap * (len(FLOW_KEY) - 1)) / len(FLOW_KEY)
        maxn = max([int(bc.get(k, 0) or 0) for k in FLOW_KEY] + [1])
        for i, key in enumerate(FLOW_KEY):
            bx = bx0 + i * (bw + gap)
            by, bh = y0 + 0.0075, h - 0.0150
            rgb = CAN_RGB[key]
            b.rect(bx, by, bw, bh, facecolor=_tint(rgb, 0.90), edgecolor=RULE, lw=0.7, zorder=2)
            b.rect(bx, by, 0.0035, bh, facecolor=_hex(rgb), zorder=4)
            n = int(bc.get(key, 0) or 0)
            # 左：占比小字；中：大数字；右：品名（中文 + 英文小字）；底：占比条
            b.text(bx + 0.0105, by + bh - 0.0090, f"占比 {n / max(1, total) * 100:.0f}%",
                   size=6.2, color=MUTED)
            t = b.text(bx + 0.0105, by + bh - 0.0285, f"{n:,}", size=17.0, family=FONT_MONO,
                       weight="bold", color=INK)
            b.text(bx + 0.0105 + b.tw(t) + 0.0050, by + bh - 0.0310, "只", size=7.0, color=MUTED)
            b.text(bx + bw - 0.0075, by + bh - 0.0130, MAT_CN[key], size=9.0, weight="bold",
                   color=INK, ha="right")
            b.text(bx + bw - 0.0075, by + bh - 0.0250, MAT_EN[key], size=5.6, color=FAINT, ha="right")
            # 占比条
            frac = n / maxn
            b.rect(bx + 0.0105, by + 0.0075, bw - 0.0210, 0.0055, facecolor="#eeeae0", zorder=3)
            b.rect(bx + 0.0105, by + 0.0075, (bw - 0.0210) * frac, 0.0055,
                   facecolor=_hex(rgb), zorder=4)
            if i < len(FLOW_KEY) - 1:
                ax0 = bx + bw + gap * 0.16
                ax1 = bx + bw + gap * 0.84
                ay = by + bh * 0.50
                b.hline(ax0, ax1 - 0.0035, ay, color=INK, lw=1.0, zorder=4)
                b.fig.add_artist(Polygon([[ax1, ay], [ax1 - 0.0052, ay + 0.0058],
                                          [ax1 - 0.0052, ay - 0.0058]], closed=True,
                                         facecolor=INK, edgecolor="none", zorder=4,
                                         transform=b.fig.transFigure))

    # ---------- 车间平面图 ----------
    def _map_panel(self, b, region=None):
        # region 可覆盖：网页端的"大屏"模式就是把平面图铺满整张画布，
        # 复用同一个绘制函数，才不会出现两块屏画出两个样子。
        x0, y0, w, h = region if region else (0.013, 0.2775, 0.3865, 0.4485)
        b.rect(x0, y0, w, h, facecolor=PANEL, edgecolor=RULE, lw=0.8, zorder=1)
        b.rect(x0 + 0.0062, y0 + h - 0.0200, 0.0055, 0.0110, facecolor=NAVY, zorder=3)
        b.text(x0 + 0.0160, y0 + h - 0.0145, "车间平面图", size=10.0, weight="bold", color=INK)
        _bw, _bh = layout.world_bounds()[1] - layout.world_bounds()[0], 16.0
        b.text(x0 + 0.0160, y0 + h - 0.0272,
               f"FLOOR PLAN · {_bw:.0f} × {_bh:.0f} m · 1:1",
               size=5.9, color=FAINT)
        n_move = sum(1 for r in self.robots.values() if r.task_id is not None and r.task_id >= 0)
        b.text(x0 + w - 0.008, y0 + h - 0.0180, f"在途 {n_move} 台 · 工位 {len(layout.all_station_points())} 处",
               size=6.9, color=MUTED, family=FONT_MONO, ha="right")
        b.hline(x0 + 0.0062, x0 + w - 0.0062, y0 + h - 0.0352, color=HAIR, lw=0.7)

        # 图例条（面板底部）
        self._map_legend(b, x0 + 0.0100, y0 + 0.0088, w - 0.0200)

        ax_x, ax_y = x0 + 0.0062, y0 + 0.0245
        ax_w, ax_h = w - 0.0124, h - 0.0400 - 0.0245 - 0.0062
        ax = b.fig.add_axes([ax_x, ax_y, ax_w, ax_h])
        ax.set_zorder(1.6)          # 必须高于面板底色矩形，否则整张平面图会被盖住
        ax.patch.set_visible(False)
        ax.set_axis_off()
        self._draw_map(b, ax, ax_w * b.W, ax_h * b.H)

    def _map_legend(self, b, x, y, w):
        sw = 0.0058
        items = [("idle", "待命"), ("to_pickup", "取货/送货"), ("loading", "装卸作业"),
                 ("charging", "充电"), ("waiting_lease", "等待泊位")]
        cx = x
        for key, label in items:
            color = ROBOT_STYLE[key][2]
            b.fig.add_artist(Polygon([[cx, y - 0.0035], [cx + sw, y + 0.0024],
                                      [cx, y + 0.0083], [cx + sw * 0.42, y + 0.0024]],
                                     closed=True, facecolor=color, edgecolor=INK, lw=0.4,
                                     zorder=5, transform=b.fig.transFigure))
            t = b.text(cx + sw + 0.0035, y + 0.0024, label, size=6.3, color=INK)
            cx += sw + 0.0035 + b.tw(t) + 0.0125
        b.hline(cx, cx + 0.0110, y + 0.0024, color=STEEL, lw=0.9, ls=(0, (3, 2)), zorder=5)
        t = b.text(cx + 0.0135, y + 0.0024, "调度路线", size=6.3, color=INK)
        cx += 0.0135 + b.tw(t) + 0.0125
        b.rect(cx + 0.0012, y - 0.0014, 0.0046, 0.0046, facecolor="none", edgecolor=STEEL,
               lw=0.7, ls=(0, (2, 1.6)), zorder=5)
        b.text(cx + 0.0085, y + 0.0024, "停靠位", size=6.3, color=INK)
        # 比例尺（右上角对齐）
        sx1 = x + w
        b.hline(sx1 - 0.0480, sx1 - 0.0080, y + 0.0010, color=INK, lw=1.1, zorder=5)
        for k in range(5):
            xx = sx1 - 0.0480 + k * 0.0100
            b.vline(xx, y + 0.0010, y + 0.0052, color=INK, lw=0.8, zorder=5)
        b.text(sx1 - 0.0080, y + 0.0068, "5 m", size=6.0, color=MUTED, ha="right",
               family=FONT_MONO)

    def _draw_map(self, b, ax, box_w_px, box_h_px):
        # 可视范围必须跟着布局走（见 layout.world_bounds 的注释）：
        # 工序净宽改成 2.0 m 之后红料库东移到 x≈28，写死的 26.25 会把
        # 货架画到地图框外、压住右侧面板。这里改成推导 + 左右各留一点边。
        wx0, wx1, wy0, wy1 = layout.world_bounds()
        x_min, x_max = wx0 - 0.25, wx1 + 0.25
        y_mid, x_span = (wy0 + wy1) / 2, (x_max - x_min)
        y_span = x_span * box_h_px / box_w_px
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_mid - y_span / 2, y_mid + y_span / 2)

        # 地面 + 1m 细格 + 柱网
        floor_w, floor_h = wx1 - wx0, wy1 - wy0
        ax.add_patch(Rectangle((wx0, wy0), floor_w, floor_h,
                               facecolor="#f8f6f0", edgecolor="none", zorder=0))
        for gx in range(int(wx0) + 1, int(wx1) + 1):
            ax.plot([gx, gx], [wy0, wy1], color="#eae6db", lw=0.35, zorder=1)
        for gy in range(int(wy0) + 1, int(wy1) + 1):
            ax.plot([wx0, wx1], [gy, gy], color="#eae6db", lw=0.35, zorder=1)
        for gx in [layout.COLUMN_PITCH * k for k in range(1, 5)
                   if layout.COLUMN_PITCH * k < wx1]:
            ax.plot([gx, gx], [wy0, wy1], color="#ddd7c8", lw=0.7, ls=(0, (6, 4)), zorder=1.2)
        ax.plot([wx0, wx1], [8, 8], color="#ddd7c8", lw=0.7, ls=(0, (6, 4)), zorder=1.2)

        # 工序流向（空筒库 → 成品库，走 y=7.5 的通道）
        ax.annotate("", xy=(layout.STORAGE_SLOTS["red"]["x"] + 0.2, 7.5), xytext=(3.0, 7.5),
                    arrowprops=dict(arrowstyle="-|>", color="#b3ada0", lw=1.1,
                                    linestyle=(0, (7, 4))), zorder=2)
        ax.text(3.10, 7.62, "投料", color=FAINT, fontsize=5.0, ha="left", va="bottom", zorder=3)
        ax.text(layout.STORAGE_SLOTS["red"]["x"] + 0.1, 7.62, "入库", color=FAINT,
                fontsize=5.0, ha="right", va="bottom", zorder=3)

        # 三个工序区
        # 工序区边界由该段自己的等料位/完工位推出，写死会在改净宽后错位
        edges = []
        for i, st in enumerate(layout.STAGES):
            left = st["wait_x"] - 1.00 if i == 0 else (st["wait_x"] + layout.STAGES[i-1]["done_x"]) / 2
            right = (st["done_x"] + layout.STAGES[i+1]["wait_x"]) / 2 if i + 1 < len(layout.STAGES) else st["done_x"] + 1.10
            edges.append((left, right))
        for st, (zx0, zx1) in zip(layout.STAGES, edges):
            rgb = _stage_rgb(st["name"])
            ax.add_patch(Rectangle((zx0, 1.55), zx1 - zx0, 11.90, facecolor=_tint(rgb, 0.925),
                                   edgecolor=_hex(rgb), lw=0.6, ls=(0, (5, 3)), zorder=1.4))
            ax.text(zx0 + 0.25, 13.10, f"{st['cn']}区", color=_hex(rgb), fontsize=6.2,
                    fontweight="bold", va="center", zorder=3)
            ax.text(zx0 + 1.45, 13.10, f"{STAGE_EN[st['name']]} × {len(st['lanes'])}",
                    color=FAINT, fontsize=4.6, va="center", zorder=3)

        # 货架（空筒库 / 成品库）
        for name, rect in layout.rack_rects().items():
            zone = layout.STORAGE[name]
            ax.add_patch(Rectangle((rect[0], rect[1]), rect[2] - rect[0], rect[3] - rect[1],
                                   facecolor="#efece3", edgecolor="#a9a396", lw=0.7,
                                   hatch="////", zorder=1.6))
            for yy in [rect[1] + 1.2, rect[1] + 3.9, rect[1] + 6.6, rect[1] + 9.3]:
                ax.plot([rect[0], rect[2]], [yy, yy], color="#c6c0b2", lw=0.4, zorder=1.7)
            ax.text(zone["x"], rect[3] + 0.72, zone["cn"], color=INK, fontsize=6.0,
                    fontweight="bold", ha="center", va="center", zorder=3)
            ax.text(zone["x"], rect[3] + 0.26, "CAN STORE" if name == "empty" else "YARN STORE",
                    color=FAINT, fontsize=4.4, ha="center", va="center", zorder=3)

        # 取放位（虚线小方块）+ 充电区 + 待命区
        for pt in layout.STORAGE_SLOT_POINTS.values():
            ax.add_patch(Rectangle((pt["x"] - 0.20, pt["y"] - 0.20), 0.40, 0.40, facecolor="none",
                                   edgecolor=STEEL, lw=0.6, ls=(0, (2, 1.6)), zorder=2))
        ax.add_patch(Rectangle((3.55, 0.35), 3.80, 1.85, facecolor="#eef1f4",
                               edgecolor="#b9c2cb", lw=0.6, zorder=1.5))
        ax.text(3.72, 1.98, "充电区 CHARGING", color=MUTED, fontsize=4.8, va="center", zorder=3)
        for c in layout.CHARGERS.values():
            ax.add_patch(Rectangle((c["x"] - 0.34, c["y"] - 1.11), 0.68, 0.52,
                                   facecolor="#cfd8e0", edgecolor=INK, lw=0.6, zorder=3))
        park = layout.park_poses(8)
        if park:
            px0 = min(p[0] for p in park) - 0.55
            px1 = max(p[0] for p in park) + 0.55
            ax.add_patch(Rectangle((px0, 0.62), px1 - px0, 1.95, facecolor="#f1efe7",
                                   edgecolor="#c6c0b2", lw=0.6, ls=(0, (4, 3)), zorder=1.5))
            ax.text(px1 - 0.15, 2.72, "待命区 AGV PARK", color=MUTED, fontsize=4.8,
                    ha="right", va="center", zorder=3)
            for p in park:
                ax.add_patch(Rectangle((p[0] - 0.28, p[1] - 0.34), 0.56, 0.68, facecolor="none",
                                       edgecolor="#c6c0b2", lw=0.5, ls=(0, (2, 2)), zorder=2))

        # 工位：待加工位（空心）/ 完工位（实心）
        for st in layout.STAGES:
            for lane in range(len(st["lanes"])):
                rgb = _hex(_stage_rgb(st["name"]))
                for kind, fc in (("waiting", "none"), ("finished", _tint(_stage_rgb(st["name"]), 0.55))):
                    pt = layout.station(st["name"], kind, lane)
                    ax.add_patch(Rectangle((pt["x"] - 0.17, pt["y"] - 0.17), 0.34, 0.34,
                                           facecolor=fc, edgecolor=rgb, lw=0.7, zorder=2.5))
                rect = layout.machine_rect(st["name"], lane)
                ax.add_patch(Rectangle((rect[0], rect[1]), rect[2] - rect[0], rect[3] - rect[1],
                                       facecolor=rgb, edgecolor=INK, lw=0.7, zorder=3))
                ax.text((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2,
                        f"{st['cn']}{lane + 1}", color="#ffffff", fontsize=4.9,
                        fontweight="bold", ha="center", va="center", zorder=4)

        # 在途任务路线
        for rid, r in sorted(self.robots.items()):
            if r.task_id is None or r.task_id < 0:
                continue
            t = self.tasks.get(r.task_id)
            if t is None or t.status not in ("assigned", "running"):
                continue
            target = (t.source_x, t.source_y) if r.state == "to_pickup" else (t.dest_x, t.dest_y)
            ax.plot([r.x, target[0]], [r.y, target[1]], color=STEEL, lw=1.0,
                    linestyle=(0, (4, 3)), zorder=5)
            ax.add_patch(Rectangle((target[0] - 0.26, target[1] - 0.26), 0.52, 0.52,
                                   facecolor="none", edgecolor=STEEL, lw=0.9, zorder=5))

        # 机台状态灯 + 机器状态（Andon 三色灯）
        for m in self.machines:
            _, _, color = MACH_STYLE.get(m.state, ("", "", GREY))
            rect = None
            for st in layout.STAGES:
                if st["name"] != m.stage:
                    continue
                for lane in range(len(st["lanes"])):
                    if layout.machine_name(m.stage, lane) == m.name:
                        rect = layout.machine_rect(m.stage, lane)
            if rect is None:
                continue
            ax.add_patch(Rectangle((rect[2] - 0.34, rect[3] + 0.06), 0.32, 0.32,
                                   facecolor=color, edgecolor=INK, lw=0.4, zorder=4))

        # 车辆：方向三角形 + 编号
        for rid, r in sorted(self.robots.items()):
            st = ROBOT_STYLE.get(r.state, ("", "", GREY))
            color = st[2]
            L, Wd = 1.00, 0.62
            pts = [(L * 0.55, 0.0), (-L * 0.45, Wd / 2), (-L * 0.30, 0.0), (-L * 0.45, -Wd / 2)]
            ca, sa = math.cos(r.yaw), math.sin(r.yaw)
            pts = [(r.x + px * ca - py * sa, r.y + px * sa + py * ca) for px, py in pts]
            ax.add_patch(Polygon(pts, closed=True, facecolor=color, edgecolor=INK, lw=0.6,
                                 zorder=6))
            lab = ax.text(r.x + 0.42, r.y + 0.72, f"AGV{rid}", color=INK, fontsize=4.9,
                          fontweight="bold", ha="left", va="bottom", zorder=7)
            lab.set_path_effects([path_effects.withStroke(linewidth=1.6, foreground="#f8f6f0")])

        # 图面装饰：柱网轴号 + 北向 + 大门
        for k in range(4):
            ax.text(layout.COLUMN_PITCH * k + layout.COLUMN_PITCH / 2, 15.55, str(k + 1),
                    color="#c2bcae", fontsize=4.4, ha="center", va="center", zorder=3)
        for k, letter in enumerate("ABC"):
            ax.text(0.35, 2.67 + k * 5.33, letter, color="#c2bcae", fontsize=4.4,
                    ha="center", va="center", zorder=3)
        ax.add_patch(Polygon([[25.35, 15.75], [25.15, 15.05], [25.55, 15.05]], closed=True,
                             facecolor=INK, edgecolor="none", zorder=3))
        ax.text(25.35, 14.85, "N", color=INK, fontsize=4.6, ha="center", va="top", zorder=3)
        ax.add_patch(Rectangle((20.6, -0.12), 2.6, 0.24, facecolor="#f8f6f0", edgecolor="none",
                               zorder=4))
        ax.text(21.9, 0.62, "物流门 GATE", color=MUTED, fontsize=4.6, ha="center",
                va="center", zorder=3)

        # 墙体（双线）+ 柱子
        ax.add_patch(Rectangle((wx0, wy0), wx1 - wx0, wy1 - wy0,
                               facecolor="none", edgecolor=INK, lw=1.4, zorder=6))
        ax.add_patch(Rectangle((0.28, 0.28), 25.44, 15.44, facecolor="none",
                               edgecolor="#b8b2a4", lw=0.5, zorder=6))
        for gx in [layout.COLUMN_PITCH * k for k in range(1, 4)]:
            for gy in [8.0]:
                ax.add_patch(Rectangle((gx - layout.COLUMN_SIZE / 2, gy - layout.COLUMN_SIZE / 2),
                                       layout.COLUMN_SIZE, layout.COLUMN_SIZE,
                                       facecolor="#b8b2a4", edgecolor=INK, lw=0.5, zorder=6.5))

    # ---------- 设备利用率 ----------
    def _util_panel(self, b):
        x0, y0, w, h = 0.4115, 0.2775, 0.2350, 0.4485
        n_mach = len(self.machines)
        avg = sum(float(m.busy_ratio) for m in self.machines) / n_mach if n_mach else None
        note = "无设备数据" if avg is None else f"平均 {avg * 100:.1f}%"
        ix, iy, iw, ih = b.panel(x0, y0, w, h, "设备利用率", "MACHINE UTILISATION", note=note)

        top = iy + ih
        # 顶部刻度 + 100% 参考线
        gx0, gx1 = ix + 0.28 * iw, ix + 0.76 * iw
        vx = ix + 0.995 * iw          # 数值列（等宽、右对齐）
        for frac, lab in ((0.0, "0"), (0.25, "25"), (0.5, "50"), (0.75, "75")):
            xx = gx0 + (gx1 - gx0) * frac
            b.vline(xx, iy + 0.014, top - 0.014, color="#e6e2d6", lw=0.6, zorder=2)
            b.text(xx, top - 0.0075, lab, size=5.6, color=FAINT, family=FONT_MONO, ha="center")
        b.vline(gx1, iy + 0.014, top - 0.014, color=RED, lw=0.9, ls=(0, (4, 2.5)), zorder=4)
        b.text(gx1, top - 0.0075, "100% 参考线", size=5.6, color=RED, family=FONT_MONO,
               ha="center")

        by_stage: dict[str, list] = {}
        for m in self.machines:
            by_stage.setdefault(m.stage, []).append(m)
        order = [s["name"] for s in layout.STAGES if s["name"] in by_stage]
        order += [k for k in by_stage if k not in order]
        n_rows = sum(1 + len(by_stage[k]) for k in order) or 1
        avail = (top - 0.016) - (iy + 0.0345)
        row_h = min(0.0305, avail / n_rows)
        cur = top - 0.018 - row_h * 0.5

        if not self.machines:
            b.text(ix + iw / 2, iy + ih / 2, "无设备数据 / NO MACHINE DATA", size=8.0,
                   color=FAINT, ha="center")
            return

        for stage in order:
            ms = sorted(by_stage[stage], key=lambda m: m.name)
            vals = [float(m.busy_ratio) for m in ms]
            s_avg = sum(vals) / len(vals)
            col = _hex(_stage_rgb(stage))
            b.rect(ix, cur - row_h * 0.46, iw, row_h * 0.92, facecolor="#f2efe6",
                   edgecolor="none", zorder=2)
            b.rect(ix + 0.0016, cur - 0.0043, 0.0038, 0.0086, facecolor=col, zorder=4)
            b.text(ix + 0.0080, cur + 0.0022, f"{STAGE_CN[stage]}区", size=7.8, weight="bold",
                   color=INK)
            b.text(ix + 0.0080, cur - 0.0058, f"{STAGE_EN[stage]} · {len(ms)} 台", size=4.8,
                   color=FAINT)
            b.text(vx, cur, f"{s_avg * 100:.0f}%", size=7.6, family=FONT_MONO,
                   weight="bold", color=col, ha="right")
            cur -= row_h
            for m in ms:
                r = max(0.0, min(1.0, float(m.busy_ratio)))
                lane = m.name.rsplit("_", 1)[-1]
                b.text(ix + 0.0155, cur, f"{STAGE_CN[stage]}{int(lane) + 1 if lane.isdigit() else ''}",
                       size=6.6, color=INK)
                b.rect(gx0, cur - 0.0055, gx1 - gx0, 0.0110, facecolor="#efebe1", zorder=3)
                b.rect(gx0, cur - 0.0055, (gx1 - gx0) * r, 0.0110, facecolor=col, zorder=4)
                b.text(vx, cur, f"{r * 100:.0f}%", size=6.6, family=FONT_MONO, color=INK,
                       ha="right", va="center")
                cur -= row_h

        # 底部口径说明（避免面板下方留白）
        y = iy + 0.0060
        b.hline(ix, ix + iw, y + 0.0145, color=HAIR, lw=0.7)
        hi = sum(1 for m in self.machines if float(m.busy_ratio) >= 0.75)
        lo = sum(1 for m in self.machines if float(m.busy_ratio) < 0.50)
        b.text(ix, y + 0.0080, "口径：开机时长占比", size=6.4, color=MUTED)
        b.text(ix + iw, y + 0.0080, f"≥75% {hi} 台 · <50% {lo} 台", size=6.4,
               family=FONT_MONO, color=MUTED, ha="right")
        b.text(ix, y - 0.0005, f"样本 {n_mach} 台 · 统计窗口 本班次", size=6.0, color=FAINT)

    # ---------- 机台状态 ----------
    def _machine_panel(self, b):
        x0, y0, w, h = 0.6585, 0.2775, 0.3285, 0.4485
        n_run = sum(1 for m in self.machines if m.state == "processing")
        wip = sum(int(m.input_count) for m in self.machines)
        ix, iy, iw, ih = b.panel(x0, y0, w, h, "机台状态", "MACHINE STATUS",
                                 note=f"运行 {n_run}/{len(self.machines)} 台 · 待加工 {wip} 只")
        top = iy + ih
        cols = [(0.010, "机台", "MACHINE", "left"), (0.240, "状态", "STATE", "left"),
                (0.590, "待加工", "IN", "right"), (0.760, "已完工", "OUT", "right"),
                (0.995, "标准节拍", "CYCLE", "right")]
        for cx, cn, en, al in cols:
            b.text(ix + cx * iw, top - 0.0090, cn, size=6.6, weight="bold", color=NAVY,
                   ha=al)
            b.text(ix + cx * iw, top - 0.0180, en, size=5.0, color=FAINT, ha=al)
        b.hline(ix, ix + iw, top - 0.0235, color=NAVY, lw=0.8)

        by_stage: dict[str, list] = {}
        for m in self.machines:
            by_stage.setdefault(m.stage, []).append(m)
        order = [s["name"] for s in layout.STAGES if s["name"] in by_stage]
        order += [k for k in by_stage if k not in order]
        n_rows = sum(1 + len(by_stage[k]) for k in order) or 1
        avail = (top - 0.0270) - (iy + 0.0320)
        row_h = min(0.0270, avail / n_rows)
        cur = top - 0.0270 - row_h * 0.5

        if not self.machines:
            b.text(ix + iw / 2, iy + ih / 2, "无设备数据 / NO MACHINE DATA", size=8.0,
                   color=FAINT, ha="center")
            return

        for stage in order:
            ms = sorted(by_stage[stage], key=lambda m: m.name)
            rgb = _stage_rgb(stage)
            running = sum(1 for m in ms if m.state == "processing")
            b.rect(ix, cur - row_h * 0.46, iw, row_h * 0.92, facecolor="#f2efe6",
                   edgecolor="none", zorder=2)
            b.rect(ix + 0.0016, cur - 0.0043, 0.0038, 0.0086, facecolor=_hex(rgb), zorder=4)
            b.text(ix + 0.0080, cur + 0.0016, f"{STAGE_CN[stage]}工序", size=7.6,
                   weight="bold", color=INK)
            b.text(ix + 0.0080, cur - 0.0060, f"{STAGE_EN[stage]} · {len(ms)} 台", size=4.8,
                   color=FAINT)
            b.text(ix + iw - 0.001, cur, f"运行 {running}/{len(ms)}", size=6.2,
                   family=FONT_MONO, color=MUTED, ha="right")
            cur -= row_h
            for m in ms:
                lane = m.name.rsplit("_", 1)[-1]
                _cn, _en, color = MACH_STYLE.get(m.state, ("未知", "UNKNOWN", GREY))
                b.text(ix + 0.010 * iw, cur, f"{STAGE_CN[stage]}{int(lane) + 1 if lane.isdigit() else ''}",
                       size=6.9, color=INK)
                sx = ix + 0.240 * iw
                b.rect(sx, cur - 0.0032, 0.0044, 0.0064, facecolor=color, zorder=4)
                b.text(sx + 0.0075, cur, _cn, size=6.9, color=color, weight="bold")
                b.text(sx + 0.0075 + 0.0290, cur, _en, size=4.8, color=FAINT)
                b.text(ix + 0.590 * iw, cur, f"{int(m.input_count)}", size=7.4,
                       family=FONT_MONO, color=INK, ha="right")
                b.text(ix + 0.760 * iw, cur, f"{int(m.output_count)}", size=7.4,
                       family=FONT_MONO, color=INK, ha="right")
                cyc = next((s["process_s"] for s in layout.STAGES if s["name"] == m.stage), None)
                b.text(ix + 0.995 * iw, cur, "—" if cyc is None else f"{cyc:.1f}s", size=6.6,
                       family=FONT_MONO, color=MUTED, ha="right")
                cur -= row_h

        # 底部状态汇总
        y = iy + 0.0075
        b.hline(ix, ix + iw, y + 0.0135, color=HAIR, lw=0.7)
        cx = ix
        for key in ("processing", "waiting", "idle", "blocked"):
            cn, en, color = MACH_STYLE[key]
            n = sum(1 for m in self.machines if m.state == key)
            if key == "waiting" and n == 0:
                continue
            b.rect(cx, y + 0.0016, 0.0044, 0.0064, facecolor=color, zorder=4)
            t = b.text(cx + 0.0075, y + 0.0048, f"{cn} {n}", size=6.9, color=INK)
            cx += 0.0075 + b.tw(t) + 0.0140

    # ---------- 车辆状态表 ----------
    def _roster_panel(self, b):
        x0, y0, w, h = 0.013, 0.0480, 0.6790, 0.2175
        robots = [self.robots[k] for k in sorted(self.robots)]
        n_move = sum(1 for r in robots if r.state in ROBOT_STYLE and
                     ROBOT_STYLE[r.state][0] in ("取货中", "送货中", "驶离中"))
        batts = [v for v in (_battery_of(r) for r in robots) if v is not None]
        if not robots:
            note = "无车辆数据"
        elif batts:
            note = f"{len(robots)} 台在线 · 在途 {n_move} 台 · 平均电量 {sum(batts) / len(batts):.0f}%"
        else:
            note = f"{len(robots)} 台在线 · 在途 {n_move} 台 · 电量未上报"
        ix, iy, iw, ih = b.panel(x0, y0, w, h, "车辆状态表", "AGV FLEET ROSTER", note=note)
        top = iy + ih

        cols = [
            (0.012, "车辆", "AGV ID", "left"),
            (0.070, "状态", "STATE", "left"),
            (0.250, "电量", "BATT", "right"),
            (0.320, "任务号", "TASK", "right"),
            (0.400, "搬运内容", "MATERIAL", "left"),
            (0.560, "起点", "FROM", "left"),
            (0.760, "终点", "TO", "left"),
            (0.905, "本单里程", "TRIP m", "right"),
            (1.000, "累计里程", "ODO m", "right"),
        ]
        for cx, cn, en, al in cols:
            b.text(ix + cx * iw, top - 0.0090, cn, size=6.6, weight="bold", color=NAVY, ha=al)
            b.text(ix + cx * iw, top - 0.0180, en, size=5.0, color=FAINT, ha=al)
        b.hline(ix, ix + iw, top - 0.0235, color=NAVY, lw=0.8)

        body_top = top - 0.0285
        if not robots:
            b.text(ix + iw / 2, (body_top + iy) / 2, "等待车辆状态上报 / NO AGV DATA",
                   size=8.0, color=FAINT, ha="center")
            return
        shown = robots[:10]
        row_h = min(0.0265, (body_top - iy) / max(1, len(shown)))
        cur = body_top - row_h * 0.5
        for i, r in enumerate(shown):
            if i % 2 == 1:
                b.rect(ix, cur - row_h * 0.46, iw, row_h * 0.92, facecolor="#f4f1ea",
                       edgecolor="none", zorder=1.5)
            b.text(ix + 0.012 * iw, cur, f"AGV-{r.robot_id:02d}", size=7.4, family=FONT_MONO,
                   weight="bold", color=INK)
            cn, en, color = ROBOT_STYLE.get(r.state, (r.state or "未知", "", GREY))
            sx = ix + 0.070 * iw
            b.rect(sx, cur - 0.0032, 0.0044, 0.0064, facecolor=color, zorder=4)
            t = b.text(sx + 0.0075, cur, cn, size=7.2, color=INK)
            b.text(sx + 0.0075 + b.tw(t) + 0.0040, cur, en, size=4.9, color=FAINT)

            batt = _battery_of(r)
            b.text(ix + 0.250 * iw, cur, "—" if batt is None else f"{batt:.0f}%", size=7.2,
                   family=FONT_MONO, color=_battery_color(batt), ha="right")
            tid = int(r.task_id) if r.task_id is not None else -1
            b.text(ix + 0.320 * iw, cur, "—" if tid < 0 else f"#{tid:04d}", size=7.2,
                   family=FONT_MONO, color=INK if tid >= 0 else FAINT, ha="right")

            task = self.tasks.get(tid) if tid >= 0 else None
            if task is None:
                for cx in (0.400, 0.560, 0.760):
                    b.text(ix + cx * iw, cur, "—", size=7.0, color=FAINT, family=FONT_MONO)
            else:
                mat = MAT_CN.get(task.material_type, task.material_type or "—")
                b.text(ix + 0.400 * iw, cur, f"{mat}搬运", size=7.0, color=INK)
                b.text(ix + 0.560 * iw, cur, _station_cn(task.source_name), size=6.9, color=INK)
                b.text(ix + 0.760 * iw, cur, _station_cn(task.dest_name), size=6.9, color=INK)

            dd, dt = float(r.distance_done), float(r.distance_total)
            trip = f"{dd:5.1f} / {dt:5.1f}" if dt > 0 else "    —    "
            b.text(ix + 0.905 * iw, cur, trip, size=7.0, family=FONT_MONO,
                   color=INK if dt > 0 else FAINT, ha="right")
            odom = _odom_of(r)
            if odom is None:                      # 消息没带里程表就退回本会话累加值
                odom = self.odo.get(r.robot_id, 0.0)
            b.text(ix + 1.000 * iw, cur, f"{odom:6.1f}", size=7.0,
                   family=FONT_MONO, color=INK, ha="right")
            cur -= row_h

        if len(robots) > len(shown):
            b.text(ix + iw, iy + 0.004, f"另有 {len(robots) - len(shown)} 台未列出", size=6.4,
                   color=MUTED, ha="right")

    # ---------- 任务节拍 / 最近完成 ----------
    def _throughput_panel(self, b):
        x0, y0, w, h = 0.7040, 0.0480, 0.2830, 0.2175
        buckets, bucket_s = self._throughput_buckets(180.0, 15.0)
        done = sorted((t for t in self.tasks.values() if t.status == "done"),
                      key=lambda t: t.task_id, reverse=True)
        note = f"近 3 分钟 · 完成 {sum(buckets)} 单" if buckets else "数据积累中"
        ix, iy, iw, ih = b.panel(x0, y0, w, h, "任务节拍 · 最近完成",
                                 "THROUGHPUT & RECENT COMPLETIONS", note=note)

        # 节拍柱图
        top = iy + ih
        ch_h = ih * 0.50
        cy0 = top - ch_h
        peak = max(buckets) if buckets else 0
        if peak > 0:
            mean = sum(buckets) / len(buckets)
            bw = iw / len(buckets)
            for i, val in enumerate(buckets):
                bh = ch_h * (val / peak) * 0.80
                color = NAVY if i < len(buckets) - 1 else AMBER
                b.rect(ix + i * bw + bw * 0.12, cy0 + 0.006, bw * 0.76, max(bh, 0.0012),
                       facecolor=color, zorder=4)
            # 均值线 + 图例（放在图上方，避免压住柱子）
            my = cy0 + 0.006 + ch_h * (mean / peak) * 0.80
            b.hline(ix, ix + iw, my, color=RED, lw=0.7, ls=(0, (3, 2)), zorder=5)
            ty = top - 0.0060
            t = b.text(ix, ty, f"峰值 {peak} 单", size=6.0, family=FONT_MONO, color=MUTED)
            b.hline(ix + b.tw(t) + 0.0080, ix + b.tw(t) + 0.0160, ty, color=RED, lw=0.7,
                    ls=(0, (3, 2)), zorder=5)
            b.text(ix + b.tw(t) + 0.0195, ty, f"均值 {mean:.1f} 单 / 15s", size=6.0,
                   family=FONT_MONO, color=RED)
        else:
            b.rect(ix, cy0 + 0.006, iw, ch_h * 0.80, facecolor="#f4f1ea", edgecolor=HAIR,
                   lw=0.6, zorder=3)
            b.text(ix + iw / 2, cy0 + 0.006 + ch_h * 0.40, "数据积累中 · 尚无节拍样本",
                   size=6.6, color=FAINT, ha="center")
        b.hline(ix, ix + iw, cy0 + 0.004, color=RULE, lw=0.7)
        b.text(ix, cy0 - 0.0060, "每 15 秒完成单数", size=5.6, color=FAINT)

        # 最近完成
        ly = cy0 - 0.0200
        b.text(ix, ly, "最近完成", size=6.8, weight="bold", color=NAVY)
        b.text(ix + 0.0480, ly, "RECENT COMPLETIONS", size=4.9, color=FAINT)
        ly -= 0.0125
        if not done:
            b.text(ix, ly, "暂无已完成任务", size=6.6, color=FAINT)
        for t in done[:3]:
            mat = MAT_CN.get(t.material_type, t.material_type or "—")
            rid = int(t.robot_id)
            b.text(ix, ly, f"#{t.task_id:04d}", size=6.3, family=FONT_MONO, color=INK)
            b.text(ix + 0.0345, ly, mat, size=6.3, color=INK)
            b.text(ix + 0.0505, ly, f"{_station_cn(t.source_name)} → {_station_cn(t.dest_name)}",
                   size=6.3, color=MUTED)
            b.text(ix + iw, ly, "—" if rid < 0 else f"AGV-{rid:02d}", size=6.3,
                   family=FONT_MONO, color=INK, ha="right")
            ly -= 0.0125

    def _throughput_buckets(self, window_s, bucket_s):
        """把 (时刻, 累计完成) 采样切成最近 window_s 的完成量柱。"""
        if len(self.hist) < 3:
            return [], bucket_s
        t_end = self.hist[-1][0]
        t0 = t_end - window_s
        n = int(window_s / bucket_s)
        buckets = [0] * n
        prev_t, prev_c = None, None
        for t, c in self.hist:
            if prev_t is not None and t >= t0:
                idx = min(n - 1, max(0, int((t - t0) / bucket_s)))
                buckets[idx] += max(0, c - prev_c)
            prev_t, prev_c = t, c
        return buckets, bucket_s

    # ---------- 页脚 ----------
    def _footer(self, b):
        b.hline(0.013, 0.987, 0.0355, color=RULE, lw=0.7)
        b.text(0.013, 0.0215, "数据源 /factory/summary · /factory/machines · /fleet/robots · "
                              "/factory/task_status", size=6.2, color=FAINT, family=FONT_MONO)
        every = f"{self.every_s:.1f} s" if self.every_s > 0 else "手动 / 单次"
        b.text(0.987, 0.0215,
               f"刷新周期 {every} · 第 {self.frames + 1:04d} 帧 · 生成 {time.strftime('%Y-%m-%d %H:%M:%S')}"
               f" · 仿真数据 SIMULATED DATA",
               size=6.2, color=FAINT, family=FONT_MONO, ha="right")


# ---------------------------------------------------------------- 入口
def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog="dashboard", description="棉纺车间生产调度看板渲染（--demo 可离线出图）")
    parser.add_argument("--demo", action="store_true",
                        help="用合成数据渲染一张看板 PNG，不启动 ROS 图（供离线评审设计）")
    parser.add_argument("--out", default=None,
                        help="--demo 模式的输出 PNG 路径（默认 <out_dir>/dashboard.png）")
    parser.add_argument("--size", default="1600x1000", help="输出像素尺寸，默认 1600x1000")
    args, _ros_argv = parser.parse_known_args(argv)   # ROS 参数原样透传，不干扰

    size = None
    if args.size:
        try:
            w_px, h_px = (int(v) for v in args.size.lower().split("x"))
            size = (w_px, h_px)
        except Exception:
            print(f"[dashboard] --size 解析失败：{args.size!r}，回退到 1600x1000", file=sys.stderr)

    layout.bootstrap()      # 先定布局，再建节点
    rclpy.init()
    node = Dashboard()
    try:
        if args.demo:
            node.load_demo()
            out = args.out or os.path.join(node.out_dir, "dashboard.png")
            path = node.render(auto=True, path=out, size=size)
            print(f"[dashboard] demo frame -> {path}")
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
