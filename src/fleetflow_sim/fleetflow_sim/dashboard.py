"""控制中心仪表盘：把车队与产线状态渲染成一张态势图。

无头运行（Agg 后端），可以：
* 每 N 秒把当前态势写成 PNG（用于 README 截图或定时存档）
* 也可手动按一次 `--once` 出图

左栏是统计与机器利用率，右栏是工厂地图与实时车队位置。
配色与 README 的深色控制中心风格一致。
"""
from __future__ import annotations

import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow, Rectangle

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fleetflow_interfaces.msg import MachineState, RobotStatus, TransportTask

from . import layout

BG = "#0b1016"
PANEL = "#111a23"
LINE = "#1e2b38"
FG = "#e8eef5"
MUTED = "#7d8f9e"
ACCENT = "#ff6a1a"
CYAN = "#2fd4c4"
COLOR_RGB = dict(
    empty=(0.62, 0.66, 0.70), green=(0.20, 0.72, 0.36),
    yellow=(0.95, 0.76, 0.20), red=(0.85, 0.24, 0.28),
)


class Dashboard(Node):
    def __init__(self):
        super().__init__("fleet_dashboard")
        self.declare_parameter("out_dir", "/tmp/fleetflow_frames")
        self.declare_parameter("every_s", 0.0)     # >0 则定时出图
        self.declare_parameter("once", False)
        self.declare_parameter("num_robots", 8)

        self.out_dir = self.get_parameter("out_dir").value
        self.robots: dict[int, RobotStatus] = {}
        self.machines: list[MachineState] = []
        self.summary = dict(materials=0, completed=0, by_color={}, pending=0, running=0, done=0, tasks_created=0)
        self.tasks: dict[int, TransportTask] = {}
        self.t0 = None
        self.frames = 0

        self.create_subscription(RobotStatus, "/fleet/robots", self.on_robot, 30)
        self.create_subscription(MachineState, "/factory/machines", self.on_machine, 30)
        self.create_subscription(String, "/factory/summary", self.on_summary, 10)
        self.create_subscription(TransportTask, "/factory/task_status", self.on_task, 30)

        every = float(self.get_parameter("every_s").value)
        if every > 0:
            self.create_timer(every, lambda: self.render(auto=True))
        elif bool(self.get_parameter("once").value):
            self.create_timer(6.0, lambda: (self.render(auto=True), rclpy.shutdown()))
        self.get_logger().info(f"dashboard up · out_dir={self.out_dir}")

    def on_robot(self, m: RobotStatus):
        self.robots[m.robot_id] = m

    def on_machine(self, m: MachineState):
        self.machines = [x for x in self.machines if x.name != m.name] + [m]

    def on_task(self, m: TransportTask):
        self.tasks[m.task_id] = m

    def on_summary(self, m: String):
        try:
            self.summary = json.loads(m.data)
        except Exception:
            pass

    # ---------- 绘图 ----------
    def render(self, auto=False):
        import os
        import time
        os.makedirs(self.out_dir, exist_ok=True)
        fig = plt.figure(figsize=(15.2, 8.4), dpi=110, facecolor=BG)
        gs = fig.add_gridspec(1, 2, width_ratios=[1, 2.35], left=0.02, right=0.985,
                              top=0.90, bottom=0.05, wspace=0.045)
        self._header(fig)
        self._left(fig.add_subplot(gs[0, 0]))
        self._map(fig.add_subplot(gs[0, 1]))
        self.frames += 1
        name = os.path.join(self.out_dir, f"control_center_{self.frames:04d}.png")
        fig.savefig(name, facecolor=BG)
        plt.close(fig)
        self.get_logger().info(f"frame -> {name}")

    def _header(self, fig):
        fig.text(0.024, 0.955, "FLEETFLOW", color=FG, fontsize=20, fontweight="bold", va="center")
        fig.text(0.024, 0.9175, "CONTROL CENTER", color=ACCENT, fontsize=11,
                 fontweight="bold", va="center")
        fig.text(0.155, 0.9265, "ROS 2 JAZZY · GAZEBO SIM 8 · MULTI-AGV MATERIAL TRANSPORT",
                 color=MUTED, fontsize=8.2, va="center")
        live = self.summary.get("running", 0) > 0 or len(self.robots) > 0
        dot = "●" if live else "○"
        fig.text(0.975, 0.955, f"{dot}  FLEET {len(self.robots)} ONLINE", color=CYAN,
                 fontsize=10, ha="right", va="center", fontweight="bold")
        fig.text(0.975, 0.917, "fabrics line · carding → drawing 1 → drawing 2",
                 color=MUTED, fontsize=8.4, ha="right", va="center")
        fig.add_artist(plt.Line2D([0.024, 0.985], [0.895, 0.895], color=LINE, lw=1.1))

    def _panel(self, ax):
        ax.set_facecolor(PANEL)
        for s in ax.spines.values():
            s.set_color(LINE)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)

    def _left(self, ax):
        self._panel(ax)
        s = self.summary
        bc = s.get("by_color", {}) or {}
        tiles = [
            ("COMPLETED", s.get("completed", 0), ACCENT),
            ("IN TRANSIT", s.get("running", 0), CYAN),
            ("QUEUED", s.get("pending", 0), FG),
            ("ON FLOOR", s.get("materials", 0), FG),
        ]
        for i, (label, val, col) in enumerate(tiles):
            x = 0.05 + (i % 2) * 0.48
            y = 0.86 - (i // 2) * 0.115
            ax.text(x, y, str(val), color=col, fontsize=22, fontweight="bold", va="center")
            ax.text(x, y - 0.043, label, color=MUTED, fontsize=8, va="center")

        ax.text(0.05, 0.60, "MATERIAL PIPELINE", color=CYAN, fontsize=8.6, fontweight="bold")
        order = ["empty", "green", "yellow", "red"]
        labels = ["EMPTY", "GREEN", "YELLOW", "RED"]
        total = max(1, sum(bc.get(k, 0) for k in order))
        x = 0.05
        for k, lab in zip(order, labels):
            n = bc.get(k, 0)
            w = 0.9 * (n / total) if n else 0.012
            ax.add_patch(Rectangle((x, 0.515), max(w, 0.012), 0.042,
                                   facecolor=COLOR_RGB[k], edgecolor="none"))
            ax.text(x, 0.487, f"{lab} {n}", color=MUTED, fontsize=7.2)
            x += max(w, 0.012) + 0.012

        ax.text(0.05, 0.40, "MACHINE UTILISATION", color=CYAN, fontsize=8.6, fontweight="bold")
        stages = {}
        for m in self.machines:
            stages.setdefault(m.stage, []).append(m.busy_ratio)
        stage_color = {s["name"]: s["rgb"] for s in layout.STAGES}
        for i, (stage, vals) in enumerate(sorted(stages.items())):
            r = sum(vals) / len(vals)
            y = 0.335 - i * 0.062
            ax.text(0.05, y, stage.upper(), color=FG, fontsize=8)
            ax.add_patch(Rectangle((0.40, y - 0.012), 0.53, 0.024, facecolor=LINE, edgecolor="none"))
            ax.add_patch(Rectangle((0.40, y - 0.012), 0.53 * r, 0.024,
                                   facecolor=stage_color.get(stage, ACCENT), edgecolor="none"))
            ax.text(0.955, y, f"{r*100:4.0f}%", color=MUTED, fontsize=7.6, ha="right")

        ax.text(0.05, 0.115, "TASKS ASSIGNED", color=CYAN, fontsize=8.6, fontweight="bold")
        ax.text(0.05, 0.052, f"{s.get('tasks_created',0)} created   ·   "
                             f"{s.get('done',0)} delivered", color=FG, fontsize=9)

    def _map(self, ax):
        self._panel(ax)
        ax.set_aspect("equal")
        ax.set_xlim(layout.FIELD["x_min"] - 1.3, layout.FIELD["x_max"] + 1.3)
        ax.set_ylim(layout.FIELD["y_min"] - 0.8, layout.FIELD["y_max"] + 0.8)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title("FACTORY MAP  ·  1:1 SCALE  ·  16 × 10 m", color=FG, fontsize=10.5, pad=7)

        ax.add_patch(Rectangle((-0.5, 0), 17, 10, facecolor="#f2f4f6",
                               edgecolor="#c8cfd6", lw=1.4, zorder=0))
        # 工序流向引导线：空料区 → 梳棉 → 拉伸1 → 拉伸2 → 成品区
        flow_y = 5.0
        ax.annotate("", xy=(13.2, flow_y), xytext=(3.0, flow_y),
                    arrowprops=dict(arrowstyle="-|>", color="#9fb0bd", lw=1.4,
                                    linestyle=(0, (7, 5))), zorder=1)
        for x, lab in ((6.0, "CARDING"), (9.0, "DRAWING 1"), (11.8, "DRAWING 2")):
            ax.text(x, flow_y - 0.46, lab, color="#8d99a6", fontsize=6.4, ha="center", zorder=2)
        # 料区
        for k, z in layout.STORAGE.items():
            ax.add_patch(Circle((z["x"], z["y"]), z["r"], facecolor=z["rgb"],
                                edgecolor="none", alpha=0.95, zorder=1))
            ax.text(z["x"], z["y"], k.upper(), color="#20262c", fontsize=7.2,
                    ha="center", va="center", zorder=4, fontweight="bold")
        # 工位 + 机器
        for s in layout.STAGES:
            for i, y in enumerate(s["lanes"]):
                st = layout.station(s["name"], "waiting", i)
                fin = layout.station(s["name"], "finished", i)
                ax.add_patch(Circle((st["x"], y), 0.30, facecolor=s["rgb"], alpha=0.30,
                                    edgecolor=s["rgb"], lw=0.9, zorder=2))
                ax.add_patch(Circle((fin["x"], y), 0.30, facecolor=s["rgb"], alpha=0.55,
                                    edgecolor=s["rgb"], lw=0.9, zorder=2))
                ax.add_patch(Rectangle((s["machine_x"] - 0.22, y - 0.22), 0.44, 0.44,
                                       facecolor=s["rgb"], edgecolor="#20262c", lw=0.8, zorder=3))
        # 在途任务路线：车 → 取货点 → 卸货点，直观表达"调度"
        for rid, r in sorted(self.robots.items()):
            if r.task_id is None or r.task_id < 0:
                continue
            t = self.tasks.get(r.task_id)
            if t is None or t.status not in ("assigned", "running"):
                continue
            target = (t.source_x, t.source_y) if r.state == "to_pickup" else (t.dest_x, t.dest_y)
            ax.plot([r.x, target[0]], [r.y, target[1]], color=ACCENT, lw=1.1,
                    alpha=0.75, zorder=5, linestyle=(0, (4, 3)))
            ax.add_patch(Circle(target, 0.17, facecolor="none", edgecolor=ACCENT,
                                lw=1.2, zorder=5))
        # 车队
        state_color = dict(idle="#8d99a6", to_pickup=CYAN, to_dropoff=ACCENT,
                           loading="#ffd166", unloading="#ffd166")
        for rid, r in sorted(self.robots.items()):
            c = state_color.get(r.state, FG)
            ax.add_patch(Circle((r.x, r.y), 0.22, facecolor=c, edgecolor=BG, lw=1.0, zorder=6))
            ax.add_patch(FancyArrow(r.x, r.y, 0.34 * __import__("math").cos(r.yaw),
                                    0.34 * __import__("math").sin(r.yaw),
                                    width=0.045, head_width=0.17, head_length=0.15,
                                    color=c, zorder=7, length_includes_head=True))
            ax.text(r.x + 0.30, r.y + 0.26, f"AGV{rid}", color="#20262c", fontsize=6.8,
                    ha="left", va="bottom", zorder=8, fontweight="bold")
        ax.text(0.5, -0.055, "● idle    ● to pickup    ● to drop-off    ● handling        "
                             "dashed = active transport route",
                transform=ax.transAxes, color=MUTED, fontsize=7.6, ha="center")


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
