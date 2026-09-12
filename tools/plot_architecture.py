#!/usr/bin/env python3
"""画出 FleetFlow-ROS2 的运行时拓扑图（README 用）。

    python3 tools/plot_architecture.py assets/readme/architecture.png

图里的节点名、命名空间和话题名与 ``launch/factory.launch.py`` 及各节点源码一致，
不是示意图：``factory_manager`` 派单、``task_scheduler`` 拍卖、``traffic_manager``
发租约、``robot_controller`` 在各自命名空间里闭环，``ros_gz_bridge`` 是唯一的
仿真边界。改代码时这张图也要跟着改。
"""
from __future__ import annotations

import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

BG = "#f4f2ed"
INK = "#1c1c1a"
MUTED = "#6d6a63"
RULE = "#c9c4b8"
GREEN = "#2e7d4f"
AMBER = "#e8a33d"
BLUE = "#3d5a80"
RED = "#c0392b"
PURPLE = "#6b4f8a"

CJK = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]


def node(ax, x, y, w, h, title, sub="", color=BLUE, fs=10.0):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.06",
                                facecolor="#ffffff", edgecolor=color, linewidth=1.8,
                                zorder=3))
    if sub:
        ax.text(x, y + h * 0.17, title, ha="center", va="center", fontsize=fs,
                fontweight="bold", color=color, zorder=4)
        ax.text(x, y - h * 0.22, sub, ha="center", va="center", fontsize=fs - 2.4,
                color=MUTED, zorder=4)
    else:
        ax.text(x, y, title, ha="center", va="center", fontsize=fs,
                fontweight="bold", color=color, zorder=4)


def arrow(ax, p, q, label="", color=INK, rad=0.0, ls="-", lw=1.5, off=0.0,
          fs=8.0, side="top"):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=13,
                                 color=color, lw=lw, linestyle=ls,
                                 connectionstyle=f"arc3,rad={rad}", zorder=2))
    if label:
        mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
        if side == "top":
            my += off
        else:
            mx += off
        ax.text(mx, my, label, ha="center", va="center", fontsize=fs, color=color,
                zorder=5,
                bbox=dict(boxstyle="round,pad=0.16", facecolor=BG, edgecolor="none"))


def main() -> int:
    dst = sys.argv[1] if len(sys.argv) > 1 else "assets/readme/architecture.png"
    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": CJK,
        "axes.unicode_minus": False, "text.color": INK,
    })
    fig, ax = plt.subplots(figsize=(16.0, 8.4), dpi=100)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, 20.0)
    ax.set_ylim(0, 10.6)
    ax.axis("off")

    ax.text(0.25, 10.30, "FleetFlow-ROS2 运行时拓扑", fontsize=17,
            fontweight="bold", color=INK, ha="left", va="top")
    ax.text(0.25, 9.86, "实线 = 话题；虚线 = 服务。每个机器人一个命名空间，"
                        "TF 是互不相交的森林，而不是一条被争用的链。",
            fontsize=10.0, color=MUTED, ha="left", va="top")

    # ---- 产线层 ----
    ax.add_patch(FancyBboxPatch((0.25, 7.55), 8.6, 1.95,
                                boxstyle="round,pad=0.04,rounding_size=0.08",
                                facecolor="#ffffff", edgecolor=RULE, linewidth=1.0,
                                zorder=1))
    ax.text(0.55, 9.28, "产线模型 · factory_manager", fontsize=11.6,
            fontweight="bold", color=GREEN, ha="left", va="top", zorder=4)
    for i, (nm, cn) in enumerate([("carding ×12", "梳棉"),
                                  ("drawing ×4", "并条"),
                                  ("roving ×4", "粗纱")]):
        node(ax, 1.85 + i * 2.35, 8.28, 2.05, 0.90, nm, cn, GREEN, fs=9.4)
    ax.text(8.55, 8.28, "机台状态\n队列 / 忙闲 / 完工", fontsize=8.6, color=MUTED,
            ha="center", va="center", linespacing=1.6, zorder=4)

    # ---- 调度层 ----
    node(ax, 3.35, 6.35, 4.2, 1.30, "task_scheduler", "五种策略 · 拍卖 / 就近 / 随机", BLUE, 11.0)
    node(ax, 8.55, 6.35, 4.2, 1.30, "traffic_manager", "工位租约 · TTL 回收", RED, 11.0)
    node(ax, 13.75, 6.35, 4.2, 1.30, "metrics_recorder", "tasks.csv · run.csv", PURPLE, 11.0)
    node(ax, 18.20, 6.35, 3.1, 1.30, "dashboard", "车间生产看板", AMBER, 11.0)

    # ---- 车队层 ----
    ax.add_patch(FancyBboxPatch((0.25, 2.55), 17.6, 2.45,
                                boxstyle="round,pad=0.04,rounding_size=0.08",
                                facecolor="#ffffff", edgecolor=RULE, linewidth=1.0,
                                zorder=1))
    ax.text(17.75, 5.16, "车队 · robot_controller ×N（命名空间 /robot_i）",
            fontsize=11.6, fontweight="bold", color=BLUE, ha="right", va="top", zorder=4)
    for i in range(4):
        node(ax, 2.15 + i * 4.35, 3.55, 3.55, 1.35,
             f"robot_{i}", "A* + 纯跟踪 + 互避 + TF", BLUE, fs=10.0)
    ax.text(19.15, 3.55, "⋯", fontsize=20, color=MUTED, ha="center", va="center")

    # ---- 仿真层 ----
    node(ax, 4.35, 1.05, 6.3, 1.25, "ros_gz_bridge", "/clock · cmd_vel · odom · scan · 相机",
         AMBER, 11.0)
    node(ax, 13.25, 1.05, 8.0, 1.25, "Gazebo Sim 8", "物理 · 激光 · 相机 · 电量消费", GREEN, 11.0)

    # ---- 连线：产线 -> 调度 ----
    arrow(ax, (4.10, 7.53), (3.35, 7.02), "/factory/tasks", GREEN, off=0.18)
    arrow(ax, (8.88, 8.30), (13.05, 7.02), "/factory/machines", GREEN, off=0.22, fs=8.2)
    # ---- 调度 -> 车队 ----
    arrow(ax, (2.35, 5.68), (2.15, 4.28), "RequestTask", BLUE, off=-0.62, fs=8.2)
    arrow(ax, (6.20, 4.28), (5.05, 5.68), "/fleet/robots", PURPLE, off=-0.34, fs=8.2)
    arrow(ax, (7.20, 5.68), (6.55, 4.28), "AcquireLease", RED, off=-0.42,
          ls=(0, (5, 3)), fs=8.0)
    arrow(ax, (8.40, 4.28), (9.05, 5.68), "ReleaseLease", RED, off=0.40,
          ls=(0, (5, 3)), fs=8.0)
    # ---- 车队 -> 仿真 ----
    arrow(ax, (3.20, 2.86), (4.35, 1.72), "/robot_i/cmd_vel", BLUE, off=0.26, fs=8.2)
    arrow(ax, (5.75, 1.72), (4.95, 2.86), "/robot_i/odom · scan", GREEN, rad=-0.12,
          off=0.40, fs=8.2)
    arrow(ax, (7.55, 1.05), (9.20, 1.05), "", AMBER)
    # ---- 遥测 ----
    arrow(ax, (14.60, 4.28), (14.35, 5.68), "/fleet/robots", PURPLE, off=-0.30, fs=8.2)
    arrow(ax, (15.88, 6.35), (16.62, 6.35), "", PURPLE)

    ax.text(0.25, 0.26, "拉起全部：ros2 launch fleetflow_sim factory.launch.py"
                        "        只跑调度逻辑（实验台架）：ros2 launch fleetflow_sim logic_only.launch.py",
            fontsize=10.0, color=MUTED, ha="left", va="bottom")

    fig.subplots_adjust(left=0.008, right=0.992, top=0.99, bottom=0.01)
    fig.savefig(dst, facecolor=BG)
    print(f"wrote {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
