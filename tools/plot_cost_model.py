#!/usr/bin/env python3
"""画出 CA-SSI 代价函数的构成图（README 用）。

    python3 tools/plot_cost_model.py assets/readme/cost-model.png

左图是车间示意图，标注代价函数里出现的三个几何量（空驶、载货、工位拥塞）；
右图是六项代价的权重条，直观说明"哪一项在决策里说话最响"。
"""
from __future__ import annotations

import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

BG = "#f4f2ed"
INK = "#1c1c1a"
MUTED = "#6d6a63"
RULE = "#c9c4b8"
GREEN = "#2e7d4f"
AMBER = "#e8a33d"
BLUE = "#3d5a80"
RED = "#c0392b"

CJK = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]

# 与 policies.py 的权重保持一致
TERMS = [
    ("① 空驶  α·‖r→src‖",                1.00, "车空跑去取货位，纯浪费", BLUE),
    ("② 载货  β·‖src→dst‖",              0.55, "满载行驶，能耗与磨损更高", BLUE),
    ("③ 工位拥塞  γ·(n_src+1.5·n_dst)",  6.00, "取放料位同时被几台车盯上", RED),
    ("④ 电量可达  δ·(需求−余量)",         0.90, "这台车能不能把活干完", AMBER),
    ("⑤ 负载均衡  η·(里程−队均)",         0.30, "抑制快车越跑越多", AMBER),
    ("⑥ 任务老化  −ζ·等待时长",           0.35, "防止低优先级任务被饿死", GREEN),
]


def main() -> int:
    dst = sys.argv[1] if len(sys.argv) > 1 else "assets/readme/cost-model.png"
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": CJK,
        "axes.unicode_minus": False,
        "text.color": INK, "axes.labelcolor": INK,
        "xtick.color": INK, "ytick.color": INK,
        "mathtext.fontset": "dejavusans",
    })
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(15.2, 5.6), dpi=100,
                                 gridspec_kw=dict(width_ratios=[1.15, 1.0], wspace=0.16))
    fig.patch.set_facecolor(BG)

    # ---------------- 左：车间示意 ----------------
    ax.set_facecolor(BG)
    ax.set_xlim(0, 20.6)
    ax.set_ylim(0, 12.4)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Rectangle((0.35, 0.35), 19.9, 10.0, facecolor="#ffffff",
                           edgecolor=RULE, linewidth=1.0))
    for gx in [3.5, 6.5, 9.5, 12.5, 15.5, 18.5]:
        ax.plot([gx, gx], [0.35, 10.35], color=RULE, lw=0.4, alpha=0.5, zorder=0)
    for gy in [2.4, 4.4, 6.4, 8.4]:
        ax.plot([0.35, 20.25], [gy, gy], color=RULE, lw=0.4, alpha=0.5, zorder=0)

    ax.text(1.0, 10.05, "代价函数的几何含义", fontsize=14, fontweight="bold",
            color=INK, ha="left", va="top")
    ax.text(1.0, 9.42, "每台空闲车对每个待办任务投一个标，代价 = 六项之和（统一折成米）",
            fontsize=9.4, color=MUTED, ha="left", va="top")

    # 梳棉机 = 取货位
    ax.add_patch(Rectangle((8.3, 3.1), 3.4, 1.9, facecolor="#dfe6e1",
                           edgecolor=GREEN, lw=1.5))
    ax.text(10.0, 4.05, "梳棉机\nCARDING", ha="center", va="center", fontsize=8.8,
            color=GREEN, fontweight="bold", linespacing=1.5)
    # 粗纱机 = 放货位
    ax.add_patch(Rectangle((8.3, 6.9), 3.4, 1.9, facecolor="#e6e1ec",
                           edgecolor="#6b4f8a", lw=1.5))
    ax.text(10.0, 7.85, "粗纱机\nROVING", ha="center", va="center", fontsize=8.8,
            color="#6b4f8a", fontweight="bold", linespacing=1.5)

    robot = (2.3, 1.5)
    ax.add_patch(Circle(robot, 0.52, facecolor=BLUE, edgecolor=INK, lw=1.0, zorder=5))
    ax.text(2.3, 0.72, "空闲车 r\nidle AGV", ha="center", va="top", fontsize=8.8,
            color=INK, linespacing=1.5)

    def arrow(p, q, color, rad=0.0, lw=1.9, ls="-"):
        ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=15,
                                     color=color, lw=lw, linestyle=ls,
                                     connectionstyle=f"arc3,rad={rad}", zorder=4))

    arrow((2.7, 2.0), (8.2, 3.35), BLUE)
    ax.text(5.55, 3.05, "① 空驶  d_deadhead", color=BLUE, fontsize=9.6,
            fontweight="bold", ha="center", va="bottom")
    arrow((8.05, 5.05), (8.05, 6.85), BLUE, rad=-0.42, ls=(0, (5, 3)))
    ax.text(4.35, 5.95, "② 载货  d_laden", color=BLUE, fontsize=9.6,
            fontweight="bold", ha="center")

    # 两个工位的拥塞：在途车 + 排队
    for cy, lab, note in ((4.05, "取料位", "n_src = 2 台在途"),
                          (7.85, "放料位", "n_dst = 2，权重 ×1.5")):
        ax.add_patch(Rectangle((13.6, cy - 0.62), 1.5, 1.24, facecolor="#f6ddd6",
                               edgecolor=RED, lw=1.2))
        arrow((12.0, cy), (13.5, cy), RED)
        ax.text(15.45, cy, f"③ {lab}拥塞", color=RED, fontsize=9.4,
                fontweight="bold", ha="left", va="center")
        ax.text(15.45, cy - 1.02, note, color=RED, fontsize=8.5,
                ha="left", va="center")
    for k in range(2):
        ax.add_patch(Circle((14.35, 1.75 + k * 0.02), 0.34, facecolor=BLUE,
                            edgecolor=INK, lw=0.7, alpha=0.7))
        ax.add_patch(Circle((14.35 + k * 0.92, 1.75), 0.34, facecolor=BLUE,
                            edgecolor=INK, lw=0.7, alpha=0.7))
    ax.text(16.05, 1.75, "已在途中", color=RED, fontsize=8.5, va="center")

    # ---------------- 右：权重条 ----------------
    bx.set_facecolor(BG)
    names = [t[0] for t in TERMS]
    weights = [t[1] for t in TERMS]
    notes = [t[2] for t in TERMS]
    cols = [t[3] for t in TERMS]
    ys = list(range(len(TERMS)))[::-1]
    bx.barh(ys, weights, height=0.62, color=cols, edgecolor=INK, lw=0.8, alpha=0.92)
    bx.set_yticks(ys)
    bx.set_yticklabels(names, fontsize=10.4)
    for y, w in zip(ys, weights):
        bx.text(w + 0.20, y + 0.02, f"{w:.2f}", va="center", ha="left", fontsize=10.4,
                fontweight="bold", family="monospace", color=INK)
        bx.text(w + 1.05, y + 0.02, dict(zip(ys, notes))[y], va="center", ha="left",
                fontsize=8.5, color=MUTED)
    bx.set_xlim(0, 10.9)
    bx.set_ylim(-0.75, len(TERMS) - 0.25)
    bx.set_xlabel("权重（等效米 / 单位）", fontsize=9.8)
    bx.grid(axis="x", color=RULE, lw=0.6, alpha=0.7)
    bx.set_axisbelow(True)
    for sp in ("top", "right", "left"):
        bx.spines[sp].set_visible(False)
    bx.tick_params(axis="y", length=0)
    bx.set_title("权重配置 · weights", fontsize=14, fontweight="bold", color=INK,
                 pad=26, loc="left")
    bx.text(0.0, 1.035, "拥塞项权重最高：它是唯一会直接造成机台停待的一项",
            transform=bx.transAxes, fontsize=9.3, color=MUTED, va="bottom")

    fig.subplots_adjust(left=0.035, right=0.975, top=0.90, bottom=0.10)
    fig.savefig(dst, facecolor=BG)
    print(f"wrote {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
