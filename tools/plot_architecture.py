#!/usr/bin/env python3
"""画出 FleetFlow-ROS2 的运行时拓扑图（README 用）。

    python3 tools/plot_architecture.py assets/readme/architecture.png

图里的节点名、命名空间、话题名与服务名与 ``launch/factory.launch.py``
及各节点源码一致，不是示意图：

* ``factory_manager``  产线模型，发 ``/factory/tasks`` ``/factory/machines``
  ``/factory/summary`` ``/factory/task_status``，收 ``/factory/completed``
* ``task_scheduler``   五种策略的顺序拍卖，服务 ``/scheduler/request_task``
* ``traffic_manager``  工位租约，服务 ``/traffic/acquire`` ``/traffic/release``
* ``robot_controller`` 每个 ``/robot_i`` 命名空间一份，A* + 纯跟踪 + 互避
* ``metrics_recorder`` ``fleet_dashboard``  只订阅，不参与控制
* ``ros_gz_bridge``    唯一的仿真边界

排版约定：四层从上到下排开，每层一个带左侧标签栏的面板；箭头只在层与层
之间的"通道"里走，每个通道里一条走线一个 x 车道，标签按车道错行摆放，
标题栏、面板标题永远不会和箭头或标签撞上。脚本末尾会用渲染器实测每个
文本的像素外框，做一次自动碰撞检查（文本×文本、文本×方框、走线×文本），
有问题直接打印出来——改坐标之后不用靠眼睛猜。
"""
from __future__ import annotations

import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

# ---------------------------------------------------------------- 视觉语言
# 与 tools/render_demo.py、tools/plot_ablation.py 完全一致
BG = "#f4f2ed"
PANEL = "#fbfaf7"
NODE = "#ffffff"
INK = "#1c1c1a"
MUTED = "#6d6a63"
RULE = "#c9c4b8"
GREEN = "#2e7d4f"
AMBER = "#e8a33d"
BLUE = "#3d5a80"
RED = "#c0392b"
PURPLE = "#6b4f8a"

CJK = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]

W, H = 1600, 880

# 布局登记表，最后统一做碰撞检查
TEXTS: list = []      # (artist, label, inside_box_id)
BOXES: list = []      # (x0, y0, x1, y1, box_id)
SEGS: list = []       # (p, q, label)


def text_w(s, fs):
    """粗略估算文字像素宽：CJK 约等于字号，拉丁约 0.55 倍。

    和 ``render_demo.py`` 里同名函数一致——用它来顺序排布一行里的多段文字，
    不要用 ``len()``，中文会被当成半角，排出来就挤在一起。
    """
    px = fs * 100.0 / 72.0
    return sum(1.0 if ord(c) > 0x2E80 else 0.55 for c in s) * px


def band_rect(x0, y0, x1, y1, accent, cn, en, head_x):
    """一层：面板 + 左侧标签栏 + 竖向强调条。"""
    ax = plt.gca()
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=PANEL,
                           edgecolor=RULE, lw=1.0, zorder=1))
    ax.add_patch(Rectangle((x0, y0), 4.0, y1 - y0, facecolor=accent,
                           edgecolor="none", zorder=2))
    yc = (y0 + y1) / 2.0
    txt(head_x, yc + 9, cn, 13.0, color=INK, weight="bold")
    txt(head_x, yc - 11, en, 8.6, color=MUTED)


def panel_box(x0, y0, x1, y1, accent=BLUE, node_id="box", lw=1.6, fill=NODE,
              container=False):
    """只画一个直角方框，文字自己摆。"""
    plt.gca().add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=fill,
                                  edgecolor=accent, lw=lw, zorder=3))
    BOXES.append((x0, y0, x1, y1, node_id, container))
    return node_id


def node(x0, y0, x1, y1, title, sub="", accent=BLUE, node_id=None,
         fs=12.6, sub_fs=9.6, sub_color=MUTED, fill=NODE, lw=1.6):
    """一个节点方框：直角、细边、白底。没有圆角卡片，也没有阴影。"""
    nid = panel_box(x0, y0, x1, y1, accent, node_id or title, lw, fill)
    xc, yc = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    if sub:
        txt(xc, yc + 9.5, title, fs, color=accent, weight="bold", inside=nid)
        txt(xc, yc - 11.0, sub, sub_fs, color=sub_color, inside=nid)
    else:
        txt(xc, yc, title, fs, color=accent, weight="bold", inside=nid)
    return nid


def txt(x, y, s, fs, color=INK, weight="normal", ha="center", va="center",
        family=None, bg=False, inside=None, on_arrow=None, zorder=6):
    kw = dict(fontsize=fs, color=color, fontweight=weight, ha=ha, va=va,
              zorder=zorder)
    if family:
        kw["family"] = family
    if bg:
        kw["bbox"] = dict(boxstyle="square,pad=0.20", facecolor=BG,
                          edgecolor="none")
        kw["zorder"] = zorder + 1
    a = plt.gca().text(x, y, s, **kw)
    TEXTS.append((a, s, inside, on_arrow))
    return a


def seg(p, q, color, lw=1.6, ls="-", label=""):
    """只画线段，不带箭头（折线的前几段用）。"""
    plt.gca().plot([p[0], q[0]], [p[1], q[1]], color=color, lw=lw, ls=ls,
                   zorder=4, solid_capstyle="butt")
    SEGS.append((p, q, label))


def head(p, q, color, lw=1.6, ls="-", label="", ms=14):
    """带箭头的最后一段。"""
    plt.gca().add_patch(FancyArrowPatch(p, q, arrowstyle="-|>",
                                        mutation_scale=ms, color=color, lw=lw,
                                        linestyle=ls, shrinkA=0, shrinkB=0,
                                        zorder=4))
    SEGS.append((p, q, label))


# ---------------------------------------------------------------- 碰撞检查
def _seg_hits_rect(p, q, r):
    """线段与矩形是否相交（Liang-Barsky 裁剪）。"""
    x0, y0, x1, y1 = r
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for d, lo, hi, s in ((dx, x0, x1, p[0]), (dy, y0, y1, p[1])):
        if abs(d) < 1e-9:
            if s < lo or s > hi:
                return False
            continue
        a, b = (lo - s) / d, (hi - s) / d
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return False
    return True


def audit(fig, strict=True):
    """用渲染器实测文本外框，检查重叠并把结果打印出来。

    检查三类问题：文本互相重叠、文本溢出它所属的方框（或压到别的方框）、
    走线从文字的实测外框里穿过去。箭头压住自己的标签是正常的（标签带底色），
    这种情况跳过。
    """
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    rects = []
    for artist, label, inside, on_arrow in TEXTS:
        bb = artist.get_window_extent(rend)
        pad = 3.0 if artist.get_bbox_patch() is not None else 1.0
        rects.append(dict(x0=bb.x0 - pad, y0=bb.y0 - pad, x1=bb.x1 + pad,
                          y1=bb.y1 + pad, s=label, inside=inside,
                          arrow=on_arrow))
    box_at = {b[4]: b for b in BOXES}
    bad = []
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            a, b = rects[i], rects[j]
            ox = min(a["x1"], b["x1"]) - max(a["x0"], b["x0"])
            oy = min(a["y1"], b["y1"]) - max(a["y0"], b["y0"])
            if ox > 1.0 and oy > 1.0:
                bad.append(f"文本重叠 {a['s']!r} × {b['s']!r} "
                           f"({ox:.0f}×{oy:.0f}px)")
    for r in rects:
        own = box_at.get(r["inside"])
        if own is not None:                      # 必须待在自己的框里
            if (r["x0"] < own[0] - 0.5 or r["y0"] < own[1] - 0.5
                    or r["x1"] > own[2] + 0.5 or r["y1"] > own[3] + 0.5):
                bad.append(f"文本出框 {r['s']!r} 超出方框 {own[4]!r}")
        for bx0, by0, bx1, by1, nid, is_cont in BOXES:
            if nid == r["inside"] or is_cont:
                continue
            ox = min(bx1, r["x1"]) - max(bx0, r["x0"])
            oy = min(by1, r["y1"]) - max(by0, r["y0"])
            if ox > 1.0 and oy > 1.0:
                bad.append(f"文本压框 {r['s']!r} × 方框 {nid!r} "
                           f"({ox:.0f}×{oy:.0f}px)")
    for p, q, name in SEGS:
        for r in rects:
            if name and name == r["arrow"]:
                continue
            if _seg_hits_rect(p, q, (r["x0"], r["y0"], r["x1"], r["y1"])):
                bad.append(f"走线压字 {name or 'arrow'} × {r['s']!r}")
    for msg in sorted(set(bad)):
        print("  [!]", msg)
    if not bad:
        print("  排版检查：无重叠")
    return not bad


# ---------------------------------------------------------------- 主图
def main() -> int:
    dst = sys.argv[1] if len(sys.argv) > 1 else "assets/readme/architecture.png"
    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": CJK,
        "axes.unicode_minus": False, "text.color": INK,
    })
    fig = plt.figure(figsize=(W / 100.0, H / 100.0), dpi=100)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    BX0, BX1 = 176, 1560          # 面板左右边界
    CX0, CX1 = 312, 1544          # 面板内容区
    HEAD_X = 242                  # 左侧标签栏中心

    # ---------------- 页眉 ----------------
    txt(40, 852, "FleetFlow-ROS2 运行时拓扑", 25.0, weight="bold", ha="left",
        va="top")
    txt(40, 812, "实线 = 话题 · 虚线 = 服务 · 每台车一个独立命名空间 /robot_i · "
                 "各车 TF 从 map 分叉，互不争用", 12.0, color=MUTED,
        ha="left", va="top")
    txt(1560, 850, "节点名 / 话题名 / 服务名均取自 launch/factory.launch.py "
                   "与各节点源码", 11.0, color=MUTED, ha="right", va="top")
    ax.plot([40, 1560], [786, 786], color=RULE, lw=1.0)

    # ---------------- 第 1 层：产线模型 ----------------
    band_rect(BX0, 658, BX1, 768, GREEN, "产线模型", "PRODUCTION", HEAD_X)
    panel_box(CX0, 666, CX1, 756, GREEN, "fm", container=True)
    x = CX0 + 16
    txt(x, 748, "factory_manager", 13.0, color=GREEN, weight="bold",
        ha="left", va="top", inside="fm")
    x += text_w("factory_manager", 13.0) + 18
    txt(x, 745, "产线模型 · 8 台机台 · 12 个在制条筒", 9.6, color=MUTED,
        ha="left", va="top", inside="fm")
    txt(CX1 - 16, 745, "工序：空筒库 → 梳棉 → 并条 → 粗纱 → 成品库", 9.6,
        color=MUTED, ha="right", va="top", inside="fm")

    stages = [("空筒库", "EMPTY CANS", MUTED), ("梳棉 ×4", "CARDING", GREEN),
              ("并条 ×2", "DRAWING", BLUE), ("粗纱 ×2", "ROVING", PURPLE),
              ("粗纱成品库", "FINISHED", RED)]
    cw, gap = 208.0, 40.0
    cy0, cy1 = 673, 725
    for i, (cn, en, col) in enumerate(stages):
        x0 = CX0 + 16 + i * (cw + gap)
        panel_box(x0, cy0, x0 + cw, cy1, col, f"stage{i}", lw=1.4)
        txt(x0 + cw / 2, cy0 + 36, cn, 11.2, color=col, weight="bold",
            inside=f"stage{i}")
        txt(x0 + cw / 2, cy0 + 16, en, 8.6, color=MUTED, inside=f"stage{i}")
        if i < len(stages) - 1:
            head((x0 + cw + 6, 699), (x0 + cw + gap - 6, 699), RULE, lw=1.4,
                 ms=11, label=f"stage-flow{i}")

    # ---------------- 第 2 层：调度与观测 ----------------
    band_rect(BX0, 480, BX1, 586, BLUE, "调度与观测", "CONTROL", HEAD_X)
    b2y0, b2y1 = 492, 548
    c2w, c2gap = 270.0, 36.0
    n2 = [("task_scheduler", "五种策略 · 顺序拍卖", BLUE),
          ("traffic_manager", "工位租约 · TTL 回收", RED),
          ("metrics_recorder", "tasks.csv · run.csv", PURPLE),
          ("fleet_dashboard", "车间生产看板", AMBER)]
    for i, (nm, sub, col) in enumerate(n2):
        x0 = CX0 + i * (c2w + c2gap)
        node(x0, b2y0, x0 + c2w, b2y1, nm, sub, col, node_id=nm)
    # 说明行靠右放：左边三段竖线走线要一路下到方框，不能压字
    txt(CX1 - 74, 568, "观测节点只订阅、不参与控制 · /factory/summary 汇总 · "
                       "/fleet/robots 为广播话题", 9.6, color=MUTED,
        ha="right")

    # ---------------- 第 3 层：车队 ----------------
    band_rect(BX0, 284, BX1, 408, AMBER, "车队", "FLEET", HEAD_X)
    panel_box(CX0, 292, CX1, 404, BLUE, "fleet-ns", container=True)
    x = CX0 + 16
    txt(x, 396, "/robot_i ×N 命名空间", 11.6, color=BLUE, weight="bold",
        ha="left", va="top", inside="fleet-ns")
    x += text_w("/robot_i ×N 命名空间", 11.6) + 20
    txt(x, 393, "robot_controller + robot_state_publisher · "
                "A* 规划 / 纯跟踪 / 互避 / TF", 9.6, color=MUTED,
        ha="left", va="top", inside="fleet-ns")
    rw, rgap = 240.0, 36.0
    ry0, ry1 = 322, 374
    for i in range(4):
        x0 = CX0 + 36 + i * (rw + rgap)
        panel_box(x0, ry0, x0 + rw, ry1, BLUE, f"rc{i}", lw=1.3, fill=PANEL)
        txt(x0 + rw / 2, ry0 + 32, f"robot_{i}", 12.0, color=BLUE,
            weight="bold", inside=f"rc{i}")
        txt(x0 + rw / 2, ry0 + 14, f"AGV {i} · 独立 TF", 9.0, color=MUTED,
            inside=f"rc{i}")
    txt(CX0 + 36 + 3 * (rw + rgap) + rw + 24, 348, "⋯ robot_N", 11.5,
        color=MUTED, ha="left", inside="fleet-ns")
    txt(CX0 + 16, 308, "每车另发 /robot_i/path（剩余规划路径，供 RViz 与录制"
                       "回放）· 完工时上报 /factory/completed",
        9.6, color=MUTED, ha="left", inside="fleet-ns")

    # ---------------- 第 4 层：仿真 ----------------
    band_rect(BX0, 96, BX1, 212, MUTED, "仿真", "SIMULATION", HEAD_X)
    node(CX0, 108, 880, 182, "ros_gz_bridge", "/clock · 每车 5 路话题",
         AMBER, node_id="bridge", fs=13.0)
    node(940, 108, CX1, 182, "Gazebo Sim 8", "物理 · 激光 · 相机 · 电量消费",
         GREEN, node_id="gz", fs=13.0)
    seg((886, 152), (934, 152), AMBER, lw=1.6, label="gz-down")
    head((934, 138), (886, 138), GREEN, lw=1.6, label="gz-up")
    # 同样靠右：cmd_vel / odom 两条竖线要从上面下来
    txt(660, 200, "ros_gz_bridge 是唯一的仿真边界：桥接 /clock 与每车的 "
                  "cmd_vel · odom · scan · tf · joint_states", 9.6,
        color=MUTED, ha="left")

    # ---------------- 通道 1：产线 ↔ 调度 ----------------
    head((380, b2y1), (380, 666), GREEN, lw=1.7, label="task_status")
    txt(380, 622, "/factory/task_status", 11.5, color=GREEN, bg=True,
        on_arrow="task_status")
    head((520, 666), (520, b2y1), GREEN, lw=1.7, label="tasks")
    txt(520, 598, "/factory/tasks", 11.5, color=GREEN, bg=True,
        on_arrow="tasks")
    head((700, 666), (700, b2y1), GREEN, lw=1.7, label="machines")
    txt(700, 598, "/factory/machines", 11.5, color=GREEN, bg=True,
        on_arrow="machines")

    # ---------------- 通道 2：调度 ↔ 车队 ----------------
    head((400, 404), (400, b2y0), BLUE, lw=1.6, ls=(0, (5, 3)),
         label="request_task")
    txt(400, 444, "/scheduler/request_task", 11.5, color=BLUE, bg=True,
        on_arrow="request_task")
    head((700, 404), (700, b2y0), RED, lw=1.6, ls=(0, (5, 3)),
         label="traffic")
    txt(700, 444, "/traffic/acquire · release", 11.5, color=RED, bg=True,
        on_arrow="traffic")
    head((1300, 404), (1300, b2y0), PURPLE, lw=1.7, label="fleet")
    txt(1300, 444, "/fleet/robots", 11.5, color=PURPLE, bg=True,
        on_arrow="fleet")

    # ---------------- 通道 3：车队 ↔ 仿真 ----------------
    head((390, 292), (390, 182), BLUE, lw=1.7, label="cmd_vel")
    txt(390, 262, "/robot_i/cmd_vel", 11.5, color=BLUE, bg=True,
        on_arrow="cmd_vel")
    head((560, 182), (560, 292), GREEN, lw=1.7, label="odom")
    txt(560, 234, "/robot_i/odom · scan", 11.5, color=GREEN, bg=True,
        on_arrow="odom")

    # ---------------- 绕行通道：/factory/completed 完工回流 ----------------
    head((1520, 404), (1520, 666), RED, lw=1.7, ls=(0, (5, 3)),
         label="completed")
    txt(1506, 620, "/factory/completed", 11.5, color=RED, ha="right", bg=True,
        on_arrow="completed")
    txt(1506, 594, "完工回流：完工计数用于推进产线", 9.4, color=MUTED,
        ha="right")

    # ---------------- 页脚 ----------------
    ax.plot([40, 1560], [80, 80], color=RULE, lw=1.0)
    txt(40, 56, "拉起全部：ros2 launch fleetflow_sim factory.launch.py"
                "       只跑调度逻辑（实验台架）："
                "ros2 launch fleetflow_sim logic_only.launch.py",
        10.4, color=MUTED, ha="left")

    ok = audit(fig)
    fig.savefig(dst, facecolor=BG)
    print(f"wrote {dst}" + ("" if ok else "  (排版检查未通过)"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
