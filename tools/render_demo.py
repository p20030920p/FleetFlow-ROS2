#!/usr/bin/env python3
"""把 ``tools/record_run.py`` 录下来的运行过程渲染成动图（README 首页用）。

    python3 tools/record_run.py /tmp/demo.jsonl --fps 10 --seconds 95     # 录
    python3 tools/render_demo.py /tmp/demo.jsonl assets/readme/demo.gif   # 渲

动图展示的是一次完整过程：空车间 → 任务下发（空心方框）→ 派车（三角车沿路径行驶、
连线指向目的地）→ 卸货完成（闪烁后留下计数）→ 产线物料逐级流转。

渲染是离屏的，所以可以画得比 Gazebo 截图干净得多，也可以任意加速：
``--speed 5`` 表示 95 秒的运行压成 19 秒播放。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "fleetflow_sim"))
from fleetflow_sim import layout as L  # noqa: E402

# ---------------------------------------------------------------- 配色
BG = "#f4f2ed"
PANEL = "#fbfaf7"
INK = "#1c1c1a"
MUTED = "#6d6a63"
RULE = "#c9c4b8"
GREEN = "#2e7d4f"
AMBER = "#e8a33d"
BLUE = "#3d5a80"
RED = "#c0392b"
PURPLE = "#6b4f8a"

FLEET = ["#1f3a5f", "#2e7d4f", "#c0392b", "#e8a33d", "#6b4f8a", "#0f7b8a",
         "#8a5a2b", "#4a4a48"]

MAT = {"empty": ("#9a958c", "空筒"), "green": (GREEN, "生条"),
       "yellow": (AMBER, "熟条"), "red": (RED, "粗纱成品")}
STAGE_CN = {"carding": "梳棉", "drawing": "并条", "roving": "粗纱"}
STAGE_EN = {"carding": "CARDING", "drawing": "DRAWING", "roving": "ROVING"}

CJK = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]

# 画布与地图区域（像素）
CW, CH = 1240, 680
MAP = dict(l=44, r=884, b=74, t=596)


def w2p(x, y):
    """世界坐标 (m) -> 画布像素。"""
    sx = (MAP["r"] - MAP["l"]) / L.BUILDING["w"]
    sy = (MAP["t"] - MAP["b"]) / L.BUILDING["h"]
    s = min(sx, sy)
    ox = MAP["l"] + ((MAP["r"] - MAP["l"]) - s * L.BUILDING["w"]) / 2
    oy = MAP["b"] + ((MAP["t"] - MAP["b"]) - s * L.BUILDING["h"]) / 2
    return ox + x * s, oy + y * s, s


def hexc(c):
    """layout.py 里的颜色是 0-1 三元组，统一转成 #rrggbb。"""
    if isinstance(c, str):
        return c
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(v * 255))) for v in c[:3])


def mix(c, f):
    """把颜色朝白色混合 f（0=原色, 1=白）。"""
    c = hexc(c).lstrip("#")
    r, g, b = (int(c[i:i + 2], 16) for i in (0, 2, 4))
    return f"#{int(r + (255 - r) * f):02x}{int(g + (255 - g) * f):02x}{int(b + (255 - b) * f):02x}"


def draw_floor(ax):
    """车间底图：地面、机器、货架、停靠位、分区标注。只画一次。"""
    _, _, s = w2p(0, 0)
    bx, by, _ = w2p(0, 0)
    W, H = L.BUILDING["w"], L.BUILDING["h"]
    ax.add_patch(Rectangle((bx, by), W * s, H * s, facecolor="#ffffff",
                           edgecolor="none", zorder=1))
    # 分区底色：按工序把 x 切成三段
    for x0, x1, col in ((0.0, 11.2, "#eef2ee"), (11.2, 17.7, "#eceff4"),
                        (17.7, 26.0, "#f1eef4")):
        px, py, _ = w2p(x0, 0)
        ax.add_patch(Rectangle((px, py), (x1 - x0) * s, H * s, facecolor=col,
                               edgecolor="none", zorder=2))
    for x in range(1, 26, 2):                      # 柱网
        px, py, _ = w2p(x, 0)
        ax.plot([px, px], [py, py + H * s], color=RULE, lw=0.4, alpha=0.5, zorder=3)
    # 机台
    for name, rect, h, rgb, stage, lane in L.all_machines():
        x0, y0, x1, y1 = rect
        px, py, _ = w2p(x0, y0)
        ax.add_patch(Rectangle((px, py), (x1 - x0) * s, (y1 - y0) * s,
                               facecolor=mix(rgb, 0.34), edgecolor=hexc(rgb), lw=1.3,
                               zorder=5))
    # 货架
    for zone, (x0, y0, x1, y1) in L.rack_rects().items():
        px, py, _ = w2p(x0, y0)
        ax.add_patch(Rectangle((px, py), (x1 - x0) * s, (y1 - y0) * s,
                               facecolor="#e6e2d8", edgecolor=MUTED, lw=0.9,
                               hatch="////", zorder=4))
    # 停靠位
    for x, y in L.all_station_points().values():
        px, py, _ = w2p(x, y)
        ax.add_patch(Rectangle((px - 2.6, py - 2.6), 5.2, 5.2, facecolor="#ffffff",
                               edgecolor=MUTED, lw=0.7, zorder=6))
    # 充电位
    for spec in (L.CHARGERS if isinstance(L.CHARGERS, list) else L.CHARGERS.values()):
        cx, cy = spec if isinstance(spec, tuple) else (spec["x"], spec["y"])
        px, py, _ = w2p(cx, cy)
        ax.add_patch(Rectangle((px - 3.4, py - 3.4), 6.8, 6.8, facecolor="#dce9f2",
                               edgecolor=BLUE, lw=0.9, zorder=6))
    # 厂房轮廓最后画（zorder 高），否则会被分区底色盖掉
    ax.add_patch(Rectangle((bx, by), W * s, H * s, facecolor="none",
                           edgecolor="#8d8880", lw=1.8, zorder=12))
    # 待命区与充电位：车会停到这里，不标出来读者会以为是"没有图例的地方"
    pk = L.PARK
    px0, py0, _ = w2p(pk["x0"] - 0.7, pk["y"] - 0.75)
    px1, py1, _ = w2p(pk["x0"] + pk["dx"] * (pk["n"] - 1) + 0.7, pk["y"] + 0.75)
    ax.add_patch(Rectangle((px0, py0), px1 - px0, py1 - py0, facecolor="#efe9d8",
                           edgecolor="#b9ad86", lw=0.9, ls=(0, (3, 2)), zorder=4))
    ax.text((px0 + px1) / 2, py0 - 7, "待命区 PARK", ha="center", va="top",
            fontsize=7.8, color="#8a7d52", zorder=8)
    for name, spec in (("充电位 CHARGER", None),):
        pass
    cx0, cy0, _ = w2p(3.6, pk["y"] - 0.75)
    cx1, cy1, _ = w2p(7.3, pk["y"] + 0.75)
    ax.add_patch(Rectangle((cx0, cy0), cx1 - cx0, cy1 - cy0, facecolor="#e2edf5",
                           edgecolor="#8bb0c9", lw=0.9, ls=(0, (3, 2)), zorder=4))
    ax.text((cx0 + cx1) / 2, cy0 - 7, "充电位 CHARGER", ha="center", va="top",
            fontsize=7.8, color="#4d7794", zorder=8)

    # 分区标注
    for stage, x in (("carding", 7.0), ("drawing", 13.6), ("roving", 18.7)):
        px, py, _ = w2p(x, L.BUILDING["h"] - 0.9)
        ax.text(px, py, f"{STAGE_CN[stage]}区  {STAGE_EN[stage]}", ha="center",
                va="center", fontsize=8.6, color=MUTED, zorder=8)
    for zone, cn in (("empty", "空筒库"), ("red", "成品库")):
        z = L.STORAGE[zone]
        px, py, _ = w2p(z["x"], 0.9)
        ax.text(px, py, cn, ha="center", va="center", fontsize=8.6, color=MUTED,
                zorder=8, rotation=90)


def short_station(name: str) -> str:
    """把 carding_waiting_2 这类内部名压成现场看得懂的短标签。"""
    if name.startswith("storage_empty"):
        return "空筒库"
    if name.startswith("storage_red"):
        return "成品库"
    for stage, cn in (("carding", "梳棉"), ("drawing", "并条"), ("roving", "粗纱")):
        if name.startswith(stage):
            tail = name.rsplit("_", 1)[-1]
            return f"{cn}{tail}"
    return name


def draw_task(ax, t, tt, alpha):
    """待办任务 = 取货位空心方框 + 一条淡虚线示意去向。

    在途任务不在这里画：它的路线由 :func:`draw_route` 按**真实规划路径**绘制。
    早前版本用"取货点直连卸货点"的直线代替路线，会横穿机台，看起来像穿墙。
    """
    if alpha <= 0.02:
        return
    col, _ = MAT.get(t["material"], (MUTED, ""))
    sx, sy, _ = w2p(t["sx"], t["sy"])
    t_a = t.get("t_assigned")
    if t_a is not None and tt >= t_a:
        return
    dx, dy, _ = w2p(t["dx"], t["dy"])
    ax.plot([sx, dx], [sy, dy], color=col, lw=1.0, alpha=0.34 * alpha,
            zorder=7, ls=(0, (2, 3)))
    r = 4.6
    ax.add_patch(Rectangle((sx - r, sy - r), 2 * r, 2 * r, facecolor="none",
                           edgecolor=col, lw=1.6, alpha=alpha, zorder=9))


def draw_route(ax, trail, remaining, col, alpha):
    """已行驶轨迹（实线）+ 剩余规划路径（虚线），这才是车真正走/要走的路线。"""
    if len(trail) > 1:
        xs, ys = zip(*trail)
        ax.plot(xs, ys, color=col, lw=1.8, alpha=0.42 * alpha, zorder=7,
                solid_capstyle="round")
    if len(remaining) > 1:
        xs, ys = zip(*remaining)
        ax.plot(xs, ys, color=col, lw=1.9, alpha=0.85 * alpha, zorder=8,
                ls=(0, (4, 3)), solid_capstyle="round")
    if remaining:
        gx, gy = remaining[-1]
        ax.add_patch(Circle((gx, gy), 5.0, facecolor="none", edgecolor=col,
                            lw=1.7, alpha=alpha, zorder=9))
    if trail:
        ax.add_patch(Circle(trail[0], 3.6, facecolor=col, edgecolor="none",
                            alpha=0.8 * alpha, zorder=9))


def text_w(txt: str, fs: float) -> float:
    """粗略估算文字像素宽：CJK 约等于字号，拉丁约 0.55 倍。

    之前用 ``len(label) * 常数``，中文被当成半角算，图例全挤在一起。
    """
    px = fs * 100.0 / 72.0
    return sum(1.0 if ord(c) > 0x2E80 else 0.55 for c in txt) * px


def draw_legend(ax):
    """底部通栏图例：说明三角/方框/连线/停靠位/货架各代表什么。"""
    y = 34
    ax.plot([0, CW], [70, 70], color=RULE, lw=1.0)
    x = 22.0
    ax.text(x, y + 14, "图例 LEGEND", fontsize=8.2, fontweight="bold", color=MUTED,
            va="center")
    x += 96

    def item(draw, label, w):
        nonlocal x
        draw(x)
        ax.text(x + w + 8, y, label, fontsize=8.2, color=INK, va="center")
        x += w + 8 + text_w(label, 8.2) + 22

    def tri(px):
        ax.add_patch(Polygon([(px + 9, y + 7), (px, y + 12), (px, y + 2)],
                             closed=True, facecolor=BLUE, edgecolor=INK, lw=0.8))
    def square(px):
        ax.add_patch(Rectangle((px + 1, y + 1), 10, 10, facecolor="none",
                               edgecolor=AMBER, lw=1.6))
    def route(px):
        ax.plot([px, px + 13], [y + 6, y + 6], color=GREEN, lw=2.0)
        ax.plot([px + 13, px + 27], [y + 6, y + 6], color=GREEN, lw=2.0,
                ls=(0, (3, 2)))
        ax.add_patch(Polygon([(px + 32, y + 6), (px + 24, y + 10), (px + 24, y + 2)],
                             closed=True, facecolor=GREEN, edgecolor="none"))
    def dock(px):
        ax.add_patch(Rectangle((px + 3, y + 3), 7, 7, facecolor="#ffffff",
                               edgecolor=MUTED, lw=0.9))
    def rack(px):
        ax.add_patch(Rectangle((px, y + 1), 13, 10, facecolor="#e6e2d8",
                               edgecolor=MUTED, lw=0.9, hatch="////"))

    item(tri, "AGV（编号见车旁）", 12)
    item(square, "待办任务", 14)
    item(route, "在途路线：实线=已行驶，虚线=剩余规划", 35)
    item(dock, "工位停靠位", 13)
    item(rack, "条筒货架", 15)
    ax.text(CW - 22, y, "颜色 = 物料：空筒 / 生条 / 熟条 / 粗纱成品",
            fontsize=8.2, color=MUTED, ha="right", va="center")


def draw_robot(ax, rid, x, y, yaw, s, trail):
    px, py, _ = w2p(x, y)
    col = FLEET[rid % len(FLEET)]
    if len(trail) > 1:
        xs, ys = zip(*[(w2p(a, b)[0], w2p(a, b)[1]) for a, b in trail])
        ax.plot(xs, ys, color=col, lw=1.4, alpha=0.28, zorder=10,
                solid_capstyle="round")
    for i, k in enumerate((1.0, 0.66, 0.34)):        # 三层箭头，最外层做描边
        tri = [(10.0 * k, 0), (-6.2 * k, 6.6 * k), (-6.2 * k, -6.6 * k)]
        pts = [(px + a * math.cos(yaw) - b * math.sin(yaw),
                py + a * math.sin(yaw) + b * math.cos(yaw)) for a, b in tri]
        ax.add_patch(Polygon(pts, closed=True,
                             facecolor=INK if i == 0 else col,
                             edgecolor="none", zorder=11 + i))
    ax.text(px, py - 15, f"AGV{rid}", ha="center", va="center", fontsize=7.6,
            color=col, fontweight="bold", zorder=15,
            bbox=dict(boxstyle="round,pad=0.12", facecolor=PANEL, edgecolor="none",
                      alpha=0.85))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("out")
    ap.add_argument("--speed", type=float, default=5.0, help="播放倍速")
    ap.add_argument("--fps", type=float, default=12.5)
    ap.add_argument("--max-frames", type=int, default=260)
    ap.add_argument("--from", dest="t_from", type=float, default=None,
                    help="只渲染该时刻之后（秒，按录制的时间轴）")
    ap.add_argument("--to", dest="t_to", type=float, default=None,
                    help="只渲染该时刻之前")
    ap.add_argument("--still-at", type=float, default=None,
                    help="不给 GIF，只导出该时刻（仿真秒）的单帧 PNG")
    ap.add_argument("--still-scale", type=float, default=1.0,
                    help="静图缩放，>1 更清晰")
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.jsonl, encoding="utf-8")]
    # 有仿真时钟就用仿真时间（Gazebo 模式实时因子远小于 1，墙钟会骗人）
    key = "sim" if rows[0].get("sim") is not None else "t"
    if args.t_from is not None or args.t_to is not None:
        lo = args.t_from if args.t_from is not None else rows[0][key]
        hi = args.t_to if args.t_to is not None else rows[-1][key]
        # 任务清单要保留窗口之前就存在的任务，否则画面里会凭空冒出方框
        keep = [r for r in rows if lo <= r[key] <= hi]
        if len(keep) < 2:
            print("窗口内没有帧"); return 2
        rows = keep
    t0, t1 = rows[0][key], rows[-1][key]
    dur = (t1 - t0) / args.speed
    if args.still_at is not None:
        n, step = 1, 0.0
        t0 = t1 = args.still_at
    else:
        n = min(args.max_frames, max(2, int(dur * args.fps)))
        step = (t1 - t0) / n
    # 任务清单取"最终版"，状态随时间重建
    tasks = rows[-1]["tasks"]
    t_first = rows[0]["t"]

    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": CJK,
                         "axes.unicode_minus": False, "text.color": INK})
    frames = []
    for k in range(n):
        tt = t0 + k * step
        idx = min(len(rows) - 1,
                  int((tt - rows[0][key]) / max(1e-6, rows[1][key] - rows[0][key])))
        rec = rows[idx]
        sc = max(1.0, args.still_scale) if args.still_at is not None else 1.0
        fig = plt.figure(figsize=(CW * sc / 100, CH * sc / 100), dpi=100)
        if sc != 1.0:                       # 单帧静图提高分辨率：整体等比放大
            ax_scale = sc
        else:
            ax_scale = 1.0
        fig.patch.set_facecolor(BG)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, CW)
        ax.set_ylim(0, CH)
        if ax_scale != 1.0:
            # 放大画布时同步放大字号，否则字会显得很小
            for t in list(ax.texts):
                t.set_fontsize(t.get_fontsize() * ax_scale)
        ax.axis("off")

        # ---- 顶栏 ----
        ax.add_patch(Rectangle((0, CH - 62), CW, 62, facecolor=PANEL,
                               edgecolor="none"))
        ax.plot([0, CW], [CH - 62, CH - 62], color=RULE, lw=1.0)
        ax.text(20, CH - 27, "棉纺车间 · 多 AGV 物料搬运全过程", fontsize=15.5,
                fontweight="bold", color=INK, va="center")
        ax.text(20, CH - 48, "FLEETFLOW-ROS2 · MATERIAL TRANSPORT, DISPATCH TO DELIVERY",
                fontsize=7.4, color=MUTED, va="center")
        done = sum(1 for t in tasks if t.get("t_done") is not None
                   and tt >= t["t_done"])
        pub = sum(1 for t in tasks if tt >= t["t_created"])
        run = sum(1 for t in tasks if tt >= t.get("t_assigned", 1e9)
                  and tt < t.get("t_done", 1e9))
        pen = pub - run - done
        for i, (lab, val, col) in enumerate((("待分配", pen, MUTED),
                                             ("在途", run, AMBER),
                                             ("已完成", done, GREEN))):
            x = CW - 300 + i * 100
            ax.text(x, CH - 24, f"{val:>3d}", fontsize=17, fontweight="bold",
                    family="monospace", color=col, ha="center", va="center")
            ax.text(x, CH - 45, lab, fontsize=8.2, color=MUTED, ha="center",
                    va="center")

        draw_floor(ax)
        draw_legend(ax)

        # ---- 任务 ----
        for t in tasks:
            if tt < t["t_created"]:
                continue
            td = t.get("t_done")
            a = 1.0 if td is None else max(0.0, 1.0 - (tt - td) / 2.5)
            if td is not None and tt >= td and a <= 0.02:
                continue
            draw_task(ax, t, tt, a if td is None or tt >= td else 1.0)

        # ---- 车队 ----
        paths = rec.get("paths", {})
        for r in rec["robots"]:
            rid, tid = r[0], r[5]
            # 当前任务的起点：向前回溯到任务号发生变化的下一帧
            j = idx
            if tid != -1:
                while j > 0:
                    prev = next((z for z in rows[j - 1]["robots"] if z[0] == rid), None)
                    if prev is None or prev[5] != tid:
                        break
                    j -= 1
            trail = [(z[1], z[2]) for k in range(j, idx + 1)
                     for z in rows[k]["robots"] if z[0] == rid]
            trail = [w2p(x, y)[:2] for x, y in trail]
            rem = [w2p(x, y)[:2] for x, y in paths.get(str(rid), [])]
            col = FLEET[rid % len(FLEET)]
            if tid != -1:
                draw_route(ax, trail, rem, col, 1.0)
            short = [(w2p(z[1], z[2])[0], w2p(z[1], z[2])[1])
                     for z in rows[max(0, idx - 20):idx + 1]["robots"]
                     if z[0] == rid] if False else None
            tail = [(w2p(z[1], z[2])[0], w2p(z[1], z[2])[1])
                    for k in range(max(0, idx - 20), idx + 1)
                    for z in rows[k]["robots"] if z[0] == rid]
            draw_robot(ax, rid, r[1], r[2], r[3], None, [] if tid != -1 else tail)

        # ---- 右栏 ----
        px0, px1 = 906, CW - 20
        ax.add_patch(Rectangle((px0, MAP["b"]), px1 - px0,
                               MAP["t"] - MAP["b"], facecolor=PANEL,
                               edgecolor=RULE, lw=1.0))
        ax.text(px0 + 16, MAP["t"] - 24, "物料流转", fontsize=11.5,
                fontweight="bold", color=INK, va="center")
        ax.text(px0 + 16, MAP["t"] - 42, "MATERIAL FLOW", fontsize=7.0,
                color=MUTED, va="center")
        yy = MAP["t"] - 76
        for key in ("empty", "green", "yellow", "red"):
            col, cn = MAT[key]
            n_k = sum(1 for t in tasks if t["material"] == key
                      and tt >= t["t_created"])
            ax.add_patch(Rectangle((px0 + 16, yy - 8), 14, 14, facecolor=col,
                                   edgecolor="none"))
            ax.text(px0 + 40, yy, cn, fontsize=9.4, color=INK, va="center")
            ax.text(px1 - 16, yy, f"{n_k}", fontsize=11, family="monospace",
                    fontweight="bold", color=col, ha="right", va="center")
            yy -= 26
        ax.plot([px0 + 16, px1 - 16], [yy + 12, yy + 12], color=RULE, lw=0.9)
        yy -= 16
        ax.text(px0 + 16, yy, "车队状态", fontsize=11.5, fontweight="bold",
                color=INK, va="center")
        ax.text(px0 + 16, yy - 18, "FLEET", fontsize=7.0, color=MUTED, va="center")
        yy -= 38
        for r in rec["robots"]:
            rid, st, bat = r[0], r[4], r[6]
            col = FLEET[rid % len(FLEET)]
            ax.add_patch(Polygon([(px0 + 20, yy + 4), (px0 + 30, yy + 9),
                                  (px0 + 20, yy + 14)], closed=True,
                                 facecolor=col, edgecolor="none"))
            ax.text(px0 + 38, yy + 9, f"AGV{rid}", fontsize=8.6, color=INK,
                    va="center")
            ax.text(px1 - 16, yy + 9, f"{bat:4.0f}%", fontsize=8.6,
                    family="monospace", color=MUTED, ha="right", va="center")
            yy -= 19
        # ---- 在途任务清单 ----
        if yy > MAP["b"] + 78:
            ax.plot([px0 + 16, px1 - 16], [yy + 6, yy + 6], color=RULE, lw=0.9)
            ax.text(px0 + 16, yy - 12, "在途任务", fontsize=10.4, fontweight="bold",
                    color=INK, va="center")
            ax.text(px0 + 74, yy - 12, "IN TRANSIT", fontsize=6.6, color=MUTED,
                    va="center")
            rows_t = [t for t in tasks
                      if tt >= t.get("t_assigned", 1e9) and tt < t.get("t_done", 1e9)]
            rows_t.sort(key=lambda z: z["t_assigned"])
            ry = yy - 32
            for t in rows_t[:4]:
                col, cn = MAT.get(t["material"], (MUTED, ""))
                ax.add_patch(Rectangle((px0 + 18, ry - 5), 9, 10, facecolor=col,
                                       edgecolor="none"))
                ax.text(px0 + 33, ry, f"T{t['id']}", fontsize=7.4,
                        family="monospace", color=MUTED, va="center")
                ax.text(px0 + 62, ry, f"{short_station(t['src'])}→{short_station(t['dst'])}",
                        fontsize=7.8, color=INK, va="center")
                ax.text(px1 - 16, ry, f"AGV{t.get('robot', 0)}", fontsize=7.4,
                        family="monospace", color=col, ha="right", va="center")
                ry -= 17
            if not rows_t:
                ax.text(px0 + 18, ry, "—", fontsize=8.0, color=RULE, va="center")

        # ---- 时间轴 ----
        frac = (tt - t0) / max(1e-6, t1 - t0)
        ax.add_patch(Rectangle((px0 + 16, MAP["b"] + 18), px1 - px0 - 32, 8,
                               facecolor="#e4e0d6", edgecolor="none"))
        ax.add_patch(Rectangle((px0 + 16, MAP["b"] + 18), (px1 - px0 - 32) * frac,
                               8, facecolor=GREEN, edgecolor="none"))
        ax.text(px0 + 16, MAP["b"] + 38, f"运行进度  {frac * 100:3.0f}%",
                fontsize=8.4, color=MUTED, va="center")

        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(buf).convert("RGB"))
        plt.close(fig)
        if k % 25 == 0:
            print(f"  frame {k}/{n}", flush=True)

    if args.still_at is not None:
        sc = max(1.0, args.still_scale)
        frames[0].resize((int(CW * sc), int(CH * sc))).save(args.out)
        print(f"wrote {args.out} · still frame at {args.still_at:g}s")
        return 0
    frames += [frames[-1]] * int(args.fps * 1.6)      # 末帧停一下，方便看清最终计数
    pal = frames[len(frames) // 2].quantize(colors=128, method=Image.MEDIANCUT)
    frames = [f.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for f in frames]
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / args.fps), loop=0, optimize=True,
                   disposal=1)
    mb = Path(args.out).stat().st_size / 1e6
    print(f"wrote {args.out} · {len(frames)} frames · {dur:.1f}s · {mb:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
