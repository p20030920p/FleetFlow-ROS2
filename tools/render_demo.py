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
    """画一个任务：待办=空心方框，在途=起点到终点的连线。"""
    if alpha <= 0.02:
        return
    col, _ = MAT.get(t["material"], (MUTED, ""))
    sx, sy, s = w2p(t["sx"], t["sy"])
    dx, dy, _ = w2p(t["dx"], t["dy"])
    t_a = t.get("t_assigned")
    running = t_a is not None and tt >= t_a
    if running:
        ax.plot([sx, dx], [sy, dy], color=col, lw=1.9, alpha=0.60 * alpha,
                zorder=7, solid_capstyle="round")
        # 终点画一个指向卸货位的箭头，一眼看出搬运方向
        ang = math.atan2(dy - sy, dx - sx)
        tip = (dx - 11 * math.cos(ang), dy - 11 * math.sin(ang))
        for sgn in (2.6, -2.6):
            ax.plot([tip[0], dx - 15 * math.cos(ang) + sgn * math.sin(ang)],
                    [tip[1], dy - 15 * math.sin(ang) - sgn * math.cos(ang)],
                    color=col, lw=1.7, alpha=0.75 * alpha, zorder=8,
                    solid_capstyle="round")
        ax.add_patch(Circle((dx, dy), 5.0, facecolor="none", edgecolor=col,
                            lw=1.7, alpha=alpha, zorder=9))
        ax.add_patch(Circle((sx, sy), 3.6, facecolor=col, edgecolor="none",
                            alpha=0.85 * alpha, zorder=9))
    else:
        r = 4.6
        ax.add_patch(Rectangle((sx - r, sy - r), 2 * r, 2 * r, facecolor="none",
                               edgecolor=col, lw=1.6, alpha=alpha, zorder=9))


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
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.jsonl, encoding="utf-8")]
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    dur = (t1 - t0) / args.speed
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
        idx = min(len(rows) - 1, int((tt - t_first) / max(1e-6, rows[1]["t"] - t_first)))
        rec = rows[idx]
        fig = plt.figure(figsize=(CW / 100, CH / 100), dpi=100)
        fig.patch.set_facecolor(BG)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, CW)
        ax.set_ylim(0, CH)
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
        trail_len = max(2, int(2.0 / max(1e-6, rows[1]["t"] - t_first)))
        for r in rec["robots"]:
            rid = r[0]
            trail = []
            for j in range(max(0, idx - trail_len), idx + 1):
                for rr in rows[j]["robots"]:
                    if rr[0] == rid:
                        trail.append((rr[1], rr[2]))
            draw_robot(ax, rid, r[1], r[2], r[3], None, trail)

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
