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
# 未登记的类型 -> 兜底配色，避免渲染时 KeyError。
# 实测踩过：录制窗口内如果没有任何物料走到最后一道工序，
# 就不会出现 "red" 这个类型，而渲染器假定四种都在，直接崩在 MAT[key]。
MAT_FALLBACK = ("#6d6a63", "其他")
_seen_unknown: set[str] = set()


def mat_of(key: str):
    """取 (颜色, 中文名)，未登记的类型给兜底色并只提示一次。"""
    if key in MAT:
        return MAT[key]
    if key not in _seen_unknown:
        _seen_unknown.add(key)
        print(f"[render_demo] 未登记的物料类型 {key!r}，用兜底色渲染")
    return MAT_FALLBACK
STAGE_CN = {"carding": "梳棉", "drawing": "并条", "roving": "粗纱"}
STAGE_EN = {"carding": "CARDING", "drawing": "DRAWING", "roving": "ROVING"}

CJK = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]

# 画布与地图区域（像素）
#
# 版面是**算出来的，不是试出来的**：厂房 120×60 m 是 2:1，地图框就取 2:1
# （920×460），上边紧贴顶栏、下边接图例，中间不留横向空带；右栏取够放两列
# 车队列表的宽度(264)。画布高度 = 顶栏 58 + 地图 460 + 图例 ~92 = 610。
#
# 上一版是 CW×CH = 1240×680、地图框 1.44:1，比厂房"方"：默认视图下厂房上下
# 各被切掉一大块，上次录制只好用 `--view 4,17,70,43` 放大局部 —— 结果东端的
# 成品库（x=106.6）整段落在视口外，"粗纱成品入库"这条腿在动图里从没出现过。
# 现在的原则：**默认视图就是整个厂房**，不靠裁视口换清晰度。
CW, CH = 1240, 610
TOPBAR_H = 58
MAP = dict(l=30, r=950, b=92, t=552)       # 920×460 = 2:1，与厂房同比例
PANEL_X0, PANEL_X1 = 964, CW - 12          # 右侧数据面板（264 宽）
LEG_RULE_Y = MAP["b"] - 30                 # 图例分隔线
LEG_ITEM_Y = MAP["b"] - 58                 # 图例文字基线


# 视口：要显示的世界矩形 (x0, y0, x1, y1)。默认整个厂房。
# 仍然保留 `--view`：出单帧静图（如 README 里的细节图）时放大局部很好用，
# 但**动图不要再裁**，否则又会把某一段流程拍到画外。
VIEW = {"x0": 0.0, "y0": 0.0, "x1": float(L.BUILDING["w"]), "y1": float(L.BUILDING["h"])}


def w2p(x, y):
    """世界坐标 (m) -> 画布像素（按 VIEW 视口缩放）。"""
    vw = max(1e-6, VIEW["x1"] - VIEW["x0"])
    vh = max(1e-6, VIEW["y1"] - VIEW["y0"])
    sx = (MAP["r"] - MAP["l"]) / vw
    sy = (MAP["t"] - MAP["b"]) / vh
    s = min(sx, sy)
    ox = MAP["l"] + ((MAP["r"] - MAP["l"]) - s * vw) / 2
    oy = MAP["b"] + ((MAP["t"] - MAP["b"]) - s * vh) / 2
    return ox + (x - VIEW["x0"]) * s, oy + (y - VIEW["y0"]) * s, s


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


def _stage_zones():
    """三个工序区的 x 边界，由各自的等料位/完工位推出。"""
    st = L.STAGES
    out = []
    for i, s in enumerate(st):
        left = s["wait_x"] - 1.00 if i == 0 else (s["wait_x"] + st[i - 1]["done_x"]) / 2.0
        right = ((st[i + 1]["wait_x"] + s["done_x"]) / 2.0 if i + 1 < len(st)
                 else s["done_x"] + 1.10)
        out.append((left, right))
    return out


def lane_band():
    """产线带的 y 范围 (band0, band1, band_mid)：平面图纵向排版全部由它推出。"""
    ys = [y for s in L.STAGES for y in s["lanes"]]
    return min(ys) - 2.6, max(ys) + 2.6, (min(ys) + max(ys)) / 2.0


def draw_floor(ax):
    """车间底图：地坪、柱网、分区、机台、货架、取放位、待命区、充电位。

    只画一次。**所有坐标都从 ``layout`` 推出，不写死世界坐标** ——
    上一版这里残留着小厂区（26×16 m）时代的三处写死值：
      * 工序区名画在 x=7.0 / 13.6 / 18.7（那是旧厂房的工序位置，
        新厂房里这三个点全在空筒库里，于是"梳棉区"标在货架上）；
      * 充电位方框画在 x=3.6~7.3（真实充电位在 x=60~68）；
      * 货架名画在 y=0.9（货架实际在 y=24.75~35.25）。
    用户看到的"整个页面都不对"就是这三处。
    """
    bx, by, s = w2p(0, 0)
    W, H = L.BUILDING["w"], L.BUILDING["h"]
    band0, band1, band_mid = lane_band()
    ax.add_patch(Rectangle((bx, by), W * s, H * s, facecolor="#ffffff",
                           edgecolor="none", zorder=1))
    # 柱网（按柱距铺满整跨，给空旷地坪一点尺度参照）
    for k in range(1, int(W // L.COLUMN_PITCH) + 1):
        gx = L.COLUMN_PITCH * k
        if gx >= W:
            break
        px, py, _ = w2p(gx, 0)
        ax.plot([px, px], [py, py + H * s], color=RULE, lw=0.4, alpha=0.55, zorder=3)
    for k in range(1, int(H // L.COLUMN_PITCH) + 1):
        gy = L.COLUMN_PITCH * k
        if gy >= H:
            break
        px, py, _ = w2p(0, gy)
        ax.plot([px, px + W * s], [py, py], color=RULE, lw=0.4, alpha=0.35, zorder=3)

    # 工序区：底色 + 区名 + 工位数，边界与名字都由布局推出
    for (zx0, zx1), st in zip(_stage_zones(), L.STAGES):
        rgb = L.MACHINE_SPEC[st["name"]]["rgb"]
        px, py, _ = w2p(zx0, band0)
        ax.add_patch(Rectangle((px, py), (zx1 - zx0) * s, (band1 - band0) * s,
                               facecolor=mix(rgb, 0.90), edgecolor=mix(rgb, 0.42),
                               lw=1.0, ls=(0, (5, 3)), zorder=2))
        ax.text(px + 5, py + (band1 - band0) * s - 9, f"{st['cn']}区", fontsize=9.0,
                fontweight="bold", color=hexc(rgb), va="center", zorder=8)
        ax.text(px + 5, py + (band1 - band0) * s - 22,
                f"{STAGE_EN[st['name']]} × {len(st['lanes'])}", fontsize=6.6,
                color=MUTED, va="center", zorder=8)

    # 机台
    for name, rect, h, rgb, stage, lane in L.all_machines():
        x0, y0, x1, y1 = rect
        px, py, _ = w2p(x0, y0)
        ax.add_patch(Rectangle((px, py), (x1 - x0) * s, (y1 - y0) * s,
                               facecolor=mix(rgb, 0.34), edgecolor=hexc(rgb), lw=1.3,
                               zorder=5))

    # 工序流向：空筒库 → 成品库，走产线带中线
    fx0, fy0, _ = w2p(L.STORAGE_SLOTS["empty"]["x"] - 1.0, band_mid)
    fx1, _, _ = w2p(L.STORAGE_SLOTS["red"]["x"] + 0.2, band_mid)
    ax.annotate("", xy=(fx1, fy0), xytext=(fx0, fy0),
                arrowprops=dict(arrowstyle="-|>", color="#b3ada0", lw=1.2,
                                linestyle=(0, (7, 4))), zorder=2)

    # 货架（空筒库 / 成品库）：位置与名字都来自 layout
    for zone, rect in L.rack_rects().items():
        x0, y0, x1, y1 = rect
        px, py, _ = w2p(x0, y0)
        ax.add_patch(Rectangle((px, py), (x1 - x0) * s, (y1 - y0) * s,
                               facecolor="#e6e2d8", edgecolor=MUTED, lw=0.9,
                               hatch="////", zorder=4))
        ax.text(px + (x1 - x0) * s / 2, py + (y1 - y0) * s + 8,
                L.STORAGE[zone]["cn"], ha="center", va="bottom", fontsize=8.8,
                color=INK, fontweight="bold", zorder=8)

    # 取放位（虚线小方框）：货架前的实际停靠点
    for pt in L.STORAGE_SLOT_POINTS.values():
        px, py, _ = w2p(pt["x"], pt["y"])
        ax.add_patch(Rectangle((px - 3.0, py - 3.0), 6.0, 6.0, facecolor="#ffffff",
                               edgecolor=MUTED, lw=0.7, ls=(0, (2, 1.6)), zorder=6))
    # 工位停靠位（等料位 / 完工位）
    for s_ in L.STAGES:
        for lane in range(len(s_["lanes"])):
            for kind in ("waiting", "finished"):
                st = L.station(s_["name"], kind, lane)
                px, py, _ = w2p(st["x"], st["y"])
                ax.add_patch(Rectangle((px - 2.6, py - 2.6), 5.2, 5.2,
                                       facecolor="#ffffff", edgecolor=MUTED, lw=0.7,
                                       zorder=6))
    # 通道里的临时障碍（车会绕开它们，画出来画面才说得通）
    for o in L.AISLE_OBSTACLES:
        px, py, _ = w2p(o["x"] - o["sx"] / 2, o["y"] - o["sy"] / 2)
        ax.add_patch(Rectangle((px, py), o["sx"] * s, o["sy"] * s,
                               facecolor="#c9c1a8", edgecolor="#8d8468", lw=0.8,
                               zorder=6))
    # 充电位（真实位置在 x=60/64/68）
    cxs = [c["x"] for c in L.CHARGERS.values()]
    cys = [c["y"] for c in L.CHARGERS.values()]
    cpx0, cpy0, _ = w2p(min(cxs) - 1.2, min(cys) - 1.2)
    cpx1, cpy1, _ = w2p(max(cxs) + 1.2, max(cys) + 1.2)
    ax.add_patch(Rectangle((cpx0, cpy0), cpx1 - cpx0, cpy1 - cpy0,
                           facecolor="#dce9f2", edgecolor=BLUE, lw=0.9,
                           ls=(0, (3, 2)), zorder=4))
    ax.text((cpx0 + cpx1) / 2, cpy0 - 6, "充电位 CHARGER", ha="center", va="top",
            fontsize=7.6, color="#4d7794", zorder=8)
    # 待命区
    pk = L.PARK
    px0, py0, _ = w2p(pk["x0"] - 0.75, pk["y"] - 0.75)
    px1, py1, _ = w2p(pk["x0"] + pk["dx"] * (pk["n"] - 1) + 0.75, pk["y"] + 0.75)
    ax.add_patch(Rectangle((px0, py0), px1 - px0, py1 - py0, facecolor="#efe9d8",
                           edgecolor="#b9ad86", lw=0.9, ls=(0, (3, 2)), zorder=4))
    ax.text((px0 + px1) / 2, py1 + 6, "待命区 PARK", ha="center", va="bottom",
            fontsize=7.6, color="#8a7d52", zorder=8)
    # 厂房轮廓最后画（zorder 高），否则会被分区底色盖掉
    ax.add_patch(Rectangle((bx, by), W * s, H * s, facecolor="none",
                           edgecolor="#8d8880", lw=1.8, zorder=12))


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
    y = LEG_ITEM_Y
    ax.plot([0, CW], [LEG_RULE_Y, LEG_RULE_Y], color=RULE, lw=1.0)
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

    # 三角标记是固定像素尺寸的"车辆标记"，不是真实占地 —— 明说一句，
    # 免得读者拿它和旁边的机台比大小。
    item(tri, "AGV（Ø0.5 m，在途时标编号）", 12)
    item(square, "待办任务", 14)
    item(route, "在途路线：实线=已行驶，虚线=剩余规划", 35)
    item(dock, "工位停靠位", 13)
    item(rack, "条筒货架", 15)
    ax.text(CW - 22, y, "颜色 = 物料：空筒 / 生条 / 熟条 / 粗纱成品",
            fontsize=8.2, color=MUTED, ha="right", va="center")


def draw_robot(ax, rid, x, y, yaw, trail, active, label_slots=None):
    """车 = 固定像素尺寸的三角箭头（+ 有任务时在车旁标编号）。

    箭头**不按世界尺度缩放**：车只有 Ø0.50 m，120 m 厂房下不到 4 px，
    按真实尺寸画就是几个看不见的点。固定尺寸 = 车间平面图上"车辆标记"的常规画法，
    尺度关系由布局本身（机台、通道、货架）体现。

    编号只在车**有任务**时画：10 台车停在待命区时彼此间距只有 25 px，
    而 "AGV10" 标签要 29 px 宽，全画出来就是一团压在车上的字。
    待命车的身份在右侧面板里本来就有（带颜色和电量）。

    在途的车也可能彼此很近（实测 AGV5/AGV6 同时停靠相邻工位时标签重叠 2 px），
    所以标签先试"车上方"，撞到已放置的标签就试"车下方"，都撞就不画 ——
    ``label_slots`` 由调用方按 rid 顺序传进来，保证同一台车每次的取舍一致。
    """
    px, py, _ = w2p(x, y)
    col = FLEET[rid % len(FLEET)]
    if len(trail) > 1:
        xs, ys = zip(*[(w2p(a, b)[0], w2p(a, b)[1]) for a, b in trail])
        ax.plot(xs, ys, color=col, lw=1.4, alpha=0.28, zorder=10,
                solid_capstyle="round")
    for i, k in enumerate((1.0, 0.66, 0.34)):        # 三层箭头，最外层做描边
        tri = [(11.0 * k, 0), (-6.8 * k, 7.2 * k), (-6.8 * k, -7.2 * k)]
        pts = [(px + a * math.cos(yaw) - b * math.sin(yaw),
                py + a * math.sin(yaw) + b * math.cos(yaw)) for a, b in tri]
        ax.add_patch(Polygon(pts, closed=True,
                             facecolor=INK if i == 0 else col,
                             edgecolor="none", zorder=11 + i))
    if not active:
        return
    txt = f"AGV{rid}"
    w = text_w(txt, 8.0) + 8.0
    for dy in (-16.0, 19.0):
        box = (px - w / 2, py + dy - 6.5, px + w / 2, py + dy + 6.5)
        if label_slots is not None and any(
                box[0] < s[2] and s[0] < box[2] and box[1] < s[3] and s[1] < box[3]
                for s in label_slots):
            continue
        if label_slots is not None:
            label_slots.append(box)
        ax.text(px, py + dy, txt, ha="center", va="center", fontsize=8.0,
                color=col, fontweight="bold", zorder=15,
                bbox=dict(boxstyle="round,pad=0.12", facecolor=PANEL,
                          edgecolor="none", alpha=0.85))
        return


def _report_bbox(ax, fig):
    """打印文字包围盒、越界文字、以及互相重叠的文字对。

    为什么需要：版面是"给眼睛看的"，而自动检查只能查几何数字。改字号/挪面板
    之后"有没有压字"没法靠数值断言 —— 用 matplotlib 真实的文字包围盒互相求交
    就可以。做规划图时踩过一次：10 台车停在待命区，编号标签彼此间距 25 px、
    而 "AGV10" 要 29 px 宽，整片糊在一起，肉眼看动图才发现。
    """
    ren = fig.canvas.get_renderer()
    items = []
    for t in ax.texts:
        s = t.get_text()
        if not s.strip():
            continue
        bb = t.get_window_extent(renderer=ren)
        items.append((s, bb))
    print(f"  [bbox] {len(items)} 个文字")
    for s, bb in items:
        if bb.x0 < -1 or bb.y0 < -1 or bb.x1 > CW + 1 or bb.y1 > CH + 1:
            print(f"    !! 越界 {s!r} bbox=({bb.x0:.0f},{bb.y0:.0f},"
                  f"{bb.x1:.0f},{bb.y1:.0f})")
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = items[i][1], items[j][1]
            ox = min(a.x1, b.x1) - max(a.x0, b.x0)
            oy = min(a.y1, b.y1) - max(a.y0, b.y0)
            if ox > 1.5 and oy > 1.5:
                print(f"    ~~ 重叠 {ox:.0f}x{oy:.0f}px: {items[i][0]!r} <-> "
                      f"{items[j][0]!r}")


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
    ap.add_argument("--view", default=None, metavar="x0,y0,x1,y1",
                    help="只渲染这个世界矩形（放大局部；默认整个厂房）")
    ap.add_argument("--still-scale", type=float, default=1.0,
                    help="静图缩放，>1 更清晰")
    ap.add_argument("--debug-bbox", action="store_true",
                    help="打印文字包围盒互相重叠的对（改版面时用来查压字）")
    args = ap.parse_args()

    if args.view:
        try:
            vx0, vy0, vx1, vy1 = (float(v) for v in args.view.split(","))
            VIEW.update(x0=vx0, y0=vy0, x1=vx1, y1=vy1)
        except ValueError:
            print(f"--view 需要 x0,y0,x1,y1，收到 {args.view!r}", file=sys.stderr)
            return 2

    rows = [json.loads(l) for l in open(args.jsonl, encoding="utf-8")]
    # 有仿真时钟就用仿真时间（Gazebo 模式实时因子远小于 1，墙钟会骗人）。
    #
    # 为什么单独取个 `tkey`：下面渲染循环里有一个
    # `for key in ("empty", "green", ...)` —— 它**复用了同一个变量名** `key`，
    # 于是 `rows[0][key]` 被覆盖成 `rows[0]["yellow"]`，直接 KeyError: 'yellow'。
    # 报错行在循环里、赋值行在循环外，隔了 30 行，看 traceback 很难一眼看出。
    # 时间键从此只叫 `tkey`，并把内层循环变量改名，避免再次同名覆盖。
    tkey = "sim" if rows[0].get("sim") is not None else "t"
    if tkey not in rows[0]:
        raise SystemExit(f"录制文件没有时间键 {tkey!r}（有 {sorted(rows[0])}）")
    if args.t_from is not None or args.t_to is not None:
        lo = args.t_from if args.t_from is not None else rows[0][tkey]
        hi = args.t_to if args.t_to is not None else rows[-1][tkey]
        # 任务清单要保留窗口之前就存在的任务，否则画面里会凭空冒出方框
        keep = [r for r in rows if lo <= r[tkey] <= hi]
        if len(keep) < 2:
            print("窗口内没有帧"); return 2
        rows = keep
    t0, t1 = rows[0][tkey], rows[-1][tkey]
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
                  int((tt - rows[0][tkey]) / max(1e-6, rows[1][tkey] - rows[0][tkey])))
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
        ax.add_patch(Rectangle((0, CH - TOPBAR_H), CW, TOPBAR_H, facecolor=PANEL,
                               edgecolor="none"))
        ax.plot([0, CW], [CH - TOPBAR_H, CH - TOPBAR_H], color=RULE, lw=1.0)
        ax.text(20, CH - 24, "棉纺车间 · 多 AGV 物料搬运全过程", fontsize=15.5,
                fontweight="bold", color=INK, va="center")
        ax.text(20, CH - 44, "FLEETFLOW-ROS2 · MATERIAL TRANSPORT, DISPATCH TO DELIVERY",
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
            # KPI 从右往左排，并给右侧面板标题留出足够间距：
            # 原来 x 从 CW-300 起、步长 100，第三个数字会顶到面板标题上。
            x = CW - 88 - (2 - i) * 92
            ax.text(x, CH - 22, f"{val:>3d}", fontsize=17, fontweight="bold",
                    family="monospace", color=col, ha="center", va="center")
            ax.text(x, CH - 43, lab, fontsize=8.2, color=MUTED, ha="center",
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
        label_slots: list[tuple[float, float, float, float]] = []
        # 按 rid 顺序画，标签避让的取舍才与遍历顺序无关（否则同一台车的标签
        # 会在相邻帧之间上下跳）
        for r in sorted(rec["robots"], key=lambda z: z[0]):
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
            # 待命车不再画尾迹：这里原来把 **已经换算成像素** 的 `tail` 传进
            # draw_robot，而它内部又做了一次 w2p —— 双重投影，尾迹画在错误位置。
            # 待命车本来就不动，尾迹没有信息量，直接不画。
            draw_robot(ax, rid, r[1], r[2], r[3], [], active=tid != -1,
                       label_slots=label_slots)

        # ---- 右栏 ----
        px0, px1 = PANEL_X0, PANEL_X1
        ax.add_patch(Rectangle((px0, MAP["b"]), px1 - px0,
                               MAP["t"] - MAP["b"], facecolor=PANEL,
                               edgecolor=RULE, lw=1.0))
        ax.text(px0 + 16, MAP["t"] - 24, "物料流转", fontsize=11.5,
                fontweight="bold", color=INK, va="center")
        ax.text(px0 + 16, MAP["t"] - 42, "MATERIAL FLOW", fontsize=7.0,
                color=MUTED, va="center")
        yy = MAP["t"] - 76
        # 只画**本次运行真的出现过**的类型，加上始终存在的空筒。
        # 顺序固定，保证同一类型在不同录制里的颜色与位置一致。
        present = {t["material"] for t in tasks}
        for mkey in [k for k in ("empty", "green", "yellow", "red")
                     if k == "empty" or k in present]:
            col, cn = mat_of(mkey)
            n_k = sum(1 for t in tasks if t["material"] == mkey
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
        yy -= 40
        # 车队列表**分两列**：10 台车单列要 190 px，会把下面的"在途任务"
        # 整段挤出面板（原来 10 台车时那一段是直接被 if 判掉、根本不显示的，
        # 而在途任务是这张图上信息量最大的一块）。
        rob = rec["robots"]
        per_col = max(1, (len(rob) + 1) // 2)
        col_w = (px1 - px0 - 32) / 2.0
        for i, r in enumerate(rob):
            c, row = divmod(i, per_col)
            cx = px0 + 18 + c * col_w
            ry = yy - row * 19
            rid, bat = r[0], r[6]
            col = FLEET[rid % len(FLEET)]
            ax.add_patch(Polygon([(cx, ry + 4), (cx + 9, ry + 9),
                                  (cx, ry + 14)], closed=True,
                                 facecolor=col, edgecolor="none"))
            ax.text(cx + 14, ry + 9, f"AGV{rid}", fontsize=8.4, color=INK,
                    va="center")
            bx = cx + col_w - 10
            ax.text(bx, ry + 9, f"{bat:3.0f}%", fontsize=8.4, family="monospace",
                    color=MUTED, ha="right", va="center")
        yy -= per_col * 19 + 6
        # ---- 在途任务清单 ----
        if yy > MAP["b"] + 96:
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
                ax.text(px0 + 32, ry, f"T{t['id']}", fontsize=7.4,
                        family="monospace", color=MUTED, va="center")
                ax.text(px0 + 58, ry, f"{short_station(t['src'])}→{short_station(t['dst'])}",
                        fontsize=7.6, color=INK, va="center")
                ax.text(px1 - 16, ry, f"AGV{t.get('robot', 0)}", fontsize=7.4,
                        family="monospace", color=FLEET[t.get("robot", 0) % len(FLEET)],
                        ha="right", va="center")
                ry -= 17
            if not rows_t:
                ax.text(px0 + 18, ry, "—", fontsize=8.0, color=RULE, va="center")

        # ---- 时间轴（单行：进度条 + 百分比文字，省出纵向空间给在途任务） ----
        frac = (tt - t0) / max(1e-6, t1 - t0)
        bar_y = MAP["b"] + 14
        ax.add_patch(Rectangle((px0 + 16, bar_y), px1 - px0 - 110, 8,
                               facecolor="#e4e0d6", edgecolor="none"))
        ax.add_patch(Rectangle((px0 + 16, bar_y), (px1 - px0 - 110) * frac,
                               8, facecolor=GREEN, edgecolor="none"))
        ax.text(px1 - 16, bar_y + 4, f"进度 {frac * 100:3.0f}%", fontsize=8.4,
                color=MUTED, ha="right", va="center")

        fig.canvas.draw()
        if args.debug_bbox:
            _report_bbox(ax, fig)
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
