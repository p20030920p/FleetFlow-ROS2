#!/usr/bin/env python3
"""画出 CA-SSI 代价函数的构成图（README 用）。

    python3 tools/plot_cost_model.py assets/readme/cost-model.png

一张图讲两件事，左右两块共用同一套配色与编号：

* 左：代价的**几何含义**。底图就是 ``render_demo.py`` 里那张车间平面图
  （同样的分区底色、柱网、机台配色、货架、停靠位），上面摞一次真实搬运：
  待命车 r → 取料位 src → 放料位 dst，分别对应 α·d_deadhead 与 β·d_laden；
  两个料位旁边的在途车数就是 γ 项里的 n_src / n_dst。
* 右：六项**权重**。条长按 ``policies.py`` 里的常量画，标签统一左对齐，
  数值紧跟条尾，量纲说明放在每根条下面一行。

权重不是抄在图里的：脚本直接 import ``fleetflow_sim.policies`` 读常量，
读不到就退回正则解析源码，任何一边改了数，图跟着变。脚本末尾用渲染器
实测每个文本的外框做碰撞检查（文本×文本、文本×方框、走线×文本）。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "fleetflow_sim"))

from fleetflow_sim import layout as L  # noqa: E402

# ---------------------------------------------------------------- 视觉语言
# 与 tools/render_demo.py、tools/plot_ablation.py 完全一致
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

CJK = ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
MONO = ["DejaVu Sans Mono", "Noto Sans Mono", "monospace"]

W, H = 1600, 800


# ---------------------------------------------------------------- 读权重
def read_weights():
    """从 policies.py 读六个权重与 AGE_CAP。

    优先真的 import（图和代码不可能对不上）；import 失败——比如这台机器
    没装 scipy——就退回解析源码，同样是以文件为准，不是把数字抄在图里。
    """
    keys = ("W_DEADHEAD", "W_LADEN", "W_CONGEST", "W_ENERGY", "W_BALANCE",
            "W_AGE", "AGE_CAP", "BATTERY_RESERVE", "DRAIN_PER_M")
    try:
        from fleetflow_sim import policies as P
        return {k: float(getattr(P, k)) for k in keys}, "import"
    except Exception as exc:                                   # pragma: no cover
        src = (ROOT / "src" / "fleetflow_sim" / "fleetflow_sim"
               / "policies.py").read_text(encoding="utf-8")
        out = {}
        for k in keys:
            m = re.search(rf"^{k}\s*=\s*([0-9.]+)", src, re.M)
            if not m:
                raise SystemExit(f"policies.py 里找不到 {k}")
            out[k] = float(m.group(1))
        print(f"  (import policies 失败：{exc}；已改为解析源码)")
        return out, "regex"


# ---------------------------------------------------------------- 画布工具
TEXTS: list = []
BOXES: list = []
SEGS: list = []


def text_w(s, fs):
    """粗略估算文字像素宽：CJK 约等于字号，拉丁约 0.55 倍（同 render_demo）。"""
    px = fs * 100.0 / 72.0
    return sum(1.0 if ord(c) > 0x2E80 else 0.55 for c in s) * px


def text_w_mono(s, fs):
    """等宽字体的宽度：DejaVu Sans Mono 每字符 0.6 em。"""
    return len(s) * 0.6 * fs * 100.0 / 72.0


def txt(x, y, s, fs, color=INK, weight="normal", ha="center", va="center",
        family=None, bg=False, inside=None, on_arrow=None, zorder=8):
    kw = dict(fontsize=fs, color=color, fontweight=weight, ha=ha, va=va,
              zorder=zorder)
    if family:
        kw["family"] = family
    if bg:
        kw["bbox"] = dict(boxstyle="square,pad=0.18", facecolor=BG,
                          edgecolor="none")
        kw["zorder"] = zorder + 1
    TEXTS.append((plt.gca().text(x, y, s, **kw), s, inside, on_arrow))


def frame(x0, y0, x1, y1, name):
    plt.gca().add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=PANEL,
                                  edgecolor=RULE, lw=1.0, zorder=0))
    BOXES.append((x0, y0, x1, y1, name, True))


def rect(x0, y0, x1, y1, name, **kw):
    kw.setdefault("zorder", 3)
    plt.gca().add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **kw))
    BOXES.append((x0, y0, x1, y1, name, False))


def arrow(p, q, color, lw=2.2, ls="-", label="", ms=15, shrink=9.0):
    # shrink：箭头在料位圆点外沿收住，不糊在标记上
    plt.gca().add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", color=color,
                                        lw=lw, linestyle=ls, mutation_scale=ms,
                                        shrinkA=shrink, shrinkB=shrink,
                                        zorder=6))
    SEGS.append((p, q, label))


def leader(p, q, color=RULE, lw=1.0, label=""):
    plt.gca().plot([p[0], q[0]], [p[1], q[1]], color=color, lw=lw,
                   zorder=5, solid_capstyle="round")
    SEGS.append((p, q, label))


# ---------------------------------------------------------------- 碰撞检查
def _seg_hits_rect(p, q, r):
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


def audit(fig):
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
        if own is not None and (r["x0"] < own[0] - 0.5 or r["y0"] < own[1] - 0.5
                                or r["x1"] > own[2] + 0.5
                                or r["y1"] > own[3] + 0.5):
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
                bad.append(f"走线压字 {name or 'leader'} × {r['s']!r}")
    for msg in sorted(set(bad)):
        print("  [!]", msg)
    if not bad:
        print("  排版检查：无重叠")
    return not bad


# ---------------------------------------------------------------- 左图：车间
PLAN = dict(x0=86, y0=96, w=884.0, h=544.0)
SCALE = PLAN["w"] / L.BUILDING["w"]          # 34 px/m，与 26×16 m 正好整除


def wp(x, y):
    """世界坐标 (m) -> 画布像素。"""
    return PLAN["x0"] + x * SCALE, PLAN["y0"] + y * SCALE


def hexc(c):
    if isinstance(c, str):
        return c
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(v * 255))) for v in c[:3])


def mix(c, f):
    """把颜色朝白色混合 f（0=原色, 1=白）。"""
    c = hexc(c).lstrip("#")
    r, g, b = (int(c[i:i + 2], 16) for i in (0, 2, 4))
    return (f"#{int(r + (255 - r) * f):02x}{int(g + (255 - g) * f):02x}"
            f"{int(b + (255 - b) * f):02x}")


def draw_floor(ax):
    """车间底图：地面、分区、柱网、机台、货架、停靠位、待命区。只画一次。"""
    bw, bh = L.BUILDING["w"], L.BUILDING["h"]
    x0, y0 = wp(0, 0)
    ax.add_patch(Rectangle((x0, y0), bw * SCALE, bh * SCALE, facecolor="#ffffff",
                           edgecolor="none", zorder=1))
    for za, zb, col in ((0.0, 11.2, "#eef2ee"), (11.2, 17.7, "#eceff4"),
                        (17.7, 26.0, "#f1eef4")):
        px, _ = wp(za, 0)
        ax.add_patch(Rectangle((px, y0), (zb - za) * SCALE, bh * SCALE,
                               facecolor=col, edgecolor="none", zorder=2))
    for gx in range(1, 26, 2):
        px, _ = wp(gx, 0)
        ax.plot([px, px], [y0, y0 + bh * SCALE], color=RULE, lw=0.4, alpha=0.5,
                zorder=3)
    for mname, r, _h, rgb, stage, lane in L.all_machines():
        px, py = wp(r[0], r[1])
        ax.add_patch(Rectangle((px, py), (r[2] - r[0]) * SCALE,
                               (r[3] - r[1]) * SCALE, facecolor=mix(rgb, 0.34),
                               edgecolor=hexc(rgb), lw=1.2, zorder=5))
        BOXES.append((px, py, px + (r[2] - r[0]) * SCALE,
                      py + (r[3] - r[1]) * SCALE, f"m-{stage}-{lane}", False))
    for zone, r in L.rack_rects().items():
        px, py = wp(r[0], r[1])
        ax.add_patch(Rectangle((px, py), (r[2] - r[0]) * SCALE,
                               (r[3] - r[1]) * SCALE, facecolor="#e6e2d8",
                               edgecolor=MUTED, lw=0.9, hatch="////", zorder=4))
        BOXES.append((px, py, px + (r[2] - r[0]) * SCALE,
                      py + (r[3] - r[1]) * SCALE, f"rack-{zone}", False))
    for x, y in L.all_station_points().values():
        px, py = wp(x, y)
        ax.add_patch(Rectangle((px - 2.4, py - 2.4), 4.8, 4.8,
                               facecolor="#ffffff", edgecolor=MUTED, lw=0.7,
                               zorder=6))
    for spec in L.CHARGERS.values():
        px, py = wp(spec["x"], spec["y"])
        ax.add_patch(Rectangle((px - 3.2, py - 3.2), 6.4, 6.4,
                               facecolor="#dce9f2", edgecolor=BLUE, lw=0.9,
                               zorder=6))
    pk = L.PARK
    pa = wp(pk["x0"] - 0.75, pk["y"] - 0.75)
    pb = wp(pk["x0"] + pk["dx"] * (pk["n"] - 1) + 0.75, pk["y"] + 0.75)
    ax.add_patch(Rectangle(pa, pb[0] - pa[0], pb[1] - pa[1],
                           facecolor="#efe9d8", edgecolor="#b9ad86", lw=0.9,
                           ls=(0, (3, 2)), zorder=4))
    ax.add_patch(Rectangle((x0, y0), bw * SCALE, bh * SCALE, facecolor="none",
                           edgecolor="#8d8880", lw=1.6, zorder=12))
    for stage, cn, en, gx in (("carding", "梳棉区", "CARDING", 5.2),
                              ("drawing", "并条区", "DRAWING", 13.6),
                              ("roving", "粗纱区", "ROVING", 18.7)):
        px, py = wp(gx, bh - 0.85)
        ax.text(px, py, f"{cn} {en}", ha="center", va="center", fontsize=8.8,
                color=MUTED, zorder=8)
    for zone, cn in (("empty", "空筒库"), ("red", "成品库")):
        z = L.STORAGE[zone]
        px, py = wp(z["x"], 1.0)
        ax.text(px, py, cn, ha="center", va="center", fontsize=8.8, color=MUTED,
                rotation=90, zorder=8)


def draw_agv(ax, x, y, color=BLUE, size=1.0, zorder=9):
    """一台车：三层箭头，最外层做描边（同 render_demo 的画法）。"""
    px, py = wp(x, y)
    for k in (1.0, 0.66, 0.34):
        tri = [(11.0 * k * size, 0), (-6.8 * k * size, 7.2 * k * size),
               (-6.8 * k * size, -7.2 * k * size)]
        ax.add_patch(Polygon([(px + a, py + b) for a, b in tri], closed=True,
                             facecolor=INK if k == 1.0 else color,
                             edgecolor="none", zorder=zorder))


def draw_legend(ax):
    """车间顶部的通栏图例：把图上出现的四种记号讲清楚（同 render_demo）。"""
    fs = 9.0
    y = 14.25
    _, py = wp(0, y)
    x = wp(0.95, y)[0]

    def item(draw, label, w):
        nonlocal x
        draw(x)
        ax.text(x + w + 7, py, label, fontsize=fs, color=INK, va="center",
                zorder=9)
        x += w + 7 + text_w(label, fs) + 34

    def dock(px):
        ax.add_patch(Circle((px + 6, py), 5.2, facecolor="#ffffff",
                            edgecolor=BLUE, lw=1.6, zorder=9))
        ax.add_patch(Circle((px + 6, py), 2.0, facecolor=BLUE, edgecolor="none",
                            zorder=9))

    def idle(px):
        ax.add_patch(Polygon([(px + 13, py), (px, py + 6.5), (px, py - 6.5)],
                             closed=True, facecolor=INK, edgecolor="none",
                             zorder=9))
        ax.add_patch(Polygon([(px + 10.5, py), (px + 1.6, py + 4.4),
                              (px + 1.6, py - 4.4)], closed=True,
                             facecolor=BLUE, edgecolor="none", zorder=9))

    def busy(px):
        ax.add_patch(Polygon([(px + 11, py), (px, py + 5.5), (px, py - 5.5)],
                             closed=True, facecolor=INK, edgecolor="none",
                             zorder=9))
        ax.add_patch(Polygon([(px + 8.8, py), (px + 1.4, py + 3.7),
                              (px + 1.4, py - 3.7)], closed=True,
                             facecolor=RED, edgecolor="none", zorder=9))

    def legs(px):
        ax.plot([px, px + 13], [py, py], color=BLUE, lw=2.0, zorder=9,
                solid_capstyle="butt")
        ax.plot([px + 13, px + 26], [py, py], color=GREEN, lw=2.0, zorder=9,
                solid_capstyle="butt")

    item(dock, "料位 src / dst", 12)
    item(idle, "待命车 r（空闲）", 13)
    item(busy, "在途 / 排队车（计入 n_src、n_dst）", 11)
    item(legs, "① 空驶 / ② 载货", 26)


# ---------------------------------------------------------------- 右图：权重
def draw_weights(ax, Wt, panel):
    x0, y0, x1, y1 = panel
    LX = x0 + 24                     # 标签列左边界（所有标签统一左对齐）
    BAR = 1330.0                     # 条形起点
    UNIT = 26.0                      # 每个单位权重的像素长度
    ROWS = [618, 530, 442, 354, 266, 178]
    terms = [
        ("① 空驶", "α·d_deadhead · 空车跑去取料位，纯浪费",
         Wt["W_DEADHEAD"], BLUE),
        ("② 载货", "β·d_laden · 载重行驶，能耗与磨损更高",
         Wt["W_LADEN"], GREEN),
        ("③ 工位拥塞", "γ·(n_src + 1.5·n_dst) · 取放料位被几台车同时盯上",
         Wt["W_CONGEST"], RED),
        ("④ 电量可达", f"δ·max(0, 需求电量 + {Wt['BATTERY_RESERVE']:.0f}% 保留 − 当前电量)",
         Wt["W_ENERGY"], AMBER),
        ("⑤ 负载均衡", "η·(该车累计里程 − 车队均值)，抑制马太效应",
         Wt["W_BALANCE"], PURPLE),
        ("⑥ 任务老化", f"−ζ·age，age 封顶 AGE_CAP = {Wt['AGE_CAP']:.0f} s（防饿死）",
         Wt["W_AGE"], MUTED),
    ]
    for k in range(7):                                   # 竖向刻度
        gx = BAR + k * UNIT
        ax.plot([gx, gx], [140, 648], color=RULE, lw=0.6, alpha=0.85, zorder=1)
        txt(gx, 122, str(k), 9.6, color=MUTED, family=MONO)
    txt(x0 + 24, 122, "等效米", 9.6, color=MUTED, ha="left")
    for (cn, note, val, col), ry in zip(terms, ROWS):
        txt(LX, ry + 2, cn, 12.4, color=col, weight="bold", ha="left")
        rect(BAR, ry - 15, BAR + val * UNIT, ry + 15, f"bar-{cn}",
             facecolor=col, edgecolor=INK, lw=0.8, zorder=3)
        txt(BAR + val * UNIT + 9, ry + 1, f"{val:.2f}", 12.4, color=INK,
            weight="bold", family=MONO, ha="left")
        txt(LX, ry - 30, note, 9.8, color=MUTED, ha="left")
    txt(LX, 92, "条长 = 权重，单位等效米；③ 拥塞是唯一直接造成机台停待的一项",
        9.8, color=MUTED, ha="left")


# ---------------------------------------------------------------- 主图
def main() -> int:
    dst = sys.argv[1] if len(sys.argv) > 1 else "assets/readme/cost-model.png"
    Wt, how = read_weights()
    print(f"  权重来源：policies.py（{how}）  "
          + " ".join(f"{k}={v:g}" for k, v in Wt.items()))

    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": CJK,
        "axes.unicode_minus": False, "text.color": INK,
        "axes.edgecolor": RULE, "axes.labelcolor": INK,
        "xtick.color": MUTED, "ytick.color": MUTED,
    })
    fig = plt.figure(figsize=(W / 100.0, H / 100.0), dpi=100)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    LP = (36, 36, 1020, 706)
    RP = (1040, 36, 1564, 706)

    # ---------------- 页眉 ----------------
    ax.add_patch(Rectangle((0, 726), W, H - 726, facecolor=PANEL,
                           edgecolor="none"))
    ax.plot([0, W], [726, 726], color=RULE, lw=1.0)
    txt(40, 782, "CA-SSI 代价函数 · 几何与权重", 20.0, weight="bold",
        ha="left", va="top")
    txt(40, 748, "cost = α·d_deadhead + β·d_laden + γ·(n_src + 1.5·n_dst) "
                 "+ δ·电量短缺 + η·里程不均衡 − ζ·age + 0.02·priority"
                 "　—— 六项统一折算成等效米，越小越好", 10.6, color=MUTED,
        ha="left", va="top")
    txt(1560, 780, "权重直接读自 src/fleetflow_sim/fleetflow_sim/policies.py",
        10.0, color=MUTED, ha="right", va="top")

    # ---------------- 左：几何 ----------------
    frame(*LP, "left-panel")
    txt(60, 688, "① 代价的几何含义", 13.6, weight="bold", ha="left", va="top")
    txt(60 + text_w("① 代价的几何含义", 13.6) + 16, 685, "THE GEOMETRY",
        9.0, color=MUTED, ha="left", va="top")
    txt(60, 664, "一台待命车 r 接到任务：先空驶到取料位 src，再载货送到放料位 dst；"
                 "n_src / n_dst 是两个料位上正在赶来 + 排队的车数",
        9.8, color=MUTED, ha="left", va="top")
    ax.plot([60, 996], [650, 650], color=RULE, lw=1.0)
    draw_floor(ax)
    draw_legend(ax)

    src = L.station("carding", "finished", 1)      # carding_finished_1
    dstn = L.station("drawing", "waiting", 1)      # drawing_waiting_1
    r = L.park_poses(3)[2]                         # 待命区第 3 个位姿

    a = wp(r[0], r[1])
    b = wp(src["x"], src["y"])
    c = wp(dstn["x"], dstn["y"])

    # 参与这次搬运的两台机器
    for stage, lane, cn in (("carding", 1, "梳棉机"),
                            ("drawing", 1, "并条机")):
        mr = L.machine_rect(stage, lane)
        mp = wp((mr[0] + mr[2]) / 2, (mr[1] + mr[3]) / 2)
        txt(mp[0], mp[1], cn, 8.6, color=hexc(L.MACHINE_SPEC[stage]["rgb"]),
            inside=f"m-{stage}-{lane}")

    # 料位标记
    for p, col in ((b, BLUE), (c, GREEN)):
        ax.add_patch(Circle(p, 8.5, facecolor="#ffffff", edgecolor=col, lw=2.0,
                            zorder=10))
        ax.add_patch(Circle(p, 3.4, facecolor=col, edgecolor="none", zorder=11))

    arrow(a, b, BLUE, label="deadhead")
    arrow(b, c, GREEN, label="laden")

    # 在途 / 排队的车：n_src = 2，n_dst = 1
    draw_agv(ax, 12.95, 5.50, RED, 0.62)
    draw_agv(ax, 12.95, 6.55, RED, 0.62)
    draw_agv(ax, 11.55, 10.50, RED, 0.62)

    # 待命车
    draw_agv(ax, r[0], r[1], BLUE, 0.78)
    txt(*wp(r[0] - 0.55, r[1]), "空闲车 r  IDLE", 9.2, color=BLUE,
        ha="right")

    # 几何量标签
    mid_d = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    mid_l = ((b[0] + c[0]) / 2, (b[1] + c[1]) / 2)
    # 竖排两行：中文主标签 + 等宽符号，横ceiling刚好落在 x=10.5~13.6 的空走廊里，
    # 不会压到两侧的梳棉机 / 并条机
    txt(mid_d[0], mid_d[1] + 11, "① 空驶", 10.4, color=BLUE, weight="bold",
        bg=True, on_arrow="deadhead")
    txt(mid_d[0], mid_d[1] - 9, "d_deadhead", 9.4, color=BLUE, family=MONO,
        bg=True, on_arrow="deadhead")
    txt(mid_l[0], mid_l[1] + 11, "② 载货", 10.4, color=GREEN, weight="bold",
        bg=True, on_arrow="laden")
    txt(mid_l[0], mid_l[1] - 9, "d_laden", 9.4, color=GREEN, family=MONO,
        bg=True, on_arrow="laden")

    # 两端点的名字 + 计数徽标
    lp = wp(10.00, 4.75)
    txt(lp[0], lp[1], "取料位 src", 10.0, color=BLUE, weight="bold", bg=True)
    leader(wp(11.05, 4.95), wp(11.50, 5.88), BLUE, 1.0, label="lead-src")
    lq = wp(13.55, 6.05)
    txt(lq[0], lq[1], "n_src = 2", 10.0, color=RED, weight="bold", family=MONO,
        ha="left", bg=True)
    lp2 = wp(13.78, 9.32)
    txt(lp2[0], lp2[1], "放料位 dst", 10.0, color=GREEN, weight="bold",
        ha="left", bg=True)
    leader(wp(13.62, 9.44), wp(12.90, 10.42), GREEN, 1.0, label="lead-dst")
    lq2 = wp(11.02, 10.50)
    txt(lq2[0], lq2[1], "n_dst = 1", 10.0, color=RED, weight="bold",
        family=MONO, ha="right", bg=True)

    # ③ 的算例：把左右两块图钉在一起（权重取自同一个常量）
    n_src, n_dst = 2, 1
    cong = Wt["W_CONGEST"] * (n_src + 1.5 * n_dst)
    cx0, cy0 = wp(15.35, 8.35)
    ax.add_patch(Rectangle((cx0, cy0 - 56), 3.0, 70, facecolor=RED,
                           edgecolor="none", zorder=7))
    txt(cx0 + 13, cy0, "③ 工位拥塞  γ·(n_src + 1.5·n_dst)", 9.8, color=RED,
        weight="bold", ha="left")
    eq = f"= {Wt['W_CONGEST']:.2f} × ({n_src} + 1.5×{n_dst}) = {cong:.1f}"
    txt(cx0 + 13, cy0 - 19, eq, 9.4, color=MUTED, ha="left", family=MONO)
    txt(cx0 + 13, cy0 - 36, "权重见右图 ③", 9.4, color=MUTED, ha="left")

    # 左图脚注：讲清楚符号与近似
    txt(60, 72, "d_deadhead = ‖r − src‖，d_laden = ‖src − dst‖，两者都用欧氏距离近似 "
                "A* 实际路径长度（相关性 > 0.99）；n_src / n_dst 为该料位当前在途 + "
                "排队的车数（图上是示例快照）", 9.4, color=MUTED, ha="left",
        va="top")
    txt(60, 50, "④ 电量可达、⑤ 负载均衡、⑥ 任务老化不含平面几何量：它们分别看电量、"
                "累计里程与等待时长，只出现在右图", 9.4, color=MUTED, ha="left",
        va="top")

    # ---------------- 右：权重 ----------------
    frame(*RP, "right-panel")
    txt(1064, 688, "② 六项权重", 13.6, weight="bold", ha="left", va="top")
    txt(1064 + text_w("② 六项权重", 13.6) + 16, 685, "THE WEIGHTS", 9.0,
        color=MUTED, ha="left", va="top")
    txt(1064, 664, "policies.py 里的常量，六项相加就是一次投标的代价",
        9.8, color=MUTED, ha="left", va="top")
    ax.plot([1064, 1540], [650, 650], color=RULE, lw=1.0)
    draw_weights(ax, Wt, RP)

    ok = audit(fig)
    fig.savefig(dst, facecolor=BG)
    print(f"wrote {dst}" + ("" if ok else "  (排版检查未通过)"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
