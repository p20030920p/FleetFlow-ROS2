#!/usr/bin/env python3
"""CA-SSI 代价函数的消融实验图。

    python3 tools/plot_ablation.py experiments/ablation/summary.csv \
        assets/readme/ablation.png

做法是"留一法"：每次只把六项代价中的一项权重置零，其余不动，跑同一组种子。
于是每根柱子相对完整模型（最上面那根）的落差，就是那一项单独的贡献。
"""
from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BG = "#f4f2ed"
INK = "#1c1c1a"
MUTED = "#6d6a63"
RULE = "#c9c4b8"
GREEN = "#2e7d4f"
RED = "#c0392b"
AMBER = "#e8a33d"
BLUE = "#3d5a80"
GREY = "#9a958c"

# 策略名 -> (中文标签, 英文标签)
LABEL = {
    "ca_ssi":    ("完整模型", "full model"),
    "ca_nocong": ("去掉 工位拥塞", "no contention"),
    "ca_noener": ("去掉 电量可达", "no energy"),
    "ca_nobal":  ("去掉 负载均衡", "no load balance"),
    "ca_noage":  ("去掉 任务老化", "no ageing"),
}
ORDER = ["ca_ssi", "ca_nocong", "ca_noage", "ca_noener", "ca_nobal"]


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("summary", nargs="?")
    ap.add_argument("out_pos", nargs="?")
    ap.add_argument("--out", default=None, help="输出 PNG（配合 --group）")
    ap.add_argument("--group", action="append", default=[],
                    metavar="LABEL=SUMMARY.CSV",
                    help="可变组数：每给一个就多画一排（按车队规模分组时用）")
    args = ap.parse_args()
    if not args.group and not (args.summary and args.out):
        ap.error("要么给 --group，要么给 summary out 两个位置参数")

    def aggregate(path):
        agg = defaultdict(lambda: defaultdict(list))
        for r in load(path):
            q = r.get("policy")
            if q not in LABEL:
                continue
            done = fnum(r.get("completed")) or 0.0
            dist = fnum(r.get("distance_total_m")) or 0.0
            for k in ("throughput_per_min", "near_miss_events", "latency_mean_s"):
                v = fnum(r.get(k))
                if v is not None:
                    agg[q][k].append(v)
            if done:
                agg[q]["distance_per_task"].append(dist / done)
        return {q: {k: (statistics.fmean(v),
                        statistics.pstdev(v) if len(v) > 1 else 0.0)
                    for k, v in m.items()} for q, m in agg.items()}

    if args.group:
        groups = []
        for item in args.group:
            if "=" not in item:
                print(f"--group 需要 LABEL=FILE，收到 {item!r}", file=sys.stderr)
                return 2
            lab, path = item.split("=", 1)
            groups.append((lab, aggregate(path)))
        data = groups[0][1]
    else:
        groups = [(None, aggregate(args.summary))]
        data = groups[0][1]
    pols = [p for p in ORDER if p in data]
    full_tp = data.get("ca_ssi", {}).get("throughput_per_min", (0.0, 0.0))[0]

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"],
        "axes.unicode_minus": False, "axes.edgecolor": RULE,
        "axes.labelcolor": INK, "text.color": INK, "ytick.color": INK,
        "xtick.color": MUTED,
    })
    if len(groups) > 1:
        # 多组：每组单独出一张，文件名加后缀（README 里可并排引用）
        import re as _re
        base = args.out or args.out_pos or "ablation.png"
        rc = 0
        for lab, gdata in groups:
            slug = _re.sub(r"[^0-9A-Za-z]+", "", str(lab)) or "x"
            one = _re.sub(r"\.png$", f"_{slug}.png", base)
            rc |= _render_one(gdata, one)
        print(f"wrote {len(groups)} panels under {base}")
        return rc

    return _render_one(data, args.out or args.out_pos)


def _render_one(data, out):
    pols = [p for p in ORDER if p in data]
    full_tp = data.get("ca_ssi", {}).get("throughput_per_min", (0.0, 0.0))[0]
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(15.0, 5.2), dpi=100,
                                 gridspec_kw=dict(width_ratios=[1.25, 1.0], wspace=0.28))
    fig.patch.set_facecolor(BG)

    ys = list(range(len(pols)))[::-1]
    vals = [data[p]["throughput_per_min"][0] for p in pols]
    errs = [data[p]["throughput_per_min"][1] for p in pols]
    cols = [GREEN if p == "ca_ssi" else (RED if full_tp and v < full_tp * 0.9 else AMBER)
            for p, v in zip(pols, vals)]
    ax.set_facecolor(BG)
    ax.barh(ys, vals, xerr=errs, height=0.58, color=cols, edgecolor=INK, lw=0.7,
            error_kw=dict(ecolor=INK, elinewidth=0.9, capthick=0.9, capsize=3.5))
    ax.set_yticks(ys)
    ax.set_yticklabels([f"{LABEL[p][0]}\n{LABEL[p][1]}" for p in pols], fontsize=9.6,
                       linespacing=1.5)
    ax.tick_params(axis="y", length=0)
    ax.set_xlabel("吞吐率  tasks / min", fontsize=9.8)
    ax.grid(axis="x", color=RULE, lw=0.6, alpha=0.7)
    ax.set_axisbelow(True)
    for sp in ("top", "right", "left"):
        ax.spines[sp].set_visible(False)
    ax.set_xlim(0, max(v + e for v, e in zip(vals, errs)) * 1.26)
    for y, p, v, e in zip(ys, pols, vals, errs):
        ax.text(v + e + max(vals) * 0.03, y, f"{v:.1f}", va="center", fontsize=9.8,
                fontweight="bold", family="monospace", color=INK)
        if p != "ca_ssi" and full_tp:
            d = (v - full_tp) / full_tp * 100.0
            ax.text(v + e + max(vals) * 0.14, y, f"{d:+.0f}%", va="center",
                    fontsize=9.0, family="monospace",
                    color=RED if d < 0 else GREEN)
    ax.set_title("留一法消融：去掉一项，吞吐掉多少", fontsize=12.6, fontweight="bold",
                 color=INK, pad=30, loc="left")
    ax.text(0.0, 1.035, "LEAVE-ONE-OUT ABLATION · 80 units in circulation",
            transform=ax.transAxes, fontsize=8.4, color=MUTED, va="bottom")

    # 右：单任务里程
    ys2 = list(range(len(pols)))[::-1]
    v2 = [data[p]["distance_per_task"][0] for p in pols]
    e2 = [data[p]["distance_per_task"][1] for p in pols]
    bx.set_facecolor(BG)
    bx.barh(ys2, v2, xerr=e2, height=0.58,
            color=[GREEN if p == "ca_ssi" else GREY for p in pols],
            edgecolor=INK, lw=0.7,
            error_kw=dict(ecolor=INK, elinewidth=0.9, capthick=0.9, capsize=3.5))
    bx.set_yticks(ys2)
    bx.set_yticklabels([LABEL[p][0] for p in pols], fontsize=9.8)
    bx.tick_params(axis="y", length=0)
    bx.set_xlabel("每单行驶  m / task", fontsize=9.8)
    bx.grid(axis="x", color=RULE, lw=0.6, alpha=0.7)
    bx.set_axisbelow(True)
    for sp in ("top", "right", "left"):
        bx.spines[sp].set_visible(False)
    bx.set_xlim(0, max(v + e for v, e in zip(v2, e2)) * 1.22)
    for y, v, e in zip(ys2, v2, e2):
        bx.text(v + e + max(v2) * 0.03, y, f"{v:.1f}", va="center", fontsize=9.8,
                fontweight="bold", family="monospace", color=INK)
    bx.set_title("代价：每单行驶里程", fontsize=12.6, fontweight="bold", color=INK,
                 pad=30, loc="left")

    fig.subplots_adjust(left=0.105, right=0.985, top=0.80, bottom=0.10)
    fig.savefig(out, facecolor=BG)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
