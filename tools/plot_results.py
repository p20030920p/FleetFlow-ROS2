#!/usr/bin/env python3
"""把两组实验（浅任务池 / 深任务池）画成一张对照图。

    python3 tools/plot_results.py experiments-shallow/summary.csv \
                                  experiments-deep/summary.csv \
                                  assets/readme/policy-comparison.png
"""
from __future__ import annotations

import csv
import statistics
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BG = "#0b1016"
PANEL = "#111a23"
FG = "#e8eef5"
MUTED = "#7d8f9e"
ACCENT = "#ff6a1a"
CYAN = "#2fd4c4"
GREY = "#8d99a6"
BAR = {"random": GREY, "nearest": CYAN, "ssi": ACCENT}
LABEL = {"random": "random", "nearest": "nearest", "ssi": "SSI auction"}
METRICS = [("throughput_per_min", "Throughput  (tasks / min)", "higher is better"),
           ("latency_mean_s", "Mean task latency  (s)", "lower is better"),
           ("distance_per_task", "Travel per task  (m)", "lower is better"),
           ("near_miss_events", "Near-miss events", "lower is better")]


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def values(rows, key):
    out = []
    for r in rows:
        if key == "distance_per_task":
            d, c = fnum(r.get("distance_total_m")), fnum(r.get("completed"))
            if d is not None and c:
                out.append(d / c)
        else:
            v = fnum(r.get(key))
            if v is not None:
                out.append(v)
    return out


def main() -> int:
    shallow_path = sys.argv[1] if len(sys.argv) > 1 else "experiments-shallow/summary.csv"
    deep_path = sys.argv[2] if len(sys.argv) > 2 else "experiments-deep/summary.csv"
    dst = sys.argv[3] if len(sys.argv) > 3 else "assets/readme/policy-comparison.png"

    conditions = [("Shallow task pool  ·  pending tasks ≤ vehicles",
                   load(shallow_path), "vehicles have little to choose from"),
                  ("Deep task pool  ·  pending tasks ≫ vehicles",
                   load(deep_path), "the assignment actually has to be chosen")]

    fig, axes = plt.subplots(2, 4, figsize=(16, 7.6), facecolor=BG)
    for row, (title, rows, note) in enumerate(conditions):
        by = defaultdict(list)
        for r in rows:
            by[r["policy"]].append(r)
        order = [p for p in ("random", "nearest", "ssi") if p in by]
        n = max((len(v) for v in by.values()), default=0)

        for col, (key, metric_title, hint) in enumerate(METRICS):
            ax = axes[row][col]
            vals = [statistics.fmean(values(by[p], key)) if values(by[p], key) else 0.0
                    for p in order]
            errs = [statistics.pstdev(values(by[p], key)) if len(values(by[p], key)) > 1 else 0.0
                    for p in order]
            bars = ax.bar(range(len(order)), vals, yerr=errs, capsize=4,
                          color=[BAR.get(p, CYAN) for p in order], width=0.6,
                          error_kw=dict(ecolor=MUTED, lw=1.1))
            ax.set_facecolor(PANEL)
            head = f"{metric_title}"
            ax.set_title(head, color=FG, fontsize=10.5, pad=8)
            if col == 0:
                ax.text(0.0, 1.16, title, transform=ax.transAxes, color=CYAN,
                        fontsize=9.5, ha="left", va="bottom")
            ax.set_xticks(range(len(order)))
            ax.set_xticklabels([LABEL.get(p, p) for p in order], color=MUTED, fontsize=8.5)
            ax.tick_params(axis="y", colors=MUTED, labelsize=8.5)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
            for sp in ("left", "bottom"):
                ax.spines[sp].set_color("#1e2b38")
            ax.grid(axis="y", color="#1e2b38", lw=0.7, alpha=0.7)
            ax.set_axisbelow(True)
            for b, v in zip(bars, vals):
                ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.1f}", ha="center",
                        va="bottom", color=FG, fontsize=9.5)
            if row == 0:
                ax.text(0.98, 0.93, hint, transform=ax.transAxes, ha="right",
                        color=MUTED, fontsize=7.4)

        # 工况标题放在该行第一张图的标题上方，避免侧边旋转文字与标题打架

    fig.suptitle(f"Task-allocation comparison  ·  {max(len(v) for v in
                 defaultdict(list, {p: [r for r in conditions[1][1] if r['policy'] == p]
                 for p in ('random','nearest','ssi')}).values())} runs per policy per condition"
                 f"  ·  8 AGVs  ·  100 s",
                 color=FG, fontsize=13, y=0.985)
    fig.tight_layout(rect=(0.01, 0, 1, 0.94), h_pad=3.4)
    fig.savefig(dst, facecolor=BG)
    print("wrote", dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
