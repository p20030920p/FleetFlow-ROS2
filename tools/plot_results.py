#!/usr/bin/env python3
"""把策略对比实验画成 README 用的对照图（工业看板配色，浅底、无发光）。

    python3 tools/plot_results.py <饱和工况.csv> <富裕工况.csv> <out.png>

每行一个工况、每列一个指标，五种策略并排。之所以要两个工况，是因为这张图
真正要说明的是**结论本身**：只有当车队成为瓶颈时，分配策略才起作用。
富裕工况那一行所有柱子几乎等高，就是这句话的证据。

``summary.csv`` 由 ``tools/run_experiments.py`` 生成，每个 (策略, 种子) 一行；
脚本按策略聚合均值与标准差，画误差棒并标注相对 ``random`` 的变化百分比。
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

STYLE = {
    "random":    ("#9a958c", "random\n随机"),
    "nearest":   ("#3d5a80", "nearest\n就近"),
    "ssi":       ("#e8a33d", "SSI\n单件拍卖"),
    "ca_ssi":    ("#2e7d4f", "CA-SSI\n本文方法"),
    "hungarian": ("#c0392b", "Hungarian\n单轮最优"),
}
ORDER = ["random", "nearest", "ssi", "ca_ssi", "hungarian"]

METRICS = [
    ("throughput_per_min", "吞吐率 Throughput", "件 / 分钟", "higher"),
    ("latency_mean_s",     "平均任务时延 Latency", "秒", "lower"),
    ("distance_per_task",  "单任务里程 Travel/task", "米", "lower"),
    ("utilisation_mean",   "车队利用率 Utilisation", "—", "higher"),
]


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def derive(rows):
    """返回 {policy: {metric: (mean, std)}}。"""
    agg = defaultdict(lambda: defaultdict(list))
    for r in rows:
        p = r.get("policy")
        if p not in STYLE:
            continue
        done = fnum(r.get("completed")) or 0.0
        dist = fnum(r.get("distance_total_m")) or 0.0
        for key in ("throughput_per_min", "latency_mean_s", "utilisation_mean"):
            v = fnum(r.get(key))
            if v is not None:
                agg[p][key].append(v)
        if done:
            agg[p]["distance_per_task"].append(dist / done)
    out = {}
    for p, m in agg.items():
        out[p] = {}
        for k, vals in m.items():
            if vals:
                out[p][k] = (statistics.fmean(vals),
                             statistics.pstdev(vals) if len(vals) > 1 else 0.0)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("saturated")
    ap.add_argument("slack")
    ap.add_argument("out")
    args = ap.parse_args()

    conds = [
        ("工况 A · 小车队  SMALL FLEET  ·  3 AGVs  ·  利用率 ≈ 0.98",
         derive(load(args.saturated)),
         "对接位竞争少 —— 策略之间有差距，但不大"),
        ("工况 B · 大车队  LARGE FLEET  ·  8 AGVs  ·  利用率 ≈ 0.73",
         derive(load(args.slack)),
         "对接位竞争激烈 —— 拥塞感知的收益被放大"),
    ]

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"],
        "axes.unicode_minus": False,
        "axes.edgecolor": RULE, "axes.labelcolor": INK, "text.color": INK,
        "xtick.color": INK, "ytick.color": MUTED,
    })
    fig, axes = plt.subplots(2, 4, figsize=(16.0, 8.6), dpi=100)
    fig.patch.set_facecolor(BG)

    for row, (title, data, note) in enumerate(conds):
        policies = [p for p in ORDER if p in data]
        ref = data.get("random", {})
        for col, (key, mtitle, unit, direction) in enumerate(METRICS):
            ax = axes[row][col]
            ax.set_facecolor(BG)
            vals = [data[p].get(key, (0.0, 0.0))[0] for p in policies]
            errs = [data[p].get(key, (0.0, 0.0))[1] for p in policies]
            cols = [STYLE[p][0] for p in policies]
            x = list(range(len(policies)))
            ax.bar(x, vals, yerr=errs, capsize=3.5, width=0.62, color=cols,
                   edgecolor=INK, linewidth=0.6,
                   error_kw=dict(ecolor=INK, elinewidth=0.9, capthick=0.9))
            if row == 0:
                ax.set_title(mtitle, fontsize=12.4, fontweight="bold", color=INK, pad=10)
            ax.set_ylabel(unit, fontsize=9.0)
            ax.set_xticks(x)
            ax.set_xticklabels([STYLE[p][1] for p in policies], fontsize=8.0,
                               linespacing=1.5)
            ax.grid(axis="y", color=RULE, linewidth=0.6, alpha=0.7)
            ax.set_axisbelow(True)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
            top = max((v + e) for v, e in zip(vals, errs)) or 1.0
            ax.set_ylim(0, top * 1.34)
            for xi, (v, e) in enumerate(zip(vals, errs)):
                ax.text(xi, v + e + top * 0.05, f"{v:.1f}", ha="center",
                        fontsize=8.8, fontweight="bold", family="monospace", color=INK)
                r = ref.get(key, (None,))[0]
                if r and abs(r) > 1e-9 and policies[xi] != "random":
                    d = (v - r) / r * 100.0
                    good = (d > 0) if direction == "higher" else (d < 0)
                    ax.text(xi, v + e + top * 0.155, f"{d:+.0f}%", ha="center",
                            fontsize=7.8, family="monospace",
                            color="#2e7d4f" if good else "#c0392b")
        axes[row][0].text(0.0, 1.30, title, transform=axes[row][0].transAxes,
                          fontsize=10.6, fontweight="bold", color=INK,
                          ha="left", va="bottom")
        axes[row][0].text(0.0, 1.185, note, transform=axes[row][0].transAxes,
                          fontsize=9.0, color=MUTED, ha="left", va="bottom")

    fig.suptitle("任务分配策略对比 · Task-allocation policy comparison",
                 fontsize=15, fontweight="bold", color=INK, x=0.010,
                 ha="left", y=0.992)
    fig.text(0.010, 0.952,
             "5 种策略 × 3 个随机种子 × 120 秒 · 误差棒 = 1σ · 百分比相对 random "
             "· 两行使用完全相同的工厂与物料模型（80 件物料），只有车队规模不同",
             fontsize=9.4, color=MUTED, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.918), h_pad=5.6)
    fig.savefig(args.out, facecolor=BG)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
