#!/usr/bin/env python3
"""把实验 summary.csv 的统计量注入 README 的占位符。

    python3 tools/update_readme_numbers.py experiments/deep_v2/summary.csv README.md

README 里用 ``@@policy_metric@@`` 形式的占位符标注待填数字（例如
``@@ca_tp@@`` = ca_ssi 的吞吐率均值）。脚本按策略聚合均值后统一替换，
这样"文档里的数字"和"实验跑出来的数字"不可能对不上。
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
from collections import defaultdict

SHORT = {"random": "rand", "nearest": "near", "ssi": "ssi",
         "ca_ssi": "ca", "hungarian": "hun"}

# 占位符后缀 -> summary.csv 字段
FIELDS = {
    "comp": "completed",
    "tp": "throughput_per_min",
    "lat": "latency_mean_s",
    "p95": "latency_p95_s",
    "nm": "near_miss_events",
    "dist": "distance_total_m",
    "util": "utilisation_mean",
}


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def collect(rows, prefix):
    agg: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for r in rows:
        p = r.get("policy")
        for key, field in FIELDS.items():
            v = fnum(r.get(field))
            if v is not None:
                agg[p][key].append(v)
        done, dist = fnum(r.get("completed")), fnum(r.get("distance_total_m"))
        if done and dist is not None:
            agg[p]["dpt"].append(dist / done)

    val = {}
    for p, m in agg.items():
        short = SHORT.get(p)
        if not short:
            continue
        for key, v in m.items():
            val[f"{prefix}{short}_{key}"] = statistics.fmean(v)

    def pct(a, b):
        return f"{(a - b) / b * 100:+.0f} %" if b else "n/a"

    subs = {k: f"{v:.{DECIMALS}f}" for k, v in val.items()}
    for key in ("tp", "dpt", "nm", "lat"):
        for pre in (prefix, ""):
            ca, ss, rd = (f"{pre}ca_{key}", f"{pre}ssi_{key}", f"{pre}rand_{key}")
            if ca in val and ss in val:
                subs[f"d_{pre}{key}_ssi"] = pct(val[ca], val[ss])
            if ca in val and rd in val:
                subs[f"d_{pre}{key}"] = pct(val[ca], val[rd])
    return subs


DECIMALS = 1


def main() -> int:
    global DECIMALS
    ap = argparse.ArgumentParser()
    ap.add_argument("summary", nargs="+", help="summary.csv，可给多个")
    ap.add_argument("--prefix", nargs="*", default=None,
                    help="每个 csv 对应的占位符前缀，如 sat slack")
    ap.add_argument("--readme", nargs="+", required=True)
    ap.add_argument("--decimals", type=int, default=1)
    args = ap.parse_args()

    DECIMALS = args.decimals
    prefixes = args.prefix or [""] * len(args.summary)
    if len(prefixes) != len(args.summary):
        print("--prefix 数量必须与 csv 数量一致")
        return 2

    subs = {}
    for path, pre in zip(args.summary, prefixes):
        with open(path, newline="", encoding="utf-8") as f:
            subs.update(collect(list(csv.DictReader(f)), pre))

    total = 0
    for path in args.readme:
        src = open(path, encoding="utf-8").read()
        for k, v in subs.items():
            token = f"@@{k}@@"
            if token in src:
                total += src.count(token)
                src = src.replace(token, v)
        open(path, "w", encoding="utf-8").write(src)
        print(f"updated {path}")
    left = sum(open(p, encoding="utf-8").read().count("@@")
               for p in args.readme)
    print(f"substituted {total} placeholders; {left} token(s) left unfilled")
    return 1 if left else 0


if __name__ == "__main__":
    sys.exit(main())
