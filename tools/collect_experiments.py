#!/usr/bin/env python3
"""把 experiments/<组>/*/metrics/run.csv 汇总成组级 summary.csv 并打印均值±标准差。

为什么不用 run_experiments.py 自带的 summary：
它只汇总**本次调用**产生的行。而当实验分批跑（比如先跑 3 个策略、隔时间再跑
另外 3 个）时，后一次调用会把前一次的 summary.csv 覆盖掉。这个脚本直接扫
每个组合目录，因此天然支持分批补齐。

用法:
    python3 tools/collect_experiments.py experiments/sat_criterion
"""
from __future__ import annotations

import csv
import os
import statistics
import sys

NUM = ["completed", "throughput_per_min", "latency_mean_s", "latency_p95_s",
       "makespan_s", "utilisation_mean", "distance_total_m", "min_robot_gap_m",
       "overlap_events", "near_miss_events", "min_robot_distance_m"]

# 策略在表格里的顺序：从"最差/最简单"到"本文方法"
ORDER = ["random", "nearest", "zone", "ssi", "hungarian",
         "ca_ssi", "ca_nocong", "ca_noener", "ca_nobal", "ca_noage"]


def main() -> int:
    root = sys.argv[1] if len(sys.argv) > 1 else "experiments/sat_criterion"
    rows = []
    for name in sorted(os.listdir(root)):
        run_csv = os.path.join(root, name, "metrics", "run.csv")
        if not os.path.isfile(run_csv):
            continue
        with open(run_csv, newline="", encoding="utf-8") as f:
            got = list(csv.DictReader(f))
        if got:
            rows.append(got[-1])
    if not rows:
        print(f"{root} 下没有可汇总的 run.csv")
        return 1

    fields = list(rows[0].keys())
    out = os.path.join(root, "summary.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    def fnum(r, k):
        try:
            return float(r.get(k, ""))
        except (TypeError, ValueError):
            return None

    policies = sorted({r["policy"] for r in rows},
                      key=lambda p: (ORDER.index(p) if p in ORDER else 99, p))
    print(f"{len(rows)} 次运行 / {len(policies)} 个策略   目录: {root}\n")
    head = (f"{'策略':<11}{'n':>2} {'完成单':>13} {'单/分钟':>15} "
            f"{'延迟均值':>13} {'利用率':>13} {'最小净距':>13} {'重叠次数':>10}")
    print(head)
    print("-" * len(head))
    for p in policies:
        rs = [r for r in rows if r["policy"] == p]
        cells = []
        for k in ("completed", "throughput_per_min", "latency_mean_s",
                  "utilisation_mean", "min_robot_gap_m", "overlap_events"):
            vs = [fnum(r, k) for r in rs]
            vs = [v for v in vs if v is not None]
            if not vs:
                cells.append("     -")
                continue
            m = statistics.fmean(vs)
            if len(vs) > 1:
                sd = statistics.stdev(vs)
                cells.append(f"{m:6.2f}±{sd:4.2f}")
            else:
                cells.append(f"{m:6.2f}     ")
        print(f"{p:<11}{len(rs):>2} " + " ".join(cells))
    print(f"\n已写出 {out}")
    print("注：± 为同策略不同种子的样本标准差。单次运行的方差很大，"
          "比较策略必须看多次重复，不能看单跑。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
