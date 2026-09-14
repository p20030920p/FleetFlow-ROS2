#!/usr/bin/env python3
"""从录制里统计"车卡死"的严重程度：行驶态但长时间不动。

用户反馈 demo 里小车会卡死，而旧 demo 没有。要客观比较配置，就需要一个
可复现的指标 —— 这个脚本给出：

  * 卡死事件数、总时长、占车时比例
  * 卡死热点（按 2 m 网格聚合），用来判断堵在哪条通道

判据：处于行驶态（to_pickup/to_dropoff/departing）却连续 ≥ MIN_STALL 秒
位移 < 0.05 m。装卸、待命不算（那些本来就不该动）。

用法: python3 tools/stall_report.py <record.jsonl> [--min-stall 8]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict

TRAVEL = {"to_pickup", "to_dropoff", "to_charger", "departing"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("--min-stall", type=float, default=8.0)
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.jsonl, encoding="utf-8")
            if l.strip()]
    if not rows:
        print("空文件"); return 1
    # 台数必须取**全程出现过的最大集合**，不能取第一帧：
    # 记录器开机时车队可能还没到齐（实测 8 车配置的首帧只有 6 台），
    # 用首帧算车时会低估基数，卡死占比随之被高估。
    seen = set()
    for r in rows:
        seen.update(b[0] for b in r.get("robots") or [])
    n_robots = len(seen)
    span = rows[-1]["t"] - rows[0]["t"]

    live = defaultdict(lambda: {"since": None, "pos": None})
    episodes = []
    for r in rows:
        t = r["t"]
        for b in r["robots"]:
            rid, x, y, st = b[0], b[1], b[2], b[4]
            d = live[rid]
            if st in TRAVEL:
                if d["since"] is None:
                    d["since"], d["pos"] = t, (x, y)
                elif math.hypot(x - d["pos"][0], y - d["pos"][1]) >= 0.05:
                    dur = t - d["since"]
                    if dur >= args.min_stall:
                        episodes.append((dur, rid, d["pos"]))
                    d["since"], d["pos"] = t, (x, y)
            else:
                if d["since"] is not None:
                    dur = t - d["since"]
                    if dur >= args.min_stall:
                        episodes.append((dur, rid, d["pos"]))
                d["since"], d["pos"] = None, None

    car_time = n_robots * span
    total = sum(e[0] for e in episodes)
    episodes.sort(reverse=True)
    print(f"{args.jsonl}")
    print(f"  {n_robots} 台车 × {span:.0f} s = {car_time:.0f} 车时"
          f"（全程出现过的车: {sorted(seen)}）")
    print(f"  卡死事件 {len(episodes)} 次 · 总时长 {total:.0f} s · "
          f"占车时 {total/max(car_time,1e-6):.1%}")
    if episodes:
        grid = defaultdict(float)
        for dur, _rid, (x, y) in episodes:
            grid[(int(x // 2) * 2, int(y // 2) * 2)] += dur
        top = sorted(grid.items(), key=lambda kv: -kv[1])[:5]
        print("  热点(x,y 各 2 m): " +
              " · ".join(f"({gx},{gy}) {d:.0f}s" for (gx, gy), d in top))
    return 0


if __name__ == "__main__":
    sys.exit(main())
