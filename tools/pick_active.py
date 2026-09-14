#!/usr/bin/env python3
"""从一次录制里挑出**活动最密集**的一段，供渲染动图。

为什么要挑：录制窗口里工厂的供料是波动的，前后两端常常只有一两个在途任务，
车几乎不动（README 动图"完全卡死"就是这么来的）。与其把整段压进动图、
让大半时间静止，不如选出持续有活动的窗口，并在图注里说明这是哪一段。

判据是**全队每帧位移之和**的滑动平均，不是任务数 —— 我们要的是"看得出在动"。

用法:
    python3 tools/pick_active.py in.jsonl --window 150 --out active.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import sys


def load(path: str) -> list[dict]:
    rows = []
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows


def displacement(a: dict, b: dict) -> float:
    ca = {r[0]: (r[1], r[2]) for r in a.get("robots") or []}
    cb = {r[0]: (r[1], r[2]) for r in b.get("robots") or []}
    return sum(math.hypot(cb[k][0] - ca[k][0], cb[k][1] - ca[k][1])
               for k in ca if k in cb)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("--window", type=float, default=150.0,
                    help="窗口长度（秒，按录制时间轴）")
    ap.add_argument("--out", default=None, help="输出 jsonl（默认覆盖输入）")
    args = ap.parse_args()

    rows = load(args.jsonl)
    if len(rows) < 10:
        print("数据太少"); return 1
    tkey = "sim" if rows[0].get("sim") is not None else "t"
    d = [displacement(a, b) for a, b in zip(rows, rows[1:])]
    # 以"累计位移"找最优窗口：位移越大越好
    ts = [r[tkey] for r in rows]
    best, bi, bj = -1.0, 0, len(rows) - 1
    j = 0
    run = 0.0
    for i in range(len(d)):
        while j < len(d) and ts[j + 1] - ts[i] <= args.window:
            run += d[j]
            j += 1
        if run > best:
            best, bi, bj = run, i, j
        run -= d[i]
    out = rows[bi:bj + 1]
    span = ts[bj] - ts[bi]
    print(f"总窗口 {ts[-1]-ts[0]:.0f}s -> 选中 t {ts[bi]:.1f}~{ts[bj]:.1f}s "
          f"({span:.0f}s, {len(out)} 帧)")
    print(f"  该窗口累计位移 {best:.1f} m（全段 {sum(d):.1f} m）")
    dst = args.out or args.jsonl
    with open(dst, "w", encoding="utf-8") as f:
        for r in out:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    print(f"  写出 {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
