#!/usr/bin/env python3
"""从 launch.log 里算每台车任务周期的分段耗时。

控制器对每次状态迁移都打了 `[INFO] [<墙钟时间戳>] [robot_i.robot_controller]:
robot_i <old> -> <new> ...`，所以不必再加埋点，直接挖日志即可。
分段定义（取相邻两次迁移的时间差）：

    idle -> waiting_lease      等租约
    waiting_lease -> to_pickup 领到租约
    to_pickup -> loading       空驶到取货点
    loading -> departing       装货停留（dwell）
    departing -> waiting_lease 驶离工位
    waiting_lease -> to_dropoff 领到卸货租约
    to_dropoff -> unloading    载货行驶
    unloading -> idle          卸货停留 + 完成

用法: python3 tools/task_timing.py <launch.log> [--top N]
"""
from __future__ import annotations

import re
import statistics
import sys
from collections import defaultdict

LINE = re.compile(
    r"\[(?P<ts>\d+\.\d+)\] \[(?P<node>robot_\d+\.robot_controller)\]: "
    r"robot_(?P<rid>\d+) (?P<old>[a-z_]+) -> (?P<new>[a-z_]+)")

# 真正要测的是**在每个状态里停留多久**，而不是迁移本身。
# 头一版按 (old,new) 给"分段"命名，结果把两段最长的行驶（to_pickup、
# to_dropoff）整个漏掉了 —— 因为它们是"停留时长"，不是一次迁移的间隔。
ZH = {
    "waiting_lease": "等租约",
    "to_pickup": "空驶→取货点",
    "loading": "装货停留",
    "departing": "驶离工位",
    "to_dropoff": "载货→卸货点",
    "unloading": "卸货停留+完工",
    "to_charger": "空驶→充电桩",
    "charging": "充电",
}


def main() -> int:
    path = sys.argv[1]
    top = 8
    if "--top" in sys.argv:
        top = int(sys.argv[sys.argv.index("--top") + 1])

    events = defaultdict(list)          # rid -> [(ts, old, new)]
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = LINE.search(line)
            if m:
                events[int(m.group("rid"))].append(
                    (float(m.group("ts")), m.group("old"), m.group("new")))

    if not events:
        print("日志里没有状态迁移记录（控制器输出可能没进这个文件）")
        return 1

    seg_times = defaultdict(list)
    for rid, evs in events.items():
        evs.sort()
        # 每次迁移 (ts, old, new)：从 ts 进入 new，直到下一次迁移才离开
        for (t0, _, n0), (t1, o1, _) in zip(evs, evs[1:]):
            if n0 != o1:
                continue                     # 日志缺口，跳过
            name = ZH.get(n0)
            if name:
                dt = t1 - t0
                if 0.0 <= dt < 900.0:
                    seg_times[name].append(dt)

    print(f"日志: {path}")
    print(f"解析到 {len(events)} 台车的状态迁移\n")
    print(f"{'分段':<18} {'次数':>5} {'均值(s)':>9} {'中位(s)':>9} "
          f"{'p90(s)':>8} {'合计(s)':>9}")
    print("-" * 62)
    totals = {k: sum(v) for k, v in seg_times.items()}
    order = sorted(totals, key=lambda k: -totals[k])
    for name in order:
        v = seg_times[name]
        v_sorted = sorted(v)
        p90 = v_sorted[int(0.9 * (len(v_sorted) - 1))]
        print(f"{name:<18} {len(v):>5} {statistics.fmean(v):>9.2f} "
              f"{statistics.median(v):>9.2f} {p90:>8.2f} {totals[name]:>9.1f}")
    print("-" * 62)
    grand = sum(totals.values())
    print(f"{'合计':<18} {'':>5} {'':>9} {'':>9} {'':>8} {grand:>9.1f}")
    print()
    print("按**总耗时**排序（这才是该优化的地方，而不是单次均值）：")
    for name in order[:top]:
        share = totals[name] / grand * 100 if grand else 0
        print(f"  {name:<18} {totals[name]:>7.1f} s  {share:>5.1f}%")

    n_done = len(seg_times.get("卸货停留+完工", []))
    if n_done:
        print(f"\n完成 {n_done} 单，平均每单从 idle 到完工 "
              f"{grand / n_done:.1f} s（含所有分段）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
