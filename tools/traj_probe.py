#!/usr/bin/env python3
"""记录车队轨迹，用来回答"拥堵到底发生在哪"。

为什么需要它：结论一直停在"A 通道太窄"，但通道宽度可以直接量，拥堵位置却
从没测过。日志里有脱困次数、有卡死时长，**没有位置** —— 于是"加宽哪条通道"
只能靠猜。这个工具把 `(x, y, state, speed)` 按固定周期落盘，事后就能算出：

  * 每台车在每个网格里停留（低速）了多久 -> 热力图，找出真正的瓶颈区；
  * 低速点集中在哪些 x 区间 -> 对应哪条通道。

用法（仿真在跑时）:
    python3 tools/traj_probe.py out.jsonl --seconds 200 --hz 5
之后:
    python3 tools/traj_probe.py --analyse out.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fleetflow_interfaces.msg import RobotStatus

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=50)

SLOW = 0.05          # m/s，低于此值算"停着"
CELL = 1.0           # m，热力图网格边长


class Traj(Node):
    def __init__(self):
        super().__init__("traj_probe")
        self.robots: dict[int, RobotStatus] = {}
        self.create_subscription(RobotStatus, "/fleet/robots", self._on, Q)

    def _on(self, m):
        self.robots[m.robot_id] = m


def record(out: str, seconds: float, hz: float) -> int:
    rclpy.init()
    n = Traj()
    period = 1.0 / max(0.1, hz)
    t0 = time.time()
    nxt = t0
    rows = 0
    with open(out, "w", encoding="utf-8") as f:
        while time.time() - t0 < seconds:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() < nxt:
                continue
            nxt += period
            for rid, r in sorted(n.robots.items()):
                f.write(json.dumps(dict(
                    t=round(time.time() - t0, 3), rid=rid,
                    x=round(r.x, 3), y=round(r.y, 3), yaw=round(r.yaw, 3),
                    spd=round(r.speed, 3), state=r.state, task=r.task_id,
                    tx=round(r.target_x, 3), ty=round(r.target_y, 3))) + "\n")
                rows += 1
    n.destroy_node()
    rclpy.shutdown()
    return rows


def analyse(path: str) -> int:
    by_robot: dict[int, list[dict]] = defaultdict(list)
    for line in open(path, encoding="utf-8"):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        by_robot[d["rid"]].append(d)
    if not by_robot:
        print("没有数据"); return 1

    # 按网格累计"停着"的时长
    slow_time: dict[tuple[int, int], float] = defaultdict(float)
    total_time: dict[tuple[int, int], float] = defaultdict(float)
    xs_stop: list[float] = []
    per_robot_stop = {}
    for rid, rows in by_robot.items():
        rows.sort(key=lambda r: r["t"])
        stop = 0.0
        for a, b in zip(rows, rows[1:]):
            dt = b["t"] - a["t"]
            if dt <= 0 or dt > 2.0:        # 丢掉采样缺口
                continue
            k = (int(math.floor(a["x"] / CELL)), int(math.floor(a["y"] / CELL)))
            total_time[k] += dt
            if a["spd"] < SLOW:
                slow_time[k] += dt
                stop += dt
                xs_stop.append(a["x"])
        per_robot_stop[rid] = stop

    print(f"总时长: {max(r['t'] for rows in by_robot.values() for r in rows):.0f} s"
          f" · 车辆 {len(by_robot)}")
    print("每台车累计停着的时间（秒）:")
    for rid in sorted(per_robot_stop):
        print(f"  R{rid}: {per_robot_stop[rid]:6.1f}")

    # 停得最久的网格
    worst = sorted(slow_time.items(), key=lambda kv: -kv[1])[:10]
    print(f"\n停着最久的 {len(worst)} 个网格（{CELL:.0f}×{CELL:.0f} m）:")
    print(f"  {'格子(x,y)':>18s} {'停着(s)':>9s} {'占比':>7s}")
    for (gx, gy), t in worst:
        share = t / max(total_time[(gx, gy)], 1e-6)
        print(f"  ({gx*CELL:5.1f},{gy*CELL:5.1f}) {t:9.1f} {share:6.0%}")

    # 按 x 分桶：哪条纵向通道最容易堵
    if xs_stop:
        print("\n按 x 区间统计'停着'的采样点（定位是哪条通道）:")
        buckets: dict[int, int] = defaultdict(int)
        for x in xs_stop:
            buckets[int(x // 2.0) * 2] += 1
        for bx in sorted(buckets):
            bar = "#" * max(1, int(buckets[bx] / max(buckets.values()) * 40))
            print(f"  x {bx:2d}–{bx+2:2d} m: {buckets[bx]:5d} {bar}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default="/tmp/traj.jsonl")
    ap.add_argument("--seconds", type=float, default=200.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--analyse", action="store_true",
                    help="不录制，分析已有的 jsonl")
    args = ap.parse_args()
    if args.analyse:
        return analyse(args.out)
    n = record(args.out, args.seconds, args.hz)
    print(f"写入 {n} 行 -> {args.out}")
    return analyse(args.out)


if __name__ == "__main__":
    sys.exit(main())
