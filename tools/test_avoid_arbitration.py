#!/usr/bin/env python3
"""避让仲裁的不变量测试（纯几何，不启动 Gazebo，秒级完成）。

为什么先写这个：第 25/26 节连续两次"改一个分支阈值"都在 Gazebo 里失败
（`avoid_cones`、`sat_yield_stop` 超时），两次共花掉 8 次运行。
原因是三个硬安全出口（sat_yield_stop / sat_escape_back / lidar_backup_static）
是**同一条互让规则的三个出口**，单独放宽一个只会把车推进另一个。

所以先把不变量写成测试，再动代码。这里复刻 `_avoid` 第 2 步的几何判据
（轮廓相交 + id 优先），对**随机位姿组合**断言：

  I1  不存在"所有车都被判定为必须停下"的配置
      —— 即至少有一台车被允许移动（否则整个车队互等成死锁）
  I2  一台车不会对"根本没和它相交"的同伴退让
      —— 即相交判据本身没有假阳性
  I3  两车轮廓不相交时，谁都不该被这条规则拦下

运行: python3 tools/test_avoid_arbitration.py
"""
from __future__ import annotations

import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _find_paths(start: str):
    d = start
    for _ in range(6):
        for rel in ("src/fleetflow_sim", "fleetflow_sim", "src"):
            cand = os.path.join(d, rel)
            if os.path.isfile(os.path.join(cand, "fleetflow_sim", "planner.py")):
                return cand
        d = os.path.dirname(d)
    raise SystemExit("找不到 fleetflow_sim 包目录")


sys.path.insert(0, _find_paths(os.path.dirname(HERE)))

from fleetflow_sim.traffic import (HULL_LEN, HULL_WID,  # noqa: E402
                                   obb_corners, obb_overlap)

MARGIN = 0.05


def corners(x, y, yaw):
    hx, hy = HULL_LEN / 2 + MARGIN, HULL_WID / 2 + MARGIN
    return obb_corners(x, y, yaw, hx, hy)


def std_stop_verdict(me, peers):
    """复刻 _avoid 第 2 步：返回 True 表示"我这条规则要求我停住"。

    逻辑与原实现一致：遍历同伴，只要与任何一台轮廓相交，且它 id 更大
    就硬停；id 更小则尝试脱离（这里只关心"是否被要求停"）。
    """
    rid, (x, y, yaw) = me
    for orid, (ox, oy, oyaw) in peers:
        if orid == rid:
            continue
        d = math.hypot(ox - x, oy - y)
        if d >= 0.356 * 2 + 2 * MARGIN:
            continue
        if obb_overlap(corners(x, y, yaw), corners(ox, oy, oyaw)):
            if orid > rid:
                return True
    return False


def main() -> int:
    rng = random.Random(20260914)
    fails = []

    # ---- 构造若干"挤在一起"的配置，检查是否存在全员停住的死锁 ----
    trials = 0
    deadlocks = 0
    for _ in range(4000):
        n = rng.choice((2, 3, 4))
        # 故意把车放在很小的范围内，制造大量相交
        base_x = rng.uniform(0.0, 20.0)
        base_y = rng.uniform(0.0, 14.0)
        poses = []
        for i in range(n):
            x = base_x + rng.uniform(-0.5, 0.5)
            y = base_y + rng.uniform(-0.5, 0.5)
            yaw = rng.uniform(-math.pi, math.pi)
            poses.append((i, (x, y, yaw)))
        trials += 1
        stopped = [std_stop_verdict(p, poses) for p in poses]
        if all(stopped):
            deadlocks += 1
            if len(fails) < 3:
                fails.append(("I1 全员停住", poses))

    print(f"I1 无全员停住：{trials - deadlocks}/{trials} 通过"
          f"（{deadlocks} 个配置里所有车都被要求停住）")

    # ---- I2/I3：不相交就不该被拦 ----
    fp = 0
    trials2 = 0
    for _ in range(4000):
        x, y = rng.uniform(0, 20), rng.uniform(0, 14)
        yaw = rng.uniform(-math.pi, math.pi)
        ox = x + rng.uniform(1.0, 3.0)      # 保证明显分开
        oy = y + rng.uniform(1.0, 3.0)
        oyaw = rng.uniform(-math.pi, math.pi)
        me = (0, (x, y, yaw))
        peers = [(0, me[1]), (1, (ox, oy, oyaw))]
        trials2 += 1
        if not obb_overlap(corners(x, y, yaw), corners(ox, oy, oyaw)):
            if std_stop_verdict(me, peers):
                fp += 1
    print(f"I2 不相交不误拦：{trials2 - fp}/{trials2} 通过"
          f"（{fp} 次假阳性）")

    print()
    if deadlocks:
        print("!! 发现死锁配置，示例：")
        for tag, poses in fails:
            print(f"   {tag}: " + "  ".join(
                f"R{r}({p[0]:.2f},{p[1]:.2f},{math.degrees(p[2]):.0f}°)"
                for r, p in poses))
        print()
        print("这正是实测里 R1 的 94% sat_yield_stop：多台车挤在一起时，")
        print("按 id 单向让行会让**所有**车都认为'该我等'，于是全队停死。")
        print("正确的仲裁必须保证：任一时刻至少有一台车被允许前进。")
        return 1
    print("全部不变量通过 —— 可以进入实测。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
