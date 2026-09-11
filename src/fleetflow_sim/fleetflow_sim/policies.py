"""任务分配策略（MRTA）。

把"哪台车去搬哪一单"形式化成一个多机器人任务分配问题：
给定待分配任务集合 T 与空闲车队 R，选择映射 f: T × R -> {执行, 不执行}，
使某个目标（通常是 makespan 或平均流程时间）最小。

本模块提供三个可切换的策略，用于对比实验：

1. ``random``     —— 随机分配。作为下界基线，说明"随便派"有多差。
2. ``nearest``    —— 请求车辆取"离自己最近的待办任务"。完全去中心化，
                     但只看单台车的局部信息，容易出现"近处任务被远处的车抢走"。
3. ``ssi``        —— 顺序单件拍卖（Sequential Single-Item Auction）。
                     每轮在所有 (空闲车, 待办任务) 组合里挑全局代价最小的一对
                     分配出去，重复直到没有可分配的组合。这是市场拍卖类
                     MRTA 方法的标准基线（Lagoudakis et al., 2005），
                     集中式但在本规模下代价可以忽略。

代价函数默认是"车辆到取货点的欧氏距离"，可换成 A* 实际路径长度。
"""
from __future__ import annotations

import math
import random
from typing import Dict, Iterable, List, Optional, Tuple

POLICIES = ("random", "nearest", "ssi")


def travel_cost(robot_xy: Tuple[float, float], task) -> float:
    """车辆到任务取货点的行驶代价。距离越短越好。"""
    return math.hypot(task.source_x - robot_xy[0], task.source_y - robot_xy[1])


def pick_random(candidates: List, robot_xy, rng: random.Random):
    return rng.choice(candidates) if candidates else None


def pick_nearest(candidates: List, robot_xy, rng: random.Random):
    """去中心化：请求者拿离自己最近的一单。"""
    if not candidates:
        return None
    return min(candidates, key=lambda t: travel_cost(robot_xy, t))


def pick_ssi(candidates: List, idle_robots: Dict[int, Tuple[float, float]],
             rng: random.Random) -> Optional[Tuple[int, object]]:
    """顺序单件拍卖：返回 (robot_id, task)，即本轮应当成交的一对。

    每台空闲车对自己的"运输成本"投标，成本最低的组合中标。
    与 ``nearest`` 的区别是它同时考虑所有空闲车，而不是只看发问的那一台。
    """
    if not candidates or not idle_robots:
        return None
    best = None
    for rid, xy in idle_robots.items():
        for t in candidates:
            c = travel_cost(xy, t)
            # 平局时用优先级、再用车 id 打破，保证结果可复现
            key = (c, t.priority, t.task_id, rid)
            if best is None or key < best[0]:
                best = (key, rid, t)
    return (best[1], best[2]) if best else None
