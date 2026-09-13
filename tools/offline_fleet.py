#!/usr/bin/env python3
"""离线复现"整场零产出"：不启 Gazebo，秒级跑完，用来设计避让层。

为什么需要它
------------
第 26、27 节三次改避让层，每次都要跑 3 次 Gazebo（约 10 分钟）才知道好坏，
两次更差、一次无效。而第 32 节发现"2 台车一次 13 单、一次 0 单"这种
**同配置相反结果**，靠 Gazebo 采样根本追不动。

所以这里把**真实的几何判据**（traffic 的 obb_*、以及 _avoid 的相交规则）
搬进一个最小多车仿真：同样的位姿、同样的冲突判定、同样的"id 小者脱离/大者等待"
仲裁，用合成的运动学推进。目标不是精确复刻 Gazebo，而是**复现失败模式**：
如果离线也能跑出"两台车互相卡住、一趟都走不完"，就可以在这里快速试改法，
再拿最有希望的版本去 Gazebo 验证。

用法:
    python3 tools/offline_fleet.py            # 扫 20 个种子，报零产出比例
    python3 tools/offline_fleet.py -n 4 -s 1  # 指定车数与种子，打印轨迹
"""
from __future__ import annotations

import argparse
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
                                   obb_corners, obb_gap, obb_overlap)

MARGIN = 0.05
CONE = True          # 是否启用前方锥形互让（_avoid 第 3 步）
V_MAX = 0.85
W_MAX = 1.6


def corners(x, y, yaw):
    return obb_corners(x, y, yaw, HULL_LEN / 2 + MARGIN, HULL_WID / 2 + MARGIN)


class Bot:
    def __init__(self, rid, x, y, yaw):
        self.rid = rid
        self.x, self.y, self.yaw = x, y, yaw
        self.goal = None
        self.done = False
        self.stuck_ticks = 0
        # 执行器状态：Gazebo 里轮子无法瞬时达到指令速度，
        # 而纯逻辑（logic_only）是瞬时到位的。这个差别很可能是关键。
        self.v_act = 0.0
        self.w_act = 0.0

    def corners(self):
        return corners(self.x, self.y, self.yaw)


def front_cone(bot, bots, slow=1.30, stop=0.60):
    """复刻 _avoid 第 3 步：前方锥形内的同伴导致减速/停车。

    注意 stop=0.60 m 与待命排间距（1.35 m）是同一量级 —— 这在实车里
    是最可疑的一条：起步时前车就在锥形内。
    """
    for other in bots:
        if other.rid == bot.rid:
            continue
        dx, dy = other.x - bot.x, other.y - bot.y
        d = math.hypot(dx, dy)
        if d > slow or d < 1e-6:
            continue
        bearing = math.atan2(dy, dx) - bot.yaw
        ang = abs(math.atan2(math.sin(bearing), math.cos(bearing)))
        if ang > 0.9:
            continue
        if other.rid < bot.rid:
            return (0.14 if d < stop else 0.40), f"front_slow(rid<{other.rid})"
        if d < stop * 0.8:
            return 0.0, f"cone_back_stop({other.rid})"
        return 0.50, f"cone_back_slow({other.rid})"
    return None, None


def avoid(bot, bots):
    """复刻 _avoid 第 2 步的相交仲裁（当前线上的规则）。

    返回 (v, w, tag)。tag 用于统计各分支命中。
    """
    me = bot.corners()
    for other in bots:
        if other.rid == bot.rid:
            continue
        d = math.hypot(other.x - bot.x, other.y - bot.y)
        if d >= 0.356 * 2 + 2 * MARGIN:
            continue
        if not obb_overlap(me, other.corners()):
            continue
        # 相交
        if other.rid > bot.rid:
            return 0.0, 0.0, "sat_yield_stop"
        # id 小者负责脱离：沿自身朝向/侧向挑一个能拉开距离的
        away = math.hypot(bot.x - other.x, bot.y - other.y) or 1e-9
        for name, vv, ww in (("fwd", 0.12, 0.0), ("fwd_r", 0.10, -0.9),
                             ("fwd_l", 0.10, 0.9), ("back", -0.10, 0.0)):
            nx = bot.x + vv * math.cos(bot.yaw) * 0.5
            ny = bot.y + vv * math.sin(bot.yaw) * 0.5
            if math.hypot(nx - other.x, ny - other.y) > away + 1e-3:
                return vv, ww, f"sat_escape_{name}"
        return 0.0, 0.0, "sat_stop"
    return None, None, "clear"


def step_bot(bot, bots, dt=0.1, rng=None, noise=0.0):
    """一个控制周期：先取纯追踪期望速度，再交给避让仲裁。

    `noise` 模拟 Gazebo 与纯逻辑的关键差别：里程计/朝向估计带噪声，
    于是车不可能像理想积分那样精确沿直线走。调大它看会不会把系统推进
    "反复相交 -> 整场卡住"的失败模式。
    """
    if bot.done:
        return "done"
    tx, ty = bot.goal
    dx, dy = tx - bot.x, ty - bot.y
    d = math.hypot(dx, dy)
    if d < 0.20:
        bot.done = True
        return "arrived"
    bearing = math.atan2(dy, dx)
    alpha = math.atan2(math.sin(bearing - bot.yaw), math.cos(bearing - bot.yaw))
    # 与 planner.PurePursuit 一致（含 d96ce88 的"先对准"分支）
    if abs(alpha) > 1.2:
        v, w = 0.0, max(-1.2, min(1.2, 2.0 * alpha))
        tag = "align"
    else:
        v = V_MAX * max(0.25, min(1.0, math.cos(alpha)))
        w = max(-W_MAX, min(W_MAX, 2.2 * alpha))
        tag = "pursuit"
    av, aw, atag = avoid(bot, bots)
    if av is not None:
        v, w, tag = av, aw, atag
    elif CONE:
        cv, ctag = front_cone(bot, bots)
        if cv is not None:
            v, tag = min(v, cv), ctag
    # 执行器一阶滞后：实际速度向指令速度逼近，时间常数 tau
    tau = 0.35
    a = min(1.0, dt / max(tau, 1e-6))
    bot.v_act += (v - bot.v_act) * a
    bot.w_act += (w - bot.w_act) * a

    # 积分用**实际**速度（而非指令速度）
    bot.x += bot.v_act * math.cos(bot.yaw) * dt
    bot.y += bot.v_act * math.sin(bot.yaw) * dt
    bot.yaw += bot.w_act * dt
    if rng is not None and noise > 0.0:
        # 位置与朝向的估计噪声：Gazebo 里必然存在，纯逻辑里没有
        bot.x += rng.gauss(0.0, noise * 0.02)
        bot.y += rng.gauss(0.0, noise * 0.02)
        bot.yaw += rng.gauss(0.0, noise * 0.05)
    if abs(v) < 1e-6 and abs(w) < 1e-6:
        bot.stuck_ticks += 1
    else:
        bot.stuck_ticks = 0
    return tag


def run(n=2, seed=1, ticks=1200, verbose=False, noise=0.0, misalign=0.0,
        same_goal=False, east=False, cone=True, corridor=False):
    """把 n 台车放在待命排里出发。

    `misalign`：出生朝向相对目标方向的随机偏置（弧度上限）。实车里
    出生朝向是固定的 +x，而目标方向各不相同，所以天然存在大偏置；
    纯逻辑因为积分理想、路径恰好顺着头朝向，几乎不会触发"先对准"分支。
    `same_goal`：让所有车抢**同一个**泊位，制造必然的收敛冲突。
    """
    global CONE
    CONE = cone
    rng = random.Random(seed)
    bots = []
    for i in range(n):
        x = 9.60 + 1.35 * i + rng.uniform(-0.05, 0.05)
        y = 1.55 + rng.uniform(-0.05, 0.05)
        yaw0 = rng.uniform(-misalign, misalign) if misalign > 0 else 0.0
        bots.append(Bot(i, x, y, yaw0))
    # 目标：空筒库的几个泊位（与实车一致，都在西侧）
    if corridor:
        # 窄通道对头：两台车在宽 1.15 m 的通道里相向而行。
        # 车体 0.44 m 宽，两侧各留 0.355 m —— 单台能过，两台迎面**过不去**。
        # 这正是 layout 里 wait_x=5.55 与 machine_x=7.00 之间那 1.15 m。
        for i, b in enumerate(bots):
            if i % 2 == 0:
                b.x, b.y, b.yaw = 5.20, 6.00, 0.0
                b.goal = (6.60, 6.00)
            else:
                b.x, b.y, b.yaw = 6.60, 6.00, math.pi
                b.goal = (5.20, 6.00)
        # 通道两侧的墙（机台与待命位边界），用 y 范围表示
    else:
        goals = ([(22.95, 5.50 + 2.5 * i) for i in range(4)] if east
                 else [(3.40, 3.00), (3.40, 6.00), (3.40, 9.00), (3.40, 12.00)])
        for i, b in enumerate(bots):
            b.goal = goals[0] if same_goal else goals[i % len(goals)]

    tags = {}
    nrng = random.Random(seed * 7919)
    for t in range(ticks):
        for b in bots:
            tag = step_bot(b, bots, rng=nrng, noise=noise)
            tags[tag] = tags.get(tag, 0) + 1
        if all(b.done for b in bots):
            break

    arrived = sum(1 for b in bots if b.done)
    return dict(arrived=arrived, n=n, ticks=t + 1, tags=tags, bots=bots)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=2)
    ap.add_argument("-s", "--seed", type=int, default=1)
    ap.add_argument("--sweep", type=int, default=0, help="扫这么多种子")
    ap.add_argument("--noise", type=float, default=0.0, help="里程计噪声强度")
    ap.add_argument("--misalign", type=float, default=0.0,
                    help="出生朝向相对目标的随机偏置上限（弧度）")
    ap.add_argument("--same-goal", action="store_true", help="所有车抢同一泊位")
    ap.add_argument("--no-cone", action="store_true", help="关闭前方锥形互让")
    ap.add_argument("--corridor", action="store_true",
                    help="窄通道对头相遇（两侧是机台，宽仅 1.15 m）")
    ap.add_argument("--east", action="store_true",
                    help="目标改在待命排**东侧**（模拟新车在东侧取料，"
                         "全排必须同向鱼贯而出）")
    args = ap.parse_args()

    if args.sweep:
        print(f"离线扫描：{args.n} 台车 × {args.sweep} 个种子  "
              f"noise={args.noise} misalign={args.misalign} "
              f"same_goal={args.same_goal}\n")
        zero = 0
        for s in range(1, args.sweep + 1):
            r = run(n=args.n, seed=s, noise=args.noise,
                    misalign=args.misalign, same_goal=args.same_goal,
                    east=args.east, cone=not args.no_cone,
                    corridor=args.corridor)
            if r["arrived"] == 0:
                zero += 1
            print(f"  seed={s:>3}  到达 {r['arrived']}/{r['n']}  "
                  f"用了 {r['ticks']} tick")
        print(f"\n  零产出（一台都没到）: {zero}/{args.sweep}")
        print("  若这里能复现，就可以在秒级迭代避让规则，再拿最有希望的版本去 Gazebo。")
        return 0

    r = run(n=args.n, seed=args.seed, verbose=True, noise=args.noise,
            misalign=args.misalign, same_goal=args.same_goal,
            east=args.east, cone=not args.no_cone, corridor=args.corridor)
    print(f"{args.n} 台车 seed={args.seed}: 到达 {r['arrived']}/{r['n']}，"
          f"{r['ticks']} tick")
    print("分支命中:", dict(sorted(r["tags"].items(), key=lambda kv: -kv[1])))
    print()
    for b in r["bots"]:
        d = math.hypot(b.goal[0] - b.x, b.goal[1] - b.y)
        print(f"  R{b.rid} pos=({b.x:6.2f},{b.y:6.2f}) yaw={math.degrees(b.yaw):+7.1f}° "
              f"goal={b.goal} 距目标 {d:5.2f} m done={b.done}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
