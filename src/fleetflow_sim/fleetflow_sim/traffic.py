"""交通协调层：ROS 1 版 ``CleanTaskScheduler`` 交通部分的完整移植。

为什么要移植
------------
ROS 2 版最初只做了"工位独占租约"（见 ``traffic_manager`` 的说明）。租约能
破坏"持有并等待"，所以不会出现**资源环**死锁；但它管不了**运动学拥堵**：
两台车在通道里对向相遇、三台车挤向同一个完工位、一台车被压成 0 速之后
再也没人管它。ROS 1 版跑得动，靠的正是这一层（原文件
``task_scheduler_ros_clean.py`` 里约 1200 行）。这里按同样的判据逐项搬过来。

分工
----
ROS 1 里调度器和导航在同一个进程，协调层直接写 ``cmd_vel``。ROS 2 把导航
拆到了每台车的控制器（它才拿得到 LiDAR 与纯追踪状态），所以这里改成
**"协调层下指令、控制器执行"**：

* 协调层决定：谁让谁、什么时候走、往哪儿让、要不要脱困；
* 控制器负责：把这些约束变成轮速，并保留自己的 LiDAR 安全层与看门狗。

对应关系（ROS 1 -> 这里）::

    update_region_tracking / find_dispersed_goal_in_region   -> 原样
    corridor_zones / corridor_priority_decision              -> 原样
    reserve_spacetime / check_spacetime_conflict             -> 原样
    detect_conflict_type / resolve_conflict_by_priority       -> 原样
    try_plan_safe_yield_path / _plan_escape_path              -> 原样
    global_deadlock_scan / force_global_deadlock_resolution   -> 原样
    try_force_pair_separation / trigger_pair_emergency_break   -> 原样
    update_robot_priority / get_effective_priority            -> 原样
    recover_stalled_robots                                    -> 改成下发 replan/脱困指令
    check_collision_nearby + build_emergency_escape_command   -> 改成下发脱困指令
    send_navigation_goal 的第 1/3 层（激光、PID 限速）         -> 留在控制器
"""
from __future__ import annotations

import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field

from . import layout
from .planner import Grid, plan

# ---------------------------------------------------------------- 避障参数
# 车体几何（与 agv.urdf.xacro 保持一致）。碰撞判据必须由几何推出，不能拍一个数：
# 车心距既不对又难调 —— 0.34 m 会真的撞上（车长 0.56 m），两车外接圆之和
# 0.712 m 又过于保守（并排本来只要 0.44 m）。
HULL_LEN = 0.56
HULL_WID = 0.44
# 轮廓外扩多少仍算"要撞了"。带余量的矩形一旦相交就是硬停。
AVOID_MARGIN = 0.05

# 间隙阈值（米）：**两车真实轮廓之间的净距**，也就是 closest_gap() 现在返回的量。
#
# 这里必须把"量"讲清楚，因为写错过一次：closest_gap() 早期拿**外扩过
# AVOID_MARGIN 的轮廓**算距离，而阈值是按真实净距标定的，两者恒差
# 2*AVOID_MARGIN = 0.10 m。后果是车心距 0.80 m（真实净距 0.24 m，根本碰不到）
# 被判成 0.14 m 开始减速，0.70 m（真实净距 0.14 m）直接硬停。
# 更糟的是 metrics 报的是真实净距，两个模块对同一对车给出不同的数，无法互证。
# 现在统一：closest_gap() 用裸轮廓，下面这些阈值就是真实净距。
COLLISION_CHECK_GAP = 0.35           # 真实净距小于此值开始分级减速
HARD_SAFETY_GAP = 0.10               # 真实净距小于此值视为已贴上 -> 停
CRITICAL_SAFETY_GAP = 0.0            # 真实净距为负（轮廓真重叠）-> 紧急脱离
COLLISION_ESCAPE_BLOCK_DISTANCE = 0.45   # 脱困时"旁边有车"的车心距判据（沿用 ROS1）
# 兼容旧名：脱困与让路逻辑里仍按"车心距"表示"旁边很近"
HARD_SAFETY_DISTANCE = 0.25
CRITICAL_SAFETY_DISTANCE = 0.15

# ---------------------------------------------------------------- 死锁参数
STUCK_DETECTION_WINDOW = 2.5        # 卡住检测时间窗口（秒）
STUCK_DISTANCE_THRESHOLD = 0.33     # 距离阈值（米）
DEADLOCK_RELEASE_DISTANCE = 0.80    # 判定死锁对已分离的距离阈值（米）
STUCK_TRIGGER_COUNT = 12            # 触发死锁判定的最小检测次数
DEADLOCK_RESOLVE_COOLDOWN = 12.0    # 死锁解锁后冷却时间（秒）
DEADLOCK_RETRY_WINDOW = 40.0        # 同一机器人对重复死锁计数窗口（秒）
PAIR_SEPARATION_HOLD_SEC = 8.0      # 双机分离保护时长（秒）
YIELD_PAIR_COOLDOWN_SEC = 2.5       # 普通让路动作去抖冷却（秒）
YIELD_REPEAT_WINDOW = 12.0          # 同一机器人重复让路统计窗口（秒）
MAX_CONSECUTIVE_YIELD_SAME_PEER = 3  # 连续对同 peer 让路阈值

# ---------------------------------------------------------------- 走廊令牌
CORRIDOR_TOKEN_TTL = 6.0            # 走廊令牌有效时长（秒）
CORRIDOR_LOG_THROTTLE_SEC = 2.0

# ---------------------------------------------------------------- 时空预约
RESERVATION_HORIZON = 8.0           # 预约时间窗口（秒）
RESERVATION_GRID_SIZE = 0.6         # 预约栅格大小（米）
RESERVATION_TIME_TOLERANCE = 0.8    # 时隙冲突容忍窗口（秒）

# ---------------------------------------------------------------- 优先级
WAIT_SPEED_THRESHOLD = 0.03         # 判定等待的速度阈值（m/s）
WAIT_BOOST_INTERVAL = 2.0           # 等待增益步长（秒）
WAIT_BOOST_STEP = 6                 # 每步等待增益分值
MAX_WAIT_PRIORITY_BOOST = 36        # 等待增益上限
PRIORITY_INHERITANCE_BONUS = 15     # 对方已让路时的继承加分
PROTECTED_PRIORITY_BONUS = 8        # 保护状态额外优先级

# ---------------------------------------------------------------- 卡死自愈
PERSISTENT_STALL_HORIZON = 8.0      # 连续等待多久算持久卡死（秒）
STALL_NEAR_TARGET_TOLERANCE = 0.75  # 距目标多近算"快到但进不去"（米）
STALL_REPLAN_COOLDOWN = 3.0         # 卡死重规划节流（秒）

# ---------------------------------------------------------------- 区域
# 收敛点：多台车会同向汇聚、需要分散落位的区域。
# ROS 1 是显式写死的字典，这里按 layout 的真实工位推出来。
REGIONS = {
    # 半径按**真实工位行的跨度**取：卡位在 y = 3/6/9/12，空筒取放位也是
    # 这四个高度，所以圆心放中间、半径要盖到 ±4.5；并条/粗纱只有 4.5/10.5
    # 两行，±3.0 就够。半径取小了会让边上的车落不进区域、分散策略失效；
    # 取大了会把相邻工序的车算进同一个区域（充电区半径就曾把梳棉待命位吞掉）。
    "empty_storage":   dict(center=(3.40, 7.50),  radius=4.60),
    "carding_waiting": dict(center=(5.55, 7.50),  radius=4.60),
    "drawing_waiting": dict(center=(12.75, 7.50), radius=3.20),
    "roving_waiting":  dict(center=(17.85, 7.50), radius=3.20),
    "red_storage":     dict(center=(22.95, 8.00), radius=3.00),
    "chargers":        dict(center=(5.45, 1.55),  radius=1.00),
}

# 狭窄通道：南北向的工序待命通道。三台机器/货架在对向车流之间留出的净宽
# 只有一车多一点，两台车对向挤进去必有一台要走回头路 —— 所以用单向令牌。
# bbox = (x_min, x_max, y_min, y_max)
CORRIDOR_ZONES = {
    "corridor_carding":  dict(bbox=(4.85, 6.25, 1.30, 14.60)),
    "corridor_drawing":  dict(bbox=(12.05, 13.45, 1.30, 14.60)),
    "corridor_roving":   dict(bbox=(17.15, 18.55, 1.30, 14.60)),
}


def obb_corners(x, y, yaw, hx, hy):
    """有向矩形的四个角点。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


def _project(poly, nx, ny):
    vals = [nx * p[0] + ny * p[1] for p in poly]
    return min(vals), max(vals)


def _sat_axes(A, B):
    """A、B 两个矩形的全部候选分离轴（每条边的法线，已单位化）。"""
    axes = []
    for poly in (A, B):
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            nx, ny = -(y2 - y1), (x2 - x1)
            n = math.hypot(nx, ny)
            if n >= 1e-9:
                axes.append((nx / n, ny / n))
    return axes


def obb_overlap(A, B) -> bool:
    """分离轴定理：两个有向矩形是否相交（与控制器用的是同一套判据）。"""
    for nx, ny in _sat_axes(A, B):
        a0, a1 = _project(A, nx, ny)
        b0, b1 = _project(B, nx, ny)
        if a1 < b0 or b1 < a0:
            return False
    return True


def _point_seg_dist(p, q1, q2) -> float:
    ex, ey = q2[0] - q1[0], q2[1] - q1[1]
    L2 = ex * ex + ey * ey
    if L2 < 1e-18:
        return math.hypot(p[0] - q1[0], p[1] - q1[1])
    t = max(0.0, min(1.0, ((p[0] - q1[0]) * ex + (p[1] - q1[1]) * ey) / L2))
    return math.hypot(p[0] - (q1[0] + t * ex), p[1] - (q1[1] + t * ey))


def obb_gap(A, B) -> float:
    """两个有向矩形的**净距**（米）：分离为正，相交为负（负值即穿透深度）。

    相交时必须返回**真实的穿透深度**，不能只返回一个"负号"。
    这一点踩过坑：早期实现相交分支用"顶点到对边的最近距离"近似，对两个矩形
    相互嵌入的情形，最近距离恒为 0 —— 于是 0.06 m 的浅重叠和 0.30 m 的深重叠
    都返回 -0.0000。后果是 `build_emergency_escape_command` 无法比较"往哪个
    方向退更能拉开距离"，所有候选看起来一样好，脱困退化成原地转，两台车
    永远分不开（实测 zone/种子2 t=8.1s，R2 与 R3 就此卡死）。
    分离轴上的最小重叠量才是正确的穿透深度。
    """
    axes = _sat_axes(A, B)
    if axes:
        min_overlap = float("inf")
        separated = False
        for nx, ny in axes:
            a0, a1 = _project(A, nx, ny)
            b0, b1 = _project(B, nx, ny)
            o = min(a1, b1) - max(a0, b0)
            if o <= 0.0:
                separated = True
                break
            min_overlap = min(min_overlap, o)
        if separated:
            # 分离：最小距离一定取在"某顶点到另一矩形某条边"上
            best = float("inf")
            for p in A:
                for k in range(4):
                    best = min(best, _point_seg_dist(p, B[k], B[(k + 1) % 4]))
            for p in B:
                for k in range(4):
                    best = min(best, _point_seg_dist(p, A[k], A[(k + 1) % 4]))
            return best
        return -min_overlap
    return float("inf")


@dataclass
class Directive:
    """下发给单台车的交通指令。"""

    speed_scale: float = 1.0
    hold: bool = False
    has_escape: bool = False
    escape_v: float = 0.0
    escape_w: float = 0.0
    replan: bool = False
    reason: str = ""
    # 让路临时路径。必须用 has_yield_path 显式区分"没有覆盖"和"覆盖已结束"，
    # 否则控制器无法判断什么时候该切回自己的任务路径。
    has_yield_path: bool = False
    yield_path: list = field(default_factory=list)


class TrafficLayer:
    """全部协调状态与算法。不含任何 ROS 依赖，便于单测。"""

    def __init__(self, num_robots: int, logger=None):
        self.num_robots = int(num_robots)
        self.log = logger
        self.grid = Grid()
        # 对照实验开关：True 时 collision_tier 退回旧的车心距判据。
        # 只由 traffic_manager 的 collision_criterion 参数设置。
        self.centre_criterion = False
        # 脱困策略开关（对照实验用）：
        #   True  = 旧行为"排除所有朝对方的线速度，剩不下就原地转"
        #   False = 新行为"选一个让净距最大的动作"（见 build_emergency_escape_command）
        self.escape_legacy = False
        self.reset()

    # ================================================================
    #  状态
    # ================================================================
    def reset(self):
        self.pos: dict[int, tuple] = {}
        self.yaw: dict[int, float] = {}
        self.speed: dict[int, float] = {}
        self.state: dict[int, str] = {}
        self.target: dict[int, tuple | None] = {}
        # self.path 是**控制器上报的**实际路径，协调层只读；
        # 让路路径必须单独放，否则下一帧就被控制器的上报覆盖掉。
        self.path: dict[int, list] = {}
        self.yield_paths: dict[int, list] = {}
        self.carrying: dict[int, bool] = {}

        # 区域
        self.region_robots = defaultdict(list)

        # 走廊令牌
        self.corridor_tokens: dict[str, dict] = {}
        self._corridor_log_t: dict[str, float] = {}

        # 时空预约
        self.reservations: dict[int, dict] = {}

        # 优先级 / 等待
        self.priorities: dict[int, int] = {}
        self.wait_since: dict[int, float | None] = {}

        # 让路
        self.yielding: dict[int, tuple] = {}          # rid -> (target, until)
        self.yield_pair_cooldown: dict[tuple, float] = {}
        self.pair_separation_active: dict[tuple, float] = {}
        self.last_yield_peer: dict[int, int] = {}
        self.last_yield_time: dict[int, float] = defaultdict(float)
        self.consecutive_yield: dict[int, int] = defaultdict(int)
        self.yield_count: dict[int, int] = defaultdict(int)

        # 死锁
        self.pair_stuck_history: dict[tuple, dict] = {}
        self.deadlock_being_resolved: dict[tuple, float] = {}
        self.pair_retry_count: dict[tuple, int] = defaultdict(int)
        self.pair_last_time: dict[tuple, float] = {}

        # 其它
        self.protected: set[int] = set()
        self.last_stall_replan: dict[int, float] = defaultdict(float)
        self.collision_active: dict[int, bool] = {}

        # 指标
        self.m = dict(
            deadlock=0, yield_total=0, yield_fallback=0, starvation_prevent=0,
            pair_separation=0, emergency_break=0, collision_event=0,
            corridor_grant=0, corridor_reuse=0, corridor_release=0,
            stall_replan=0, stall_near_target=0, escape=0, reservation_conflict=0,
        )

    # ================================================================
    #  输入
    # ================================================================
    def update_robot(self, rid, x, y, yaw, speed, state, target, path):
        self.pos[rid] = (float(x), float(y))
        self.yaw[rid] = float(yaw)
        self.speed[rid] = float(speed)
        self.state[rid] = str(state)
        self.target[rid] = (float(target[0]), float(target[1])) if target else None
        self.path[rid] = list(path or [])

    def forget_robot(self, rid):
        for d in (self.pos, self.yaw, self.speed, self.state, self.target,
                  self.path, self.yield_paths, self.wait_since):
            d.pop(rid, None)
        self.reservations.pop(rid, None)
        self.yielding.pop(rid, None)
        self.protected.discard(rid)

    def effective_path(self, rid):
        """当前真正在走的路径：让路路径优先，否则用控制器上报的路径。"""
        return self.yield_paths.get(rid) or self.path.get(rid) or []

    # ================================================================
    #  区域追踪与分散（ROS1: update_region_tracking 等）
    # ================================================================
    def get_region_name_for_position(self, pos):
        cands = []
        for name, info in REGIONS.items():
            cx, cy = info["center"]
            r = max(info["radius"], 1e-6)
            d = math.hypot(pos[0] - cx, pos[1] - cy)
            if d <= r:
                cands.append((d / r, d, name))
        if not cands:
            return None
        cands.sort(key=lambda t: (t[0], t[1]))
        return cands[0][2]

    def update_region_tracking(self):
        self.region_robots.clear()
        for rid, p in self.pos.items():
            name = self.get_region_name_for_position(p)
            if name:
                self.region_robots[name].append(rid)

    def find_dispersed_goal_in_region(self, region_name, preferred_goal=None):
        """在区域内挑一个离现有同伴最远的落位点（8 方向 × 4 距离采样）。"""
        if region_name not in REGIONS:
            return preferred_goal
        occupants = self.region_robots.get(region_name) or []
        if not occupants:
            return preferred_goal or REGIONS[region_name]["center"]

        cx, cy = REGIONS[region_name]["center"]
        radius = REGIONS[region_name]["radius"]
        cands = []
        for ang in range(0, 360, 45):
            rad = math.radians(ang)
            for d in (0.3, 0.6, 0.9, 1.2):
                if d < radius:
                    cands.append((cx + d * math.cos(rad), cy + d * math.sin(rad)))
        if preferred_goal:
            cands.append(tuple(preferred_goal))
        if not cands:
            return preferred_goal or (cx, cy)

        best, best_min = None, -1.0
        for pt in cands:
            if not self.is_position_free(pt[0], pt[1], pad=0.0):
                continue
            dmin = min(
                (math.hypot(pt[0] - self.pos[r][0], pt[1] - self.pos[r][1])
                 for r in occupants if r in self.pos),
                default=float("inf"),
            )
            if dmin > best_min:
                best_min, best = dmin, pt
        return best if best else (preferred_goal or (cx, cy))

    # ================================================================
    #  走廊单向令牌（ROS1: get_corridor_for_position 等）
    # ================================================================
    def get_corridor_for_position(self, pos):
        if not pos:
            return None
        x, y = pos
        for cid, info in CORRIDOR_ZONES.items():
            x0, x1, y0, y1 = info["bbox"]
            if x0 <= x <= x1 and y0 <= y <= y1:
                return cid
        return None

    def get_robot_corridor_state(self, rid):
        """返回 (走廊 ID, 行进方向符号)。"""
        if rid not in self.pos:
            return None, 0
        cid = self.get_corridor_for_position(self.pos[rid])
        if cid is None:
            return None, 0
        path = self.effective_path(rid)
        if not path:
            return cid, 0
        curr = self.pos[rid]
        nxt = path[0]
        dx, dy = nxt[0] - curr[0], nxt[1] - curr[1]
        if abs(dx) >= abs(dy):
            return cid, (1 if dx >= 0 else -1)
        return cid, (1 if dy >= 0 else -1)

    def cleanup_corridor_tokens(self, now):
        for cid in list(self.corridor_tokens):
            tok = self.corridor_tokens.get(cid, {})
            holder, until = tok.get("holder"), tok.get("until", 0.0)
            reason = None
            if now >= until:
                reason = "expired"
            elif holder not in self.pos:
                reason = "holder_missing"
            elif self.get_corridor_for_position(self.pos[holder]) != cid:
                reason = "holder_left"
            if reason:
                self.m["corridor_release"] += 1
                self._clog(f"release:{cid}",
                           f"corridor token released: {cid} reason={reason} holder=R{holder}")
                self.corridor_tokens.pop(cid, None)

    def _clog(self, key, msg):
        now = time.time()
        if now - self._corridor_log_t.get(key, 0.0) < CORRIDOR_LOG_THROTTLE_SEC:
            return
        self._corridor_log_t[key] = now
        self._log(msg)

    def corridor_priority_decision(self, r1, r2, now):
        """走廊单向令牌：返回 True 表示 r1 需要让路，None 表示不适用。"""
        c1, d1 = self.get_robot_corridor_state(r1)
        c2, d2 = self.get_robot_corridor_state(r2)
        if c1 is None or c2 is None or c1 != c2 or d1 == 0 or d2 == 0 or d1 == d2:
            return None

        tok = self.corridor_tokens.get(c1)
        if tok and tok.get("until", 0.0) > now:
            holder = tok.get("holder")
            if holder in (r1, r2):
                self.m["corridor_reuse"] += 1
                tok["until"] = now + CORRIDOR_TOKEN_TTL
                other = r2 if holder == r1 else r1
                self._clog(f"reuse:{c1}:{holder}",
                           f"corridor token reuse: {c1} holder=R{holder} pass=R{holder} block=R{other}")
                return holder != r1

        p1, _, _ = self.get_effective_priority(r1, now, peer_id=r2)
        p2, _, _ = self.get_effective_priority(r2, now, peer_id=r1)
        if p1 > p2:
            holder = r1
        elif p2 > p1:
            holder = r2
        else:
            holder = r1 if r1 < r2 else r2

        self.corridor_tokens[c1] = dict(
            holder=holder,
            direction=d1 if holder == r1 else d2,
            until=now + CORRIDOR_TOKEN_TTL,
        )
        self.m["corridor_grant"] += 1
        self._clog(f"grant:{c1}",
                   f"corridor token grant: {c1} holder=R{holder} "
                   f"block=R{r2 if holder == r1 else r1}")
        return holder != r1

    # ================================================================
    #  优先级与等待（ROS1: update_robot_priority 等）
    # ================================================================
    def update_robot_priority(self, rid):
        """载货中 +20；到站但未卸完 +10（ROS 1 用的是任务 status）。"""
        st = self.state.get(rid, "idle")
        if st in ("idle", "waiting_lease"):
            self.priorities[rid] = 0
            return
        p = 0
        if st in ("to_dropoff", "departing"):
            p = 20
        elif st in ("loading", "unloading"):
            p = 10
        elif st in ("to_pickup", "to_charger"):
            p = 5
        self.priorities[rid] = p

    def update_robot_wait_state(self, rid, now):
        """MOVING 且有目标/路径但长期低速 → 累计等待时间。"""
        st = self.state.get(rid, "idle")
        if st in ("idle", "loading", "unloading", "charging", "waiting_lease"):
            self.wait_since[rid] = None
            return
        if not self.path.get(rid) and self.target.get(rid) is None:
            self.wait_since[rid] = None
            return
        if self.speed.get(rid, 0.0) < WAIT_SPEED_THRESHOLD:
            if self.wait_since.get(rid) is None:
                self.wait_since[rid] = now
        else:
            self.wait_since[rid] = None

    def get_wait_duration(self, rid, now):
        t = self.wait_since.get(rid)
        return 0.0 if t is None else max(0.0, now - t)

    def get_effective_priority(self, rid, now, peer_id=None):
        """有效优先级 = 基础 + 等待增益 + 继承/保护加权（反饥饿）。"""
        self.update_robot_priority(rid)
        base = self.priorities.get(rid, 0)
        steps = int(self.get_wait_duration(rid, now) / WAIT_BOOST_INTERVAL)
        boost = min(MAX_WAIT_PRIORITY_BOOST, steps * WAIT_BOOST_STEP)
        eff = base + boost
        if rid in self.protected:
            eff += PROTECTED_PRIORITY_BONUS
        if peer_id is not None and peer_id in self.yielding:
            target, until = self.yielding[peer_id]
            if target == rid and now < until:
                eff += PRIORITY_INHERITANCE_BONUS
        return eff, base, boost

    def is_yielding_to(self, rid, target_id, now):
        info = self.yielding.get(rid)
        if not info:
            return False
        target, until = info
        return target == target_id and now < until

    @staticmethod
    def _pair_key(r1, r2):
        return (r1, r2) if r1 < r2 else (r2, r1)

    # ================================================================
    #  时空预约（ROS1: reserve_spacetime 等）
    # ================================================================
    def reserve_spacetime(self, rid, path, now):
        if not path or len(path) < 2:
            return
        v = max(0.3, self.speed.get(rid, 0.3))
        g = RESERVATION_GRID_SIZE
        nodes, edges = [], []
        t_acc = 0.0
        nodes.append((int(path[0][0] / g), int(path[0][1] / g), now))
        for i in range(1, len(path)):
            prev_pt, curr_pt = path[i - 1], path[i]
            dt = math.hypot(curr_pt[0] - prev_pt[0], curr_pt[1] - prev_pt[1]) / v
            t0 = now + t_acc
            t_acc += dt
            if t_acc > RESERVATION_HORIZON:
                break
            t1 = now + t_acc
            edges.append((int(prev_pt[0] / g), int(prev_pt[1] / g),
                          int(curr_pt[0] / g), int(curr_pt[1] / g), t0, t1))
            nodes.append((int(curr_pt[0] / g), int(curr_pt[1] / g), t1))
        self.reservations[rid] = dict(nodes=nodes, edges=edges)

    def clear_spacetime_reservation(self, rid):
        self.reservations.pop(rid, None)

    @staticmethod
    def _time_overlap(a0, a1, b0, b1, tol=0.0):
        return max(a0, b0) <= min(a1, b1) + tol

    def check_spacetime_conflict(self, rid, now):
        """近网格 + 时间近邻，或对向/同向换边冲突。"""
        if rid not in self.reservations:
            return False, None
        mine = self.reservations[rid]
        my_nodes, my_edges = mine["nodes"], mine["edges"]
        tol = RESERVATION_TIME_TOLERANCE

        for oid, other in self.reservations.items():
            if oid == rid:
                continue
            for mx, my, mt in my_nodes:
                if mt < now:
                    continue
                for ox, oy, ot in other["nodes"]:
                    if ot < now:
                        continue
                    if abs(mx - ox) <= 1 and abs(my - oy) <= 1 and abs(mt - ot) < tol:
                        return True, oid
            for ax, ay, bx, by, at0, at1 in my_edges:
                if at1 < now:
                    continue
                for ox, oy, px, py, ot0, ot1 in other["edges"]:
                    if ot1 < now:
                        continue
                    reverse = (ax == px and ay == py and bx == ox and by == oy)
                    same = (ax == ox and ay == oy and bx == px and by == py)
                    if (reverse or same) and self._time_overlap(at0, at1, ot0, ot1, tol * 0.5):
                        return True, oid
        return False, None

    # ================================================================
    #  冲突分类与裁决（ROS1: detect_conflict_type / resolve_conflict_by_priority）
    # ================================================================
    def detect_conflict_type(self, r1, r2):
        p1_, p2_ = self.effective_path(r1), self.effective_path(r2)
        if not (p1_ and p2_):
            return "none", 0.0, None
        p1, p2 = self.pos[r1], self.pos[r2]
        n1, n2 = p1_[0], p2_[0]
        ang1 = math.atan2(n1[1] - p1[1], n1[0] - p1[0])
        ang2 = math.atan2(n2[1] - p2[1], n2[0] - p2[0])
        diff = abs(math.degrees(math.atan2(math.sin(ang1 - ang2), math.cos(ang1 - ang2))))
        rel = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]) - ang1)
        rel = (rel + 180.0) % 360.0 - 180.0
        rel_pos = "front"
        if 45 < rel <= 135:
            rel_pos = "left"
        elif -135 <= rel < -45:
            rel_pos = "right"
        elif rel > 135 or rel < -135:
            rel_pos = "back"
        if diff < 60:
            return "following", diff, rel_pos
        if 60 <= diff < 120:
            return "perpendicular", diff, rel_pos
        if 150 <= diff <= 210:
            return "opposite", diff, rel_pos
        return "diagonal", diff, rel_pos

    def resolve_conflict_by_priority(self, r1, r2, now):
        """返回 True 表示 r1 应该让路。"""
        # 对方已在让我 → 我不该再让，避免双向振荡
        if self.is_yielding_to(r2, r1, now):
            return False
        if self.is_yielding_to(r1, r2, now):
            return True

        d = self.corridor_priority_decision(r1, r2, now)
        if d is not None:
            return d

        ctype, _, rel_pos = self.detect_conflict_type(r1, r2)
        p1, base1, boost1 = self.get_effective_priority(r1, now, peer_id=r2)
        p2, base2, boost2 = self.get_effective_priority(r2, now, peer_id=r1)
        winner = ("me" if p1 > p2 else "other" if p2 > p1 else ("me" if r1 < r2 else "other"))
        base_winner = ("me" if base1 > base2 else "other" if base2 > base1
                       else ("me" if r1 < r2 else "other"))
        if winner != base_winner and (boost1 > 0 or boost2 > 0):
            self.m["starvation_prevent"] += 1

        if ctype == "opposite":
            q1, q2 = self.effective_path(r1)[0], self.effective_path(r2)[0]
            d1 = math.hypot(q1[0] - self.pos[r1][0], q1[1] - self.pos[r1][1])
            d2 = math.hypot(q2[0] - self.pos[r2][0], q2[1] - self.pos[r2][1])
            if d1 < d2 * 0.8:
                return False
            if d2 < d1 * 0.8:
                return True
            return winner != "me"
        if ctype in ("perpendicular", "diagonal"):
            return winner != "me"
        if ctype == "following":
            return rel_pos == "front" and winner != "me"
        return winner != "me"

    # ================================================================
    #  让路路径规划
    # ================================================================
    def is_position_free(self, x, y, pad=0.0):
        f = layout.FIELD
        if not (f["x_min"] + pad <= x <= f["x_max"] - pad
                and f["y_min"] + pad <= y <= f["y_max"] - pad):
            return False
        for x0, y0, x1, y1 in layout.static_boxes():
            if x0 - pad <= x <= x1 + pad and y0 - pad <= y <= y1 + pad:
                return False
        return True

    def is_position_safe_for_yield(self, rid, target, min_dist=0.42):
        if rid not in self.pos:
            return False
        if not self.is_position_free(target[0], target[1], pad=0.30):
            return False
        for oid, opos in self.pos.items():
            if oid == rid:
                continue
            if math.hypot(target[0] - opos[0], target[1] - opos[1]) < min_dist:
                return False
        return True

    def _plan_to(self, rid, target, priority_ignored=None):
        """用共享栅格规划到 target（同伴按动态层绕开）。"""
        peers = [(p[0], p[1]) for r, p in self.pos.items() if r != rid]
        self.grid.set_dynamic(peers, radius=0.50)
        try:
            p = plan(self.grid, self.pos[rid], target)
        finally:
            self.grid.clear_dynamic()
        return p or []

    def try_plan_safe_yield_path(self, rid, avoid_rid=None, hold_seconds=4.0,
                                lateral_offsets=None, backward_offsets=None, now=None):
        """优先侧移、次选后移。成功返回路径，并在内部登记让路状态。"""
        if rid not in self.pos:
            return []
        now = now or time.time()
        curr = self.pos[rid]
        yaw = self.yaw.get(rid, 0.0)
        lateral_offsets = lateral_offsets or [0.40]
        backward_offsets = backward_offsets or [0.45]

        cands = []
        for off in lateral_offsets:
            cands.append((curr[0] - off * math.sin(yaw), curr[1] + off * math.cos(yaw)))
            cands.append((curr[0] + off * math.sin(yaw), curr[1] - off * math.cos(yaw)))
        for off in backward_offsets:
            cands.append((curr[0] - off * math.cos(yaw), curr[1] - off * math.sin(yaw)))

        for tgt in cands:
            if not self.is_position_safe_for_yield(rid, tgt):
                continue
            p = self._plan_to(rid, tgt)
            if len(p) >= 2:
                if avoid_rid is not None:
                    self.yielding[rid] = (avoid_rid, now + hold_seconds)
                return p
        return []

    def _plan_escape_path(self, rid, other_id, distance_candidates, side_candidates,
                          min_dist=0.35):
        if rid not in self.pos or other_id not in self.pos:
            return []
        curr, other = self.pos[rid], self.pos[other_id]
        vx, vy = curr[0] - other[0], curr[1] - other[1]
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            vx, vy = math.cos(self.yaw.get(rid, 0.0)), math.sin(self.yaw.get(rid, 0.0))
            norm = 1.0
        ux, uy = vx / norm, vy / norm
        px, py = -uy, ux
        for dist in distance_candidates:
            for side in side_candidates:
                tgt = (curr[0] + ux * dist + px * side, curr[1] + uy * dist + py * side)
                if not self.is_position_safe_for_yield(rid, tgt, min_dist=min_dist):
                    continue
                p = self._plan_to(rid, tgt)
                if len(p) >= 2:
                    return p
        return []

    def activate_yield_path(self, rid, target_id, path, hold_until):
        self.yield_paths[rid] = list(path)
        self.yielding[rid] = (target_id, hold_until)

    def clear_yield_path(self, rid):
        self.yield_paths.pop(rid, None)

    def is_robot_yielding(self, rid, now):
        """返回 (是否在让路窗口, 是否允许移动)。窗口结束就要求重规划。"""
        if rid not in self.yielding:
            return False, True
        target, until = self.yielding[rid]
        if now >= until:
            del self.yielding[rid]
            self.clear_yield_path(rid)
            return False, True
        return True, bool(self.yield_paths.get(rid))

    # ================================================================
    #  紧急脱困（ROS1: get_closest_robot / choose_escape_rotation 等）
    # ================================================================
    def get_closest_robot(self, rid):
        if rid not in self.pos:
            return None, float("inf")
        p = self.pos[rid]
        best, bd = None, float("inf")
        for oid, op in self.pos.items():
            if oid == rid:
                continue
            d = math.hypot(p[0] - op[0], p[1] - op[1])
            if d < bd:
                best, bd = oid, d
        return best, bd

    def get_relative_robot_sector(self, rid, other):
        if rid not in self.pos or other not in self.pos:
            return "unknown"
        p, o = self.pos[rid], self.pos[other]
        bearing = math.atan2(o[1] - p[1], o[0] - p[0])
        rel = math.atan2(math.sin(bearing - self.yaw.get(rid, 0.0)),
                         math.cos(bearing - self.yaw.get(rid, 0.0)))
        a = abs(rel)
        if a <= math.radians(60):
            return "front"
        if a >= math.radians(120):
            return "back"
        return "left" if rel > 0 else "right"

    def is_linear_motion_toward_robot(self, rid, other, lin):
        if abs(lin) < 1e-6 or rid not in self.pos or other not in self.pos:
            return False
        p, o = self.pos[rid], self.pos[other]
        vx, vy = o[0] - p[0], o[1] - p[1]
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            return True
        yaw = self.yaw.get(rid, 0.0)
        d = 1.0 if lin > 0 else -1.0
        return (math.cos(yaw) * d * vx / norm + math.sin(yaw) * d * vy / norm) > 0.25

    def is_linear_motion_toward_any_close_robot(self, rid, lin, distance=None):
        distance = HARD_SAFETY_DISTANCE if distance is None else distance
        if rid not in self.pos:
            return False
        p = self.pos[rid]
        for oid, op in self.pos.items():
            if oid == rid:
                continue
            if math.hypot(p[0] - op[0], p[1] - op[1]) < distance \
                    and self.is_linear_motion_toward_robot(rid, oid, lin):
                return True
        return False

    def choose_escape_rotation(self, rid, distance=None):
        """左右哪边同伴压力小就往哪边转。"""
        distance = COLLISION_ESCAPE_BLOCK_DISTANCE if distance is None else distance
        if rid not in self.pos:
            return 1.0
        p = self.pos[rid]
        yaw = self.yaw.get(rid, 0.0)
        pressure = 0.0
        for oid, op in self.pos.items():
            if oid == rid:
                continue
            d = math.hypot(p[0] - op[0], p[1] - op[1])
            if d >= distance:
                continue
            bearing = math.atan2(op[1] - p[1], op[0] - p[0])
            rel = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
            pressure += math.sin(rel) / max(d, 0.05)
        return -1.0 if pressure > 0.05 else 1.0

    def build_emergency_escape_command(self, rid, closest):
        """脱困速度：**保证一定给出一个"把两车拉开"的动作**，实在拉不开才原地转。

        旧实现是"先把'朝对方开'的方向全排除掉，剩不下就原地转"。当两台车已经
        压在一起时这个判据自相矛盾：任何方向上的移动都会让**某一点**更靠近对方，
        于是候选全被排除，只能原地转 —— 而原地转不会改变净距，下一帧继续被判定
        为重叠，永远转下去。
        实测现场（`zone` 种子 2，t=8.1s）：R2 与 R3 车心距 0.92 m、指令
        `emergency_from_R3`、`esc=1`，而 v 恒为 0.000，两台车就此卡死。

        新实现改成"选一个让净距**最大化**的动作"：对每个候选按短时预测估计
        净距变化，取最优；只要有任何候选能让净距变大就执行它。只有真的全都
        变差（例如被机台和同伴夹住）才退化为原地转 —— 那种情况靠转朝向改变
        几何，下一帧再试。
        """
        if closest is None or rid not in self.pos or closest not in self.pos:
            return 0.0, 0.0
        if self.escape_legacy:
            # 旧行为：凡是"朝对方开"的方向一律排除，剩不下就原地转。
            # 两台车已经压在一起时这个判据自相矛盾 —— 任何移动都会让某一点
            # 更靠近对方，于是候选全被排除，只能原地转，净距永不改变。
            sector0 = self.get_relative_robot_sector(rid, closest)
            cands0 = {
                "front": [-0.16, 0.12], "back": [0.16, -0.12],
                "left": [-0.12, 0.12], "right": [-0.12, 0.12],
            }.get(sector0, [])
            block = max(HARD_SAFETY_DISTANCE, COLLISION_ESCAPE_BLOCK_DISTANCE)
            for lin in cands0:
                if not self.is_linear_motion_toward_any_close_robot(rid, lin,
                                                                    distance=block):
                    return lin, 0.0
            return 0.0, self.choose_escape_rotation(rid, distance=block)
        sector = self.get_relative_robot_sector(rid, closest)
        # 候选线速度按"面朝哪边"给出：正 = 沿车头，负 = 倒车。
        # 倒车是最有效的脱离手段之一，所以第一个候选就把它列上。
        cands = {
            "front": [-0.16, 0.12], "back": [0.16, -0.12],
            "left": [-0.12, 0.12], "right": [-0.12, 0.12],
        }.get(sector, [0.12, -0.12])

        hx, hy = HULL_LEN / 2 + AVOID_MARGIN, HULL_WID / 2 + AVOID_MARGIN
        me = obb_corners(*self.pos[rid], self.yaw.get(rid, 0.0), hx, hy)
        other = obb_corners(*self.pos[closest], self.yaw.get(closest, 0.0), hx, hy)
        gap0 = obb_gap(me, other)

        yaw = self.yaw.get(rid, 0.0)
        dt = 0.5                       # 预测 0.5 s 后的位置
        best_lin, best_gap = None, gap0
        for lin in cands:
            nx = self.pos[rid][0] + lin * math.cos(yaw) * dt
            ny = self.pos[rid][1] + lin * math.sin(yaw) * dt
            after = obb_corners(nx, ny, yaw, hx, hy)
            g = obb_gap(after, other)
            if g > best_gap + 1e-6:
                best_gap, best_lin = g, lin
        if best_lin is not None:
            return best_lin, 0.0
        # 全都拉不开：靠转朝向换几何，下一帧重新判
        return 0.0, self.choose_escape_rotation(rid)

    def closest_gap(self, rid):
        """最近同伴及其与我的**真实轮廓净距**（米）。

        用**裸轮廓**（不加 AVOID_MARGIN），这样返回值与
        `metrics.min_robot_gap_m` 是同一个量，也和下面那些阈值同一单位。
        AVOID_MARGIN 只属于控制器里那层"带余量就算撞上"的硬判据，
        不该混进几何距离本身 —— 混进来过一次，导致两个模块对同一对车
        报出相差 0.10 m 的两个数。
        """
        if rid not in self.pos:
            return None, float("inf")
        hx, hy = HULL_LEN / 2, HULL_WID / 2
        me = obb_corners(*self.pos[rid], self.yaw.get(rid, 0.0), hx, hy)
        best, bg = None, float("inf")
        for oid, op in self.pos.items():
            if oid == rid:
                continue
            # 粗筛：车心距大于外接圆之和 + 阈值就不可能是最近的
            if math.hypot(self.pos[rid][0] - op[0], self.pos[rid][1] - op[1]) > 2.0:
                continue
            other = obb_corners(op[0], op[1], self.yaw.get(oid, 0.0), hx, hy)
            g = obb_gap(me, other)
            if g < bg:
                best, bg = oid, g
        return best, bg

    def collision_tier(self, rid):
        """(是否有碰撞风险, 速度系数, 是否紧急) —— ROS1 check_collision_nearby。

        判据从"车心距"改成了**轮廓净距**，与控制器里的有向矩形硬安全判据一致。
        原因见文件头 COLLISION_CHECK_GAP 的注释：车心距在密集车队里会把
        正常并排/跟车误判成"快撞了"，全队被压到 0.3 倍速。
        """
        if rid not in self.pos:
            return False, 1.0, False
        if self.centre_criterion:
            # 旧判据（存档对照）：车心距 1.5 m 起减速、0.25 m 硬停。
            closest, d = self.get_closest_robot(rid)
            if closest is None:
                return False, 1.0, False
            if d < CRITICAL_SAFETY_DISTANCE:
                return True, 0.0, True
            if d < HARD_SAFETY_DISTANCE:
                return True, 0.35, False
            if d < 1.5:
                return True, max(0.3, d / 1.5), False
            return False, 1.0, False
        closest, gap = self.closest_gap(rid)
        if closest is None:
            return False, 1.0, False
        # 硬停交回控制器（它才有 LiDAR 与运动方向信息），这里只做分级限速
        if gap >= COLLISION_CHECK_GAP:
            return False, 1.0, False
        if gap <= CRITICAL_SAFETY_GAP:
            return True, 0.0, True
        if gap < HARD_SAFETY_GAP:
            return True, 0.0, False
        # 真实净距 0.10~0.35 m 之间线性降速：0.35 -> 1.0，0.10 -> 0.35
        fac = 0.35 + 0.65 * (gap - HARD_SAFETY_GAP) / (COLLISION_CHECK_GAP - HARD_SAFETY_GAP)
        return True, max(0.0, min(1.0, fac)), False

    # ================================================================
    #  死锁扫描与解锁（ROS1: global_deadlock_scan 等）
    # ================================================================
    def global_deadlock_scan(self, now):
        ids = sorted(self.pos.keys())
        navigable = ("to_pickup", "to_dropoff", "to_charger", "departing")
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                r1, r2 = ids[i], ids[j]
                key = self._pair_key(r1, r2)

                if (self.state.get(r1) not in navigable
                        and self.state.get(r2) not in navigable):
                    self.pair_stuck_history.pop(key, None)
                    continue

                sep_until = self.pair_separation_active.get(key, 0.0)
                if now < sep_until:
                    continue
                if key in self.pair_separation_active:
                    del self.pair_separation_active[key]

                if key in self.deadlock_being_resolved:
                    if now < self.deadlock_being_resolved[key]:
                        continue
                    del self.deadlock_being_resolved[key]

                p1, p2 = self.pos[r1], self.pos[r2]
                d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

                # 已充分分离：清掉历史重试，避免 retry 单调上升
                if d > DEADLOCK_RELEASE_DISTANCE:
                    self.pair_retry_count.pop(key, None)
                    self.pair_last_time.pop(key, None)

                if d < STUCK_DISTANCE_THRESHOLD:
                    rec = self.pair_stuck_history.setdefault(
                        key, dict(first=now, last=now, cnt=0, min_d=d))
                    rec["last"] = now
                    rec["cnt"] += 1
                    rec["min_d"] = min(rec["min_d"], d)

                    if (now - rec["first"] >= STUCK_DETECTION_WINDOW
                            and rec["cnt"] >= STUCK_TRIGGER_COUNT):
                        if now - self.pair_last_time.get(key, 0.0) > DEADLOCK_RETRY_WINDOW:
                            self.pair_retry_count[key] = 0
                        self.pair_retry_count[key] += 1
                        retry = self.pair_retry_count[key]
                        self.pair_last_time[key] = now

                        self.m["deadlock"] += 1
                        cooldown = DEADLOCK_RESOLVE_COOLDOWN + min(8.0, 1.5 * max(0, retry - 1))
                        self.deadlock_being_resolved[key] = now + cooldown
                        self.pair_stuck_history.pop(key, None)
                        self._log(f"deadlock: R{r1} <-> R{r2} d={d:.2f}m retry={retry}")
                        self.force_global_deadlock_resolution(r1, r2, retry, now)
                elif key in self.pair_stuck_history:
                    self.pair_stuck_history.pop(key, None)

    def force_global_deadlock_resolution(self, r1, r2, retry_count=1, now=None):
        """让路方升级链：safe-yield → 双机分离 → 紧急刹停。"""
        now = now or time.time()
        p1, _, b1 = self.get_effective_priority(r1, now, peer_id=r2)
        p2, _, b2 = self.get_effective_priority(r2, now, peer_id=r1)

        decision = self.corridor_priority_decision(r1, r2, now)
        if decision is True:
            yielder, winner = r1, r2
        elif decision is False:
            yielder, winner = r2, r1
        elif p1 < p2:
            yielder, winner = r1, r2
        elif p1 > p2:
            yielder, winner = r2, r1
        else:
            yielder, winner = (r1, r2) if r1 > r2 else (r2, r1)

        self._log(f"resolve: R{yielder} yields to R{winner} retry={retry_count}")

        if retry_count >= 6:
            self.trigger_pair_emergency_break(
                r1, r2, hold_sec=min(4.0, 2.5 + 0.2 * retry_count), now=now)
            return
        if retry_count >= 2 and self.try_force_pair_separation(r1, r2, retry_count, now):
            return

        hold_secs = min(14.0, 4.0 + 2.0 * max(0, retry_count - 1))
        lateral = [0.40, 0.55, 0.75] if retry_count >= 3 else [0.40]
        backward = [0.45, 0.65] if retry_count >= 3 else [0.45]

        safe = self.try_plan_safe_yield_path(
            yielder, avoid_rid=winner, hold_seconds=hold_secs,
            lateral_offsets=lateral, backward_offsets=backward, now=now)
        if safe:
            self.activate_yield_path(yielder, winner, safe, now + hold_secs)
            self.yield_count[yielder] += 1
            self.m["yield_total"] += 1
            self._log(f"deadlock resolve: R{yielder} safe-yield to R{winner}")
            return

        if self.try_force_pair_separation(yielder, winner, retry_count, now):
            return

        # 兜底：原地等待。这里不"盲目后退"—— 后退方向未必安全。
        self.yielding[yielder] = (winner, now + hold_secs)
        # 清掉让路路径而不是清 self.path —— self.path 是控制器上报的，
        # 写它既没意义，下一帧也会被覆盖。没有让路路径 + 在让路窗口内
        # => 控制器收到 hold，就是"原地等待"。
        self.clear_yield_path(yielder)
        self.yield_count[yielder] += 1
        self.m["yield_total"] += 1
        self.m["yield_fallback"] += 1
        if retry_count >= 3:
            self._log(f"deadlock: forcing R{winner} to re-plan (retry={retry_count})")

    def try_force_pair_separation(self, r1, r2, retry_count=1, now=None):
        """双机同时主动分离：比单边 hold 更快打破互推僵局。"""
        if r1 not in self.pos or r2 not in self.pos:
            return False
        now = now or time.time()
        key = self._pair_key(r1, r2)
        hold = min(14.0, PAIR_SEPARATION_HOLD_SEC + 1.0 * max(0, retry_count - 1))
        dists = [0.65, 0.90, 1.15]
        sides = [0.0, 0.35, -0.35, 0.55, -0.55]

        p1 = self._plan_escape_path(r1, r2, dists, sides, min_dist=0.32)
        p2 = self._plan_escape_path(r2, r1, dists, sides, min_dist=0.32)
        if not p1 and not p2:
            return False

        if p1:
            self.activate_yield_path(r1, r2, p1, now + hold)
        if p2:
            self.activate_yield_path(r2, r1, p2, now + hold)

        self.clear_spacetime_reservation(r1)
        self.clear_spacetime_reservation(r2)
        self.pair_separation_active[key] = now + hold
        self.deadlock_being_resolved[key] = max(
            self.deadlock_being_resolved.get(key, 0.0), now + hold + 2.0)
        self.m["pair_separation"] += 1
        self._log(f"pair separation: R{r1}<->R{r2} retry={retry_count} "
                  f"p1={'Y' if p1 else 'N'} p2={'Y' if p2 else 'N'}")
        return True

    def trigger_pair_emergency_break(self, r1, r2, hold_sec=2.5, now=None):
        """高重试兜底：双机短时刹停，清预约，等重新规划。"""
        now = now or time.time()
        key = self._pair_key(r1, r2)
        for r in (r1, r2):
            self.clear_yield_path(r)
        self.yielding[r1] = (r2, now + hold_sec)
        self.yielding[r2] = (r1, now + hold_sec)
        self.clear_spacetime_reservation(r1)
        self.clear_spacetime_reservation(r2)
        self.pair_separation_active[key] = now + hold_sec
        self.deadlock_being_resolved[key] = max(
            self.deadlock_being_resolved.get(key, 0.0), now + hold_sec + 2.0)
        self.m["emergency_break"] += 1
        self._log(f"emergency break pair R{r1}<->R{r2} hold={hold_sec:.1f}s")

    # ================================================================
    #  普通让路动作（ROS1: execute_yield_action）
    # ================================================================
    def execute_yield_action(self, r1, r2, now):
        last_peer = self.last_yield_peer.get(r1)
        if last_peer == r2 and now - self.last_yield_time.get(r1, 0.0) <= YIELD_REPEAT_WINDOW:
            self.consecutive_yield[r1] += 1
        else:
            self.consecutive_yield[r1] = 1
        self.last_yield_peer[r1] = r2
        self.last_yield_time[r1] = now
        self.yield_pair_cooldown[self._pair_key(r1, r2)] = now + YIELD_PAIR_COOLDOWN_SEC

        # 对同一对象反复让路 → 升级为死锁级处理
        if self.consecutive_yield[r1] >= MAX_CONSECUTIVE_YIELD_SAME_PEER:
            self.force_global_deadlock_resolution(r1, r2, self.consecutive_yield[r1], now)
            return

        safe = self.try_plan_safe_yield_path(r1, avoid_rid=r2, hold_seconds=4.0, now=now)
        if safe:
            self.activate_yield_path(r1, r2, safe, now + 4.0)
            self.yield_count[r1] += 1
            self.m["yield_total"] += 1
            return

        if self.try_force_pair_separation(r1, r2, self.consecutive_yield[r1], now):
            return

        self.yielding[r1] = (r2, now + 3.0)
        self.yield_count[r1] += 1
        self.m["yield_total"] += 1
        self.m["yield_fallback"] += 1
        self.clear_yield_path(r1)

    # ================================================================
    #  卡死自愈（ROS1: recover_stalled_robots）
    # ================================================================
    def recover_stalled_robots(self, now, directives):
        """
        ROS 1 在这里直接改路径/触发到达；ROS 2 改成下发指令：
          * 快到目标却进不去 -> 解除一切压制 + 请求重规划（让它能落位）
          * 远未到目标       -> 节流请求重规划
          * 重规划也不行     -> 方向安全的紧急脱困
        """
        for rid in list(self.pos.keys()):
            if rid not in self.pos:
                continue
            if self.state.get(rid) not in ("to_pickup", "to_dropoff", "to_charger", "departing"):
                continue
            target = self.target.get(rid)
            if target is None:
                continue
            if self.get_wait_duration(rid, now) < PERSISTENT_STALL_HORIZON:
                continue

            d_tgt = math.hypot(target[0] - self.pos[rid][0], target[1] - self.pos[rid][1])
            d = directives[rid]

            if d_tgt <= STALL_NEAR_TARGET_TOLERANCE:
                self.m["stall_near_target"] += 1
                self.clear_spacetime_reservation(rid)
                self.wait_since[rid] = None
                d.hold = False
                d.speed_scale = 1.0
                d.replan = True
                d.reason = "stalled_near_target"
                self._log(f"R{rid} stalled near target ({d_tgt:.2f}m), forcing replan")
                continue

            if now - self.last_stall_replan.get(rid, 0.0) < STALL_REPLAN_COOLDOWN:
                continue
            self.last_stall_replan[rid] = now
            self.clear_spacetime_reservation(rid)
            self.yielding.pop(rid, None)
            self.m["stall_replan"] += 1
            d.replan = True
            d.reason = "stalled_replan"

            # 重规划也走不动，且轮廓已经贴得很近 → 直接给方向安全的脱困速度。
            # 用净距而不是车心距：车心距 0.45 m 时两台车其实已经嵌在一起了。
            closest, gap = self.closest_gap(rid)
            if closest is not None and gap < 0.30:
                v, w = self.build_emergency_escape_command(rid, closest)
                d.has_escape = True
                d.escape_v, d.escape_w = v, w
                d.reason = f"stalled_escape_from_R{closest}"
                self.m["escape"] += 1
                self._log(f"R{rid} stalled {self.get_wait_duration(rid, now):.1f}s, "
                          f"escape from R{closest} (gap={gap:.2f}m)")
            else:
                # 把 gap 一起打出来：否则只能看到"没触发脱困"，
                # 却不知道到底是因为身边没人（closest 为 None）还是
                # 离得还不够近（gap >= 0.30）—— 排查时这两种原因
                # 指向完全不同的修法，日志里必须先分得开。
                self._log(f"R{rid} stalled {self.get_wait_duration(rid, now):.1f}s, "
                          f"replan (nearest={closest} gap="
                          f"{'inf' if gap == float('inf') else f'{gap:.2f}m'})")

    # ================================================================
    #  周期运行
    # ================================================================
    def cleanup_runtime_navigation_states(self, now):
        for rid in list(self.pos.keys()):
            if self.state.get(rid) not in ("to_pickup", "to_dropoff", "to_charger", "departing"):
                self.clear_spacetime_reservation(rid)
            yi = self.yielding.get(rid)
            if yi and now >= yi[1] and not self.yield_paths.get(rid):
                self.yielding.pop(rid, None)

    def coordinate(self, rid, now):
        """单车的协调判定，返回 Directive（对应 ROS1 navigate_along_path 的协调段）。"""
        d = Directive()

        is_yield, can_move = self.is_robot_yielding(rid, now)
        yielding_mode = is_yield and can_move
        if is_yield and not can_move:
            self.clear_spacetime_reservation(rid)
            d.hold = True
            d.reason = "yield_hold"
            return d

        if not self.effective_path(rid):
            self.clear_spacetime_reservation(rid)
            return d

        if yielding_mode:
            # 让路执行阶段不再触发新的让路判定，避免振荡
            d.has_yield_path = True
            d.yield_path = list(self.yield_paths[rid])
            return d

        # 预约用"实际在走的路径"（让路期间是让路路径，否则是任务路径）
        self.reserve_spacetime(rid, self.effective_path(rid), now)
        conflict, other = self.check_spacetime_conflict(rid, now)
        if conflict and other is not None:
            pkey = self._pair_key(rid, other)
            if now < self.pair_separation_active.get(pkey, 0.0):
                conflict = False
            elif self.is_yielding_to(other, rid, now):
                conflict = False
            elif now < self.yield_pair_cooldown.get(pkey, 0.0):
                conflict = False
        if conflict and other is not None:
            self.m["reservation_conflict"] += 1
            if self.resolve_conflict_by_priority(rid, other, now):
                self.execute_yield_action(rid, other, now)
                if rid in self.yielding and self.yield_paths.get(rid):
                    d.has_yield_path = True
                    d.yield_path = list(self.yield_paths[rid])
                else:
                    d.hold = True
                    d.reason = f"yield_to_R{other}"
                return d

        # 分级限速（ROS1 的 check_collision_nearby 三层）
        has_coll, speed_fac, emg = self.collision_tier(rid)
        if has_coll:
            if not self.collision_active.get(rid, False):
                self.m["collision_event"] += 1
                self.collision_active[rid] = True
            closest, _ = self.get_closest_robot(rid)
            if emg:
                v, w = self.build_emergency_escape_command(rid, closest)
                d.has_escape = True
                d.escape_v, d.escape_w = v, w
                d.hold = False
                d.speed_scale = 0.0
                d.reason = f"emergency_from_R{closest}"
                self.m["escape"] += 1
                self.clear_spacetime_reservation(rid)
                return d
            d.speed_scale = min(d.speed_scale, speed_fac)
            d.reason = "collision_slow"
        else:
            self.collision_active[rid] = False
        return d

    def tick(self, now=None):
        """跑一个协调周期，返回 {rid: Directive}。"""
        now = now or time.time()
        self.cleanup_corridor_tokens(now)
        self.cleanup_runtime_navigation_states(now)
        self.update_region_tracking()
        self.global_deadlock_scan(now)
        for rid in list(self.pos.keys()):
            self.update_robot_wait_state(rid, now)

        directives = {rid: Directive() for rid in self.pos}
        self.recover_stalled_robots(now, directives)
        for rid in list(self.pos.keys()):
            d = self.coordinate(rid, now)
            base = directives[rid]
            # 合并：hold 与 escape 优先级最高
            if d.hold:
                base.hold, base.reason = True, d.reason
            if d.has_escape:
                base.has_escape, base.escape_v, base.escape_w = True, d.escape_v, d.escape_w
                base.reason = base.reason or d.reason
            base.speed_scale = min(base.speed_scale, d.speed_scale)
            base.replan = base.replan or d.replan
            if d.has_yield_path:
                base.has_yield_path = True
                base.yield_path = d.yield_path
            if not base.reason:
                base.reason = d.reason
        return directives

    def stats(self):
        return dict(self.m)

    def _log(self, msg):
        if self.log:
            self.log.info(f"[traffic] {msg}")
