"""单台 AGV 的控制器：真实车队行为的完整状态机。

相比"取货→送货"两段式，这里补齐了真实 AGV 车队必备的几件事：

1. **完整 TF 树**：每台车发布 ``map → robot_<id>/odom → robot_<id>/base_footprint``，
   配合每车一个 ``robot_state_publisher``（frame_prefix）与桥接进来的 joint_states，
   每台车在 TF 上都是一个独立、可被 RViz/Nav2 直接消费的机器人。
2. **交通租约**：停靠工位前必须拿到租约；驶离工位后立刻释放。
   车辆同时最多持有一个租约，且只在"不持有任何租约"的前提下申请下一个，
   从结构上破坏死锁的 hold-and-wait 条件（详见 traffic_manager.py）。
3. **车车互让**：订阅车队位姿，对前方近距离同伴减速或停车；
   对称僵局用机器人 id 做确定性优先级打破（id 小的先行）。
4. **LiDAR 安全层**：前向扇区出现障碍直接停车，是避障的最后一道保险。
5. **电量与充电**：按里程耗电，低于阈值自动去充电桩排队、充电、回岗。
6. **卡死看门狗**：定速状态下长时间无位移则重新规划；仍无解则上报任务失败，
   由工厂主控重新派单——真实系统里这就是"车辆故障后任务回收"。

状态机：
    idle → (低电量) → to_charger → charging → idle
    idle → to_pickup → loading → departing → to_dropoff → unloading → idle
    任何移动状态都可能进入 waiting_lease（等目标工位空出来）
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Path

from fleetflow_interfaces.msg import RobotStatus, TrafficDirective, TransportTask
from fleetflow_interfaces.srv import AcquireLease, ReleaseLease, RequestTask

from . import layout
from .planner import Grid, PurePursuit, plan
from .qos import command_qos, sensor_qos, state_qos

_GRID = None


def shared_grid() -> Grid:
    global _GRID
    if _GRID is None:
        _GRID = Grid()
    return _GRID


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _obb_corners(x, y, yaw, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


def _obb_overlap(A, B) -> bool:
    """分离轴定理：两个有向矩形是否相交。"""
    for poly in (A, B):
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            nx, ny = -(y2 - y1), (x2 - x1)
            n = math.hypot(nx, ny)
            if n < 1e-9:
                continue
            nx, ny = nx / n, ny / n
            pa = [nx * q[0] + ny * q[1] for q in A]
            pb = [nx * q[0] + ny * q[1] for q in B]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("start_x", 1.0)
        self.declare_parameter("start_y", 5.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("dwell_s", 1.2)
        self.declare_parameter("use_internal_kinematics", False)
        # 出库错峰：第 i 台车等 i*start_stagger_s 秒再开始领任务
        self.declare_parameter("start_stagger_s", 0.0)
        # 安全 / 车队行为
        # 安全距离必须由**车体几何**推出来，不能拍一个数：
        # 车体 0.56×0.44 m，外接圆半径 0.356 m，所以两车在任意朝向下不重叠
        # 至少需要 0.712 m 的**车心距**。曾经写死 0.34 m —— 那是主动把车开到
        # 必然重叠的距离上，Gazebo 里就会真的撞在一起。
        self.hull_len, self.hull_wid = 0.56, 0.44
        self.hull_r = math.hypot(self.hull_len, self.hull_wid) / 2.0   # 0.356 m
        self.hull_diag = 2 * self.hull_r                              # 0.712 m
        # 碰撞判据的安全余量：车体各外扩这么多，仍算"要撞了"
        self.declare_parameter("avoid_margin_m", 0.05)
        # 锥形互让负责"礼貌"，真正的防撞由有向矩形判据兜底。
        # 这两个值保持与 experiments/ 里那批实验一致，否则文档中的数字就与代码脱节。
        self.declare_parameter("avoid_slow_m", 1.30)
        self.declare_parameter("avoid_stop_m", 0.60)
        # 前方锥形互让（本地礼貌限速）。协调层已经接管车车让行，默认关闭；
        # 打开可用于对照实验（见 _avoid 第 3 步的注释）。
        self.declare_parameter("avoid_cones", False)
        self.declare_parameter("scan_stop_m", 0.38)
        self.declare_parameter("stuck_timeout_s", 10.0)
        # 电量
        self.declare_parameter("battery_drain_per_m", 0.55)
        # 车体几何：LiDAR 安装点相对车体中心的纵向偏移，用于逐角度屏蔽自身回波
        self.hull_len, self.hull_wid, self.lidar_dx = 0.56, 0.44, 0.18
        # 凸出车体矩形、且落在雷达扫描平面上的部件，按 (cx, cy, r) 圆列出。
        # 数值取自 agv.urdf.xacro：轮子 origin y=±sep/2=±0.21、半径 0.075
        # （轮心 z=-0.07，扫描面 z≈0.115 与轮相割，所以确实会被扫到）；
        # 万向轮在车尾 (x=-0.22, r=0.045)，同样列出以覆盖车尾回波。
        sep = 0.42
        self.self_circles = [
            (0.0, +sep / 2, 0.075),
            (0.0, -sep / 2, 0.075),
            (-0.22, 0.0, 0.045),
        ]
        self.declare_parameter("scan_self_mask_m", 0.06)
        self.declare_parameter("battery_low_pct", 30.0)
        self.declare_parameter("battery_resume_pct", 88.0)
        self.declare_parameter("charge_rate_pct_s", 3.0)
        # 排障开关：把 _avoid 各分支的累计次数打出来。默认关，避免刷屏。
        self.declare_parameter("debug_avoid", False)
        # 排障开关：把"前方有近障碍"的原始 scan 帧写到这个目录（空=关）
        self.declare_parameter("debug_scan_dir", "")

        self.rid = int(self.get_parameter("robot_id").value)
        self.ns = f"robot_{self.rid}"
        self.dwell = float(self.get_parameter("dwell_s").value)
        self.self_kin = bool(self.get_parameter("use_internal_kinematics").value)
        self.avoid_cones = bool(self.get_parameter("avoid_cones").value)
        # 错峰起飞时刻：按 id 递增，避免整排出库时同时抢道
        stagger = float(self.get_parameter("start_stagger_s").value)
        self.start_at = time.time() + stagger * self.rid

        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.yaw = float(self.get_parameter("start_yaw").value)
        self.x0, self.y0, self.yaw0 = self.x, self.y, self.yaw

        self.state = "idle"
        self.task: TransportTask | None = None
        self.held: str | None = None          # 当前持有的租约
        self._pending_lease = ""
        self._lease_kind = "station"
        self.timer_until = 0.0
        self.grid = shared_grid()
        self.pursuit = PurePursuit()
        self.total_len = 0.0
        self.seen_len = 0.0
        self.battery = 100.0
        self.charging_events = 0
        self.replans = 0
        self.failures = 0
        self.yields = 0

        # 卡死检测
        self._last_progress_xy = (self.x, self.y)
        self._last_progress_t = time.time()
        self._stuck_strikes = 0
        # LiDAR 安全
        self._scan_min = 99.0
        self._scan_left = 99.0
        self._scan_front = 99.0
        self._scan_right = 99.0
        # 车队
        self.peers: dict[int, RobotStatus] = {}
        # _avoid 分支计数：默认只累积，由 debug_dump 参数决定是否打出来
        self._avoid_hist: dict[str, int] = {}
        self._dbg_dir = str(self.get_parameter("debug_scan_dir").value)
        self._dbg_frames: list = []
        # 交通协调层下发的指令与它的时间戳
        self._traffic: TrafficDirective | None = None
        self._traffic_t = 0.0
        self._traffic_replan_t = 0.0
        self._yield_active = False

        # ---- ROS 接口 ----
        self.pub_cmd = self.create_publisher(Twist, f"/{self.ns}/cmd_vel", command_qos(10))
        self.pub_status = self.create_publisher(RobotStatus, "/fleet/robots", state_qos(20))
        self.pub_done = self.create_publisher(Int32, "/factory/completed", state_qos(20))
        self.pub_failed = self.create_publisher(TransportTask, "/factory/task_status", state_qos(20))
        # 剩余规划路径：RViz 可视化 / 录制回放都要用，所以照发不误（很小，2 Hz）
        self.pub_path = self.create_publisher(Path, f"/{self.ns}/path", state_qos(2))
        if not self.self_kin:
            self.create_subscription(Odometry, f"/{self.ns}/odom", self.on_odom, sensor_qos(10))
            self.create_subscription(LaserScan, f"/{self.ns}/scan", self.on_scan, sensor_qos(5))
        self.create_subscription(RobotStatus, "/fleet/robots", self.on_peer, state_qos(20))
        # 交通协调层的指令：谁让谁、什么时候停、往哪让。控制器只负责执行。
        self.create_subscription(
            TrafficDirective, f"/{self.ns}/traffic", self.on_traffic, state_qos(10))

        self.cli_task = self.create_client(RequestTask, "/scheduler/request_task")
        self.cli_acquire = self.create_client(AcquireLease, "/traffic/acquire")
        self.cli_release = self.create_client(ReleaseLease, "/traffic/release")
        self.tf = TransformBroadcaster(self)

        self.create_timer(0.1, self.loop)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(0.2, self.broadcast_tf)
        self.create_timer(0.5, self.publish_path)
        if bool(self.get_parameter("debug_avoid").value):
            self.create_timer(10.0, self._dump_avoid)
        if self._dbg_dir:
            self.create_timer(20.0, self._dump_scan_frames)
        self.get_logger().info(
            f"{self.ns} up at ({self.x:.1f},{self.y:.1f}) "
            f"[{'internal kinematics' if self.self_kin else 'gazebo odom'}]"
        )

    # ---------- 输入 ----------
    def on_odom(self, m: Odometry):
        p = m.pose.pose.position
        self.x, self.y = p.x + self.x0, p.y + self.y0     # odom 相对出生点
        self.yaw = yaw_from_quat(m.pose.pose.orientation) + self.yaw0

    def _self_range(self, ang: float) -> float:
        """从雷达到**整车轮廓**在给定方向上的距离（车体系，前方为 +x）。

        为什么要算这个：LiDAR 装在车头 (x=+0.18)，而车体本身就在它的视场里。
        实测中这些**自身回波**让车永久停在原地（v=0 → 看门狗判卡死 → 放弃任务
        → 全线停摆）。真实 AGV 的做法是按轮廓逐角度屏蔽自身回波，而不是拍一个
        常数距离：常数要么漏（屏蔽不掉车尾）、要么瞎（把真障碍也屏蔽了）。

        轮廓必须包含**凸出车体矩形之外的部件**，否则它们会被当成真障碍：
        轮子是 0.075 半径、装在 y=±0.21 的圆柱，外缘到 y=±0.285 —— 比车体
        半宽 0.22 还外凸 0.065 m。从雷达 (0.18,0) 看过去，两侧轮子在
        **0.283 m / ±56°** 处；只按矩形算出的遮挡距离是 0.112 m，那一对回波
        就会被误判成"车头前 0.28 m 有障碍物"。

        实测结论（别再来回试）：**单台车静止在空地上，全周最小回波 0.92 m，
        也就是自身轮廓根本不在视场内** —— 雷达装在车头足够靠前。所以正常的
        直行并不会被自身回波挡住；这一层是给"贴住同伴/机台时车体扭转、
        轮子扫进视场"这类边缘情形兜底的。真正的历史故障是规划器与物理世界
        不一致（见 tools/build_world.py 与 layout.static_boxes 的差集），
        不要把这里的掩膜当成主因去调。
        """
        lx = self.lidar_dx
        dx, dy = math.cos(ang), math.sin(ang)
        best = float("inf")

        # 1) 车体矩形：与四条边求交，取最近的正向交点
        hx, hy = self.hull_len / 2.0, self.hull_wid / 2.0
        for wall_x in (-hx, hx):
            if abs(dx) > 1e-9:
                t = (wall_x - lx) / dx
                y = dy * t
                if t > 0 and -hy - 1e-9 <= y <= hy + 1e-9:
                    best = min(best, t)
        for wall_y in (-hy, hy):
            if abs(dy) > 1e-9:
                t = (wall_y - 0.0) / dy
                x = lx + dx * t
                if t > 0 and -hx - 1e-9 <= x <= hx + 1e-9:
                    best = min(best, t)

        # 2) 凸出部件（两侧轮子 / 后面万向轮）：按圆处理，取正向交点
        for cx, cy, r in self.self_circles:
            ox, oy = lx - cx, 0.0 - cy        # 从圆心指向雷达
            b = ox * dx + oy * dy
            c = ox * ox + oy * oy - r * r
            disc = b * b - c
            if disc < 0.0:
                continue
            sq = math.sqrt(disc)
            for t in (b - sq, b + sq):
                if t > 0:
                    best = min(best, t)
                    break
        return 0.0 if best == float("inf") else best

    def on_scan(self, m: LaserScan):
        """只关心前向扇区的最小距离，作为最后一道安全闸。

        两处必须做对，否则会变成"永远停住"而不是"安全"：

        1. **忽略物理上不可能的近距离回波。** 车体半宽 0.22 m、全长 0.56 m，
           LiDAR 又装在车头往前 0.18 m 处，所以任何**外部**物体都不可能出现在
           距雷达 0.24 m 以内而不已经嵌进车体里。实测中反复出现 0.15 m 的回波
           （量程下限 0.12 m，能通过 range_min 过滤），它把车钉死在原地：
           看门狗判定卡死 → 重规划 → 仍卡死 → 放弃任务 → 全线停摆。
        2. **只看运动方向。** 侧后方的近距离物体不该让一台正在直行的车停下，
           那是车车互让锥形区的职责。
        """
        if not m.ranges:
            return
        n = len(m.ranges)
        margin = float(self.get_parameter("scan_self_mask_m").value)
        rmin = float(m.range_min)

        def sector_min(lo, hi):
            """扇区内**自身轮廓以外**的最小距离；没有有效回波返回 99。"""
            best = 99.0
            for i in range(lo, hi):
                r = m.ranges[i]
                if not (rmin < r < m.range_max):
                    continue
                ang = m.angle_min + i * m.angle_increment
                if r > self._self_range(ang) + margin:    # 自身轮廓以外的才算障碍
                    best = min(best, r)
            return best

        # 三等分：左 / 前 / 右。用于"前方受阻时朝空的一侧转出去"。
        # ROS 1 的 check_laser_obstacle 就是这么分的，这里保持同一套判据。
        third = max(1, n // 3)
        self._scan_left = sector_min(0, third)
        self._scan_front = sector_min(third, 2 * third)
        self._scan_right = sector_min(2 * third, n)
        # 急停只看 ±40°：侧后方的近距离物体不该让一台直行的车停下。
        half = max(1, int(n * 0.11))
        self._scan_min = sector_min(n // 2 - half, n // 2 + half)
        # 排障：把"前方有近障碍"的那一帧各角度回波落盘，用于反推到底是什么挡住
        if self._dbg_dir and self._scan_min < 0.55 and len(self._dbg_frames) < 40:
            self._dbg_frames.append(dict(
                t=time.time(), x=self.x, y=self.y, yaw=self.yaw, state=self.state,
                scan_min=self._scan_min, front=self._scan_front,
                left=self._scan_left, right=self._scan_right,
                ranges=[round(r, 3) if math.isfinite(r) else None for r in m.ranges],
                angle_min=m.angle_min, angle_inc=m.angle_increment))

    def on_peer(self, m: RobotStatus):
        self.peers[m.robot_id] = m

    def on_traffic(self, m: TrafficDirective):
        self._traffic = m
        self._traffic_t = time.time()

    # ---------- 交通指令执行 ----------
    def _apply_traffic(self, v: float, w: float, now: float,
                       local_v: float = None, local_w: float = None,
                       pursuit_v: float = None, pursuit_w: float = None):
        """把协调层的约束落到 (v, w) 上。协调层决定"让不让"，这里只管执行。

        指令带时间戳：协调层若挂了或丢包，超过 1 秒就按"无指令"处理。
        否则一台车会被最后一帧 hold 永久压住 —— 那是比死锁更难查的故障。
        """
        d = self._traffic
        if d is None or now - self._traffic_t > 1.0:
            if self._yield_active:
                self._yield_active = False
                self._restore_task_path()
            return v, w

        # --- 让路路径覆盖 ---
        if d.has_yield_path and len(d.yield_x) >= 2:
            if not self._yield_active:
                self.pursuit.set_path(list(zip(d.yield_x, d.yield_y)))
                self._yield_active = True
                self.get_logger().info(
                    f"{self.ns}: yield path ({len(d.yield_x)} pts) reason={d.reason}")
        elif self._yield_active:
            # 让路窗口结束 → 切回自己的任务路径
            self._yield_active = False
            self._restore_task_path()
            self.get_logger().info(f"{self.ns}: yield done, resuming task path")

        # --- 重规划请求 ---
        if d.replan and now - self._traffic_replan_t > 1.5:
            self._traffic_replan_t = now
            tx, ty = self._target_xy()
            if tx is not None:
                self.replans += 1
                self._go_to(tx, ty, self.state, reset_strikes=False)

        # --- 直接速度覆盖（紧急脱困，方向安全性已由协调层判定）---
        if d.has_escape:
            return float(d.escape_v), float(d.escape_w)

        if d.hold:
            self.yields += 1
            return 0.0, 0.0

        # ------------------------------------------------------------------
        # 协调层覆盖"本地把自己摁成零速"。
        #
        # 这是一个结构性缺陷：`_avoid` 先运行，它可以返回 (0,0)（相交让行、
        # LiDAR 急停）；随后 `_apply_traffic` 只能**按比例缩放**速度，
        # 0 乘任何系数还是 0 —— 于是协调层精心规划的让路路径**永远无法生效**。
        # 实测（离线 tools/offline_fleet.py，两车窄通道对头）：整场 2999 tick
        # 卡死，其中 `sat_escape_back` 占 2436 tick、纯追踪只跑了 4 tick；
        # 允许协调层接管后，**同一场景 81 tick 双双到达**。
        #
        # Gazebo 里对应的现象就是"车原地打转 / 整场零产出"（第 32 节：
        # esc 与 stall 同时涨到 87，一趟都走不完）。
        #
        # 因此：当本地避障把速度摁成 0，而协调层给了**让路路径**时，
        # 由协调层接管，用低速沿让路路径走出去。协调层是全局视角，
        # 它已经判定这条让路路径是安全的（is_position_safe_for_yield）。
        # ------------------------------------------------------------------
        if (local_v is not None and abs(local_v) < 1e-6
                and abs(local_w or 0.0) < 1e-6
                and d.has_yield_path and len(d.yield_x) >= 2):
            self._overridden_by_traffic = getattr(
                self, "_overridden_by_traffic", 0) + 1
            if self._overridden_by_traffic in (1, 20, 100):
                self.get_logger().info(
                    f"{self.ns}: local avoidance stalled; coordinator yield path "
                    f"takes over (#{self._overridden_by_traffic}, "
                    f"reason={d.reason})")
            return 0.20, 0.0

        if d.speed_scale < 1.0:
            v *= max(0.0, float(d.speed_scale))
        return v, w

    def _restore_task_path(self):
        """让路结束后回到自己的任务路径。"""
        tx, ty = self._target_xy()
        if tx is not None:
            self._go_to(tx, ty, self.state, reset_strikes=False)

    def publish_status(self):
        s = RobotStatus()
        s.robot_id = self.rid
        s.state = self.state
        s.x, s.y, s.yaw = float(self.x), float(self.y), float(self.yaw)
        s.task_id = self.task.task_id if self.task else -1
        s.distance_total = float(self.total_len)
        s.distance_done = float(self.seen_len)
        s.battery = float(self.battery)
        s.odom_total = float(self.total_len) + float(self.seen_len)
        # 交通协调层要用的三项：没有它们，协调层就无法判断"是不是真的停着"
        # （反饥饿增益）、也拿不到当前阶段目标（冲突裁决与卡死自愈）。
        s.speed = float(getattr(self, "_v_cmd", 0.0))
        tx, ty = self._target_xy()
        s.has_target = tx is not None
        s.target_x = float(tx) if tx is not None else 0.0
        s.target_y = float(ty) if ty is not None else 0.0
        self.pub_status.publish(s)

    def publish_path(self):
        """发布剩余路径。

        规划器给的是栅格 A* + 视线拉直后的折线，画出来就是车真正会走的路线；
        没有它，任何可视化都只能画一条取货点到卸货点的直线，看起来像穿墙。
        """
        m = Path()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        if not self.pursuit.finished:
            for x, y in self.pursuit.remaining_path():
                ps = PoseStamped()
                ps.header = m.header
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.w = 1.0
                m.poses.append(ps)
        self.pub_path.publish(m)

    def broadcast_tf(self):
        """map → robot/odom（出生点，静态）与 robot/odom → base_footprint（动态）。"""
        now = self.get_clock().now().to_msg()
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = "map"
        t_map.child_frame_id = f"{self.ns}/odom"
        t_map.transform.translation.x = float(self.x0)
        t_map.transform.translation.y = float(self.y0)
        q = quat_from_yaw(self.yaw0)
        t_map.transform.rotation.x, t_map.transform.rotation.y = q[0], q[1]
        t_map.transform.rotation.z, t_map.transform.rotation.w = q[2], q[3]

        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = f"{self.ns}/odom"
        t_odom.child_frame_id = f"{self.ns}/base_footprint"
        t_odom.transform.translation.x = float(self.x - self.x0)
        t_odom.transform.translation.y = float(self.y - self.y0)
        q2 = quat_from_yaw(self.yaw - self.yaw0)
        t_odom.transform.rotation.x, t_odom.transform.rotation.y = q2[0], q2[1]
        t_odom.transform.rotation.z, t_odom.transform.rotation.w = q2[2], q2[3]
        self.tf.sendTransform([t_map, t_odom])

    def _set_state(self, new: str, note: str = ""):
        """状态迁移集中入口：变更时打一行日志，便于观测与排障。"""
        if new == self.state:
            return
        self.get_logger().info(
            f"{self.ns} {self.state} -> {new}"
            + (f"  ({note})" if note else "")
            + (f"  task={self.task.task_id}" if self.task else "")
        )
        self.state = new

    # ---------- 主循环 ----------
    def loop(self):
        now = time.time()
        if self.state == "idle":
            self._decide()
        elif self.state in ("to_pickup", "to_dropoff", "to_charger", "departing"):
            self._follow(now)
        elif self.state in ("loading", "unloading", "charging"):
            self._hold(now)
        elif self.state == "waiting_lease":
            self._retry_lease()

    # ---------- 决策 ----------
    def _decide(self):
        # 起步错峰：8 台车挤在 1.35 m 间距的待命排里，同时起步会立刻互锁
        # （实测 2 台车 3.0 单/分钟、4 台 6.0 单/分钟线性增长，到 8 台塌到 0~1）。
        # 真实车队出库也要按序放行，所以用发车间隔而不是"把车摆开"——
        # 后者会让世界上没有一台车停在该停的地方。
        if time.time() < self.start_at:
            return
        if self.battery < float(self.get_parameter("battery_low_pct").value):
            self._start_charging()
            return
        self._request_task()

    def _start_charging(self):
        if not self.cli_acquire.service_is_ready():
            return
        self.charging_events += 1
        self._lease_wait_t = time.time()
        self._lease_kind = "charger"
        self._pending_lease = self._nearest_charger()
        self._set_state("waiting_lease")
        self._request_lease(self._pending_lease)

    def _nearest_charger(self) -> str:
        best, bd = None, 1e9
        for name, c in layout.CHARGERS.items():
            d = math.hypot(c["x"] - self.x, c["y"] - self.y)
            if d < bd:
                best, bd = name, d
        return best

    # ---------- 租约 ----------
    def _request_lease(self, resource: str):
        if not self.cli_acquire.service_is_ready():
            return
        req = AcquireLease.Request()
        req.robot_id, req.resource = self.rid, resource
        self.cli_acquire.call_async(req).add_done_callback(
            lambda f, r=resource: self._on_lease(f, r))

    def _on_lease(self, fut, resource: str):
        try:
            res = fut.result()
        except Exception:
            return
        if res is None or not res.granted:
            return                                    # 留在 waiting_lease 稍后重试
        if self.state != "waiting_lease":
            # 期间状态已变（例如任务被中止），立刻归还，避免泄漏租约
            self._held_override = resource
            self._release_override(resource)
            return
        self.held = resource
        if self._lease_kind == "charger":
            self._go_to(layout.CHARGERS[resource]["x"], layout.CHARGERS[resource]["y"], "to_charger")
        elif self._lease_kind == "source":
            self._go_to(self.task.source_x, self.task.source_y, "to_pickup")
        else:
            self._go_to(self.task.dest_x, self.task.dest_y, "to_dropoff")

    def _release_override(self, resource: str):
        if not self.cli_release.service_is_ready():
            return
        req = ReleaseLease.Request()
        req.robot_id, req.resource = self.rid, resource
        self.cli_release.call_async(req)

    def _retry_lease(self):
        if self.held is not None:
            return
        # 充电桩冲突回退：所有车都选"最近的"，会在同一个桩前排队互堵。
        # 等太久就改去另一个桩 —— 真实车队也会做这种可用性重选。
        if self._lease_kind == "charger":
            waited = time.time() - getattr(self, "_lease_wait_t", time.time())
            if waited > 8.0:
                others = [c for c in layout.CHARGERS if c != self._pending_lease]
                if others:
                    self.get_logger().info(
                        f"{self.ns}: charger {self._pending_lease} busy, switching to {others[0]}")
                    self._pending_lease = others[0]
                self._lease_wait_t = time.time()
        self._request_lease(self._pending_lease)

    def _release(self):
        if self.held is None:
            return
        if self.cli_release.service_is_ready():
            req = ReleaseLease.Request()
            req.robot_id, req.resource = self.rid, self.held
            self.cli_release.call_async(req)
        self.held = None

    # ---------- 任务 ----------
    def _request_task(self):
        if not self.cli_task.service_is_ready():
            self._svc_wait = getattr(self, "_svc_wait", 0) + 1
            if self._svc_wait in (50, 300):          # 5s / 30s 各提示一次
                self.get_logger().warn(f"{self.ns}: /scheduler/request_task not ready yet")
            return
        self._req_calls = getattr(self, "_req_calls", 0) + 1
        req = RequestTask.Request()
        req.robot_id, req.x, req.y = self.rid, float(self.x), float(self.y)
        fut = self.cli_task.call_async(req)
        fut.add_done_callback(self._on_task)
        if self._req_calls in (1, 200):
            self.get_logger().info(f"{self.ns}: task request #{self._req_calls} sent")

    def _on_task(self, fut):
        if self.state != "idle":
            return
        try:
            res = fut.result()
        except Exception as exc:
            self.get_logger().warn(f"{self.ns}: task request failed: {exc}")
            return
        if res is None:
            return
        if not res.has_task:
            self._no_task = getattr(self, "_no_task", 0) + 1
            if self._no_task in (1, 100):
                self.get_logger().info(f"{self.ns}: no task yet ({res.message})")
            return
        self.task = res.task
        self._lease_kind = "source"
        self._pending_lease = self.task.source_name
        self._set_state("waiting_lease")
        self._request_lease(self._pending_lease)
        self.get_logger().info(
            f"{self.ns} <- task {self.task.task_id} [{self.task.material_type}] "
            f"{self.task.source_name} -> {self.task.dest_name}"
        )

    def _plan_around_peers(self, gx, gy):
        """规划一条路径，同伴只作为**逐渐放松**的软约束。

        为什么不能"规划失败就放弃任务"：同伴的动态层半径是
        `hull_r + grid.inflate = 0.636 m`，在 0.1 m 栅格上约 169 格/台。
        车队一密（实测 16 台时）这些格子足以把通道整条封死，A* 在起点就被
        围住并返回空路径，`_go_to` 随即 `_abort_task("no path")` ——
        实测一次 150 s 的运行里出现 **81 次** no path 放弃，
        出料率反而从 8 台的 13~14 单/分钟掉到 7.1。

        正确做法是分级放宽：同伴是**会动的**，为它留一个永久禁区没有依据。
        真正的避碰由 LiDAR 安全层与协调层的互让负责，路径规划只需要"尽量"
        绕开同伴，而不是"绕不开就不干"。
        """
        peers = [(p.x, p.y) for rid, p in self.peers.items() if rid != self.rid]
        full = self.hull_r + self.grid.inflate
        attempts = (
            ("绕开同伴", full),            # 首选：按整车外接圆 + 膨胀绕行
            ("半程余量", max(self.hull_r, full / 2.0)),
            ("仅按车体", self.hull_r),
            ("忽略同伴", 0.0),             # 最后：只按静态障碍规划
        )
        for why, radius in attempts:
            if radius > 0.0:
                self.grid.set_dynamic(peers, radius=radius)
            else:
                self.grid.clear_dynamic()
            path = plan(self.grid, (self.x, self.y), (gx, gy))
            self.grid.clear_dynamic()
            if path:
                if why != "绕开同伴":
                    # 只在退让时记一笔，便于观察车队密度是不是过高
                    self._relaxed_plans = getattr(self, "_relaxed_plans", 0) + 1
                    self.get_logger().info(
                        f"{self.ns}: planned with {why} (radius {radius:.2f} m) "
                        f"-> {len(path)} pts  [relaxed #{self._relaxed_plans}]")
                return path
        return []

    def _go_to(self, gx, gy, state, reset_strikes: bool = True):
        path = self._plan_around_peers(gx, gy)
        if not path:
            # 连"忽略同伴"都无路可走 -> 这是真的没有通路（被机台/墙围死），
            # 才值得放弃任务。
            self.get_logger().warn(f"{self.ns}: no path to ({gx:.1f},{gy:.1f})")
            self._abort_task("no path")
            return
        self.pursuit.set_path(path + [(gx, gy)])
        self.total_len = self.pursuit.remaining()
        self.seen_len = 0.0
        self._set_state(state, f"-> ({gx:.1f},{gy:.1f}) len={self.total_len:.2f}m")
        self._last_progress_xy = (self.x, self.y)
        self._last_progress_t = time.time()
        if reset_strikes:
            self._stuck_strikes = 0

    # ---------- 行驶 ----------
    def _follow(self, now):
        v, w = self.pursuit.step(self.x, self.y, self.yaw)
        self.seen_len = max(0.0, self.total_len - self.pursuit.remaining())

        if self._arrived():
            self._stop()
            if self.state == "departing":
                self._release()                      # 驶离后才释放，避免占着不放
                if self.task is not None:
                    self._begin_dropoff()
                else:
                    self._set_state("idle")          # 充电完成后的离场
                return
            self.timer_until = now + self.dwell
            self.state = ("loading" if self.state == "to_pickup"
                          else "unloading" if self.state == "to_dropoff" else "charging")
            return

        # 租约续租：在途期间周期性 re-acquire，持有者是自己就会刷新 TTL，
        # 避免长途行驶途中租约过期、别人又把位子抢走。
        if self.held and now - getattr(self, "_lease_renew_t", 0.0) > 10.0:
            self._lease_renew_t = now
            self._request_lease(self.held)

        # 重规划：只在**路径真的被占住**时才重算。
        # 曾经是每 3 秒无条件重算一次，结果是纯追踪的路径被反复重置：
        # 车刚开始转向就被打断，于是永远在原地来回摆，一步也走不出去。
        if (not self._yield_active
                and now - getattr(self, "_last_replan_t", 0.0) > 1.0
                and self._path_blocked_by_peer()):
            self._last_replan_t = now
            tx, ty = self._target_xy()
            if tx is not None:
                self._go_to(tx, ty, self.state, reset_strikes=False)

        v_pt, w_pt = v, w                 # 纯追踪的期望速度（协调层接管时要用）
        v, w = self._avoid(v, w, now)
        # 交通指令放在局部避障之后：协调层是全局视角，它的 hold / 脱困
        # 必须能覆盖单车自己的局部判断，否则两台车会同时"礼貌地"往中间挤。
        v, w = self._apply_traffic(v, w, now, local_v=v, local_w=w,
                                   pursuit_v=v_pt, pursuit_w=w_pt)
        self._watchdog(now)
        self._v_last = v
        self._v_cmd = v
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub_cmd.publish(cmd)
        if self.self_kin:
            dt = 0.1
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += w * dt
            self.battery = max(0.0, self.battery - abs(v) * dt * float(
                self.get_parameter("battery_drain_per_m").value))

    def _path_blocked_by_peer(self, horizon: float = 2.2) -> bool:
        """剩余路径的前 horizon 米内是否有同伴占位。"""
        rest = self.pursuit.remaining_path()
        if not rest:
            return False
        pts, acc, prev = [], 0.0, (self.x, self.y)
        for q in rest:
            acc += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = q
            pts.append(q)
            if acc > horizon:
                break
        for p in self.peers.values():
            if p.robot_id == self.rid:
                continue
            for q in pts:
                if math.hypot(q[0] - p.x, q[1] - p.y) < 0.75:
                    return True
        return False

    def _target_xy(self):
        if self.state == "to_pickup" and self.task:
            return self.task.source_x, self.task.source_y
        if self.state == "to_dropoff" and self.task:
            return self.task.dest_x, self.task.dest_y
        if self.state == "to_charger" and self.held in layout.CHARGERS:
            return layout.CHARGERS[self.held]["x"], layout.CHARGERS[self.held]["y"]
        return None, None

    def _docking(self) -> bool:
        """是否处于"靠泊段"：离最终目标足够近，且剩余路径很短。"""
        tx, ty = self._target_xy()
        if tx is None:
            return False
        return (math.hypot(tx - self.x, ty - self.y) < 0.80
                and self.pursuit.remaining() < 1.20)

    def _arrived(self) -> bool:
        # 让路途中走完的是"让路路径"，不是任务目标点。
        # 不挡这一下，车会在让路结束时误判到达、直接开始装卸。
        if self._yield_active:
            return False
        if self.pursuit.finished:
            return True
        tx, ty = self._target_xy()
        if tx is None:
            return False
        d = math.hypot(tx - self.x, ty - self.y)
        if d < 0.20:
            return True
        # ---------------------------------------------------------------
        # 靠泊收敛：距离够近、且只剩最后一个路径点时，直接算到达。
        #
        # 为什么必须加这一条 —— 实测（.workbench/pursuitchk.py）：
        # 车开到世界 (3.21, 3.08)，目标是 storage_empty_slot_0 (3.40, 3.00)，
        # 距离 **0.21 m**，而到达阈值是 0.20 m —— 差 **1 cm** 判不到达；
        # 同一时刻朝向误差 **-175.9°**（车头几乎背对目标），纯追踪于是持续
        # 输出 v=0, w=±1.2 原地旋转。而**原地转不改变到目标的距离**，
        # 那 1 cm 就永远补不上：车无限旋转，yaw 极差 6.28 rad（整圈），
        # 有效位移速度只有上报速度的 40%（0.133 vs 0.33 m/s）。
        #
        # 逻辑上也不该要求"对准了才算到"：装卸不需要车头精确朝向目标，
        # 实车进站本来就是先到位、有必要再微调朝向。0.35 m 这点额外宽容
        # 只为吃掉"路径最后一点已在脚下、朝向却差 180°"这个死锁。
        if d < 0.35 and len(self.pursuit.remaining_path()) <= 1:
            return True
        return False

    def _front_peer(self, dist: float = 0.60, half_angle: float = 0.9):
        """正前方锥形内最近的同伴（没有则 None）。

        LiDAR 只告诉我们"前方有东西、多远、左右哪边空"，分不出那是机台还是
        同伴。而"往哪边退"这件事两者答案相反：机台要挑空的一侧，
        同伴要挑**远离它**的一侧。所以这里用车队状态补上这个区分。
        """
        best, bd = None, dist
        for rid, p in self.peers.items():
            if rid == self.rid:
                continue
            dx, dy = p.x - self.x, p.y - self.y
            d = math.hypot(dx, dy)
            if d > bd:
                continue
            bearing = math.atan2(dy, dx) - self.yaw
            if abs(math.atan2(math.sin(bearing), math.cos(bearing))) > half_angle:
                continue
            best, bd = p, d
        return best

    def _abump(self, why: str):
        """统计 _avoid 里到底哪一条在限速 —— 排障用，不开日志就没有开销。"""
        self._avoid_hist[why] = self._avoid_hist.get(why, 0) + 1

    def _dump_scan_frames(self):
        """把收集到的"前方近障碍"原始帧写成 JSON，供离线反推。"""
        if not self._dbg_frames:
            return
        import json
        import os
        os.makedirs(self._dbg_dir, exist_ok=True)
        p = os.path.join(self._dbg_dir, f"scanfram_{self.ns}.json")
        try:
            with open(p, "w") as f:
                json.dump(dict(robot=self.rid, frames=self._dbg_frames), f)
        except OSError as exc:
            self.get_logger().warn(f"{self.ns}: cannot write scan frames: {exc}")

    def _dump_avoid(self):
        """把本车 _avoid 分支计数打成一行，便于对比"到底谁在压速度"。"""
        h = self._avoid_hist
        if not h:
            return
        n = max(1, h.get("called", 1))
        top = ", ".join(f"{k}={v}({v/n*100:.0f}%)"
                        for k, v in sorted(h.items(), key=lambda kv: -kv[1]) if k != "called")
        tx, ty = self._target_xy()
        self.get_logger().info(
            f"AVOID {self.ns} state={self.state} pos=({self.x:.2f},{self.y:.2f}) "
            f"tgt={'-' if tx is None else f'({tx:.2f},{ty:.2f})'} "
            f"scan={self._scan_min:.2f} called={n} :: {top}")

    def _avoid(self, v: float, w: float, now: float = 0.0):
        """车车互让 + LiDAR 安全层。"""
        slow = float(self.get_parameter("avoid_slow_m").value)
        stop = float(self.get_parameter("avoid_stop_m").value)
        self._abump("called")
        # 1) LiDAR：前向有东西就停。
        #    靠泊例外：进入目标点附近后，急停阈值必须放宽到小于到达判定，
        #    否则"急停 0.38m > 到达 0.20m"会形成一个永远进不去的死区 ——
        #    车停在 0.28m 处，看门狗判定卡死，几次之后放弃任务。
        #    这是实车也采用的做法：靠泊段切到低速对接模式。
        scan_stop = float(self.get_parameter("scan_stop_m").value)
        if self._docking():
            scan_stop = min(scan_stop, 0.12)
        if now < getattr(self, "_creep_until", 0.0):
            scan_stop = min(scan_stop, 0.14)
            v = min(v, 0.16)
        if self._scan_min < scan_stop and v > 0.0:
            self.yields += 1
            # 只停不转是不够的：车头顶住机台之后，纯追踪只会一直让它往前走，
            # 于是"停住 → 看门狗重规划 → 路径还是那一条 → 继续停住"，
            # 变成永久卡死（实测 8 台车里 3 台就这么废掉）。
            # ROS 1 的 check_laser_obstacle 在同样情形下是**低速倒车 + 转向
            # 较空的一侧**，这里按同一套判据补上；靠泊/爬行阶段除外，
            # 否则永远进不了工位。
            if (not self._docking()) and now >= getattr(self, "_creep_until", 0.0):
                # 前方到底是机台还是同伴，决定往哪退：
                #   机台 -> 转向左右较空的一侧；
                #   同伴 -> 转向**远离它**的一侧。
                # 原来一律"哪边空往哪转"，当同伴在正前方时左右对称，
                # 判据退化成永远选同一侧，车就贴住对方原地打转（实测 R6/R7
                # 在待命区并排卡死 60 s，实走里程 0）。
                turn = 1.0 if self._scan_left > self._scan_right else -1.0
                blocker = self._front_peer()
                if blocker is not None:
                    bearing = math.atan2(blocker.y - self.y, blocker.x - self.x)
                    rel = math.atan2(math.sin(bearing - self.yaw), math.cos(bearing - self.yaw))
                    turn = -1.0 if rel > 0.0 else 1.0     # 反着对方转
                    self._abump("lidar_backup_peer")
                elif self._scan_front < 0.5:
                    self._abump("lidar_backup_static")
                else:
                    self._abump("lidar_stop")
                    return 0.0, w * 0.3
                return -0.10, turn
            self._abump("lidar_stop")
            return 0.0, w * 0.3
        # 2) 硬安全距离：用**有向矩形**判据，而不是圆盘。
        #    只用"车心距"既不对又难调：0.34 m 会真的撞上（车长 0.56 m），
        #    改成两车外接圆之和 0.712 m 又过于保守 —— 并排本来只需要 0.44 m，
        #    结果是车频繁互停、吞吐从 130 单掉到 14 单。
        #    位置和朝向都在 /fleet/robots 里，所以直接用分离轴定理判两个矩形是否相交：
        #    既不会漏掉真碰撞，也不会挡掉合法的并排通行。
        margin = float(self.get_parameter("avoid_margin_m").value)
        hx, hy = self.hull_len / 2 + margin, self.hull_wid / 2 + margin
        me = _obb_corners(self.x, self.y, self.yaw, hx, hy)
        for rid, p in self.peers.items():
            if rid == self.rid:
                continue
            d = math.hypot(p.x - self.x, p.y - self.y)
            close = d < self.hull_diag + 2 * margin      # 粗筛，省掉绝大多数 SAT 计算
            if close and _obb_overlap(me, _obb_corners(p.x, p.y, p.yaw, hx, hy)):
                hard = True
            else:
                hard = False
            if hard or d < 1e-6:
                # 轮廓已经相交。这里必须给**被挡住的那台**一条出路，否则两台车
                # 会互相僵住。原来只有"id 小的一律沿自身朝向挪"这一条，
                # 而并排待命时两台车朝向相同 —— 车体系里的"远离对方"垂直于
                # 前进方向，余弦约等于 0，于是永远不满足 >0.25，直接 return 0。
                # 实测后果：sat_stop 占 R4 全部 _avoid 调用的 74%，车纹丝不动。
                # 现在改成：优先沿路径走，其次侧向让开，最后才原地等。
                self.yields += 1
                if rid > self.rid:
                    self._abump("sat_yield_stop")
                    return 0.0, 0.0                     # 让行车完全停住（等对方腾位）
                ax, ay = self.x - p.x, self.y - p.y     # 从对方指向我
                away = math.hypot(ax, ay)
                if away < 1e-6:
                    self._abump("sat_coincident")
                    return 0.0, 0.0
                # 沿自身朝向、以及左右侧向，挑一个真正能拉开距离的
                for name, vv, ww in (("fwd", min(v, 0.12), w),
                                     ("fwd_r", 0.10, -0.9),
                                     ("fwd_l", 0.10, 0.9),
                                     ("back", -0.10, 0.0)):
                    nx = self.x + vv * math.cos(self.yaw) * 0.5
                    ny = self.y + vv * math.sin(self.yaw) * 0.5
                    if math.hypot(nx - p.x, ny - p.y) > away + 1e-3:
                        self._abump(f"sat_escape_{name}")
                        return vv, ww
                self._abump("sat_stop")
                return 0.0, 0.0

        # 3) 前方锥形内的互让：只看会挡我路的那台
        #    这一层是"车队互让"的本地近似。协调层（traffic.py）已经用
        #    冲突分类 + 优先级 + 走廊令牌 + 让路路径做了同一件事，两套叠在一起
        #    就是双重限速：实测指令速度中位数被压到 0.15 m/s（上限 0.85），
        #    34.9% 的指令落在 ≤0.30 m/s。所以默认关掉，由协调层统一裁决；
        #    保留参数是为了能把它打开做对照实验。
        #    注意：关掉的只是"减速礼貌"，真正的防撞仍由第 2 步的有向矩形硬
        #    判据兜底，不会漏碰撞。
        if not self.avoid_cones:
            if v > 0.05:
                if self._scan_left < 0.30:
                    w = min(w, -0.35)
                    self._abump("graze_left")
                elif self._scan_right < 0.30:
                    w = max(w, 0.35)
                    self._abump("graze_right")
            self._abump("clear")
            return v, w
        for rid, p in self.peers.items():
            if rid == self.rid:
                continue
            dx, dy = p.x - self.x, p.y - self.y
            d = math.hypot(dx, dy)
            if d > slow or d < 1e-6:
                continue
            bearing = math.atan2(dy, dx) - self.yaw
            ang = abs(math.atan2(math.sin(bearing), math.cos(bearing)))
            if ang > 0.9:                              # 不在前方约 52° 内
                continue
            if rid < self.rid:                          # 确定性优先级：id 小的先行通过
                if d < stop:
                    v = min(v, 0.14)                    # 低速挤过去，而不是无视对方
                    self._abump("cone_front_squeeze")
                else:
                    v = min(v, 0.40)
                    self._abump("cone_front_slow")
            else:
                if d < stop * 0.8:
                    self.yields += 1
                    self._abump("cone_back_stop")
                    return 0.0, 0.0                     # 让行车完全停住
                v = min(v, 0.50)
                self._abump("cone_back_slow")
        # 4) 侧擦修正：一侧贴得太近就往另一侧修一点，避免一路蹭着机台走。
        #    这也是 ROS 1 check_laser_obstacle 里 left/right < 0.3 那两条。
        if v > 0.05:
            if self._scan_left < 0.30:
                w = min(w, -0.35)
                self._abump("graze_left")
            elif self._scan_right < 0.30:
                w = max(w, 0.35)
                self._abump("graze_right")
        self._abump("clear")
        return v, w

    def _watchdog(self, now):
        """长时间没有位移 → 重规划 → 仍不动则放弃任务并上报失败。"""
        moved = math.hypot(self.x - self._last_progress_xy[0], self.y - self._last_progress_xy[1])
        if moved > 0.08:
            self._last_progress_xy = (self.x, self.y)
            self._last_progress_t = now
            self._stuck_strikes = 0
            return
        if now - self._last_progress_t < float(self.get_parameter("stuck_timeout_s").value):
            return
        self._stuck_strikes += 1
        self._last_progress_t = now
        tx, ty = self._target_xy()
        # 卡死时把"谁挡着我"打出来，排障用；同时也是可观测性的一部分
        nearest = min(((math.hypot(q.x - self.x, q.y - self.y), rid)
                       for rid, q in self.peers.items() if rid != self.rid),
                      default=(99.0, -1))
        self.get_logger().warn(
            f"{self.ns} stalled#{self._stuck_strikes} state={self.state} "
            f"pos=({self.x:.2f},{self.y:.2f}) target=({tx if tx is None else round(tx,2)},"
            f"{ty if ty is None else round(ty,2)}) nearest_peer={nearest[1]}@{nearest[0]:.2f}m "
            f"scan_min={self._scan_min:.2f} v_last={getattr(self,'_v_last',0):.2f} "
            f"pursuit_idx={self.pursuit.idx}/{len(self.pursuit.path)}")
        # 第二次卡死时进入爬行：把急停阈值压到最低，用极低速试走一小段。
        # 真实产线上，一次未被证实的近距离回波不该让整条线停下来，
        # 而应当降级为"慢速通过 + 记录"，只有真的走不动才放弃任务。
        if self._stuck_strikes == 2:
            self._creep_until = now + 6.0
            self.get_logger().warn(f"{self.ns}: stalled twice, creeping to clear it")
        if self._stuck_strikes < 3 and tx is not None:
            self.replans += 1
            self.get_logger().warn(f"{self.ns}: stalled, re-planning")
            # 关键：由看门狗触发的重规划不能清零计数，否则会陷入
            # "卡死 → 重规划 → 计数归零 → 卡死" 的无限循环，永远不升级到放弃任务
            self._go_to(tx, ty, self.state, reset_strikes=False)
        else:
            self.failures += 1
            self._abort_task("stalled twice")

    def _hold(self, now):
        self._stop()
        if self.state == "charging":
            rate = float(self.get_parameter("charge_rate_pct_s").value)
            self.battery = min(100.0, self.battery + rate * 0.1)
            if self.battery >= float(self.get_parameter("battery_resume_pct").value):
                # 先驶离充电位再释放租约：否则下一台拿到租约的车会撞上还占着桩位的自己
                self._go_to(self.x + 1.4, self.y, "departing")
            return
        if now < self.timer_until:
            return
        if self.state == "loading":
            # 载货后先驶离工位再释放租约：既不占位，也不挡别人进站
            self._go_to(*self._staging_point(), "departing")
            return
        # unloading
        self._release()
        done = Int32()
        done.data = self.task.task_id
        self.pub_done.publish(done)
        self.get_logger().info(f"{self.ns} delivered task {self.task.task_id}")
        self.task = None
        self._set_state("idle")

    def _staging_point(self):
        """工位旁的临时等待点：沿来路退开，避免堵住进站通道。"""
        if self.task is None:
            return self.x, self.y
        dx = self.task.dest_x - self.task.source_x
        dy = self.task.dest_y - self.task.source_y
        n = math.hypot(dx, dy) or 1.0
        return self.x - dx / n * 1.0, self.y - dy / n * 1.0

    def _begin_dropoff(self):
        self._lease_kind = "dest"
        self._pending_lease = self.task.dest_name
        self._set_state("waiting_lease")
        self._request_lease(self._pending_lease)

    def _abort_task(self, why: str):
        if self.task is not None:
            self.task.status = "failed"
            self.task.robot_id = self.rid
            self.pub_failed.publish(self.task)
            self.get_logger().warn(f"{self.ns} task {self.task.task_id} aborted: {why}")
        self._release()
        self.task = None
        self._set_state("idle")

    def _stop(self):
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = RobotController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
