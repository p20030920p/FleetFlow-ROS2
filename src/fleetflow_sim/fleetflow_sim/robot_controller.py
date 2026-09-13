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

from fleetflow_interfaces.msg import RobotStatus, TransportTask
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
        # 锥形互让只负责"礼貌"，真正的防撞由上面的有向矩形判据兜底。
        # 阈值给太大车会变得过度胆小：实测 slow=1.8/stop=1.0 时 6 台车互相让到
        # 几乎不动（86 次卡死、310 秒只送达 12 单）。
        self.declare_parameter("avoid_slow_m", 1.20)
        self.declare_parameter("avoid_stop_m", 0.70)
        self.declare_parameter("scan_stop_m", 0.38)
        self.declare_parameter("stuck_timeout_s", 10.0)
        # 电量
        self.declare_parameter("battery_drain_per_m", 0.55)
        # 车体几何：LiDAR 安装点相对车体中心的纵向偏移，用于逐角度屏蔽自身回波
        self.hull_len, self.hull_wid, self.lidar_dx = 0.56, 0.44, 0.18
        self.declare_parameter("scan_self_mask_m", 0.06)
        self.declare_parameter("battery_low_pct", 30.0)
        self.declare_parameter("battery_resume_pct", 88.0)
        self.declare_parameter("charge_rate_pct_s", 3.0)

        self.rid = int(self.get_parameter("robot_id").value)
        self.ns = f"robot_{self.rid}"
        self.dwell = float(self.get_parameter("dwell_s").value)
        self.self_kin = bool(self.get_parameter("use_internal_kinematics").value)

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
        # 车队
        self.peers: dict[int, RobotStatus] = {}

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

        self.cli_task = self.create_client(RequestTask, "/scheduler/request_task")
        self.cli_acquire = self.create_client(AcquireLease, "/traffic/acquire")
        self.cli_release = self.create_client(ReleaseLease, "/traffic/release")
        self.tf = TransformBroadcaster(self)

        self.create_timer(0.1, self.loop)
        self.create_timer(0.2, self.publish_status)
        self.create_timer(0.2, self.broadcast_tf)
        self.create_timer(0.5, self.publish_path)
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
        """从雷达到**车体轮廓**在给定方向上的距离（车体系，前方为 +x）。

        为什么要算这个：LiDAR 装在车头 (x=+0.18)，而车体是 0.56×0.44 的矩形，
        所以它的两条前角在 ±66°、约 0.24 m 处 —— 正好落在前向安全扇区里。
        实测中这一对**对称的自身回波**让车永久停在原地（v=0 → 看门狗判卡死 →
        放弃任务 → 全线停摆）。真实 AGV 的做法是按轮廓逐角度屏蔽自身回波，
        而不是拍一个常数距离：常数要么漏（屏蔽不掉车尾）、要么瞎（把真障碍也屏蔽了）。
        """
        # 车体矩形（车体系）与雷达安装点
        hx, hy = self.hull_len / 2.0, self.hull_wid / 2.0
        lx = self.lidar_dx
        dx, dy = math.cos(ang), math.sin(ang)
        best = float("inf")
        # 与四条边求交，取最近的正向交点
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
        half = max(1, int(n * 0.11))                      # 约 ±40°
        lo, hi = n // 2 - half, n // 2 + half
        margin = float(self.get_parameter("scan_self_mask_m").value)
        rmin = float(m.range_min)
        front = []
        for i in range(lo, hi):
            r = m.ranges[i]
            if not (rmin < r < m.range_max):
                continue
            ang = m.angle_min + i * m.angle_increment
            if r > self._self_range(ang) + margin:        # 自身轮廓以外的才算障碍
                front.append(r)
        self._scan_min = min(front) if front else 99.0

    def on_peer(self, m: RobotStatus):
        self.peers[m.robot_id] = m

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

    def _go_to(self, gx, gy, state, reset_strikes: bool = True):
        # 把当前同伴位置写进动态层，让 A* 直接绕开，而不是硬挤
        peers = [(p.x, p.y) for rid, p in self.peers.items() if rid != self.rid]
        # 动态障碍层要按"同伴的外接圆 + 栅格本身的膨胀量"来画，
        # 这样 A* 绕开同伴时留出的余量，和绕开墙、机台时是一样的。
        self.grid.set_dynamic(peers, radius=self.hull_r + self.grid.inflate)
        path = plan(self.grid, (self.x, self.y), (gx, gy))
        self.grid.clear_dynamic()
        if not path:
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
        if (now - getattr(self, "_last_replan_t", 0.0) > 1.0
                and self._path_blocked_by_peer()):
            self._last_replan_t = now
            tx, ty = self._target_xy()
            if tx is not None:
                self._go_to(tx, ty, self.state, reset_strikes=False)

        v, w = self._avoid(v, w, now)
        self._watchdog(now)
        self._v_last = v
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
        if self.pursuit.finished:
            return True
        tx, ty = self._target_xy()
        if tx is None:
            return False
        return math.hypot(tx - self.x, ty - self.y) < 0.20

    def _avoid(self, v: float, w: float, now: float = 0.0):
        """车车互让 + LiDAR 安全层。"""
        slow = float(self.get_parameter("avoid_slow_m").value)
        stop = float(self.get_parameter("avoid_stop_m").value)
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
                # 双方都停会死锁，所以用确定性优先级放一台走 —— 但**只放它"往外走"**。
                # 早前的写法是"id 小的一律以 0.12 m/s 挪"，结果当它的路径正好朝向
                # 对方时，就是主动往车里开：实测出现 1900 次重叠，最小车心距 0.259 m。
                self.yields += 1
                if rid > self.rid:
                    return 0.0, 0.0                     # 让行车完全停住
                ax, ay = self.x - p.x, self.y - p.y     # 从对方指向我
                away = math.hypot(ax, ay)
                if away < 1e-6:
                    return 0.0, 0.0
                # 只有当前进方向确实在"远离对方"时才允许低速脱离
                if (math.cos(self.yaw) * ax + math.sin(self.yaw) * ay) / away > 0.25:
                    return min(v, 0.12), w
                return 0.0, 0.0

        # 3) 前方锥形内的互让：只看会挡我路的那台
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
                else:
                    v = min(v, 0.40)
            else:
                if d < stop * 0.8:
                    self.yields += 1
                    return 0.0, 0.0                     # 让行车完全停住
                v = min(v, 0.50)
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
