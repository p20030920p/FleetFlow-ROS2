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
        self.declare_parameter("avoid_slow_m", 1.30)
        self.declare_parameter("avoid_stop_m", 0.60)
        self.declare_parameter("scan_stop_m", 0.38)
        self.declare_parameter("stuck_timeout_s", 10.0)
        # 电量
        self.declare_parameter("battery_drain_per_m", 0.55)
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

    def on_scan(self, m: LaserScan):
        """只关心前向扇区的最小距离，作为最后一道安全闸。"""
        if not m.ranges:
            return
        n = len(m.ranges)
        half = max(1, int(n * 0.11))                      # 约 ±40°
        lo, hi = n // 2 - half, n // 2 + half
        front = [r for r in m.ranges[lo:hi] if m.range_min < r < m.range_max]
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
        self.grid.set_dynamic(peers, radius=0.50)
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

        # 周期性重规划：出发时算好的路径会随着同伴移动而失效，
        # 真实系统也是按固定频率重规划，而不是只在卡死时才重算。
        if now - getattr(self, "_last_replan_t", 0.0) > 3.0:
            self._last_replan_t = now
            tx, ty = self._target_xy()
            if tx is not None and self.pursuit.remaining() > 0.8:
                self._go_to(tx, ty, self.state, reset_strikes=False)

        v, w = self._avoid(v, w)
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

    def _avoid(self, v: float, w: float):
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
        if self._scan_min < scan_stop and v > 0.0:
            self.yields += 1
            return 0.0, w * 0.3
        # 2) 同伴：只看"我前方锥形"里的车
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
            hard = 0.34                                 # 硬安全距离：谁都不能再靠近
            if d < hard:
                self.yields += 1
                return 0.0, 0.0
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
