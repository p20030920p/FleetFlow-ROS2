"""交通管理节点：工位租约 + 完整交通协调层。

两层职责，别混淆
----------------
1. **资源层（租约）**：保证同一个工位/充电桩同一时刻只有一台车停靠。
   它破坏的是死锁四条件里的"持有并等待"，所以不会出现资源环死锁。
2. **运动层（``traffic.TrafficLayer``）**：租约管不到的部分 —— 通道里对向
   相遇、多车挤向同一完工位、被压成 0 速之后没人管。这一层是从 ROS 1 版
   完整移植过来的（时空预约、走廊单向令牌、优先级反饥饿、冲突裁决、
   安全让路、双机分离、紧急刹停、卡死自愈）。

为什么拆成"协调层下指令、控制器执行"
------------------------------------
ROS 1 里调度器和导航同进程，协调层直接写 ``cmd_vel``。ROS 2 把导航拆到了
每台车的控制器（它才拿得到 LiDAR 与纯追踪状态），所以协调层改为发布
``/robot_i/traffic``（``TrafficDirective``）：协调层仍然决定**谁让谁、
什么时候走、往哪让**，控制器只负责把这些约束变成轮速。
"""
from __future__ import annotations

import time
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String

from fleetflow_interfaces.msg import RobotStatus, TrafficDirective
from fleetflow_interfaces.srv import AcquireLease, ReleaseLease

from . import layout
from .qos import state_qos
from .traffic import CORRIDOR_ZONES, REGIONS, TrafficLayer


class Lease:
    __slots__ = ("robot_id", "since", "ttl")

    def __init__(self, robot_id: int, ttl: float):
        self.robot_id = robot_id
        self.since = time.time()
        self.ttl = ttl


class TrafficManager(Node):
    def __init__(self):
        super().__init__("traffic_manager")
        self.declare_parameter("lease_ttl_s", 120.0)
        self.declare_parameter("max_wait_s", 45.0)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("num_robots", 8)
        # 碰撞判据开关，**只为对照实验**保留：
        #   outline（默认）= 两车轮廓净距，与控制器一致
        #   centre        = 旧的车心距判据（1.5 m 起减速），存档对照用
        # 留着它是为了能回答"改成轮廓判据到底有没有用"，而不是只能靠记忆对比。
        self.declare_parameter("collision_criterion", "outline")
        # 脱困策略开关，同样只为对照实验保留：legacy = 旧行为（原地转）
        self.declare_parameter("escape_strategy", "gap")
        # 卡死判定地平线（秒）：连续低速多久算"持久卡死"。
        # 做成参数是为了判断"卡死自愈"在帮忙还是捣乱（拉大等于关掉）。
        self.declare_parameter("stall_horizon_s", 8.0)
        # 脱困选举：一对贴住的车只让一台脱困，另一台原地让出空间。
        # 关掉 = 双方各自脱困（旧行为，实测会互推）。
        # 默认 false —— 见 findings 第 62 节，开启会让卡死恶化 4 倍
        self.declare_parameter("escape_election", False)

        self.ttl = float(self.get_parameter("lease_ttl_s").value)
        self.leases: dict[str, Lease] = {}
        self.waiting: dict[str, deque] = {}
        self.granted_total = 0
        self.rejected_total = 0
        self.expired_total = 0
        self.waits: dict[int, float] = {}

        known = list(layout.all_station_points()) + ["charger_0", "charger_1"]
        self.known_resources = set(known)

        # ---- 交通协调层 ----
        n = int(self.get_parameter("num_robots").value)
        self.layer = TrafficLayer(n, logger=self.get_logger())
        crit = str(self.get_parameter("collision_criterion").value)
        self.layer.centre_criterion = (crit == "centre")
        self.layer.escape_legacy = (str(self.get_parameter("escape_strategy").value)
                                    == "legacy")
        # 卡死判定地平线：连续低速多久算"持久卡死"。
        # QoS 修好之后它变得很敏感（假的位姿/速度曾让它恒定触发），
        # 所以做成可调参数，用来判断"卡死自愈"到底是在帮忙还是在捣乱。
        self.layer.stall_horizon = float(self.get_parameter("stall_horizon_s").value)
        self.layer.escape_election_on = bool(
            self.get_parameter("escape_election").value)
        self.peers: dict[int, RobotStatus] = {}
        self.last_seen: dict[int, float] = {}

        # ---- ROS 接口 ----
        self.create_service(AcquireLease, "/traffic/acquire", self.on_acquire)
        self.create_service(ReleaseLease, "/traffic/release", self.on_release)

        self.create_subscription(RobotStatus, "/fleet/robots", self.on_status, state_qos(40))
        self.directive_pubs: dict[int, object] = {}
        for i in range(n):
            self.directive_pubs[i] = self.create_publisher(
                TrafficDirective, f"/robot_{i}/traffic", state_qos(10))
            self.create_subscription(
                Path, f"/robot_{i}/path", lambda m, r=i: self.on_path(m, r), state_qos(5))

        self.pub = self.create_publisher(String, "/traffic/status", state_qos(5))
        self.pub_directives = self.create_publisher(
            TrafficDirective, "/traffic/directives", state_qos(40))

        self.create_timer(1.0, self.sweep)
        self.create_timer(2.0, self.report)
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"traffic_manager up · {len(known)} resources · lease ttl {self.ttl:.0f}s · "
            f"coordination {rate:.0f} Hz · regions={len(REGIONS)} corridors={len(CORRIDOR_ZONES)}"
        )

    # ---------- 状态输入 ----------
    def on_status(self, m: RobotStatus):
        rid = int(m.robot_id)
        self.peers[rid] = m
        self.last_seen[rid] = time.time()
        target = (m.target_x, m.target_y) if getattr(m, "has_target", False) else None
        self.layer.update_robot(
            rid, m.x, m.y, m.yaw, getattr(m, "speed", 0.0), m.state, target,
            self.layer.path.get(rid, []),
        )
        if rid not in self.directive_pubs and rid < 32:
            self.directive_pubs[rid] = self.create_publisher(
                TrafficDirective, f"/robot_{rid}/traffic", state_qos(10))

    def on_path(self, m: Path, rid: int):
        self.layer.update_robot(
            rid,
            self.layer.pos.get(rid, (0.0, 0.0))[0],
            self.layer.pos.get(rid, (0.0, 0.0))[1],
            self.layer.yaw.get(rid, 0.0),
            self.layer.speed.get(rid, 0.0),
            self.layer.state.get(rid, "idle"),
            self.layer.target.get(rid),
            [(p.pose.position.x, p.pose.position.y) for p in m.poses],
        )

    # ---------- 协调周期 ----------
    def tick(self):
        now = time.time()
        # 掉线清理：15 秒没有状态上报就当它不在了，否则它的预约会永久占位
        for rid, t in list(self.last_seen.items()):
            if now - t > 15.0:
                self.layer.forget_robot(rid)
                self.last_seen.pop(rid, None)

        directives = self.layer.tick(now)
        for rid, d in directives.items():
            pub = self.directive_pubs.get(rid)
            if pub is None:
                continue
            msg = TrafficDirective()
            msg.robot_id = rid
            msg.speed_scale = float(d.speed_scale)
            msg.hold = bool(d.hold)
            msg.has_escape = bool(d.has_escape)
            msg.escape_v = float(d.escape_v)
            msg.escape_w = float(d.escape_w)
            msg.replan = bool(d.replan)
            msg.has_yield_path = bool(d.has_yield_path)
            msg.yield_x = [float(p[0]) for p in d.yield_path]
            msg.yield_y = [float(p[1]) for p in d.yield_path]
            msg.reason = d.reason
            pub.publish(msg)
            self.pub_directives.publish(msg)

    # ---------- 租约服务 ----------
    def on_acquire(self, req: AcquireLease.Request, res: AcquireLease.Response):
        rid, resource = int(req.robot_id), req.resource
        now = time.time()
        lease = self.leases.get(resource)

        if lease is not None and now - lease.since > lease.ttl:
            self.leases.pop(resource, None)
            self.expired_total += 1
            lease = None

        if lease is None or lease.robot_id == rid:
            self.leases[resource] = Lease(rid, self.ttl)
            self.granted_total += 1
            self.waits.pop(rid, None)
            res.granted, res.holder, res.message = True, rid, "granted"
            return res

        q = self.waiting.setdefault(resource, deque())
        if rid not in q:
            q.append(rid)
        self.waits.setdefault(rid, now)
        self.rejected_total += 1
        waited = now - self.waits[rid]
        res.granted = False
        res.holder = lease.robot_id
        res.message = f"held by {lease.robot_id}, waited {waited:.1f}s"
        if waited > float(self.get_parameter("max_wait_s").value):
            res.message += " (over max_wait: consider re-planning)"
        return res

    def on_release(self, req: ReleaseLease.Request, res: ReleaseLease.Response):
        rid, resource = int(req.robot_id), req.resource
        lease = self.leases.get(resource)
        if lease is None or lease.robot_id == rid:
            self.leases.pop(resource, None)
            self.waiting.pop(resource, None)
            self.waits.pop(rid, None)
            res.ok = True
        else:
            res.ok = False
        return res

    # ---------- 维护 ----------
    def sweep(self):
        now = time.time()
        for name, lease in list(self.leases.items()):
            if now - lease.since > self.ttl:
                self.leases.pop(name, None)
                self.expired_total += 1
                self.get_logger().warn(f"lease expired: {name} (was robot {lease.robot_id})")

    def report(self):
        s = self.layer.stats()
        txt = (
            f"traffic: held={len(self.leases)} waiting={sum(len(q) for q in self.waiting.values())} "
            f"granted={self.granted_total} rejected={self.rejected_total} expired={self.expired_total} "
            f"| deadlock={s['deadlock']} yield={s['yield_total']} fallback={s['yield_fallback']} "
            f"sep={s['pair_separation']} embrake={s['emergency_break']} esc={s['escape']} "
            f"coll={s['collision_event']} corridor={s['corridor_grant']}/{s['corridor_reuse']} "
            f"stall={s['stall_replan']}/{s['stall_near_target']} anti_starve={s['starvation_prevent']}"
        )
        msg = String()
        msg.data = txt
        self.pub.publish(msg)


def main():
    from rclpy.executors import ExternalShutdownException

    layout.bootstrap()      # 先定布局，再建节点
    rclpy.init()
    node = TrafficManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
