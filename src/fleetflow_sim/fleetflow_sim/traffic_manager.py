"""交通管理：工位与充电桩的独占租约。

为什么需要它
------------
只有"逻辑上把任务分给不同的车"是不够的——物理上两台车仍可能同时压向同一个
工位，或者互相堵在门口。真实车队里这件事由交通管理（traffic management）
负责，这里是它的最小可用实现。

死锁是怎么被结构性排除的
------------------------
经典的死锁四个条件里，本设计直接破坏"持有并等待"（hold and wait）：

* 一台车**同时最多只持有一个资源**：它正在停靠的那个工位，或正在接近的那个工位；
* 离开工位时立刻释放，因此**在途车辆不持有任何资源**；
* 申请目标租约被拒时，它就停在自己当前位置等，而不是"抱着 A 去抢 B"。

不持有资源就不会形成循环等待，因此系统不会死锁——这比事后做死锁检测更省事。
租约还带 TTL 兜底：车辆异常退出后，它占的工位会自动过期回收。
"""
from __future__ import annotations

import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fleetflow_interfaces.srv import AcquireLease, ReleaseLease

from . import layout
from .qos import state_qos


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

        self.ttl = float(self.get_parameter("lease_ttl_s").value)
        self.leases: dict[str, Lease] = {}
        self.waiting: dict[str, deque] = {}
        self.granted_total = 0
        self.rejected_total = 0
        self.expired_total = 0
        self.waits: dict[int, float] = {}          # robot_id -> 开始等待的时刻

        known = list(layout.all_station_points()) + ["charger_0", "charger_1"]
        self.known_resources = set(known)

        self.create_service(AcquireLease, "/traffic/acquire", self.on_acquire)
        self.create_service(ReleaseLease, "/traffic/release", self.on_release)
        self.pub = self.create_publisher(String, "/traffic/status", state_qos(5))
        self.create_timer(1.0, self.sweep)
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f"traffic_manager up · {len(known)} resources · lease ttl {self.ttl:.0f}s"
        )

    # ---------- 服务 ----------
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

        # 被占：拒绝并让调用方稍后重试（非阻塞，避免服务端持有上下文）
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
            if now - lease.since > lease.ttl:
                self.leases.pop(name, None)
                self.expired_total += 1
                self.get_logger().warn(f"lease expired: {name} (was robot {lease.robot_id})")

    def report(self):
        s = String()
        s.data = (
            f"traffic: held={len(self.leases)} waiting={sum(len(q) for q in self.waiting.values())} "
            f"granted={self.granted_total} rejected={self.rejected_total} expired={self.expired_total}"
        )
        self.pub.publish(s)


def main():
    from rclpy.executors import ExternalShutdownException

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
