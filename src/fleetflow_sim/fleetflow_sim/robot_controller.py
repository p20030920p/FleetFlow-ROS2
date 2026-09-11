"""单台 AGV 的控制器：取货 → 送货 的状态机 + A* 路径跟随。

每台车一个实例，运行在自己的命名空间 robot_<id> 下：
    订阅  <ns>/odom            里程计（Gazebo DiffDrive 插件发布）
    发布  <ns>/cmd_vel         速度指令
    发布  /fleet/robots        车队状态（供调度器与仪表盘消费）
    发布  /factory/completed   送达回执
    调用  /scheduler/request_task  空闲时主动拉活

状态机：IDLE → TO_PICKUP → LOADING → TO_DROPOFF → UNLOADING → IDLE
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32

from fleetflow_interfaces.msg import RobotStatus, TransportTask
from fleetflow_interfaces.srv import RequestTask

from . import layout
from .planner import Grid, PurePursuit, plan

# 与 Gazebo 世界共用的静态栅格（障碍不变，建一次即可）
_GRID = None


def shared_grid() -> Grid:
    global _GRID
    if _GRID is None:
        _GRID = Grid()
    return _GRID


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("start_x", 1.0)
        self.declare_parameter("start_y", 5.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("dwell_s", 1.2)
        # 无 Gazebo 的"纯逻辑"模式：自行积分运动学。可在没有仿真器/GPU 的机器上
        # 验证调度逻辑，也用于低成本出图。
        self.declare_parameter("use_internal_kinematics", False)

        self.rid = int(self.get_parameter("robot_id").value)
        self.ns = f"robot_{self.rid}"
        self.dwell = float(self.get_parameter("dwell_s").value)
        self.self_kin = bool(self.get_parameter("use_internal_kinematics").value)

        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.yaw = float(self.get_parameter("start_yaw").value)

        self.state = "idle"
        self.task: TransportTask | None = None
        self.timer_until = 0.0
        self.grid = shared_grid()
        self.pursuit = PurePursuit()
        self.total_len = 0.0
        self.seen_len = 0.0

        self.pub_cmd = self.create_publisher(Twist, f"/{self.ns}/cmd_vel", 10)
        self.pub_status = self.create_publisher(RobotStatus, "/fleet/robots", 10)
        self.pub_done = self.create_publisher(Int32, "/factory/completed", 10)
        if not self.self_kin:
            self.create_subscription(Odometry, f"/{self.ns}/odom", self.on_odom, 10)
        self.cli = self.create_client(RequestTask, "/scheduler/request_task")

        self.create_timer(0.1, self.loop)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(
            f"{self.ns} controller up at ({self.x:.1f}, {self.y:.1f})"
            + ("  [internal kinematics]" if self.self_kin else "  [gazebo odom]")
        )

    # ---------- 里程计 ----------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def publish_status(self):
        m = RobotStatus()
        m.robot_id = self.rid
        m.state = self.state
        m.x, m.y, m.yaw = float(self.x), float(self.y), float(self.yaw)
        m.task_id = self.task.task_id if self.task else -1
        m.distance_total = float(self.total_len)
        m.distance_done = float(self.seen_len)
        self.pub_status.publish(m)

    # ---------- 主循环 ----------
    def loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.state == "idle":
            self._try_take_task()
        elif self.state in ("to_pickup", "to_dropoff"):
            self._follow(now)
        elif self.state in ("loading", "unloading"):
            self._hold(now)

    def _hold(self, now):
        self._stop()
        if now >= self.timer_until:
            if self.state == "loading":
                self._go_to(self.task.dest_x, self.task.dest_y, "to_dropoff")
            else:
                done = Int32()
                done.data = self.task.task_id
                self.pub_done.publish(done)
                self.get_logger().info(f"{self.ns} delivered task {self.task.task_id}")
                self.task = None
                self.state = "idle"

    def _try_take_task(self):
        if not self.cli.service_is_ready():
            return
        req = RequestTask.Request()
        req.robot_id = self.rid
        req.x, req.y = float(self.x), float(self.y)
        fut = self.cli.call_async(req)
        fut.add_done_callback(self._on_task)

    def _on_task(self, fut):
        if self.state != "idle":
            return
        try:
            res = fut.result()
        except Exception:
            return
        if res is None or not res.has_task:
            return
        self.task = res.task
        self.get_logger().info(
            f"{self.ns} <- task {self.task.task_id} [{self.task.material_type}] "
            f"{self.task.source_name} -> {self.task.dest_name}"
        )
        self._go_to(self.task.source_x, self.task.source_y, "to_pickup")

    def _go_to(self, gx, gy, state):
        path = plan(self.grid, (self.x, self.y), (gx, gy))
        if not path:
            self.get_logger().warn(f"{self.ns} no path to ({gx:.1f},{gy:.1f})")
            self.state = "idle"
            self.task = None
            return
        path = path + [(gx, gy)]
        self.pursuit.set_path(path)
        self.total_len = self.pursuit.remaining()
        self.seen_len = 0.0
        self.state = state

    def _follow(self, now):
        if self.task is None:
            self.state = "idle"
            return
        v, w = self.pursuit.step(self.x, self.y, self.yaw)
        self.seen_len = max(0.0, self.total_len - self.pursuit.remaining())
        arrived = self.pursuit.finished or (
            math.hypot(
                self.task.source_x - self.x if self.state == "to_pickup" else self.task.dest_x - self.x,
                self.task.source_y - self.y if self.state == "to_pickup" else self.task.dest_y - self.y,
            )
            < 0.18
        )
        if arrived:
            self._stop()
            self.timer_until = now + self.dwell
            self.state = "loading" if self.state == "to_pickup" else "unloading"
            return
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        if self.self_kin:
            # 一阶运动学积分，dt 与 loop 定时器一致
            dt = 0.1
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw += w * dt

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
