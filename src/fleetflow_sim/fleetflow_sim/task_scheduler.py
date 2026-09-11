"""任务调度器：优先级队列 + 就近分配。

采用 pull 模型：AGV 空闲时调用 /scheduler/request_task 主动拉活，
调度器在候选任务里挑"优先级最高、且离该车最近"的一条给它。

好处是不会出现两台车抢同一个任务，也不需要中心节点维护每台车的忙碌状态。
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fleetflow_interfaces.msg import RobotStatus, TransportTask
from fleetflow_interfaces.srv import RequestTask


class TaskScheduler(Node):
    def __init__(self):
        super().__init__("task_scheduler")
        self.declare_parameter("reserve_radius", 1.6)

        self.pending: dict[int, TransportTask] = {}
        self.running: dict[int, int] = {}      # task_id -> robot_id
        self.robots: dict[int, RobotStatus] = {}
        self.assigned_count = 0

        self.create_subscription(TransportTask, "/factory/tasks", self.on_task, 30)
        self.create_subscription(RobotStatus, "/fleet/robots", self.on_robot, 30)
        self.create_subscription(TransportTask, "/factory/task_status", self.on_status, 30)
        self.srv = self.create_service(RequestTask, "/scheduler/request_task", self.on_request)
        self.pub_status = self.create_publisher(TransportTask, "/factory/task_status", 30)
        self.create_timer(2.0, self.log_state)
        self.get_logger().info("task_scheduler up · waiting for tasks")

    # ---------- 输入 ----------
    def on_task(self, msg: TransportTask):
        if msg.task_id in self.running:
            return
        self.pending[msg.task_id] = msg

    def on_robot(self, msg: RobotStatus):
        self.robots[msg.robot_id] = msg
        # 自愈：车辆报告空闲，却还挂在某个任务上，说明完成通知丢了，这里补一次释放
        if msg.state == "idle" and msg.task_id == -1:
            for tid, rid in list(self.running.items()):
                if rid == msg.robot_id:
                    self.running.pop(tid, None)

    def on_status(self, msg: TransportTask):
        if msg.status in ("done", "failed"):
            self.pending.pop(msg.task_id, None)
            self.running.pop(msg.task_id, None)

    # ---------- 分配 ----------
    def _score(self, task: TransportTask, x: float, y: float):
        """越小越优先：先看优先级，再看取货点距离。"""
        d = math.hypot(task.source_x - x, task.source_y - y)
        return (task.priority, d)

    def on_request(self, req: RequestTask.Request, res: RequestTask.Response):
        # 已经被别的车抢走的任务不再分配
        busy_ids = set(self.running.values())
        if req.robot_id in busy_ids:
            res.has_task = False
            res.message = "robot already holds a task"
            return res
        cands = [t for t in self.pending.values() if t.status == "pending"]
        if not cands:
            res.has_task = False
            res.message = "no pending task"
            return res
        best = min(cands, key=lambda t: self._score(t, req.x, req.y))
        best.status = "assigned"
        best.robot_id = req.robot_id
        self.running[best.task_id] = req.robot_id
        self.pending.pop(best.task_id, None)
        self.assigned_count += 1
        self.pub_status.publish(best)
        res.has_task = True
        res.task = best
        res.message = f"task {best.task_id} -> robot {req.robot_id}"
        self.get_logger().info(
            f"assign task {best.task_id} [{best.material_type}] "
            f"{best.source_name} -> {best.dest_name}  to robot {req.robot_id}"
        )
        return res

    def log_state(self):
        s = String()
        s.data = (
            f"scheduler: pending={len(self.pending)} running={len(self.running)} "
            f"assigned_total={self.assigned_count} robots_seen={len(self.robots)}"
        )
        self.get_logger().debug(s.data)


def main():
    rclpy.init()
    node = TaskScheduler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
