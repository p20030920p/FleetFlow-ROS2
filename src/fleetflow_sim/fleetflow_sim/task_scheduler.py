"""任务调度器：可切换的分配策略 + 车辆心跳看门狗。

分配策略由参数 ``policy`` 选择（见 policies.py）：

* ``random``  —— 随机分配，作为下界基线
* ``nearest`` —— 请求车辆取离自己最近的一单（去中心化，只看局部信息）
* ``ssi``     —— 顺序单件拍卖：在所有 (空闲车, 待办任务) 组合里
                 反复挑**空驶距离**最小的一对成交（Lagoudakis et al., 2005）
* ``ca_ssi``  —— 本文方法。代价函数加入工位拥塞、电量可达性、车队负载均衡
* ``hungarian``—— 匈牙利最优指派，作为单轮静态最优的参照上界

仍然采用 pull 模型：车辆空闲时会来问一次，调度器把"这一轮该给它什么"告诉它。
SSI 需要全局视野，所以调度器在每次请求时用 ``/fleet/robots`` 里的空闲车位置
重算一次派单表，车辆来问时取走属于自己的那一条——既保留了 pull 的无竞争特性，
又能表达集中式拍卖的分配结果。

看门狗：某台车拿着任务却长时间没有状态上报（掉线/崩溃），调度器把任务收回
队列并记录一次 reassignment，即真实系统里的"任务回收重派"。
"""
from __future__ import annotations

import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fleetflow_interfaces.msg import MachineState, RobotStatus, TransportTask
from fleetflow_interfaces.srv import RequestTask

from . import policies
from .qos import state_qos


class TaskScheduler(Node):
    def __init__(self):
        super().__init__("task_scheduler")
        self.declare_parameter("policy", "nearest")
        self.declare_parameter("robot_timeout_s", 8.0)
        self.declare_parameter("seed", 7)

        self.policy = str(self.get_parameter("policy").value)
        self.rng = random.Random(int(self.get_parameter("seed").value))
        if self.policy not in policies.POLICIES:
            self.get_logger().warn(f"unknown policy {self.policy}, falling back to nearest")
            self.policy = "nearest"

        self.pending: dict[int, TransportTask] = {}
        self.running: dict[int, int] = {}          # task_id -> robot_id
        self.robots: dict[int, RobotStatus] = {}
        self.last_seen: dict[int, float] = {}
        self.assigned_count = 0
        self.reassignments = 0
        self.plan_table: dict[int, TransportTask] = {}
        self.created_at: dict[int, float] = {}     # 任务入队时刻（算老化用）
        self.last_task: dict[int, TransportTask] = {}
        self.machines: dict[str, MachineState] = {}

        self.create_subscription(TransportTask, "/factory/tasks", self.on_task, state_qos(40))
        self.create_subscription(RobotStatus, "/fleet/robots", self.on_robot, state_qos(40))
        self.create_subscription(TransportTask, "/factory/task_status", self.on_status, state_qos(40))
        self.create_subscription(MachineState, "/factory/machines", self.on_machine, state_qos(40))
        self.srv = self.create_service(RequestTask, "/scheduler/request_task", self.on_request)
        self.pub_status = self.create_publisher(TransportTask, "/factory/task_status", state_qos(40))
        self.create_timer(1.0, self.watchdog)
        self.create_timer(3.0, self.report)
        self.get_logger().info(f"task_scheduler up · policy={self.policy}")

    # ---------- 输入 ----------
    def on_task(self, m: TransportTask):
        if m.task_id not in self.running:
            self.pending[m.task_id] = m
            self.last_task[m.task_id] = m
            self.created_at.setdefault(m.task_id, time.time())
            self.plan_table.clear()

    def on_machine(self, m: MachineState):
        self.machines[m.name] = m

    def on_robot(self, m: RobotStatus):
        self.robots[m.robot_id] = m
        self.last_seen[m.robot_id] = time.time()
        # 自愈：车辆报告空闲却还挂在某任务上，说明完成通知丢了
        if m.state == "idle" and m.task_id == -1:
            for tid, rid in list(self.running.items()):
                if rid == m.robot_id:
                    self.running.pop(tid, None)
                    self.plan_table.clear()

    def on_status(self, m: TransportTask):
        if m.status in ("done", "failed"):
            self.pending.pop(m.task_id, None)
            self.running.pop(m.task_id, None)
            self.created_at.pop(m.task_id, None)
            self.plan_table.clear()

    # ---------- 分配 ----------
    def _idle_robots(self) -> dict:
        return {rid: (r.x, r.y) for rid, r in self.robots.items()
                if r.state == "idle" and r.task_id == -1}

    def _fleet_state(self) -> dict:
        """空闲车队的完整状态：位置 + 电量 + 累计里程。

        SSI 只需要 ``(x, y)``；CA-SSI 还需要电量（可达性）与累计里程（均衡），
        这正是它比 SSI 多出来的信息量。
        """
        out = {}
        for rid, r in self.robots.items():
            if r.state != "idle" or r.task_id != -1:
                continue
            out[rid] = {
                "x": float(r.x), "y": float(r.y),
                "battery": float(getattr(r, "battery", 100.0)),
                "odom": float(getattr(r, "odom_total", 0.0)),
            }
        return out

    def _dock_load(self) -> dict:
        """每个工位当前"有多少台车正在赶来或已占用"。

        这就是 SSI 看不见的那一维信息：纺织车间每个机台只有 1 个对接位，
        同一工位被 3 台车同时盯上就会互相等待。这里把
        (a) 已在途任务的起终点   (b) 机台自身的待加工队列
        都折算成拥塞度。
        """
        load: dict[str, int] = {}
        for tid, rid in self.running.items():
            t = self.last_task.get(tid)
            if t is None:
                continue
            load[t.source_name] = load.get(t.source_name, 0) + 1
            load[t.dest_name] = load.get(t.dest_name, 0) + 1
        for m in self.machines.values():
            q = int(getattr(m, "input_count", 0))
            if q <= 0:
                continue
            # 关键：必须累加到**工位名**上（carding_waiting_1），
            # 而不是机台名（carding_1）——任务的起终点用的是前者。
            key = getattr(m, "waiting_station", "") or m.name
            load[key] = load.get(key, 0) + min(q, 3)   # 队列最多计 3
        return load

    def _age(self, task) -> float:
        return max(0.0, time.time() - self.created_at.get(task.task_id, time.time()))

    def _recompute_plan(self):
        """集中式策略：反复取代价最小的 (车, 任务) 对成交，得到本轮派单表。

        ``ssi`` / ``ca_ssi`` / ``hungarian`` 三者共用这一拍卖流程，只有代价
        函数不同，因此结果差异完全来自代价函数本身（可做消融分析）。
        """
        self.plan_table = {}
        cands = [t for t in self.pending.values() if t.status == "pending"]
        if self.policy == "ssi":
            idle = dict(self._idle_robots())
            while cands and idle:
                pick = policies.pick_ssi(cands, idle, self.rng)
                if pick is None:
                    break
                rid, task = pick
                self.plan_table[rid] = task
                cands = [t for t in cands if t.task_id != task.task_id]
                idle.pop(rid, None)
            return

        fleet = self._fleet_state()
        load = self._dock_load()
        picker = policies.PICKERS[self.policy]
        while cands and fleet:
            pick = picker(cands, fleet, self.rng, now=time.time(), dock_load=load)
            if pick is None:
                break
            rid, task = pick
            self.plan_table[rid] = task
            cands = [t for t in cands if t.task_id != task.task_id]
            fleet.pop(rid, None)

    def on_request(self, req: RequestTask.Request, res: RequestTask.Response):
        if int(req.robot_id) in set(self.running.values()):
            res.has_task, res.message = False, "robot already holds a task"
            return res
        cands = [t for t in self.pending.values() if t.status == "pending"]
        self._reqs = getattr(self, "_reqs", 0) + 1
        if self._reqs in (1, 100, 400):
            self.get_logger().info(
                f"request #{self._reqs} from robot {req.robot_id}: "
                f"pending={len(self.pending)} usable={len(cands)} running={len(self.running)}")
        if not cands:
            res.has_task, res.message = False, "no pending task"
            return res

        chosen = None
        if self.policy == "random":
            chosen = policies.pick_random(cands, (req.x, req.y), self.rng)
        elif self.policy == "nearest":
            chosen = policies.pick_nearest(cands, (req.x, req.y), self.rng)
        else:  # ssi / ca_ssi / hungarian —— 集中式拍卖
            if not self.plan_table:
                self._recompute_plan()
            chosen = self.plan_table.pop(int(req.robot_id), None)
            if chosen is not None and chosen.task_id not in self.pending:
                chosen = None
            if chosen is None:
                res.has_task, res.message = False, "auction did not award this robot"
                return res

        if chosen is None:
            res.has_task, res.message = False, "nothing suitable"
            return res

        chosen.status = "assigned"
        chosen.robot_id = int(req.robot_id)
        self.last_task[chosen.task_id] = chosen
        self.running[chosen.task_id] = int(req.robot_id)
        self.pending.pop(chosen.task_id, None)
        self.assigned_count += 1
        self.plan_table.clear()
        self.pub_status.publish(chosen)
        res.has_task, res.task = True, chosen
        res.message = f"task {chosen.task_id} -> robot {req.robot_id} ({self.policy})"
        self.get_logger().info(
            f"[{self.policy}] task {chosen.task_id} [{chosen.material_type}] "
            f"{chosen.source_name} -> {chosen.dest_name} => robot {req.robot_id}"
        )
        return res

    # ---------- 看门狗 ----------
    def watchdog(self):
        now = time.time()
        timeout = float(self.get_parameter("robot_timeout_s").value)
        for tid, rid in list(self.running.items()):
            if now - self.last_seen.get(rid, now) < timeout:
                continue
            self.running.pop(tid, None)
            self.reassignments += 1
            self.get_logger().warn(f"robot {rid} silent for >{timeout:.0f}s, reclaiming task {tid}")
            self.plan_table.clear()

    def report(self):
        idle = len(self._idle_robots())
        self.get_logger().info(
            f"POOL pending={len(self.pending)} running={len(self.running)} "
            f"idle={idle} fleet={len(self.robots)} assigned={self.assigned_count}")
        s = String()
        s.data = (f"scheduler[{self.policy}]: pending={len(self.pending)} "
                  f"running={len(self.running)} assigned={self.assigned_count} "
                  f"reassign={self.reassignments}")
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
