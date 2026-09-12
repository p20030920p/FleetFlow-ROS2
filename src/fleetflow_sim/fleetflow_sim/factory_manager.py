"""工厂主控：维护物料流转与机器状态，产出搬运任务。

职责
----
* 物料抽象模型：每件物料有 id / 颜色 / 所在工位
* 机器按 process_s 加工，把物料变成下一道工序的颜色
* 任何需要位移的物料都生成一条 TransportTask，交给调度器
* 对外发布机器状态与汇总统计（供仪表盘渲染）

它不关心"哪台 AGV 来搬"，那是 task_scheduler 的事。

颜色语义（与旧版控制中心图例一致）：
    空桶区=empty → 梳棉等料位=empty → 梳棉完工位=green
    → 拉伸1等料位=green → 拉伸1完工位=yellow
    → 拉伸2等料位=yellow → 拉伸2完工位=red → 红料区/成品区
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32, String

from fleetflow_interfaces.msg import MachineState, TransportTask

from . import layout

FLOW = ["empty", "green", "yellow", "red"]


class Station:
    __slots__ = ("name", "x", "y", "material", "reserved")

    def __init__(self, name, x, y):
        self.name, self.x, self.y = name, x, y
        self.material = None      # 物料 id
        self.reserved = None      # 已被任务预定


class Machine:
    def __init__(self, spec, lane):
        self.name = f"{spec['name']}_machine_{lane}"
        self.stage = spec["name"]
        self.stage_idx = [s["name"] for s in layout.STAGES].index(spec["name"])
        self.lane = lane
        self.x, self.y = spec["machine_x"], spec["lanes"][lane]
        self.process_s = spec["process_s"]
        self.busy_until = 0.0
        self.done_count = 0
        self.busy_time = 0.0
        self.started = time.time()
        self.holding = None

    @property
    def state(self):
        return "processing" if self.holding is not None else "idle"


class Material:
    __slots__ = ("mid", "mtype", "where")

    def __init__(self, mid, mtype, where):
        self.mid, self.mtype, self.where = mid, mtype, where


class FactoryManager(Node):
    def __init__(self):
        super().__init__("factory_manager")
        self.declare_parameter("num_materials", 10)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("max_tasks_in_flight", 6)

        self.stations: dict[str, Station] = {}
        for key, z in layout.STORAGE.items():
            self.stations[f"storage_{key}"] = Station(f"storage_{key}", z["x"], z["y"])
        for s in layout.STAGES:
            for lane in range(len(s["lanes"])):
                for kind in ("waiting", "finished"):
                    st = layout.station(s["name"], kind, lane)
                    self.stations[st["name"]] = Station(st["name"], st["x"], st["y"])

        for name, pt in layout.STORAGE_SLOT_POINTS.items():
            self.stations[name] = Station(name, pt["x"], pt["y"])
        self.machines = [Machine(s, lane) for s in layout.STAGES for lane in range(len(s["lanes"]))]

        n = int(self.get_parameter("num_materials").value)
        self.materials: dict[int, Material] = {
            i: Material(i, "empty", "storage_empty") for i in range(n)
        }
        self.next_mid = n
        self.completed = 0

        self.tasks: dict[int, TransportTask] = {}
        self.task_material: dict[int, int] = {}   # task_id -> material id
        self.next_task_id = 1
        self.in_flight = 0

        self.pub_task = self.create_publisher(TransportTask, "/factory/tasks", 20)
        self.pub_machines = self.create_publisher(MachineState, "/factory/machines", 20)
        self.pub_summary = self.create_publisher(String, "/factory/summary", 10)
        self.pub_task_status = self.create_publisher(TransportTask, "/factory/task_status", 20)
        self.create_subscription(Int32, "/factory/completed", self.on_completed, 20)
        self.create_subscription(TransportTask, "/factory/task_status", self.on_task_status, 20)

        hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(2.0, self.reannounce)      # 周期性重播待认领任务
        self.create_timer(0.5, self.publish_summary)
        self.get_logger().info(
            f"factory_manager up · {len(self.materials)} units · {len(self.machines)} machines"
        )

    # ---------- 任务状态回调 ----------
    def on_task_status(self, msg: TransportTask):
        self.tasks[msg.task_id] = msg
        if msg.status == "failed":
            self.in_flight = max(0, self.in_flight - 1)
            for name in (msg.source_name, msg.dest_name):
                st = self.stations.get(name)
                if st is not None and st.reserved == msg.task_id:
                    st.reserved = None

    def on_completed(self, msg: Int32):
        """AGV 送达：物料落到目标工位，颜色按目标工位所属工序更新。"""
        task = self.tasks.get(msg.data)
        if task is None:
            return
        src = self.stations.get(task.source_name)
        dst = self.stations.get(task.dest_name)
        mid = self.task_material.get(task.task_id)
        if src is not None:
            if src.material == mid:
                src.material = None
            if src.reserved == task.task_id:
                src.reserved = None
        if dst is not None:
            # 停靠位是"临时泊位"，不驻留物料，否则这个位会被永久占用
            if "_slot_" not in dst.name:
                dst.material = mid
            dst.reserved = None
        mat = self.materials.get(mid)
        if mat is not None:
            if task.dest_name.startswith("storage_red"):
                # 进入成品/红料区视为完工
                self.materials.pop(mid, None)
                self.completed += 1
            else:
                mat.where = task.dest_name
                mat.mtype = task.material_type
        task.status = "done"
        self.tasks[task.task_id] = task
        self.pub_task_status.publish(task)   # 让调度器释放这台车
        self.in_flight = max(0, self.in_flight - 1)

    # ---------- 主循环 ----------
    def tick(self):
        now = time.time()
        self._output_from_machines(now)
        self._feed_machines(now)
        self._spawn_transports()

    def _output_from_machines(self, now):
        for m in self.machines:
            if m.holding is None or m.busy_until > now:
                continue
            out = self.stations[f"{m.stage}_finished_{m.lane}"]
            if out.material is not None:
                continue
            out.material = m.holding
            mat = self.materials.get(m.holding)
            if mat is not None:
                mat.where = out.name
                mat.mtype = FLOW[min(m.stage_idx + 1, len(FLOW) - 1)]
            m.holding = None
            m.done_count += 1

    def _feed_machines(self, now):
        for m in self.machines:
            if m.holding is not None:
                continue
            src = self.stations[f"{m.stage}_waiting_{m.lane}"]
            if src.material is None or src.reserved is not None:
                continue
            m.holding = src.material
            src.material = None
            m.busy_until = now + m.process_s
            m.busy_time += m.process_s

    def _free_slot(self, prefix):
        for st in self.stations.values():
            if st.name.startswith(prefix) and st.material is None and st.reserved is None:
                return st
        return None

    def _spawn_transports(self):
        limit = int(self.get_parameter("max_tasks_in_flight").value)

        # 1) 空桶区 → 梳棉等料位
        #    取货点不再是料区中心，而是环绕料区的空闲停靠位，避免所有车挤同一个点
        for mid, mat in list(self.materials.items()):
            if self.in_flight >= limit:
                return
            if mat.where != "storage_empty":
                continue
            dock = self._free_slot("storage_empty_slot_")
            dst = self._free_slot("carding_waiting_")
            if dock is None or dst is None:
                continue
            self._emit_task(mid, dock, dst)

        # 2) 某段完工位 → 下一段等料位；最后一段 → 红料区
        for mat in list(self.materials.values()):
            if self.in_flight >= limit:
                return
            src = self.stations.get(mat.where)
            if src is None or "_finished_" not in src.name or src.reserved is not None:
                continue
            stage_name = src.name.split("_finished_")[0]
            names = [s["name"] for s in layout.STAGES]
            i = names.index(stage_name)
            dst = (self._free_slot(f"{names[i+1]}_waiting_") if i + 1 < len(names)
                   else self._free_slot("storage_red_slot_"))
            if dst is None:
                continue
            self._emit_task(mat.mid, src, dst)

    def reannounce(self):
        """真实车队里任务公告是周期性的；这里重播所有 status=pending 的任务，
        使晚启动的订阅者也能拿到，避免"公告早于订阅"造成的任务丢失。"""
        n = 0
        for t in self.tasks.values():
            if t.status == "pending":
                self.pub_task.publish(t)
                n += 1
        if n:
            self.get_logger().debug(f"re-announced {n} pending tasks")

    def _emit_task(self, mid, src, dst):
        # 物料到达目标工位后的颜色 = 该工位所属工序填装后的状态
        if "_waiting_" in dst.name:
            stage_name = dst.name.split("_waiting_")[0]
            idx = [s["name"] for s in layout.STAGES].index(stage_name)
            mtype = FLOW[idx]
        else:
            mtype = "red" if dst.name == "storage_red" else self.materials[mid].mtype

        t = TransportTask()
        t.task_id = self.next_task_id
        t.priority = int(max(0.0, 12.0 - mid * 0.4))
        t.material_type = mtype
        t.source_name = src.name
        t.dest_name = dst.name
        t.source_x, t.source_y = float(src.x), float(src.y)
        t.dest_x, t.dest_y = float(dst.x), float(dst.y)
        t.status = "pending"
        t.robot_id = -1
        self.tasks[t.task_id] = t
        self.task_material[t.task_id] = mid
        self.next_task_id += 1
        self.in_flight += 1
        src.reserved = t.task_id
        dst.reserved = t.task_id     # 目标工位同样要占住，否则多个任务会挤同一个槽位
        self.pub_task.publish(t)

    # ---------- 对外状态 ----------
    def publish_summary(self):
        for m in self.machines:
            msg = MachineState()
            msg.name = m.name
            msg.stage = m.stage
            msg.state = m.state
            msg.x, msg.y = float(m.x), float(m.y)
            wait = self.stations[f"{m.stage}_waiting_{m.lane}"]
            fin = self.stations[f"{m.stage}_finished_{m.lane}"]
            msg.waiting_station = wait.name
            msg.finished_station = fin.name
            msg.input_count = 1 if wait.material is not None else 0
            msg.output_count = m.done_count + (1 if fin.material is not None else 0)
            uptime = max(0.1, time.time() - m.started)
            msg.busy_ratio = float(min(1.0, m.busy_time / uptime))
            self.pub_machines.publish(msg)

        by_color = {c: 0 for c in FLOW}
        for mat in self.materials.values():
            by_color[mat.mtype] = by_color.get(mat.mtype, 0) + 1
        s = String()
        s.data = json.dumps(
            dict(
                materials=len(self.materials),
                completed=self.completed,
                by_color=by_color,
                pending=sum(1 for t in self.tasks.values() if t.status == "pending"),
                running=sum(1 for t in self.tasks.values() if t.status in ("assigned", "running")),
                done=sum(1 for t in self.tasks.values() if t.status == "done"),
                tasks_created=self.next_task_id - 1,
            )
        )
        self.pub_summary.publish(s)


def main():
    rclpy.init()
    node = FactoryManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
