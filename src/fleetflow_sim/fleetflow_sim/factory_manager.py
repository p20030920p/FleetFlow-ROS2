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
        # ---------------------------------------------------------------
        # 逐工序归因（第 57 节的下一步）。
        #
        # 只知道"成品少"没法定位：可能是机器在等上游料（**饿**），
        # 也可能是机器闲着手、但完工位被占住没人运走（**堵**）。
        # 两者修法完全不同，所以把空闲时间按原因拆开累计：
        #   idle_starved  : 空闲，且本工序 waiting 位没料  -> 上游运输没送到
        #   idle_blocked  : 空闲，waiting 有料但吃不下     -> 完工位/下游堵住
        # 判定在 _feed_machines 里做（那里才知道 waiting 位有没有料）。
        self.idle_starved_time = 0.0
        self.idle_blocked_time = 0.0
        self._last_t = time.time()
        self._last_state = "idle"
        self._last_blocked = False

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
        # 36 = 任务比例 36:36:6:1 的第一项（投料单数），见 README
        self.declare_parameter("num_materials", 36)
        # 成品是否回收复用（关掉 = 每件料只走一遍，产线会跑空）
        self.declare_parameter("recycle_finished", True)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("max_tasks_in_flight", 6)
        # 任务在途超过这么久没有推进就回收（秒）。没有这个兜底，
        # 一辆卡住的车会把派单预算永久占死 —— 见 in_flight 的注释。
        self.declare_parameter("stale_task_timeout_s", 45.0)
        # 投料 / 推进的派单优先级（第 55 节）。
        #   True（默认）= 先推进完工位，再投料新桶
        #   False        = 旧行为，先投料（实测会把在途额度吃满，饿死推进）
        # 默认 False = 保留**旧顺序**（先投料、后推进）。
        # 第 56 节的 600 s 配对实验里，改成"先推进"反而更差
        # （成品 2.33 vs 5.67 件/600 s），所以改回默认旧行为，
        # 参数保留以便复现那组测量。
        self.declare_parameter("advance_first", False)

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

        self.recycle_finished = bool(self.get_parameter("recycle_finished").value)
        n = int(self.get_parameter("num_materials").value)
        self.materials: dict[int, Material] = {
            i: Material(i, "empty", "storage_empty") for i in range(n)
        }
        self.next_mid = n
        self.completed = 0

        self.tasks: dict[int, TransportTask] = {}
        self.task_material: dict[int, int] = {}   # task_id -> material id
        self.next_task_id = 1
        # in_flight 改为**由状态推导**，不再靠 += / -= 维护。
        # 旧实现只在 on_completed 与 on_task_status(failed) 两处递减，而 failed
        # 只有 robot_controller._abort_task 一个发布者、且只由看门狗触发。
        # 于是一辆卡在 to_pickup 的车既不送达也不上报失败，in_flight 永久多记，
        # 而 _spawn_transports 开头就是 `if in_flight >= limit: return` —— 工厂
        # 从此不再派单。实测：tasks_created=6 而 done=1，in_flight≈5（上限 6），
        # 整场只剩最初那几单，这正是"零产出"运行的共同特征。
        self.in_flight = 0            # 缓存值，每帧由 _recount_in_flight() 刷新
        self.task_started: dict[int, float] = {}   # task_id -> 首次在途时刻
        self.reclaimed = 0

        self.pub_task = self.create_publisher(TransportTask, "/factory/tasks", 20)
        self.pub_machines = self.create_publisher(MachineState, "/factory/machines", 20)
        self.pub_summary = self.create_publisher(String, "/factory/summary", 10)
        self.pub_task_status = self.create_publisher(TransportTask, "/factory/task_status", 20)
        self.create_subscription(Int32, "/factory/completed", self.on_completed, 20)
        self.create_subscription(TransportTask, "/factory/task_status", self.on_task_status, 20)
        self.advance_first = bool(self.get_parameter("advance_first").value)
        self.get_logger().info(f"pipeline at start: {self.pipeline_snapshot()}")
        # 每 10 s 打一行管线水位（INFO，所以默认就能看到）：
        # 这是解释"为什么这次只跑了 12 单"的唯一现场记录。
        self.create_timer(10.0, self.log_pipeline)

        hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(2.0, self.reannounce)      # 周期性重播待认领任务
        self.create_timer(0.5, self.publish_summary)
        self.get_logger().info(
            f"factory_manager up · {len(self.materials)} units · {len(self.machines)} machines"
        )

    # ---------- 任务状态回调 ----------
    def on_task_status(self, msg: TransportTask):
        """只吸收**状态**，不要整体覆盖本地任务副本。

        这里曾经写成 `self.tasks[msg.task_id] = msg`，而 /factory/task_status
        的发布者不止一个（调度器派单时也发），于是工厂自己那份任务会被
        外部 publish 的版本替换掉 —— 账本对象不该被外部覆盖。
        """
        local = self.tasks.get(msg.task_id)
        if local is not None:
            local.status = msg.status
        else:
            self.tasks[msg.task_id] = msg
        if msg.status == "failed":
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
                # 进入成品/红料区视为完工。
                #
                # **成品回收复用**：早期实现是 `materials.pop()`，于是每件料只能
                # 走一遍全流程，跑完 N 件之后整座厂就再也没有可搬的料 ——
                # 表现为"跑着跑着彻底停了"。真实车间的条筒是循环使用的，
                # 所以这里把料放回空筒库重新入线，产线才能持续运转。
                # 这样任务比例也才能稳定在 36:36:6:1（见 README 的说明）。
                if self.recycle_finished:
                    mat.where = "storage_empty"
                    mat.mtype = "empty"
                else:
                    self.materials.pop(mid, None)
                self.completed += 1
            else:
                mat.where = task.dest_name
                mat.mtype = task.material_type
        task.status = "done"
        self.tasks[task.task_id] = task
        self.pub_task_status.publish(task)   # 让调度器释放这台车
        # in_flight 由 _recount_in_flight() 推导，不再手动递减

    # ---------- 在途记账 ----------
    def _recount_in_flight(self, now: float):
        """由任务状态重算 in_flight，并回收长期无进展的任务。

        推导而非累加：只要有一条路径漏掉递减，累加器就会永久漂移，
        而漂移的后果是**整座工厂停止派单**。
        """
        alive = 0
        for tid, t in list(self.tasks.items()):
            if t.status in ("done", "failed"):
                self.task_started.pop(tid, None)
                continue
            alive += 1
            self.task_started.setdefault(tid, now)
        self.in_flight = alive

        # 陈旧任务回收：真实仓储系统里就是"任务超时回收"，
        # 不能指望每台车都老实上报自己的失败。
        timeout = float(self.get_parameter("stale_task_timeout_s").value)
        for tid, t in list(self.tasks.items()):
            if t.status in ("done", "failed"):
                continue
            started = self.task_started.get(tid, now)
            if now - started < timeout:
                continue
            self.reclaimed += 1
            self.get_logger().warn(
                f"task {tid} stale {now - started:.0f}s "
                f"(src={t.source_name} dst={t.dest_name}) -> reclaim")
            for name in (t.source_name, t.dest_name):
                st = self.stations.get(name)
                if st is not None and st.reserved == tid:
                    st.reserved = None
            t.status = "failed"
            self.pub_task_status.publish(t)      # 让调度器释放这台车
            self.task_started.pop(tid, None)

    # ---------- 主循环 ----------
    def tick(self):
        now = time.time()
        self._output_from_machines(now)
        self._feed_machines(now)
        self._recount_in_flight(now)
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

    def pipeline_snapshot(self) -> dict:
        """物料此刻"堆在哪一段"，用来解释吞吐抖动。

        第 42 节量到一件反直觉的事：等待派单的时间几乎是 0
        （mean=0.0s, max=0.1s），派单本身从来不是瓶颈；而不同运行的
        产出从 12 单到 36 单差 3 倍，运输时间却是稳定的 18~21 s。
        差值只能来自**工厂往里放料的速率**。

        这正是布局决定的结构性上限：
          * 只有 `*_finished_*` 上的物料才会被派单运输；
          * 一台机器只有在自己的 `waiting` 位空着时才吃料；
          * 而每台机器的 `finished` 位只有一个。
        所以如果下一工序的 waiting 位被占满，上一工序的 finished 位
        就会堵住、机器停转、任务流断供 —— 表现为"跑着跑着没单了"。
        """
        at = {"waiting": 0, "finished": 0, "machine": 0, "storage": 0}
        for mat in self.materials.values():
            name = mat.where or ""
            if name.startswith("storage_"):
                at["storage"] += 1
            elif "_waiting_" in name:
                at["waiting"] += 1
            elif "_finished_" in name:
                at["finished"] += 1
        for m in self.machines:
            if m.holding is not None:
                at["machine"] += 1
        # 每个下一工序还剩几个 waiting 位可收（= 还能放多少料进去）
        room = {}
        names = [x["name"] for x in layout.STAGES]
        for i, st in enumerate(layout.STAGES):
            if i + 1 >= len(names):
                continue
            nxt = names[i + 1]
            free = sum(1 for k, v in self.stations.items()
                       if k.startswith(f"{nxt}_waiting_") and v.material is None)
            room[st["name"]] = free
        return dict(**at, in_flight=self.in_flight, room=room)

    def log_pipeline(self):
        s = self.pipeline_snapshot()
        room = ",".join(f"{k}:{v}" for k, v in s.pop("room").items())
        self.get_logger().info(
            f"pipeline {s} · 下游可收 {room} · done={self.completed}")
        # 逐工序归因：按 stage 汇总忙/饿/堵（工程单位：秒）
        agg: dict[str, list[float]] = {}
        for m in self.machines:
            a = agg.setdefault(m.stage, [0.0, 0.0, 0.0])
            a[0] += m.busy_time
            a[1] += m.idle_starved_time
            a[2] += m.idle_blocked_time
        parts = []
        for st in [x["name"] for x in layout.STAGES]:
            if st not in agg:
                continue
            busy, starved, blocked = agg[st]
            tot = busy + starved + blocked
            if tot <= 0:
                continue
            parts.append(f"{st}: 忙{busy/tot:4.0%} 饿{starved/tot:4.0%} 堵{blocked/tot:4.0%}")
        if parts:
            self.get_logger().info("stage 忙/饿/堵 · " + " · ".join(parts))

    def _feed_machines(self, now):
        for m in self.machines:
            # 先结算上一段空闲时间属于哪一类（用**上一次**的观测）
            dt = now - m._last_t
            if dt > 0 and m.holding is None:
                if m._last_blocked:
                    m.idle_blocked_time += dt
                else:
                    m.idle_starved_time += dt
            m._last_t = now
            if m.holding is not None:
                continue
            src = self.stations[f"{m.stage}_waiting_{m.lane}"]
            # waiting 位有料 = 上游送到了，只是还没轮到（或刚吃完）
            m._last_blocked = src.material is not None
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
        if self.advance_first:
            self._spawn_advance(limit)
            self._spawn_inject(limit)
        else:
            self._spawn_inject(limit)
            self._spawn_advance(limit)

    def _spawn_advance(self, limit):
        """完工位 → 下一段等料位（最后一段 → 红料区）。"""

        # ---------------------------------------------------------------
        # **顺序就是优先级，而原来的顺序是反的。**
        #
        # 旧写法把"空筒区 → 梳棉等料位"（以下称**投料**）放在第 1 段，
        # 把"完工位 → 下一段等料位"（以下称**推进**）放在第 2 段。
        # 而两段开头都有 `if self.in_flight >= limit: return` ——
        # 于是只要有空筒取放位和空的梳棉等料位，投料就会把在途额度**吃满**，
        # 第 2 段永远轮不到。实测（第 55 节）：
        #
        #     storage_empty → carding   派单 25 次
        #     carding_finished → drawing 派单 11 次，其中只有 1 单真正送到
        #
        # 结果是产线**只进不出**：物料一批批灌进梳棉，却卡在完工位上运不走，
        # 整场 180 s 只做出 1~4 件成品。反映在账上就是"总送达 34 单、
        # 其中 17 单是投料、成品只有 4 件"。
        #
        # 现在把顺序倒过来：**先推进、后投料**。物料在产线里往前走才是
        # 产出，投料只是喂料；喂料不该挤掉推进。
        # ---------------------------------------------------------------

        # 1) 某段完工位 → 下一段等料位；最后一段 → 红料区
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

    def _spawn_inject(self, limit):
        """空桶区 → 梳棉等料位。

        取货点不再是料区中心，而是环绕料区的空闲停靠位，避免所有车挤同一个点。
        """
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
        snap = self.pipeline_snapshot()
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
                in_flight=self.in_flight,
                reclaimed=self.reclaimed,
                # 注意：snapshot 里已经带了 in_flight，不能再展开进来
                # —— dict() 会因关键字重复直接抛 TypeError，把工厂打死。
                waiting=snap["waiting"], finished=snap["finished"],
                machine=snap["machine"], storage=snap["storage"],
                room=snap["room"],
            )
        )
        self.pub_summary.publish(s)


def main():
    layout.bootstrap()      # 先定布局，再建节点
    rclpy.init()
    node = FactoryManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 见 robot_controller.main：关闭时 destroy_node 可能抛异常并刷 Traceback，
        # 无害但会让人以为启动失败，统一吞掉。
        try:
            node.destroy_node()
        except Exception:                                     # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
