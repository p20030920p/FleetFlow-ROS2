"""指标采集：把一次运行变成可分析的 CSV。

学术上评价一个多机调度系统，光说"跑通了"没有意义，得有量化的指标。这里采集：

* **makespan** —— 首次派单到最后一单送达的墙钟时间
* **throughput** —— 每分钟完成任务数
* **task latency** —— 每单从生成到送达的时延（均值 / 中位 / P95）
* **robot utilisation** —— 每台车"在动或在装卸"的时间占比
* **distance travelled** —— 车队总行驶里程
* **traffic wait** —— 交通租约平均等待时间（衡量死锁/拥塞压力）
* **min inter-robot distance** —— 任意两车最近距离（安全指标，避免碰撞的证据）
* **charging events / reassignments** —— 电量与看门狗触发的行为次数

数据增量写盘，所以即使实验被中途 kill，已完成的样本也不会丢。
"""
from __future__ import annotations

import csv
import json
import math
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from fleetflow_interfaces.msg import RobotStatus, TransportTask

from .qos import state_qos
from .traffic import circle_gap

TASK_FIELDS = ["task_id", "material", "source", "dest", "robot_id",
               "created_s", "assigned_s", "delivered_s", "latency_s", "travel_cost"]
RUN_FIELDS = ["run_label", "policy", "num_robots", "wall_s", "makespan_s", "completed",
              "throughput_per_min", "latency_mean_s", "latency_p50_s", "latency_p95_s",
              "utilisation_mean", "distance_total_m", "traffic_rejected", "traffic_expired",
              "min_robot_distance_m", "near_miss_events", "charging_events", "reassignments",
              # 轮廓净距与真实重叠次数：车心距在密集车队里没有安全含义（两车并排
              # 车心距本来就只有 0.44 m），新增这两列才是可与控制器判据对照的指标。
              "min_robot_gap_m", "overlap_events",
              "dup_target_ticks", "dup_target_events"]


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__("metrics_recorder")
        self.declare_parameter("out_dir", "/tmp/fleetflow_metrics")
        self.declare_parameter("policy", "nearest")
        self.declare_parameter("run_label", "run")
        self.declare_parameter("near_miss_m", 0.55)

        self.out_dir = str(self.get_parameter("out_dir").value)
        self.policy = str(self.get_parameter("policy").value)
        self.label = str(self.get_parameter("run_label").value)
        self.near_miss = float(self.get_parameter("near_miss_m").value)
        os.makedirs(self.out_dir, exist_ok=True)
        self.tasks_path = os.path.join(self.out_dir, "tasks.csv")
        self.run_path = os.path.join(self.out_dir, "run.csv")
        with open(self.tasks_path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(TASK_FIELDS)

        self.t0 = time.time()
        self.created: dict[int, dict] = {}
        self.rows: list[dict] = []
        self.robots: dict[int, RobotStatus] = {}
        self.moving: dict[int, float] = {}
        self.dist: dict[int, float] = {}
        self._last_xy: dict[int, tuple] = {}
        self._last_t: dict[int, float] = {}
        self.min_pair_dist = 1e9
        self.near_misses = 0
        self.min_gap = 1e9
        self.overlaps = 0
        self._in_near = set()
        # --- 重复泊位派单（第 40 节的假设，必须实测而不是推理） ---
        # 判据：两台车**同时**把 target_x/y 指向同一个点（容差 0.15 m）。
        # 工厂侧 _free_slot 已经用 reserved 保证一个工位只派一次，所以只要
        # 这里非零，就说明派单层真的把同一泊位给了两台车。
        self.dup_target_ticks = 0
        self.dup_target_events = 0
        self._dup_key: tuple | None = None
        self.dup_examples: list[str] = []
        self.traffic_rejected = 0
        self.traffic_expired = 0
        self.charging = 0
        self.reassign = 0
        self.first_assign: float | None = None

        self.create_subscription(TransportTask, "/factory/tasks", self.on_task, state_qos(50))
        self.create_subscription(TransportTask, "/factory/task_status", self.on_status, state_qos(50))
        self.create_subscription(Int32, "/factory/completed", self.on_done, state_qos(50))
        self.create_subscription(RobotStatus, "/fleet/robots", self.on_robot, state_qos(50))
        self.create_subscription(String, "/traffic/status", self.on_traffic, state_qos(5))
        self.create_subscription(String, "/factory/summary", self.on_summary, state_qos(5))

        self.create_timer(0.2, self.tick)
        self.create_timer(5.0, self.write_run)
        self.get_logger().info(
            f"metrics up · policy={self.policy} label={self.label} out={self.out_dir}"
        )

    # ---------- 回调 ----------
    def on_task(self, m: TransportTask):
        self.created[m.task_id] = dict(
            task_id=m.task_id, material=m.material_type, source=m.source_name,
            dest=m.dest_name, robot_id=-1, created_s=time.time() - self.t0,
            assigned_s="", delivered_s="", latency_s="",
            travel_cost="",
        )

    def on_status(self, m: TransportTask):
        if m.status == "failed":
            self.reassign += 1
        row = self.created.get(m.task_id)
        if row is not None and not row["assigned_s"]:
            row["assigned_s"] = round(time.time() - self.t0, 3)
            row["robot_id"] = m.robot_id
            if self.first_assign is None:
                self.first_assign = time.time() - self.t0

    def on_done(self, m: Int32):
        row = self.created.pop(m.data, None)
        if row is None:
            return
        now = time.time() - self.t0
        row["delivered_s"] = round(now, 3)
        row["latency_s"] = round(now - float(row["created_s"]), 3)
        self.rows.append(row)
        with open(self.tasks_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([row[k] for k in TASK_FIELDS])
            f.flush()

    def on_robot(self, m: RobotStatus):
        now = time.time()
        prev = self.robots.get(m.robot_id)
        if prev is not None and prev.state != "to_charger" and m.state == "to_charger":
            self.charging += 1
        if prev is not None:
            dt = now - self._last_t.get(m.robot_id, now)
            if m.state not in ("idle", "charging"):
                self.moving[m.robot_id] = self.moving.get(m.robot_id, 0.0) + dt
            last = self._last_xy.get(m.robot_id)
            if last is not None:
                self.dist[m.robot_id] = self.dist.get(m.robot_id, 0.0) + math.hypot(
                    m.x - last[0], m.y - last[1])
        self._last_xy[m.robot_id] = (m.x, m.y)
        self._last_t[m.robot_id] = now
        self.robots[m.robot_id] = m

    def on_traffic(self, m: String):
        for tok in m.data.replace(":", " ").split():
            pass
        try:
            parts = dict(kv.split("=") for kv in m.data.split() if "=" in kv and not kv.startswith("traffic:"))
            self.traffic_rejected = int(parts.get("rejected", self.traffic_rejected))
            self.traffic_expired = int(parts.get("expired", self.traffic_expired))
        except Exception:
            pass

    def on_summary(self, m: String):
        try:
            json.loads(m.data)
        except Exception:
            pass

    # ---------- 计算 ----------
    def tick(self):
        """统计车车安全裕度。

        判据必须与控制器一致：用**有向矩形之间的净距**，而不是车心距。
        车心距在密集车队里没有意义 —— 车是 Ø0.50 圆盘，车心距 0.50 m 就是刚好
        贴上，而更大的车心距并不代表安全（要看朝向下是否真的分开）。所以：
          * min_robot_gap_m：全队在整个运行期的最小轮廓净距，负值 = 真的重叠
          * overlap_events：轮廓实际相交的次数（这才是"碰撞"）
        """
        ids = list(self.robots)
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = self.robots[ids[i]], self.robots[ids[j]]
                d = math.hypot(a.x - b.x, a.y - b.y)
                if d < self.min_pair_dist:
                    self.min_pair_dist = d
                gap = circle_gap((a.x, a.y), (b.x, b.y))
                if gap < self.min_gap:
                    self.min_gap = gap
                key = (ids[i], ids[j])
                if gap < 0.0:
                    if key not in self._in_near:      # 只在开始重叠那一刻计一次
                        self._in_near.add(key)
                        self.overlaps += 1
                else:
                    self._in_near.discard(key)

        # 同泊位重复占用：按点分组，任何一组 >1 台车即记一次
        groups: dict[tuple, list[int]] = {}
        for rid, r in self.robots.items():
            if getattr(r, "task_id", -1) == -1:
                continue
            tx, ty = float(r.target_x), float(r.target_y)
            if tx == 0.0 and ty == 0.0:
                continue            # 无目标时控制器上报 0,0，不是真目标
            groups.setdefault((round(tx / 0.15), round(ty / 0.15)), []).append(rid)
        dup = tuple(sorted(tuple(sorted(v)) for v in groups.values() if len(v) > 1))
        if dup:
            self.dup_target_ticks += 1
            if dup != self._dup_key:
                self.dup_target_events += 1
                self._dup_key = dup
                self.get_logger().warn(f"duplicate berth target: {dup}")
        else:
            self._dup_key = None

    def snapshot(self) -> dict:
        lat = [float(r["latency_s"]) for r in self.rows if r["latency_s"] != ""]
        lat_sorted = sorted(lat)
        wall = time.time() - self.t0
        util = []
        for rid in self.robots:
            util.append(self.moving.get(rid, 0.0) / max(wall, 1e-6))
        makespan = None
        if self.first_assign is not None and self.rows:
            makespan = round(max(float(r["delivered_s"]) for r in self.rows) - self.first_assign, 3)
        p95 = lat_sorted[int(0.95 * (len(lat_sorted) - 1))] if lat_sorted else ""
        return dict(
            run_label=self.label, policy=self.policy, num_robots=len(self.robots),
            wall_s=round(wall, 1), makespan_s=makespan if makespan is not None else "",
            completed=len(self.rows),
            throughput_per_min=round(len(self.rows) / max(wall, 1e-6) * 60, 3),
            latency_mean_s=round(statistics.fmean(lat), 3) if lat else "",
            latency_p50_s=round(statistics.median(lat), 3) if lat else "",
            latency_p95_s=round(p95, 3) if p95 != "" else "",
            utilisation_mean=round(statistics.fmean(util), 3) if util else "",
            distance_total_m=round(sum(self.dist.values()), 2),
            traffic_rejected=self.traffic_rejected, traffic_expired=self.traffic_expired,
            min_robot_distance_m=round(self.min_pair_dist, 3) if self.min_pair_dist < 1e8 else "",
            near_miss_events=self.near_misses,
            min_robot_gap_m=round(self.min_gap, 3) if self.min_gap < 1e8 else "",
            overlap_events=self.overlaps,
            dup_target_ticks=self.dup_target_ticks,
            dup_target_events=self.dup_target_events,
            charging_events=self.charging, reassignments=self.reassign,
        )

    def write_run(self):
        snap = self.snapshot()
        with open(self.run_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=RUN_FIELDS)
            w.writeheader()
            w.writerow(snap)
        self.get_logger().info(
            f"metrics: {snap['completed']} done · makespan={snap['makespan_s']}s · "
            f"throughput={snap['throughput_per_min']}/min · min_gap={snap['min_robot_gap_m']}m · "
            f"overlaps={snap['overlap_events']} · dup={snap['dup_target_events']}"
        )


def main():
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = MetricsRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.write_run()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:                                     # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
