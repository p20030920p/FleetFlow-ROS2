#!/usr/bin/env python3
"""把一次运行的全过程录成 JSONL，供 ``tools/render_demo.py`` 离线渲染成动图。

    # 终端 1：跑逻辑栈（不启 Gazebo，墙钟准确，几十秒就能跑完一个完整循环）
    ros2 launch fleetflow_sim logic_only.launch.py num_robots:=6 policy:=ca_ssi \
        num_materials:=40 max_tasks_in_flight:=10

    # 终端 2：同步录制
    python3 tools/record_run.py /tmp/demo.jsonl --fps 10 --seconds 90

为什么先录后渲：Gazebo 的实时因子远低于 1，直接录屏只能得到慢动作；
而调度与交通逻辑在 ``logic_only`` 下与仿真模式完全一致，
所以"录状态、离屏渲染"既快又能画得干净。

每行是一条快照：``{"t":…, "robots":[…], "tasks":[…], "machines":[…], "summary":{…}}``。
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from fleetflow_interfaces.msg import MachineState, RobotStatus, TransportTask

STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=50)

# /clock 由 ros_gz_bridge 以 best-effort 发出；reliable 订阅者与它不兼容，收不到任何消息。
# best-effort 订阅对两种发布端都能工作，所以时钟单独用这一套。
CLOCK_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10)


class Recorder(Node):
    def __init__(self, path: str, fps: float, seconds: float, min_robots: int = 0,
                 max_robots: int = 16):
        super().__init__("run_recorder")
        self.fh = open(path, "w", encoding="utf-8")
        self.fps = fps
        self.min_robots = min_robots
        self.seconds = seconds
        self.t0 = None          # 车队集齐之后才开始计时，首帧不会只有一台车
        self.sim0 = None
        self.t_end = None
        self.robots: dict[int, RobotStatus] = {}
        self.tasks: dict[int, dict] = {}
        self.machines: dict[str, MachineState] = {}
        self.paths: dict[int, list] = {}       # robot_id -> 剩余规划路径
        self.summary: dict = {}
        self.sim_t: float | None = None
        self.n = 0

        self.create_subscription(RobotStatus, "/fleet/robots", self.on_robot, STATE_QOS)
        self.create_subscription(TransportTask, "/factory/tasks", self.on_task, STATE_QOS)
        self.create_subscription(TransportTask, "/factory/task_status", self.on_status,
                                 STATE_QOS)
        self.create_subscription(MachineState, "/factory/machines", self.on_machine,
                                 STATE_QOS)
        self.create_subscription(String, "/factory/summary", self.on_summary, STATE_QOS)
        # 仿真时钟：Gazebo 模式下实时因子远小于 1，回放必须按仿真时间而不是墙钟，
        # 否则"完整过程"会被压缩成慢动作或者干脆录不到几步。
        self.create_subscription(Clock, "/clock", self.on_clock, CLOCK_QOS)
        # 每台车的剩余规划路径：动画要画"真正会走的路线"，直线画法看起来像穿墙
        for rid in range(max_robots):
            self.create_subscription(Path, f"/robot_{rid}/path",
                                     lambda m, r=rid: self.on_path(m, r), STATE_QOS)
        self.create_timer(1.0 / fps, self.snap)
        self.get_logger().info(f"recording -> {path} @ {fps:g} Hz")

    # ---------- 输入 ----------
    def on_robot(self, m: RobotStatus):
        self.robots[m.robot_id] = m

    def on_task(self, m: TransportTask):
        self.tasks.setdefault(m.task_id, {
            "id": m.task_id, "material": m.material_type,
            "src": m.source_name, "dst": m.dest_name,
            "sx": round(m.source_x, 3), "sy": round(m.source_y, 3),
            "dx": round(m.dest_x, 3), "dy": round(m.dest_y, 3),
            "status": m.status, "robot": -1,
            "t_created": round(time.time() - (self.t0 or time.time()), 2),
        })

    def on_status(self, m: TransportTask):
        t = self.tasks.get(m.task_id)
        if t is None:
            self.on_task(m)
            t = self.tasks[m.task_id]
        t["status"] = m.status
        # 记下"被派发"的时刻，渲染器才能重建任意时刻的任务状态
        # （否则 JSONL 里只有最终状态，动画会一上来就全是已完成）
        if m.status == "assigned" and "t_assigned" not in t:
            t["t_assigned"] = round(time.time() - (self.t0 or time.time()), 2)
            t["robot"] = int(m.robot_id)
        if m.status in ("done", "failed"):
            t.setdefault("t_assigned", round(time.time() - (self.t0 or time.time()), 2))
            t["t_done"] = round(time.time() - (self.t0 or time.time()), 2)
            t["robot"] = int(m.robot_id)

    def on_machine(self, m: MachineState):
        self.machines[m.name] = m

    def on_clock(self, m: Clock):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def on_path(self, m: Path, rid: int):
        self.paths[rid] = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
                           for p in m.poses]

    def on_summary(self, m: String):
        try:
            self.summary = json.loads(m.data)
        except (TypeError, ValueError):
            pass

    # ---------- 落盘 ----------
    def snap(self):
        now = time.time()
        if self.t0 is None:
            if len(self.robots) < self.min_robots:
                return
            self.t0 = now
            self.sim0 = self.sim_t
            self.t_end = None if self.seconds <= 0 else now + self.seconds
            self.get_logger().info(f"fleet ready ({len(self.robots)} robots) - recording starts")
            return
        if self.t_end is not None and now >= self.t_end:
            self.fh.close()
            self.get_logger().info(f"recorded {self.n} frames -> done")
            raise SystemExit(0)
        rec = {
            "t": round(now - self.t0, 3),
            "sim": None if self.sim_t is None else round(self.sim_t - (self.sim0 or self.sim_t), 3),
            "robots": [[r.robot_id, round(r.x, 3), round(r.y, 3), round(r.yaw, 3),
                        r.state, int(r.task_id), round(float(r.battery), 1)]
                       for r in sorted(self.robots.values(), key=lambda z: z.robot_id)],
            "tasks": list(self.tasks.values()),
            "machines": [[m.name, m.stage, m.state, int(m.input_count),
                          int(m.output_count), round(float(m.busy_ratio), 3)]
                         for m in self.machines.values()],
            "paths": {str(k): v for k, v in self.paths.items() if v},
            "summary": self.summary,
        }
        self.fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.fh.flush()
        self.n += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--max-robots", type=int, default=16,
                    help="订阅 /robot_0..N/path 的上限")
    ap.add_argument("--min-robots", type=int, default=0,
                    help="集齐这么多台车之后才开始计时（首帧才有完整车队）")
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.out, args.fps, args.seconds, args.min_robots,
                    args.max_robots)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.fh.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("recorder stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
