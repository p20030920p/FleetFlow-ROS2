#!/usr/bin/env python3
"""工厂终态采样：把"跑着跑着停了"那一刻的现场完整拍下来。

之前只能靠任务日志反推（谁到了哪、哪条任务没派出去），
反复猜错。这里直接订阅工厂自己发布的状态，抓最后一帧：

  /factory/summary   -> materials/completed/in_flight/reclaimed/waiting/
                        finished/machine/storage/room
  /factory/machines  -> 每台机器 holding（= 是否卡着成品）、input/output、
                        busy_ratio
  /fleet/robots      -> 每台车的 state / task_id / target / 到目标的距离

用法: python3 tools/factory_state.py [采样秒数]
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fleetflow_interfaces.msg import MachineState, RobotStatus

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=50)


class Probe(Node):
    def __init__(self):
        super().__init__("factory_state_probe")
        self.summary = {}
        self.machines = {}
        self.robots = {}
        self.create_subscription(String, "/factory/summary", self._on_sum, Q)
        self.create_subscription(MachineState, "/factory/machines", self._on_m, Q)
        self.create_subscription(RobotStatus, "/fleet/robots", self._on_r, Q)

    def _on_sum(self, m):
        try:
            self.summary = json.loads(m.data)
        except Exception:                                    # noqa: BLE001
            self.summary = {"raw": m.data}

    def _on_m(self, m):
        self.machines[m.name] = m

    def _on_r(self, m):
        self.robots[m.robot_id] = m

    def dump(self):
        s = self.summary
        print("--- 工厂总账 ---")
        for k in ("materials", "completed", "tasks_created", "pending", "running",
                  "done", "in_flight", "reclaimed", "waiting", "finished",
                  "machine", "storage"):
            if k in s:
                print(f"  {k:14s} {s[k]}")
        if "room" in s:
            print(f"  {'下游可收':14s} {s['room']}")
        lost = s.get("materials", 0) - s.get("completed", 0)
        if "waiting" in s:
            print(f"  {'仍在管线内':14s} {lost}  (= materials - completed)")

        print("--- 机器（holding 非空 = 成品没被拉走） ---")
        for name in sorted(self.machines):
            m = self.machines[name]
            print(f"  {name:12s} state={m.state:8s} in={m.input_count} "
                  f"out={m.output_count} busy={m.busy_ratio:.2f} "
                  f"wait={m.waiting_station} fin={m.finished_station}")

        print("--- 车辆 ---")
        for rid in sorted(self.robots):
            r = self.robots[rid]
            d = math.hypot(r.x - r.target_x, r.y - r.target_y)
            print(f"  R{rid} {r.state:14s} task={r.task_id:4d} "
                  f"pos=({r.x:5.2f},{r.y:5.2f}) tgt=({r.target_x:5.2f},{r.target_y:5.2f}) "
                  f"d={d:4.2f} spd={r.speed:4.2f}")


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.dump()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
