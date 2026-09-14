#!/usr/bin/env python3
"""停摆现场取证：车不动的时候，到底是谁按着它？

终态采样（tools/factory_state.py）能回答"车在哪、离目标多远"，
但分不清下面两种完全不同的原因：

  A. **上层在按着它** —— 协调层 hold / 控制器 _hold 状态机，指令就是 0；
  B. **下层没执行** —— 上层一直在下发非零 cmd_vel，可车就是不动
     （物理、桥接、时钟、订阅者丢失……）。

这两种原因的修法毫无交集，所以必须把**指令**和**实际位姿**同时采样。
本脚本按固定周期打印一行时间线，一眼就能看出是 A 还是 B：

  t=12.3 R0 st=to_pickup v_cmd=0.00 w=0.00 spd=0.00 pos=(..) tgt=(..)
       ^ 指令 0        ^ 实际 0        -> 上层按着（A）
  t=13.3 R0 st=to_pickup v_cmd=0.60 w=0.00 spd=0.00 pos=(..) tgt=(..)
       ^ 指令 0.60     ^ 实际 0        -> 下层没动（B）

用法: python3 tools/stall_trace.py [秒数] [采样间隔]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

from fleetflow_interfaces.msg import RobotStatus, TrafficDirective

Q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST, depth=50)

N_ROBOTS = 4


class Trace(Node):
    def __init__(self):
        super().__init__("stall_trace")
        self.status: dict[int, RobotStatus] = {}
        self.cmd: dict[int, Twist] = {}
        self.dir: dict[int, TrafficDirective] = {}
        self.cmd_count = {i: 0 for i in range(N_ROBOTS)}
        self.cmd_last = {i: 0.0 for i in range(N_ROBOTS)}
        for i in range(N_ROBOTS):
            self.create_subscription(RobotStatus, "/fleet/robots", self._st, Q)
            self.create_subscription(Twist, f"/robot_{i}/cmd_vel",
                                     self._mk_cmd(i), Q)
            self.create_subscription(TrafficDirective, f"/robot_{i}/traffic",
                                     self._mk_dir(i), Q)

    def _st(self, m):
        self.status[m.robot_id] = m

    def _mk_cmd(self, i):
        def cb(m):
            self.cmd[i] = m
            self.cmd_count[i] += 1
            self.cmd_last[i] = time.time()
        return cb

    def _mk_dir(self, i):
        def cb(m):
            self.dir[i] = m
        return cb

    def line(self, elapsed: float):
        out = [f"t={elapsed:6.1f}"]
        for i in sorted(self.status):
            s = self.status[i]
            c = self.cmd.get(i)
            v = c.linear.x if c else float("nan")
            w = c.angular.z if c else float("nan")
            d = self.dir.get(i)
            flags = ""
            if d is not None:
                if d.hold:
                    flags += "HOLD "
                if getattr(d, "has_escape", False):
                    flags += "ESC "
                if getattr(d, "replan", False):
                    flags += "REPLAN "
            age = time.time() - self.cmd_last[i]
            out.append(
                f"| R{i} {s.state[:11]:11s} v_cmd={v:5.2f} w={w:5.2f} "
                f"spd={s.speed:5.2f} d_tgt={math.hypot(s.x-s.target_x, s.y-s.target_y):4.2f} "
                f"cmd_age={age:4.1f}s {flags}")
        print(" ".join(out), flush=True)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    n = Trace()
    t0 = time.time()
    nxt = t0
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() >= nxt:
            nxt += period
            n.line(time.time() - t0)
    print("cmd_vel 收到条数:", {i: n.cmd_count[i] for i in sorted(n.cmd_count)})
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
