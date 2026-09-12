#!/usr/bin/env python3
"""把 Gazebo 世界里的抓图机位存成 PNG。

前置：Gazebo 已在跑，且 /view_*/image 已桥接到 ROS 2（factory.launch.py 默认会桥接）。

    python3 tools/capture_views.py /tmp/shots /view_iso/image /view_top/image
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# 相机流用 best-effort + 深度 1：只关心"最新一帧"，
# 用 reliable/深度10 会让桥接端的图像缓冲无限增长（实测涨到 5GB 触发 OOM）。
CAM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class Capture(Node):
    def __init__(self, outdir, topics):
        super().__init__("capture_views")
        self.outdir, self.topics, self.got = outdir, topics, {}
        os.makedirs(outdir, exist_ok=True)
        for t in topics:
            self.create_subscription(Image, t, lambda m, t=t: self.cb(m, t), CAM_QOS)

    def cb(self, m, t):
        if t in self.got:
            return
        ch = 4 if m.encoding in ("rgba8", "bgra8") else 3
        stride = m.step if m.step else m.width * ch
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, stride // ch, ch)[:, :, :3]
        if m.encoding in ("rgb8", "rgba8"):
            img = img[:, :, ::-1]
        name = t.strip("/").replace("/", "_") + ".png"
        cv2.imwrite(os.path.join(self.outdir, name), img)
        self.got[t] = name
        print(f"saved {name}  {img.shape[1]}x{img.shape[0]}", flush=True)


def main():
    outdir, topics = sys.argv[1], sys.argv[2:]
    rclpy.init()
    node = Capture(outdir, topics)
    t0 = time.time()
    while rclpy.ok() and len(node.got) < len(topics) and time.time() - t0 < 40:
        rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()
    print(f"captured {len(node.got)}/{len(topics)}")


if __name__ == "__main__":
    main()
