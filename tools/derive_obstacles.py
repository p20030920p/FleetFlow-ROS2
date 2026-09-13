#!/usr/bin/env python3
"""从 SDF 世界提取"规划器必须避开的障碍"，写进 layout 用的 JSON。

为什么需要这个
--------------
`layout.static_boxes()` 是**手写**的机台/货架/充电柜矩形。而
`tools/build_world.py` 给每台机器额外加了一堆带 collision 的零件
（并条机后部条筒架、粗纱机端板、梳棉机电机与黄色警示垫……）。
两边的清单从来没有对齐过，后果是：

  * A* 认为某条路合法（它的膨胀障碍里没有这些零件），
  * 车照着走，
  * LiDAR 却在半路撞见这些"规划器不知道的"物体，于是停下、重规划、
    再走同一条路 —— 表现为"路径合法但走不通"的卡顿。

手写清单没法保持同步，所以改成从世界的**真实碰撞体**生成。
生成物 `obstacles_world.json` 由 `static_boxes()` 读入并与手写清单合并。

用法：
    python3 tools/derive_obstacles.py            # 写入 package 的 config/
"""
from __future__ import annotations

import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _find_paths(start: str):
    """返回可能让 `import fleetflow_sim` 生效的目录列表（优先源码目录）。

    覆盖两种布局，且做**两层**下探（本仓库是 <repo>/src/fleetflow_sim/fleetflow_sim/）：
      <repo>/fleetflow_sim/
      <repo>/src/fleetflow_sim/
    不写死层级，因为 source 过 ROS 环境后 PYTHONPATH 会被改过。
    """
    d = start
    for _ in range(6):
        for rel in ("src/fleetflow_sim", "fleetflow_sim", "src"):
            cand = os.path.join(d, rel)
            if os.path.isfile(os.path.join(cand, "fleetflow_sim", "layout.py")):
                return cand
        d = os.path.dirname(d)
    raise SystemExit("找不到 fleetflow_sim 包目录（应从仓库内运行）")


PKG = _find_paths(os.path.dirname(HERE))          # 含 fleetflow_sim/ 的目录
sys.path.insert(0, PKG)

from fleetflow_sim import layout  # noqa: E402

SDF = os.path.join(PKG, "worlds", "textile_factory.sdf")
OUT = os.path.join(PKG, "config", "obstacles_world.json")

# 只收"真的会挡住车"的零件：
#   底面低于这个高度 -> 车过不去（贴地标线不算，它们没有 collision，本来就不会进来）
MIN_TOP = 0.12
#   顶面高于这个高度 -> 车能压过去，不算障碍
MIN_BOTTOM_CLEAR = 0.30


def parse_models(sdf_text: str):
    """返回 [(name, x, y, z, sx, sy, sz)]，只含有 collision 的 model。"""
    out = []
    for mm in re.finditer(r'<model name="([^"]+)">(.*?)</model>', sdf_text, re.S):
        name, body = mm.group(1), mm.group(2)
        mp = re.search(r"<pose>([^<]*)</pose>", body)
        if not mp:
            continue
        try:
            pose = [float(v) for v in mp.group(1).split()]
        except ValueError:
            continue
        if len(pose) < 3:
            continue
        for cm in re.finditer(r"<collision[^>]*>(.*?)</collision>", body, re.S):
            cb = cm.group(1)
            sz = re.search(r"<box>\s*<size>([^<]*)</size>", cb)
            if sz:
                try:
                    d = [float(v) for v in sz.group(1).split()]
                except ValueError:
                    continue
            else:
                cy = re.search(
                    r"<cylinder>\s*<radius>([^<]*)</radius>\s*<length>([^<]*)</length>",
                    cb)
                if not cy:
                    continue
                r_, l_ = float(cy.group(1)), float(cy.group(2))
                d = [2 * r_, 2 * r_, l_]
            if len(d) < 3:
                continue
            out.append((name, pose[0], pose[1], pose[2], d[0], d[1], d[2]))
    return out


def main() -> int:
    sdf_text = open(SDF).read()
    models = parse_models(sdf_text)

    # 规划器已知的矩形，各自按车体半径膨胀后就是"A* 认为不能走"的区域
    pad = layout.STATIC_INFLATE
    # 只比**手写**清单：static_boxes() 已经并入了本脚本上一轮的输出，
    # 拿它做比较会让生成结果不幂等（第二次跑会得到 0 个）。
    known = [(x0 - pad, y0 - pad, x1 + pad, y1 + pad)
             for x0, y0, x1, y1 in layout.base_static_boxes()]

    def fully_known(x0, y0, x1, y1, tol=1e-6):
        """这个碰撞体是否已被某个已知障碍（含膨胀量）完全覆盖。

        完全覆盖就不用重复登记 —— 规划器本来就不会往那儿走。
        """
        for kx0, ky0, kx1, ky1 in known:
            if kx0 - tol <= x0 and y0 >= ky0 - tol and x1 <= kx1 + tol and y1 <= ky1 + tol:
                return True
        return False

    extra, skipped = [], 0
    for name, x, y, z, sx, sy, sz in models:
        hx, hy, hz = sx / 2, sy / 2, sz / 2
        x0, y0, x1, y1 = x - hx, y - hy, x + hx, y + hy
        bottom, top = z - hz, z + hz
        if top < MIN_TOP or bottom > MIN_BOTTOM_CLEAR:
            continue
        # 完全在厂房外
        f = layout.FIELD
        if x1 < f["x_min"] or x0 > f["x_max"] or y1 < f["y_min"] or y0 > f["y_max"]:
            continue
        if fully_known(x0, y0, x1, y1):
            skipped += 1
            continue
        extra.append(dict(name=name, rect=[round(x0, 3), round(y0, 3),
                                           round(x1, 3), round(y1, 3)],
                          height=round(top, 3)))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump(dict(source=os.path.basename(SDF),
                       note="由 tools/derive_obstacles.py 从世界碰撞体生成，勿手改",
                       inflate=layout.STATIC_INFLATE,
                       obstacles=extra), f, ensure_ascii=False, indent=1)

    print(f"世界里的碰撞体 {len(models)} 个")
    print(f"  已被手写清单（含 {layout.STATIC_INFLATE} m 膨胀）覆盖，跳过 {skipped} 个")
    print(f"  额外登记 {len(extra)} 个 -> {OUT}")
    print()
    for e in extra[:20]:
        print(f"  {e['name'][:34]:34s} rect={e['rect']} h={e['height']}")
    if len(extra) > 20:
        print(f"  ... 其余 {len(extra)-20} 个见 JSON")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
