"""厂房布局：一座小型纺纱车间的真实几何。

与早期版本的区别
----------------
* 机器不再是"半径 0.3 的圆柱"，而是按真实纺纱设备尺度的**矩形占地**：
  梳棉机 3.5 × 1.2 m，并条机 2.5 × 0.9 m，粗纱机 3.0 × 1.4 m。
* 搬运单元是纺纱厂真实的**条筒（sliver can）**，直径 0.45 m、高 0.95 m，
  用颜色标记所处工序（空筒 → 生条 → 熟条 → 粗纱）。
* 储料区是**货架**，取放位在货架前的通道上，而不是料区中心的一个点。
* 工序沿用"储料 → 梳棉 → 并条 → 粗纱 → 成品"的总体格局。

坐标系：厂房内净尺寸 26 × 16 m，原点在左下角，x 向右、y 向上（俯视）。
"""
from __future__ import annotations

# ---------------------------------------------------------------- 厂房
BUILDING = dict(w=26.0, h=16.0, wall_h=6.0)
FIELD = dict(x_min=0.85, x_max=25.15, y_min=0.85, y_max=15.15)

# ---------------------------------------------------------------- 机器
MACHINE_SPEC = {
    "carding": dict(len=3.50, wid=1.20, h=2.00, rgb=(0.32, 0.45, 0.40)),   # 梳棉机
    "drawing": dict(len=2.50, wid=0.90, h=1.60, rgb=(0.26, 0.36, 0.52)),   # 并条机
    "roving":  dict(len=3.00, wid=1.40, h=2.10, rgb=(0.42, 0.32, 0.50)),   # 粗纱机
}

STAGES = [
    dict(name="carding", cn="梳棉", lanes=[3.0, 6.0, 9.0, 12.0],
         machine_x=7.00, wait_x=5.55, done_x=11.55, process_s=6.0),
    dict(name="drawing", cn="并条", lanes=[4.5, 10.5],
         machine_x=13.60, wait_x=12.75, done_x=17.20, process_s=7.5),
    dict(name="roving", cn="粗纱", lanes=[4.5, 10.5],
         machine_x=18.70, wait_x=17.85, done_x=22.30, process_s=9.0),
]

# 储料区：货架贴墙，取放位在货架前的通道
RACK = dict(w=1.10, h=2.40, depth=1.90)
# 只有"空筒库"与"粗纱成品库"是任务的起终点；中间工序的缓存就是各自的完工位。
STORAGE = {
    "empty": dict(x=1.55, y=8.0, cn="空筒库", rgb=(0.60, 0.64, 0.68)),
    "red":   dict(x=24.45, y=8.0, cn="粗纱成品库", rgb=(0.80, 0.26, 0.28)),
}

# 取放位：空筒库与梳棉通道同高，成品库与粗纱通道同高
STORAGE_SLOTS = {
    "empty": dict(x=3.40, ys=[3.0, 6.0, 9.0, 12.0]),
    "red":   dict(x=22.95, ys=[5.5, 8.0, 10.5]),
}

# 充电位：机柜贴墙，AGV 停在机柜前方的停靠点
CHARGER_CABINET_DY = -0.85          # 机柜相对停靠点的偏移（贴向下墙）
CHARGERS = {
    "charger_0": dict(x=4.30, y=1.55, rgb=(0.10, 0.60, 0.85)),
    "charger_1": dict(x=6.60, y=1.55, rgb=(0.10, 0.60, 0.85)),
}

# AGV 待命区（贴下墙一字排开，避开所有取放位）
PARK = dict(y=1.55, x0=9.60, dx=1.35, n=8)

FLEET_COLORS = [
    (0.13, 0.42, 0.78), (0.90, 0.45, 0.10), (0.16, 0.62, 0.42), (0.72, 0.20, 0.32),
    (0.45, 0.28, 0.72), (0.10, 0.60, 0.65), (0.85, 0.55, 0.12), (0.30, 0.32, 0.38),
]

# 条筒（搬运单元）
CAN = dict(radius=0.225, height=0.95)
CAN_RGB = {
    "empty": (0.62, 0.66, 0.70),
    "green": (0.22, 0.68, 0.38),
    "yellow": (0.92, 0.74, 0.22),
    "red": (0.80, 0.26, 0.28),
}
MATERIAL_FLOW = ["empty", "green", "yellow", "red"]

# 建筑结构
COLUMN_PITCH = 6.50
COLUMN_SIZE = 0.42
TRUSS_PITCH = 6.50

# 取放位名字 -> 坐标（调度与交通管理共用）
STORAGE_SLOT_POINTS = {
    f"storage_{zone}_slot_{i}": dict(name=f"storage_{zone}_slot_{i}", x=spec["x"], y=y)
    for zone, spec in STORAGE_SLOTS.items()
    for i, y in enumerate(spec["ys"])
}


def machine_rect(stage: str, lane: int):
    """某台机器的占地矩形 (x0, y0, x1, y1)。"""
    for s in STAGES:
        if s["name"] == stage:
            spec = MACHINE_SPEC[stage]
            y = s["lanes"][lane]
            return (s["machine_x"], y - spec["wid"] / 2,
                    s["machine_x"] + spec["len"], y + spec["wid"] / 2)
    raise KeyError(stage)


def machine_name(stage: str, lane: int) -> str:
    return f"{stage}_machine_{lane}"


def all_machines():
    """[(名字, 矩形, 高度, 配色, 工序, 通道), ...]"""
    out = []
    for s in STAGES:
        spec = MACHINE_SPEC[s["name"]]
        for lane in range(len(s["lanes"])):
            out.append((machine_name(s["name"], lane), machine_rect(s["name"], lane),
                        spec["h"], spec["rgb"], s["name"], lane))
    return out


def station(stage: str, kind: str, lane: int) -> dict:
    for s in STAGES:
        if s["name"] != stage:
            continue
        x = {"waiting": s["wait_x"], "machine": s["machine_x"], "finished": s["done_x"]}[kind]
        return dict(name=f"{stage}_{kind}_{lane}", x=x, y=s["lanes"][lane],
                    stage=stage, lane=lane, kind=kind)
    raise KeyError(stage)


def stage_of(material: str) -> int:
    return {"empty": 0, "green": 0, "yellow": 1, "red": 2}.get(material, 3)


RACK_LEN = 10.5          # 货架长度，贴左右墙、留出柱距


def rack_rects():
    """货架占地矩形：南北向长条货架，贴左右墙。"""
    out = {}
    for name, z in STORAGE.items():
        out[name] = (z["x"] - RACK["depth"] / 2, z["y"] - RACK_LEN / 2,
                     z["x"] + RACK["depth"] / 2, z["y"] + RACK_LEN / 2)
    return out


def all_station_points() -> dict:
    pts = {k: (v["x"], v["y"]) for k, v in STORAGE_SLOT_POINTS.items()}
    for s in STAGES:
        for i in range(len(s["lanes"])):
            for kind in ("waiting", "finished"):
                st = station(s["name"], kind, i)
                pts[st["name"]] = (st["x"], st["y"])
    pts.update({k: (v["x"], v["y"]) for k, v in CHARGERS.items()})
    return pts


def static_boxes():
    """A* 用的静态障碍矩形 [(x0, y0, x1, y1), ...]。"""
    boxes = [m[1] for m in all_machines()]
    boxes += list(rack_rects().values())
    for c in CHARGERS.values():
        cy = c["y"] + CHARGER_CABINET_DY
        boxes.append((c["x"] - 0.34, cy - 0.26, c["x"] + 0.34, cy + 0.26))
    return boxes


def park_poses(n: int):
    """待命区位姿 [(x, y, yaw), ...]。"""
    out = []
    for i in range(n):
        row, col = divmod(i, PARK["n"])
        out.append((PARK["x0"] + PARK["dx"] * col, PARK["y"] + 1.15 * row, 0.0))
    return out
