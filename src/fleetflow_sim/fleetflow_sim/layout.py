"""厂房布局：与 worlds/textile_factory.sdf 里的坐标一一对应。

产线是四段：
    empty 储料区 → 梳棉(carding) → drawing1 → drawing2 → 成品区/红料区
每段有三类工位：waiting（等料）/ machine（机器本体）/ finished（完工缓存）。
AGV 只在 waiting 与 finished 之间搬运，从不驶入 machine 圆柱。
"""
import math

# 可行驶范围（墙内侧留出车体半径）
FIELD = dict(x_min=0.55, x_max=15.45, y_min=0.75, y_max=9.25)

# 大圆盘料区
STORAGE = {
    "empty":     dict(x=2.0,  y=2.0, r=0.90, rgb=(0.62, 0.66, 0.70), label="Empty storage"),
    "green":     dict(x=2.0,  y=8.0, r=0.90, rgb=(0.20, 0.72, 0.36), label="Green buffer"),
    "yellow":    dict(x=14.0, y=2.0, r=0.90, rgb=(0.95, 0.76, 0.20), label="Yellow buffer"),
    "red":       dict(x=14.0, y=8.0, r=0.90, rgb=(0.85, 0.24, 0.28), label="Red buffer"),
    "completed": dict(x=14.0, y=5.0, r=0.80, rgb=(0.98, 0.60, 0.12), label="Completed"),
}

# 三段加工工序：lanes 是并行通道的 y 坐标
STAGES = [
    dict(name="carding",  lanes=[1.5, 3.5, 5.5, 7.5], wait_x=5.0,  machine_x=6.0,  done_x=7.0,
         rgb=(0.16, 0.62, 0.42), process_s=6.0),
    dict(name="drawing1", lanes=[2.5, 6.5],           wait_x=8.0,  machine_x=9.0,  done_x=10.0,
         rgb=(0.16, 0.45, 0.78), process_s=7.5),
    dict(name="drawing2", lanes=[2.5, 6.5],           wait_x=10.8, machine_x=11.8, done_x=12.8,
         rgb=(0.45, 0.28, 0.72), process_s=9.0),
]

# 取货/卸货停靠位：一个料区只有一个几何中心，但真实车队不会让所有车挤同一个点，
# 而是在料区周围划出若干停靠位（dock slot）。这里按圆周均匀分布生成。
# 停靠位布局。空桶料区在厂房角落，整圈放不下，所以沿"朝向产线"的弧线展开；
# 红色料区在大致空阔的右上方，用整圈即可。间距必须足够大（≈1.3 m）：
# 间距太小会让多台车在泊位区互相堵死，这是实测踩过的坑。
STORAGE_SLOTS = {
    "empty": dict(radius=1.70, angles=(-20, 25, 70, 115)),
    "red":   dict(radius=1.35, angles=(45, 135, 225, 315)),
}


def _slots(zone: str):
    z = STORAGE[zone]
    spec = STORAGE_SLOTS[zone]
    if spec.get("angles"):
        angles = spec["angles"]
    else:
        n = spec.get("count", 4)
        angles = tuple(360.0 * i / n for i in range(n))
    out = {}
    for i, deg in enumerate(angles):
        ang = math.radians(deg)
        out[f"storage_{zone}_slot_{i}"] = dict(
            name=f"storage_{zone}_slot_{i}",
            x=z["x"] + spec["radius"] * math.cos(ang),
            y=z["y"] + spec["radius"] * math.sin(ang),
        )
    return out


STORAGE_SLOT_POINTS = {}
for _z in STORAGE_SLOTS:
    STORAGE_SLOT_POINTS.update(_slots(_z))


# 充电桩：电量低于阈值时车辆自动回冲（真实 AGV 车队的标配行为）
CHARGERS = {
    "charger_0": dict(x=0.9, y=5.0, r=0.45, rgb=(0.10, 0.60, 0.85)),
    "charger_1": dict(x=15.1, y=5.0, r=0.45, rgb=(0.10, 0.60, 0.85)),
}

# 物料随工序变色：空桶 → 绿 → 黄 → 红（与旧版控制中心图例一致）
MATERIAL_FLOW = ["empty", "green", "yellow", "red"]

# AGV 车队配色（截图里能一眼分辨）
FLEET_COLORS = [
    (0.13, 0.42, 0.78), (0.90, 0.45, 0.10), (0.16, 0.62, 0.42), (0.72, 0.20, 0.32),
    (0.45, 0.28, 0.72), (0.10, 0.60, 0.65), (0.85, 0.55, 0.12), (0.30, 0.32, 0.38),
    (0.62, 0.30, 0.60), (0.20, 0.50, 0.30),
]


def station(stage: str, kind: str, lane: int) -> dict:
    """返回某个工位的名字与坐标。kind ∈ {waiting, machine, finished}。"""
    for s in STAGES:
        if s["name"] != stage:
            continue
        y = s["lanes"][lane]
        x = {"waiting": s["wait_x"], "machine": s["machine_x"], "finished": s["done_x"]}[kind]
        return dict(name=f"{stage}_{kind}_{lane}", x=x, y=y, stage=stage, lane=lane, kind=kind)
    raise KeyError(stage)


def stage_of(material: str) -> int:
    """物料处于第几段工序：empty/绿=0（待梳棉），黄=1，红=2，完工=3。"""
    return {"empty": 0, "green": 0, "yellow": 1, "red": 2}.get(material, 3)


def machine_centers() -> list:
    """机器圆柱中心，用作 A* 的静态障碍。"""
    out = []
    for s in STAGES:
        for y in s["lanes"]:
            out.append((s["machine_x"], y, 0.30))
    return out


def all_station_points() -> dict:
    """所有可停靠点的字典 {名字: (x, y)}。"""
    pts = {f"storage_{k}": (v["x"], v["y"]) for k, v in STORAGE.items()}
    pts.update({k: (v["x"], v["y"]) for k, v in CHARGERS.items()})
    pts.update({k: (v["x"], v["y"]) for k, v in STORAGE_SLOT_POINTS.items()})
    for s in STAGES:
        for i, _ in enumerate(s["lanes"]):
            for kind in ("waiting", "finished"):
                st = station(s["name"], kind, i)
                pts[st["name"]] = (st["x"], st["y"])
    return pts
