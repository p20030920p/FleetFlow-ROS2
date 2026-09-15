"""厂房布局：一座小型纺纱车间的真实几何。

与早期版本的区别
----------------
* 机器不再是"半径 0.3 的圆柱"，而是按真实纺纱设备尺度的**矩形占地**：
  梳棉机 3.5 × 1.2 m，并条机 2.5 × 0.9 m，粗纱机 3.0 × 1.4 m。
* 搬运单元是纺纱厂真实的**条筒（sliver can）**，直径 0.45 m、高 0.95 m，
  用颜色标记所处工序（空筒 → 生条 → 熟条 → 粗纱）。
* 储料区是**货架**，取放位在货架前的通道上，而不是料区中心的一个点。
* 工序沿用"储料 → 梳棉 → 并条 → 粗纱 → 成品"的总体格局。

坐标系：厂房内净尺寸 **120 × 60 m**，原点在左下角，x 向右、y 向上（俯视）。

尺度基准（为什么是这些数字）
--------------------------
* **车**：50 cm 圆盘差速底盘（`AGV = dict(dia=0.50)`），工业上常见规格。
  含安全边距后，两车交会需要约 0.50 + 2×0.15 = **0.80 m** 净距。
* **通道**：留 **3.0 m**，是交会需求的 3.75 倍 —— 一台车停靠时另一台可以正常绕过，
  不必倒车。早期版本通道只有 0.65 m（比车还窄），必然堵死。
* **工位间距**：同排 **3.0 m**（6 个工位铺开 15 m），车停靠时不侵占邻位。
* **厂区**：120 × 60 m = 7200 m²，接近一座真实纺纱车间的单跨厂房。
"""
from __future__ import annotations

# ---------------------------------------------------------------- 厂房
# 厂房外形尺寸。**w 必须等于 world_bounds 的宽度** —— 渲染器用它做
# 世界→像素的缩放，写死就会在改布局后把货架画到图外（第 64 节）。
BUILDING = dict(w=120.0, h=60.0, wall_h=7.0)
FIELD = dict(x_min=2.0, x_max=118.0, y_min=2.0, y_max=58.0)

# ---------------------------------------------------------------- 车
# 50 cm 圆盘差速底盘。圆车的好处：任意朝向的占地都相同，避让判据不必分朝向，
# 这也是工业 AGV 多用圆车的原因（松灵/宇树这类底盘的公开规格即此量级）。
AGV = dict(dia=0.50, height=0.30, wheel_base=0.36, max_v=1.20,
           lidar_h=0.22, lidar_range=12.0)

# 通道与工位间距（改这两个数就能整体调松紧；见文件头注释）
AISLE = 3.00          # 主通道净宽
LANE_PITCH = 3.00     # 同排相邻工位间距
LANES_PER_STAGE = 6   # 每道工序 6 个工位（梳棉 : 并条 : 粗纱 = 6 : 6 : 6）

# ---------------------------------------------------------------- 机器
# 真实纺纱设备的占地（公开规格量级）。机器**宽**决定同排工位间距，
# **长**决定该工序在 x 方向占多宽 —— 两者都按 50 cm 车能绕行的余量取。
MACHINE_SPEC = {
    "carding": dict(len=3.00, wid=1.20, h=2.20, rgb=(0.32, 0.45, 0.40)),   # 梳棉机
    "drawing": dict(len=2.20, wid=1.00, h=1.80, rgb=(0.26, 0.36, 0.52)),   # 并条机
    "roving":  dict(len=3.20, wid=1.40, h=2.40, rgb=(0.42, 0.32, 0.50)),   # 粗纱机
}

# ---------------------------------------------------------------- 世界坐标
# x 是工序流向：储料 → 梳棉 → 并条 → 粗纱 → 成品库。
# 每段占 15 m（等料位 / 机器 / 完工位），段间留 6.2~9.4 m 的横向通道 ——
# 车间里本来就有主通道，同时让 3.0 m 的会车余量在两端都留得下。
_LANE_Y0, _LANE_DY = 22.0, 3.20          # 6 条产线的 y 坐标：22.0 … 38.0
LANES = [_LANE_Y0 + _LANE_DY * i for i in range(LANES_PER_STAGE)]

STAGES = [
    dict(name="carding", cn="梳棉", lanes=list(LANES),
         wait_x=25.00, machine_x=29.00, done_x=33.00, process_s=6.0),
    dict(name="drawing", cn="并条", lanes=list(LANES[:6]),
         wait_x=42.20, machine_x=45.00, done_x=47.20, process_s=7.5),
    dict(name="roving",  cn="粗纱", lanes=list(LANES[:6]),
         wait_x=56.40, machine_x=60.00, done_x=63.20, process_s=9.0),
]

# 工序之间 / 料区之间的净宽，全部 >= AISLE
STAGE_GAP = AISLE                 # 相邻工序之间的净宽（米）
_BASE_WAIT_X = STAGES[0]["wait_x"]
_CARDING_MACHINE_DX = STAGES[0]["machine_x"] - STAGES[0]["wait_x"]
_CARDING_DONE_DX = STAGES[0]["done_x"] - STAGES[0]["wait_x"]
_STAGE_MACHINE_DX = STAGES[1]["machine_x"] - STAGES[1]["wait_x"]
_STAGE_DONE_DX = STAGES[1]["done_x"] - STAGES[1]["wait_x"]

# ---------------------------------------------------------------- 储料
# 货架贴东西两端墙，取放位在货架前的通道上。
RACK = dict(w=1.60, h=3.20, depth=3.20)
STORAGE = {
    "empty": dict(x=5.00,  y=30.0, cn="空筒库",   rgb=(0.60, 0.64, 0.68)),
    "red":   dict(x=112.00, y=30.0, cn="粗纱成品库", rgb=(0.80, 0.26, 0.28)),
}
# 6 个取放位，间距 3.0 m，与产线同高
STORAGE_SLOT_YS = [_LANE_Y0 + _LANE_DY * i for i in range(6)]
STORAGE_SLOTS = {
    "empty": dict(x=10.40, ys=list(STORAGE_SLOT_YS)),
    "red":   dict(x=106.60, ys=list(STORAGE_SLOT_YS)),
}

# 待命区：放在**生产区正北的中央通道**，而不是贴南墙。
#
# 为什么（第 68 节）：厂区放大到 120×60 m 之后，一次运输是双程 ~390 m，
# 其中"从待命区开到取货点"这一段完全是空驶。待命区原先贴南墙放在 x=40~57，
# 而空筒库在西端 x=10.4、成品库在东端 x=106.6 —— 单趟定位就要 30~60 m。
# 放到生产区（x≈25~63）正北的中央，到两端的距离都缩短，
# 而且仍避开所有取放位与生产通道（生产区 y=20.8~39.2）。
PARK = dict(y=46.0, x0=34.0, dx=3.50, n=10)   # 一排 10 个待命位，与默认车队一致
CHARGER_CABINET_DY = -1.20
CHARGERS = {
    "charger_0": dict(x=60.0, y=46.0, rgb=(0.10, 0.60, 0.85)),
    "charger_1": dict(x=64.0, y=46.0, rgb=(0.10, 0.60, 0.85)),
    "charger_2": dict(x=68.0, y=46.0, rgb=(0.10, 0.60, 0.85)),
}


# 储料区：货架贴墙，取放位在货架前的通道

# 取放位：空筒库与梳棉通道同高，成品库与粗纱通道同高

# 空筒取放位数量上限（第 52 节）。
#
# 这是全厂**喂料并发**的硬上限：工厂的取货点是"环绕料区的空闲停靠位"，
# 每个位一次只放 1 件，所以开局最多只能并发 4 单，之后每有一件走到下一
# 工序才腾出空位、放一件新料。实测 4 台车 300 s 只做出 16~22 件，
# 而 16 件物料里结束时还有 12 件躺在库里 —— 说明**产量主要卡在喂料并发**，
# 不在车队。做成可调参数是为了能直接验证这句话：翻倍之后产量该上去。
EMPTY_SLOT_YS = [3.0, 6.0, 9.0, 12.0]
EXTRA_EMPTY_SLOT_YS = [4.5, 7.5, 10.5, 13.5]      # 与上面交错，避免并排过近
EMPTY_SLOTS_MAX = 8


def empty_slot_ys(n: int) -> list[float]:
    """按需要的取放位数量给出 y 坐标（4 个 -> 原布局，8 个 -> 交错铺满）。"""
    n = max(1, min(int(n), EMPTY_SLOTS_MAX))
    if n <= len(EMPTY_SLOT_YS):
        return EMPTY_SLOT_YS[:n]
    return sorted(EMPTY_SLOT_YS + EXTRA_EMPTY_SLOT_YS[: n - len(EMPTY_SLOT_YS)])

# 充电位：机柜贴墙，AGV 停在机柜前方的停靠点
CHARGER_CABINET_DY = -0.85          # 机柜相对停靠点的偏移（贴向下墙）

# AGV 待命区（贴下墙一字排开，避开所有取放位）

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


def _rebuild_park():
    """待命排随厂房宽度居中（粗纱/红料位会因净宽调整而东移）。"""
    east = STORAGE["red"]["x"] + RACK["depth"] / 2
    PARK["x0"] = max(9.60, (east - 1.0 - PARK["dx"] * (PARK["n"] - 1)) / 2
                     + PARK["dx"] * (PARK["n"] - 1) * 0.25)


def _rebuild_slot_points():
    STORAGE_SLOT_POINTS.clear()
    for zone, spec in STORAGE_SLOTS.items():
        for i, y in enumerate(spec["ys"]):
            name = f"storage_{zone}_slot_{i}"
            STORAGE_SLOT_POINTS[name] = dict(name=name, x=spec["x"], y=y)


def set_empty_slots(n: int) -> int:
    """改空筒取放位数量，重建 STORAGE_SLOT_POINTS，返回实际生效的数量。

    必须在**任何节点构造之前**调用：`STORAGE_SLOT_POINTS` 是模块级字典，
    `factory_manager` / `dashboard` / `live_view` 都在构造时就把工位建好了，
    构造之后再改就只改了布局、没改工位，两边对不上。
    所以调用点放在各可执行文件 `main()` 的第一行（见 `bootstrap()`）。
    """
    n = max(1, min(int(n), EMPTY_SLOTS_MAX))
    STORAGE_SLOTS["empty"]["ys"] = empty_slot_ys(n)
    _rebuild_slot_points()
    return n


def bootstrap() -> int:
    """可执行文件入口的统一起手：按环境变量调整布局。

    为什么用环境变量而不是 ROS 参数：布局必须在节点构造前定下来，
    而 ROS 参数要等节点构造后才能读 —— 顺序上就是矛盾的。
    launch 里 `SetEnvironmentVariable('FLEETFLOW_EMPTY_SLOTS', ...)` 即可，
    不设则用默认的 4 个，与历史行为完全一致。
    """
    import os
    raw = os.environ.get("FLEETFLOW_EMPTY_SLOTS", "").strip()
    if not raw:
        return len(STORAGE_SLOTS["empty"]["ys"])
    try:
        return set_empty_slots(int(raw))
    except ValueError:
        return len(STORAGE_SLOTS["empty"]["ys"])



def world_bounds() -> tuple[float, float, float, float]:
    """厂房在世界里的可视边界 (x_min, x_max, y_min, y_max)。

    直接用厂房外形（`BUILDING`），**不再由工位包络推算** —— 推算出来的只是
    "工位覆盖到哪"，不是厂房边界，会让渲染器把 120×60 m 的厂房画成 114×16 m，
    平面图随之被压扁、与真实比例不符。
    """
    return 0.0, BUILDING["w"], 0.0, BUILDING["h"]

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


# 正常生产时通道里本来就有的东西：待转运的条筒托盘、清洁工具车、临时堆放的棉包。
# 它们不在机台行列上，而是压在**通道中间**——车必须中途绕开，而不是沿着机弄直着走。
AISLE_OBSTACLES = [
    dict(name="pallet_a", x=9.90, y=7.50, sx=1.10, sy=0.90, cn="待转条筒"),
    dict(name="cart_b",   x=15.60, y=7.50, sx=0.90, sy=0.80, cn="清洁车"),
    dict(name="bale_c",   x=20.80, y=4.50, sx=1.20, sy=0.95, cn="棉包"),
]


def obstacle_rects():
    out = []
    for o in AISLE_OBSTACLES:
        out.append((o["x"] - o["sx"] / 2, o["y"] - o["sy"] / 2,
                    o["x"] + o["sx"] / 2, o["y"] + o["sy"] / 2))
    return out


# 规划器给静态障碍留的膨胀量（车体外接圆半径 0.356 上取整到栅格可用的值）。
# 必须与 planner.Grid 的 inflate 默认值一致 —— 世界审计脚本靠它判断
# "某个碰撞体是否已被已知障碍覆盖"。
STATIC_INFLATE = 0.28

# 从世界碰撞体自动导出的补充障碍（tools/derive_obstacles.py 生成）。
# 手写清单只覆盖机台的**主体**矩形，而 build_world.py 还给机器加了电机、
# 警示垫、并条机后部条筒架、粗纱机端板等外凸零件，以及办公室隔墙。
# 规划器看不见它们，车就会"照合法路径走、半路被 LiDAR 拦下"。
_WORLD_OBSTACLES = None


def world_obstacle_rects():
    """自动导出的补充障碍矩形；文件缺失时返回空表（不致命）。"""
    global _WORLD_OBSTACLES
    if _WORLD_OBSTACLES is None:
        import json
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        cands = [
            # 源码树（--symlink-install 与直接跑源码都走这条）
            os.path.join(here, "..", "config", "obstacles_world.json"),
            # 安装后的 share 目录
            os.path.join(here, "..", "..", "share", "fleetflow_sim", "config",
                         "obstacles_world.json"),
        ]
        _WORLD_OBSTACLES = []
        for path in cands:
            try:
                with open(path) as f:
                    data = json.load(f)
                _WORLD_OBSTACLES = [tuple(o["rect"]) for o in data.get("obstacles", [])]
                break
            except (OSError, ValueError, KeyError):
                continue
    return _WORLD_OBSTACLES


def base_static_boxes():
    """**手写**的静态障碍矩形：机台主体 / 货架 / 充电柜 / 通道杂物。

    单独留一个函数是必要的：`tools/derive_obstacles.py` 要拿它判断"某个
    碰撞体是否已被已知障碍覆盖"。如果它去比 `static_boxes()`（已经并入了
    导出结果），那第二次生成时所有零件都"已被覆盖"，导出清单会变成空 ——
    生成器就不幂等了。这个坑实际踩到过：重跑一次从 15 个变 0 个。
    """
    boxes = [m[1] for m in all_machines()]
    boxes += list(rack_rects().values())
    boxes += obstacle_rects()
    for c in CHARGERS.values():
        cy = c["y"] + CHARGER_CABINET_DY
        boxes.append((c["x"] - 0.34, cy - 0.26, c["x"] + 0.34, cy + 0.26))
    return boxes


def static_boxes():
    """A* 用的静态障碍矩形 [(x0, y0, x1, y1), ...]。

    = 手写清单（业务语义：哪台机器占哪块地）+ 从世界碰撞体导出的补充障碍
    （物理事实：电机、警示垫、并条机条筒架、粗纱机端板、办公室隔墙……）。
    两套合并是刻意的，缺了后者就会出现"路径合法但半路被 LiDAR 拦下"。
    """
    return base_static_boxes() + world_obstacle_rects()


def park_poses(n: int):
    """待命区位姿 [(x, y, yaw), ...]。"""
    out = []
    for i in range(n):
        row, col = divmod(i, PARK["n"])
        out.append((PARK["x0"] + PARK["dx"] * col, PARK["y"] + 1.15 * row, 0.0))
    return out
