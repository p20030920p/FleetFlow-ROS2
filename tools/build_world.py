#!/usr/bin/env python3
"""生成 Gazebo Sim 8 的纺纱车间世界。

设计目标：**像一座真车间，而不是一堆几何体**
* 厂房结构：混凝土地坪、带窗带的围墙、柱网、屋架、灯具
* 真实设备：按纺纱厂尺度建模的梳棉机 / 并条机 / 粗纱机（含喂入、牵伸、卷绕、控制面板）
* 真实物料：条筒（sliver can），颜色标记工序
* 车间要素：货架、安全黄线、通道标线、斑马警示带、充电机柜、待命位
* 三个抓图机位 + 一个设备特写机位

    python3 tools/build_world.py            # 写入包内 worlds/textile_factory.sdf
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "fleetflow_sim"))
from fleetflow_sim import layout as L  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "src", "fleetflow_sim",
                   "worlds", "textile_factory.sdf")

# ------------------------------------------------------------------ 材质
def mat(rgb, a=1.0, rough=0.65, metal=0.0):
    r, g, b = rgb
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} {a}</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} {a}</diffuse>"
            f"<specular>0.18 0.18 0.18 1</specular>"
            f"<pbr><metal><roughness>{rough}</roughness><metalness>{metal}</metalness>"
            f"</metal></pbr></material>")


def box(name, cx, cy, cz, sx, sy, sz, rgb, **kw):
    return (f'<model name="{name}"><static>true</static><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>'
            f'<link name="l"><collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'
            f'</geometry></collision><visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size>'
            f'</box></geometry>{mat(rgb, **kw)}</visual></link></model>')


def cyl(name, cx, cy, cz, r, h, rgb, rot=(0, 0, 0), **kw):
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{cx:.3f} {cy:.3f} {cz:.3f} {rot[0]:.4f} {rot[1]:.4f} {rot[2]:.4f}</pose>'
            f'<link name="l"><collision name="c"><geometry><cylinder><radius>{r:.3f}</radius>'
            f'<length>{h:.3f}</length></cylinder></geometry></collision>'
            f'<visual name="v"><geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length>'
            f'</cylinder></geometry>{mat(rgb, **kw)}</visual></link></model>')


PARTS: list[str] = []


def add(svg):
    PARTS.append("    " + svg)


# ------------------------------------------------------------------ 厂房
def building():
    W, H, WH = L.BUILDING["w"], L.BUILDING["h"], L.BUILDING["wall_h"]
    add(box("floor_slab", W / 2, H / 2, -0.10, W, H, 0.20, (0.56, 0.57, 0.58), rough=0.94))
    add(box("floor_wear", W / 2, H / 2, 0.005, W - 1.2, H - 1.2, 0.01, (0.52, 0.53, 0.54), rough=0.96))

    t = 0.28
    # 南墙与东墙是等距机位最近的两面：做成 1.2 m 的"剖切矮墙"，
    # 保留"这是一栋厂房"的读法，同时让机位能看进车间内部。
    CUT = 1.20
    add(box("wall_south", W / 2, -t / 2, CUT / 2, W + 2 * t, t, CUT, (0.86, 0.86, 0.84), rough=0.9))
    add(box("wall_east", W + t / 2, H / 2, CUT / 2, t, H, CUT, (0.86, 0.86, 0.84), rough=0.9))
    add(box("wall_north", W / 2, H + t / 2, WH / 2, W + 2 * t, t, WH, (0.86, 0.86, 0.84), rough=0.9))
    add(box("wall_west", -t / 2, H / 2, WH / 2, t, H, WH, (0.86, 0.86, 0.84), rough=0.9))
    # 剖切面的"切口"色带，读起来像被切开而不是本来就这么矮
    add(box("cut_s", W / 2, -t / 2, CUT + 0.02, W + 2 * t, t + 0.02, 0.05, (0.62, 0.63, 0.62)))
    add(box("cut_e", W + t / 2, H / 2, CUT + 0.02, t + 0.02, H, 0.05, (0.62, 0.63, 0.62)))
    # 窗带（略透的青蓝色，给车间一点天光感）—— 只留远侧两面
    for i, (nm, cx, cy, sx, sy) in enumerate([
            ("win_n", W / 2, H + t / 2, W - 0.5, t + 0.02),
            ("win_w", -t / 2, H / 2, t + 0.02, H - 0.5)]):
        add(box(nm, cx, cy, 4.10, sx, sy, 1.30, (0.42, 0.58, 0.70), a=0.55, rough=0.15, metal=0.1))

    # 柱网
    n = 0
    for x in [0.55, L.COLUMN_PITCH + 0.55, 2 * L.COLUMN_PITCH + 0.55, W - 0.55]:
        for y in [0.55, H - 0.55]:
            add(box(f"col_{n}", x, y, WH / 2, L.COLUMN_SIZE, L.COLUMN_SIZE, WH,
                    (0.80, 0.80, 0.78), rough=0.85))
            n += 1
    # 屋架 + 灯具（不铺屋面板，否则俯视机位被挡）
    for i, x in enumerate([L.TRUSS_PITCH * k + 1.0 for k in range(4)]):
        add(box(f"truss_{i}", x, H / 2, WH - 0.30, 0.26, H, 0.40, (0.55, 0.56, 0.58), metal=0.6, rough=0.4))
        add(box(f"truss_lo_{i}", x, H / 2, WH - 0.62, 0.14, H, 0.16, (0.50, 0.51, 0.53), metal=0.6))
    for i, x in enumerate([L.TRUSS_PITCH * k + 4.2 for k in range(4)]):
        for j, y in enumerate([4.0, 8.0, 12.0]):
            add(box(f"lamp_{i}_{j}", x, y, WH - 0.72, 1.30, 0.30, 0.10, (1.0, 0.98, 0.90),
                    rough=0.3))
    cleaner_rails()


def cleaner_rails():
    """架空巡回清洁轨道 —— 棉纺车间最有辨识度的设备。

    每个工序的每一条机弄上方都有一条纵向风道，风道上每隔 1.5 m 挂一个吸嘴，
    沿轨道往复巡回把飞花吸走。少了它，车间看起来就只是"一堆盒子"。
    """
    for o in L.AISLE_OBSTACLES:
        # 通道里的临时堆放物：有实体、有警示色，车绕开它们
        add(box(f"obs_{o['name']}", o["x"], o["y"], 0.34, o["sx"], o["sy"], 0.68,
                (0.72, 0.68, 0.52), rough=0.85))
        add(box(f"obsband_{o['name']}", o["x"], o["y"], 0.70, o["sx"] + 0.06,
                o["sy"] + 0.06, 0.06, (0.92, 0.74, 0.22)))
        add(box(f"obsfoot_{o['name']}", o["x"], o["y"], 0.06, o["sx"] + 0.16,
                o["sy"] + 0.16, 0.10, (0.34, 0.33, 0.30)))

    for spec in L.STAGES:
        stage = spec["name"]
        h = L.MACHINE_SPEC[stage]["h"]
        cx = spec["machine_x"]
        half = L.MACHINE_SPEC[stage]["len"] / 2.0 + 1.9   # 风道比机台两头各长出一截
        z = h + 0.52
        for li, lane in enumerate(spec["lanes"]):
            add(box(f"cln_{stage}_{li}", cx, lane, z, 2 * half, 0.15, 0.20,
                    (0.72, 0.74, 0.76), metal=0.5, rough=0.35))
            n = int(2 * half / 1.5)
            for k in range(n + 1):
                x = cx - half + 1.5 * k
                add(cyl(f"clnz_{stage}_{li}_{k}", x, lane, z - 0.24, 0.045, 0.30,
                        (0.58, 0.60, 0.62), metal=0.5, rough=0.35))
            # 吊杆：把风道挂到屋架上
            for k in range(3):
                x = cx - half + 0.8 + k * (2 * half - 1.6) / 2.0
                add(box(f"clnh_{stage}_{li}_{k}", x, lane, (z + L.BUILDING["wall_h"]) / 2,
                        0.06, 0.06, L.BUILDING["wall_h"] - z, (0.55, 0.57, 0.59),
                        metal=0.6))


# ------------------------------------------------------------------ 地面标线
def markings():
    # 安全黄线：沿每条生产通道两侧
    for s in L.STAGES:
        for lane in s["lanes"]:
            x0 = s["wait_x"] - 0.95
            x1 = s["done_x"] + 0.95
            for sy in (-0.90, 0.90):
                add(box(f"line_{s['name']}_{lane}_{'a' if sy < 0 else 'b'}",
                        (x0 + x1) / 2, lane + sy, 0.012, x1 - x0, 0.10, 0.02, (0.92, 0.80, 0.10)))
    # 混凝土地坪伸缩缝（每 6 m 一道，真实车间地面就有）
    for i in range(1, 5):
        add(box(f"joint_x_{i}", i * 6.0, 8.0, 0.010, 0.06, 15.6, 0.016, (0.52, 0.53, 0.54)))
    for j in range(1, 3):
        add(box(f"joint_y_{j}", 13.0, j * 5.5, 0.010, 25.6, 0.06, 0.016, (0.52, 0.53, 0.54)))
    # 参观/物流通道（浅色环氧地坪带）
    for j, (y0, w) in enumerate([(7.30, 1.40)]):
        add(box(f"walkway_{j}", 13.0, y0, 0.008, 25.0, w, 0.014, (0.66, 0.67, 0.66), rough=0.88))

    # 主通道中心虚线
    for i in range(17):
        add(box(f"dash_{i}", 1.8 + i * 1.45, 8.0, 0.012, 0.70, 0.09, 0.02, (0.90, 0.90, 0.86)))
    # 取放位标记：每个泊位一个环
    for name, pt in L.STORAGE_SLOT_POINTS.items():
        col = (0.22, 0.68, 0.38) if "empty" in name else (0.80, 0.26, 0.28)
        for k in range(8):
            a0 = 2 * math.pi * k / 8
            add(box(f"slot_{name}_{k}", pt["x"] + 0.40 * math.cos(a0), pt["y"] + 0.40 * math.sin(a0),
                    0.012, 0.16, 0.10, 0.02, col))
    # 泊位间的斑马警示带
    for zone, spec in L.STORAGE_SLOTS.items():
        for i, y in enumerate(spec["ys"]):
            for k in range(3):
                add(box(f"hz_{zone}_{i}_{k}", spec["x"] - 0.62 - k * 0.28, y, 0.012,
                        0.16, 1.05, 0.02, (0.95, 0.82, 0.10) if k % 2 == 0 else (0.15, 0.15, 0.16)))
    # 待命区方框
    p0 = L.PARK["x0"] - 0.75
    p1 = L.PARK["x0"] + L.PARK["dx"] * (L.PARK["n"] - 1) + 0.75
    for sy in (-0.72, 0.72):
        add(box(f"park_line_{'a' if sy < 0 else 'b'}", (p0 + p1) / 2, L.PARK["y"] + sy,
                0.012, p1 - p0, 0.08, 0.02, (0.94, 0.94, 0.92)))
    for i in range(L.PARK["n"] + 1):
        add(box(f"park_tick_{i}", p0 + i * L.PARK["dx"], L.PARK["y"], 0.012, 0.08, 1.44, 0.02,
                (0.94, 0.94, 0.92)))


# ------------------------------------------------------------------ 机器
def carding_machine(name, x0, y, rgb):
    """梳棉机：喂入段 + 锡林罩壳 + 道夫卷绕 + 控制面板。"""
    cx = x0 + 1.75
    add(box(f"{name}_plinth", cx, y, 0.09, 3.50, 1.20, 0.18, (0.30, 0.31, 0.33), metal=0.4, rough=0.6))
    add(box(f"{name}_body", cx, y, 0.72, 3.30, 1.06, 1.08, rgb, rough=0.45, metal=0.25))
    add(box(f"{name}_housing", x0 + 1.35, y, 1.66, 1.70, 1.18, 0.62, (0.38, 0.42, 0.44),
            metal=0.45, rough=0.4))
    add(cyl(f"{name}_cylinder", x0 + 1.35, y, 1.66, 0.30, 1.22, (0.55, 0.57, 0.60),
            rot=(math.pi / 2, 0, 0), metal=0.7, rough=0.25))
    add(box(f"{name}_feed", x0 + 0.34, y, 1.33, 0.66, 1.00, 0.62, rgb, rough=0.5))
    add(box(f"{name}_feed_hopper", x0 + 0.20, y, 1.86, 0.34, 0.86, 0.44, (0.72, 0.74, 0.76)))
    add(cyl(f"{name}_coiler", x0 + 3.05, y, 1.30, 0.24, 0.52, (0.62, 0.64, 0.66), metal=0.6, rough=0.3))
    add(box(f"{name}_panel", cx - 0.30, y + 0.58, 1.16, 0.52, 0.10, 0.42, (0.16, 0.17, 0.19)))
    add(box(f"{name}_screen", cx - 0.30, y + 0.64, 1.18, 0.40, 0.03, 0.28, (0.20, 0.55, 0.72),
            rough=0.2))
    add(cyl(f"{name}_motor", x0 + 0.70, y - 0.70, 0.34, 0.20, 0.44, (0.30, 0.32, 0.34),
            rot=(math.pi / 2, 0, 0), metal=0.6))
    # 机架立柱与护罩，让它不像一个整块方盒
    for sx2 in (-1, 1):
        for dx in (-1.55, 1.55):
            add(box(f"{name}_post_{'a' if sx2<0 else 'b'}_{'l' if dx<0 else 'r'}",
                    x0 + 1.75 + dx, y + sx2 * 0.56, 0.60, 0.10, 0.10, 1.20, (0.22, 0.24, 0.26),
                    metal=0.5))
    add(box(f"{name}_guard", x0 + 1.35, y - 0.62, 1.66, 1.74, 0.05, 0.66, (0.72, 0.74, 0.70),
            a=0.75, rough=0.25))
    add(cyl(f"{name}_duct", x0 + 2.30, y + 0.40, 2.05, 0.14, 1.40, (0.66, 0.68, 0.70),
            rot=(math.pi / 2, 0, 0), metal=0.55, rough=0.35))
    add(box(f"{name}_hazard", x0 + 0.02, y, 0.30, 0.05, 1.10, 0.34, (0.94, 0.78, 0.10)))


def drawing_machine(name, x0, y, rgb):
    """并条机：机架 + 后部条筒架 + 两个圈条头。"""
    cx = x0 + 1.25
    add(box(f"{name}_plinth", cx, y, 0.08, 2.50, 0.90, 0.16, (0.30, 0.31, 0.33), metal=0.4))
    add(box(f"{name}_body", cx, y, 0.62, 2.30, 0.84, 0.92, rgb, rough=0.45, metal=0.25))
    add(box(f"{name}_draft", cx, y, 1.18, 2.10, 0.70, 0.22, (0.60, 0.62, 0.64), metal=0.6, rough=0.3))
    for i, dx in enumerate((-0.55, 0.55)):
        add(cyl(f"{name}_head_{i}", cx + dx, y, 1.42, 0.19, 0.36, (0.66, 0.68, 0.70),
                metal=0.6, rough=0.3))
    for i, dx in enumerate((-0.80, -0.27, 0.27, 0.80)):
        add(cyl(f"{name}_creel_{i}", cx + dx, y - 0.78, 0.48, 0.21, 0.92, (0.66, 0.68, 0.70)))
    add(box(f"{name}_panel", cx + 0.62, y + 0.46, 1.02, 0.42, 0.10, 0.36, (0.16, 0.17, 0.19)))
    add(box(f"{name}_screen", cx + 0.62, y + 0.51, 1.04, 0.32, 0.03, 0.24, (0.20, 0.55, 0.72)))


def roving_machine(name, x0, y, rgb):
    """粗纱机：机架 + 牵伸区 + 锭翼/筒管阵列（两排）。"""
    cx = x0 + 1.50
    add(box(f"{name}_plinth", cx, y, 0.09, 3.00, 1.40, 0.18, (0.30, 0.31, 0.33), metal=0.4))
    add(box(f"{name}_body", cx, y, 0.68, 2.80, 1.32, 1.00, rgb, rough=0.45, metal=0.25))
    add(box(f"{name}_draft", cx, y, 1.36, 2.60, 0.80, 0.28, (0.60, 0.62, 0.64), metal=0.6, rough=0.3))
    for row, dy in enumerate((-0.44, 0.44)):
        for i in range(7):
            bx = cx - 1.20 + i * 0.40
            add(cyl(f"{name}_bob_{row}_{i}", bx, y + dy, 1.74, 0.075, 0.44, (0.86, 0.84, 0.80)))
            add(cyl(f"{name}_fly_{row}_{i}", bx, y + dy, 2.02, 0.045, 0.26, (0.55, 0.57, 0.60),
                    metal=0.7, rough=0.25))
    for sgn in (-1, 1):
        add(box(f"{name}_end_{'a' if sgn < 0 else 'b'}", cx + sgn * 1.45, y, 1.00, 0.24, 1.36, 1.60, rgb))
    add(box(f"{name}_panel", cx + 1.00, y + 0.72, 1.06, 0.44, 0.10, 0.38, (0.16, 0.17, 0.19)))
    add(box(f"{name}_screen", cx + 1.00, y + 0.77, 1.08, 0.34, 0.03, 0.26, (0.20, 0.55, 0.72)))


BUILDERS = dict(carding=carding_machine, drawing=drawing_machine, roving=roving_machine)


# ------------------------------------------------------------------ 条筒 / 货架 / 充电桩
def can(name, x, y, kind, z=0.0):
    rgb = L.CAN_RGB[kind]
    r, h = L.CAN["radius"], L.CAN["height"]
    s = cyl(f"{name}_b", x, y, z + h / 2, r, h, rgb, rough=0.55)
    s += cyl(f"{name}_rim", x, y, z + h - 0.02, r * 1.06, 0.05, (0.30, 0.31, 0.33), metal=0.6, rough=0.3)
    return s


def storage_rack(zone, spec):
    """货架：立柱 + 三层横梁 + 层板 + 其上的条筒。"""
    x0, y0, x1, y1 = L.rack_rects()[zone]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    sx, sy = x1 - x0, y1 - y0
    rgb = L.STORAGE[zone]["rgb"]
    add(box(f"rack_{zone}_base", cx, cy, 0.06, sx, sy, 0.12, (0.42, 0.44, 0.46), metal=0.5))
    for lv, z in enumerate((0.62, 1.32, 2.02)):
        add(box(f"rack_{zone}_shelf_{lv}", cx, cy, z, sx - 0.10, sy - 0.06, 0.05,
                (0.55, 0.57, 0.60), metal=0.55, rough=0.4))
        for i in range(6):
            yy = y0 + 0.80 + i * (sy - 1.60) / 5
            for sx2 in (-1, 1):
                add(cyl(f"rack_{zone}_can_{lv}_{i}_{'a' if sx2 < 0 else 'b'}",
                        cx + sx2 * sx * 0.22, yy, z + 0.50, 0.205, 0.90, rgb, rough=0.6))
    for i in range(5):
        yy = y0 + 0.10 + i * (sy - 0.20) / 4
        for sx2 in (-1, 1):
            add(box(f"rack_{zone}_post_{i}_{'a' if sx2 < 0 else 'b'}",
                    cx + sx2 * (sx / 2 - 0.06), yy, 1.22, 0.11, 0.11, 2.44,
                    (0.48, 0.50, 0.53), metal=0.5))
    add(box(f"rack_{zone}_label", cx + sx / 2 + 0.02, cy, 2.30, 0.03, sy - 0.4, 0.34, rgb))


def charger(name, spec):
    cx, cy = spec["x"], spec["y"] + L.CHARGER_CABINET_DY
    add(box(f"{name}_cab", cx, cy, 0.62, 0.66, 0.50, 1.24, (0.86, 0.87, 0.88), rough=0.5))
    add(box(f"{name}_strip", cx, cy + 0.27, 1.00, 0.52, 0.03, 0.16, spec["rgb"], rough=0.3))
    add(box(f"{name}_base", cx, spec["y"], 0.02, 0.90, 0.90, 0.04, (0.25, 0.35, 0.45)))


def park_bay():
    for i, (x, y, _) in enumerate(L.park_poses(8)):
        add(box(f"park_pad_{i}", x, y, 0.014, 0.92, 0.72, 0.02, (0.62, 0.63, 0.64), rough=0.9))


def control_room():
    add(box("office_base", 24.5, 14.1, 0.06, 2.60, 2.60, 0.12, (0.62, 0.63, 0.64)))
    add(box("office_wall_s", 24.5, 12.85, 1.30, 2.60, 0.12, 2.60, (0.78, 0.80, 0.82), rough=0.6))
    add(box("office_wall_w", 23.25, 14.1, 1.30, 0.12, 2.60, 2.60, (0.78, 0.80, 0.82), rough=0.6))
    add(box("office_glass", 24.5, 12.92, 1.70, 1.80, 0.05, 1.00, (0.35, 0.55, 0.68), a=0.55, rough=0.15))


# ------------------------------------------------------------------ 相机
def camera(name, x, y, z, pitch, yaw, w=1600, h=1000, fov=1.05):
    """机位相机。

    ``always_on=0`` 是关键：相机只在有订阅者时才渲染。默认没人订阅，
    于是完整仿真跑起来不用为 4 路 1600x1000 的离屏渲染买单（实测这一项就能把
    实时因子压到远低于 1）；需要出图时桥接对应话题，传感器自动激活。
    """
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x} {y} {z} 0 {pitch:.4f} {yaw:.4f}</pose><link name="l">'
            f'<sensor name="cam" type="camera"><camera><horizontal_fov>{fov}</horizontal_fov>'
            f'<image><width>{w}</width><height>{h}</height></image>'
            f'<clip><near>0.1</near><far>200</far></clip></camera>'
            f'<always_on>0</always_on><update_rate>2</update_rate><visualize>false</visualize>'
            f'<topic>{name}/image</topic></sensor></link></model>')


def cameras():
    W, H = L.BUILDING["w"], L.BUILDING["h"]
    out = [
        camera("view_top", 13.0, 8.0, 23.0, math.pi / 2, math.pi / 2, 1280, 800, 1.18),
        camera("view_iso", 36.0, -13.0, 26.0, 0.706, 2.43, 1280, 800, 1.00),
        camera("view_line", 2.8, 10.5, 1.90, 0.06, 0.08, 1280, 720, 1.30),
        camera("view_machine", 2.4, 4.6, 2.70, 0.24, 0.64, 1280, 800, 1.10),
    ]
    return out


# ------------------------------------------------------------------ 组装
def build():
    building()
    markings()
    for name, rect, h, rgb, stage, lane in L.all_machines():
        BUILDERS[stage](name, rect[0], (rect[1] + rect[3]) / 2, rgb)
    for zone in L.STORAGE:
        storage_rack(zone, L.STORAGE[zone])
    for cname, cspec in L.CHARGERS.items():
        charger(cname, cspec)
    park_bay()
    control_room()
    # 泊位点上**不放实体条筒**。条筒存放在货架上（storage_rack 已经摆满），
    # 泊位点是 AGV 的停车位置，地面上有画好的泊位标线。
    # 曾经在这里放过实体条筒，结果 AGV 被要求开进一个被占住的点：LiDAR 急停
    # 距离(0.38m)大于到达判定(0.20m)，车永远进不去，反复判"卡死"并放弃任务。

    head = f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="textile_mill">
    <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="skylight">
      <cast_shadows>true</cast_shadows><pose>0 0 24 0 0 0</pose>
      <diffuse>1.00 0.99 0.94 1</diffuse><specular>0.22 0.22 0.22 1</specular>
      <direction>-0.42 0.30 -1</direction>
      <shadow><cascade_count>3</cascade_count><texture_size>2048</texture_size>
      <cascade_distribution>0.45 0.75 0.95</cascade_distribution></shadow>
    </light>
    <light type="point" name="lamp_0"><cast_shadows>false</cast_shadows>
      <pose>5 5 5.2 0 0 0</pose><diffuse>0.82 0.80 0.74 1</diffuse><specular>0.14 0.14 0.14 1</specular>
      <attenuation><range>18</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.006</quadratic></attenuation>
    </light>
    <light type="point" name="lamp_1"><cast_shadows>false</cast_shadows>
      <pose>11 11 5.2 0 0 0</pose><diffuse>0.82 0.80 0.74 1</diffuse><specular>0.14 0.14 0.14 1</specular>
      <attenuation><range>18</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.006</quadratic></attenuation>
    </light>
    <light type="point" name="lamp_2"><cast_shadows>false</cast_shadows>
      <pose>19 6 5.2 0 0 0</pose><diffuse>0.82 0.80 0.74 1</diffuse><specular>0.14 0.14 0.14 1</specular>
      <attenuation><range>18</range><constant>0.6</constant><linear>0.05</linear><quadratic>0.006</quadratic></attenuation>
    </light>
    <scene><ambient>0.42 0.43 0.45 1</ambient><background>0.74 0.78 0.82 1</background><shadows>true</shadows></scene>'''
    out = [head]
    out += PARTS
    out += ["    " + c for c in cameras()]
    out.append("  </world>\n</sdf>\n")
    return "\n".join(out)


if __name__ == "__main__":
    text = build()
    with open(os.path.normpath(OUT), "w", encoding="utf-8") as f:
        f.write(text)
    print(f"wrote {os.path.normpath(OUT)} · {len(PARTS)} models · {len(text)//1024} KB")
