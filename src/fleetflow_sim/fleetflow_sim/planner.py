"""栅格 A* 与纯追踪控制器。

厂房里障碍不多（机器圆柱 + 外墙），所以用 0.1 m 栅格、8 邻域 A*，再做一次
视线拉直（string pulling），最后交给纯追踪（pure pursuit）跟随。
"""
from __future__ import annotations

import heapq
import math

from . import layout

RES = 0.10  # 栅格分辨率 (m)


class Grid:
    """把厂房离散成栅格，并标记障碍。"""

    def __init__(self, res: float = RES, inflate: float = 0.28):
        self.res = res
        self.x_min, self.y_min = layout.FIELD["x_min"], layout.FIELD["y_min"]
        self.x_max, self.y_max = layout.FIELD["x_max"], layout.FIELD["y_max"]
        self.nx = int(round((self.x_max - self.x_min) / res)) + 1
        self.ny = int(round((self.y_max - self.y_min) / res)) + 1
        self.blocked = bytearray(self.nx * self.ny)      # 静态层：机器/货架/充电柜
        self.dynamic = bytearray(self.nx * self.ny)      # 动态层：其他车辆
        self.inflate = inflate
        for x0, y0, x1, y1 in layout.static_boxes():
            self._paint_box(x0, y0, x1, y1, inflate)

    def _paint_box(self, x0, y0, x1, y1, pad):
        """矩形障碍按车体半径外扩后写入静态层。"""
        i0, j0 = self.to_ij(x0 - pad, y0 - pad)
        i1, j1 = self.to_ij(x1 + pad, y1 + pad)
        for i in range(max(0, i0), min(self.nx, i1 + 1)):
            for j in range(max(0, j0), min(self.ny, j1 + 1)):
                self.blocked[j * self.nx + i] = 1

    def set_dynamic(self, points, radius: float = 0.45):
        """把当前其他车辆的位置写成动态障碍。

        真实多机系统里这叫动态代价地图：路径规划时就把同伴算进去，
        比"沿固定路径硬挤 + 局部避让"更不容易堵死。
        """
        self.dynamic = bytearray(self.nx * self.ny)
        for x, y in points:
            i0, j0 = self.to_ij(x - radius, y - radius)
            i1, j1 = self.to_ij(x + radius, y + radius)
            for i in range(max(0, i0), min(self.nx, i1 + 1)):
                for j in range(max(0, j0), min(self.ny, j1 + 1)):
                    px, py = self.to_xy(i, j)
                    if (px - x) ** 2 + (py - y) ** 2 <= radius * radius:
                        self.dynamic[j * self.nx + i] = 1

    def clear_dynamic(self):
        self.dynamic = bytearray(self.nx * self.ny)

    def _paint_disc(self, cx, cy, r):
        i0, i1 = self.to_ij(cx - r - self.res, cy - r - self.res)
        j0, j1 = self.to_ij(cx + r + self.res, cy + r + self.res)
        for i in range(max(0, i0), min(self.nx, i1 + 1)):
            for j in range(max(0, j0), min(self.ny, j1 + 1)):
                x, y = self.to_xy(i, j)
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                    self.blocked[j * self.nx + i] = 1

    def to_ij(self, x, y):
        return (int(round((x - self.x_min) / self.res)), int(round((y - self.y_min) / self.res)))

    def to_xy(self, i, j):
        return (self.x_min + i * self.res, self.y_min + j * self.res)

    def free(self, i, j) -> bool:
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return False
        k = j * self.nx + i
        return not self.blocked[k] and not self.dynamic[k]

    def nearest_free(self, x, y):
        """把落在障碍里的目标点吸到最近的可行驶格。"""
        i, j = self.to_ij(x, y)
        if self.free(i, j):
            return i, j
        for r in range(1, 40):
            for di in range(-r, r + 1):
                for dj in (-r, r):
                    if self.free(i + di, j + dj):
                        return i + di, j + dj
            for dj in range(-r + 1, r):
                for di in (-r, r):
                    if self.free(i + di, j + dj):
                        return i + di, j + dj
        return i, j


def astar(grid: Grid, start, goal):
    """返回 [(x, y), ...]；无解返回 []。start/goal 是 (x, y)。"""
    si, sj = grid.nearest_free(*start)
    gi, gj = grid.nearest_free(*goal)
    if (si, sj) == (gi, gj):
        return [grid.to_xy(gi, gj)]
    openq = [(0.0, si, sj)]
    came: dict = {}
    g = {(si, sj): 0.0}
    h = lambda i, j: math.hypot(i - gi, j - gj)
    seen = set()
    while openq:
        _, i, j = heapq.heappop(openq)
        if (i, j) in seen:
            continue
        seen.add((i, j))
        if (i, j) == (gi, gj):
            path = [(gi, gj)]
            while path[-1] in came:
                path.append(came[path[-1]])
            path.reverse()
            return [grid.to_xy(a, b) for a, b in path]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ni, nj = i + di, j + dj
            if not grid.free(ni, nj):
                continue
            step = math.hypot(di, dj)
            ng = g[(i, j)] + step
            if ng < g.get((ni, nj), 1e18):
                g[(ni, nj)] = ng
                came[(ni, nj)] = (i, j)
                heapq.heappush(openq, (ng + h(ni, nj), ni, nj))
    return []


def _visible(grid: Grid, a, b) -> bool:
    """两点之间是否无遮挡（用于路径拉直）。"""
    ax, ay = a
    bx, by = b
    dist = math.hypot(bx - ax, by - ay)
    steps = max(2, int(dist / (grid.res * 0.5)))
    for k in range(1, steps):
        t = k / steps
        if not grid.free(*grid.to_ij(ax + (bx - ax) * t, ay + (by - ay) * t)):
            return False
    return True


def smooth(grid: Grid, path):
    """视线拉直：能直达就跳过中间点。"""
    if len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _visible(grid, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan(grid: Grid, start, goal):
    return smooth(grid, astar(grid, start, goal))


def path_length(path) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


class PurePursuit:
    """沿折线路径跟随，输出 (v, w)。"""

    def __init__(self, lookahead: float = 0.55, v_max: float = 0.85, w_max: float = 1.6):
        self.lookahead = lookahead
        self.v_max = v_max
        self.w_max = w_max
        self.path: list = []
        self.idx = 0
        self.px = 0.0
        self.py = 0.0
        self._have_pose = False

    def set_path(self, path):
        self.path = list(path)
        self.idx = 0

    @property
    def finished(self) -> bool:
        return not self.path or self.idx >= len(self.path)

    def remaining_path(self) -> list:
        """还没走完的路径点（含当前索引那一点），供可视化使用。"""
        if self.finished:
            return []
        return list(self.path[self.idx:])

    def remaining(self) -> float:
        if self.finished:
            return 0.0
        rest = self.path[self.idx:]
        if self._have_pose:
            return path_length([(self.px, self.py)] + rest)
        return path_length(rest)

    def step(self, x, y, yaw):
        """推进一步，返回 (v, w)。到达终点返回 (0, 0)。"""
        self.px, self.py = x, y
        self._have_pose = True
        if self.finished:
            return 0.0, 0.0
        # 命中当前目标点就前进
        tx, ty = self.path[self.idx]
        if math.hypot(tx - x, ty - y) < 0.22 and self.idx < len(self.path) - 1:
            self.idx += 1
            tx, ty = self.path[self.idx]
        # 取前视点
        while self.idx < len(self.path) - 1:
            tx, ty = self.path[self.idx]
            if math.hypot(tx - x, ty - y) >= self.lookahead:
                break
            self.idx += 1
        tx, ty = self.path[self.idx]
        if self.idx == len(self.path) - 1 and math.hypot(tx - x, ty - y) < 0.10:
            # 到达终点：必须把自己标记为完成，否则调用方拿到的 v=0 却以为还没到，
            # 在没有坐标目标的状态（如离站 staging）下会永久停住。
            self.idx = len(self.path)
            return 0.0, 0.0
        dx, dy = tx - x, ty - y
        # 车体系下的横向误差
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        lx = dx * cy - dy * sy
        ly = dx * sy + dy * cy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0
        alpha = math.atan2(ly, lx)
        # ---------------------------------------------------------------
        # 先对准、再前进。这一条是 Gazebo 能不能干活的关键。
        #
        # 旧实现在朝向误差大时**仍然保持前进**：
        #     v = v_max * max(0.25, ...)        -> 最低也有 0.21 m/s
        #     if abs(alpha) > 1.2: v = 0.15     -> 误差 >69° 时仍以 0.15 前进
        #     w = clamp(2.2*alpha, ±1.6)
        # 于是误差超过约 69° 之后，前进与转向互相抵消，车沿半径
        # v/w = 0.15/1.6 ≈ 0.094 m 的圆**无限打转**，alpha 永不收敛。
        #
        # 实测（8 车、90 s）：Gazebo 里单台车 yaw 极差达 **6.28 rad（整整一圈）**、
        # 平均 |w| 0.47~0.85 rad/s，而同一场景在纯逻辑（理想运动学、出身朝向
        # 正好沿路径）下换向次数为 **0**。后果是有效位移速度只有 0.133 m/s，
        # 而同一时刻上报速度均值 0.33 m/s —— 三分之二的运动被浪费在转圈上，
        # 取货段中位耗时因此从 7.5 s 涨到 22.5 s。
        #
        # 真实 AGV 的做法是"原地对准后再走"：误差大就停下来转，误差小了再加速。
        # 这同时消除了打转，也让路径跟踪更准。
        if abs(alpha) > 1.2:
            # 误差 > 69°：停住，原地转（用较低的角速度，避免过冲再反向）
            return 0.0, max(-1.2, min(1.2, 2.0 * alpha))
        if abs(alpha) > 0.5:
            # 误差 29°~69°：慢速前进 + 正常转向
            return min(0.25, self.v_max * 0.3), max(-self.w_max,
                                                     min(self.w_max, 2.2 * alpha))
        v = self.v_max * max(0.25, min(1.0, lx / max(dist, 1e-6)))
        w = max(-self.w_max, min(self.w_max, 2.2 * alpha))
        return v, w
