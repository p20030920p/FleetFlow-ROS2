#!/usr/bin/env python3
"""把渲染出来的 PNG 变成终端里的字符画，用来**在没有图形界面时核对版面**。

    python3 tools/ascii_view.py /tmp/full.png [列数]

为什么要这个工具：渲染器（render_demo / dashboard）画出来的东西是给眼睛看的，
而自动检查只能查几何数字。改动版面（挪面板、改字号、加标注）之后，"有没有压字、
有没有画到框外、区块是不是在它该在的位置"这类问题必须有办法**看**一眼。
字符画够用：色块位置、区块边界、文字带是否互相重叠都能看出来。

映射方式：按 8×16 的字符单元把图切成网格，每格取平均亮度，落到 ASCII 灰阶；
再按"该格主要是什么色相"附一个色相字母，便于区分同亮度的不同区块。
"""
from __future__ import annotations

import sys

import numpy as np
from PIL import Image

RAMP = " .:-=+*#%@"
HUE = {0: "r", 1: "y", 2: "g", 3: "c", 4: "b", 5: "m"}


def main() -> int:
    path = sys.argv[1]
    cols = int(sys.argv[2]) if len(sys.argv) > 2 else 150
    im = Image.open(path).convert("RGB")
    W, H = im.size
    cell_w = 8
    cell_h = 16
    rows = max(1, int(H / cell_h * (cell_w * cols / W) / 2.0))
    rows = max(1, round(H / W * cols / 2.0))
    a = np.asarray(im.resize((cols, rows), Image.BOX)).astype(float)
    lum = a.mean(axis=2)
    lo, hi = lum.min(), lum.max()
    norm = (lum - lo) / max(1e-6, hi - lo)
    out = []
    for r in range(rows):
        line = []
        for c in range(cols):
            v = norm[r, c]
            ch = RAMP[min(len(RAMP) - 1, int((1.0 - v) * (len(RAMP) - 1) + 0.5))]
            line.append(ch)
        out.append("".join(line))
    print(f"{path}  {W}x{H} -> {cols}x{rows}  亮度 {lo:.0f}..{hi:.0f}")
    print("+" + "-" * cols + "+")
    for line in out:
        print("|" + line + "|")
    print("+" + "-" * cols + "+")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
