#!/usr/bin/env python3
"""把三张车队动图并成一张对照图：同一时刻、不同车队规模并排。

为什么并排而不是各放一张：README 首页只能放一张图，而"车队规模如何影响产出"
正是要展示的结论。并排放在同一时间轴上，读者一眼就能看出
车少时通道更空、车多时同一时刻在途任务更多。

用法:
    python3 tools/combine_demos.py out.gif --panel "1 AGV=a.gif" --panel "2 AGVs=b.gif" \
        --panel "4 AGVs=c.gif"
每张输入 GIF 的帧数需一致（同一录制脚本、同一 --max-frames 参数即可）。
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageSequence

# 面板之间的间隔与顶部标签高度（像素，按 1240×680 的画布比例）
GAP = 14
LABEL_H = 34
BG = (244, 242, 237)


def load_frames(path: str) -> list[Image.Image]:
    im = Image.open(path)
    return [fr.convert("RGB") for fr in ImageSequence.Iterator(im)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--panel", action="append", default=[], metavar="LABEL=GIF")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--scale", type=float, default=1.0)
    args = ap.parse_args()
    if len(args.panel) < 2:
        print("至少给两个 --panel LABEL=GIF", file=sys.stderr)
        return 2

    panels = []
    for item in args.panel:
        if "=" not in item:
            print(f"--panel 需要 LABEL=GIF，收到 {item!r}", file=sys.stderr)
            return 2
        label, path = item.split("=", 1)
        panels.append((label, load_frames(path)))

    n = min(len(f) for _, f in panels)
    if n < 2:
        print("帧数太少", file=sys.stderr)
        return 2
    w, h = panels[0][1][0].size
    if args.scale != 1.0:
        w, h = int(w * args.scale), int(h * args.scale)

    cols = len(panels)
    W = cols * w + (cols + 1) * GAP
    H = LABEL_H + h + 2 * GAP
    try:
        from PIL import ImageFont
        font = ImageFont.truetype(
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 22)
    except Exception:                                          # noqa: BLE001
        font = ImageDraw.Draw(Image.new("RGB", (1, 1))).getfont()

    out_frames = []
    for i in range(n):
        canvas = Image.new("RGB", (W, H), BG)
        draw = ImageDraw.Draw(canvas)
        for c, (label, frames) in enumerate(panels):
            x = GAP + c * (w + GAP)
            fr = frames[i]
            if fr.size != (w, h):
                fr = fr.resize((w, h), Image.LANCZOS)
            canvas.paste(fr, (x, GAP + LABEL_H))
            draw.text((x + 6, GAP + 4), label, fill=(28, 28, 26), font=font)
        out_frames.append(canvas)

    out_frames[0].save(args.out, save_all=True, append_images=out_frames[1:],
                       duration=int(1000 / max(1.0, args.fps)), loop=0,
                       optimize=True)
    print(f"wrote {args.out} · {len(out_frames)} frames · {W}×{H}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
