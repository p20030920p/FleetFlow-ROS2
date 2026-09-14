#!/usr/bin/env python3
"""GIF 活动度验收：**必须用像素差，不能只看"帧是否不同"**。

踩过的坑（用户报告"demo 中完全卡死了"）：上一版 GIF 有 260 帧、
用 PIL 逐帧哈希验证过"259/259 帧互不相同"，看着合格 ——
但帧间平均像素差只有 **0.104 / 255**，肉眼完全静止。
原因是把 200 s 仿真压进 260 帧（每帧 0.77 s），而那段时间工厂只维持
2 个在途任务，车几乎不动。**"字节不同"和"看得出在动"是两件事。**

判据（对一张合格的 README 动图）:
  * 帧间平均像素差 中位 >= 1.0 灰阶
  * 有明显变化的帧占比 >= 80%

用法: python3 tools/check_gif.py assets/readme/demo.gif
"""
from __future__ import annotations

import sys

import numpy as np
from PIL import Image, ImageSequence

# 判据用"**明显变化的像素占比**"，而不是全局平均像素差。
# 为什么：小车在 1240×680 的图上只占约 0.05% 面积，全局平均会被大片
# 静止的底图稀释，量不出"看得见在动"。用变化像素占比才对得上肉眼感受，
# 而且可以拿**用户认可的旧版**当基准标定：
#     旧版（认可）  变化像素中位 0.480%，100% 的帧 >0.2%
#     失败版（卡死）变化像素中位 0.077%，只有 31% 的帧 >0.2%
# 阈值取在两者之间，偏保守。
PIXEL_DELTA = 6          # 单像素差超过它才算"这一像素变了"
MEDIAN_MIN = 0.30        # 变化像素占比中位数下限（%）
ALIVE_FRAC_MIN = 0.85    # "变化像素 > ALIVE_MIN_PCT" 的帧占比下限
ALIVE_MIN_PCT = 0.20     # 单帧变化像素占比超过它才算"这帧看得出在动"


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "assets/readme/demo.gif"
    im = Image.open(path)
    prev = None
    means, pcts = [], []
    for fr in ImageSequence.Iterator(im):
        a = np.asarray(fr.convert("L"), dtype=np.int16)
        if prev is not None:
            dd = np.abs(a - prev)
            means.append(float(dd.mean()))
            pcts.append(float((dd > PIXEL_DELTA).mean()) * 100.0)
        prev = a
    if not pcts:
        print("GIF 只有一帧，谈不上动画")
        return 1
    d = np.array(pcts)
    med = float(np.median(d))
    alive = float((d > ALIVE_MIN_PCT).mean())
    print(f"{path}")
    print(f"  帧数          {im.n_frames}  尺寸 {im.size}")
    print(f"  帧间平均像素差 {np.mean(means):.3f}   （参考值，不单独作判据）")
    print(f"  变化像素占比   中位 {med:.3f}%  最大 {d.max():.2f}%")
    print(f"  看得出在动     {alive:.0%} 的帧")

    ok = True
    if med < MEDIAN_MIN:
        print(f"  [FAIL] 变化像素中位 {med:.3f}% < {MEDIAN_MIN}% —— 肉眼会认为静止")
        ok = False
    if alive < ALIVE_FRAC_MIN:
        print(f"  [FAIL] 只有 {alive:.0%} 的帧有可见变化 < {ALIVE_FRAC_MIN:.0%}")
        ok = False
    if ok:
        print("  [ok] 动图活动度合格")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
