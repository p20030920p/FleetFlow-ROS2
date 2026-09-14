#!/usr/bin/env python3
"""起仿真前的体检：把"打不开/闪退"这类问题在启动前就暴露出来。

Gazebo 界面打不开的原因里，绝大多数和本项目代码无关，而是环境问题。
逐条检查比对着黑窗口猜快得多：

  1. ROS 与工作区是否已 source —— 否则 ros2 / fleetflow_sim 都找不到；
  2. 是否有**孤儿 Gazebo 进程** —— 上一轮被 kill -9 后服务端会留下来，
     新的界面会连到那个旧服务端上，表现是窗口打开但什么都不渲染；
  3. 显示环境 —— 没有 DISPLAY / WAYLAND_DISPLAY 时起 GUI 必然失败，
     这种情况应该用 gui:=false（无头服务端）配合网页看板；
  4. 渲染后端 —— 打印 OpenGL renderer 与 Ogre 相关警告。
     在虚拟机 / 容器 / 远程桌面里常见的是拿不到硬件 GL，
     Ogre 起不来就会直接退出（"force quit"）。
  5. 显卡设备是否存在。

用法: python3 tools/preflight.py
"""
from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys

OK, WARN, BAD = "ok  ", "warn", "FAIL"


def _run(cmd: list[str], timeout: float = 8.0) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except Exception as exc:                                  # noqa: BLE001
        return 1, str(exc)


def check_ros() -> list[tuple[str, str, str]]:
    out = []
    if shutil.which("ros2"):
        rc, txt = _run(["ros2", "pkg", "prefix", "fleetflow_sim"])
        if rc == 0:
            out.append((OK, "工作区", txt.strip().splitlines()[-1]))
        else:
            out.append((BAD, "工作区", "找不到 fleetflow_sim —— 先 source install/setup.bash"))
    else:
        out.append((BAD, "ROS", "找不到 ros2 —— 先 source /opt/ros/jazzy/setup.bash"))
    return out


def check_orphans() -> list[tuple[str, str, str]]:
    rc, txt = _run(["ps", "-eo", "args", "--no-headers"])
    lines = [l for l in txt.splitlines()
             if ("gz sim" in l or "sim server" in l or "sim gui" in l)
             and "grep" not in l]
    nodes = [l for l in txt.splitlines()
             if any(k in l for k in ("factory_manager", "robot_controller",
                                     "traffic_manager", "task_scheduler"))]
    res = []
    if lines or nodes:
        res.append((BAD, "残留进程",
                    f"Gazebo {len(lines)} 个 / 仿真节点 {len(nodes)} 个 "
                    f"—— 先跑 bash tools/gz_reset.sh"))
    else:
        res.append((OK, "残留进程", "无"))
    return res


def check_display() -> list[tuple[str, str, str]]:
    res = []
    disp = os.environ.get("DISPLAY")
    wl = os.environ.get("WAYLAND_DISPLAY")
    if disp:
        res.append((OK, "DISPLAY", disp))
    elif wl:
        res.append((OK, "WAYLAND_DISPLAY", wl))
    else:
        res.append((BAD, "显示环境",
                    "DISPLAY 与 WAYLAND_DISPLAY 都没有 —— gui:=true 必然失败；"
                    "改用 gui:=false web:=true 走网页看板"))
    if not shutil.which("glxinfo"):
        res.append((WARN, "glxinfo", "未安装（想查渲染后端可 apt install mesa-utils）"))
    return res


def check_gpu() -> list[tuple[str, str, str]]:
    res = []
    devs = sorted(glob.glob("/dev/dri/*"))
    if devs:
        res.append((OK, "显卡设备", ", ".join(devs)))
    else:
        res.append((WARN, "显卡设备",
                    "没有 /dev/dri/* —— 软件渲染下 Ogre 可能起不来"))
    rc, txt = _run(["glxinfo", "-B"]) if shutil.which("glxinfo") else (1, "")
    if rc == 0:
        for key in ("OpenGL renderer string", "OpenGL version string"):
            for line in txt.splitlines():
                if key in line:
                    val = line.split(":", 1)[1].strip()
                    tag = WARN if "llvmpipe" in val.lower() or "swrast" in val.lower() else OK
                    res.append((tag, key.replace(" string", ""), val))
                    if tag == WARN:
                        res.append((WARN, "渲染后端",
                                    "软件渲染（llvmpipe/swrast）—— Gazebo 界面容易崩；"
                                    "优先用 gui:=false + web:=true"))
                    break
    return res


def main() -> int:
    print("=== 仿真启动前体检 ===\n")
    checks: list[tuple[str, str, str]] = []
    checks += check_ros()
    checks += check_orphans()
    checks += check_display()
    checks += check_gpu()
    width = max(len(n) for _, n, _ in checks)
    bad = 0
    for tag, name, detail in checks:
        print(f"[{tag}] {name:<{width}}  {detail}")
        if tag == BAD:
            bad += 1
    print()
    if bad:
        print(f"{bad} 项需要先处理。")
        return 1
    print("未发现阻塞项。若界面仍闪退，请把终端里 Ogre/EGL 那几行贴出来。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
