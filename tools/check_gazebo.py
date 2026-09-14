#!/usr/bin/env python3
"""quick start 自检：把"进不去 Gazebo / 渲染不出来"拆成可判定的三关。

用户报告本地按 quick start 跑不通 Gazebo 与渲染，但在开发机上
Gazebo 服务端、GUI（`gz gui` 进程存活 120 s 且持续渲染）、带相机的
无头渲染三条路都是通的。说明问题多半在**环境**而不是代码，
可环境问题最怕"一句话报错"，所以这里逐关验证并给出**具体缺什么**：

  第 1 关  服务端   —— `gz sim -s -r --headless-rendering` 能否起来并推进仿真
  第 2 关  界面     —— `gz sim -r` 的 gui 进程能否存活（需要 DISPLAY/WAYLAND + GL）
  第 3 关  渲染     —— 相机能否出图（无头渲染，不需要显示）

用法:
    python3 tools/check_gazebo.py            # 三关全测（界面那关会开窗口）
    python3 tools/check_gazebo.py --no-gui   # 只测服务端与渲染（CI/无显示环境）

退出码 0 表示三关（或所选关卡）都通过。
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

OK, WARN, BAD = "ok  ", "warn", "FAIL"


def find_ws() -> Path:
    here = Path(__file__).resolve()
    for base in [here.parent, *here.parents]:
        if (base / "install" / "setup.bash").is_file():
            return base
    raise SystemExit("找不到工作区根（未找到 install/setup.bash，先 colcon build）")


def find_world(ws: Path) -> Path:
    hits = glob.glob(str(ws / "install" / "fleetflow_sim" / "share" /
                        "fleetflow_sim" / "worlds" / "*.sdf"))
    if not hits:
        raise SystemExit("找不到世界文件，install/share 不完整")
    return Path(sorted(hits)[0])


def run(args, env, timeout=25.0):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout,
                           env=env, errors="replace")
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired as exc:
        parts = []
        for chunk in (exc.stdout, exc.stderr):
            if chunk is None:
                continue
            parts.append(chunk.decode("utf-8", "replace")
                         if isinstance(chunk, bytes) else chunk)
        return None, "".join(parts)
    except Exception as exc:                                  # noqa: BLE001
        return 1, str(exc)


def env_for(ws: Path) -> dict:
    env = dict(os.environ)
    ros = "/opt/ros/jazzy/setup.bash"
    setup = ws / "install" / "setup.bash"
    cmd = f"source {ros} >/dev/null 2>&1; source {setup} >/dev/null 2>&1; env -0"
    p = subprocess.run(["bash", "-c", cmd], capture_output=True)
    for chunk in p.stdout.split(b"\0"):
        if b"=" in chunk:
            k, v = chunk.split(b"=", 1)
            env[k.decode()] = v.decode(errors="replace")
    return env


def check_server(ws: Path, env: dict, world: Path) -> tuple[str, str]:
    rc, out = run(["gz", "sim", "-s", "-r", "--headless-rendering", str(world)],
                  env, timeout=20)
    if rc is None:
        return OK, "服务端 20 s 内持续运行（已按超时结束）"
    tail = "\n".join(out.strip().splitlines()[-4:])
    return BAD, f"服务端提前退出 rc={rc}\n{tail}"


def check_gui(ws: Path, env: dict, world: Path, seconds: float = 25.0) -> tuple[str, str]:
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return BAD, "没有 DISPLAY / WAYLAND_DISPLAY —— 界面无法启动，改用 gui:=false"
    p = subprocess.Popen(["gz", "sim", "-r", str(world)], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, errors="replace", start_new_session=True)
    alive_at = 0.0
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(1.0)
        if p.poll() is not None:
            break
        alive_at = time.time() - (deadline - seconds)
    rc = p.poll()
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except Exception:                                         # noqa: BLE001
        pass
    try:
        tail = (p.stdout.read() or "") if p.stdout else ""
    except Exception:                                         # noqa: BLE001
        tail = ""
    tail = "\n".join([l for l in tail.strip().splitlines()[-5:]])
    if rc is None:
        return OK, f"界面进程存活 {alive_at:.0f} s 且未退出（`gz gui` 已在渲染）"
    return BAD, (f"界面进程 {alive_at:.0f} s 后退出 rc={rc}\n{tail}\n"
                 f"提示：虚拟机/容器里常见原因是拿不到硬件 GL，"
                 f"用 gui:=false web:=true 走网页看板")


def check_render(ws: Path, env: dict) -> tuple[str, str]:
    """无头渲染：不起界面，只让相机出图。这是 assets 的生成路径。"""
    script = ws / "tools" / "capture_views.py"
    if not script.is_file():
        return WARN, "tools/capture_views.py 不存在，跳过"
    if shutil.which("ros2") is None:
        return BAD, "找不到 ros2"
    return WARN, ("需要先跑 `ros2 launch fleetflow_sim factory.launch.py "
                  "bridge_cameras:=true`，再执行 "
                  "`python3 tools/capture_views.py /tmp/shots /view_iso/image`"
                  "（本自检不代跑，避免与你的终端抢进程）")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-gui", action="store_true", help="跳过界面那关")
    args = ap.parse_args()

    ws = find_ws()
    world = find_world(ws)
    env = env_for(ws)
    print(f"工作区 {ws}")
    print(f"世界文件 {world}\n")

    results = [("服务端", *check_server(ws, env, world))]
    if not args.no_gui:
        results.append(("界面", *check_gui(ws, env, world)))
    results.append(("渲染", *check_render(ws, env)))

    bad = 0
    for name, tag, msg in results:
        print(f"[{tag}] {name}")
        for line in msg.splitlines():
            print(f"        {line}")
        if tag == BAD:
            bad += 1
    print()
    if bad:
        print(f"{bad} 关未通过 —— 把上面每关的输出贴出来即可定位。")
        return 1
    print("所选关卡全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
