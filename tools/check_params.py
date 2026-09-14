#!/usr/bin/env python3
"""参数声明自检：每个 `get_parameter("x")` 都必须有对应的 `declare_parameter("x")`。

为什么值得单独写一个检查 —— 这个错误**不会在启动时报错**，
而是让整个节点当场崩掉，表现为"跑满 325 s、完成 0 单、里程 0 m"。
排查时它在 launch.log 里只留一行 `ParameterNotDeclaredException`，
很容易被当成"吞吐又抖了"，从而浪费一整轮实验。

真实案例（第 48 节）：给 traffic_manager 新增 `stall_horizon_s` 时只加了
`get_parameter` 与 launch 传参，漏了 `declare_parameter`，
结果 3 臂 × 300 s 的 Gazebo 配对实验**全部作废**（两臂都是 0 单）。

用法: python3 tools/check_params.py    # 有缺失则 exit 1
"""
from __future__ import annotations

import re
import sys
from pathlib import Path


def find_paths() -> tuple[Path, Path]:
    """从本文件往上找仓库根（含 src/fleetflow_sim/fleetflow_sim/planner.py）。"""
    here = Path(__file__).resolve()
    for base in [here.parent, *here.parents]:
        pkg = base / "src" / "fleetflow_sim" / "fleetflow_sim"
        if (pkg / "planner.py").is_file():
            return base, pkg
    raise SystemExit("找不到仓库根（未找到 src/fleetflow_sim/fleetflow_sim/planner.py）")


GET = re.compile(r'get_parameter\(\s*"([A-Za-z_0-9]+)"\s*\)')
DECL = re.compile(r'declare_parameter\(\s*"([A-Za-z_0-9]+)"')


def main() -> int:
    _, pkg = find_paths()
    bad = 0
    checked = 0
    for path in sorted(pkg.glob("*.py")):
        src = path.read_text(encoding="utf-8")
        gets = set(GET.findall(src))
        if not gets:
            continue
        checked += 1
        decl = set(DECL.findall(src))
        missing = sorted(gets - decl)
        if missing:
            bad += 1
            print(f"FAIL {path.name}: 读取了但未声明 -> {missing}")
        else:
            print(f"ok   {path.name}: {len(gets)} 个参数均已声明")
    if not checked:
        print("未发现任何参数读取点 —— 路径可能不对")
        return 1
    if bad:
        print(f"\n{bad} 个文件存在未声明参数 —— 这些节点会在启动时崩溃并导致整场零产出。")
        return 1
    print("\n全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
