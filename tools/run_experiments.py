#!/usr/bin/env python3
"""可复现的分配策略对比实验。

跑法（在已 source 好工作空间的终端里）：

    python3 tools/run_experiments.py --policies random nearest ssi \
        --seeds 1 2 3 --seconds 100 --robots 8 --out experiments

它会为每个 (策略, 随机种子) 组合启动一次 ``logic_only`` 运行，收集
``<out>/<policy>_s<seed>/metrics/{tasks,run}.csv``，最后汇总成
``<out>/summary.csv``（每组合一行）并打印均值表。

设计取舍
--------
* 用纯逻辑模式而不是 Gazebo：调度与交通逻辑完全一致，但省掉了渲染开销，
  单轮从分钟级降到一分多钟，才能做多组重复实验。
* 固定 ``--seed``：random 策略的随机性、以及平局打破都可复现。
* 每组独立目录：即使中途失败也不会污染其他样本。
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time

# 字段表**从 metrics 导入**，不再手抄一份。
#
# 手抄过两次，两次都漏了：metrics.RUN_FIELDS 一加新列，这里的副本没跟上，
# 汇总时就报 `ValueError: dict contains fields not in fieldnames:
# 'dup_target_ticks', 'dup_target_events'` —— 而且是在**所有运行都跑完之后**
# 才炸，白等一整轮。直接从源头导入，这个漂移就不可能再发生。
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "fleetflow_sim"))
try:
    from fleetflow_sim.metrics import RUN_FIELDS            # noqa: E402
except Exception as _exc:                                   # noqa: BLE001
    # 没 source 工作区时 fleetflow_interfaces 导入不了。这时宁可**直接报错**，
    # 也不要退回一份手抄的字段表 —— 那正是导致本轮白跑一整轮的原因。
    raise SystemExit(
        "无法从 fleetflow_sim.metrics 导入 RUN_FIELDS（" + str(_exc) + "）。\n"
        "请先 source /opt/ros/jazzy/setup.bash 与本工作区的 install/setup.bash。")


def run_once(policy: str, seed: int, seconds: int, robots: int, out_root: str,
             drain: float, extra: list) -> dict | None:
    out = os.path.join(out_root, f"{policy}_s{seed}")
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out, exist_ok=True)
    cmd = [
        "ros2", "launch", "fleetflow_sim", "logic_only.launch.py",
        f"num_robots:={robots}", f"policy:={policy}", f"seed:={seed}",
        f"run_label:={policy}_s{seed}", f"metrics_dir:={out}/metrics",
        f"out_dir:={out}/frames", "frame_every:=0.0", "dashboard:=false",
        f"battery_drain:={drain}",
    ] + extra
    log = open(os.path.join(out, "launch.log"), "w")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    print(f"  [{policy} seed={seed}] running {seconds}s ...", flush=True)
    time.sleep(seconds)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)   # 让节点走正常退出路径，落盘指标
    except ProcessLookupError:
        pass
    time.sleep(6)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    log.close()

    run_csv = os.path.join(out, "metrics", "run.csv")
    if not os.path.exists(run_csv):
        print(f"  [{policy} seed={seed}] no metrics produced", flush=True)
        return None
    with open(run_csv, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else None


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policies", nargs="+", default=["random", "nearest", "ssi"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    ap.add_argument("--seconds", type=int, default=100)
    ap.add_argument("--robots", type=int, default=8)
    ap.add_argument("--drain", type=float, default=0.55, help="电量消耗 %/m")
    ap.add_argument("--out", default="experiments")
    ap.add_argument("--extra", nargs="*", default=[], help="额外的 launch 参数")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    results = []
    print(f"experiments: {len(args.policies)} policies x {len(args.seeds)} seeds "
          f"x {args.seconds}s x {args.robots} robots")
    for policy in args.policies:
        for seed in args.seeds:
            row = run_once(policy, seed, args.seconds, args.robots, args.out,
                           args.drain, args.extra)
            if row:
                results.append(row)

    if not results:
        print("no results")
        return 1
    summary = os.path.join(args.out, "summary.csv")
    with open(summary, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=RUN_FIELDS)
        w.writeheader()
        w.writerows(results)

    print(f"\nwrote {summary}\n")
    metrics = ["completed", "throughput_per_min", "latency_mean_s", "latency_p95_s",
               "makespan_s", "utilisation_mean", "distance_total_m", "near_miss_events",
               "reassignments"]
    print(f"{'policy':<9} " + " ".join(f"{m[:11]:>12}" for m in metrics))
    for policy in args.policies:
        rows = [r for r in results if r["policy"] == policy]
        if not rows:
            continue
        cells = []
        for m in metrics:
            vals = [to_float(r.get(m)) for r in rows]
            vals = [v for v in vals if v is not None]
            cells.append(f"{statistics.fmean(vals):12.2f}" if vals else f"{'-':>12}")
        print(f"{policy:<9} " + " ".join(cells))
    return 0


if __name__ == "__main__":
    sys.exit(main())
