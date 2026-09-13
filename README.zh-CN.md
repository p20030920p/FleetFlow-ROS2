<div align="center">

# FleetFlow-ROS2

**纺织厂多 AGV 物料搬运仿真**

<sub>ROS 2 Jazzy · Gazebo Sim 8 · 每车独立命名空间与 TF · 租约式停靠 · CA-SSI 分配</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-orange)](https://gazebosim.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#快速开始)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](#快速开始)

[快速开始](#快速开始) &nbsp;•&nbsp; [车队](#车队) &nbsp;•&nbsp; [分配](#分配) &nbsp;•&nbsp; [结果](#结果) &nbsp;•&nbsp; [消融](#消融) &nbsp;•&nbsp; [稳定性](#稳定性)

*[English](README.md) &nbsp;|&nbsp; 简体中文*

</div>

![一次完整搬运循环：任务以空心方框出现，拍卖逐单派车，车辆沿规划路径行驶并绕开通道里堆放的托盘](assets/readme/demo.gif)

*一个完整循环，从发放任务到送达。实线=已行驶，虚线=剩余规划；通道里的托盘是 A\* 代价图里的真实静态障碍。*

5 台 AGV 在 26 × 16 米的车间里，于梳棉、并条、粗纱机台之间搬运条筒。调度器派单，
每台车自己规划并执行路径，生产看板显示车间状态。

| | |
|---|---|
| **分配** —— 8 台车、同一座工厂 | CA-SSI **18.7** 件/分 · SSI 15.2 · random 10.6 |
| **改进前 → 改进后** | 吞吐 **+23 %** · 单任务里程 **−16 %** · 近距事件 **−53 %** |
| **谁在扛代价模型** | 工位拥塞（去掉 **−20 %**）· 电量可达（去掉 **−16 %**） |
| **单轮最优** | `hungarian` 从不获胜 —— 14.9 对 18.7 |
| **已验证** | 协同栈，墙钟 · **Gazebo 物理尚未** —— 见[稳定性](#稳定性) |

---

## 快速开始

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch fleetflow_sim factory.launch.py                    # Gazebo（无头）
ros2 launch fleetflow_sim factory.launch.py headless:=false gui:=true
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi   # 不启 Gazebo
```

相机话题**默认不桥接**。桥接的 `sensor_msgs/Image` 一旦订阅端跟不上，缓冲会无限增长 ——
实测约 5 GB RSS，足以触发 OOM killer。相机是 `always_on=0`（有订阅才渲染），且一次只桥一路：

```bash
ros2 launch fleetflow_sim factory.launch.py bridge_cameras:=true
ros2 run ros_gz_bridge parameter_bridge "/view_iso/image@sensor_msgs/msg/Image@gz.msgs.Image" &
python3 tools/capture_views.py /tmp/shots /view_iso/image && kill %1
```

---

## 车队

世界由 [`tools/build_world.py`](tools/build_world.py) 生成：527 个模型 —— 机弄、条筒货架、
架空巡回清洁轨道、通道托盘、地面警示标线。

![车间 3/4 剖视](assets/readme/gazebo-iso.png)

*3/4 剖视：机弄、条筒货架、架空巡回清洁轨道。*

<p align="center">
  <img src="assets/readme/gazebo-top.png" width="32%" alt="俯视">
  <img src="assets/readme/gazebo-line.png" width="32%" alt="沿产线">
  <img src="assets/readme/gazebo-machine.png" width="32%" alt="机台近景">
</p>

*俯视 · 沿产线 · 机台近景。*

![控制室视图：车间平面图、车辆、规划路线与物料流转计数](assets/readme/control-center.png)

*与动图同一个渲染器 —— 既能出静图也能定时出帧，两者不会走样。*

![运行时拓扑](assets/readme/architecture.png)

*factory_manager → task_scheduler → 带命名空间的 robot_controller → Gazebo，租约服务横跨其间。*

| 关注点 | 实现 |
|---|---|
| 命名空间 / TF | 每台车一个命名空间与一个 `robot_state_publisher`；TF 是森林，不是一条被争用的链。 |
| 停靠 | 基于租约，带 TTL 回收。每台车最多持一个租约，不会"抱着一个去抢另一个"，等待发生在开阔地面 —— 不存在 *hold-and-wait*，也就没有循环等待。 |
| 避障 | 互让 + 确定性优先级；每次 A\* 前把同伴注入动态障碍层；独立的 LiDAR 急停。 |
| 电量 | 按里程耗电；低于阈值停止投标，用同一套租约排队充电。 |
| 派单 | 拉取式：车辆只在空闲时要任务，从机制上消除双重分配竞态。 |

---

## 分配

运输指派是多机器人任务分配问题。只认空驶距离在棉纺车间里是错的：机台只有**一个**对接位、
电量是**硬**约束、此刻最便宜的车不等于一个班次里最便宜的车队。

| 策略 | 代价 |
|---|---|
| `random` | 无 —— 下界 |
| `nearest` | 取自己最近的任务，仅局部 |
| `ssi` | 以空驶距离为代价的顺序单件拍卖 |
| **`ca_ssi`** | **同一套拍卖，六项工业代价** |
| `hungarian` | 对该代价矩阵求线性分配最优 |

![CA-SSI 的六项代价与权重](assets/readme/cost-model.png)

*权重单位是等效米，渲染时从 `policies.py` 读取。*

```
J = α‖p_r − s_t‖ + β‖s_t − g_t‖ + γ(n_src + 1.5 n_dst)
  + δ·max(0, e_need + reserve − e_r) + η(d_r − d̄)/d_max − ζ·min(age, 20) + 0.02·prio
```

绕开一个被占用的对接位，等价于多开 `γ/α = 6` 米。老化项必须封顶；不封顶它会压过其余所有项，
拍卖退化成 FIFO。

---

## 结果

5 策略 × 3 种子 × 120 秒，在制 80 件物料，工厂完全相同 —— 只有车队规模不同。

![两个车队规模下的吞吐、时延、单任务里程与利用率](assets/readme/policy-comparison.png)

*误差棒 1σ；百分比相对 `random`。*

| 策略 | 3 台 件/分 | 3 台 米/单 | 8 台 件/分 | 8 台 米/单 | 8 台 近距事件 |
|---|---:|---:|---:|---:|---:|
| random | 9.0 | 12.2 | 10.6 | 13.8 | 6.7 |
| nearest | 10.4 | 9.8 | 12.3 | 11.7 | 4.3 |
| ssi | 10.0 | 10.1 | 15.2 | 12.9 | 6.3 |
| **ca_ssi** | **10.7** | **9.1** | **18.7** | **10.8** | **3.0** |
| hungarian | 9.9 | 10.4 | 14.9 | 11.9 | 6.5 |

**改进前 → 改进后。** 同一份代码，只改了 `ca_ssi_cost`。

| 指标 | 车队 | SSI | CA-SSI | 变化 | 相对 `random` |
|---|---|---:|---:|---:|---:|
| 吞吐 | 3 台 | 10.0 | 10.7 | **+7 %** | +18 % |
| 吞吐 | 8 台 | 15.2 | 18.7 | **+23 %** | +77 % |
| 每单里程 | 3 台 | 10.1 | 9.1 | **−10 %** | −25 % |
| 每单里程 | 8 台 | 12.9 | 10.8 | **−16 %** | −22 % |
| 近距事件 | 8 台 | 6.3 | 3.0 | **−53 %** | −55 % |

优势随竞争强度增长：3 台车很少争同一个单泊位工位，8 台车是常态。`hungarian` 优化一张静态矩阵，
看不见拥塞、电量与排队 —— 最优的分配方案不等于最优的系统。在制物料只有 28 件时工厂"缺料饿死"，
所有策略收敛到约 16 件/分钟，用什么规则根本测不出来。

---

## 消融

六项代价逐项置零；同一套拍卖、同一组种子、同一座工厂。

![CA-SSI 六项代价的留一法消融](assets/readme/ablation.png)

*8 台车、3 种子、80 件物料。*

| 代价模型 | 件/分 | 相对完整 | 米/单 |
|---|---:|---:|---:|
| **CA-SSI（六项齐全）** | **17.45** | — | **11.3** |
| − 工位拥塞（γ） | 13.92 | **−20 %** | 12.6 |
| − 电量可达（δ） | 14.63 | **−16 %** | 12.3 |
| − 负载均衡（η） | 17.29 | −1 % | 11.6 |
| − 任务老化（ζ） | 17.32 | −1 % | 10.9 |

真正扛事的是两项。负载均衡与任务老化在这个工况下是**零结果** —— 120 秒太短攒不出里程优势，
而防饿死项的职责是兜住最坏情况、不是抬高平均值。两项都保留，但都不声称在这组实验里回了本。

---

## 稳定性

| 机制 | 作用 |
|---|---|
| 租约 + TTL | 一个工位一个持有者；不存在 hold-and-wait；持有者失联后回收。 |
| 看门狗 | 10 秒无位移 → 重规划；第二次 → 低速爬行；第三次 → 放弃并重新入队。 |
| 靠泊模式 | 距目标 0.8 m 内急停阈值降到 0.12 m —— 急停距离永远不能大于到达判定。 |
| 车体轮廓屏蔽 | 逐角度把回波与车体自身轮廓比较；否则两个前角正好落在 0.24 m 的急停扇区里。 |
| 按需重规划 | 只在同伴占住前方 2.2 m 路径时才重算 —— 定时重算会打断转弯，车在原地来回摆。 |

4 台 AGV、200 秒墙钟：**35 次派单、5 次送达、0 报错、0 次看门狗中止、0 次"无路径"，
空闲内存不低于 6.9 GB。**

**尚不可信的部分。** Gazebo 下底盘不复现指令运动：`cmd_vel` 持续 0.5 m/s 下达 15 秒，
只前进 0.17 m（理论 7.5 m）。协同层指令是正确的（93 % 的速度指令非零、均值 0.30 m/s），
所以问题在轮地接触模型上。这也是计时实验走 `logic_only.launch.py` 的原因 ——
协同栈完全相同，只是墙钟准确。

---

## 复现

```bash
# 三组实验
python3 tools/run_experiments.py --policies random nearest ssi ca_ssi hungarian \
    --seeds 1 2 3 --seconds 120 --robots 3 --drain 0.10 --out experiments/saturated \
    --extra max_tasks_in_flight:=18 num_materials:=80
python3 tools/run_experiments.py --policies random nearest ssi ca_ssi hungarian \
    --seeds 1 2 3 --seconds 120 --robots 8 --drain 0.10 --out experiments/slack \
    --extra max_tasks_in_flight:=18 num_materials:=80
python3 tools/run_experiments.py \
    --policies ca_ssi ca_nocong ca_noener ca_nobal ca_noage \
    --seeds 1 2 3 --seconds 120 --robots 8 --drain 0.10 --out experiments/ablation \
    --extra max_tasks_in_flight:=18 num_materials:=80

# 出图
python3 tools/plot_results.py experiments/saturated/summary.csv \
    experiments/slack/summary.csv assets/readme/policy-comparison.png
python3 tools/plot_ablation.py experiments/ablation/summary.csv assets/readme/ablation.png
python3 tools/plot_cost_model.py assets/readme/cost-model.png
python3 tools/plot_architecture.py assets/readme/architecture.png

# 动图分两步：先录一次运行，再离屏重画
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=6 policy:=ca_ssi
python3 tools/record_run.py /tmp/run.jsonl --fps 10 --seconds 95 --min-robots 6
python3 tools/render_demo.py /tmp/run.jsonl assets/readme/demo.gif --speed 6
```

| 指标 | 定义 |
|---|---|
| makespan | 最后一次送达 − 第一次派单 |
| 吞吐 | 每分钟完成任务数 |
| 时延 | 送达 − 生成，均值 / p50 / p95 |
| 利用率 | 处于"行驶或装卸"的时间占比 |
| 近距事件 | 任意两车距离小于 0.55 m 的上升沿次数 |

---

## 接口

| 接口 | 类型 | |
|---|---|---|
| `/factory/tasks` | `TransportTask` | 新任务，周期性重播 |
| `/factory/task_status` | `TransportTask` | 派发、完成、失败 |
| `/factory/machines` | `MachineState` | 状态、位置、进出料数、忙闲比 |
| `/fleet/robots` | `RobotStatus` | 位姿、状态、电量、当前任务 |
| `/scheduler/request_task` | srv | 车辆拉取下一个任务 |
| `/traffic/acquire`, `/traffic/release` | srv | 工位与充电桩租约 |
| `/robot_N/path` | `nav_msgs/Path` | 剩余规划路径 |

## 目录结构

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/       # 消息与服务定义
├── src/fleetflow_sim/
│   ├── fleetflow_sim/              # layout · planner · factory_manager
│   │                               # task_scheduler · traffic_manager
│   │                               # robot_controller · policies · dashboard
│   ├── worlds/textile_factory.sdf  # 由脚本生成
│   ├── urdf/agv.urdf.xacro
│   └── launch/
├── tools/                          # build_world · run_experiments · record_run
│                                   # render_demo · plot_* · capture_views
└── experiments/                    # 原始 CSV：saturated/ slack/ ablation/
```

## 边界

- `ca_ssi` 给"它看得见的"拥塞定价，看不见"下一次派单会造成的"拥塞；一次只派一单，不规划多单路线。
- 拉取式派单是刻意的：push 模型必须维护每台车的忙碌状态，一次完成通知丢失就会让那台车永久"忙碌"。
- 电量参数调成"一次短运行内能观察到充电"，是行为演示，不是能耗研究。
