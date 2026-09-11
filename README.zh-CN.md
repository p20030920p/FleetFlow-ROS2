<p align="right">
  <a href="./README.md">English</a> · <strong>简体中文</strong>
</p>

# FleetFlow-ROS2

**纺织厂多 AGV 物料搬运仿真** · ROS 2 Jazzy + Gazebo Sim 8

<p align="center">
  <img src="./assets/readme/control-center.png" width="100%" alt="FleetFlow 控制中心：左侧实时统计，右侧工厂地图，含 AGV、调度路线与停靠位">
</p>

多台 AGV 在梳棉、拉伸、粗纱机器之间搬运物料桶。优先级调度器派发运输任务，每台车自己规划路径并执行，控制中心实时显示车队状态、机器利用率与各工序进度。

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo%20Sim-8-orange" alt="Gazebo Sim 8">
  <img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white" alt="Python 3.12">
</p>

## 效果图

<p align="center">
  <img src="./assets/readme/gazebo-iso.png" width="49%" alt="Gazebo 工厂的 3/4 视角，AGV 在机器组之间搬运物料">
  <img src="./assets/readme/gazebo-line.png" width="49%" alt="沿产线的低角度视角，两台 AGV 与梳棉机">
</p>

## 为什么这是真的多机系统，而不是"多机演示"

在同一个 Gazebo 世界里生成几个模型是简单的部分。让一支车队表现得像车队，靠的是它周围的一切——这些在这里都有实现：

| 关注点 | 实际实现 |
| --- | --- |
| **命名空间与 TF** | 每台车跑在自己的命名空间下，配独立的 `robot_state_publisher` 与 `frame_prefix`，于是 TF 是一片互不干扰的森林（`robot_0/odom → robot_0/base_footprint → robot_0/base_link → …`），而不是一条互相争夺的链。 |
| **与数据匹配的 QoS** | 传感器流（scan、里程计）用 best-effort + 浅队列，慢订阅者拖不住发布端；指令、任务与车队状态用 reliable + 较深队列。 |
| **交通管理** | 停靠由显式的租约服务串行化：进入工位前必须持有租约，物理驶离后才释放。 |
| **结构性无死锁** | 每台车**同时最多持有一个**工位租约，且不会"抱着一个去抢另一个"，等待都发生在开阔地面上。这直接破坏了死锁的 *hold-and-wait* 条件，循环等待无法形成。租约还带 TTL，车辆异常退出后工位会自动回收。 |
| **消除单点瓶颈** | 每个料区不是只有一个共享坐标，而是在圆周上划出多个**停靠位**。没有它时所有车都往同一个点挤，互让逻辑会互相锁死、看门狗只能中止任务——这正是促使这次重构的失败模式。 |
| **车车互相避让** | 每台车都能看到同伴位姿并作出让步：进入前向锥形区减速、对高优先级车辆停车、并在硬安全距离处无条件停住。优先级是确定性的（id 小者先行），用来打破对称僵局。 |
| **把同伴纳入规划** | 每次调用 A\* 之前，其他车辆会作为动态障碍层注入，路径直接绕开交通，而不是只靠局部反应。 |
| **LiDAR 安全层** | 独立监视前向扇区，紧急半径内出现任何东西都直接停车，不受规划器意愿影响。 |
| **电量与充电** | 电量按行驶里程消耗。低于阈值时车辆停止接单，通过与工位同一套租约机制排队进入充电位，充满后归队。 |
| **看门狗与任务回收** | 长时间无位移的车辆先重规划，再放弃任务；调度器另外回收"停止上报"车辆手里的任务。两者都计入 reassignment。 |
| **拉取式分配** | 车辆只在空闲时主动要任务，从机制上消除 push 模型的双重分配竞态。 |

## 把分配问题写清楚

把运输任务指派给车辆，是多机器人任务分配（**MRTA**）问题的一个实例。
设 `T` 为待办任务集合、`R` 为空闲车辆集合，定义边际代价

```
c(r, t) = ‖ p_r − s_t ‖₂          # 车辆位姿到任务取货点的行驶代价
```

我们要找一个分配方案，使某个车队目标（总行驶距离 / 平均任务时延 / makespan）最小。

这里实现了三种策略，好让"选哪种"变成可测量的结论而不是假设：

| 策略 | 规则 | 使用的信息 |
| --- | --- | --- |
| `random` | 请求车辆随机取一个待办任务 | 无 —— 作为下界基线 |
| `nearest` | 请求车辆取离自己代价最小的任务 | 仅局部（完全去中心化） |
| `ssi` | 顺序单件拍卖：反复把全局代价最小的 `(r, t)` 组合判给中标者 | 全局位姿与任务集 |

`ssi` 属于市场拍卖类 MRTA 方法（single-item auction，Lagoudakis et al., 2005）。
它本身是集中式的，但**任务的交付仍然是拉取式**：调度器算好派单表，每台车来问时取走自己那一行，
因此保留了 pull 模型"不会两台车抢同一单"的性质。

## 车队是怎么被度量的

每次运行都会增量写出两个 CSV（`tasks.csv`、`run.csv`），中途被 kill 也不会丢数据。

| 指标 | 定义 |
| --- | --- |
| **makespan** | 最后一次送达 − 第一次派单 |
| **吞吐** | 每分钟完成任务数 |
| **任务时延** | 送达 − 生成，报告均值 / p50 / p95 |
| **车队利用率** | 每台车处于"行驶或装卸"的时间占比 |
| **总行驶里程** | 里程计位移求和 |
| **近距事件** | 任意两车距离小于 0.55 m 的上升沿次数 |
| **最小车距** | 任意两车最近距离 —— 安全性的直接证据 |
| **租约拒绝 / 过期** | 交通竞争压力与 TTL 回收次数 |
| **reassignment** | 因卡死或失联被回收的任务数 |

## 实验结果

每个工况下每种策略跑 3 轮，8 台 AGV，每轮 100 秒，随机种子固定，并**关掉充电干扰**
（把耗电调低，避免它混淆分配策略这一个变量）。

**工况 A —— 浅任务池**（12 件物料，同时在途任务上限 6）

| 策略 | 完成数 | 吞吐/分 | 时延 均值 / p95（秒） | 每单行驶（米） |
| --- | ---: | ---: | ---: | ---: |
| random | 27.0 | 14.8 | 15.8 / 21.5 | 10.3 |
| nearest | 29.7 | 16.3 | 15.7 / 21.6 | 9.7 |
| ssi | 27.0 | 14.8 | 15.7 / 21.4 | 9.7 |

**工况 B —— 深任务池**（28 件物料，同时在途任务上限 18）

| 策略 | 完成数 | 吞吐/分 | 时延 均值 / p95（秒） | 每单行驶（米） |
| --- | ---: | ---: | ---: | ---: |
| random | 21.7 | 13.1 | 16.9 / 22.3 | 10.2 |
| nearest | 21.0 | 12.7 | 16.6 / 22.2 | 10.3 |
| **ssi** | **29.7** | **17.9** | **16.2 / 21.0** | **10.0** |

<p align="center">
  <img src="./assets/readme/policy-comparison.png" width="100%" alt="三种分配策略在浅任务池与深任务池下的吞吐、时延、每单行驶距离与近距事件对比">
</p>

### 这些数字说明什么

1. **浅任务池下，用什么策略无所谓。** 待办任务很少多于空闲车辆，车通常根本没得挑 ——
   14.8 / 16.3 / 14.8 单每分钟的差异在运行间噪声之内。
2. **深任务池下拍卖明显领先：吞吐高出 37%。** 更有意思的是 *为什么* `nearest` 并不比 `random` 好：
   它是"谁先来问就给谁最近的单"，于是一台车可能抢走本该属于另一台近得多的车的任务。
   选择少时这很少发生，选择多时它一直在发生。SSI 通过"同时对所有空闲车与所有待办任务打分"消除了这个效应。
3. **时延在不同策略间基本持平**（16–17 秒），因为它主要由机器加工时间决定而不是行驶时间。
   分配策略体现在**吞吐**上，而不是体现在"单件任务要多久"。
4. **每一轮的安全指标都守住了。** 最近车距从未低于 0.34 米的硬安全距离，近距事件保持在每 100 秒个位数。

### 如实的局限

- 每个工况 3 个种子，足以分辨 37% 的差距，但不足以分辨 10% 以内的差异 ——
  工况 A 里的排序应当视为噪声。
- `ssi` 是集中式的，前提是调度器手里的车辆位姿足够新；如果基于过期位置做拍卖，这部分优势会打折。
- 工况 A 里真正的约束是**工作量而不是车队规模**：车队利用率约 0.67，说明有四分之一的运力在等活干。

## 快速开始

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 完整仿真（Gazebo 无头运行，每 3 秒输出一帧控制中心图）
ros2 launch fleetflow_sim factory.launch.py

# 想开 Gazebo 界面
ros2 launch fleetflow_sim factory.launch.py headless:=false gui:=true

# 只跑调度逻辑：不启 Gazebo、不需要 GPU。CI 和下面的实验用的就是它
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi
```

Gazebo 起来之后抓三个机位：

```bash
python3 tools/capture_views.py /tmp/shots /view_iso/image /view_top/image /view_line/image
```

## 复现这组对比实验

```bash
# 工况 A —— 浅任务池
python3 tools/run_experiments.py --policies random nearest ssi --seeds 1 2 3 \
    --seconds 100 --robots 8 --drain 0.10 --out experiments/shallow

# 工况 B —— 深任务池
python3 tools/run_experiments.py --policies random nearest ssi --seeds 1 2 3 \
    --seconds 100 --robots 8 --drain 0.10 --out experiments/deep \
    --extra max_tasks_in_flight:=18 num_materials:=28

python3 tools/plot_results.py experiments/shallow/summary.csv \
    experiments/deep/summary.csv assets/readme/policy-comparison.png
```

每个 `(策略, 种子)` 组合在独立目录里跑一次，随机种子固定，因此策略本身与平局打破都可复现。
上表背后的原始 CSV 已提交在 `experiments/` 下，不重跑也能核对数字。

## 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/factory/tasks` | `TransportTask` | 新的搬运任务；周期性重播，晚启动的订阅者也不会漏 |
| `/factory/task_status` | `TransportTask` | 分配、完成与失败 |
| `/factory/completed` | `Int32` | 送达回执 |
| `/factory/machines` | `MachineState` | 状态、位置、进出料计数、繁忙率 |
| `/factory/summary` | `String`（JSON） | 控制中心渲染用的汇总 |
| `/fleet/robots` | `RobotStatus` | 每台车的位姿、状态与当前任务 |
| `/scheduler/request_task` | `RequestTask` | 车辆主动拉取下一个任务 |
| `/traffic/acquire`、`/traffic/release` | `AcquireLease` / `ReleaseLease` | 工位与充电桩租约 |
| `/robot_N/cmd_vel`、`/robot_N/odom`、`/robot_N/scan`、`/robot_N/joint_states` | ROS 2 ↔ Gazebo | 按车辆桥接并加命名空间 |

## 目录结构

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/   # 消息与服务定义
├── src/fleetflow_sim/
│   ├── fleetflow_sim/
│   │   ├── layout.py           # 共享厂房几何、停靠位、充电桩
│   │   ├── planner.py          # 栅格 A*、视线拉直、纯追踪、动态障碍层
│   │   ├── factory_manager.py  # 物料与机器模型、任务生成
│   │   ├── task_scheduler.py   # 分配策略 + 心跳看门狗
│   │   ├── traffic_manager.py  # 工位租约，结构性无 hold-and-wait
│   │   ├── robot_controller.py # 单车状态机、TF、避让、电量
│   │   ├── metrics.py          # 实验用 CSV 指标
│   │   ├── policies.py         # random / nearest / SSI
│   │   ├── qos.py              # 按数据类别划分的 QoS
│   │   └── dashboard.py        # 控制中心渲染
│   ├── worlds/textile_factory.sdf
│   ├── urdf/agv.urdf.xacro
│   └── launch/                 # factory.launch.py · logic_only.launch.py
├── tools/                      # capture_views · run_experiments · plot_results
└── experiments/                # 上面那组对比的原始 CSV
```

## 说明与边界

- 调度刻意用 pull 模型。push 模型必须维护每台车的忙碌状态，一旦完成通知丢失，那台车就永久"忙碌"——
  这正是促使这次重构的 bug。pull 让这种失效模式不可能发生，剩余情况（车辆彻底不响应）由心跳看门狗兜住。
- 分配是短视的：代价只算"到取货点的行驶距离"，既不看未来任务，也不看拥堵。加入拥堵感知代价、
  或带排序的拍卖，是下一步最自然的方向。
- `logic_only.launch.py` 不启 Gazebo 就能跑完整协同栈，上面那组策略对比就是这样在可接受的时间内复现的。
  该模式下里程计由本地积分得到，但协同逻辑与仿真模式完全一致。
- 电量参数调成了"一次短运行内即可观察到充电行为"。真实 AGV 单次充电能跑远得多：
  这里的电量模型是行为演示，不是能耗研究。

## 关于这些数字的来处

两种启动模式下的协同栈是完全相同的，但上面的测量都是用 `logic_only.launch.py` 得到的。
在没有独立显卡的机器上，Gazebo 的渲染会把实时因子压到 1 以下，于是每一个墙钟指标 ——
吞吐、时延、看门狗超时 —— 都会被一个与分配策略无关的系数缩放。
把对比实验放到无渲染模式下跑，才能保证"被测变量"是唯一的变量。
而 Gazebo 仍然是所有物理相关部分的基准：接触、里程计、LiDAR，以及本页顶部的那些截图。
