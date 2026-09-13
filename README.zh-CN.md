<p align="right">
  <a href="./README.md">English</a> · <strong>简体中文</strong>
</p>

<h1 align="center">FleetFlow-ROS2</h1>

<p align="center"><b>纺织厂多 AGV 物料搬运仿真</b><br>
ROS 2 Jazzy · Gazebo Sim 8 · Ubuntu 24.04</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo%20Sim-8-orange" alt="Gazebo Sim 8">
  <img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white" alt="Python 3.12">
</p>

<p align="center">
  <img src="./assets/readme/demo.gif" width="100%" alt="Gazebo 中录制的完整搬运循环：任务以空心方框出现，拍卖逐单派车，车辆沿规划路径行驶并绕开通道里堆放的托盘，完成数与物料流转计数同步推进">
</p>

<p align="center">
  <sub><b>Gazebo 实录，从发放任务到送达。</b>实线=已行驶，虚线=剩余规划；
  车辆绕开停在通道里的托盘。</sub>
</p>

5 台 AGV 在 26 × 16 米的车间里，于梳棉、并条、粗纱机台之间搬运条筒。调度器派单，
每台车自己规划并执行路径，生产看板显示车间状态。

## 快速开始

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch fleetflow_sim factory.launch.py                    # Gazebo（无头）
ros2 launch fleetflow_sim factory.launch.py headless:=false gui:=true
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi   # 不启 Gazebo
```

相机话题**默认不桥接**：`sensor_msgs/Image` 的订阅端一旦跟不上，桥接端缓冲会无限增长
（实测约 5 GB RSS，足以触发 OOM killer）。相机是 `always_on=0`（有订阅者才渲染），且一次只桥一路：

```bash
ros2 launch fleetflow_sim factory.launch.py bridge_cameras:=true
ros2 run ros_gz_bridge parameter_bridge "/view_iso/image@sensor_msgs/msg/Image@gz.msgs.Image" &
python3 tools/capture_views.py /tmp/shots /view_iso/image && kill %1
```

## 车间

由 [`tools/build_world.py`](./tools/build_world.py) 生成 —— 534 个模型：机弄、条筒货架、
架空巡回清洁轨道、通道里堆放的托盘、地面警示标线。

<p align="center">
  <img src="./assets/readme/gazebo-iso.png" width="49%" alt="车间 3/4 剖视">
  <img src="./assets/readme/gazebo-top.png" width="49%" alt="车间俯视">
  <img src="./assets/readme/gazebo-line.png" width="49%" alt="沿产线的低角度视角">
  <img src="./assets/readme/gazebo-machine.png" width="49%" alt="梳棉机近景">
</p>

## 控制室

<p align="center">
  <img src="./assets/readme/control-center.png" width="100%" alt="控制室视图：车间平面图、车辆、规划路线与物料流转计数">
</p>

与上面动图用的是同一个渲染器 —— 它既能出静图也能定时出帧，所以这张图和本页不会走样。

## 实际跑起来的东西

<p align="center">
  <img src="./assets/readme/architecture.png" width="100%" alt="运行时拓扑">
</p>

| 关注点 | 实现 |
| --- | --- |
| **命名空间 / TF** | 每台车一个命名空间与一个 `robot_state_publisher`，TF 是互不相交的森林，而不是一条被争用的链。 |
| **按数据类别分 QoS** | scan 与里程计用 best-effort + 浅队列；指令、任务与车队状态用 reliable + 较深队列。 |
| **停靠** | 基于租约。每台车最多持一个租约，不会"抱着一个去抢另一个"，等待发生在开阔地面 —— 这破坏了死锁的 *hold-and-wait* 条件。租约带 TTL，可回收。 |
| **多停靠位** | 每个料区在圆周上划出多个停靠位而不是一个共享点；否则所有车往同一点挤，互让逻辑锁死，看门狗只能中止任务。 |
| **避障** | 互让（前向锥形区减速、对高优先级车停车、硬安全距离停车）、确定性优先级打破对称僵局、每次 A\* 前把同伴注入动态障碍层、独立的 LiDAR 急停。 |
| **电量与恢复** | 按里程耗电；低于阈值停止投标，用同一套租约排队充电后归队。卡死车辆先重规划再放弃；调度器回收失联车辆的任务。 |
| **拉取式分配** | 车辆只在空闲时要任务，从机制上消除 push 模型的双重分配竞态。 |

## 分配问题

把运输任务指派给车辆，是多机器人任务分配问题。教科书代价只有一个空驶距离
`c(r,t) = ‖p_r − s_t‖₂`，它在棉纺车间里错在三处：机台只有**一个**对接位；电量是**硬**约束；
此刻最便宜的车不等于一个班次里最便宜的车队。

| 策略 | 规则 | 使用的信息 |
| --- | --- | --- |
| `random` | 随机取一个待办任务 | 无 —— 下界 |
| `nearest` | 取自己最近的任务 | 仅局部 |
| `ssi` | 以空驶距离为代价的顺序单件拍卖 | 全局位姿 + 任务集 |
| **`ca_ssi`** | **同一套拍卖，六项工业代价** | **+ 拥塞、电量、负载、老化** |
| `hungarian` | 对该代价矩阵求线性分配最优 | 全局、单轮 |

`ca_ssi` **不动拍卖机制**，只换代价函数 —— 所以对比隔离的是代价模型，而不是算法类别。

<p align="center">
  <img src="./assets/readme/cost-model.png" width="100%" alt="CA-SSI 代价函数的六项构成及其权重">
</p>

```
J(r,t) = α·‖p_r − s_t‖                    ① 空驶
       + β·‖s_t − g_t‖                    ② 载货行驶
       + γ·(n_src + 1.5·n_dst)            ③ 工位拥塞
       + δ·max(0, e_need + reserve − e_r) ④ 电量可达性
       + η·(d_r − d̄)/d_max                ⑤ 负载均衡
       − ζ·min(age, 20)                   ⑥ 任务老化      (+ 0.02·优先级)
```

权重单位是**等效米**，所以可读：绕开一个被占用的对接位，等价于多开 `γ/α = 6` 米。
放货位权重 1.5×，因为卸不掉货的车会堵住通道。老化项**必须封顶** ——
不封顶它会压过其余所有项，拍卖退化成 FIFO。

## 实验结果

5 策略 × 3 种子 × 120 秒，在制 80 件物料，工厂完全相同，只有车队规模不同。

<p align="center">
  <img src="./assets/readme/policy-comparison.png" width="100%" alt="五种策略在两个车队规模下的吞吐、时延、单任务里程与利用率">
</p>

| 策略 | 3 台 吞吐/分 | 3 台 米/单 | 8 台 吞吐/分 | 8 台 米/单 | 8 台 近距事件 |
| --- | ---: | ---: | ---: | ---: | ---: |
| random | 9.0 | 12.2 | 10.6 | 13.8 | 6.7 |
| nearest | 10.4 | 9.8 | 12.3 | 11.7 | 4.3 |
| ssi | 10.0 | 10.1 | 15.2 | 12.9 | 6.3 |
| **ca_ssi** | **10.7** | **9.1** | **18.7** | **10.8** | **3.0** |
| hungarian | 9.9 | 10.4 | 14.9 | 11.9 | 6.5 |

**改进前 → 改进后**（同一份代码，只改了 `ca_ssi_cost`）：

| 指标 | 车队 | SSI | CA-SSI | 变化 | 相对 `random` |
| --- | --- | ---: | ---: | ---: | ---: |
| 吞吐 | 3 台 | 10.0 | 10.7 | **+7 %** | +18 % |
| 吞吐 | 8 台 | 15.2 | 18.7 | **+23 %** | +77 % |
| 每单里程 | 3 台 | 10.1 | 9.1 | **−10 %** | −25 % |
| 每单里程 | 8 台 | 12.9 | 10.8 | **−16 %** | −22 % |
| 近距事件 | 8 台 | 6.3 | 3.0 | **−53 %** | −55 % |

- **优势随竞争强度增长。** 3 台车很少争同一个单泊位工位；8 台车是常态，此时同一项代价
  值 +23 % 吞吐和一半的近距事件。`ca_ssi` 是唯一能"看见自己即将制造的拥塞"的策略。
- **`nearest` 并不稳定优于 `random`** —— 谁先来问就给谁最近的单，于是可能抢走本该属于
  另一台近得多的车的任务。
- **`hungarian` 是单轮最优，但从不获胜。** 它优化一张静态矩阵，看不见拥塞、电量与排队。
  最优的分配方案 ≠ 最优的系统。
- **结论依赖负载。** 在制物料只有 28 件时工厂"缺料饿死"，所有策略收敛到约 16 件/分钟，
  用什么规则根本测不出来。**不写明负载条件的策略研究不可复现。**

## 消融

六项代价逐项置零，同一套拍卖、同一组种子、同一座工厂。

<p align="center">
  <img src="./assets/readme/ablation.png" width="100%" alt="CA-SSI 六项代价的留一法消融">
</p>

| 代价模型 | 吞吐/分 | 相对完整 | 米/单 | 时延（秒） |
| --- | ---: | ---: | ---: | ---: |
| **CA-SSI（六项齐全）** | **17.45** | — | **11.3** | **17.6** |
| − 工位拥塞（γ） | 13.92 | **−20 %** | 12.6 | 18.6 |
| − 电量可达（δ） | 14.63 | **−16 %** | 12.3 | 18.0 |
| − 负载均衡（η） | 17.29 | −1 % | 11.6 | 18.0 |
| − 任务老化（ζ） | 17.32 | −1 % | 10.9 | 17.0 |

真正扛事的是两项：拥塞项值 **20 %** 吞吐，电量项值 **16 %**（即便在 0.10 %/m 的温和耗电下 ——
接下跑不完的任务就得中途脱离去充电）。负载均衡与任务老化在这个工况下是**如实的零结果**：
120 秒太短，攒不出里程优势；而防饿死项的职责是兜住最坏情况，不是抬高平均值。
两项都保留，但都不声称在这组实验里回了本。

## 稳定性

| 机制 | 作用 |
| --- | --- |
| **带 TTL 的租约** | 一个工位同时只有一个持有者，不存在 hold-and-wait；持有者失联后自动回收。 |
| **看门狗 + 重规划** | 10 秒无位移 → 重规划；第二次 → 低速爬行；第三次 → 放弃任务并重新入队。 |
| **靠泊模式** | 距目标 0.8 m 内，LiDAR 急停阈值降到 0.12 m —— 急停距离**永远不能大于到达判定**。 |
| **车体轮廓屏蔽** | 逐角度把回波与车体自身轮廓比较；否则车的两个前角正好落在 0.24 m 的急停扇区里。 |
| **按需重规划** | 只在同伴真的占住前方 2.2 m 路径时才重算 —— 定时重算会把纯追踪的路径反复重置，车在原地来回摆。 |

墙钟实测（4 台 AGV、200 秒，在泊位与标线修复之前）：
**35 次派单、5 次送达、0 报错、0 次看门狗中止、0 次"无路径"，空闲内存不低于 6.9 GB。**

**尚不可信的部分。** Gazebo 下车辆并不能复现指令运动：直接以 0.5 m/s 持续下发 15 秒，
底盘只前进 0.17 m（理论 7.5 m）。协同层的指令是正确的 —— 93% 的速度指令非零、均值 0.30 m/s
—— 所以问题出在轮地接触模型上，这也是计时实验走 `logic_only.launch.py` 的原因。
写在这里而不是藏起来：分配算法的结论是扎实的，物理可复现性还不是。

## 展望

1. **车队最贵的是停台，不是行驶。** 每一项都在给**工厂的**成本定价：拥塞项是空驶距离的
   6 倍，因为多开 6 米很便宜，停一台梳棉机不便宜。
2. **机制固定，领域知识从代价进来。** 新约束（经轴必须立式运输、交接班时通道单行）
   是**加一项**，不是重写调度器；工艺工程师不读代码也能读懂权重。
3. **拥塞感知可迁移。** 所有存在单泊位资源的领域 —— 码头、晶圆厂、医院物流、机场廊桥 ——
   都是这个结构，同一套台架在那边也能量。
4. **安全余量会复利。** 拥塞写进目标函数而不是留给恢复策略，汇合更少、僵持更少、
   看门狗介入更少、安全半径可以更紧。近距事件只是这个循环的开端。
5. **每个结论都可证伪。** 所有数字都来自 `experiments/` 下的 CSV，一条命令、固定种子产出。
   把 `ca_ssi` 换成自己的代价函数，重跑即可。

## 复现

```bash
# 三组实验：车队受限期 / 运力富裕 / 消融
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

# 动图分两步：先录一次真实运行，再离屏重画
ros2 launch fleetflow_sim factory.launch.py num_robots:=5 policy:=ca_ssi
python3 tools/record_run.py /tmp/run.jsonl --fps 10 --seconds 600 --min-robots 5
python3 tools/render_demo.py /tmp/run.jsonl assets/readme/demo.gif --speed 8
```

| 指标 | 定义 |
| --- | --- |
| makespan | 最后一次送达 − 第一次派单 |
| 吞吐 | 每分钟完成任务数 |
| 任务时延 | 送达 − 生成，报告均值 / p50 / p95 |
| 利用率 | 每台车处于"行驶或装卸"的时间占比 |
| 近距事件 | 任意两车距离小于 0.55 m 的上升沿次数 |
| reassignment | 因卡死或失联被回收的任务数 |

## 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/factory/tasks` | `TransportTask` | 新任务，周期性重播 |
| `/factory/task_status` | `TransportTask` | 派发、完成、失败 |
| `/factory/completed` | `Int32` | 送达回执 |
| `/factory/machines` | `MachineState` | 状态、位置、进出料数、忙闲比 |
| `/factory/summary` | `String`（JSON） | 看板渲染用的汇总 |
| `/fleet/robots` | `RobotStatus` | 位姿、状态、电量、当前任务 |
| `/scheduler/request_task` | `RequestTask` | 车辆拉取下一个任务 |
| `/traffic/acquire`, `/traffic/release` | srv | 工位与充电桩租约 |
| `/robot_N/path` | `nav_msgs/Path` | 剩余规划路径（RViz、录制） |

## 目录结构

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/       # 消息与服务定义
├── src/fleetflow_sim/
│   ├── fleetflow_sim/
│   │   ├── layout.py               # 车间几何、停靠位、通道障碍
│   │   ├── planner.py              # 栅格 A*、视线拉直、纯追踪
│   │   ├── factory_manager.py      # 物料与机台模型
│   │   ├── task_scheduler.py       # 分配策略 + 看门狗
│   │   ├── traffic_manager.py      # 工位租约
│   │   ├── robot_controller.py     # 状态机、TF、避让、电量
│   │   ├── policies.py             # random / nearest / SSI / CA-SSI / 匈牙利
│   │   ├── qos.py · metrics.py · dashboard.py
│   ├── worlds/textile_factory.sdf  # 由脚本生成
│   ├── urdf/agv.urdf.xacro
│   └── launch/
├── tools/                          # build_world · run_experiments · record_run
│                                   # render_demo · plot_* · capture_views
└── experiments/                    # 原始 CSV：saturated/ slack/ ablation/
```

## 说明与边界

- 拉取式派单是刻意的：push 模型必须维护每台车的忙碌状态，一次完成通知丢失就会让那台车
  永久"忙碌" —— 这正是促使这次重构的 bug。
- `ca_ssi` 给"它看得见的"拥塞定价，看不见"下一次派单会造成的"拥塞；而且一次只派一单，
  不规划多单路线。滚动时域上的排序拍卖是下一步。
- `logic_only.launch.py` 的里程计由本地积分得到，协同逻辑与仿真模式完全一致。
- 电量参数调成了"一次短运行内能观察到充电"，是行为演示，不是能耗研究。
