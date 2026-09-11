<p align="right">
  <a href="./README.md">English</a> · <strong>简体中文</strong>
</p>

# FleetFlow-ROS2

**纺织厂多 AGV 物料搬运仿真** · ROS 2 Jazzy + Gazebo Sim 8

<p align="center">
  <img src="./assets/readme/control-center.png" width="100%" alt="FleetFlow 控制中心：左侧实时统计，右侧工厂地图与 6 台 AGV 的调度路线">
</p>

6 台 AGV 在梳棉、拉伸、粗纱机器之间搬运物料桶。优先级调度器派发运输任务，每台车自己规划路径并执行，控制中心实时显示车队状态、机器利用率与各工序进度。

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

## 它在做什么

物料沿四段产线流动，颜色标记所处工序：**空 → 绿 → 黄 → 红**。

1. AGV 从空桶区取一只空桶，放到梳棉等料位。
2. 梳棉机消耗它，在完工位产出一只绿桶。
3. 下一台 AGV 依次把它送到拉伸 1、拉伸 2，最后送进红料区，计为完工。

机器有 `idle` / `processing` 两种状态，每道工序耗时固定 —— 所以只有车队跟得上，产线才流得动。

## 实际跑起来的东西

| 组成 | 实现 |
| --- | --- |
| 工厂世界 | `worlds/textile_factory.sdf` —— Gazebo Sim 8，16 × 10 m 厂房、5 个料区、3 道工序共 8 台机器、3 个抓图机位 |
| 车辆 | `urdf/agv.urdf.xacro` —— 差速底盘、180 线 LiDAR、每台车不同配色，由 `ros_gz_sim create` 注入 |
| 任务生成 | `factory_manager` —— 物料与机器模型，每次位移产出一条 `TransportTask` |
| 任务分配 | `task_scheduler` —— 优先级队列，空闲车辆**主动拉取**，因此不会两台车抢同一个任务 |
| 行驶 | `robot_controller` × N —— 0.1 m 栅格 A\*、视线拉直、纯追踪跟随、装卸状态机 |
| 监控 | `fleet_dashboard` —— 把控制中心渲染成 PNG，不需要浏览器 |

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

# 完全不启 Gazebo：只跑调度逻辑，适合没有 GPU 的机器
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8
```

Gazebo 起来之后抓三个机位：

```bash
python3 tools/capture_views.py /tmp/shots /view_iso/image /view_top/image /view_line/image
```

## 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/factory/tasks` | `TransportTask` | 新的搬运任务 |
| `/factory/task_status` | `TransportTask` | 分配与完成，用于释放车辆 |
| `/factory/completed` | `Int32` | 车辆送达回执 |
| `/factory/machines` | `MachineState` | 状态、位置、进出料计数、繁忙率 |
| `/factory/summary` | `String`（JSON） | 控制中心渲染用的汇总 |
| `/fleet/robots` | `RobotStatus` | 每台车的位姿、状态与当前任务 |
| `/scheduler/request_task` | `RequestTask` 服务 | 车辆主动拉取下一个任务 |
| `/robot_N/cmd_vel`、`/robot_N/odom`、`/robot_N/scan` | ROS 2 ↔ Gazebo | 由 `ros_gz_bridge` 桥接，按车辆加命名空间 |

## 目录结构

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/   # 消息与服务定义
├── src/fleetflow_sim/
│   ├── fleetflow_sim/          # layout · planner(A*) · factory_manager · task_scheduler
│   │                           # robot_controller · dashboard
│   ├── worlds/textile_factory.sdf
│   ├── urdf/agv.urdf.xacro
│   └── launch/                 # factory.launch.py · logic_only.launch.py
├── assets/readme/              # 上面这些截图
└── tools/capture_views.py      # 抓取 Gazebo 机位成 PNG
```

## 本机验证结果

在 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Sim 8.11 上跑 6 台 AGV：**25 次送达**，物料分布在四个工序，LiDAR 约 4 Hz 持续出流，三个机位抓图成功，仿真无报错。上面那张控制中心图就是该次运行中 50 帧里的一帧。

## 说明

- 调度刻意用 pull 模型：车辆空闲时才去要任务，从机制上消除了 push 模型的双重分配竞态。
- 提供 `logic_only.launch.py`，是为了在没有 GPU、甚至没有仿真器的机器上也能验证调度逻辑 —— CI 跑的就是它。
- 这是 ROS 2 版本；更早的 ROS 1 Noetic 实现放在另一个仓库。
