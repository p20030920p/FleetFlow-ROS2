<p align="right">
  <strong>English</strong> · <a href="./README.zh-CN.md">简体中文</a>
</p>

# FleetFlow-ROS2

**Multi-AGV material transport for a textile mill** · ROS 2 Jazzy + Gazebo Sim 8

<p align="center">
  <img src="./assets/readme/control-center.png" width="100%" alt="FleetFlow control centre: live statistics on the left, factory map on the right with six AGVs and their active transport routes">
</p>

Six AGVs move material barrels between carding, drawing and roving machines. A priority scheduler hands out transport work, each vehicle plans its own route and drives it, and a control centre reports fleet state, machine utilisation and per-stage progress.

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo%20Sim-8-orange" alt="Gazebo Sim 8">
  <img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white" alt="Python 3.12">
</p>

## Gallery

<p align="center">
  <img src="./assets/readme/gazebo-iso.png" width="49%" alt="A 3/4 view of the Gazebo factory with AGVs moving between machine groups">
  <img src="./assets/readme/gazebo-line.png" width="49%" alt="A low-angle view along the production line with two AGVs and the carding machines">
</p>

## What it does

Material moves along a four-stage line, and its colour tracks the stage: **empty → green → yellow → red**.

1. An AGV takes an empty barrel from the empty storage and drops it at a carding waiting slot.
2. Carding consumes it and outputs a green barrel at the finished slot.
3. The next AGV carries it to drawing 1, then to drawing 2, then to the red buffer, which counts as completed.

Machines hold `idle` / `processing` state and take a fixed process time per stage, so the line only flows when the fleet keeps up.

## What is actually running

| Piece | Implementation |
| --- | --- |
| Factory world | `worlds/textile_factory.sdf` — Gazebo Sim 8, 16 × 10 m floor, 5 storage zones, 8 machines across 3 stages, 3 capture cameras |
| Vehicle | `urdf/agv.urdf.xacro` — differential drive, 180-beam LiDAR, per-vehicle colour, spawned by `ros_gz_sim create` |
| Task generation | `factory_manager` — material and machine model, emits one `TransportTask` per move |
| Assignment | `task_scheduler` — priority queue, vehicles **pull** work when idle, so two AGVs never take the same job |
| Driving | `robot_controller` × N — 0.1 m grid A\*, string-pulled path, pure-pursuit following, load/unload state machine |
| Monitoring | `fleet_dashboard` — renders the control centre to PNG, no browser required |

## Quick start

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Full simulation (headless Gazebo, writes control-centre frames every 3 s)
ros2 launch fleetflow_sim factory.launch.py

# With the Gazebo GUI instead
ros2 launch fleetflow_sim factory.launch.py headless:=false gui:=true

# No Gazebo at all: scheduling logic only, useful on machines without a GPU
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8
```

Capture the three camera views once Gazebo is up:

```bash
python3 tools/capture_views.py /tmp/shots /view_iso/image /view_top/image /view_line/image
```

## Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/factory/tasks` | `TransportTask` | new transport work |
| `/factory/task_status` | `TransportTask` | assignment and completion, so a vehicle is released |
| `/factory/completed` | `Int32` | delivery receipt from a vehicle |
| `/factory/machines` | `MachineState` | state, position, input/output counts, busy ratio |
| `/factory/summary` | `String` (JSON) | counts the control centre renders |
| `/fleet/robots` | `RobotStatus` | pose, state and current task per vehicle |
| `/scheduler/request_task` | `RequestTask` service | a vehicle pulls its next job |
| `/robot_N/cmd_vel`, `/robot_N/odom`, `/robot_N/scan` | ROS 2 ↔ Gazebo | bridged by `ros_gz_bridge`, namespaced per vehicle |

## Layout

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/   # msgs + srv (TransportTask, MachineState, RobotStatus, RequestTask)
├── src/fleetflow_sim/
│   ├── fleetflow_sim/          # layout · planner (A*) · factory_manager · task_scheduler
│   │                           # robot_controller · dashboard
│   ├── worlds/textile_factory.sdf
│   ├── urdf/agv.urdf.xacro
│   └── launch/                 # factory.launch.py · logic_only.launch.py
├── assets/readme/              # screenshots used above
└── tools/capture_views.py      # grab the Gazebo camera views as PNG
```

## Verified on this machine

Run on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Sim 8.11, 6 AGVs: **25 deliveries**, material spread across all four stages, LiDAR streaming at ~4 Hz, three camera views captured, and no simulation errors. The control-centre frame above is one of 50 rendered during that run.

## Notes

- The scheduler uses a pull model on purpose: a vehicle asks for work only when it is idle, which removes the double-assignment race a push model has.
- `logic_only.launch.py` exists so the scheduling logic can be tested without a GPU or a simulator — it is what CI would run.
- This is the ROS 2 line. The earlier ROS 1 Noetic implementation lives in a separate repository.
