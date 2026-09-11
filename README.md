<p align="right">
  <strong>English</strong> · <a href="./README.zh-CN.md">简体中文</a>
</p>

# FleetFlow-ROS2

**Multi-AGV material transport for a textile mill** · ROS 2 Jazzy + Gazebo Sim 8

<p align="center">
  <img src="./assets/readme/control-center.png" width="100%" alt="FleetFlow control centre: live statistics on the left, factory map on the right with AGVs, their active transport routes and dock slots">
</p>

A fleet of AGVs moves material barrels between carding, drawing and roving machines. A priority scheduler hands out transport work, each vehicle plans its own route and drives it, and a control centre reports fleet state, machine utilisation and per-stage progress.

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

## Why this is a real multi-robot system, not a demo of one

Spawning several models in one Gazebo world is the easy part. What makes a fleet behave like
a fleet is everything around it — and each of those pieces is implemented here:

| Concern | What is actually implemented |
| --- | --- |
| **Namespaces and TF** | Every vehicle runs in its own namespace with its own `robot_state_publisher` and `frame_prefix`, so TF is a forest of disjoint trees (`robot_0/odom → robot_0/base_footprint → robot_0/base_link → …`) rather than one contested chain. |
| **QoS that matches the data** | Sensor streams (scan, odometry) use best-effort with shallow queues so a slow subscriber cannot stall the publisher; commands, task and fleet state use reliable delivery with deeper queues. |
| **Traffic management** | Docking is serialised by an explicit lease service. A vehicle must hold the lease for the station it is about to enter, and releases it once it has physically left. |
| **Deadlock freedom by construction** | A vehicle holds **at most one** station lease, is never in a position to hold one while requesting another, and waits on open floor rather than inside a station. That removes the *hold-and-wait* condition, so no circular wait can form. Leases also carry a TTL, which reclaims the station if a vehicle dies holding it. |
| **No single-point bottleneck** | Storage areas expose several *dock slots* on a ring instead of one shared coordinate. Without them every vehicle converges on the same point, the reciprocal avoidance deadlocks, and the watchdog aborts the task — the failure mode that motivated the redesign. |
| **Reciprocal collision avoidance** | Each vehicle sees peer poses and yields: it slows inside a look-ahead cone, stops for a higher-priority peer, and always stops at a hard safety distance. Priorities are deterministic (lower id proceeds), which breaks symmetric standoffs. |
| **Peer-aware planning** | Other vehicles are injected as a dynamic obstacle layer before each A\* call, so paths route around traffic instead of relying on local reactions alone. |
| **LiDAR safety layer** | The forward sector of the scan is watched independently; anything inside the emergency radius stops the vehicle regardless of what the planner wants. |
| **Battery and charging** | Energy drains per metre travelled. Below a threshold a vehicle stops taking work, queues for a charging bay through the same lease mechanism, charges, and rejoins the fleet. |
| **Watchdog and task reclamation** | A vehicle that stops making progress re-plans, then abandons the task; the scheduler separately reclaims tasks from vehicles that stop reporting. Both are counted as reassignments. |
| **Pull-based allocation** | Vehicles ask for work only when idle, which removes the double-assignment race a push model has. |

## The allocation problem, stated properly

Assigning transport work to vehicles is an instance of **multi-robot task allocation (MRTA)**.
Let `T` be the set of pending tasks, `R` the set of idle vehicles, and

```
c(r, t) = ‖ p_r − s_t ‖₂          # travel cost: vehicle pose to task pickup point
```

the marginal cost of assigning task `t` to vehicle `r`. We look for an assignment that
minimises a fleet objective — total travel, mean task latency, or makespan.

Three policies are implemented so the choice can be measured rather than assumed:

| Policy | Rule | Information used |
| --- | --- | --- |
| `random` | the requesting vehicle takes a uniformly random pending task | none — lower bound |
| `nearest` | the requesting vehicle takes its own minimum-cost task | local only (fully decentralised) |
| `ssi` | sequential single-item auction: repeatedly award the globally cheapest `(r, t)` pair until no pair remains | global poses and task set |

`ssi` follows the market-based MRTA line (single-item auctions, Lagoudakis et al., 2005).
It is centralised, but the *delivery* of work stays pull-based: the scheduler computes the
award table and each vehicle collects its own row when it asks, so the no-contention property
of the pull model is preserved.

## How the fleet is measured

Every run writes two CSVs (`tasks.csv`, `run.csv`) incrementally, so a run killed mid-flight
still yields data.

| Metric | Definition |
| --- | --- |
| **makespan** | last delivery − first assignment |
| **throughput** | completed tasks per minute of wall time |
| **task latency** | delivery − creation, reported as mean / p50 / p95 |
| **fleet utilisation** | share of wall time each vehicle spends moving or handling |
| **distance travelled** | summed odometry displacement |
| **near-miss events** | rising-edge count of pairs closer than 0.55 m |
| **min inter-robot distance** | closest approach of any pair — the safety evidence |
| **traffic rejections / expiries** | lease contention and TTL recoveries |
| **reassignments** | tasks reclaimed after a stall or a silent vehicle |

## Results

3 runs per policy per condition, 8 AGVs, 100 s each, fixed seeds, no charging
(energy drain turned down so it cannot confound the comparison).

**Condition A — shallow task pool** (12 units, at most 6 tasks in flight)

| Policy | Completed | Throughput /min | Latency mean / p95 (s) | Travel per task (m) |
| --- | ---: | ---: | ---: | ---: |
| random | 27.0 | 14.8 | 15.8 / 21.5 | 10.3 |
| nearest | 29.7 | 16.3 | 15.7 / 21.6 | 9.7 |
| ssi | 27.0 | 14.8 | 15.7 / 21.4 | 9.7 |

**Condition B — deep task pool** (28 units, at most 18 tasks in flight)

| Policy | Completed | Throughput /min | Latency mean / p95 (s) | Travel per task (m) |
| --- | ---: | ---: | ---: | ---: |
| random | 21.7 | 13.1 | 16.9 / 22.3 | 10.2 |
| nearest | 21.0 | 12.7 | 16.6 / 22.2 | 10.3 |
| **ssi** | **29.7** | **17.9** | **16.2 / 21.0** | **10.0** |

<p align="center">
  <img src="./assets/readme/policy-comparison.png" width="100%" alt="Throughput, latency, travel per task and near-miss events for the three allocation policies, under a shallow and a deep task pool">
</p>

### What the numbers say

1. **Under a shallow task pool the policy is irrelevant.** Pending tasks rarely
   outnumber idle vehicles, so a vehicle usually has nothing to choose between —
   14.8 / 16.3 / 14.8 tasks per minute are within run-to-run noise.
2. **Under a deep task pool the auction pulls ahead: +37 % throughput** over both
   baselines. The interesting part is *why* `nearest` is no better than `random`:
   it answers whichever vehicle asks first, so a vehicle can claim a task that a much
   closer peer was about to take. With few choices that rarely happens; with many it
   happens constantly. SSI removes the effect by scoring all idle vehicles against all
   pending tasks at once.
3. **Latency is flat across policies** (16–17 s), because it is dominated by machine
   process time rather than by travel. Allocation shows up in throughput, not in how
   long a single task takes.
4. **Safety held in every run.** Closest approach never went below the 0.34 m hard
   limit, and near-miss events stayed in single digits per 100 s run.

### Honest caveats

- Three seeds per condition is enough to resolve a 37 % gap, not differences below
  roughly 10 %. Treat the Condition A ordering as noise.
- `ssi` is centralised and assumes the scheduler has reasonably fresh vehicle poses;
  an auction built on stale positions would give back part of that advantage.
- The workload, not the fleet, is the binding constraint in Condition A: fleet
  utilisation sits at ~0.67, so a quarter of the fleet's time is spent waiting for work.

## Running it

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Full simulation (headless Gazebo, writes control-centre frames every 3 s)
ros2 launch fleetflow_sim factory.launch.py

# With the Gazebo GUI
ros2 launch fleetflow_sim factory.launch.py headless:=false gui:=true

# Scheduling logic only — no Gazebo, no GPU. This is what CI and the
# experiments below use.
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi
```

Capture the three camera views once Gazebo is up:

```bash
python3 tools/capture_views.py /tmp/shots /view_iso/image /view_top/image /view_line/image
```

## Reproducing the comparison

```bash
# Condition A — shallow task pool
python3 tools/run_experiments.py --policies random nearest ssi --seeds 1 2 3 \
    --seconds 100 --robots 8 --drain 0.10 --out experiments/shallow

# Condition B — deep task pool
python3 tools/run_experiments.py --policies random nearest ssi --seeds 1 2 3 \
    --seconds 100 --robots 8 --drain 0.10 --out experiments/deep \
    --extra max_tasks_in_flight:=18 num_materials:=28

python3 tools/plot_results.py experiments/shallow/summary.csv \
    experiments/deep/summary.csv assets/readme/policy-comparison.png
```

Each `(policy, seed)` pair runs in its own directory with a fixed seed, so both the policy
and the tie-breaking are reproducible. The raw CSVs behind the tables above are committed
under `experiments/`, so the numbers can be checked without re-running anything.

## Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/factory/tasks` | `TransportTask` | new transport work, re-announced periodically so a late subscriber cannot miss it |
| `/factory/task_status` | `TransportTask` | assignment, completion and failure |
| `/factory/completed` | `Int32` | delivery receipt |
| `/factory/machines` | `MachineState` | state, position, in/out counts, busy ratio |
| `/factory/summary` | `String` (JSON) | counts the control centre renders |
| `/fleet/robots` | `RobotStatus` | pose, state and current task per vehicle |
| `/scheduler/request_task` | `RequestTask` | a vehicle pulls its next job |
| `/traffic/acquire`, `/traffic/release` | `AcquireLease` / `ReleaseLease` | station and charger leases |
| `/robot_N/cmd_vel`, `/robot_N/odom`, `/robot_N/scan`, `/robot_N/joint_states` | ROS 2 ↔ Gazebo | bridged and namespaced per vehicle |

## Layout

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/   # msgs + srv
├── src/fleetflow_sim/
│   ├── fleetflow_sim/
│   │   ├── layout.py           # shared factory geometry, dock slots, chargers
│   │   ├── planner.py          # grid A*, string pulling, pure pursuit, dynamic layer
│   │   ├── factory_manager.py  # material and machine model, task generation
│   │   ├── task_scheduler.py   # allocation policies + heartbeat watchdog
│   │   ├── traffic_manager.py  # station leases, hold-and-wait-free by design
│   │   ├── robot_controller.py # per-vehicle state machine, TF, avoidance, battery
│   │   ├── metrics.py          # CSV metrics for experiments
│   │   ├── policies.py         # random / nearest / SSI
│   │   ├── qos.py              # QoS profiles per data class
│   │   └── dashboard.py        # control centre renderer
│   ├── worlds/textile_factory.sdf
│   ├── urdf/agv.urdf.xacro
│   └── launch/                 # factory.launch.py · logic_only.launch.py
├── tools/                      # capture_views · run_experiments · plot_results
└── experiments/                # raw CSVs from the comparison above
```

## Notes and limits

- The scheduler uses a pull model deliberately. A push model has to track every vehicle's
  busy state, and a lost completion message leaves a vehicle permanently "busy" — which is
  exactly the bug that motivated this design. Pull makes that failure mode impossible, and a
  heartbeat watchdog covers the remaining case where a vehicle stops responding.
- Allocation is myopic: cost is travel distance to pickup, with no lookahead over future
  tasks or congestion. Adding congestion-aware or auction-with-sequencing costs is the
  obvious next step.
- `logic_only.launch.py` runs the whole coordination stack without Gazebo, which is how the
  policy comparison is reproduced in reasonable time. Odometry is integrated locally in that
  mode; the coordination logic is identical.
- Battery parameters are tuned so a charging cycle is observable inside a short run. Real
  AGVs manage far more distance per charge; the model is a behaviour demonstration, not an
  energy study.
