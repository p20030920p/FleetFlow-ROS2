<div align="center">

# FleetFlow-ROS2

**Multi-AGV material transport for a textile mill**

<sub>ROS 2 Jazzy · Gazebo Sim 8 · per-vehicle namespaces and TF · lease-based docking · CA-SSI allocation</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8-orange)](https://gazebosim.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#quick-start)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](#quick-start)

[Quick start](#quick-start) &nbsp;•&nbsp; [Fleet](#fleet) &nbsp;•&nbsp; [Allocation](#allocation) &nbsp;•&nbsp; [Results](#results) &nbsp;•&nbsp; [Ablation](#ablation) &nbsp;•&nbsp; [Stability](#stability)

*English &nbsp;|&nbsp; [中文](README.zh-CN.md)*

</div>

![A full transport cycle: tasks appear as hollow squares, the auction awards each to a vehicle, the vehicle drives its planned path and detours around pallets parked in the aisle](assets/readme/demo.gif)

*One cycle, dispatch to delivery. Solid line = covered, dashed = remaining plan. The aisle
pallets are static obstacles in the A\* cost map.*

*Recorded 2026-09-15 · 10 AGVs · 36 units · 120 × 60 m plant · logic stack · 60 s of a 260 s run ·
90 frames, rendered offscreen on a viewport covering storage → carding → drawing → roving.
Ideal kinematics, not Gazebo contact; passing clearance 3.8× the 0.80 m requirement.*

Ten AGVs move cans between carding, drawing and roving machines on a 120 × 60 m floor. A scheduler
assigns the work, each vehicle plans and drives its own route, and a shift board reports the floor.

| | |
|---|---|
| **Scaling** — identical plant, 80 units | 1 AGV **3.0** · 2 AGVs **6.4** · 4 AGVs **9.6–10.6** tasks/min |
| **Efficiency** | 9.4–11.2 m per task · 16–19 s latency · utilisation **0.93–0.98** |
| **What carries the cost model** | dock contention (**−20 %** if removed) · energy feasibility (**−16 %**) |
| **Single-round optimum** | `hungarian` never wins — 14.9 vs 18.7 |
| **Verified** | coordination stack, wall-clock · **Gazebo physics is not yet** — see [Stability](#stability) |

---

## Quick start

```bash
source /opt/ros/jazzy/setup.bash          # required first: provides ros2 and colcon
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd FleetFlow-ROS2                         # build from the repo root, not the workspace root
colcon build --symlink-install
source install/setup.bash                 # per-shell; re-run in every new terminal

python3 tools/preflight.py                # environment: leftovers, DISPLAY, GL
python3 tools/check_gazebo.py             # three gates: server, GUI, rendering

ros2 launch fleetflow_sim factory.launch.py                  # Gazebo server only
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=10 policy:=ssi   # no Gazebo

# Gazebo GUI and the floor plan live in a browser, side by side
ros2 launch fleetflow_sim factory.launch.py gui:=true web:=true num_robots:=10
# open http://127.0.0.1:8080 — the board streams on /stream and /map/stream
```

`check_gazebo.py` tests three independently failable things and names the one that breaks:
**server** (headless physics, 20 s), **GUI** (`gz gui` must survive 25 s; needs
`DISPLAY`/`WAYLAND_DISPLAY` and a working GL stack), **rendering** (camera images, needs
`bridge_cameras:=true`). `--no-gui` skips the window.

| Symptom | Cause | Fix |
|---|---|---|
| Window opens, renders nothing | orphaned server from a `kill -9` captures it | `bash tools/gz_reset.sh` |
| Window force-quits at once | no display, or software GL (llvmpipe/swrast) | `gui:=false web:=true` — no GPU needed |

Both views read the same topics, so putting them next to each other is the quickest way to check
that the map, headings and task flow agree with the 3D scene. The page streams MJPEG; clicking it
switches to the plan drawn full-screen (`Esc` or a second click returns), and `/map.png` fetches
that still directly. `web_port`, `web_size`, `web_every` override port, resolution and refresh
interval. On its own: `ros2 run fleetflow_sim live_view`.

The browser view is the same renderer as the screenshots, redrawn live, so it runs at **4-6 fps**
(250 ms for the board, 150 ms for the plan) — that is matplotlib's ceiling, not a setting. The
animation at the top of this page is 12.5 fps because it is **rendered offline** from a recorded
run, where nothing has to keep up with the simulation; see [Reproducing](#reproducing) for that
pipeline.

`gui:=false` (default) starts **one** server with `--headless-rendering`; `gui:=true` starts **one**
GUI. Never both — a second server is what makes a GUI window open and render nothing. If a run was
killed rather than interrupted, its server survives as an orphan and the next GUI may attach to it:
`bash tools/gz_reset.sh` clears them.

Camera topics are **not bridged by default**. A bridged `sensor_msgs/Image` whose subscriber falls
behind grows without bound — measured at ~5 GB RSS, enough to trigger the OOM killer. Cameras are
`always_on=0` (they render only while subscribed) and are bridged one at a time:

```bash
ros2 launch fleetflow_sim factory.launch.py bridge_cameras:=true
ros2 run ros_gz_bridge parameter_bridge "/view_iso/image@sensor_msgs/msg/Image@gz.msgs.Image" &
python3 tools/capture_views.py /tmp/shots /view_iso/image && kill %1
```

---

## Fleet

The world is generated by [`tools/build_world.py`](tools/build_world.py): 527 models — machine lanes,
can racks, overhead travelling cleaners, aisle pallets, hazard markings.

![Cutaway iso view of the mill](assets/readme/gazebo-iso.png)

*Cutaway iso: machine lanes, can racks, overhead travelling cleaners.*

<p align="center">
  <img src="assets/readme/gazebo-top.png" width="32%" alt="Plan view">
  <img src="assets/readme/gazebo-line.png" width="32%" alt="Along the line">
  <img src="assets/readme/gazebo-machine.png" width="32%" alt="Machine level">
</p>

*Plan view · along the line · machine level.*

![Control-room view: floor plan with vehicles, their planned routes and material-flow counters](assets/readme/control-center.png)

*The same renderer emits stills and timed frames, so the floor view and the animation cannot drift apart.*

![Runtime topology](assets/readme/architecture.png)

*factory_manager → task_scheduler → namespaced robot_controllers → Gazebo, with the lease service across it.*

| Concern | Implementation |
|---|---|
| Namespaces / TF | One namespace and one `robot_state_publisher` per vehicle; TF is a forest, not one contested chain. |
| Docking | Lease-based, TTL-reclaimed. A vehicle holds at most one lease, never holds one while requesting another, and waits on open floor — no *hold-and-wait*, so no circular wait. |
| Avoidance | Reciprocal yielding with deterministic priorities; peers injected as a dynamic obstacle layer before each A\*; independent LiDAR stop. |
| Energy | Drain per metre; below threshold a vehicle stops bidding and queues for a charger through the same lease mechanism. |
| Dispatch | Pull-based: vehicles ask only when idle, which removes the double-assignment race. |

---

## Allocation

Transport assignment is **multi-robot task allocation**. Deadhead distance alone is wrong for a mill: a
machine has exactly **one** docking position, battery is a **hard** constraint, and the cheapest vehicle
now is not the cheapest fleet over a shift.

| Policy | Cost |
|---|---|
| `random` | none — lower bound |
| `nearest` | own nearest task, local only |
| `ssi` | sequential single-item auction over deadhead distance |
| **`ca_ssi`** | **the same auction, six-term industrial cost** |
| `hungarian` | linear-assignment optimum of that cost matrix |

![The six cost terms and their weights](assets/readme/cost-model.png)

*Weights in equivalent metres, read from `policies.py` at render time.*

```
J = α‖p_r − s_t‖ + β‖s_t − g_t‖ + γ(n_src + 1.5 n_dst)
  + δ·max(0, e_need + reserve − e_r) + η(d_r − d̄)/d_max − ζ·min(age, 20) + 0.02·prio
```

Avoiding one contended dock costs the same as `γ/α = 6 m` of driving. Ageing is capped; uncapped it
dominates every other term and the auction degenerates to FIFO.

---

## Results

> **Throughput scales with fleet size.** Gazebo, 24 units, three 300 s runs per fleet:

| AGVs | transport ops per 300 s | per min | hull overlaps |
|---:|---|---:|---:|
| 1 | @@GZ1@@ | @@GZ1TP@@ | @@GZ1OV@@ |
| 2 | @@GZ2@@ | @@GZ2TP@@ | @@GZ2OV@@ |
| 4 | 16 · 10 · 13 · 14 · 22 · 18 | 1.8–5.5 | 0 |

> **What the column counts.** These are *transport operations* — one move of material from A to B
> — not finished products. A finished unit needs five or more of them, so product counts are an
> order of magnitude lower: about 0.6 units per minute in logic mode, while machines still sit
> idle **76–100 % of the time waiting for material**. For finished output read the factory's
> `done`, logged every ten seconds.
>
> `/robot_i/odom` had incompatible QoS (bridge RELIABLE, controller BEST_EFFORT), so under DDS no
> message was delivered and every layer reasoned about a robot still at its spawn pose. Fixing it
> roughly doubled output and took false stalls from 96–232 to zero.

## Allocation

Transport assignment is **multi-robot task allocation**. Deadhead distance alone is wrong for a mill: a
machine has exactly **one** docking position, battery is a **hard** constraint, and the cheapest vehicle
now is not the cheapest fleet over a shift.

| Policy | Cost |
|---|---|
| `random` | none — lower bound |
| `nearest` | own nearest task, local only |
| `ssi` | sequential single-item auction over deadhead distance |
| **`ca_ssi`** | **the same auction, six-term industrial cost** |
| `hungarian` | linear-assignment optimum of that cost matrix |

![The six cost terms and their weights](assets/readme/cost-model.png)

*Weights in equivalent metres, read from `policies.py` at render time.*

```
J = α‖p_r − s_t‖ + β‖s_t − g_t‖ + γ(n_src + 1.5 n_dst)
  + δ·max(0, e_need + reserve − e_r) + η(d_r − d̄)/d_max − ζ·min(age, 20) + 0.02·prio
```

Avoiding one contended dock costs the same as `γ/α = 6 m` of driving. Ageing is capped; uncapped it
dominates every other term and the auction degenerates to FIFO.

---

## Results

> **Fleet size is a design parameter, not a target.** Four robots, same plant, 24 units,
> six 300 s Gazebo runs — the spread is the point:

| AGVs | transport ops per 300 s | per min | hull overlaps |
|---:|---|---:|---:|
| **4** (recommended) | 16 · 10 · 13 · 14 · 22 · 18 | 1.8–5.5 | 0 |
| 8 | 4 · 7 · 9 | 0.7–1.7 | 0 · 0 · 2 |

> **Eight robots deliver half of what four do.** Yields jump from single digits to 42–87, so
> the larger fleet is not busier — it spends most of its time giving way. **Four robots are the
> economic fleet for this layout.** Note that the constraint is not the fleet: see below.
>
> **What the column counts.** These are *transport operations* - one move of material from A to
> B - not finished products. A finished unit needs five or more of them, so product counts are
> an order of magnitude lower: about 0.6 units per minute in logic mode, while machines still
> sit idle **76-100 % of the time waiting for material**. For finished output read the
> factory's `done`, logged every ten seconds.
>
> `/robot_i/odom` had incompatible QoS (bridge RELIABLE, controller BEST_EFFORT), so under DDS
> no message was delivered and every layer reasoned about a robot still at its spawn pose.
> Fixing it roughly doubled output and took false stalls from 96–232 to zero.
>
> Variance remains: the same command over the same duration swings about twofold.
> Full evidence: [docs/gazebo-throughput-findings.md](docs/gazebo-throughput-findings.md).

**The limit is buffer capacity, not the fleet or the aisles.** Median delivery latency is 13 s;
output collapses in the line because each unit waits for processing, transport and a free downstream
berth at every stage, and only about twelve are in process at once — a ceiling near 5 units/min.
Fleet size, docking-slot count and a larger in-flight cap were each measured; none raises it.

**Stage spacing sets the deadlock threshold.** Two vehicles need **0.44 + 2 × 0.05 = 0.74 m** of hull
clearance to pass. Below that a docking vehicle blocks the lane permanently. The original plant had
**0.65 m** between the drawing finished berth and the roving waiting berth, and vehicles froze there.
`stage_gap` now defaults to **2.0 m**.

Stage spacing is now a parameter (`stage_gap`, default **2.0 m**) and the effect is a clean
threshold, measured with 4 AGVs over 200 s:

| Stage gap | Stall share of fleet time |
|---:|---:|
| **0.65 m** (below the 0.74 m passing requirement) | **13.2 %** |
| 1.20 m | 0.0 % |
| 2.00 m (default) | 0.0 % |
| 3.00 m | 0.0 % |

Three further 200 s runs at the default gap also measured zero. The demo above is recorded with the
default, so it contains no freezes.

![Live dashboard, four AGVs working](assets/readme/live-board-4agv.png)

*Live board from Gazebo — `web:=true`, then `http://127.0.0.1:8080`; `/stream` and `/map/stream` are multipart streams carrying PNG frames.*

5 policies × 3 seeds × 120 s, 80 units in circulation, identical plant — only the fleet differs.

*Historical record, measured before the traffic layer and controller shared one collision
criterion. Not reproducible under the current code (a fresh 80-unit, 8-AGV `ca_ssi` run lands at
6–8 tasks/min). Treat as unverified until re-run.*

![Throughput, latency, travel per task and utilisation at 1, 2 and 4 AGVs](assets/readme/policy-comparison.png)

*Error bars 1σ; percentages relative to `random`. 5 policies × 3 seeds × 120 s, 80 units in
circulation, identical plant — only the fleet size differs.*

**Throughput · latency (s) · travel per task (m) · utilisation**

| Fleet | random | nearest | SSI | CA-SSI | Hungarian |
|---|---|---|---|---|---|
| 1 AGV | 2.0 · 20.9 · 14.3 · 0.86 | 2.9 · 15.4 · 10.2 · 0.98 | 3.0 · 16.0 · 9.4 · 0.97 | 3.0 · 16.2 · 9.5 · 0.98 | 3.0 · 16.0 · 9.4 · 0.97 |
| 2 AGVs | 3.9 · 19.2 · 15.9 · 0.97 | 6.4 · 18.1 · 9.8 · 0.97 | 6.4 · 18.3 · 9.9 · 0.97 | 6.2 · 16.2 · 9.6 · 0.98 | 6.4 · 16.7 · 9.3 · 0.97 |
| 4 AGVs | 7.4 · 20.8 · 14.6 · 0.98 | 9.2 · 18.1 · 11.8 · 0.94 | 10.4 · 18.0 · 10.2 · 0.93 | 9.6 · 18.7 · 11.2 · 0.93 | 10.6 · 18.3 · 11.1 · 0.96 |

**Throughput scales close to linearly with fleet size**: 3.0 → 6.4 → 10.4 tasks/min for SSI, and
utilisation stays at **0.93–0.98** throughout. Above two vehicles the four allocation rules are
within one standard deviation of each other, so **the allocation rule is not what limits this
plant**. The differentiator is `random`, which costs 24–35 % of throughput and 35–50 % of travel at
every fleet size.

What limits output is berth and buffer capacity, not allocation: at four vehicles the plant runs
96 % of the time with machines **starved 76–100 %** — they wait for material rather than for a
vehicle.

---

## Ablation

Each cost term removed on its own; same auction, seeds and plant.

![Leave-one-out ablation of the cost terms at 1, 2 and 4 AGVs](assets/readme/ablation.png)

*2 seeds per cell, 80 units. Percentages relative to the full cost model.*

| Cost model | 1 AGV | 2 AGVs | 4 AGVs |
|---|---:|---:|---:|
| **CA-SSI (full)** | 3.03 | 6.80 | 7.67 |
| − dock contention (γ) | 2.77 | 6.06 | 9.68 |
| − energy feasibility (δ) | 2.77 | 6.55 | 7.59 |
| − load balance (η) | 3.03 | 5.82 | 9.90 |
| − task ageing (ζ) | 3.02 | 6.12 | 8.84 |

Removing **dock contention** (γ) or **load balance** (η) raises four-vehicle throughput by **26 %** and **29 %**, while both help at one and two vehicles. Two seeds per cell, so read the effect sizes as indicative - but the reversal is consistent across both seeds, and it is why the cost model is not claimed to be optimal at every fleet size.

---

## Stability

| Mechanism | Purpose |
|---|---|
| Leases + TTL | One holder per station; no hold-and-wait; reclaimed if a holder dies. |
| Watchdog | 10 s without progress → re-plan; second strike → creep; third → abandon and re-queue. |
| Docking mode | Inside 0.8 m of the target the LiDAR stop drops to 0.12 m, so the emergency stop can never exceed the arrival tolerance. |
| Hull self-mask | Returns tested against the vehicle's own outline angle by angle; its front corners otherwise sit in the stop sector at 0.24 m. |
| Event-driven re-planning | Only when a peer occupies the next 2.2 m of the path — a timer resets pure pursuit mid-turn and the vehicle oscillates in place. |
| Collision test | Oriented rectangles against each other (separating-axis), not centre distance. See below. |

4 AGVs, 300 s wall clock, six Gazebo runs, everything enabled: **16 · 10 · 13 · 14 · 22 · 18 transport
operations, zero hull intersections, zero false stalls, no crashes.** Output varies about twofold
across identical commands.

**Why centre distance is the wrong test.** The body is 0.56 × 0.44 m: two vehicles need 0.44 m
centre distance side by side, 0.56 m nose to tail, and 0.712 m to be safe at *any* orientation. The
code used 0.34 m and applied it only inside a ±52° front cone, so it commanded vehicles into each
other and ignored anything off to the side. Gazebo showed it — 1900 overlap events, closest approach
0.259 m. It is now a separating-axis test between the two oriented rectangles.

**Two log-silent faults were fixed.**

1. `/robot_i/odom` QoS. The bridge publishes RELIABLE, the controller subscribed BEST_EFFORT; DDS
   delivers *nothing* across that pair. Every layer above the driver reasoned about a robot that had
   not moved since spawn, and the pose-difference speed derived from it was identically zero, so the
   coordinator read the whole fleet as permanently stopped and fired 96–232 false stall recoveries
   per run. Output roughly doubled after the fix; false stalls reached zero.
2. The coordinator judged "is this robot stopped?" from the *last commanded* velocity. A stopped
   robot stops being commanded, so the field froze at its pre-stop value and an idle vehicle
   reported 0.85 m/s. Wait detection, anti-starvation priority and yield decisions were dead code.

**Can supply caps concurrency.** Four empty-can docking slots exist, so at most four tasks start
concurrently and a can enters only when one advances a stage. The per-minute figures therefore mix
supply, the plant's internal rate (about 8 units/min, set by the middle stage) and transport.
Fleet-size comparisons hold because all arms share the supply limit; the absolute figure is not a
measure of fleet capability.

The animation at the top predates these fixes, so its vehicles pass closer than physics allows.

---

## Reproducing

```bash
# experiments
python3 tools/run_experiments.py --policies random nearest ssi ca_ssi hungarian \
    --seeds 1 2 3 --seconds 120 --robots 3 --drain 0.10 --out experiments/saturated \
    --extra max_tasks_in_flight:=18 num_materials:=80
python3 tools/run_experiments.py --policies random nearest ssi ca_ssi hungarian \
    --seeds 1 2 3 --seconds 120 --robots 8 --drain 0.10 --out experiments/slack \   # study scale
    --extra max_tasks_in_flight:=18 num_materials:=80
python3 tools/run_experiments.py \
    --policies ca_ssi ca_nocong ca_noener ca_nobal ca_noage \
    --seeds 1 2 3 --seconds 120 --robots 8 --drain 0.10 --out experiments/ablation \
    --extra max_tasks_in_flight:=18 num_materials:=80

# figures
python3 tools/plot_results.py experiments/saturated/summary.csv \
    experiments/slack/summary.csv assets/readme/policy-comparison.png
python3 tools/plot_ablation.py experiments/ablation/summary.csv assets/readme/ablation.png
python3 tools/plot_cost_model.py assets/readme/cost-model.png
python3 tools/plot_architecture.py assets/readme/architecture.png

# the animation is two steps: record a run, then redraw it offscreen
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=6 policy:=ca_ssi
python3 tools/record_run.py /tmp/run.jsonl --fps 10 --seconds 95 --min-robots 6
python3 tools/render_demo.py /tmp/run.jsonl assets/readme/demo.gif --speed 6
```

| Metric | Definition |
|---|---|
| makespan | last delivery − first assignment |
| throughput | completed tasks per minute of wall time |
| latency | delivery − creation, mean / p50 / p95 |
| utilisation | share of wall time spent moving or handling |
| near-miss | rising-edge count of pairs closer than 0.55 m |

---

## Interfaces

| Interface | Type | |
|---|---|---|
| `/factory/tasks` | `TransportTask` | new work, re-announced periodically |
| `/factory/task_status` | `TransportTask` | assignment, completion, failure |
| `/factory/machines` | `MachineState` | state, position, in/out counts, busy ratio |
| `/fleet/robots` | `RobotStatus` | pose, state, battery, current task |
| `/scheduler/request_task` | srv | a vehicle pulls its next job |
| `/traffic/acquire`, `/traffic/release` | srv | station and charger leases |
| `/robot_N/path` | `nav_msgs/Path` | remaining planned path |

## Layout

```text
FleetFlow-ROS2/
├── src/fleetflow_interfaces/       # msgs + srv
├── src/fleetflow_sim/
│   ├── fleetflow_sim/              # layout · planner · factory_manager
│   │                               # task_scheduler · traffic_manager
│   │                               # robot_controller · policies · dashboard · live_view
│   ├── worlds/textile_factory.sdf  # generated
│   ├── urdf/agv.urdf.xacro
│   └── launch/
├── tools/                          # build_world · run_experiments · record_run
│                                   # render_demo · plot_* · capture_views
└── experiments/                    # raw CSVs: saturated/ slack/ ablation/
```

## Limits

- `ca_ssi` prices the contention it can see, not the contention its next assignment will cause, and it
  assigns one task at a time rather than a route of several.
- Pull dispatch is deliberate: a push model must track every vehicle's busy state, and one lost
  completion message leaves a vehicle permanently busy.
- Battery parameters are tuned so a charging cycle fits inside a short run — a behaviour demonstration,
  not an energy study.
