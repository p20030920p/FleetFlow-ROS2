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

> **Recorded 2026-09-14** from the current build: 6 AGVs and 40 units in the logic stack, one
> 60 s stretch of a longer run, 90 frames, rendered offscreen. It replaces a capture made before
> the odometry-QoS and speed-reporting fixes, in which vehicles passed closer than their bodies
> allow under physics — that file is no longer in the repository. Motion here is ideal
> kinematics, not Gazebo contact.
>
> The window is chosen for activity, not for peak output: `tools/pick_active.py` selects the
> 60 s with the most fleet displacement, and `tools/check_gif.py` verifies the result. An
> earlier attempt was discarded because it passed a naive "all frames differ" check while
> moving almost nothing — median 0.077 % of pixels changed per frame against 0.436 % here.

Five AGVs move cans between carding, drawing and roving machines on a 26 × 16 m floor. A scheduler
assigns the work, each vehicle plans and drives its own route, and a shift board reports the floor.

| | |
|---|---|
| **Allocation** — 8 AGVs, identical plant | CA-SSI **18.7** tasks/min · SSI 15.2 · random 10.6 |
| **Before → after** | **+23 %** throughput · **−16 %** m/task · **−53 %** near-miss |
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

python3 tools/preflight.py                # checks leftovers, DISPLAY, GL before launching

ros2 launch fleetflow_sim factory.launch.py                  # Gazebo server only
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi   # no Gazebo

# Gazebo GUI and the floor plan live in a browser, side by side
ros2 launch fleetflow_sim factory.launch.py gui:=true web:=true num_robots:=4
# open http://127.0.0.1:8080 — the board streams on /stream and /map/stream
```

> **If the Gazebo window force-quits or opens empty**, run `bash tools/gz_reset.sh` first: an
> orphaned server from a previous `kill -9` will capture the new window. Then run
> `python3 tools/preflight.py`, which reports leftovers, `DISPLAY`, `/dev/dri` and the GL
> renderer in one go. Software rendering (llvmpipe/swrast) is the usual cause on VMs and in
> containers — use `gui:=false web:=true` there, which needs no GPU at all.

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
> This round's progress is on root causes. On the Gazebo side, `/robot_i/odom` had
> incompatible QoS — the bridge publishes RELIABLE while the controller subscribed
> BEST_EFFORT, so under DDS not one message was delivered and both the controller and the
> coordinator believed every robot was still at its spawn pose. With that fixed, four robots
> roughly doubled their output, false stalls fell from 96–232 to zero, and hull overlaps
> went to zero.
>
> What is still unsolved is **variance**: the same command over the same duration can swing
> by a factor of two. Evidence and the failed attempts:
> [docs/gazebo-throughput-findings.md](docs/gazebo-throughput-findings.md).

**Where the plant actually loses its time.** A trajectory probe settles this, and it corrects an
earlier reading of mine. Eight robots over 240 s spend **52 % of their time idle** — so the fleet is
not blocked, and the aisle is not the limiter. A single delivery has a median latency of 13 s. What
collapses is the line: 17 loads reach carding, 8 reach drawing, 2 reach roving, 2 finish, because a
unit must pass processing, waiting, transport and waiting again at every stage, and about twelve are
in process at once — a ceiling near 5 units/min against 4.6–5.5 measured.

Congestion is real but secondary: low-speed time clusters at the storage→carding, carding→drawing and
drawing→roving lanes (the last contains a 0.65 m pinch). The lever is **buffer capacity at each
stage**, not aisle width — the work-in-process count sets the ceiling directly, and widening an aisle
does not raise it while half the fleet sits idle. Fleet size, docking-slot count and a larger
in-flight cap were all measured and none of them helps.

![Live dashboard, four AGVs working](assets/readme/live-board-4agv.png)

*Live board from Gazebo — `web:=true`, then `http://127.0.0.1:8080`; `/stream` and `/map/stream` are multipart streams carrying PNG frames.*

5 policies × 3 seeds × 120 s, 80 units in circulation, identical plant — only the fleet differs.

> **Status of these numbers.** They were measured before the traffic layer and the
> controller were put on a single collision criterion, and I have not been able to
> reproduce them under the current code (a fresh 80-unit, 8-AGV, `ca_ssi` run lands
> at 6–8 tasks/min rather than 18.7). The table is left in place as the historical
> record; treat it as unverified until the set is re-run. See
> [docs/gazebo-throughput-findings.md](docs/gazebo-throughput-findings.md).

![Throughput, latency, travel per task and utilisation at two fleet sizes](assets/readme/policy-comparison.png)

*Error bars 1σ; percentages relative to `random`.*

| Policy | 3 AGVs: tasks/min | 3 AGVs: m/task | 8 AGVs: tasks/min | 8 AGVs: m/task | 8 AGVs: near-miss |
|---|---:|---:|---:|---:|---:|
| random | 9.0 | 12.2 | 10.6 | 13.8 | 6.7 |
| nearest | 10.4 | 9.8 | 12.3 | 11.7 | 4.3 |
| ssi | 10.0 | 10.1 | 15.2 | 12.9 | 6.3 |
| **ca_ssi** | **10.7** | **9.1** | **18.7** | **10.8** | **3.0** |
| hungarian | 9.9 | 10.4 | 14.9 | 11.9 | 6.5 |

**Before → after.** Same code, only `ca_ssi_cost` changed.

| Metric | Fleet | SSI | CA-SSI | Change | vs `random` |
|---|---|---:|---:|---:|---:|
| Throughput | 3 AGVs | 10.0 | 10.7 | **+7 %** | +18 % |
| Throughput | 8 AGVs | 15.2 | 18.7 | **+23 %** | +77 % |
| Travel per task | 3 AGVs | 10.1 | 9.1 | **−10 %** | −25 % |
| Travel per task | 8 AGVs | 12.9 | 10.8 | **−16 %** | −22 % |
| Near-miss events | 8 AGVs | 6.3 | 3.0 | **−53 %** | −55 % |

The edge grows with contention: three vehicles rarely want the same single-berth station, eight do it
constantly. `hungarian` optimises a static matrix and cannot see contention, energy or queueing — an
optimal assignment is not an optimal system. With only 28 units the plant is WIP-starved, every policy
converges to ≈16 tasks/min and the choice of rule is not measurable at all.

---

## Ablation

Each term removed on its own; same auction, seeds and plant.

![Leave-one-out ablation of the six cost terms](assets/readme/ablation.png)

*8 AGVs, 3 seeds, 80 units.*

| Cost model | tasks/min | vs full | m/task |
|---|---:|---:|---:|
| **CA-SSI (all six)** | **17.45** | — | **11.3** |
| − dock contention (γ) | 13.92 | **−20 %** | 12.6 |
| − energy feasibility (δ) | 14.63 | **−16 %** | 12.3 |
| − load balance (η) | 17.29 | −1 % | 11.6 |
| − task ageing (ζ) | 17.32 | −1 % | 10.9 |

Two terms carry the model. Load balance and ageing are **nulls on this workload** — 120 s is too short
to build an odometer advantage, and an anti-starvation term bounds the worst case rather than raising
the average. Both are kept; neither is claimed to pay for itself here.

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
operations, zero hull intersections, zero false stalls, no crashes.** The spread across identical
commands is the honest part of that line: output varies about twofold.

**Why centre distance is the wrong test.** The body is 0.56 × 0.44 m: two vehicles need 0.44 m
centre distance side by side, 0.56 m nose to tail, and 0.712 m to be safe at *any* orientation. The
code used 0.34 m and applied it only inside a ±52° front cone, so it commanded vehicles into each
other and ignored anything off to the side. Gazebo showed it — 1900 overlap events, closest approach
0.259 m. It is now a separating-axis test between the two oriented rectangles.

**Two root causes found this round, both invisible from the logs.**

The first: `/robot_i/odom` had incompatible QoS. The bridge publishes RELIABLE, the controller
subscribed BEST_EFFORT, and under DDS that pair delivers *nothing* — not fewer frames, none. Every
layer above the driver therefore reasoned about a robot that had not moved since spawn, and the
pose-difference speed derived from it was identically zero, so the coordinator concluded the whole
fleet was permanently stopped and fired 96–232 false stall recoveries per run. After the fix, four
robots roughly doubled their output and false stalls went to zero.

The second: the coordinator judged "is this robot stopped?" from the *last commanded* velocity. A
robot that stops stops being commanded, so the field froze at its pre-stop value and an idle vehicle
reported 0.85 m/s. Wait detection, anti-starvation priority and yield decisions were all dead code
while the fleet was actually blocked.

**What limits Gazebo is partly not the fleet.** Only four empty-can docking slots exist, so at most
four tasks can start concurrently and a new can enters only when one advances a stage. A dashboard
capture at the end of a run showed 12 of 16 cans still in the store. The deliveries-per-minute
figure therefore mixes can supply, the plant's internal rate (about 8 finished units/min, set by the
middle stage) and transport. Fleet-size comparisons remain valid because all arms share that supply
limit, but the absolute number is not a measure of fleet capability.

**Retracted.** An earlier version of this section concluded that Gazebo was not a usable throughput
demonstrator and that the chassis did not reproduce commanded motion. Both were wrong. A commanded
0.6 m/s over 8 s moves 4.085 m natively and 3.952 m through ROS — 85 % and 82 % of ideal, ordinary
acceleration and settling — and the real faults were the QoS mismatch and the speed reporting above.

The animation at the top of this page predates those fixes, so its vehicles pass closer than physics
now permits.

---

## Reproducing

```bash
# experiments
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
