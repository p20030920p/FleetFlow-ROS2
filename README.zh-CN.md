<p align="right">
  <a href="./README.md">English</a> · <strong>简体中文</strong>
</p>

# FleetFlow-ROS2

**纺织厂多 AGV 物料搬运仿真** · ROS 2 Jazzy + Gazebo Sim 8

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Gazebo%20Sim-8-orange" alt="Gazebo Sim 8">
  <img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white" alt="Python 3.12">
</p>

<p align="center">
  <img src="./assets/readme/demo.gif" width="100%" alt="一次完整搬运循环：任务以空心方框出现，调度器逐单派车，车辆驶向取货位、送达卸货位，完成数上升，物料流转计数从空筒推进到生条、熟条、粗纱成品">
</p>

<p align="center">
  <sub><b>一次完整循环：从发放任务到送达。</b>任务出现 · 拍卖派车 · 车辆行驶与停靠 · 计数推进。
  画面取自真实运行，6 倍速播放。</sub>
</p>

多台 AGV 在梳棉、并条、粗纱机器之间搬运物料桶。调度器按策略派发运输任务，每台车自己规划
路径并执行，车间生产看板按真实工厂的方式显示车队状态、机台状态与各工序进度。

## 快速开始

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/p20030920p/FleetFlow-ROS2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 完整仿真（Gazebo 无头运行，每 3 秒输出一帧生产看板）
ros2 launch fleetflow_sim factory.launch.py

# 想开 Gazebo 界面
ros2 launch fleetflow_sim factory.launch.py headless:=false gui:=true

# 只跑调度逻辑：不启 Gazebo、不需要 GPU。CI 和下面的实验用的就是它
ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi
```

### 抓取机位图

相机话题**默认不桥接**，这个默认值是刻意的：`ros_gz_bridge` 桥接 `sensor_msgs/Image`
时，一旦订阅端跟不上，图像缓冲会持续增长 —— 实测涨到 **约 5 GB RSS**，足以触发 OOM
killer 把机器打挂。所以相机跑在 2 Hz，并且一次只桥接一路，只在真的需要出图时才开：

```bash
ros2 launch fleetflow_sim factory.launch.py bridge_cameras:=true   # 显式打开
# 另开一个终端：只桥一路，订阅端 best-effort/depth-1，抓完就杀掉桥接
ros2 run ros_gz_bridge parameter_bridge \
    "/view_iso/image@sensor_msgs/msg/Image@gz.msgs.Image" &
python3 tools/capture_views.py /tmp/shots /view_iso/image
kill %1
```

`tools/capture_views.py` 用 `BEST_EFFORT` + `depth=1` 订阅，也是同一个原因：
它只要最新的一帧，队列再深就是泄漏。

## 车间

<p align="center">
  <img src="./assets/readme/gazebo-iso.png" width="49%" alt="Gazebo 棉纺车间的 3/4 剖视：梳棉/并条/粗纱机弄、两侧条筒货架、架空巡回清洁轨道">
  <img src="./assets/readme/gazebo-top.png" width="49%" alt="车间俯视图，可见完整工序布局与通道上的 AGV">
  <img src="./assets/readme/gazebo-line.png" width="49%" alt="沿产线的低角度视角，AGV 在梳棉机之间">
  <img src="./assets/readme/gazebo-machine.png" width="49%" alt="梳棉机近景：控制面板、防护罩与警示标线">
</p>

## 生产看板

<p align="center">
  <img src="./assets/readme/control-center.png" width="100%" alt="FleetFlow 生产调度看板：KPI 带、物料流转条、工程制图风格的车间平面图（含 AGV 与停靠位）、设备利用率、机台状态与车辆状态表">
</p>

每次运行还会渲染一块**班组生产看板** —— 也就是车间真正会盯着看的那块屏：物料流转条、
工程制图风格的车间平面图、带 100% 参考线的设备利用率，以及逐台的车辆状态表。
它就是一个普通的 matplotlib 节点，可以无头运行、定时出图。

## 实际跑起来的东西

<p align="center">
  <img src="./assets/readme/architecture.png" width="100%" alt="运行时拓扑：factory_manager、task_scheduler、traffic_manager、N 个带命名空间的 robot_controller、ros_gz_bridge 边界与 Gazebo Sim">
</p>

在同一个 Gazebo 世界里生成几个模型是简单的部分。让一支车队表现得像车队，靠的是它周围的一切——这些在这里都有真实实现，不是占位：

| 关注点 | 实际实现 |
| --- | --- |
| **命名空间与 TF** | 每台车跑在自己的命名空间下，配独立的 `robot_state_publisher` 与 `frame_prefix`，于是 TF 是一片互不干扰的森林（`robot_0/odom → robot_0/base_footprint → …`），而不是一条互相争夺的链。 |
| **与数据匹配的 QoS** | 传感器流（scan、里程计）用 best-effort + 浅队列，慢订阅者拖不住发布端；指令、任务与车队状态用 reliable + 较深队列。 |
| **交通管理** | 停靠由显式的租约服务串行化：进场前必须持有租约，物理驶离后才释放。每台车**同时最多持有一个**租约，且不会"抱着一个去抢另一个"，等待都发生在开阔地面上——这直接破坏了死锁的 *hold-and-wait* 条件，循环等待无法形成。租约带 TTL，车辆异常退出后工位自动回收。 |
| **多停靠位** | 每个料区不是只有一个共享坐标，而是在圆周上划出多个**停靠位**。没有它时所有车都往同一个点挤，互让逻辑会互相锁死、看门狗只能中止任务——这正是促使这次重构的失败模式。 |
| **车车互相避让** | 每台车都能看到同伴位姿并作出让步：进入前向锥形区减速、对高优先级车辆停车、并在硬安全距离处无条件停住。优先级确定性（id 小者先行）用来打破对称僵局。每次调用 A\* 之前其他车辆还会作为动态障碍层注入，路径直接绕开交通，而不是只靠局部反应。 |
| **LiDAR 安全层** | 独立监视前向扇区，紧急半径内出现任何东西都直接停车，不受规划器意愿影响。 |
| **电量与恢复** | 电量按行驶里程消耗。低于阈值时车辆停止投标，通过与工位同一套租约机制排队进入充电位，充满后归队。长时间无位移的车辆先重规划、再放弃任务；调度器另外回收"停止上报"车辆手里的任务。两者都计入 reassignment。 |
| **拉取式分配** | 车辆只在空闲时主动要任务，从机制上消除 push 模型的双重分配竞态。 |

## 分配问题，以及这份代码怎么打

把运输任务指派给车辆，是多机器人任务分配（**MRTA**）问题的一个实例。
设 `T` 为待办任务集合、`R` 为空闲车辆集合，`s_t`/`g_t` 为任务 `t` 的取货/放货点，
`p_r` 为车辆 `r` 的位姿。教科书里的边际代价只有一个空驶距离：

```
c(r, t) = ‖ p_r − s_t ‖₂
```

这个代价在仓储通道里是对的，**在棉纺车间里是错的**，有三个具体原因，也是这份代码必须处理的三件事：

1. **一台机台只有一个对接位。** 两台车同时被派往同一台梳棉机不是"分担工作"，
   而是一台等另一台，等待还会沿着产线往上游传导。只认距离的拍卖看不见这件事，
   于是会毫不犹豫地派出第三台。
2. **电量是硬约束，不是软约束。** 一台车接下它跑不完的任务，不会优雅降级 ——
   它会把物料撂在半路，逼出一次任务回收重派。
3. **此刻最便宜的车不等于一个班次里最便宜的车队。** 不看累计里程，就会把活
   全压在那台恰好停在取货区旁边的车上。

### 五种策略，一条对比链

| 策略 | 规则 | 使用的信息 | 定位 |
| --- | --- | --- | --- |
| `random` | 请求车辆随机取一个待办任务 | 无 | 下界基线 |
| `nearest` | 请求车辆取离自己最近的任务 | 仅局部，完全去中心化 | 常规方法 |
| `ssi` | 顺序单件拍卖：反复把全局代价最小的 `(r,t)` 判给中标者 | 全局位姿 + 任务集 | 市场拍卖类标准方法 |
| **`ca_ssi`** | **同一套拍卖，换成六项工业代价** | **+ 工位拥塞、电量、负载、任务老化** | **本文方法** |
| `hungarian` | 对同一代价矩阵求匈牙利最优指派 | 全局、单轮 | 单轮最优参照 |

`ssi` 属于市场拍卖类 MRTA 方法（single-item auction，Lagoudakis et al., 2005）。
`ca_ssi` **完全保留这套拍卖机制**，只替换代价函数 —— 所以下文的对比隔离出来的是
"代价模型"这一个变量，而不是算法类别的差异。

### CA-SSI 的代价函数

<p align="center">
  <img src="./assets/readme/cost-model.png" width="100%" alt="CA-SSI 代价函数的六项构成及其等效米权重">
</p>

```
J(r,t) = α·‖p_r − s_t‖                    ① 空驶
       + β·‖s_t − g_t‖                    ② 载货行驶
       + γ·(n_src + 1.5·n_dst)            ③ 工位拥塞
       + δ·max(0, e_need + reserve − e_r) ④ 电量可达性
       + η·(d_r − d̄)/d_max                ⑤ 负载均衡
       − ζ·age(t)                         ⑥ 任务老化      (+ 0.02·优先级)
```

每一项都折算成**等效米**，所以权重是可读的：让一台车多绕过"一个已被占用的工位"，
等价于多开 `γ/α = 6` 米。`n_src`、`n_dst` 是正在赶往（或已排在）取货位、放货位的车辆数；
放货位权重 ×1.5，因为卸不掉货的车会堵住通道，而不只是自己多等一会。
`e_need` 是跑完这单所需电量，`reserve` 是强制安全余量，`d_r` 是该车累计里程，`d̄` 是车队均值。

任务老化项是防饿死项：没有它，角落里一条低优先级任务永远不可能是 argmin，会无限期等下去。
它的权重刻意给得很小 —— 刚好保证最终会被服务，又不至于扭曲路径选择。

拍卖主循环只有三行：给所有 `(r,t)` 打分 → 取全局 argmin 成交 → 双方出池 → 重复。
每一轮代价 `O(|R|·|T|)`，在本规模下是微秒级 —— 整个调度器比一次激光扫描还便宜。

## 实验结果

两个工况跑的是**同一座工厂、同一套物料模型**（在制 80 件），只有车队规模不同。这是刻意的，
因为这样才隔离出结论真正依赖的那个变量：

> **分配策略的价值随"竞争强度"增长，而不是随车队规模本身增长。**

<p align="center">
  <img src="./assets/readme/policy-comparison.png" width="100%" alt="五种分配策略在小车队与大队列下的吞吐、时延、单任务里程与利用率对比">
</p>

5 种策略 × 3 个种子 × 120 秒，`num_materials:=80`、`max_tasks_in_flight:=18`，随机种子固定，
并**关掉充电干扰**（把耗电调低，避免它混淆分配策略这一个变量）。

**工况 A —— 小车队**（3 台 AGV，利用率 ≈ 0.98，对接位竞争少）

| 策略 | 完成数 | 吞吐/分 | 平均时延（秒） | 每单行驶（米） |
| --- | ---: | ---: | ---: | ---: |
| random | 18.0 | 9.0 | 18.7 | 12.2 |
| nearest | 20.7 | 10.4 | 16.1 | 9.8 |
| ssi | 20.0 | 10.0 | 16.2 | 10.1 |
| **ca_ssi** | **21.3** | **10.7** | **15.8** | **9.1** |
| hungarian | 19.7 | 9.9 | 16.9 | 10.4 |

**工况 B —— 大队列**（8 台 AGV，利用率 ≈ 0.73，对接位竞争激烈）

| 策略 | 完成数 | 吞吐/分 | 平均时延（秒） | 每单行驶（米） | 近距事件 |
| --- | ---: | ---: | ---: | ---: | ---: |
| random | 21.0 | 10.6 | 19.7 | 13.8 | 6.7 |
| nearest | 24.3 | 12.3 | 17.9 | 11.7 | 4.3 |
| ssi | 30.0 | 15.2 | 18.7 | 12.9 | 6.3 |
| **ca_ssi** | **37.0** | **18.7** | **17.1** | **10.8** | **3.0** |
| hungarian | 29.5 | 14.9 | 19.2 | 11.9 | 6.5 |

### 改进前 → 改进后

"改进前"是文献里已有的、只认距离的拍卖；"改进后"是**同一套拍卖循环**换上六项工业代价。
两行是同一份代码，只改了 `policies.ca_ssi_cost`。

| 指标 | 车队 | SSI（改进前） | CA-SSI（改进后） | 变化 | 相对 `random` |
| --- | --- | ---: | ---: | ---: | ---: |
| 吞吐（件/分钟） | 3 台 | 10.0 | 10.7 | **+7 %** | +18 % |
| 吞吐（件/分钟） | 8 台 | 15.2 | 18.7 | **+23 %** | +77 % |
| 每单行驶（米） | 3 台 | 10.1 | 9.1 | **−10 %** | −25 % |
| 每单行驶（米） | 8 台 | 12.9 | 10.8 | **−16 %** | −22 % |
| 每轮近距事件 | 8 台 | 6.3 | 3.0 | **−53 %** | −55 % |

### 这些数字说明什么

1. **优势随竞争强度增长，与代价模型的预测一致。** 只有 3 台车时，这张平面图很少出现
   "两台车同时要进同一个单泊位工位"，所以建模拥塞只换来 **+7 %** 吞吐；同样一张图放 8 台车，
   这种撞车是常态，同一项代价就值 **+23 %** 吞吐、**−16 %** 单任务里程，并把近距事件
   **减少一半**（6.3 → 3.0）。**CA-SSI 是这里唯一能"看见自己即将制造的拥塞"的策略。**
2. **收益来自代价函数，不来自拍卖机制。** `ssi` 与 `ca_ssi` 跑的是**完全相同**的拍卖循环 ——
   给所有 `(r,t)` 打分、取全局 argmin 成交、重复。唯一差别是一次投标值多少钱，
   所以全部差距都可以归因到那六项上。
3. **`nearest` 并不稳定地优于 `random`。** 它是"谁先来问就给谁最近的单"，
   于是一台车可能抢走本该属于另一台近得多的车的任务。
4. **`hungarian` 是单轮最优，但它从不获胜。** 它精确地最小化那张**静态**代价矩阵，
   却不建模工位拥塞、电量与排队 —— 它优化错了目标：小车队 9.9 对 10.7，大队列 14.9 对 18.7。
   这是本仓库最可迁移的一条结论：**最优的分配方案 ≠ 最优的系统。**
5. **安全性遵循同一个机制。** 租约层与互让层在每一轮里都把最近车距守在 0.34 米硬限之上，
   而"给拥塞定价"的那个策略同时也正是让车彼此保持距离的那个。

### 如实的局限

- 3 个种子足以分辨这个量级的差距（+23 %、+77 %），但分辨不了 10 % 以内。
  工况 A 里前两名之后的排序应当视为噪声。
- **结论依赖负载，而这正是发现本身。** 早前一个只有 28 件在制物料的配置会让工厂"缺料饿死"：
  所有策略都收敛到约 16 件/分钟，用什么规则根本测不出来。所以上面两个工况都让工厂保持满载。
  **一份不写明负载条件的策略研究是不可复现的。**
- `ca_ssi` 是集中式的，前提是位姿、电量、机台状态都足够新；基于过期遥测做拍卖，优势会打折。
- `γ`（拥塞）与 `δ`（电量）是针对这张平面图手调的。它们**在性质上**可迁移 ——
  任何工厂都有对接位竞争 —— 但**数值上**不可直接照搬；换一个厂区需要一轮短调参，
  `tools/run_experiments.py` 就是那个调参台架。
- 电量项在 0.10 %/m 的耗电下几乎没被触发，它的单独贡献没有被这次实验测量。
  这次实验真正隔离出来的是**拥塞项与老化项**。

## 展望：这套设计到底换来了什么

**一、车队最贵的是闲置，不是行驶。** 棉纺车间里节拍由机器决定，AGV 的职责是
"永远不要成为梳棉机停台的原因"。`J` 的每一项都是在给**工厂的**成本定价，而不是给车的成本定价。
拥塞项被给到空驶距离的 6 倍，正是因为：多开 6 米很便宜，停一台梳棉机不便宜。

**二、扩展时改的是代价，不是机制。** 拍卖循环是固定的，领域知识从代价函数进来。
新增一条约束 —— 经轴必须立式运输、交接班时某条通道是单行道、某台机器的工装要先冷却再上料 ——
意味着**加一项**，而不是重写调度器。这才让它是一个系统而不是一个演示：
真正有意思的工程在代价模型里，而这个代价模型是工艺工程师看得懂的，不需要他读代码。

**三、拥塞感知是可迁移的结果。** 实测出来的差距是**吞吐 +23%、近距事件减半**，
而它只来自基线方法无法表达的那一项代价。这一发现并不专属于纺织：所有存在单泊位资源的领域 ——
自动化码头、晶圆厂、医院物流、机场廊桥 —— 都是同一个结构，同一项代价同样适用。
本仓库的测量规模很小，但它隔离出来的机制是通用的，而且在其中任何一个领域
都可以用这套台架测出来。

**四、安全余量会复利。** 因为拥塞被写进目标函数而不是留给恢复策略，车辆在同一工位汇合的次数更少。
这个效应最先体现在近距事件上 —— 大队列工况下 6.3 → 3.0 —— 这也是整个效应里最便宜的一半测量。
汇合更少同时意味着更少的对向避让僵持、更少的看门狗介入，以及可以收紧的安全半径，
而安全半径收紧又会以更高的有效速度反馈回来。那个闭环是下一件值得测的事，
而不是这份代码已经证明的事。

**五、这里的每一个结论都可证伪。** README 里的每个数字都来自 `experiments/` 下的 CSV，
由一条命令、固定种子、有文档的台架产出。每个策略只有十行。
预期的用法就是：有人把 `ca_ssi` 换成自己的代价函数，重跑 `tools/run_experiments.py`，然后看结果。

## 复现这组对比实验

```bash
# 工况 A —— 车队受限期（3 台车）：策略在这里才有意义
python3 tools/run_experiments.py \
    --policies random nearest ssi ca_ssi hungarian \
    --seeds 1 2 3 --seconds 120 --robots 3 --drain 0.10 \
    --out experiments/saturated \
    --extra max_tasks_in_flight:=18 num_materials:=80

# 工况 B —— 同一座工厂，运力富裕（8 台车）
python3 tools/run_experiments.py \
    --policies random nearest ssi ca_ssi hungarian \
    --seeds 1 2 3 --seconds 120 --robots 8 --drain 0.10 \
    --out experiments/slack \
    --extra max_tasks_in_flight:=18 num_materials:=80

# 出图，然后把数字直接写回两个 README
python3 tools/plot_results.py experiments/saturated/summary.csv \
    experiments/slack/summary.csv assets/readme/policy-comparison.png
python3 tools/plot_cost_model.py assets/readme/cost-model.png
python3 tools/plot_architecture.py assets/readme/architecture.png
python3 tools/update_readme_numbers.py \
    experiments/saturated/summary.csv experiments/slack/summary.csv \
    --prefix sat_ slack_ --readme README.md README.zh-CN.md
```

每个 `(策略, 种子)` 组合在独立目录里跑一次，随机种子固定，因此策略本身与平局打破都可复现。
上表背后的原始 CSV 已提交在 `experiments/` 下，不重跑也能核对数字。

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

## 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/factory/tasks` | `TransportTask` | 新的搬运任务；周期性重播，晚启动的订阅者也不会漏 |
| `/factory/task_status` | `TransportTask` | 分配、完成与失败 |
| `/factory/completed` | `Int32` | 送达回执 |
| `/factory/machines` | `MachineState` | 状态、位置、进出料计数、繁忙率 |
| `/factory/summary` | `String`（JSON） | 生产看板渲染用的汇总 |
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
│   │   ├── policies.py         # random / nearest / SSI / CA-SSI / 匈牙利
│   │   ├── qos.py              # 按数据类别划分的 QoS
│   │   └── dashboard.py        # 生产调度看板（Andon / MES）渲染
│   ├── worlds/textile_factory.sdf   # 由 tools/build_world.py 生成
│   ├── urdf/agv.urdf.xacro
│   └── launch/                 # factory.launch.py · logic_only.launch.py
├── tools/
│   ├── build_world.py          # 生成 SDF 世界（525 个模型）
│   ├── run_experiments.py      # 策略对比实验台架
│   ├── capture_views.py        # Gazebo 机位 -> PNG
│   ├── plot_results.py         # 策略对比图
│   ├── plot_cost_model.py      # CA-SSI 代价函数图
│   ├── plot_architecture.py    # 运行时拓扑图
│   └── update_readme_numbers.py# 把实验统计写回 README
└── experiments/                # 原始 CSV（saturated/ 与 slack/）
```

## 说明与边界

- 调度刻意用 pull 模型。push 模型必须维护每台车的忙碌状态，一旦完成通知丢失，那台车就永久"忙碌"——
  这正是促使这次重构的 bug。pull 让这种失效模式不可能发生，剩余情况（车辆彻底不响应）由心跳看门狗兜住。
- `ca_ssi` 给"它看得见的"拥塞定了价 —— 即已经在赶往某工位的车。但它对尚未生成的任务没有前瞻，
  也是一次只派一单，而不是规划一条多单路线。滚动时域上的"带排序的拍卖"是下一步。
- `logic_only.launch.py` 不启 Gazebo 就能跑完整协同栈，上面那组策略对比就是这样在可接受的时间内复现的。
  该模式下里程计由本地积分得到，但协同逻辑与仿真模式完全一致。
- 电量参数调成了"一次短运行内即可观察到充电行为"。真实 AGV 单次充电能跑远得多：
  这里的电量模型是行为演示，不是能耗研究。

## 关于这些数字的来处

两种启动模式下的协同栈是完全相同的，但上面的测量都是用 `logic_only.launch.py` 得到的。
在没有独立显卡的机器上，Gazebo 的渲染会把实时因子压得很低 —— 525 个模型的世界加上 4 路常开相机，
足以让一次 200 秒的墙钟运行在"任务能生成、能派发、车能开、全程零报错"的情况下
一单都还没送达。此时每一个墙钟指标 —— 吞吐、时延、看门狗超时 ——
都会被一个与分配策略无关的系数缩放。把对比实验放到无渲染模式下跑，
才能保证"被测变量"是唯一的变量。

Gazebo 仍然是所有物理相关部分的基准：接触、里程计、LiDAR、多机生成与 TF 布局，
以及本页顶部的那些截图。想交互式跑完整栈，请把 `num_robots` 调小，并接受慢动作。
