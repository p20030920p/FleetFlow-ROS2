"""QoS 分型。

真实机器人系统不会所有话题都用同一套 QoS：
* 传感器流（scan / odom）丢一帧无所谓，用 best-effort + 小队列，避免慢订阅者拖累发布端；
* 指令与状态（cmd_vel / 任务 / 车队状态）必须可靠，用 reliable + 较深队列；
* 参数与地图这类"晚加入也要拿到"的数据用 transient_local。
"""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def sensor_qos(depth: int = 5) -> QoSProfile:
    """高频传感器流：允许丢帧，队列浅。

    **可靠性必须是 RELIABLE，尽管名字叫 sensor。**
    这是本项目踩得最深的一个坑，第 47 节记录：`ros_gz_bridge` 出来
    的 `/robot_i/odom` 与 `/scan` 都是 **RELIABLE** 发布者，而这里原来
    用 BEST_EFFORT 订阅 —— 在 DDS 里 RELIABLE 发布者 + BEST_EFFORT
    订阅者 = **QoS 不兼容，一条都收不到**（`ros2 topic info -v` 能直接
    看到 Publisher: RELIABLE / Subscription: BEST_EFFORT）。

    后果不是"丢几帧"，而是彻底失聪：
      * `on_odom` 永不触发 -> 车不知道自己真实位姿，交通协调层、规划器、
        纯追踪全都建立在出生点上；
      * `_meas_speed`（位姿差分）恒为 0 -> 协调层判定"所有车永久停车"
        -> 超过 8 s 全员触发 `stalled_replan`，Gazebo 里 300 s 出现
        96~232 次假卡死，车被反复要求重规划、原地打转。

    之前 scan 的同类问题只修了 scan 那一个订阅，没意识到 odom 也超长
    受害（同样的 `pkill -x parameter_bridge` 教训）。这里一次性修根。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def command_qos(depth: int = 10) -> QoSProfile:
    """控制指令：可靠传输，队列适中。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def state_qos(depth: int = 20) -> QoSProfile:
    """状态广播（车队状态、任务状态）：可靠 + 深队列，容忍突发。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def latched_qos(depth: int = 1) -> QoSProfile:
    """只关心"最新一版"的配置类话题。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
