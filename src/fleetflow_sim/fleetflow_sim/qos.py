"""QoS 分型。

真实机器人系统不会所有话题都用同一套 QoS：
* 传感器流（scan / odom）丢一帧无所谓，用 best-effort + 小队列，避免慢订阅者拖累发布端；
* 指令与状态（cmd_vel / 任务 / 车队状态）必须可靠，用 reliable + 较深队列；
* 参数与地图这类"晚加入也要拿到"的数据用 transient_local。
"""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def sensor_qos(depth: int = 5) -> QoSProfile:
    """高频传感器流：允许丢帧，队列浅。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
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
