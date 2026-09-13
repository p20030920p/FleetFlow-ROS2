# 交通协调层 -> 单车控制器的执行指令。
#
# 为什么要有这一条话题：ROS 1 版里调度器和导航在同一个进程里，协调层直接写
# cmd_vel 就行。ROS 2 版把导航拆到了每台车的控制器里（它才拿得到 LiDAR 和
# 纯追踪状态），所以协调层改成"下指令、控制器执行"：
# 协调层仍然决定**谁让谁、什么时候走、往哪让**，控制器负责把这些约束变成
# 轮速。这样既保住了 ROS 1 的协调逻辑，又不用把导航搬回中心节点。
int32 robot_id

# 速度缩放：0~1，乘到控制器自己算出的线速度上（分级减速用）
float64 speed_scale

# 立即停住，优先级最高（让路 fallback / 死锁冷却期内）
bool hold

# 直接速度覆盖：仅用于紧急脱困（方向安全性由协调层判定）
bool has_escape
float64 escape_v
float64 escape_w

# 请求控制器按当前阶段目标重规划（脱困/让路结束后）
bool replan

# 让路临时路径。必须用 has_yield_path 显式区分"本次没有覆盖"与"覆盖已结束"，
# 否则控制器无法判断何时切回自己的任务路径。
bool has_yield_path
float64[] yield_x
float64[] yield_y

# 触发原因，纯观测用途
string reason
