int32 robot_id
string state            # idle / to_pickup / to_dropoff / loading / unloading / charging
float64 x
float64 y
float64 yaw
int32 task_id           # -1 = 空闲
float64 distance_done
float64 distance_total
float64 battery          # 电量百分比 0-100
float64 odom_total       # 累计行驶里程 m（用于负载均衡）

# --- 交通协调层需要、但原来没有上报的状态 ---
# 缺了这两项，协调层就无法重现 ROS 1 的两个关键判据：
#   * 等待判定（反饥饿优先级增益）要知道车是不是真的停着；
#   * 冲突裁决（对向谁先行）与卡死自愈要知道当前阶段目标点在哪。
float64 speed            # 当前线速度 m/s
bool has_target          # 当前阶段是否有目标点（装卸/充电中为 false）
float64 target_x
float64 target_y
