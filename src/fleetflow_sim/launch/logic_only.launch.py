"""纯逻辑模式：不启动 Gazebo，只跑调度 / 交通 / 车队 / 指标。

用途
----
* 在没有仿真器或 GPU 的机器上验证多机逻辑（CI 友好）
* 跑分配策略对比实验：Gazebo 渲染一帧的成本远高于调度本身，
  做 3 组 × 多轮对比时用它可以把单轮时间从分钟级压到几十秒

    ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 policy:=ssi \\
        seconds:=120 out_dir:=/tmp/exp_ssi
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleetflow_sim import layout

PKG = "fleetflow_sim"


def _spawn(context, *args, **kwargs):
    n = int(LaunchConfiguration("num_robots").perform(context))
    out_dir = LaunchConfiguration("out_dir").perform(context)
    every = float(LaunchConfiguration("frame_every").perform(context))
    materials = int(LaunchConfiguration("num_materials").perform(context))
    policy = LaunchConfiguration("policy").perform(context)
    label = LaunchConfiguration("run_label").perform(context)
    metrics_dir = LaunchConfiguration("metrics_dir").perform(context)
    seed = int(LaunchConfiguration("seed").perform(context))
    no_dash = LaunchConfiguration("dashboard").perform(context).lower() == "true"

    actions = [
                SetEnvironmentVariable("FLEETFLOW_EMPTY_SLOTS",
                               LaunchConfiguration("empty_slots")),
Node(package=PKG, executable="factory_manager", name="factory_manager", output="screen",
             parameters=[dict(num_materials=materials,
                              stale_task_timeout_s=float(LaunchConfiguration("stale_task_timeout").perform(context)),
                              advance_first=LaunchConfiguration("advance_first").perform(context).lower() == "true",
                              max_tasks_in_flight=int(LaunchConfiguration("max_tasks_in_flight").perform(context)))]),
        Node(package=PKG, executable="traffic_manager", name="traffic_manager", output="screen",
             parameters=[dict(stall_horizon_s=float(
                 LaunchConfiguration("stall_horizon").perform(context)),
                 escape_election=LaunchConfiguration("escape_election").perform(context).lower() == "true")]),
        Node(package=PKG, executable="task_scheduler", name="task_scheduler", output="screen",
             parameters=[dict(policy=policy, seed=seed)]),
        Node(package=PKG, executable="metrics", name="metrics_recorder", output="screen",
             parameters=[dict(out_dir=metrics_dir, policy=policy, run_label=label,
                              num_robots=n)]),
    ]
    if no_dash:
        actions.append(
            Node(package=PKG, executable="dashboard", name="fleet_dashboard", output="screen",
                 parameters=[dict(out_dir=out_dir, every_s=every)])
        )
    for i in range(n):
        # 待命区贴着下墙一字排开，避开所有取放位与生产通道
        poses = layout.park_poses(n)
        x, y, _yaw = poses[i]
        actions.append(
            Node(package=PKG, executable="robot_controller", name="robot_controller",
                 namespace=f"robot_{i}", output="screen",
                 parameters=[dict(robot_id=i, start_x=x, start_y=y,
                                  battery_drain_per_m=float(LaunchConfiguration("battery_drain").perform(context)),
                                  stuck_timeout_s=float(LaunchConfiguration("stuck_timeout").perform(context)),
                                  cancel_on_factory=LaunchConfiguration("cancel_on_factory").perform(context).lower() == "true",
                                  true_speed_report=LaunchConfiguration("true_speed_report").perform(context).lower() == "true",
                                  use_internal_kinematics=True)])
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="8"),
        DeclareLaunchArgument("max_tasks_in_flight", default_value="6"),
        DeclareLaunchArgument("num_materials", default_value="12"),
        DeclareLaunchArgument("stuck_timeout", default_value="10.0"),
        DeclareLaunchArgument("battery_drain", default_value="0.55"),
        DeclareLaunchArgument("cancel_on_factory", default_value="true"),
        DeclareLaunchArgument("advance_first", default_value="false",
                              description="先推进完工位再投料（false = 旧顺序）"),
        DeclareLaunchArgument("empty_slots", default_value="4",
                              description="空筒取放位数量（= 喂料并发上限），4~8"),
        # 默认 false：实测"让一台车原地让位"会把通道堵死，
        # 卡死时长从 3.6% 恶化到 15.2%（见 findings 第 62 节）。
        DeclareLaunchArgument("escape_election", default_value="false"),
        DeclareLaunchArgument("stall_horizon", default_value="8.0"),
        DeclareLaunchArgument("true_speed_report", default_value="true"),
        DeclareLaunchArgument("stale_task_timeout", default_value="45.0"),
        DeclareLaunchArgument("policy", default_value="nearest"),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("run_label", default_value="logic"),
        DeclareLaunchArgument("out_dir", default_value="/tmp/fleetflow_frames"),
        DeclareLaunchArgument("metrics_dir", default_value="/tmp/fleetflow_metrics"),
        DeclareLaunchArgument("frame_every", default_value="2.0"),
        DeclareLaunchArgument("dashboard", default_value="true"),
        OpaqueFunction(function=_spawn),
    ])
