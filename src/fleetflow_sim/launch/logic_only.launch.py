"""纯逻辑模式：不启动 Gazebo，只跑调度与车队（控制器自行积分运动学）。

用途
----
* 在没有仿真器 / GPU 的机器上验证调度逻辑（CI 友好）
* 低成本批量出态势图

    ros2 launch fleetflow_sim logic_only.launch.py num_robots:=8 seconds:=60
"""
from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleetflow_sim import layout

PKG = "fleetflow_sim"


def _spawn(context, *args, **kwargs):
    n = int(LaunchConfiguration("num_robots").perform(context))
    out_dir = LaunchConfiguration("out_dir").perform(context)
    every = LaunchConfiguration("frame_every").perform(context)
    materials = LaunchConfiguration("num_materials").perform(context)
    actions = [
        Node(package=PKG, executable="factory_manager", name="factory_manager", output="screen",
             parameters=[dict(num_materials=int(materials))]),
        Node(package=PKG, executable="task_scheduler", name="task_scheduler", output="screen"),
        Node(package=PKG, executable="dashboard", name="fleet_dashboard", output="screen",
             parameters=[dict(out_dir=out_dir, every_s=float(every))]),
    ]
    for i in range(n):
        x = 1.0 + 0.55 * (i % 2)
        y = 1.6 + 1.05 * (i // 2)
        actions.append(
            Node(package=PKG, executable="robot_controller", name="robot_controller",
                 namespace=f"robot_{i}", output="screen",
                 parameters=[dict(robot_id=i, start_x=x, start_y=y,
                                  use_internal_kinematics=True)])
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="8"),
        DeclareLaunchArgument("num_materials", default_value="12"),
        DeclareLaunchArgument("out_dir", default_value="/tmp/fleetflow_frames"),
        DeclareLaunchArgument("frame_every", default_value="2.0"),
        OpaqueFunction(function=_spawn),
    ])
