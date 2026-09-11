"""一键启动 FleetFlow-ROS2 全系统。

    ros2 launch fleetflow_sim factory.launch.py                       # 无头 + 出图
    ros2 launch fleetflow_sim factory.launch.py headless:=false       # 带 Gazebo 界面
    ros2 launch fleetflow_sim factory.launch.py num_robots:=4

启动内容：
* Gazebo Sim 8（工厂世界，含 3 个抓图机位）
* num_robots 台 AGV（xacro 生成 → ros_gz_sim create 注入）
* ros_gz_bridge：/clock、每台车的 cmd_vel / odom / scan、3 路相机
* factory_manager / task_scheduler / robot_controller × N / dashboard
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from fleetflow_sim import layout

PKG = "fleetflow_sim"


def _robots(context, *args, **kwargs):
    share = get_package_share_directory(PKG)
    xacro_file = os.path.join(share, "urdf", "agv.urdf.xacro")
    n = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    actions = []

    # 出生点：厂房左侧一列，避免叠在一起
    for i in range(n):
        x = 1.0 + 0.55 * (i % 2)
        y = 1.6 + 1.05 * (i // 2)
        rgb = layout.FLEET_COLORS[i % len(layout.FLEET_COLORS)]
        color = f"{rgb[0]} {rgb[1]} {rgb[2]}"
        urdf = Command(["xacro ", xacro_file, f" robot_id:={i}", f' body_color:="{color}"'])
        actions.append(
            Node(
                package="ros_gz_sim", executable="create", output="log",
                arguments=["-name", f"robot_{i}", "-string", urdf,
                           "-x", str(x), "-y", str(y), "-z", "0.08"],
            )
        )
        actions.append(
            Node(
                package=PKG, executable="robot_controller", name="robot_controller",
                namespace=f"robot_{i}", output="screen",
                parameters=[dict(robot_id=i, start_x=x, start_y=y,
                                 use_sim_time=use_sim_time)],
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    world = os.path.join(share, "worlds", "textile_factory.sdf")

    headless = LaunchConfiguration("headless")
    num_robots = LaunchConfiguration("num_robots")

    # ---- Gazebo Sim 8 ----
    gz_args = ["-s", "-r"]
    gz_args.append("--headless-rendering")
    gz = ExecuteProcess(
        cmd=["gz", "sim", *gz_args, world],
        output="log",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-r", world], output="log",
                            condition=IfCondition(LaunchConfiguration("gui")))

    # ---- 话题桥接 ----
    bridge_topics = [
        "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        "/view_top/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/view_iso/image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/view_line/image@sensor_msgs/msg/Image@gz.msgs.Image",
    ]
    n = 6  # 固定桥接上限，多余的车不会报错（桥接未出现的话题会等待）
    for i in range(n):
        bridge_topics += [
            f"/robot_{i}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            f"/robot_{i}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            f"/robot_{i}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            f"/robot_{i}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        ]
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="ros_gz_bridge", output="screen", arguments=bridge_topics,
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    core = [
        Node(package=PKG, executable="factory_manager", name="factory_manager", output="screen",
             parameters=[dict(num_materials=LaunchConfiguration("num_materials"),
                              use_sim_time=use_sim_time)]),
        Node(package=PKG, executable="task_scheduler", name="task_scheduler", output="screen",
             parameters=[dict(use_sim_time=use_sim_time)]),
        Node(package=PKG, executable="dashboard", name="fleet_dashboard", output="screen",
             parameters=[dict(out_dir=LaunchConfiguration("out_dir"),
                              every_s=LaunchConfiguration("frame_every"),
                              use_sim_time=use_sim_time)]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("num_robots", default_value="4"),
        DeclareLaunchArgument("num_materials", default_value="12"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("out_dir", default_value="/tmp/fleetflow_frames"),
        DeclareLaunchArgument("frame_every", default_value="3.0"),
        gz, gz_gui, bridge,
        *core,
        OpaqueFunction(function=_robots),
    ])
