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
from launch.conditions import IfCondition, LaunchConfigurationEquals  # noqa: F401
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
        # 待命区贴着下墙一字排开，避开所有取放位与生产通道
        poses = layout.park_poses(n)
        x, y, _yaw = poses[i]
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
        # 每车一个 robot_state_publisher，frame_prefix 让 TF 帧互不冲突
        actions.append(
            Node(
                package="robot_state_publisher", executable="robot_state_publisher",
                name="robot_state_publisher", namespace=f"robot_{i}", output="log",
                parameters=[dict(robot_description=ParameterValue(urdf, value_type=str),
                                 frame_prefix=f"robot_{i}/",
                                 use_sim_time=use_sim_time)],
            )
        )
        actions.append(
            Node(
                package=PKG, executable="robot_controller", name="robot_controller",
                namespace=f"robot_{i}", output="screen",
                parameters=[dict(robot_id=i, start_x=x, start_y=y,
                                  battery_drain_per_m=float(LaunchConfiguration("battery_drain").perform(context)),
                                  stuck_timeout_s=float(LaunchConfiguration("stuck_timeout").perform(context)),
                                  debug_avoid=LaunchConfiguration("debug_avoid").perform(context).lower() == "true",
                                  debug_scan_dir=LaunchConfiguration("debug_scan_dir").perform(context),
                                  start_stagger_s=float(LaunchConfiguration("start_stagger").perform(context)),
                                  avoid_cones=LaunchConfiguration("avoid_cones").perform(context).lower() == "true",
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
    # 只允许**一个** Gazebo 进程，两种模式二选一：
    #   gui:=false（默认）—— 只起服务端并加 --headless-rendering，供 CI 与批量截图
    #   gui:=true         —— 起带界面的完整仿真
    #
    # 以前 gui 与 headless 是两个互不相干的开关，于是 `gui:=true` 而 headless 仍取默认
    # true 时会**同时起两个服务端**：界面很可能连到那个带 --headless-rendering 的服务端，
    # 表现就是窗口打开但什么都渲染不出来。只设 `headless:=false` 则两个都不起。
    # 现在 headless 只作为兼容别名保留，取 false 等同于要界面。
    gui_on = PythonExpression(
        ["'", LaunchConfiguration("gui"), "' == 'true' or '",
         LaunchConfiguration("headless"), "' == 'false'"])
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", world],
        output="log",
        condition=IfCondition(PythonExpression(["not (", gui_on, ")"])),
    )
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-r", world], output="log",
                            condition=IfCondition(gui_on))

    # ---- 话题桥接 ----
    # **方向必须显式写出来**，这是本项目最贵的一个坑：
    #   `A@B@C` = 双向    `A@B]C` = ROS->GZ    `A@B[C` = GZ->ROS
    # 一开始全部写成 `@..@..@`，于是 cmd_vel 变成双向：控制器发到 ROS 的指令被
    # 桥送进 GZ，同一座桥的 GZ->ROS 方向又把 GZ 上的这条指令发回 ROS，
    # 而它立刻又被 ROS->GZ 方向送回 GZ —— 一个**自激回路**，指令条数每一步翻倍。
    # 症状极具迷惑性：轮子转速是对的（odom 里 twist 就是 0.6），但车几乎不前进、
    # 里程计位姿在出生点附近乱跳，还伴随 "A message was lost"。
    # 上游话题一律单向；只有 /clock 也只需要 GZ->ROS。
    # 相机的桥接是可选的：ros_gz_bridge 桥接 Image 时若订阅端跟不上，图像缓冲会
    # 持续增长（实测涨到 ~5GB 触发 OOM killer）。所以默认只桥接非图像话题；
    # 需要截图时显式打开 bridge_cameras:=true，且建议一次只开一路
    # （见 tools/capture_views.py 与 assets/readme 的生成步骤）。
    def _bridge(context, *args, **kwargs):
        topics = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        if LaunchConfiguration("bridge_cameras").perform(context).lower() == "true":
            topics += [
                "/view_top/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/view_iso/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/view_line/image@sensor_msgs/msg/Image[gz.msgs.Image",
            ]
        n = 6  # 固定桥接上限，多余的车不会报错（桥接未出现的话题会等待）
        for i in range(n):
            topics += [
                # 下行：唯一的指令通道
                f"/robot_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                # 上行：状态与传感
                f"/robot_{i}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                f"/robot_{i}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"/robot_{i}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                f"/robot_{i}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            ]
        return [Node(package="ros_gz_bridge", executable="parameter_bridge",
                     name="ros_gz_bridge", output="screen", arguments=topics)]

    # 实时网页看板：与 Gazebo 同源话题，便于并排比对映射是否正确
    web_node = Node(
        package=PKG, executable="live_view", name="fleet_live_view", output="screen",
        parameters=[dict(port=LaunchConfiguration("web_port"),
                         size=LaunchConfiguration("web_size"),
                         refresh_s=LaunchConfiguration("web_every"))],
        condition=IfCondition(LaunchConfiguration("web")),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    core = [
        Node(package=PKG, executable="factory_manager", name="factory_manager", output="screen",
             parameters=[dict(num_materials=LaunchConfiguration("num_materials"),
                              max_tasks_in_flight=LaunchConfiguration("max_tasks_in_flight"),
                              use_sim_time=use_sim_time)]),
        Node(package=PKG, executable="traffic_manager", name="traffic_manager", output="screen",
             parameters=[dict(use_sim_time=use_sim_time,
                              num_robots=LaunchConfiguration("num_robots"),
                              collision_criterion=LaunchConfiguration("collision_criterion"),
                              escape_strategy=LaunchConfiguration("escape_strategy"))]),
        Node(package=PKG, executable="task_scheduler", name="task_scheduler", output="screen",
             parameters=[dict(policy=LaunchConfiguration("policy"),
                              seed=LaunchConfiguration("seed"),
                              use_sim_time=use_sim_time)]),
        Node(package=PKG, executable="metrics", name="metrics_recorder", output="screen",
             parameters=[dict(out_dir=LaunchConfiguration("metrics_dir"),
                              policy=LaunchConfiguration("policy"),
                              run_label=LaunchConfiguration("run_label"),
                              use_sim_time=use_sim_time)]),
        Node(package=PKG, executable="dashboard", name="fleet_dashboard", output="screen",
             parameters=[dict(out_dir=LaunchConfiguration("out_dir"),
                              every_s=LaunchConfiguration("frame_every"),
                              use_sim_time=use_sim_time)]),
    ]

    return LaunchDescription([
        # 兼容旧写法：headless:=false 现在等价于 gui:=true
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="true = 带界面的 Gazebo；false = 无头服务端"),
        DeclareLaunchArgument("num_robots", default_value="4"),
        DeclareLaunchArgument("max_tasks_in_flight", default_value="6"),
        DeclareLaunchArgument("num_materials", default_value="12"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("out_dir", default_value="/tmp/fleetflow_frames"),
        DeclareLaunchArgument("frame_every", default_value="3.0"),
        DeclareLaunchArgument("stuck_timeout", default_value="10.0"),
        # 实时网页看板：gui:=true web:=true 可同时看 Gazebo 与 matplotlib 平面图
        DeclareLaunchArgument("web", default_value="false"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("web_every", default_value="0.25"),
        DeclareLaunchArgument("web_size", default_value="1280x800"),
        DeclareLaunchArgument("battery_drain", default_value="0.55"),
        DeclareLaunchArgument("policy", default_value="nearest"),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("bridge_cameras", default_value="false",
                              description="桥接 /view_*/image（截图用，注意桥接端内存）"),
        DeclareLaunchArgument("escape_strategy", default_value="gap",
                              description="gap=按净距最大化脱困（默认）；legacy=旧行为（对照用）"),
        DeclareLaunchArgument("collision_criterion", default_value="outline",
                              description="outline=轮廓净距（默认）；centre=旧车心距判据（对照实验用）"),
        DeclareLaunchArgument("avoid_cones", default_value="false",
                              description="开启控制器本地前方锥形互让（默认关，交给协调层）"),
        DeclareLaunchArgument("start_stagger", default_value="0.0",
                              description="出库错峰：第 i 台车等 i*该值秒再领任务"),
        DeclareLaunchArgument("debug_avoid", default_value="false",
                              description="打印每车 _avoid 分支计数（排障用，会刷屏）"),
        DeclareLaunchArgument("debug_scan_dir", default_value="",
                              description="把前方近障碍的原始 scan 帧写到此目录（排障用）"),
        DeclareLaunchArgument("run_label", default_value="gazebo"),
        DeclareLaunchArgument("metrics_dir", default_value="/tmp/fleetflow_metrics"),
        gz_headless, gz_gui, web_node, OpaqueFunction(function=_bridge),
        *core,
        OpaqueFunction(function=_robots),
    ])
