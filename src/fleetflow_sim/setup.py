from glob import glob
import os

from setuptools import find_packages, setup

package_name = "fleetflow_sim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="QuanQuan",
    maintainer_email="129049044+p20030920p@users.noreply.github.com",
    description="Multi-AGV material transport simulation for a textile mill (ROS 2 Jazzy + Gazebo Sim 8).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "factory_manager = fleetflow_sim.factory_manager:main",
            "task_scheduler = fleetflow_sim.task_scheduler:main",
            "robot_controller = fleetflow_sim.robot_controller:main",
            "dashboard = fleetflow_sim.dashboard:main",
            "traffic_manager = fleetflow_sim.traffic_manager:main",
            "metrics = fleetflow_sim.metrics:main",
            "live_view = fleetflow_sim.live_view:main",
        ],
    },
)
