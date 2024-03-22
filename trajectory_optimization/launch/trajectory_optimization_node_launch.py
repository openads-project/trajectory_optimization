#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("trajectory_optimization"),
        "config",
        "params.yaml"
    )

    launch_file = os.path.join(
        get_package_share_directory("demo_trajectory_pub"),
        "launch",
        "demo_trajectory_pub_node.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file)
        ),
        Node(
            package="trajectory_optimization",
            executable="trajectory_optimization_node",
            name="trajectory_optimization_node",
            output="screen",
            emulate_tty=True,
            parameters=[config],
            remappings=[
                ("~/reference_trajectory", "/demo_trajectory_pub_node/demo_trajectory")
            ]
        )
    ])
