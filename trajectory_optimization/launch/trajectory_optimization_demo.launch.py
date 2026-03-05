#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    trajectory_optimization_launch_file = os.path.join(
        get_package_share_directory("trajectory_optimization"),
        "launch",
        "trajectory_optimization.launch.py"
    )

    demo_pub_launch_file = os.path.join(
        get_package_share_directory("demo_trajectory_pub"),
        "launch",
        "demo_trajectory_pub_node.launch.py"
    )
    
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(trajectory_optimization_launch_file),
            launch_arguments=[
                ("reference_trajectory_topic", "/demo_trajectory_pub_node/reference_trajectory"),
                ("ego_data_topic", "/demo_trajectory_pub_node/ego_data"),
                ("object_list_topic", "/demo_trajectory_pub_node/object_list")
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_pub_launch_file)
        ),
        static_transform_publisher
    ])
