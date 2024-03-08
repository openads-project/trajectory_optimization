#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("trajectory_optimization"),
        "config",
        "params.yaml"
    )


    return LaunchDescription([
        Node(
            package="trajectory_optimization",
            executable="trajectory_optimization_node",
            name="trajectory_optimization_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
