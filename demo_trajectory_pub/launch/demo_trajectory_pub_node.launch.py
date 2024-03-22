#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("demo_trajectory_pub"),
        "config",
        "params.yaml"
    )


    return LaunchDescription([
        Node(
            package="demo_trajectory_pub",
            executable="demo_trajectory_pub_node",
            name="demo_trajectory_pub_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
