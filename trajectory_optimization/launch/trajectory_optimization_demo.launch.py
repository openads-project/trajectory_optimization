#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    trajectory_optimization_launch_file = os.path.join(
        get_package_share_directory("trajectory_optimization"), "launch", "trajectory_optimization.launch.py"
    )

    demo_pub_launch_file = os.path.join(
        get_package_share_directory("dummy_input_generation"), "launch", "dummy_input_generation_node.launch.py"
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen",
    )

    return LaunchDescription(
        [
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(trajectory_optimization_launch_file),
                        launch_arguments=[
                            ("reference_trajectory_topic", "/dummy_input_generation_node/reference_trajectory"),
                            ("ego_data_topic", "/dummy_input_generation_node/ego_data"),
                            ("object_list_topic", "/dummy_input_generation_node/object_list"),
                        ],
                    )
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(demo_pub_launch_file))],
            ),
            static_transform_publisher,
        ]
    )
