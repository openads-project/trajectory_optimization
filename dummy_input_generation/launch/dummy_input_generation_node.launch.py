#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from tracetools_launch.action import Trace


def generate_launch_description():
    """Generate launch description for dummy input generation node."""

    remappable_topics = [
        DeclareLaunchArgument("ego_data_topic", default_value="~/ego_data"),
        DeclareLaunchArgument("object_list_topic", default_value="~/object_list"),
        DeclareLaunchArgument("reference_trajectory_topic", default_value="~/reference_trajectory"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="dummy_input_generation_node", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(get_package_share_directory("dummy_input_generation"), "config", "params.yml"),
            description="path to parameter file",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument("trace", default_value="false", description="enable tracing"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="dummy_input_generation",
            executable="dummy_input_generation_node",
            name=LaunchConfiguration("name"),
            namespace=LaunchConfiguration("namespace"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        ),
        Trace(
            session_name="trace",
            dual_session=True,
            condition=IfCondition(LaunchConfiguration("trace")),
        ),
    ]

    return LaunchDescription(
        [
            *args,
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            *nodes,
        ]
    )
