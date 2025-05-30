#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description_with_resolved_launch_args(launch_context):

    # get the driving mode from the launch context
    driving_mode = LaunchConfiguration('driving_mode').perform(launch_context)

    if driving_mode == "ackermann":
        executable_name = "trajectory_optimization_ackermann_node"
    elif driving_mode == "rws":
        executable_name = "trajectory_optimization_rws_node"
    else:
        raise ValueError(f"Invalid driving mode: {driving_mode}")

    # define other launch arguments / remappings / nodes
    
    remappable_topics = [
        DeclareLaunchArgument("ego_data_topic", default_value="~/ego_data"),
        DeclareLaunchArgument("object_list_topic", default_value="~/object_list"),
        DeclareLaunchArgument("reference_trajectory_topic", default_value="~/reference_trajectory"),
        DeclareLaunchArgument("route_topic", default_value="~/route"),
        DeclareLaunchArgument("trajectory_topic", default_value="~/trajectory"),
        DeclareLaunchArgument("object_circles_topic", default_value="~/visualization/object_circles"),
    ]
    
    args = [
        DeclareLaunchArgument("name", default_value=executable_name, description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("trajectory_optimization"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        *remappable_topics
    ]

    return [
        *args,
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        Node(
            package="trajectory_optimization",
            executable=executable_name,
            name=LaunchConfiguration("name"),
            namespace=LaunchConfiguration("namespace"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        )
    ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('driving_mode', default_value='ackermann'),
        OpaqueFunction(function=generate_launch_description_with_resolved_launch_args)
    ])