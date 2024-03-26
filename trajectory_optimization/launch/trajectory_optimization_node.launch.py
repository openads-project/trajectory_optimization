#!/usr/bin/env python3

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():


    params_arg = DeclareLaunchArgument('params', default_value=PathJoinSubstitution([
        get_package_share_directory("trajectory_optimization"), "config", "params.yml"])
    )
    node_name_arg = DeclareLaunchArgument('node_name', default_value='trajectory_optimization')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')

    drivable_space_topic_arg = DeclareLaunchArgument('drivable_space_topic', default_value='~/drivable_space')
    ego_data_topic_arg = DeclareLaunchArgument('ego_data_topic', default_value='~/ego_data')
    object_list_topic_arg = DeclareLaunchArgument('object_list_topic', default_value='~/object_list')
    reference_trajectory_topic_arg = DeclareLaunchArgument('reference_trajectory_topic', default_value='~/reference_trajectory')
    route_topic_arg = DeclareLaunchArgument('route_topic', default_value='~/route')

    trajectory_topic_arg = DeclareLaunchArgument('trajectory_topic', default_value='~/trajectory')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')


    return LaunchDescription([
        params_arg,
        node_name_arg,
        namespace_arg,
        drivable_space_topic_arg,
        ego_data_topic_arg,
        object_list_topic_arg,
        reference_trajectory_topic_arg,
        route_topic_arg,
        trajectory_topic_arg,
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        Node(
            package="trajectory_optimization",
            executable="trajectory_optimization_node",
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration('params')],
            remappings=[
                ("~/drivable_space", LaunchConfiguration('drivable_space_topic')),
                ("~/ego_data", LaunchConfiguration('ego_data_topic')),
                ("~/object_list", LaunchConfiguration('object_list_topic')),
                ("~/reference_trajectory", LaunchConfiguration('reference_trajectory_topic')),
                ("~/route", LaunchConfiguration('route_topic')),
                ("~/trajectory", LaunchConfiguration('trajectory_topic'))
            ]
        )
    ])
