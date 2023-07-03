'''
Author       : dwayne
Date         : 2023-07-02
LastEditTime : 2023-07-02
Description  : 

Copyright (c) 2023 by dwayne, All Rights Reserved. 
'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction, ExecuteProcess
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams, RewrittenYaml
from launch.logging import launch_config
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    log_level = LaunchConfiguration('log_level')

    config = os.path.join(
        get_package_share_directory('mpc_follower'),
        'config',
        'mpc_controller.yaml'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description="log_level")

    mpc_controller_node = Node(
        package='mpc_follower',
        executable='mpc_controller_node',
        name='mpc_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[config])

    ld = LaunchDescription(
        [declare_log_level_cmd, mpc_controller_node])

    return ld
