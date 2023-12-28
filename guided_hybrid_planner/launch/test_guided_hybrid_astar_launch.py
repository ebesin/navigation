'''
Author       : dwayne
Date         : 2023-04-19
LastEditTime : 2023-05-28
Description  :

Copyright (c) 2023 by dwayne, All Rights Reserved.
'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction, ExecuteProcess
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams, RewrittenYaml
from launch.logging import launch_config
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    lifecycle_nodes = ['guided_hybrid_planner_flow', "map_server"]
    param_file_name = "test_guided_hybrid_astar_params.yaml"

    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config')
    namespace = LaunchConfiguration('namespace')

    guided_hybrid_astar_dir = get_package_share_directory(
        'guided_hybrid_planner')
    workspace_dir = '/home/dwayne/workspace/navigation/nav2_ws'

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description="")

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            guided_hybrid_astar_dir, 'params', param_file_name))

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            guided_hybrid_astar_dir, 'map', 'sim_orchard3.yaml'),
        description='Full path to map file to load')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            guided_hybrid_astar_dir, 'rviz', 'test_guided_hybrid2.rviz'))

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'ROS_LOG_DIR', '/home/dwayne/workspace/navigation/nav2_ws/src/navigation/guided_hybrid_planner/log')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0',
                           '0', 'map', 'base_link']
            ),
            Node(
                package='guided_hybrid_planner',
                executable='guided_hybrid_planner_flow',
                name="guided_hybrid_planner_flow",
                output='log',
                # output='screen',
                respawn=use_respawn,
                parameters=[configured_params],
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart},
                            {'use_sim_time': use_sim_time},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    register_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=start_rviz_cmd,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[load_nodes]
                )
            ]
        )
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(register_handler)

    return ld
