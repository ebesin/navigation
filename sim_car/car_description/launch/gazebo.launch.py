'''
Author       : iPEK
Date         : 2023-03-11
LastEditTime : 2023-03-15
Description  : 

Copyright (c) 2023 by iPEK, All Rights Reserved. 
'''
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_car = get_package_share_directory('sim_car')
    world = LaunchConfiguration("world")

    world_config = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_sim_car, 'worlds', 'orchard.world'), ''],
          description='SDF world file')

    # # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(pkg_sim_car, 'launch', 'spawn_red_crash.launch.py'),
            os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
        )
    )    

    ld = LaunchDescription()
    ld.add_action(world_config)
    ld.add_action(gazebo)
    ld.add_action(car)
    return ld