#!/usr/bin/env python3
#
# Copyright 2023 EduRobotAILab CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leo Cho

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [    
    DeclareLaunchArgument('use_sim_time', 
                          default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) clock if true'),
]

def generate_launch_description():
    pkg_share_bringup = get_package_share_directory('theimc_bringup')
    pkg_share_description = get_package_share_directory('theimc_description')

    params_ydlidar = PathJoinSubstitution([pkg_share_bringup, 'params', 'theimc_ydlidar.yaml']) 
    urdf_file = os.path.join(pkg_share_description, 'urdf', 'theimc.urdf')

    # Read URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    motor_drive_cmd = Node(
        package='theimc_bringup',
        executable='bringup_node',
        name='bringup_node',
        output='screen'
    )
 


      
    ydlidar_cmd = Node(
        package='sllider_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_ydlidar],
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description_content},
        ],
    )



    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(motor_drive_cmd)
    ld.add_action(ydlidar_cmd)


    return ld
