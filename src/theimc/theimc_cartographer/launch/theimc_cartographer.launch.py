# !/usr/bin/env/ python3
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]

def generate_launch_description():
    
    pkg_share_cartographer = get_package_share_directory('theimc_cartographer')
    pkg_share_teleop = get_package_share_directory('theimc_teleop')
    params_rviz = PathJoinSubstitution([pkg_share_cartographer, 'rviz', 'theimc_cartographer.rviz'])      
    cartographer_config = LaunchConfiguration('cartographer_config', default=os.path.join(pkg_share_cartographer, 'params'))
    lua_config = LaunchConfiguration('lua_config', default='theimc_cartographer.lua')
   
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', params_rviz]
    )
   
    cartoprapher_cmd = Node(
        package='cartographer_ros', 
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-configuration_directory', cartographer_config,
                   '-configuration_basename', lua_config]
    )
    
    occupancy_grid_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],        
        emulate_tty=True,   
    )
       

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(cartoprapher_cmd)
    ld.add_action(occupancy_grid_cmd)
    ld.add_action(rviz_cmd)

    return ld

# To save map
# ros2 run nav2_map_server map_saver_cli -f ~/map
