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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [    
    DeclareLaunchArgument('use_sim_time', 
                          default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) clock if true'),
]

def generate_launch_description():
    
    pkg_share_navigation2 = get_package_share_directory('theimc_navigation2')
    pkg_share_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    params_nav2_rviz = PathJoinSubstitution([pkg_share_nav2_bringup, 'rviz', 'nav2_default_view.rviz'])    
       
    params_nav2_map = PathJoinSubstitution([pkg_share_navigation2, 'maps', 'map1.yaml'])        
    params_nav2 = PathJoinSubstitution([pkg_share_navigation2, 'params', 'theimc_nav2_params.yaml'])    
    launch_nav2_bringup = PathJoinSubstitution([pkg_share_nav2_bringup, 'launch', 'bringup_launch.py'])
    
    map = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
        
    dla_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=params_nav2_map,
        description='Full path to map yaml file to load')
    
    dla_params_nav2_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_nav2,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_nav2_bringup]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': map,            
            'params_file': params_file}.items())
        
    rviz_cmd = Node(        
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', params_nav2_rviz])
        
    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(dla_map_yaml_cmd)    
    ld.add_action(dla_params_nav2_cmd)     
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_cmd)

    return ld

