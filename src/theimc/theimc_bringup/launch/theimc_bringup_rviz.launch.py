

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
    
    pkg_share_bringup = get_package_share_directory('theimc_bringup')
    pkg_share_teleop = get_package_share_directory('theimc_teleop')
    
    params_rviz = PathJoinSubstitution([pkg_share_bringup, 'rviz', 'theimc_bringup.rviz'])    
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', params_rviz]
    )
               
       
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz_cmd)

    return ld
