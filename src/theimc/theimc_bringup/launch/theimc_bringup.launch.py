

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, LifecycleNode

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
    urdf_file = os.path.join(pkg_share, 'urdf', 'theimc.urdf')
    
    motor_drive_cmd = Node(
        package='theimct_bringup',
        executable='bringup_node',
        name='bringup_node',
        output='screen'
    )
       
    ydlidar_cmd = Node(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
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
            {'robot_description': Command(['xacro', ' ', urdf_xacro])},
        ],
    )
        
s
    ld = LaunchDescription(ARGUMENTS)    
    ld.add_action(robot_state_publisher_cmd)        
    ld.add_action(motor_drive_cmd)        
    ld.add_action(ydlidar_cmd)        

    return ld
