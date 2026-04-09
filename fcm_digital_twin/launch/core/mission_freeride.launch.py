import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. SLAM in mapping mode
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_mapping.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. NAVIGATION (absolutely the same, it can work with the growing map)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 3. Radiation Nodes (Keeping them to generate threats in new rooms)
    radiation_server_node = Node(
        package='fcm_digital_twin',
        executable='radiation_field_server',
        name='radiation_field_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'is_active': False
        }]      
    )

    alara_reflex_node = Node(
        package='fcm_digital_twin',
        executable='alara_speed_reflex',
        name='alara_speed_reflex',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_launch,

        TimerAction(period=2.0, actions=[nav_launch]),
        radiation_server_node,
        alara_reflex_node
    ])