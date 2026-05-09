import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # === Master Launch Arguments ===
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='shelter_map', description='Name of the SLAM posegraph'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )

    # Variables for convenient transfer down the hierarchy
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')

    # === 1. LIFELONG SLAM ===
    slam_lifelong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_lifelong.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_name': map_name
        }.items()
    )

    # === 2. NAVIGATION (Without AMCL) ===
    nav_lifelong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # === 3. CUSTOM NODES FCM ===
    radiation_server_node = Node(
        package='fcm_digital_twin',
        executable='radiation_field_server',
        name='radiation_field_server',
        output='screen'#,
    #    parameters=[{'use_sim_time': use_sim_time}]
    )

    alara_reflex_node = Node(
        package='fcm_costmap_plugins',
        executable='alara_speed_reflex_node',
        name='alara_speed_reflex',
        output='screen'#,
        #parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        
        slam_lifelong_launch,
        radiation_server_node,
        alara_reflex_node,
        
        TimerAction(period=6.0, actions=[nav_lifelong_launch])
    ])