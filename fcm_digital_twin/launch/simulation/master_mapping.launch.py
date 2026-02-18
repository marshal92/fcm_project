import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 1. Глобальный аргумент: какой мир грузим
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value=PathJoinSubstitution([fcm_pkg, 'worlds', 'kitchen.sdf']),
        description='Global world path for mapping'
    )

    # 2. Вызов Газебо + God Mode
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'mapping.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 3. Вызов SLAM (Чистый маппинг)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_mapping.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Вызов RViz с твоим сохраненным конфигом
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([fcm_pkg, 'config', 'rviz', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        sim_launch,
        slam_launch,
        rviz_node
    ])