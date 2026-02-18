import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Путь к конфигу
    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('fcm_digital_twin'), 'config', 'slam', 'slam.yaml']
    )

    # 2. Путь к стандартному лаунчу slam_toolbox 
    slam_launch_path = PathJoinSubstitution(
         [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Запуск SLAM с моим yaml
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_config_path
            }.items()
        )
    ])