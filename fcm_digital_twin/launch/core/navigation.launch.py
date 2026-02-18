import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Путь к НАШЕМУ пакету
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 2. Путь к ТВОЕМУ выстраданному конфигу
    nav2_config_path = PathJoinSubstitution(
        [fcm_pkg, 'config', 'nav2', 'navigation.yaml']
    )

    # 3. Путь к стандартному лаунчу движка Nav2
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    return LaunchDescription([
        # Аргументы, которые мы можем менять при запуске
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Навигации (AMCL) ОБЯЗАТЕЛЬНО нужна карта. 
        # Оставляем аргумент, чтобы передавать её путь из консоли или Master Launch.
        DeclareLaunchArgument('map', description='Full path to map yaml file'),

        # Запускаем всю махину Nav2 с нашим файлом
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_config_path
            }.items()
        )
    ])