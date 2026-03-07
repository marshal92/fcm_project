import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. Путь к НАШЕМУ пакету
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 2. Путь к ТВОЕМУ выстраданному конфигу
    nav2_config_path = PathJoinSubstitution(
        [fcm_pkg, 'config', 'nav2', 'navigation.yaml']
    )

    bt_xml_path = PathJoinSubstitution(
        [fcm_pkg, 'behavior_trees', 'tiered_survival.xml']
    )
    
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_config_path,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path
        },
        convert_types=True
    )

    # 3. Путь к стандартному лаунчу движка Nav2
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    return LaunchDescription([
        # Аргументы, которые мы можем менять при запуске
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Оставляем аргумент, чтобы передавать её путь из консоли или Master Launch.
        DeclareLaunchArgument('map', description='Full path to map yaml file'),

        # Запускаем всю махину Nav2 с нашим файлом
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': configured_nav2_params
            }.items()
        )
    ])