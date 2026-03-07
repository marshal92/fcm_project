import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # Путь к нашему конфигу (там лежат настройки контроллера, танка и т.д.)
    nav2_config_path = PathJoinSubstitution(
        [fcm_pkg, 'config', 'nav2', 'navigation.yaml']
    )
    # 2. ДИНАМИЧЕСКИЙ ПУТЬ К ДЕРЕВУ
    bt_xml_path = PathJoinSubstitution(
        [fcm_pkg, 'behavior_trees', 'tiered_survival.xml']
    )

    # 3. МАГИЯ: Перезаписываем параметр на лету
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_config_path,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path
        },
        convert_types=True
    )

    # ВАЖНО: Мы вызываем НЕ bringup_launch.py, а navigation_launch.py
    # Это запустит планировщик и контроллер, но БЕЗ AMCL и Map Server!
    nav2_navigation_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': configured_nav2_params
            }.items()
        )
    ])