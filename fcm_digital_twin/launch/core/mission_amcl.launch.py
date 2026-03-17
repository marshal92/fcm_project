import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')

    # Аргументы
    map_name_arg = DeclareLaunchArgument('map_name', default_value='shelter_map')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Путь к файлу карты (.yaml). Предполагается, что карты лежат в папке maps/
    map_yaml_file = PathJoinSubstitution([fcm_pkg, 'maps', [map_name, '.yaml']])

    # === 1. ЛОКАЛИЗАЦИЯ (AMCL + Map Server) ===
    # Это заменит Lifelong SLAM. Читает статичную карту и раскидывает частицы.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'localization_launch.py'])),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # === 2. НАВИГАЦИЯ ===
    # Переиспользуем твой идеальный кусок кода с Behavior Tree
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # === 3. КАСТОМНЫЕ УЗЛЫ FCM ===
    radiation_server_node = Node(
        package='fcm_digital_twin',
        executable='radiation_field_server',
        name='radiation_field_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    alara_reflex_node = Node(
        package='fcm_digital_twin',
        executable='alara_speed_reflex',
        name='alara_speed_reflex',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        
        localization_launch,
        radiation_server_node,
        alara_reflex_node,
        
        # Даем Map Server'у пару секунд на загрузку карты в память
        TimerAction(period=2.0, actions=[nav_launch])
    ])