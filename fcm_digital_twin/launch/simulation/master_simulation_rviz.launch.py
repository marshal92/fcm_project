import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # === АРГУМЕНТЫ ===
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='shelter.sdf',
        description='Name of Gazebo world file'
    )
    
    # НОВЫЙ АРГУМЕНТ ДЛЯ КАРТЫ (Имя файла без расширения!)
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='shelter_map',
        description='Name of the SLAM posegraph (without .posegraph extension)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # === 1. ЗАПУСК СИМУЛЯЦИИ ===
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'simulation.launch.py'])),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # === 2. ЗАПУСК SLAM (ПРОКИДЫВАЕМ ИМЯ КАРТЫ СЮДА) ===
    slam_lifelong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_lifelong.launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_name': LaunchConfiguration('map_name') # <--- Передаем аргумент в SLAM!
        }.items()
    )

    # === 3. ЗАПУСК НАВИГАЦИИ ===
    nav_lifelong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    # === 4. ЗАПУСК РАДИАЦИОННОЙ ПОДСИСТЕМЫ (ЦИФРОВОЙ ДВОЙНИК + ALARA) ===
    radiation_server_node = Node(
        package='fcm_digital_twin',
        executable='radiation_field_server',
        name='radiation_field_server',
        output='screen'
    )

    alara_reflex_node = Node(
        package='fcm_digital_twin',
        executable='alara_speed_reflex',
        name='alara_speed_reflex',
        output='screen'
    )

    # === 5. ЗАПУСК RVIZ ===
    rviz_config_path = PathJoinSubstitution([fcm_pkg, 'config', 'rviz', 'mapping.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # === 6. ЗАПУСК SHADOW MODE (ЦИФРОВОЙ ДВОЙНИК) ===
    shadow_teleop_node = Node(
        package='fcm_digital_twin',
        executable='shadow_teleop',
        name='shadow_teleop',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        world_file_arg,
        map_name_arg,
        use_sim_time_arg,
        
        simulation_launch,
        TimerAction(period=4.0, actions=[slam_lifelong_launch]),
        TimerAction(period=6.0, actions=[nav_lifelong_launch]),
        TimerAction(period=7.0, actions=[radiation_server_node, alara_reflex_node]),
        TimerAction(period=8.0, actions=[rviz_node]),
        TimerAction(period=10.0, actions=[shadow_teleop_node])
    ])