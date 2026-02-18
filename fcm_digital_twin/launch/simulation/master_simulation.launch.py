import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # === АРГУМЕНТЫ (Можно менять из терминала) ===
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([fcm_pkg, 'worlds', 'kitchen.sdf']),
        description='Path to Gazebo world file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # === 1. ЗАПУСК СИМУЛЯЦИИ (Ищем в папке launch/simulation) ===
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'simulation.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # === 2. ЗАПУСК SLAM (Ищем в папке launch/core) ===
    slam_lifelong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_lifelong.launch.py'])),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # === 3. ЗАПУСК НАВИГАЦИИ (Ищем в папке launch/core) ===
    nav_lifelong_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # === 4. ЗАПУСК RVIZ ===
    rviz_config_path = PathJoinSubstitution([fcm_pkg, 'config', 'rviz', 'mapping.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        
        # Запускаем всё по очереди для стабильности
        simulation_launch,
        
        # Даем Газебо время на запуск
        TimerAction(period=4.0, actions=[slam_lifelong_launch]),
        TimerAction(period=6.0, actions=[nav_lifelong_launch]),
        TimerAction(period=8.0, actions=[rviz_node])
    ])