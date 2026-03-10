import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='shelter.sdf', description='Name of Gazebo world file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'simulation.launch.py'])),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'send_buffer_limit': 100000000
        }],
        output='screen'
    )

    mission_manager_node = Node(
        package='fcm_digital_twin',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        
        simulation_launch,
        # Запускаем менеджер миссий сразу
        mission_manager_node,
        # Запускаем мост с задержкой
        TimerAction(period=3.0, actions=[foxglove_bridge_node])
    ])