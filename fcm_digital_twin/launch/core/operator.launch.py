import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # === АРГУМЕНТЫ ===
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='shelter.sdf', description='Name of Gazebo world file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # === 1. SHADOW MODE (Цифровой двойник) ===
    shadow_teleop_node = Node(
        package='fcm_digital_twin',
        executable='shadow_teleop',
        name='shadow_teleop',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === 2. HEARTBEAT (Пульс для защиты) ===
    heartbeat_pub_node = Node(
        package='fcm_digital_twin',
        executable='heartbeat_pub',
        name='heartbeat_pub',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === 3. SDF VISUALIZER (3D окружение для Foxglove) ===
    sdf_visualizer_node = Node(
        package='fcm_digital_twin',
        executable='sdf_visualizer_node',
        name='sdf_visualizer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world_file': LaunchConfiguration('world_file') 
        }]
    )

    # === 4. УПРАВЛЕНИЕ С ГЕЙМПАДА (Прямое управление танком) ===
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])
        ),
        # Укажи здесь 'xbox', 'steamdeck' или свой конфиг
        launch_arguments={'joy_config': 'xbox'}.items() 
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_file_arg,
        
        shadow_teleop_node,
        heartbeat_pub_node,
        sdf_visualizer_node,
        teleop_launch
    ])