import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Аргумент для загрузки 3D-модели помещения (Shelter) как фона в Foxglove
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='shelter.sdf', description='Name of SDF world file'
    )

    # 1. МОСТ FOXGLOVE (Живет на сервере!)
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765, 'send_buffer_limit': 100000000, 'use_sim_time': False}],
        output='screen'
    )

    # 2. СЕТЕВОЙ ОРКЕСТРАТОР (Слушает Foxglove, шлет команды на робота)
    twin_orchestrator_node = Node(
        package='fcm_digital_twin',
        executable='twin_orchestrator',
        name='twin_orchestrator',
        output='screen'
    )

    # 3. SHADOW MODE (Идеальный скрипт для реального робота)
    shadow_teleop_node = Node(
        package='fcm_digital_twin',
        executable='shadow_teleop_real',
        name='shadow_teleop',
        output='screen'
    )

    # 4. SDF VISUALIZER (3D окружение стен для наглядности в интерфейсе)
    sdf_visualizer_node = Node(
        package='fcm_digital_twin',
        executable='sdf_visualizer_node',
        name='sdf_visualizer_node',
        output='screen',
        parameters=[{'world_file': LaunchConfiguration('world_file')}]
    )

    # 5. ПУЛЬС (Защита от потери связи оператора с танком)
    heartbeat_pub_node = Node(
        package='fcm_digital_twin',
        executable='heartbeat_pub',
        name='heartbeat_pub',
        output='screen'
    )

    # 6. УПРАВЛЕНИЕ С ГЕЙМПАДА (Геймпад воткнут в ноутбук)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])
        ),
        launch_arguments={'joy_config': 'xbox'}.items() 
    )

    return LaunchDescription([
        world_file_arg,
        
        foxglove_node,
        twin_orchestrator_node,
        shadow_teleop_node,
        sdf_visualizer_node,
        heartbeat_pub_node,
        teleop_launch
    ])