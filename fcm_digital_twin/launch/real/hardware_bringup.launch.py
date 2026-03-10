import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. ЛИДАР (С обязательными аргументами портов и фреймов)
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/lidar', 'frame_id': 'laser'}.items()
    )

    # 2. КАМЕРА (Вызов твоего скрипта напрямую)
    camera_launch = ExecuteProcess(
        cmd=['ros2', 'launch', os.path.expanduser('~/ros2_ws/run_camera.launch.py')],
        output='screen'
    )

    # 3. ИНФРАСТРУКТУРА (Foxglove с расширенным буфером и Менеджер)
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': 8765, 
            'send_buffer_limit': 100000000, 
            'use_sim_time': False
        }]
    )

    mission_manager_node = Node(
        package='fcm_digital_twin',
        executable='mission_manager',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        sllidar_launch,
        camera_launch,
        foxglove_node,
        mission_manager_node
    ])