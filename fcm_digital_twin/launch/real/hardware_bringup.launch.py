import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 0. БАЗОВЫЙ BRINGUP LINOROBOT2 (Запускаем ядро робота)
    linorobot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('linorobot2_bringup'), 'launch', 'bringup.launch.py')
        )
    )
        
    # 1. ЛИДАР
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/lidar', 'frame_id': 'laser'}.items()
    )

    # 2. КАМЕРА
    camera_launch = ExecuteProcess(
        cmd=['ros2', 'launch', os.path.expanduser('~/ros2_ws/run_camera.launch.py')],
        output='screen'
    )

    # 3. УПРАВЛЕНИЕ МИССИЯМИ (Бортовой менеджер)
    mission_manager_node = Node(
        package='fcm_digital_twin',
        executable='mission_manager',
        # Сообщаем менеджеру, что он на реальном железе:
        parameters=[{'is_simulation': False}]
    )

    return LaunchDescription([
        linorobot_bringup,
        sllidar_launch,
        camera_launch,
        mission_manager_node
    ])