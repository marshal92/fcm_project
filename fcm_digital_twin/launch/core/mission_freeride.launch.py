import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. SLAM в режиме чистого МАППИНГА с нуля
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_mapping.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. НАВИГАЦИЯ (абсолютно та же самая, она умеет работать с растущей картой)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 3. УЗЛЫ РАДИАЦИИ (Оставляем их, чтобы можно было генерировать угрозы в новых комнатах)
    radiation_server_node = Node(
        package='fcm_digital_twin',
        executable='radiation_field_server',
        name='radiation_field_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'is_active': False
        }]      
    )

    alara_reflex_node = Node(
        package='fcm_digital_twin',
        executable='alara_speed_reflex',
        name='alara_speed_reflex',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_launch,
        # Даем SLAMу 2 секунды на создание первой пустой карты (0,0,0), затем пускаем водителя
        TimerAction(period=2.0, actions=[nav_launch]),
        radiation_server_node,
        alara_reflex_node
    ])