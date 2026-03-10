import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 1. Глобальный аргумент: имя файла мира (как его ждет mapping.launch.py)
    world_file_arg = DeclareLaunchArgument(
        'world_file', 
        default_value='kitchen.sdf',
        description='Name of the SDF world file in the worlds folder'
    )

    use_sim_time = 'true' # Для мастера маппинга это всегда симуляция

    # 2. Вызов Газебо + God Mode
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'mapping.launch.py'])
        ),
        launch_arguments={'world_file': LaunchConfiguration('world_file')}.items()
    )

    # 3. Вызов SLAM (Чистый маппинг)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_mapping.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Вызов Nav2 (Нужен для объезда препятствий и работы explore_lite)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Пульс (Heartbeat) - чтобы Nav2 не паниковал и не триггерил дерево выживания
    heartbeat_node = Node(
        package='fcm_digital_twin',
        executable='heartbeat_pub',
        name='heartbeat_pub',
        output='screen'
    )

    # 6. Вызов RViz с твоим сохраненным конфигом
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([fcm_pkg, 'config', 'rviz', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_file_arg,
        sim_launch,
        slam_launch,
        nav_launch,
        heartbeat_node,
        rviz_node
    ])