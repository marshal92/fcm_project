import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Ищем пути к пакетам
    fcm_pkg = FindPackageShare('fcm_digital_twin')
    lino_gazebo_pkg = FindPackageShare('linorobot2_gazebo')

    # 2. Указываем путь к ТВОЕМУ миру
    world_path = PathJoinSubstitution([fcm_pkg, 'worlds', 'kitchen.sdf'])

    # 3. ВЫЗЫВАЕМ ОРИГИНАЛЬНЫЙ ЛАУНЧ, НО С ТВОИМ МИРОМ
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([lino_gazebo_pkg, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world_path,  # Подменяем аргумент 'world'
            'spawn_x': '0.0',
            'spawn_y': '0.0'
        }.items()
    )

    return LaunchDescription([
        gazebo_launch
    ])