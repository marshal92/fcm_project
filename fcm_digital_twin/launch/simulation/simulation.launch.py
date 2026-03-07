import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Ищем пути к пакетам
    fcm_pkg = FindPackageShare('fcm_digital_twin')
    lino_gazebo_pkg = FindPackageShare('linorobot2_gazebo')

    # 2. Объявляем аргумент для ИМЕНИ файла (по умолчанию shelter.sdf)
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='shelter.sdf',
        description='Name of the Gazebo world file'
    )

    # 3. Динамически склеиваем полный путь
    world_path = PathJoinSubstitution([
        fcm_pkg, 
        'worlds', 
        LaunchConfiguration('world_file')
    ])

    # 4. ВЫЗЫВАЕМ ОРИГИНАЛЬНЫЙ ЛАУНЧ
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([lino_gazebo_pkg, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world_path,  # linorobot ожидает полный путь, отдаем ему склеенный!
            'spawn_x': '0.0',
            'spawn_y': '0.0'
        }.items()
    )

    return LaunchDescription([
        world_file_arg,
        gazebo_launch
    ])