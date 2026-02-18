import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# ИМПОРТИРУЕМ ИНСТРУМЕНТ ДЛЯ ПОДМЕНЫ ПУТЕЙ В YAML
from nav2_common.launch import RewrittenYaml 

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 1. Путь к нашему базовому конфигу (внутри которого можно оставить пустые поля или любой мусор)
    slam_config_path = PathJoinSubstitution(
        [fcm_pkg, 'config', 'slam', 'slam_lifelong.yaml']
    )

    # 2. Динамический путь к картам (будет работать на любой флешке и любом ПК)
    posegraph_path = PathJoinSubstitution(
        [fcm_pkg, 'maps', 'kitchen_posegraph']
    )

    # 3. МАГИЯ: Перезаписываем параметры в оперативной памяти!
    configured_params = RewrittenYaml(
        source_file=slam_config_path,
        root_key='',
        param_rewrites={
            'map_file_name': posegraph_path, # Подменяем путь
            'mode': 'localization'           # Жестко фиксируем режим           # Жестко фиксируем режим
        },
        convert_types=True
    )

    slam_launch_path = PathJoinSubstitution(
         [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # ВАЖНО: Передаем СГЕНЕРИРОВАННЫЙ конфиг, а не путь к файлу
                'slam_params_file': configured_params 
            }.items()
        )
    ])