import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Import the tool for rewriting paths in YAML
from nav2_common.launch import RewrittenYaml 

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # ACCEPTING ARGUMENT FROM THE MASTER
    map_name_arg = DeclareLaunchArgument('map_name', default_value='shelter_map')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    slam_config_path = PathJoinSubstitution(
        [fcm_pkg, 'config', 'slam', 'slam_lifelong.yaml']
    )

    # Dynamic path: Takes the name from the map_name argument
    posegraph_path = PathJoinSubstitution(
        [fcm_pkg, 'maps', LaunchConfiguration('map_name')]
    )

    configured_params = RewrittenYaml(
        source_file=slam_config_path,
        root_key='',
        param_rewrites={
            'map_file_name': posegraph_path, 
            'mode': 'localization'           
        },
        convert_types=True
    )
    slam_launch_path = PathJoinSubstitution(
         [FindPackageShare('slam_toolbox'), 'launch', 'localization_launch.py']
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': configured_params 
            }.items()
        )
    ])