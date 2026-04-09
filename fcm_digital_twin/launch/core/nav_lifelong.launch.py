import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 1. Path to our config (there are controller settings, tank settings, etc.)
    nav2_config_path = PathJoinSubstitution(
        [fcm_pkg, 'config', 'nav2', 'navigation.yaml']
    )
    # 2. Dynamic path to the behavior tree
    bt_xml_path = PathJoinSubstitution(
        [fcm_pkg, 'behavior_trees', 'tiered_survival.xml']
    )

    # 3. Rewrite params
    #configured_nav2_params = RewrittenYaml(
    #    source_file=nav2_config_path,
    #    root_key='',
    #    param_rewrites={
    #        'default_nav_to_pose_bt_xml': bt_xml_path
    #    },
    #    convert_types=True
    #)

    nav2_navigation_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_config_path
            }.items()
        )
    ])