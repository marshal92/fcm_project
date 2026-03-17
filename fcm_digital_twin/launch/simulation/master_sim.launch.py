import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def inject_env_var(context, *args, **kwargs):
    use_3d = context.launch_configurations.get('use_3d_lidar', 'false')
    os.environ['USE_3D_LIDAR'] = use_3d
    return []

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # === 1. АРГУМЕНТЫ ===
    world_file_arg = DeclareLaunchArgument('world_file', default_value='shelter.sdf')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_3d_lidar_arg = DeclareLaunchArgument('use_3d_lidar', default_value='false')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_3d_lidar = LaunchConfiguration('use_3d_lidar')

    setup_env_func = OpaqueFunction(function=inject_env_var)

    # === 2. БАЗОВАЯ СИМУЛЯЦИЯ (Gazebo) ===
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'simulation.launch.py'])),
        launch_arguments={'world_file': LaunchConfiguration('world_file'), 'use_sim_time': use_sim_time}.items()
    )
    
    # === 3. ИНТЕРФЕЙС И ОРКЕСТРАТОРЫ ===
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765, 'use_sim_time': use_sim_time, 'send_buffer_limit': 100000000, 'max_qos_depth': 10}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    mission_manager_node = Node(
        package='fcm_digital_twin',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{'is_simulation': use_sim_time}]
    )

    twin_orchestrator_node = Node(
        package='fcm_digital_twin',
        executable='twin_orchestrator',
        name='twin_orchestrator',
        output='screen'
        # Скрипт реактивный, параметр use_sim_time ему не нужен
    )

    # === 4. УЗЛЫ ОПЕРАТОРА (Только симуляция!) ===
    shadow_teleop_node = Node(
        package='fcm_digital_twin',
        executable='shadow_teleop_sim',
        name='shadow_teleop',
        output='screen'
    )

    heartbeat_pub_node = Node(
        package='fcm_digital_twin',
        executable='heartbeat_pub',
        name='heartbeat_pub',
        output='screen'
    )

    sdf_visualizer_node = Node(
        package='fcm_digital_twin',
        executable='sdf_visualizer_node',
        name='sdf_visualizer_node',
        output='screen',
        parameters=[{'world_file': LaunchConfiguration('world_file')}]
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])
        ),
        launch_arguments={'joy_config': 'xbox'}.items() 
    )

    # === 5. PIPELINE 3D ЛИДАРА ===
    gz_bridge_3d_node = Node(
        condition=IfCondition(use_3d_lidar),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[('/lidar3d/points', '/points')],
        output='screen'
    )

    stabilized_frame_node = Node(
        condition=IfCondition(use_3d_lidar),
        package='fcm_digital_twin',
        executable='stabilized_frame_publisher',
        name='stabilized_frame_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pc_to_laserscan_node = Node(
        condition=IfCondition(use_3d_lidar),
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/points'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_stabilized',
            'min_height': 0.15,
            'max_height': 1.5,  
            'use_sim_time': use_sim_time
        }]
    )
        
    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        use_3d_lidar_arg,
        
        setup_env_func, 
        
        simulation_launch,
        mission_manager_node,
        twin_orchestrator_node,
        
        shadow_teleop_node,
        heartbeat_pub_node,
        sdf_visualizer_node,
        teleop_launch,
        
        gz_bridge_3d_node,
        stabilized_frame_node,
        pc_to_laserscan_node,
        
        TimerAction(period=3.0, actions=[foxglove_bridge_node])
    ])