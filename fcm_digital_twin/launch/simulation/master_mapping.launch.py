import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Function for injecting environment variable 
def inject_env_var(context, *args, **kwargs):
    use_3d = context.launch_configurations.get('use_3d_lidar', 'false')
    os.environ['USE_3D_LIDAR'] = use_3d
    return []

def generate_launch_description():
    fcm_pkg = FindPackageShare('fcm_digital_twin')

    # 1. Global arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file', 
        default_value='kitchen.sdf',
        description='Name of the SDF world file in the worlds folder'
    )
    
    use_3d_lidar_arg = DeclareLaunchArgument(
        'use_3d_lidar', 
        default_value='false',
        description='Use 3D lidar and pointcloud_to_laserscan pipeline'
    )

    use_3d_lidar = LaunchConfiguration('use_3d_lidar')
    use_sim_time = 'true'

    setup_env_func = OpaqueFunction(function=inject_env_var)

    # 2. Dynamically call the simulation launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'simulation', 'mapping.launch.py'])
        ),
        launch_arguments={'world_file': LaunchConfiguration('world_file')}.items()
    )

    # 3. Dynamically call the SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'slam_mapping.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Dynamically call the Nav2 launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fcm_pkg, 'launch', 'core', 'nav_lifelong.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Heartbeat
    heartbeat_node = Node(
        package='fcm_digital_twin',
        executable='heartbeat_pub',
        name='heartbeat_pub',
        output='screen'
    )

    # 6. Gamepad control 
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])
        ),
        launch_arguments={'joy_config': 'xbox'}.items() 
    )

    # 7. SDF VISUALIZER
    sdf_visualizer_node = Node(
        package='fcm_digital_twin',
        executable='sdf_visualizer_node',
        name='sdf_visualizer_node',
        output='screen',
        parameters=[{'world_file': LaunchConfiguration('world_file')}]
    )

    # 8. Launch RViz 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([fcm_pkg, 'config', 'rviz', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}]
    )

    # === 9. PIPELINE 3D Lidar ===
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
        parameters=[{'use_sim_time': True}]
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
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        world_file_arg,
        use_3d_lidar_arg,
        setup_env_func,
        
        sim_launch,
        slam_launch,
        nav_launch,
        heartbeat_node,
        teleop_launch,
        sdf_visualizer_node,
        rviz_node,
        
        gz_bridge_3d_node,
        stabilized_frame_node,
        pc_to_laserscan_node
    ])