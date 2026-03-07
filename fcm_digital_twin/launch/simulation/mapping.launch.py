# fcm_digital_twin/launch/simulation/mapping.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    robot_base = os.getenv('LINOROBOT2_BASE')
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )
    
    # МАГИЯ ЗДЕСЬ: Мы динамически склеиваем путь к папке worlds и имя файла из аргумента
    world_path = PathJoinSubstitution([
        FindPackageShare("fcm_digital_twin"), 
        "worlds", 
        LaunchConfiguration('world_file')  # <- Берет значение из аргумента ниже
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        urdf_path, " ",
        "sim:=true ",        
        "god_mode:=true"     # Идеальная одометрия для симуляции
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        
        # ТЕПЕРЬ АРГУМЕНТ - ЭТО ТОЛЬКО ИМЯ ФАЙЛА
        DeclareLaunchArgument('world_file', default_value='shelter.sdf', description='Name of the world file (e.g. empty.sdf)'),
        
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        
        # 1. Запуск Газебо (передаем наш склеенный world_path)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'gz_args': [' -r -s ', world_path]}.items()
        ),

        # 2. GUI Газебо
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={'gz_args': [' -g']}.items()
        ),

        # 3. Спавн робота
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description', '-entity', 'linorobot2', 
                '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'), '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        # 4. Мост ROS-Gazebo
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V", 
            ],
            remappings=[
                ('/camera/camera_info', '/camera/color/camera_info'),
                ('/camera/image', '/camera/color/image_raw'),
                ('/camera/depth_image', '/camera/depth/image_rect_raw'),
                ('/camera/points', '/camera/depth/color/points'),
            ]
        ),

        # 5. Robot State Publisher (С НАШИМ god_mode)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
        ),
    ])