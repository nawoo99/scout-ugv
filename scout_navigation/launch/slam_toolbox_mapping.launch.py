import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("scout_navigation"),
                                   'config', 'slam_toolbox_mapping.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_use_rviz_argument = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz')

    # Start SLAM Toolbox node (async) with odometry remapping
    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('/odom', '/odometry/filtered')  # ✅ Use filtered odometry
        ]
    )

    # RViz node (optional)
    rviz_config_file = os.path.join(get_package_share_directory('scout_navigation'), 'config', 'map.rviz')
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    # Static transform: base_link → os_sensor (LIDAR)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_lidar',
        output='screen',
        arguments=['0', '0', '0.6', '0', '0', '0', 'base_link', 'os_sensor']
    )

    # Static transform: base_link → vectornav (IMU)
    static_tf_base_to_vectornav = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_vectornav',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'vectornav']
    )

    # LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_rviz_argument)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(start_rviz_node)
    ld.add_action(static_tf_base_to_lidar)
    ld.add_action(static_tf_base_to_vectornav)

    return ld

