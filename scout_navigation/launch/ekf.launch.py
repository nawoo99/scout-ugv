from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch arguments (optional for flexibility)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (e.g., for bag playback)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        declare_use_sim_time,

        # Static TF: base_link -> vectornav (IMU)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_vectornav',
            arguments=[
                '0', '0', '0.2',
                '0', '0', '0',
                'base_link', 'vectornav'
            ],
            output='screen'
        ),

        # Static TF: base_link -> os_sensor (LIDAR)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_os_sensor',
            arguments=[
                '0', '0', '0.6',
                '0', '0', '0',
                'base_link', 'os_sensor'
            ],
            output='screen'
        ),

        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                '/home/imad/scout_navigation_ws/src/scout_navigation/config/ekf.yaml',
                {'use_sim_time': use_sim_time}
            ]
        )
    ])

