from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Static transform from base_link to os_sensor (adjust if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_os_sensor_to_base',
            arguments=[
                '0', '0', '0',              # translation x y z
                '0', '0', '0', '1',         # rotation quaternion x y z w
                'base_link', 'os_sensor'    # parent, child
            ]
        ),

        # Convert Ouster pointcloud to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/ouster/points'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'os_sensor',
                'output_frame': 'os_sensor',
                'transform_tolerance': 0.05,
                'min_height': 0.3,      # Filter out ground reflections
                'max_height': 1.2,      # Covers box-level height
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.0087,  # 360° / ~415 beams
                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 25.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        ),

        # Relay /scan → /scan_reliable with RELIABLE QoS
        Node(
            package='topic_tools',
            executable='relay',
            name='scan_qos_relay',
            output='screen',
            parameters=[{
                'input_topic': '/scan'
            }],
            remappings=[
                ('/scan', '/scan_reliable')
            ]
        )
    ])

