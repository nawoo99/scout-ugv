from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scout_navigation',
            executable='lidar_processing_node',
            name='lidar_processing_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/ouster/signal_image', '/ouster/signal_image'),
                ('/ouster/points', '/your_point_cloud_topic'),
                ('/annotated_image', '/ouster/points'),
                ('/ouster/bounding_boxes_3d', '/ouster/bounding_boxes_3d')
            ]
        )
    ])

