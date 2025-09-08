from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/imad/scout_navigation_ws/src/scout_navigation/config/rviz_config.rviz']
        )
    ])

