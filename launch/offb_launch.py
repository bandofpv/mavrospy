from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavrospy',
            executable='offb_node_py',
            name='offb_node',
            output='screen',
            parameters=[
            ]
        )
    ])

