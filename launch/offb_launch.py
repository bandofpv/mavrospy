from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavrospy',  # Replace with your package name
            executable='offb_node_py',   # Replace with your node executable name
            name='offb_node',
            output='screen',
            parameters=[
                # Add any parameters if your node uses them, e.g. {'param_name': value}
            ]
        )
    ])

