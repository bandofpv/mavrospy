from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavrospy',
            executable='square_py',
            name='square',
            output='screen',
       )
    ])

