import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():
    # Declare the fcu_url argument
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='URL for MAVROS FCU connection'
    )

    # Path to the px4.launch file in MAVROS
    px4_launch_path =  os.path.expanduser('~/ros2_ws/install/mavros/share/mavros/launch/px4.launch')

    return LaunchDescription([
        # Declare the fcu_url argument
        fcu_url_arg,

        # Include px4.launch with the fcu_url argument
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(px4_launch_path),
            launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()
        ),

        Node(
            package='mavrospy',
            executable='square_py',
            name='square',
            output='screen',
            parameters=[
            ]
        ),

        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'gz_x500'],
            output='screen',
            cwd="/home/sim/PX4-Autopilot",
            shell=True
        )
    ])

