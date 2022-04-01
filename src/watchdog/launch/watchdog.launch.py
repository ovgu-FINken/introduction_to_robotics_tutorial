from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package="watchdog",
            namespace="turtle1",
            executable="watchdog",
            name="watchdog"
        ),
        Node(
            package="watchdog",
            namespace="turtle1",
            executable="controller",
            name="controller"
        ),

    ])
