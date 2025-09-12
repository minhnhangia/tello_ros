from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver', 
            executable='tello_driver_main', 
            output='screen'
        ),
    ])
