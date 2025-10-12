from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for a single drone


def generate_launch_description():
    dr1_ns = 'drone1'

    dr1_params = [{
        'drone_ip': '192.168.10.1',
        'command_port': 8889,  # send commands to Tello from this (local) UDP port
        'drone_port': 8889,     # send commands to this (Tello) UDP port
        'data_port': 8890,      # receive Tello state on this UDP port
        'video_stream_url': 'udp://0.0.0.0:11111'  # OpenCV/FFmpeg URL
    }]

    return LaunchDescription([
        Node(
            package='tello_driver', 
            executable='tello_driver_main', 
            output='screen',
            name='driver1', 
            namespace=dr1_ns, 
            parameters=dr1_params),
    ])
