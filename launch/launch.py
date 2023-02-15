from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity2ros',
            parameters=[{
                "ROS_IP": "192.168.0.81",
            }]
        ),
        Node(
            package='nemo_server',
            executable='camera_sub',
            name='camera_sub',
        ),
    ])