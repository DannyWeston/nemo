import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nemo'),
        'config',
        'simulator.yaml'
    )

    ld = LaunchDescription()

    unity2ros = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity2ros',
        parameters=[config]
    )

    planner = Node(
        package='nemo',
        executable='planner',
        name='planner',
        parameters=[config]
    )

    recorder = Node(
        package='nemo',
        executable='recorder',
        name='recorder',
        parameters=[config]
    )

    localiser = Node(
        package='nemo',
        executable='localiser',
        name='localiser',
        parameters=[config]
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge',
        parameters=[config]
    )
    
    ld.add_action(unity2ros) # Unity 2 Ros
    ld.add_action(rosbridge) # ROSLIB Websocket

    # Nemo Actions
    ld.add_action(planner)
    ld.add_action(recorder)
    ld.add_action(localiser)
    return ld