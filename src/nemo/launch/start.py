import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nemo'),
        'config',
        'start.yaml'
    )

    ld = LaunchDescription()

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

    hardware = Node(
        package='nemo',
        executable='hardware',
        name='hardware',
        parameters=[config]
    )

    camera = Node(
        package='nemo',
        executable='camera',
        name='camera',
        parameters=[config]
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge',
        parameters=[config]
    )
    
    ld.add_action(rosbridge) # ROSLIB Websocket

    # Nemo Actions
    ld.add_action(planner)
    ld.add_action(recorder)
    ld.add_action(localiser)
    ld.add_action(hardware)
    ld.add_action(camera)

    return ld