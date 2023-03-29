import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nemo_server'),
        'config',
        'parameters.yaml'
    )

    ld = LaunchDescription()

    planner = Node(
        package='nemo_server',
        executable='planner',
        name='planner',
        parameters=[config]
    )

    camera_pub = Node(
        package='nemo_server',
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
    
    ld.add_action(planner)
    ld.add_action(camera_pub)

    ld.add_action(rosbridge)

    return ld