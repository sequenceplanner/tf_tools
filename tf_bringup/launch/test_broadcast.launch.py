import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    scenario_path = "/home/endre/ws/src/tf_tools/tf_bringup/scenario"
    parameters = {
        "scenario_path": scenario_path
    }
    
    tf_broadcast_node = Node(
        package="tf_broadcast",
        executable="tf_broadcast",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    nodes_to_start = [
        tf_broadcast_node
    ]

    return LaunchDescription(nodes_to_start)
