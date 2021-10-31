import os
import json
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    scenario = "scenario_2"
    tf_scene_dir = FindPackageShare("tf_scene").find("tf_scene")
    tf_bringup_dir = FindPackageShare("tf_bringup").find("tf_bringup")

    parameters = {
        "scenario_path": os.path.join(tf_scene_dir, "scenarios", scenario)
    }

    rviz_config_file = os.path.join(tf_bringup_dir, "config", f"{scenario}.rviz")
    
    tf_lookup_node = Node(
        package="tf_lookup",
        executable="tf_lookup",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    tf_broadcast_node = Node(
        package="tf_broadcast",
        executable="tf_broadcast",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    tf_sms_node = Node(
        package="tf_sms",
        executable="tf_sms",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    nodes_to_start = [
        tf_lookup_node,
        tf_broadcast_node,
        tf_sms_node,
        rviz_node
    ]

    return LaunchDescription(nodes_to_start)
