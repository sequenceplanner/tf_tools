import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    scenario = "scenario_1"
    tf_scene_dir = FindPackageShare("tf_scene").find("tf_scene")
    tf_bringup_dir = FindPackageShare("tf_bringup").find("tf_bringup")

    parameters = {
        "scenario_path": os.path.join(tf_scene_dir, "scenarios", scenario)
    }

    
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

    # tf_test_load = Node(
    #     package="tf_scenario_loader",
    #     executable="tf_scenario_loader",
    #     namespace="",
    #     output="screen",
    #     parameters=[parameters],
    #     remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    #     emulate_tty=True
    # )
    

    nodes_to_start = [
        tf_lookup_node,
        tf_broadcast_node,
        tf_sms_node,
        #tf_test_load
    ]

    return LaunchDescription(nodes_to_start)
