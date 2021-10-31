from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tf_broadcast_node = Node(
        package="tf_lookup",
        executable="tf_broadcast",
        namespace="",
        output="screen",
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    tf_lookup_node = Node(
        package="tf_lookup",
        executable="tf_lookup",
        namespace="",
        output="screen",
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    tf_lookup_client_node = Node(
        package="tf_lookup",
        executable="tf_lookup_client",
        namespace="",
        output="screen",
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True
    )

    nodes_to_start = [
        tf_broadcast_node,
        tf_lookup_node,
        tf_lookup_client_node
    ]

    return LaunchDescription(nodes_to_start)
