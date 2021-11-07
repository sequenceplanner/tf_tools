import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_tools_msgs.srv import ManipulateBroadcast
# from tf_tools_msgs.srv import GetBroadcastedFrames
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import tf2_ros

class TFBroadcastNode(Node):
    def __init__(self):
        super().__init__("tf_broadcast")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.broadcast_service = self.create_service(
            ManipulateBroadcast, "manipulate_broadcast", self.broadcast_callback
        )
        
        # self.get_frames_service = self.create_service(
        #     GetBroadcastedFrames, "get_broadcasted_frames", self.get_frames_callback
        # )

        self.create_timer(0.01, self.tf_broadcaster_timer_callback)
        self.create_timer(1.0, self.static_tf_broadcaster_timer_callback)

        self.get_logger().info("TF Scene Manipulation Node started.")

        self.scenario_path = self.declare_parameter("scenario_path", "default_value")
        self.scenario_path = (
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )

        self.included_items_paths = []
        if self.scenario_path == "default_value":
            self.get_logger().warn("Parameter 'scenario_path' not specified.")
        else:
            for thing in os.listdir(os.path.join(self.scenario_path)):
                thing_parameters = {}
                with open(os.path.join(self.scenario_path, thing)) as jsonfile:
                    try:
                        thing_parameters = json.load(jsonfile)
                    except ValueError as e:
                        self.get_logger().error("Couldn't load json file.")
                    else:
                        if thing_parameters["show"]:
                            self.included_items_paths.append(
                                os.path.join(self.scenario_path, thing)
                            )

        self.included_items = [json.load(open(x)) for x in self.included_items_paths]

        self.static_transforms = []
        self.static_names = []
        self.active_transforms = []
        self.active_names = []

        for item in self.included_items:
            if item["active"]:
                self.active_names.append(item["child_frame"])
                self.active_transforms.append(self.generate_transform_from_json(item))
            else:
                self.static_names.append(item["child_frame"])
                self.static_transforms.append(self.generate_transform_from_json(item))

        self.get_logger().info(
            f"Added static frames: '{self.static_names}'."
        )
        self.get_logger().info(
            f"Added active frames: '{self.active_names}'."
        )

    def tf_broadcaster_timer_callback(self):
        try:
            for tf in self.active_transforms:
                if tf is not None:
                    if tf != TransformStamped():
                        current_time = self.get_clock().now().seconds_nanoseconds()
                        t = Time()
                        t.sec = current_time[0]
                        t.nanosec = current_time[1]
                        tf.header.stamp = t
                        self.tf_broadcaster.sendTransform(tf)
        finally:
            pass

    def static_tf_broadcaster_timer_callback(self):
        try:
            for tf in self.static_transforms:
                self.static_tf_broadcaster.sendTransform(tf)
        finally:
            pass

    def generate_transform_from_json(self, item):
        header = Header()
        header.frame_id = item["parent_frame"]
        header.stamp = Time()

        tf = TransformStamped()
        tf.header = header
        tf.child_frame_id = item["child_frame"]
        tf.transform.translation.x = item["transform"]["translation"]["x"]
        tf.transform.translation.y = item["transform"]["translation"]["y"]
        tf.transform.translation.z = item["transform"]["translation"]["z"]
        tf.transform.rotation.x = item["transform"]["rotation"]["x"]
        tf.transform.rotation.y = item["transform"]["rotation"]["y"]
        tf.transform.rotation.z = item["transform"]["rotation"]["z"]
        tf.transform.rotation.w = item["transform"]["rotation"]["w"]
        return tf

    def generate_transform(self, item):
        header = Header()
        header.frame_id = item.parent_frame
        header.stamp = Time()

        tf = TransformStamped()
        tf.header = header
        tf.child_frame_id = item.child_frame
        tf.transform.translation.x = item.transform.translation.x
        tf.transform.translation.y = item.transform.translation.y
        tf.transform.translation.z = item.transform.translation.z
        tf.transform.rotation.x = item.transform.rotation.x
        tf.transform.rotation.y = item.transform.rotation.y
        tf.transform.rotation.z = item.transform.rotation.z
        tf.transform.rotation.w = item.transform.rotation.w
        return tf

    def get_transform(self, name, active):
        if active:
            for frame in self.active_transforms:
                if frame.child_frame_id == name:
                    return frame
        else:
            for frame in self.static_transforms:
                if frame.child_frame_id == name:
                    return frame

    def broadcast_callback(self, request, response):
        active_tf = self.get_transform(request.transform.child_frame_id, True)
        static_tf = self.get_transform(request.transform.child_frame_id, False)
        if request.command == "update":
            if active_tf != None:
                if active_tf in self.active_transforms:
                    self.active_transforms.remove(active_tf)
                    self.active_transforms.append(request.transform)
                    self.get_logger().info(f"Updated active frame '{request.transform.child_frame_id}'.")
                    response.success = True
                    return response
                else:
                    self.get_logger().error(f"Found active name '{request.transform.child_frame_id}', but frame is wrong, investigate.")
                    response.success = False
                    return response
            else:
                if static_tf != None:
                    if static_tf in self.static_transforms:
                        self.get_logger().warn(f"Can't manipulate static frame '{request.transform.child_frame_id}'.")
                        response.success = False
                        return response
                    else:
                        self.get_logger().warn(f"Can't manipulate static frame '{request.transform.child_frame_id}'.")
                        self.get_logger().error(f"Found static name '{request.transform.child_frame_id}', but frame is wrong, investigate.")
                        response.success = False
                        return response
                else:
                    self.active_names.append(request.transform.child_frame_id)
                    self.active_transforms.append(request.transform)
                    self.get_logger().info(f"Added active frame '{request.transform.child_frame_id}'.")
                    response.success = True
                    return response
                    
                
        if request.command == "remove":
            if active_tf != None:
                if active_tf in self.active_transforms:
                    self.get_logger().info(f"Removing active frame '{request.transform.child_frame_id}'.")
                    self.active_transforms.remove(active_tf)
                    self.active_names.remove(request.transform.child_frame_id)
                    response.success = True
                    return response
                else:
                    self.get_logger().error(f"Found active name '{request.transform.child_frame_id}', but frame is wrong, investigate.")
                    response.success = False
                    return response
            else:
                if static_tf != None:
                    if static_tf in self.static_transforms:
                        self.get_logger().warn(f"Can't remove static frame '{request.transform.child_frame_id}'.")
                        response.success = False
                        return response
                    else:
                        self.get_logger().warn(f"Can't manipulate static frame '{request.transform.child_frame_id}'.")
                        self.get_logger().error(f"Found static name '{request.transform.child_frame_id}', but frame is wrong, investigate.")
                        response.success = False
                        return response
                else:
                    self.get_logger().info(f"Active frame '{request.transform.child_frame_id}' doesn't exist.")
                    response.success = False
                    return response

    # def get_frames_callback(self, request, response):
    #     response.active_transforms = self.active_names
    #     response.static_transforms = self.static_names
    #     return response

def main(args=None):
    rclpy.init(args=args)

    node = TFBroadcastNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
