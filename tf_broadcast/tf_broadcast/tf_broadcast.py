from turtle import st
from typing import Optional, List
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_tools_msgs.srv import ManipulateBroadcast
from tf_tools_msgs.srv import GetBroadcastedFrames
from tf_tools_msgs.srv import LoadBroadcastedFrames
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import tf2_ros

class TFBroadcastNode(Node):
    def __init__(self):
        super().__init__("tf_broadcast") # type: ignore

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.broadcast_service = self.create_service(
            ManipulateBroadcast, "manipulate_broadcast", self.broadcast_callback
        )
        
        self.get_frames_service = self.create_service(
            GetBroadcastedFrames, "get_broadcasted_frames", self.get_frames_callback
        )

        self.load_frames_service = self.create_service(
            LoadBroadcastedFrames, "load_broadcasted_frames", self.load_frames_callback
        )

        self.create_timer(0.01, self.tf_broadcaster_timer_callback)
        self.create_timer(1.0, self.static_tf_broadcaster_timer_callback)

        self.static_transforms: List[TransformStamped] = []
        self.active_transforms: List[TransformStamped] = []

        self.get_logger().info("TF Broadcaster Node started.")



    def tf_broadcaster_timer_callback(self):
        try:
            self.active_transforms = [t for t in self.active_transforms if t is not None and t != TransformStamped()]
            for tf in self.active_transforms:
                current_time = self.get_clock().now().seconds_nanoseconds()
                t = Time()
                t.sec = current_time[0]
                t.nanosec = current_time[1]
                tf.header.stamp = t
            self.tf_broadcaster.sendTransform(self.active_transforms)
        finally:
            pass

    def static_tf_broadcaster_timer_callback(self):
        try:
            self.static_tf_broadcaster.sendTransform(self.static_transforms)
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
                    self.active_transforms.append(request.transform)
                    self.get_logger().info(f"Added active frame '{request.transform.child_frame_id}'.")
                    response.success = True
                    return response
                    
                
        if request.command == "remove":
            if active_tf != None:
                if active_tf in self.active_transforms:
                    self.get_logger().info(f"Removing active frame '{request.transform.child_frame_id}'.")
                    self.active_transforms.remove(active_tf)
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

    def get_frames_callback(self, request: GetBroadcastedFrames.Request, response: GetBroadcastedFrames.Response) -> GetBroadcastedFrames.Response:
        response.active_transforms = self.active_transforms
        response.static_transforms = self.static_transforms
        return response

    def load_frames_callback(self, request: LoadBroadcastedFrames.Request, response: LoadBroadcastedFrames.Response) -> LoadBroadcastedFrames.Response:
        self.included_items_paths = []
        if list(request.json):
            act_ts = []
            st_ts = []
            for json in request.json:
                try:
                    t = json.load(json)
                except ValueError as e:
                    self.get_logger().error("Couldn't load json in broadcaster: {e}")
                    response.success = False
                    return response
                else:
                    if t["show"] and t["active"]:
                        act_ts.append(self.generate_transform_from_json(t))
                    if t["show"]:
                        st_ts.append(self.generate_transform_from_json(t))
            
            if request.reload:
                self.active_transforms = act_ts
                self.static_transforms = st_ts
                self.get_logger().info(
                    f"reloading all frames"
                )
            else:
                self.upd_transforms(act_ts, st_ts)

        elif request.reload:
            self.active_transforms = list(request.active_transforms)
            self.static_transforms = list(request.static_transforms)
            self.get_logger().info(
                f"reloading all frames"
            )
        else:
           self.upd_transforms(list(request.active_transforms), list(request.static_transforms)) 

        response.success = True
        return response
                
    def upd_transforms(self, active_ts: List[TransformStamped], static_ts: List[TransformStamped]):
        m = set([x.child_frame_id for x in active_ts])
        self.active_transforms = [x for x in self.active_transforms if not x in m]
        self.active_transforms.extend(active_ts)
        
        m = set([x.child_frame_id for x in static_ts])
        self.static_transforms = [x for x in self.static_transforms if not x in m]
        self.static_transforms.extend(static_ts)

        self.get_logger().info(
            f"updated transforms"
        )



def main(args=None):
    rclpy.init(args=args)

    node = TFBroadcastNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
