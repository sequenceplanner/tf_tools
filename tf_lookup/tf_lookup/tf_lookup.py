import rclpy
import tf2_ros
import time
import datetime

from rclpy.node import Node
from tf_tools_msgs.srv import LookupTransform
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from threading import Lock


class Variables:
    parent = ""
    child = ""
    result = TransformStamped()
    mutex = Lock()
    result = TransformStamped()
    info_once = True
    warn_once = True


class TFLookupNode(Node):
    def __init__(self):
        super().__init__("tf_lookup")

        Variables.result = TransformStamped()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.01, self.tf_lookup_timer)

        self.get_logger().info("TF Lookup Node started")

    def tf_lookup_timer(self):
        if Variables.parent != "" and Variables.child != "":
            try:
                if Variables.info_once:
                    self.get_logger().info(
                        f"Doing lookup tf: {Variables.parent} to: {Variables.child}"
                    )
                    Variables.info_once = False
                Variables.mutex.acquire()

                Variables.result = self.tf_buffer.lookup_transform(
                    Variables.parent,
                    Variables.child,
                    tf2_ros.Time(seconds=0, nanoseconds=0),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                if Variables.warn_once:
                    self.get_logger().warning(
                        f"Failed to lookup tf '{Variables.parent}' to '{Variables.child}', wait..."
                    )
                    Variables.warn_once = False
            else:
                self.get_logger().info(
                    f"Success in lookup tf for '{Variables.parent}' to '{Variables.child}'"
                )
            finally:
                Variables.mutex.release()


class TFLookupServerNode(Node):
    def __init__(self):
        super().__init__("tf_lookup_server")

        self.tf_lookup_service = self.create_service(
            LookupTransform, "tf_lookup", self.tf_lookup_callback
        )

        self.get_logger().info("TF Lookup Server Node started")

    def tf_lookup_callback(self, request, response):
        deadline = datetime.datetime.now() + datetime.timedelta(
            milliseconds=request.deadline
        )
        Variables.mutex.acquire()
        Variables.result = TransformStamped()
        Variables.parent = request.parent_id
        Variables.child = request.child_id
        Variables.info_once = True
        Variables.warn_once = True
        Variables.mutex.release()

        while rclpy.ok():
            time.sleep(0.01)
            if datetime.datetime.now() < deadline:
                if Variables.result != TransformStamped():
                    self.get_logger().info(
                        f"Looked up tf for '{Variables.parent}' to '{Variables.child}'"
                    )
    
                    response.success = True
                    response.transform = Variables.result
                    Variables.mutex.acquire()
                    Variables.parent = ""
                    Variables.child = ""
                    Variables.result = TransformStamped()
                    Variables.mutex.release()
    
                    return response
            else:
                self.get_logger().error(
                    f"Failed to lookup tf, deadline expired for '{Variables.parent}' to '{Variables.child}'"
                )

                Variables.mutex.acquire()
                Variables.parent = ""
                Variables.child = ""
                Variables.result = TransformStamped()
                Variables.mutex.release()

                response.success = False
                response.transform = TransformStamped()
                return response


def main(args=None):
    rclpy.init(args=args)
    try:
        s = TFLookupServerNode()
        l = TFLookupNode()

        executor = MultiThreadedExecutor()
        executor.add_node(s)
        executor.add_node(l)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            s.destroy_node()
            l.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
