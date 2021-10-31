import rclpy
import time
from rclpy.node import Node
from tf_tools_msgs.srv import ManipulateBroadcast
from geometry_msgs.msg import TransformStamped


class TFBroadcastClientNode(Node):
    def __init__(self):
        super().__init__("tf_broadcast_client")

        self.transform = TransformStamped()

        self.client = self.create_client(ManipulateBroadcast, "manipulate_broadcast")
        self.request = ManipulateBroadcast.Request()
        self.response = ManipulateBroadcast.Response()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "tf broadcast service not available, waiting again..."
            )

        time.sleep(3)
        self.send_valid_request()

    def send_valid_request(self):
        tf = TransformStamped()
        tf.header.frame_id = "frame_4"
        tf.child_frame_id = "frame_2"
        self.request.command = "update"
        self.request.transform = tf
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"request sent: {self.request}")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().error(
                        f"manipulate broadcast service call failed with: {(e,)}"
                    )
                else:
                    self.response = response
                    self.get_logger().info(
                        f"manipulate broadcast result: {self.response}"
                    )
                finally:
                    self.get_logger().info(
                        f"manipulate broadcast service call completed"
                    )
                break


def main(args=None):
    rclpy.init(args=args)

    node = TFBroadcastClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
