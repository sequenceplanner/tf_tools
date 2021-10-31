import rclpy
from rclpy.node import Node
from tf_tools_msgs.srv import LookupTransform
from geometry_msgs.msg import TransformStamped


class TFLookupClientNode(Node):
    def __init__(self):
        super().__init__("tf_lookup_client")

        self.transform = TransformStamped()

        self.client = self.create_client(LookupTransform, "tf_lookup")
        self.request = LookupTransform.Request()
        self.response = LookupTransform.Response()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("tf lookup service not available, waiting again...")

        self.send_valid_request()
        self.send_invalid_request()

    def send_valid_request(self):
        self.request.parent_id = "frame_3"
        self.request.child_id = "frame_1"
        self.request.deadline = 3000
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"request sent: {self.request}")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().error(f"service call failed with: {(e,)}")
                else:
                    self.response = response
                    self.get_logger().info(f"lookup result: {self.response}")
                finally:
                    self.get_logger().info(f"service call completed")
                break

    def send_invalid_request(self):
        self.request.parent_id = "world"
        self.request.child_id = "frame_5"
        self.request.deadline = 3000
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"request sent: {self.request}")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().error(f"service call failed with: {(e,)}")
                else:
                    self.response = response
                    self.get_logger().info(f"lookup result: {self.response}")
                finally:
                    self.get_logger().info(f"service call completed")
                break


def main(args=None):
    rclpy.init(args=args)

    node = TFLookupClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
