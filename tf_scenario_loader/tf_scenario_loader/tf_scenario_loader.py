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

class TFScenarioLoaderNode(Node):
    def __init__(self):
        super().__init__("tf_scenario_loader") # type: ignore

        static_transforms: List[TransformStamped] = []
        active_transforms: List[TransformStamped] = []

        self.declare_parameter("scenario_path", "default_value")
        
        print(f"params: {self._parameters}")
        scenario_path = (
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )

        included_items_paths = []
        if scenario_path == "default_value":
            self.get_logger().warn("Parameter 'scenario_path' not specified.")
        else:
            for thing in os.listdir(os.path.join(scenario_path)):
                thing_parameters = {}
                with open(os.path.join(scenario_path, thing)) as jsonfile:
                    try:
                        thing_parameters = json.load(jsonfile)
                    except ValueError as e:
                        self.get_logger().error("Couldn't load json file.")
                    else:
                        if thing_parameters["show"]:
                            included_items_paths.append(
                                os.path.join(scenario_path, thing)
                            )

        included_items = [json.load(open(x)) for x in included_items_paths]

        static_transforms = []
        static_names = []
        active_transforms = []
        active_names = []

        for item in included_items:
            if item["active"]:
                active_names.append(item["child_frame"])
                active_transforms.append(self.generate_transform_from_json(item))
            else:
                static_names.append(item["child_frame"])
                static_transforms.append(self.generate_transform_from_json(item))

        

        service = self.create_client(LoadBroadcastedFrames, "load_broadcasted_frames", )
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting for broadcaster...')
        
        req = LoadBroadcastedFrames.Request()
        req.active_transforms = active_transforms
        req.static_transforms = static_transforms
        req.reload = True

        res: LoadBroadcastedFrames.Response = service.call(req)
        if res.success:
            self.get_logger().info("We have successfully loaded the scenario")
        else:
            self.get_logger().info("Hmm, did not work!")


    

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





def main(args=None):
    rclpy.init(args=args)

    node = TFScenarioLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
